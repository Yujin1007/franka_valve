// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_valve/vision_controller.h>
#include <franka_valve/trajectory.h>
#include <cmath>
#include <fstream>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <zmq.hpp>
#include <string>
#include <iostream>

#define GRIPPER_OPEN 1
#define GRIPPER_CLOSE 0
#define GRIPPER_STOP -1


namespace franka_valve {

bool VisionController::init(hardware_interface::RobotHW* robot_hw,
                                     ros::NodeHandle& node_handle) {
  string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("VisionController: Could not read parameter arm_id");
    return false;
  }
  vector<string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "VisionController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  std::vector<double> k_gains_;
  _k_gains(7);
  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "VisionController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  } else {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> k_gains_map(k_gains_.data());
    _k_gains = k_gains_map;
  }

  std::vector<double> d_gains_;
  _d_gains(7);
  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "VisionController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  } else {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> d_gains_map(d_gains_.data());
    _d_gains = d_gains_map;
  }

  std::vector<double> xk_gains_;
  _xk_gains(6);
  if (!node_handle.getParam("xk_gains", xk_gains_) || xk_gains_.size() != 6) {
    ROS_ERROR(
        "VisionController:  Invalid or no xk_gain parameters provided, aborting "
        "controller init!");
    return false;
  } else {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xk_gains_map(xk_gains_.data());
    _xk_gains = xk_gains_map;
  }

  std::vector<double> xd_gains_;
  _xd_gains(7);
  if (!node_handle.getParam("xd_gains", xd_gains_) || xd_gains_.size() != 6) {
    ROS_ERROR(
        "VisionController:  Invalid or no xd_gain parameters provided, aborting "
        "controller init!");
    return false;
  } else {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xd_gains_map(xd_gains_.data());
    _xd_gains = xd_gains_map;
  }

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("VisionController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("VisionController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ =
        make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  }

  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "VisionController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("VisionController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ =
        make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "VisionController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "VisionController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      effort_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("VisionController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void VisionController::starting(const ros::Time& /* time */) {
  // Get initial end-effector's pose
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();

  gripper_close = false;
  gripper_open = false;
  gripper_home = false;
  gripper_stop = false;

  _elapsed_time_ = ros::Duration(0.0);
  _t = _elapsed_time_.toSec();
  _printtime = 0.0;
  _timeinterval = 0.1;

  _operation_time = ros::Time::now();

  // Initial setting / calculate hand trajectory in advance
  _init_theta = M_PI * 0.0;
  _goal_theta = M_PI * 1.0;
  _bool_plan.fill(false);
  _cnt_plan = 0;
  _cnt = 0;
  _zmq_receive_time = 0;
  _accum_time = 0;
  _bool_plan[_cnt_plan] = true;

  _q.setZero(7);
  _qdot.setZero(7);
  _q_des.setZero(7);
  _qdot_des.setZero(7);
  _x_hand.setZero(6);
  _xdot_hand.setZero(6);
  _x_des_hand.setZero(6);
  _xdot_des_hand.setZero(6);
  _R_des_hand.setIdentity(3, 3);
  _R_hand.setIdentity(3, 3);
  _Rdot_des_hand.setIdentity(3, 3);
  _Rdot_hand.setIdentity(3, 3);
  _J_hands(6, 7);
  _mass(7, 7);
  _coriolis(7, 1);
  _target_plan.clear();
  _Tvr.setIdentity(4, 4);
  _motion_done = false;

  _cnt_plot = 0;
  _q_plot.setZero(1000, 7);
  _qdes_plot.setZero(1000, 7);
  _x_plot.setZero(100000, 6);
  _x_des_plot.setZero(100000, 6);
  _xdot_plot.setZero(100000, 6);
  _xdot_des_plot.setZero(100000, 6);

  _time_plot.setZero(100000, 1);

  _robot.pos << 0, 0, 0;
  _robot.zrot = 0;      // M_PI;
  _robot.ee_align = 0;  // DEG2RAD * (-90);

  Matrix3d rot_obj(3, 3);
  // rot_obj << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  rot_obj << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  _obj.name = "HANDLE_VALVE";
  // _obj.o_margin << 0, 0.149, 0;
  _obj.o_margin << 0, 0, 0.149;
  _obj.o_margin = rot_obj * _obj.o_margin;
  _obj.r_margin << -0.12, 0, 0;  // East side of the origin
  _obj.r_margin = rot_obj * _obj.r_margin;
  _obj.grab_dir << _obj.o_margin.cross(_obj.r_margin);  //_obj.r_margin;  //
  // _obj.pos << 0.60, 0.0, 0.2;
  _obj.pos << 0.580994101778967, -0.045675755104744684, 0.115;

  // obj.pos << 0.44296161, -0.2819804 ,  0.00434591;
  _gripper_close = 0.01;  // 0.01;
  _gripper_open = 0.04;
  _obj.grab_vector = _obj.grab_dir.normalized();
  _obj.normal_vector = -_obj.o_margin.normalized();
  _obj.radius = _obj.r_margin.norm();

  Vector4d xaxis;
  Vector4d yaxis;
  Vector4d zaxis;
  Vector4d porg;
  Matrix4d Tvb;  // valve handle -> valve base
  Matrix4d Tbu;  // valve base -> universal
  Matrix4d Tur;  // universal -> robot
  Matrix4d Tvr;  // valve handle -> valve vase -> universal -> robot!!

  // calc target x,y,z
  xaxis << _obj.r_margin.normalized(), 0;
  yaxis << _obj.o_margin.normalized().cross(_obj.r_margin.normalized()), 0;
  zaxis << _obj.o_margin.normalized(), 0;
  porg << _obj.o_margin, 1;

  Tvb << xaxis, yaxis, zaxis, porg;

  Tbu << 1, 0, 0, _obj.pos(0), 0, 1, 0, _obj.pos(1), 0, 0, 1, _obj.pos(2), 0, 0, 0, 1;

  Tur << cos(-_robot.zrot), sin(-_robot.zrot), 0, _robot.pos(0), -sin(-_robot.zrot),
      cos(-_robot.zrot), 0, _robot.pos(1), 0, 0, 1, -_robot.pos(2), 0, 0, 0, 1;

  Tvr << Tur * Tbu * Tvb; 
  _Tvr = Tvr;
  
  
  UpdateStates();
  InitMotionPlan();
  
}

void VisionController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  _elapsed_time_ += period;
  _t = _elapsed_time_.toSec();
  // _t += 0.001;
  _dt = period.toSec();
  // Get Franka's model information
  ros::Duration execution_time = ros::Time::now() - _operation_time;
  // ROS_INFO("%f seconds.", execution_time.toSec());
  _operation_time = ros::Time::now();
  _accum_time += execution_time.toSec();
  _cnt += 1;

  UpdateStates();
  if (!_motion_done) {
    Target target = _target_plan[_cnt_plan];
    if (_bool_plan[_cnt_plan] && !target.state.empty()) {
      MotionPlan(target);
      switch (target.gripper)
      {
      case 1:
        gripper_open = true;
        gripper_close = false;
        gripper_stop = false;
        break;
      case 0:
        gripper_close = true;
        gripper_open = false;
        gripper_stop = false;
        break;
      case -1:
        gripper_stop = true;
        gripper_close = false;
        gripper_open = false;
        break;      
      default:
        break;
      }
    }
    
    int joint_done = 0;
    int hand_done = 0;
    int valve_done = 0;
    if (target.state == "home") {
      JointTrajectory.update_time(_t);
      _q_des = JointTrajectory.position_cubicSpline();
      _qdot_des = JointTrajectory.velocity_cubicSpline();

      _torque_des = JointControl();
      // GripperControl(); // planning + torque generation
      if (JointTrajectory.check_trajectory_complete() == 1) {
        _cnt_plan += 1;
        _bool_plan[_cnt_plan] = true;
        joint_done = 1;
        printf("target %d is over, move to next target %d\n", _cnt_plan - 1, _cnt_plan);
        printf("It was joint impedance control.\n");
        cout << "gripper :" << target.gripper << endl;
        cout << "x_hand :" << _x_hand.transpose() << endl;
        cout << "d_hand :" << _x_des_hand.transpose() << endl;
        cout << "error  :" << (_x_des_hand - _x_hand).transpose() << endl;
        cout << "q : " << _q.transpose() << endl ;
        cout << "avg time : " << _accum_time / _cnt << endl<<endl;
        _accum_time = 0.0;
        _cnt = 0;
      }
    } else if (target.state == "taskspace") {
      HandTrajectory.update_time(_t);
      _x_des_hand.head(3) = HandTrajectory.position_cubicSpline();
      _xdot_des_hand.head(3) = HandTrajectory.velocity_cubicSpline();
      _R_des_hand = HandTrajectory.rotationCubic();
      _x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(_R_des_hand);
      _xdot_des_hand.segment<3>(3) = HandTrajectory.rotationCubicDot();

      _torque_des = OperationalSpaceControl();
      // GripperControl();

      if (HandTrajectory.check_trajectory_complete() == 1) {
        _cnt_plan += 1;
        _bool_plan[_cnt_plan] = true;
        hand_done = 1;
        printf("target %d is over, move to next target %d\n", _cnt_plan - 1, _cnt_plan);
        printf("It was operational space control.\n");
        cout << "gripper :" << target.gripper << endl;
        cout << "x_hand :" << _x_hand.transpose() << endl;
        cout << "d_hand :" << _x_des_hand.transpose() << endl;
        cout << "error  :" << (_x_des_hand - _x_hand).transpose() << endl;
        cout << "q : " << _q.transpose() << endl;
        cout << "avg time : " << _accum_time / _cnt << endl<<endl;
        _accum_time = 0.0;
        _cnt = 0;
      }

    } else if (target.state == "onvalve") {
      _theta_des = CircularTrajectory.update_time(_t);
      _x_des_hand.head(3) = CircularTrajectory.position_circular();
      _xdot_des_hand.head(3) = CircularTrajectory.velocity_circular();
      _x_des_hand.tail(3) = CircularTrajectory.rotation_circular();
      _xdot_des_hand.tail(3) = CircularTrajectory.rotationdot_circular();
      _R_des_hand =
          CustomMath::GetBodyRotationMatrix(_x_des_hand(3), _x_des_hand(4), _x_des_hand(5));

      _torque_des = OperationalSpaceControl();

      // if ((_t - _printtime > _timeinterval) && !_motion_done) {
      //   _printtime = _t;
      //   _qdes_plot.row(_cnt_plot) = _q_des;
      //   _q_plot.row(_cnt_plot) = _q;
      //   _x_des_plot.row(_cnt_plot) = _x_des_hand;
      //   _x_plot.row(_cnt_plot) = _x_hand;
      //   _cnt_plot += 1;

        
      // }
      // GripperControl();
      if (CircularTrajectory.check_trajectory_complete() == 1) {
        _cnt_plan += 1;
        _bool_plan[_cnt_plan] = true;
        valve_done = 1;
        printf("Valve trajectory over");
        printf("target %d is over, move to next target %d\n", _cnt_plan - 1, _cnt_plan);
        cout << "gripper :" << target.gripper << endl;
        cout << "avg time : " << _accum_time / _cnt << endl;
        _accum_time = 0.0;
        _cnt = 0;

        // gripper_stop = true;
        // gripper_close = false;
        // gripper_open = false;
      }
    }

    // Gripper
    if (gripper_close) { //grasp
      epsilon.inner = 0.005;  // 0.005;
      epsilon.outer = 0.005;  // 0.005;
      close_goal.speed = 0.1;
      close_goal.width = 0.025;  // 0.003;
      close_goal.force = 300.0;
      close_goal.epsilon = epsilon;
      gripper_ac_close.sendGoal(close_goal);
      gripper_close = false;

      printf("gripper close\n");
    }
    if (gripper_open) { // move
      open_goal.speed = 0.1;
      open_goal.width = 0.08;
      gripper_ac_open.sendGoal(open_goal);
      gripper_open = false;

      printf("gripper open\n");
    }

    if (gripper_stop) {
      gripper_ac_stop.sendGoal(stop_goal);
      gripper_stop = false;

      printf("gripper stop\n");
    }

    for (size_t i = 0; i < 7; ++i) {
      effort_handles_[i].setCommand(_torque_des[i]);
      // effort_handles_[i].setCommand(_coriolis(i));
    }
  } else {

    // if (_cnt_plot != 0) {
    //   printf("save csv\n");
    //   std::string filename;

    //   filename = "x.csv";
    //   writeToCSVfile(filename, _x_plot, _cnt_plot);

    //   filename = "xdot.csv";
    //   writeToCSVfile(filename, _xdot_plot,_cnt_plot);

    //   filename = "x_des.csv";
    //   writeToCSVfile(filename, _x_des_plot, _cnt_plot);

    //   filename = "xdot_des.csv";
    //   writeToCSVfile(filename, _xdot_des_plot,_cnt_plot);

    //   filename = "time.csv";
    //   writeToCSVfile(filename, _time_plot,_cnt_plot);

    //   _cnt_plot = 0;
    // }
    array<double, 7> coriolis_array = model_handle_->getCoriolis();
    for (size_t i = 0; i < 7; ++i) {
      effort_handles_[i].setCommand(coriolis_array[i]);
    }
  }
}



void VisionController::gripperCloseCallback(const std_msgs::Bool& msg) {
  gripper_close = msg.data;
  gripper_open = false;
}

void VisionController::gripperOpenCallback(const std_msgs::Bool& msg) {
  gripper_open = msg.data;
  gripper_close = false;
}

void VisionController::gripperStopCallback(const std_msgs::Bool& msg) {
  gripper_stop = msg.data;
  gripper_open = false;
  gripper_close = false;
}

array<double, 7> VisionController::saturateTorqueRate(
    const array<double, 7>& tau_d_calculated,
    const array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + max(min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}


void VisionController::writeToCSVfile(string filename, MatrixXd input_matrix, int cnt) {
  // Open the file for writing
  
  string path = "/home/kist/catkin_ws/src/franka_ros/franka_valve/data/franka_vision/";
  path = path + filename;
  std::ofstream outputFile(path);

  if (!outputFile.is_open()) {
    std::cerr << "Error opening file " << filename << std::endl;
  }
  // outputFile << "qd1,q1,qd2,q2,qd3,q3,qd4,q4,qd5,q5,qd6,q6,qd7,q7,xd,x,yd,y,zd,z,rd,r,pd,p,yd,y"
  //            << endl;
  // Write the matrix data to the CSV file
  int len = input_matrix.cols();
  for (Eigen::Index i = 0; i < cnt; ++i) {
    for (Eigen::Index j = 0; j < len; ++j) {
      outputFile << input_matrix(i, j);
      outputFile << ",";
    }
    outputFile << "\n";  // Move to the next row
  }
  outputFile.close();
}

void VisionController::InitMotionPlan() {
  // zmq
  ros::Time zmq_wait_time = ros::Time::now();
  Matrix4d cam_to_target;
  
  zmq::context_t context (1);
  zmq::socket_t socket (context, ZMQ_REQ);

  std::cout << "Connecting to hello world server…" << std::endl;
  socket.connect ("tcp://161.122.114.1:5555");

  zmq::message_t request (6);
  memcpy ((void *) request.data (), "Hello", 5);
  socket.send (request);

  //  Get the reply.
  zmq::message_t reply;
  float test[16];
  socket.recv (&reply);
  // test = reply.data();

  memcpy(test, reply.data(), sizeof(float) * 16);
  for (int i = 0 ; i < 4; i++){
    for (int j = 0 ; j < 4; j++){
      cam_to_target(i,j) = test[4*i+j];
    }
  }
  
  // cam_to_target << 0.962775, -0.0965821, -0.252461, -0.0167192, 0.171395, 0.940349,   0.293884,  -0.235368, 0.209018,  -0.326215,   0.921898,   0.583981, 0, 0, 0, 1;

  std::cout << "cam_to_target" << std::endl;
  std::cout << cam_to_target << std::endl;
  // cam_to_target(1,3) *= -1.;

  // start position -> move to valve -> generate circular trajectory -> start position

  double motion_time_const = 10.0;
  double motion_time;

  Matrix4d base_to_ee;
  base_to_ee.setZero(4,4);
  for (int i = 0 ; i < 3; i++){
    for (int j = 0 ; j < 3; j++){
      base_to_ee(i,j) = _R_hand(i,j);
    }
  }
  base_to_ee(0,3) = _x_hand(0);
  base_to_ee(1,3) = _x_hand(1);
  base_to_ee(2,3) = _x_hand(2);
  base_to_ee(3,3) = 1.0;
  Vector3d ee_rpy;
  ee_rpy = CustomMath::GetBodyRotationAngle(_R_hand);

  std::cout << "base_to_ee" << std::endl;
  std::cout << base_to_ee << std::endl;
  std::cout << "ee_rpy" << std::endl;
  std::cout << ee_rpy << std::endl;

  Matrix4d ee_to_cam;
  ee_to_cam.setZero(4,4);
  ee_to_cam(3,3) = 1.0;

  float theta = M_PI / 2.0;
  // float theta = 0.;
  ee_to_cam(0,0) = cos(theta);
  ee_to_cam(0,1) = -sin(theta);
  ee_to_cam(1,0) = sin(theta);
  ee_to_cam(1,1) = cos(theta);
  ee_to_cam(2,2) = 1.0;
  ee_to_cam(0,3) = -0.05;
  ee_to_cam(1,3) = -0.1;
  ee_to_cam(2,3) = 0.06;

  // ee_to_cam(0,3) = -0.09;
  // ee_to_cam(1,3) = -0.05;
  // ee_to_cam(2,3) = 0.04;

  std::cout << "ee_to_cam" << std::endl;
  std::cout << ee_to_cam << std::endl;
  Matrix3d ee_to_cam_rot;
  for (int i = 0 ; i < 3; i++){
    for (int j = 0 ; j < 3; j++){
      ee_to_cam_rot(i,j) = ee_to_cam(i,j);
    }
  }
  // Vector3d cam_rpy;
  // cam_rpy = CustomMath::GetBodyRotationAngle(ee_to_cam_rot);
  // std::cout << "cam_rpy" << std::endl;
  // std::cout << cam_rpy << std::endl;
  // return;

  Matrix4d base_to_target;
  Matrix4d base_to_sub_target;
  base_to_target = base_to_ee * ee_to_cam * cam_to_target;
  // base_to_target = ee_to_cam * cam_to_target;
  // base_to_target(1,3) *= -1;
  // base_to_target = base_to_ee * base_to_target;
  // base_to_target = ee_to_cam.inverse() * base_to_target;
  // base_to_target = cam_to_target.inverse() * ee_to_cam.inverse() * base_to_ee.inverse();
  base_to_sub_target = base_to_target;

  Vector3d base_to_goal_t;
  Vector3d base_to_subgoal_t;
  Matrix3d base_to_target_rot;
  base_to_target_rot.setZero(3,3);
  for (int i = 0 ; i < 3; i++){
    for (int j = 0 ; j < 3; j++){
      base_to_target_rot(i,j) = base_to_target(i,j);
    }
  }
  for (int i = 0 ; i < 3; i++){
    base_to_subgoal_t(i) = base_to_target(i,3);
    base_to_goal_t(i) = base_to_target(i,3);
  }
  base_to_goal_t = base_to_target_rot.transpose() * base_to_goal_t;
  // z = 0.06, x = -0.1, y = -0.06
  // base_to_goal_t[0] += 0.05;
  // base_to_goal_t[1] -= 0.1;
  base_to_goal_t[2] += 0.01;
  base_to_goal_t = base_to_target_rot * base_to_goal_t;
  base_to_target_rot = base_to_target_rot * ee_to_cam_rot.transpose();



  base_to_subgoal_t = base_to_target_rot.transpose() * base_to_subgoal_t;
  base_to_subgoal_t(2) -= 0.1;
  base_to_subgoal_t = base_to_target_rot * base_to_subgoal_t;
  for (int i = 0 ; i < 3; i++){
    base_to_sub_target(i,3) = base_to_subgoal_t(i);
    base_to_target(i,3) = base_to_goal_t(i);
  }
  
  // Vector3d sub_target_rpy;
  // sub_target_rpy = CustomMath::GetBodyRotationAngle(base_to_target_rot);
  Vector3d target_rpy;
  target_rpy = CustomMath::GetBodyRotationAngle(base_to_target_rot);
  // // target_rpy[0] += M_PI * 2.0;
  // // target_rpy[2] -= M_PI * 2.0;
  // base_to_target_rot = CustomMath::GetBodyRotationMatrix(target_rpy[0], target_rpy[1], target_rpy[2]);
  for (int i = 0 ; i < 3; i++){
    for (int j = 0 ; j < 3; j++){
      base_to_target(i,j) = base_to_target_rot(i,j);
      base_to_sub_target(i,j) = base_to_target_rot(i,j);
    }
  }

  std::cout << "base_to_target" << std::endl;
  std::cout << base_to_target << std::endl;
  std::cout << "base_to_sub_target" << std::endl;
  std::cout << base_to_sub_target << std::endl;
  std::cout << "target_rpy" << std::endl;
  std::cout << target_rpy << std::endl;
  std::cout << "==================================" << std::endl;
  

  ros::Duration zmq_receive_time = ros::Time::now() - zmq_wait_time;
  _zmq_receive_time = zmq_receive_time.toSec();
  

  Target task;
  task.state = "taskspace";

  Target home;
  home.state = "home";
  // home.q_goal = {0.6272754044329962, -1.1570632703442354, -0.7078373341253018, -2.0651029322099097, -0.42034600183099496, 1.5838923196335506, 0.15774797016768116};
  home.q_goal = {0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397};
  home.time = 3.0;
  // _target_plan.push_back(home);
  // _target_plan.back().gripper = GRIPPER_OPEN;  //_gripper_open;
  // _target_plan.back().time = 3.0;


  // Objects obj_above = _obj;
  // obj_above.o_margin = obj_above.o_margin + obj_above.o_margin.normalized() * 0.05;
  // _target_plan.push_back(TargetTransformMatrix(obj_above, _robot, init_theta));

  task.x = base_to_sub_target(0,3);
  task.y = base_to_sub_target(1,3);
  task.z = base_to_sub_target(2,3);
  task.roll = target_rpy(0);
  task.pitch = target_rpy(1);
  task.yaw = target_rpy(2);
  
  task.gripper = GRIPPER_STOP;  //_gripper_open;
  task.time = 10.0;
  _target_plan.push_back(task);

  task.x = base_to_target(0,3);
  task.y = base_to_target(1,3);
  task.z = base_to_target(2,3);
  task.roll = target_rpy(0);
  task.pitch = target_rpy(1);
  task.yaw = target_rpy(2);
  
  task.gripper = GRIPPER_STOP;  //_gripper_open;
  task.time = 3.0;
  _target_plan.push_back(task);

 
  // _target_plan.push_back(home);
  // _target_plan.back().gripper = GRIPPER_STOP;  //_gripper_open;
  // home.q_goal = {0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397};

  // _target_plan.back().time = 3.0;
}

void VisionController::MotionPlan(Target target) {
  _bool_plan[_cnt_plan] = false;
  float start_time = _t + _zmq_receive_time;
  
  float end_time = start_time + target.time;
  _zmq_receive_time = 0.0;

  if (target.state == "home") {
    VectorXd q_goal(7), qdot_goal(7);

    q_goal << target.q_goal[0], target.q_goal[1], target.q_goal[2], target.q_goal[3],
        target.q_goal[4], target.q_goal[5], target.q_goal[6];
    qdot_goal.setZero();

    JointTrajectory.reset_initial(start_time, _q, _qdot);
    JointTrajectory.update_goal(q_goal, qdot_goal, end_time);

  } else if (target.state == "taskspace") {
    VectorXd x_goal_hand(6), xdot_goal_hand(6);

    x_goal_hand(0) = target.x;
    x_goal_hand(1) = target.y;
    x_goal_hand(2) = target.z;
    x_goal_hand(3) = target.roll;
    x_goal_hand(4) = target.pitch;
    x_goal_hand(5) = target.yaw;

    xdot_goal_hand.setZero();

    HandTrajectory.reset_initial(start_time, _x_hand, _xdot_hand);
    HandTrajectory.update_goal(x_goal_hand, xdot_goal_hand, end_time);
    cout<<"_x_hand start  ;"<<_x_hand.transpose()<<endl;
    

  } else if (target.state == "onvalve") {
    CircularTrajectory.reset_initial(start_time, _obj.grab_vector, _obj.normal_vector, _obj.radius,
                                     _Tvr, _dt);
    CircularTrajectory.update_goal(end_time, _init_theta, _goal_theta);
  }
}

void VisionController::UpdateStates() {
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();
  array<double, 7> coriolis_array = model_handle_->getCoriolis();
  array<double, 7> gravity_array = model_handle_->getGravity();
  array<double, 49> mass_array = model_handle_->getMass();

  const array<double, 42>& jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  _J_hands = Map<const Matrix<double, 6, 7>>(jacobian_array.data());
  _mass = Map<const Matrix<double, 7, 7>>(mass_array.data());
  _coriolis = Map<const Matrix<double, 7, 1>>(coriolis_array.data());
  vector<double> armateur = {0.5, 0.5, 0.3, 0.3, 0.3, 0.3, 0.2, 0.12};
  for (size_t i = 0; i < 7; ++i) {
    _mass(i, i) += armateur[i];
  }

  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_map(robot_state_.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> qdot_map(robot_state_.dq.data());
  _q = q_map;
  _qdot = qdot_map;

  Affine3d ee_pose(Matrix4d::Map(robot_state_.O_T_EE.data()));
  Vector3d ee_position(ee_pose.translation());
  Matrix3d ee_rotation(ee_pose.linear());
  _x_hand.head(3) = ee_position;
  _x_hand.tail(3) = CustomMath::GetBodyRotationAngle(ee_rotation);
  // cout<<"_x_hand : "<<_x_hand.transpose()<<endl;
  _R_hand = ee_rotation;

  _xdot_hand = _J_hands * _qdot;
  _Rdot_hand = CustomMath::GetBodyRotationMatrix(_xdot_hand(3), _xdot_hand(4), _xdot_hand(5));

  if (_cnt_plan < _target_plan.size()) {
    _motion_done = false;
  } else {
    _motion_done = true;
  }

  // if ((_t - _printtime > _timeinterval) && !_motion_done) {
  //   _printtime = _t;

  //   _qdes_plot.row(_cnt_plot) = _q_des;
  //   _q_plot.row(_cnt_plot) = _q;
  //   _xdes_plot.row(_cnt_plot) = _x_des_hand;
  //   _x_plot.row(_cnt_plot) = _x_hand;

  //   _cnt_plot += 1;
  // }
}
array<double, 7> VisionController::OperationalSpaceControl() {
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();
  VectorXd force(6), torque(7), x_err_hand(6), xdot_err_hand(6);

  Matrix<double, 7, 6> J_bar_hands(CustomMath::pseudoInverseQR(_J_hands));
  Matrix<double, 6, 6> lambda(CustomMath::pseudoInverseQR(_J_hands.transpose()) * _mass *
                              J_bar_hands);

  x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);
  x_err_hand.segment(3, 3) = -CustomMath::getPhi(_R_hand, _R_des_hand);

  xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
  xdot_err_hand.segment(3, 3) = -CustomMath::getPhi(_Rdot_hand, _Rdot_des_hand);
  // cout<<"x_err : "<<x_err_hand.transpose()<<endl;
  // cout<<"x_des : "<<_x_des_hand.transpose()<<endl;
  // cout<<"x_now : "<<_x_hand.transpose()<<endl;
  
  // cout<<"xd_err : "<<xdot_err_hand.transpose()<<endl;
  // cout<<"xd_des : "<<_xdot_des_hand.transpose()<<endl;
  // cout<<"xd_now : "<<_xdot_hand.transpose()<<endl;
  
  // force = k_gains_ * x_err_hand + d_gains_ * xdot_err_hand;
  force = _xk_gains.cwiseProduct(x_err_hand) + _xd_gains.cwiseProduct(xdot_err_hand);

  torque = _J_hands.transpose() * lambda * force + coriolis_factor_ * _coriolis;

  array<double, 7> tau_d;
  for (int i = 0; i < 7; ++i) {
    tau_d[i] = torque(i);
  }
  array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d, robot_state_.tau_J_d);

  _x_plot.row(_cnt_plot) = _x_hand.transpose();

  _x_des_plot.row(_cnt_plot) = _x_des_hand.transpose();

  _xdot_plot.row(_cnt_plot) = _xdot_hand.transpose();

  _xdot_des_plot.row(_cnt_plot) = _xdot_des_hand.transpose();
  VectorXd time(1);
  time(0) = _t;
  _time_plot.row(_cnt_plot) = time;
  _cnt_plot += 1;

  return tau_d_saturated;
}

// void VisionController::CLIK()
// {
// 	_x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);

// 	_x_err_hand.segment(3, 3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

// 	_J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);
// 	_qdot_des = _J_bar_hands * (_xdot_des_hand + _x_kp * (_x_err_hand));// +
// _x_err_hand.norm()*_x_force); 	_q_des = _q_des + _dt * _qdot_des;

// 	_torque = Model._A * (_kpj * (_q_des - _q) + _kdj * (_qdot_des - _qdot)) + Model._bg;
// }

array<double, 7> VisionController::JointControl() {
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();

  VectorXd force(6), torque(7), torque_0(7);

  // cout<<"k gain : "<<_k_gains.transpose()<<endl;
  // cout<<"torque : "<<torque.transpose()<<endl;
  // if (_t - _printtime > _timeinterval) {
  //   _printtime = _t;
  // cout<<"q des : "<<_q_des.transpose()<<endl;
  // cout<<"q     : "<<_q.transpose()<<endl;
  // cout<<"torque  : "<<torque.transpose()<<endl;
  // cout<<"torque0 : "<<torque_0.transpose()<<endl;
  // cout<<"qd-q : "<<(_q_des - _q).transpose()<<endl;
  // cout<<"entry : "<<_mass.diagonal().transpose()<<endl;
  // cout<<"mass :\n"<<_mass<<endl;
  // std::array<double, 7> g = model_handle_->getGravity();
  // Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(g.data());
  // Eigen::Map<const Eigen::Matrix<double, 7, 1>> torque_robot(robot_state_.tau_J.data());

  // cout<<"g: "<<gravity.transpose()<<endl;
  // cout<<"total (b+g+tau) :"<<(gravity+torque).transpose()<<endl;
  // cout<<"torque_robot    :"<<torque_robot.transpose()<<endl;
  // }

  torque = _mass * (_k_gains.cwiseProduct(_q_des - _q) + _d_gains.cwiseProduct(_qdot_des - _qdot)) +
           coriolis_factor_ * _coriolis;
  // for (size_t i=0; i<7;++i){
  //   torque(i)
  // }
  // torque.setZero(7);

  torque_0 = _mass.diagonal().cwiseProduct(_k_gains.cwiseProduct(_q_des - _q) +
                                           _d_gains.cwiseProduct(_qdot_des - _qdot)) +
             coriolis_factor_ * _coriolis;

  array<double, 7> tau_d;
  for (int i = 0; i < 7; ++i) {
    tau_d[i] = torque(i);
  }
  array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d, robot_state_.tau_J_d);

  return tau_d_saturated;
}

VisionController::Target VisionController::TargetTransformMatrix(Objects obj,
                                                                                   Robot robot,
                                                                                   double angle) {
  // frame u : universal
  // frame v : valve rotation axis
  // frame b : valve base orgin (same rotation with frame u)
  // frame g : gripper
  // frame e : end-effector
  Vector4d xaxis;
  Vector4d yaxis;
  Vector4d zaxis;
  Vector4d porg;
  Matrix4d Tvb;  // valve handle -> valve base
  Matrix4d Tbu;  // valve base -> universal
  Matrix4d Tur;  // universal -> robot
  Matrix4d Tvr;  // valve handle -> valve vase -> universal -> robot!!

  Target target;
  Vector4d tmp;
  Matrix3d Tug;  // universal -> gripper
  Matrix3d Tge;  // gripper -> end-effector
  Matrix3d Tue;  // universal -> gripper -> end-effector
  // calc target x,y,z
  xaxis << obj.r_margin.normalized(), 0;
  yaxis << obj.o_margin.normalized().cross(obj.r_margin.normalized()), 0;
  zaxis << obj.o_margin.normalized(), 0;
  porg << obj.o_margin, 1;

  Tvb << xaxis, yaxis, zaxis, porg;

  Tbu << 1, 0, 0, obj.pos(0), 0, 1, 0, obj.pos(1), 0, 0, 1, obj.pos(2), 0, 0, 0, 1;

  Tur << cos(-robot.zrot), sin(-robot.zrot), 0, robot.pos(0), -sin(-robot.zrot), cos(-robot.zrot),
      0, robot.pos(1), 0, 0, 1, -robot.pos(2), 0, 0, 0, 1;

  Tvr << Tur * Tbu * Tvb;

  tmp << obj.r_margin.norm() * cos(angle), obj.r_margin.norm() * sin(angle), 0, 1;
  tmp << Tvr * tmp;
  target.x = tmp(0);
  target.y = tmp(1);
  target.z = tmp(2);

  // calc target r,p,y
  Tge << cos(robot.ee_align), -sin(robot.ee_align), 0, sin(robot.ee_align), cos(robot.ee_align), 0,
      0, 0, 1;
  Tug = VisionController::R3D(obj, -obj.o_margin.normalized(), angle);
  Tue << Tug * Tge;

  target.yaw = atan2(Tue(1, 0), Tue(0, 0)) + robot.zrot;
  target.pitch = atan2(-Tue(2, 0), sqrt(pow(Tue(2, 1), 2) + pow(Tue(2, 2), 2)));
  target.roll = atan2(Tue(2, 1), Tue(2, 2));

  target.yaw = fmod(target.yaw + M_PI, 2 * M_PI);
  if (target.yaw < 0) {
    target.yaw += 2 * M_PI;
  }
  target.yaw = target.yaw - M_PI;

  target.pitch = fmod(target.pitch + M_PI, 2 * M_PI);
  if (target.pitch < 0) {
    target.pitch += 2 * M_PI;
  }
  target.pitch = target.pitch - M_PI;

  target.roll = fmod(target.roll + M_PI, 2 * M_PI);

  if (target.roll < 0) {
    target.roll += 2 * M_PI;
  }
  target.roll = target.roll - M_PI;

  return target;
}

Matrix3d VisionController::R3D(Objects obj, Vector3d unitVec, double angle) {
  Matrix3d Tug;
  angle = -angle;  // frame은 반대 방향으로 회전 해야지, gripper방향이 유지된다.
  double cosAngle = cos(angle);
  double sinAngle = sin(angle);
  double x = unitVec(0);
  double y = unitVec(1);
  double z = unitVec(2);
  Matrix3d rotMatrix;
  rotMatrix << cosAngle + (1 - cosAngle) * x * x, (1 - cosAngle) * x * y - sinAngle * z,
      (1 - cosAngle) * x * z + sinAngle * y, (1 - cosAngle) * y * x + sinAngle * z,
      cosAngle + (1 - cosAngle) * y * y, (1 - cosAngle) * y * z - sinAngle * x,
      (1 - cosAngle) * z * x - sinAngle * y, (1 - cosAngle) * z * y + sinAngle * x,
      cosAngle + (1 - cosAngle) * z * z;

  Tug << rotMatrix * obj.grab_dir.normalized(),
      rotMatrix * -obj.o_margin.normalized().cross(obj.grab_dir.normalized()),
      rotMatrix * -obj.o_margin.normalized();
  return Tug;
}

}  // namespace franka_valve

PLUGINLIB_EXPORT_CLASS(franka_valve::VisionController,
                       controller_interface::ControllerBase)