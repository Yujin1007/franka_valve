// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_valve/rl_controller.h>
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

#define GRIPPER_OPEN 1
#define GRIPPER_CLOSE 0
#define GRIPPER_STOP -1

namespace franka_valve {

bool RLController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("RLController: Could not read parameter arm_id");
    return false;
  }
  vector<string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "RLController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  std::vector<double> k_gains_;
  _k_gains(7);
  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "RLController:  Invalid or no k_gain parameters provided, aborting "
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
        "RLController:  Invalid or no d_gain parameters provided, aborting "
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
        "RLController:  Invalid or no xk_gain parameters provided, aborting "
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
        "RLController:  Invalid or no xd_gain parameters provided, aborting "
        "controller init!");
    return false;
  } else {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xd_gains_map(xd_gains_.data());
    _xd_gains = xd_gains_map;
  }

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("RLController: coriolis_factor not found. Defaulting to " << coriolis_factor_);
  }
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("RLController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ =
        make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  }

  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("RLController: Exception getting state handle from interface: " << ex.what());
    return false;
  }
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("RLController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ =
        make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("RLController: Exception getting model handle from interface: " << ex.what());
    return false;
  }
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("RLController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      effort_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("RLController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void RLController::starting(const ros::Time& /* time */) {
  // Get initial end-effector's pose
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();

  // Classifier_cclk.Initialize("classifier_cclk");
  // Classifier_clk.Initialize("classifier_clk");
  gripper_close = false;
  gripper_open = false;
  gripper_stop = false;

  _elapsed_time_ = ros::Duration(0.0);
  _t = _elapsed_time_.toSec();
  _printtime = 0.0;
  _timeinterval = 0.1;
  _steptime = 0.0;
  _stepinterval = 0.1;
  _cnt_step = 0;
  zmq_start = true;
  zmq_cnt = 0;
  _cnt_recv = -1000;
  _q_mppi.setZero(7);
  _qdot_mppi.setZero(7);

  // Initial setting / calculate hand trajectory in advance
  _init_theta = M_PI * 0;
  _goal_theta = M_PI * 1;
  int direction;
  direction = (_init_theta > _goal_theta) ? -1 : 1;
  _bool_plan.fill(false);
  _cnt_plan = 0;
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
  _q_normalize.setZero(7);
  _q_normalize_max.setZero(7);
  _q_normalize_min.setZero(7);
  _q_normalize_max << 2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159;
  _q_normalize_min << -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159;

  _stack = 5;
  _action.setZero(6);
  _drpy_pre.setZero(3);
  _drpy_out.setZero(3);
  _drpy.setZero(3);
  _obs_obj.setZero(14);
  _obs_q.setZero(_stack * 7);
  _obs_rpy.setZero(_stack * 6);
  _obs_rpyplan.setZero(_stack * 6);
  _obs_posplan.setZero(_stack * 3);
  _obs_pos.setZero(_stack * 3);
  _observation.setZero(139);
  _rpy_plan.setZero(3);
  _motion_done = false;

  _cnt_plot = 0;
  _cnt_idx = 0;
  _drpy_plot.setZero(10000000, 3);
  _torque_plot.setZero(10000000, 7);
  _observation_data.setZero(100000, 139);
  _action_data.setZero(100000, 3);
  _q_plot.setZero(10000, 7);
  _qdot_plot.setZero(10000, 7);
  _qdes_plot.setZero(10000, 7);
  _x_plot.setZero(10000, 6);
  _xdot_plot.setZero(10000, 6);

  _xdes_plot.setZero(10000, 6);
  // _xdoterr_plot.setZero(10000, 3);
  // _xerr_plot.setZero(10000, 3);

  _robot.pos << 0, 0, 0;
  _robot.zrot = 0;      // M_PI;
  _robot.ee_align = 0;  // DEG2RAD * (-90);
  // _robot.ee_align = DEG2RAD * (-45);

  Matrix3d rot_obj(3, 3);
  // rot_obj << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  rot_obj << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  _obj.name = "HANDLE_VALVE";
  // _obj.o_margin << 0, 0.149, 0;
  _obj.o_margin << 0, 0, 0.143;
  _obj.o_margin = rot_obj * _obj.o_margin;
  _obj.r_margin << -0.12, 0, 0;  // East side of the origin
  _obj.r_margin = rot_obj * _obj.r_margin;
  _obj.grab_dir << _obj.o_margin.cross(_obj.r_margin);  //_obj.r_margin;  //
  // _obj.pos << 0.60, 0.0, 0.2;
  _obj.pos << 0.580994101778967, -0.045675755104744684, 0.115;
  // obj.pos << 0.44296161, -0.2819804 ,  0.00434591;
  // _gripper_close = 0.01;  // 0.01;
  // _gripper_open = 0.04;
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

  InitMotionPlan(_init_theta, _goal_theta);
  _obs_obj << _obj.pos(0), _obj.pos(1), _obj.pos(2), rot_obj(0, 0), rot_obj(1, 0), rot_obj(2, 0),
      rot_obj(0, 1), rot_obj(2, 1), rot_obj(3, 1), _init_theta / (2 * M_PI), direction, 1, 0, 1;

  // _obs_obj<<0.5,	0	,0.899999976158142	,0.000796326727141,	0.999999701976776
  // ,0,	-0.999999701976776,	0.000796326727141,	0	,0.75	,-1	,1,0,	1;

  // cout << "object : \n" << _obs_obj.transpose() << endl;
  UpdateStates();

  // string line;
  // // vector<vector<float>> sim_actions;

  // ifstream file;
  // file.open("/home/kist/Downloads/compare_robot/test_act.csv");
  // while (getline(file, line)) {
  //   vector<float> row;
  //   istringstream iss(line);
  //   float value;
  //   while (iss >> value) {
  //     row.push_back(value);
  //     if (iss.peek() == ',')
  //       iss.ignore();
  //   }

  //   sim_actions.push_back(row);
  // }
}

void RLController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  _elapsed_time_ += period;
  _t = _elapsed_time_.toSec();
  _dt = period.toSec();
  // Get Franka's model information

  // transform useful robot states into Eigen format (_q,_qdot,_x_hand, _xdot_hand, rot.. )

  UpdateStates();
  // cout << "x_hand :" << _x_hand.transpose() << endl;
  if (!_motion_done) {
    Target target = _target_plan[_cnt_plan];
    if (_bool_plan[_cnt_plan] && !target.state.empty()) {
      MotionPlan(target);
      switch (target.gripper) {
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
    if ((target.state == "home") || (target.state == "stop")) {
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
        cout << "x_hand :" << _x_hand.transpose() << endl;
        cout << "d_hand :" << _x_des_hand.transpose() << endl;
        cout << "error  :" << (_x_des_hand - _x_hand).transpose() << endl;
        cout << "q : " << _q.transpose() << endl << endl;
      }
    } else if ((target.state == "nearvalve") || (target.state == "detach")) {
      HandTrajectory.update_time(_t);
      _x_des_hand.head(3) = HandTrajectory.position_cubicSpline();
      _xdot_des_hand.head(3) = HandTrajectory.velocity_cubicSpline();
      _R_des_hand = HandTrajectory.rotationCubic();
      _x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(_R_des_hand);
      _xdot_des_hand.segment<3>(3) = HandTrajectory.rotationCubicDot();
      _Rdot_des_hand = CustomMath::GetBodyRotationMatrix(_xdot_des_hand(3), _xdot_des_hand(4),
                                                         _xdot_des_hand(5));

      // _torque_des = OperationalSpaceControl();
      _torque_des = CLIK();
      // GripperControl();

      if (HandTrajectory.check_trajectory_complete() == 1) {
        _cnt_plan += 1;
        _bool_plan[_cnt_plan] = true;
        hand_done = 1;
        printf("target %d is over, move to next target %d \n", _cnt_plan - 1, _cnt_plan);
        printf("It was operational space control.\n");
        // cout << "x_hand :" << _x_hand.transpose() << endl;
        // cout << "d_hand :" << _x_des_hand.transpose() << endl;
        // cout << "error  :" << (_x_des_hand - _x_hand).transpose() << endl;
        // cout<<"##################"<<endl;
        // cout << "q : " << _q.transpose() << endl << endl;
        // cout<<"##################"<<endl;
        // cout<<"_rpy_plan"<<_rpy_plan.transpose()<<endl;

        // MatrixXd R_plan_hand(3, 3);
        // R_plan_hand = CustomMath::GetBodyRotationMatrix(_rpy_plan(0), _rpy_plan(1),
        // _rpy_plan(2)); cout<<R_plan_hand<<endl;
      }

    } else if (target.state == "onvalve") {
      _theta_des = CircularTrajectory.update_time(_t);
      _x_des_hand.head(3) = CircularTrajectory.position_circular();
      _xdot_des_hand.head(3) = CircularTrajectory.velocity_circular();
      // for observation (desired rpy angle reference)
      _rpy_plan = CircularTrajectory.rotation_circular();

      if (_t - _steptime > _stepinterval) {
        _cnt_step = 0;
        _steptime = _t;
        // if (_cnt_plot == 0){
        //   _observation << _obs_obj, _obs_q, _obs_rpy, _obs_rpyplan, _obs_posplan, _obs_pos;
        // }
        // else{
        //   Observation();
        // }
        Observation();
        // for (size_t i=0;i<139;i++){
        //   _observation(i) = sim_actions[_cnt_plot][i];
        // }
        _action = RL_Actor.Forward(_observation);
        // _action <<
        // sim_actions[_cnt_plot][0],sim_actions[_cnt_plot][1],sim_actions[_cnt_plot][2],sim_actions[_cnt_plot][3],sim_actions[_cnt_plot][4],sim_actions[_cnt_plot][5];

        _drpy_pre = _drpy;

        _drpy = CustomMath::rotation6d2euler(_action);

        // _observation_data.row(_cnt_plot) = _observation;
        // _action_data.row(_cnt_plot) = _drpy;
        // _qdes_plot.row(_cnt_plot) = _q_des;
        // _q_plot.row(_cnt_plot) = _q;
        // _qdot_plot.row(_cnt_plot) = _qdot;
        // _xdes_plot.row(_cnt_plot) = _xdot_des_hand;
        // _x_plot.row(_cnt_plot) = _xdot_hand;
        // _cnt_plot += 1;
      }

      if (_drpy_pre[0] == 0.0) {
        if (_cnt_step <= 50) {
          _drpy_out = (_drpy - _drpy_pre) / 50 * _cnt_step + _drpy_pre;

        } else {
          _drpy_out = _drpy;
        }
        
        cout << "cnt :" << _cnt_step << " drpy:" << _drpy_pre.transpose() << " out:" <<
        _drpy_out.transpose()  <<endl;

      } else {
        if (_cnt_step <= 20) {
          _drpy_out = (_drpy - _drpy_pre) / 20 * _cnt_step + _drpy_pre;

        } else {
          _drpy_out = _drpy;
        }
        
      //   cout << "cnt :" << _cnt_step << " drpy:" << _drpy_pre.transpose() << " out:" <<
      //   _drpy_out.transpose() << endl;
      }

      // _drpy_out = _drpy;

      // _observation_data.row(_cnt_plot) = _observation;

      // if (target.time - (_t - _start_time) < 0.05) {
      //   _drpy_out << 0, 0, 0;
      //   _cnt_step = 0;
      // }
      _cnt_step += 1;
      _drpy_plot.row(_cnt_idx) = _drpy_out;

      _x_des_hand.tail(3) = CircularTrajectory.drpy2nextrpy(_drpy_out, _x_des_hand.tail(3));

      _xdot_des_hand.tail(3) = _drpy_out;
      // cout << "drpy out : " << _drpy_out.transpose() << endl;
      _R_des_hand =
          CustomMath::GetBodyRotationMatrix(_x_des_hand(3), _x_des_hand(4), _x_des_hand(5));
      _Rdot_des_hand = CustomMath::GetBodyRotationMatrix(_xdot_des_hand(3), _xdot_des_hand(4),
                                                         _xdot_des_hand(5));

      // _torque_des = OperationalSpaceControl();
      _torque_des = CLIK();
      _cnt_idx += 1;

      VectorXd torque(7);

      for (int i = 0; i < 7; ++i) {
        torque(i) = _torque_des[i];
      }
      _torque_plot.row(_cnt_idx) = torque;
      _q_plot.row(_cnt_plot) = _q;
      _qdot_plot.row(_cnt_plot) = _qdot;
      _x_plot.row(_cnt_plot) = _x_hand;
      _xdot_plot.row(_cnt_plot) = _xdot_hand;
      _drpy_plot.row(_cnt_plot) = _drpy_out;
      _cnt_plot += 1;
      if (CircularTrajectory.check_trajectory_complete() == 1) {
        _cnt_plan += 1;
        _bool_plan[_cnt_plan] = true;
        valve_done = 1;
        printf("on valve target %d is over, move to next target %d\n", _cnt_plan - 1, _cnt_plan);

        cout << "x_hand :" << _x_hand.transpose() << endl;

        cout << "d_hand :" << _x_des_hand.transpose() << endl;

        cout << "error  :" << (_x_des_hand - _x_hand).transpose() << endl;

        cout << "q : " << _q.transpose() << endl << endl;
        gripper_stop = true;
        gripper_close = false;
        gripper_open = false;
      }
    }

    // Gripper
    if (gripper_close) {
      epsilon.inner = 0.05;  // 0.005;
      epsilon.outer = 0.05;  // 0.005;
      close_goal.speed = 0.1;
      close_goal.width = 0.025;  // 0.003;
      close_goal.force = 300.0;
      close_goal.epsilon = epsilon;
      gripper_ac_close.sendGoal(close_goal);
      gripper_close = false;
    }
    if (gripper_open) {
      open_goal.speed = 0.1;
      open_goal.width = 0.08;
      gripper_ac_open.sendGoal(open_goal);
      gripper_open = false;
    }
    if (gripper_stop) {
      gripper_ac_stop.sendGoal(stop_goal);
      gripper_stop = false;

      printf("gripper stop\n");
    }

    for (size_t i = 0; i < 7; ++i) {
      effort_handles_[i].setCommand(_torque_des[i]);
      // effort_handles_[i].setCommand(0.0);
    }

    

  } else {
    array<double, 7> coriolis_array = model_handle_->getCoriolis();
    for (size_t i = 0; i < 7; ++i) {
      effort_handles_[i].setCommand(coriolis_array[i]);
    }

    if (_cnt_plot != 0) {
      // printf("save csv\n");
      // std::string filename;
      // filename = "q.csv";
      // writeToCSVfile(filename, _q_plot, _cnt_plot);

      // filename = "qdot.csv";
      // writeToCSVfile(filename, _qdot_plot, _cnt_plot);

      // // filename = "osc_obs.csv";
      // // writeToCSVfile(filename, _observation_data,_cnt_plot);

      // filename = "hand.csv";
      // writeToCSVfile(filename, _x_plot, _cnt_plot);

      // filename = "drpy.csv";
      // writeToCSVfile(filename, _drpy_plot, _cnt_idx);

      // filename = "handdot.csv";
      // writeToCSVfile(filename, _xdot_plot,_cnt_idx);

      // filename = "clik_q.csv";
      // writeToCSVfile(filename, _q_plot,_cnt_plot);

      // filename = "clik_qdot.csv";
      // writeToCSVfile(filename, _qdot_plot,_cnt_plot);

      // filename = "clik_obs.csv";
      // writeToCSVfile(filename, _observation_data,_cnt_plot);

      // filename = "clik_drpy.csv";
      // writeToCSVfile(filename, _action_data,_cnt_plot);

      // filename = "clik_drpy.csv";
      // writeToCSVfile(filename, _drpy_plot,_cnt_idx);

      // filename = "clik_torque.csv";
      // writeToCSVfile(filename, _torque_plot,_cnt_idx);
      _cnt_plot = 0;
    }
  
  
  }
}

void RLController::writeToCSVfile(string filename, MatrixXd input_matrix, int cnt) {
  // Open the file for writing
  string path = "/home/kist/catkin_ws/src/franka_ros/franka_valve/data/";
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

void RLController::gripperCloseCallback(const std_msgs::Bool& msg) {
  gripper_close = msg.data;
  gripper_open = false;
}

void RLController::gripperOpenCallback(const std_msgs::Bool& msg) {
  gripper_open = msg.data;
  gripper_close = false;
}

void RLController::gripperStopCallback(const std_msgs::Bool& msg) {
  gripper_stop = msg.data;
  gripper_open = false;
  gripper_close = false;
}

array<double, 7> RLController::saturateTorqueRate(
    const array<double, 7>& tau_d_calculated,
    const array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + max(min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}
void RLController::InitMotionPlan(double init_theta, double goal_theta) {
  // start position -> move to valve -> generate circular trajectory -> start position

  double motion_time_const = 10.0;
  double motion_time;

  Target onvalve;
  onvalve.state = "onvalve";

  Target detach;
  detach.state = "detach";
  detach.gripper = GRIPPER_OPEN;

  Target stop;
  stop.state = "stop";

  Target sim_q;
  sim_q.state = "home";
  sim_q.q_goal = {0.519943799922697,  -0.659952333162895, -0.201899050887602, -1.54192546153917,
                  -0.563581560578316, 2.4272166518006,    -0.469298839272077};

  Target home;
  home.state = "home";
  home.q_goal = {0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397};
  // home.q_goal = {0.374, -1.02, 0.245, -1.51, 0.0102, 0.655,  0.3};

  _target_plan.push_back(home);
  _target_plan.back().gripper = GRIPPER_OPEN;  //_gripper_open;
  _target_plan.back().time = 2.0;

  Objects obj_above = _obj;
  obj_above.o_margin = obj_above.o_margin + obj_above.o_margin.normalized() * 0.05;
  _target_plan.push_back(TargetTransformMatrix(obj_above, _robot, init_theta));
  _target_plan.back().gripper = GRIPPER_OPEN;  //_gripper_open;
  _target_plan.back().time = 5.0;
  _target_plan.back().state = "nearvalve";
  cout << "target 1:" << _target_plan.back().x << "," << _target_plan.back().y << ","
       << _target_plan.back().z << "," << _target_plan.back().roll << ","
       << _target_plan.back().pitch << "," << _target_plan.back().yaw << endl;

  _target_plan.push_back(TargetTransformMatrix(_obj, _robot, init_theta));
  _target_plan.back().gripper = GRIPPER_OPEN;  //_gripper_open;
  _target_plan.back().time = 1.0;
  _target_plan.back().state = "nearvalve";

  // _target_plan.push_back(sim_q);
  // _target_plan.back().gripper = true;  //_gripper_open;
  // _target_plan.back().time = 3.0;

  cout << "target 2:" << _target_plan.back().x << "," << _target_plan.back().y << ","
       << _target_plan.back().z << "," << _target_plan.back().roll << ","
       << _target_plan.back().pitch << "," << _target_plan.back().yaw << endl;

  _target_plan.push_back(TargetTransformMatrix(_obj, _robot, init_theta));
  _target_plan.back().gripper = GRIPPER_CLOSE;  //_gripper_close;
  _target_plan.back().time = 0.5;
  _target_plan.back().state = "nearvalve";
  cout << "target 3:" << _target_plan.back().x << "," << _target_plan.back().y << ","
       << _target_plan.back().z << "," << _target_plan.back().roll << ","
       << _target_plan.back().pitch << "," << _target_plan.back().yaw << endl;

  _target_plan.push_back(onvalve);
  _target_plan.back().gripper = GRIPPER_CLOSE;  //_gripper_close;
  motion_time = abs(motion_time_const * abs(goal_theta - init_theta) * _obj.r_margin.norm());
  _target_plan.back().time = motion_time;
  // _target_plan.back().gripper = _gripper_close;
  cout << "target 4:" << _target_plan.back().x << "," << _target_plan.back().y << ","
       << _target_plan.back().z << "," << _target_plan.back().roll << ","
       << _target_plan.back().pitch << "," << _target_plan.back().yaw << endl;

  _target_plan.push_back(stop);
  _target_plan.back().time = 1.0;
  _target_plan.back().gripper = GRIPPER_STOP;

  _target_plan.push_back(detach);
  _target_plan.back().time = 1.0;
  _target_plan.back().gripper = GRIPPER_OPEN;

  _target_plan.push_back(home);
  _target_plan.back().gripper = GRIPPER_OPEN;  //_gripper_open;
  _target_plan.back().time = 3.0;
}

void RLController::MotionPlan(Target target) {
  _bool_plan[_cnt_plan] = false;
  _start_time = _t;
  _end_time = _start_time + target.time;

  if (target.state == "home") {
    VectorXd q_goal(7), qdot_goal(7);

    q_goal << target.q_goal[0], target.q_goal[1], target.q_goal[2], target.q_goal[3],
        target.q_goal[4], target.q_goal[5], target.q_goal[6];
    qdot_goal.setZero();

    JointTrajectory.reset_initial(_start_time, _q, _qdot);
    JointTrajectory.update_goal(q_goal, qdot_goal, _end_time);

  } else if (target.state == "stop") {
    VectorXd q_goal(7), qdot_goal(7);

    q_goal = _q;
    qdot_goal.setZero();

    // cout<<"q_now:"<<_q.transpose()<<endl;
    // cout<<"q_dot:"<<_qdot.transpose()<<endl;

    JointTrajectory.reset_initial(_start_time, _q, _qdot);
    JointTrajectory.update_goal(q_goal, qdot_goal, _end_time);

  } else if (target.state == "nearvalve") {
    VectorXd x_goal_hand(6), xdot_goal_hand(6);

    x_goal_hand(0) = target.x;
    x_goal_hand(1) = target.y;
    x_goal_hand(2) = target.z;
    x_goal_hand(3) = target.roll;
    x_goal_hand(4) = target.pitch;
    x_goal_hand(5) = target.yaw;

    xdot_goal_hand.setZero();

    HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
    HandTrajectory.update_goal(x_goal_hand, xdot_goal_hand, _end_time);

  } else if (target.state == "onvalve") {
    cout << "\n\n######################ON VALVE##########################\n<<endl";

    _x_des_hand = _x_hand;
    _xdot_des_hand = _xdot_hand;
    _q_des = _q;
    _qdot_des = _qdot;

    CircularTrajectory.reset_initial(_start_time, _obj.grab_vector, _obj.normal_vector, _obj.radius,
                                     _Tvr, _dt);
    CircularTrajectory.update_goal(_end_time, _init_theta, _goal_theta);
  } else if (target.state == "detach") {
    VectorXd x_goal_hand(6), xdot_goal_hand(6);
    VectorXd tmp(3);
    tmp << 0, 0, -_obj.o_margin.norm() * 0.5;
    tmp << CustomMath::GetBodyRotationMatrix(_x_hand(3), _x_hand(4), _x_hand(5)) * tmp;
    x_goal_hand(0) = _x_hand(0) + tmp(0);
    x_goal_hand(1) = _x_hand(1) + tmp(1);
    x_goal_hand(2) = _x_hand(2) + tmp(2);
    x_goal_hand(3) = _x_hand(3);
    x_goal_hand(4) = _x_hand(4);
    x_goal_hand(5) = _x_hand(5);
    xdot_goal_hand.setZero();

    HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
    HandTrajectory.update_goal(x_goal_hand, xdot_goal_hand, _end_time);
  }
}

void RLController::UpdateStates() {
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
  // cout<<"position : "<<ee_position.transpose()<<endl;
  _x_hand.head(3) = ee_position;
  _x_hand.tail(3) = CustomMath::GetBodyRotationAngle(ee_rotation);
  _R_hand = ee_rotation;

  _xdot_hand = _J_hands * _qdot;
  _Rdot_hand = CustomMath::GetBodyRotationMatrix(_xdot_hand(3), _xdot_hand(4), _xdot_hand(5));

  if (_cnt_plan < _target_plan.size()) {
    _motion_done = false;
  } else {
    _motion_done = true;
  }
  ZMQ();
}

void RLController::Observation() {
  MatrixXd R_plan_hand(3, 3);
  R_plan_hand = CustomMath::GetBodyRotationMatrix(_rpy_plan(0), _rpy_plan(1), _rpy_plan(2));
  for (size_t i = 0; i < 7; ++i) {
    _q_normalize(i) =
        (_q(i) - _q_normalize_min(i)) / (_q_normalize_max(i) - _q_normalize_min(i)) * (1 - (-1)) -
        1;
  }
  Matrix3d Tge, R_hand;
  Tge << cos(M_PI_4), -sin(M_PI_4), 0, sin(M_PI_4), cos(M_PI_4), 0, 0, 0, 1;
  R_hand << _R_hand * Tge;
  R_plan_hand << R_plan_hand * Tge;

  if (_observation.isZero()) {
    // _obs_rpy.head(6) << R_hand(0, 0), R_hand(0, 1), R_hand(0, 2), R_hand(1, 0), R_hand(1, 1),
    //     R_hand(1, 2);
    // _obs_rpyplan.head(6) << R_plan_hand(0, 0), R_plan_hand(0,1), R_plan_hand( 0,2),
    //     R_plan_hand(1,0), R_plan_hand(1, 1), R_plan_hand(1,2);

    _obs_rpy.head(6) << R_hand(0, 0), R_hand(1, 0), R_hand(2, 0), R_hand(0, 1), R_hand(1, 1),
        R_hand(2, 1);
    _obs_rpyplan.head(6) << R_plan_hand(0, 0), R_plan_hand(1, 0), R_plan_hand(2, 0),
        R_plan_hand(0, 1), R_plan_hand(1, 1), R_plan_hand(2, 1);

    _obs_posplan.head(3) << _x_des_hand.head(3);
    _obs_pos.head(3) << _x_hand.head(3);
    _obs_q.head(7) << _q_normalize;
    for (size_t i = 1; i < _stack; i++) {
      _obs_rpy.segment<6>(6 * i) << _obs_rpy.head(6);
      _obs_rpyplan.segment<6>(6 * i) << _obs_rpyplan.head(6);
      _obs_posplan.segment<3>(3 * i) << _obs_posplan.head(3);
      _obs_pos.segment<3>(3 * i) << _obs_pos.head(3);
      _obs_q.segment<7>(7 * i) << _obs_q.head(7);
    }
  } else {
    VectorXd q_tmp((_stack - 1) * 7);
    q_tmp = _obs_q.head((_stack - 1) * 7);
    VectorXd rpy_tmp((_stack - 1) * 6);
    rpy_tmp = _obs_rpy.head((_stack - 1) * 6);
    VectorXd rpyplan_tmp((_stack - 1) * 6);
    rpyplan_tmp = _obs_rpyplan.head((_stack - 1) * 6);
    VectorXd posplan_tmp((_stack - 1) * 3);
    posplan_tmp = _obs_posplan.head((_stack - 1) * 3);
    VectorXd pos_tmp((_stack - 1) * 3);
    pos_tmp = _obs_pos.head((_stack - 1) * 3);

    _obs_q.tail((_stack - 1) * 7) = q_tmp;
    _obs_rpy.tail((_stack - 1) * 6) = rpy_tmp;
    _obs_rpyplan.tail((_stack - 1) * 6) = rpyplan_tmp;
    _obs_posplan.tail((_stack - 1) * 3) = posplan_tmp;
    _obs_pos.tail((_stack - 1) * 3) = pos_tmp;

    _obs_rpy.head(6) << R_hand(0, 0), R_hand(1, 0), R_hand(2, 0), R_hand(0, 1), R_hand(1, 1),
        R_hand(2, 1);
    _obs_rpyplan.head(6) << R_plan_hand(0, 0), R_plan_hand(1, 0), R_plan_hand(2, 0),
        R_plan_hand(0, 1), R_plan_hand(1, 1), R_plan_hand(2, 1);
    _obs_posplan.head(3) << _x_des_hand.head(3);
    _obs_pos.head(3) << _x_hand.head(3);
    _obs_q.head(7) << _q_normalize;
  }

  _observation << _obs_obj, _obs_q, _obs_rpy, _obs_rpyplan, _obs_posplan, _obs_pos;
}

void RLController::ZMQ_receive() {
  socket2.connect("tcp://161.122.114.37:5558");

  while (1) {
    auto result = socket2.recv(reply);

    memcpy(Buffer_Robot.data(), reply.data(), 7 * sizeof(float));
    _cnt_recv = 0;
    for (size_t i = 0; i < 7; ++i) {
      _q_mppi(i) = Buffer_Robot[i];
      _qdot_mppi(i) = Buffer_Robot[i + 7];
    }
  }
}

void RLController::ZMQ() {
  zmq_cnt += 1;

  if ((zmq_start) && (zmq_cnt == 1000)) {
    // cout<<"zmq :"<<zmq_cnt<<endl;
    std::chrono::steady_clock::time_point st__start_time;
    st__start_time = std::chrono::steady_clock::now();

    double control_time_real_ = 0.0;

    socket.bind("tcp://*:5557");
    control_time_real_ = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now() - st__start_time)
                             .count();
    control_time_real_ = control_time_real_ / 1000;
    // cout << "all : " << control_time_real_ << "ms" << endl << endl;
    zmq_start = false;
    zmq_cnt = 0;

    std::thread th1(&RLController::ZMQ_receive, this);

    th1.detach();

    // sleep(1);
    // cout<<"join";
  }
  if (zmq_start == false) {
    for (int i = 0; i < 7; i++) {
      Buffer[i] = _q(i);
      Buffer[i + 7] = _qdot(i);
      // cout<<Buffer[i];
    }
    // cout<<endl;

    zmq_cnt = 0;
    socket.send(zmq::buffer(Buffer), zmq::send_flags::dontwait);
  }
}
array<double, 7> RLController::OperationalSpaceControl() {
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();
  VectorXd force(6), torque(7), x_err_hand(6), xdot_err_hand(6);
  force.setZero(6);
  torque.setZero(7);
  x_err_hand.setZero(6);
  xdot_err_hand.setZero(6);

  Matrix<double, 7, 6> J_bar_hands(CustomMath::pseudoInverseQR(_J_hands));
  Matrix<double, 6, 6> lambda(CustomMath::pseudoInverseQR(_J_hands.transpose()) * _mass *
                              J_bar_hands);

  x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);
  x_err_hand.segment(3, 3) = -CustomMath::getPhi(_R_hand, _R_des_hand);

  xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
  xdot_err_hand.segment(3, 3) = -CustomMath::getPhi(_Rdot_hand, _Rdot_des_hand);

  // force = k_gains_ * x_err_hand + d_gains_ * xdot_err_hand;

  force = _xk_gains.cwiseProduct(x_err_hand) + _xd_gains.cwiseProduct(xdot_err_hand);
  // force = _xk_gains.cwiseProduct(x_err_hand) + _xd_gains.cwiseProduct(-_xdot_hand);

  torque = _J_hands.transpose() * lambda * force + coriolis_factor_ * _coriolis;

  _torque_plot.row(_cnt_idx) = torque;
  array<double, 7> tau_d;
  for (int i = 0; i < 7; ++i) {
    tau_d[i] = torque(i);
  }
  array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d, robot_state_.tau_J_d);

  if (_cnt_idx < 5) {
    cout.precision(3);
    cout << "action :" << _action.transpose() << endl;
    cout << "drpy     :" << _drpy.transpose() << endl;
    cout << "drpy out :" << _drpy_out.transpose() << endl;
    cout << "xerr:" << x_err_hand.transpose() << endl;
    cout << "xderr:" << xdot_err_hand.transpose() << endl;
    cout << "xdot    " << _xdot_hand.transpose() << endl;
    cout << "xdotdes " << _xdot_des_hand.transpose() << endl;

    cout << force.transpose() << endl << endl;
    cout << torque.transpose() << endl << endl;
    cout << "#####" << endl;
  }
  return tau_d_saturated;
}

array<double, 7> RLController::CLIK() {
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();
  VectorXd torque(7), x_err_hand(6);
  Matrix<double, 7, 6> J_bar_hands(CustomMath::pseudoInverseQR(_J_hands));

  x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);
  x_err_hand.segment(3, 3) = -CustomMath::getPhi(_R_hand, _R_des_hand);

  _qdot_des = J_bar_hands * (_xdot_des_hand + 1 * (x_err_hand));  // + _x_err_hand.norm()*_x_force);
  _q_des = _q_des + _dt * _qdot_des;

  torque = _mass * (_k_gains.cwiseProduct(_q_des - _q) + _d_gains.cwiseProduct(_qdot_des - _qdot)) +
           coriolis_factor_ * _coriolis;
  array<double, 7> tau_d;

  for (int i = 0; i < 7; ++i) {
    tau_d[i] = torque(i);
  }
  array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d, robot_state_.tau_J_d);

  // if (_cnt_idx < 5){
  //     cout.precision(3);
  //     cout<<"action :"<<_action.transpose()<<endl;
  //     cout<<"drpy     :"<<_drpy.transpose()<<endl;
  //     cout<<"drpy out :"<<_drpy_out.transpose()<<endl;
  //     cout<<"xerr:"<<x_err_hand.transpose()<<endl;
  //     cout<<"xdot    "<<_xdot_hand.transpose()<<endl;
  //     cout<<"xdotdes "<<_xdot_des_hand.transpose()<<endl;

  //     cout<<torque.transpose()<<endl<<endl;
  //     cout<<"#####"<<endl;

  //   }

  return tau_d_saturated;
}

array<double, 7> RLController::JointControl() {
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();

  VectorXd force(6), torque(7), torque_0(7);

  torque = _mass * (_k_gains.cwiseProduct(_q_des - _q) + _d_gains.cwiseProduct(_qdot_des - _qdot)) +
           coriolis_factor_ * _coriolis;

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

RLController::Target RLController::TargetTransformMatrix(Objects obj, Robot robot, double angle) {
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
  Tug = RLController::R3D(obj, -obj.o_margin.normalized(), angle);
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

Matrix3d RLController::R3D(Objects obj, Vector3d unitVec, double angle) {
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

PLUGINLIB_EXPORT_CLASS(franka_valve::RLController, controller_interface::ControllerBase)
