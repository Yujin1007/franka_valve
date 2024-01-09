// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_valve/joint_position_controller.h>

#include <pthread.h>
#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>

namespace franka_valve {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle) {
  // position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  // if (position_joint_interface_ == nullptr) {
  //   ROS_ERROR("JointPositionController: Error getting position joint interface from hardware!");
  //   return false;
  // }

  string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("RLController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  // position_joint_handles_.resize(7);
  // for (size_t i = 0; i < 7; ++i) {
  //   try {
  //     position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
  //   } catch (const hardware_interface::HardwareInterfaceException& e) {
  //     ROS_ERROR_STREAM("JointPositionController: Exception getting joint handles: " << e.what());
  //     return false;
  //   }
  // }

auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("OperationalSpaceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  }
    
  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointSpaceImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      effort_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointSpaceImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  // for (size_t i = 0; i < q_start.size(); i++) {
  //   if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
  //     ROS_ERROR_STREAM(
  //         "JointPositionController: Robot is not in the expected starting position for "
  //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //     return false;
  //   }
  // }

 auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("JointSpaceImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ =
        make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointSpaceImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  // pthread_t recv_th1;
  // int cnt_th1;
  // // cnt_th1 = pthread_create(&recv_th1, NULL, &ZMQ_receive, (void *)&ctime);
  // cnt_th1 = pthread_create(&recv_th1, NULL, ZMQ_receive, (void*)&ctime);
  // // std::thread recv_th(ZMQ_receive);
  // pthread_join(recv_th1,0);
  // for (size_t i = 0; i < 7; ++i) {
  //   initial_pose_[i] = position_joint_handles_[i].getPosition();
  // }
  elapsed_time_ = ros::Duration(0.0);
  _q.setZero(7);
  _qdot.setZero(7);
  zmq_start = true;
  zmq_cnt = 0;
  _cnt_recv = -1000;
  _q_traj.setZero(7, 10);
  _qdot_traj.setZero(7, 10);
  _q_tmp.setZero(7);
  _qdot_tmp.setZero(7);
  _q_mppi.setZero(7);
  _qdot_mppi.setZero(7);
  _torque_des.setZero(7);

  _old_q_mppi.setZero(7);
  _old_qdot_mppi.setZero(7);
  _q_delta.setZero(7);
  _qdot_delta.setZero(7);

  _time = 0.0;
  _ZMQ_receive_start = true;
}

void JointPositionController::ZMQ_receive() {
  // cout<<"1"<<endl;
  socket2.connect("tcp://161.122.114.37:5558");
  // cout<<"2"<<endl;
  while (1) {
    // cout<<"3"<<endl;
    // if(_time >= 5.0)
    // {
    _old_q_mppi = _q;
    _old_qdot_mppi = _qdot;
    
    auto result = socket2.recv(reply);
    if(_ZMQ_receive_start == true)
    {
      _old_q_mppi = _q;
      _old_qdot_mppi = _qdot;
      _ZMQ_receive_start = false;
    }
    // cout<<"zmq 3:"<<zmq_cnt<<endl;
    memcpy(Buffer_Robot.data(), reply.data(), 14 * sizeof(float));
    _cnt_recv = 0;
    for (size_t i=0;i<7;++i){
      _q_mppi(i) = Buffer_Robot[i];
      _qdot_mppi(i) = Buffer_Robot[i+7];
      // cout<<Buffer_Robot[i]<<endl;
      // cout<<Buffer_Robot[i+7]<<endl<<endl;
    }
    for(int i = 0;i<14; i++)
    {
      cout<<Buffer_Robot[i]<<" , " <<endl;
    }

    _q_delta = _q_mppi - _old_q_mppi;
    _q_delta = _q_delta / 4;
    _qdot_delta = _qdot_mppi - _old_qdot_mppi;
    _qdot_delta = _qdot_delta / 4;
    
    cout<<"_time          : "<<_time<<endl;
    // cout<<"_q_mppi        : "<<_q_mppi.transpose()<<endl;
    // cout<<"_old_q_mppi    : "<<_old_q_mppi.transpose()<<endl;
    // cout<<"_qdot_mppi     : "<<_qdot_mppi.transpose()<<endl;
    // cout<<"_old_qdot_mppi : "<<_old_qdot_mppi.transpose()<<endl;
    // cout<<"_qdot_delta    : "<<_qdot_delta.transpose()<<endl<<endl;

    
    for (size_t j = 0; j < 7; ++j) {
      for (size_t i = 0; i < 10; ++i) {
        if (i == 0) {
          _qdot_traj(j, 0) = _qdot_delta(j) + _old_qdot_mppi(j);
          _q_traj(j, 0) = _q_delta(j) + _old_q_mppi(j);
        } else {
          _qdot_traj(j, i) = _qdot_traj(j, i - 1) + _qdot_delta(j);
          _q_traj(j, i) = _q_traj(j, i - 1) + _q_delta(j);
        }
      }
    }
  }
}

void JointPositionController::ZMQ() {
  zmq_cnt += 1;

  if ((zmq_start) && (zmq_cnt == 1000)) {
    // cout<<"zmq :"<<zmq_cnt<<endl;
    std::chrono::steady_clock::time_point st_start_time;
    st_start_time = std::chrono::steady_clock::now();

    double control_time_real_ = 0.0;

    socket.bind("tcp://*:5557");
    // cout<<"zmq 1:"<<zmq_cnt<<endl;
    // cout<<input<<endl;s
    control_time_real_ = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now() - st_start_time)
                             .count();
    control_time_real_ = control_time_real_ / 1000;
    // cout << "all : " << control_time_real_ << "ms" << endl << endl;
    zmq_start = false;
    zmq_cnt = 0;

    std::thread th1(&JointPositionController::ZMQ_receive, this);

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
    //     if (socket.send(zmq::buffer(Buffer), zmq::send_flags::dontwait)) {
    //     // 데이터가 성공적으로 송신된 경우
    //     std::cout << "Message sent successfully." << std::endl;
    // } else {
    //     // 큐가 가득 차서 송신에 실패한 경우
    //     std::cerr << "Message not sent. Handle the case where the message queue is full." << std::endl;
    //     // 이곳에서 적절한 오류 처리를 수행하거나 재시도할 수 있습니다.
    // }

  }
}

void JointPositionController::controller(){
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();
  array<double, 7> coriolis_array = model_handle_->getCoriolis();
  array<double, 49> mass_array = model_handle_->getMass();
_mass = Map<const Matrix<double, 7, 7>>(mass_array.data());
  _coriolis = Map<const Matrix<double, 7, 1>>(coriolis_array.data());
  vector<double> armateur = {0.5, 0.5, 0.3, 0.3, 0.3, 0.3, 0.2, 0.12};
  for (size_t i = 0; i < 7; ++i) {
    _mass(i, i) += armateur[i];
  }
  // cout<<"_q_traj.col(_cnt_recv) : "<<_q_traj.col(_cnt_recv).transpose()<<endl;
  // cout<<"_q                     : "<<_q.transpose()<<endl;
  // cout<<"_qdot_traj.col(_cnt_recv) : "<<_qdot_traj.col(_cnt_recv).transpose()<<endl;
  // cout<<"_qdot                     : "<<_qdot.transpose()<<endl;
  // cout<<"_coriolis : "<<_coriolis.transpose()<<endl<<endl;
  // cout<<"_mass : "<<_mass<<endl<<endl;

  _q_err = _q_traj.col(_cnt_recv) - _q;  
  // _qdot_err = - _qdot;
  _qdot_err = _qdot_traj.col(_cnt_recv) - _qdot;
  // for(int i = 0 ; i<7; i++)
  // {
  //   if(_q_err(i) > 0.1)
  //   {
  //     _q_err(i) = 0.1;
  //   }
  //   else if(_q_err(i) < -0.1)
  //   {
  //     _q_err(i) = -0.1;
  //   }
  // }
  _torque_des = _mass*(400 * _q_err + 40*_qdot_err);// + _coriolis;

  array<double, 7> tau_d;
  for (int i = 0; i < 7; ++i) {
    tau_d[i] = _torque_des(i);
  }
  array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d, robot_state_.tau_J_d);  
  for (int i = 0; i < 7; ++i) {
    _torque_des(i)=tau_d_saturated[i];
  }
  // cout<<"_q_err                 : "<<_q_err.transpose()<<endl;
  // cout<<"_qdot_err              : "<<_qdot_err.transpose()<<endl;
  // cout<<"_torque_des              : "<<_torque_des.transpose()<<endl<<endl;
}

array<double, 7> JointPositionController::saturateTorqueRate(
    const array<double, 7>& tau_d_calculated,
    const array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + max(min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void JointPositionController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  elapsed_time_ += period;
  // const franka::RobotState& robot_state_ = state_handle_->getRobotState();
  _time = elapsed_time_.toSec();
  for (size_t i = 0; i < 7; ++i) {
    _q(i) = effort_handles_[i].getPosition();
    _qdot(i) = effort_handles_[i].getVelocity();
  }
  _q_tmp = _q;
  _qdot_tmp = _qdot;
  // Eigen::Map<const Eigen::Matrix<double, 7, 1>> _q(robot_state_.q.data());
  // Eigen::Map<const Eigen::Matrix<double, 7, 1>> _qdot(robot_state_.dq.data());

  // ZMQ_send(_q, _qdot);
  ZMQ();
  controller();
  // cout << "trajectory :\n" << _q_traj << endl;
  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.4;
  // printf("\ndesired : ");
  // double tmp_sum = 0;
  // for (size_t i = 0; i < 7; ++i) {
  //   tmp_sum += _q_traj(i, 0);
  // }
  // if (elapsed_time_.toSec() >= 5.0) {
  if (_ZMQ_receive_start == false) {
    // for (size_t i = 0; i < 7; ++i) {
    //   // position_joint_handles_[i].setCommand(_q_traj(i, _cnt_recv));
    //   position_joint_handles_[i].setCommand(_q_mppi(i));
    // }

    for (size_t i = 0; i < 7; ++i) {

      // effort_handles_[i].setCommand(_torque_des(i));
      // array<double, 7> coriolis_array = model_handle_->getCoriolis();
      effort_handles_[i].setCommand(0.0);
    }

    _cnt_recv += 1;
    if (_cnt_recv == 10) {
      _cnt_recv = 9;
    }
  } else {
     for (size_t i = 0; i < 7; ++i) {
      effort_handles_[i].setCommand(0.0);
      // array<double, 7> coriolis_array = model_handle_->getCoriolis();
      // effort_handles_[i].setCommand(0.0);
    }
  }
  // cout<<_q_mppi.transpose()<<endl;
  // printf("\ncurrent : ");
  // cout<<_q.transpose()<<endl;
  // cout<<_qdot.transpose()<<endl;
}

}  // namespace franka_valve

PLUGINLIB_EXPORT_CLASS(franka_valve::JointPositionController, controller_interface::ControllerBase)
