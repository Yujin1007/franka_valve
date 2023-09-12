// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_valve/operational_space_controller.h>
#include <franka_valve/trajectory.h>
#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

namespace franka_valve {

bool OperationalSpaceController::init(hardware_interface::RobotHW* robot_hw,
                                      ros::NodeHandle& node_handle) {

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("OperationalSpaceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "OperationalSpaceController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_)) {
    ROS_INFO_STREAM(
        "OperationalSpaceController:  Invalid or no k_gain parameters provided,  Use default value ");
  }

  if (!node_handle.getParam("d_gains", d_gains_)) {
    ROS_INFO_STREAM(
        "OperationalSpaceController:  Invalid or no d_gain parameters provided, Use default value");
  }

    
  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("OperationalSpaceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

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

 auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }







  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "OperationalSpaceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "OperationalSpaceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  //   torques_publisher_.init(node_handle, "torque_comparison", 1);

  //   std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void OperationalSpaceController::starting(const ros::Time& /* time */) {
  
  // Get initial end-effector's pose
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();
  
  
  gripper_close = false;
  gripper_open = false;

  elapsed_time_ = ros::Duration(0.0);
  printtime = 0.0;
  timeinterval = 0.1;

  // Initial setting / calculate hand trajectory in advance

  const std::array<double, 42>& jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  Eigen::Affine3d ee_pose(Eigen::Matrix4d::Map(robot_state_.O_T_EE.data()));
  Eigen::Vector3d _ee_position(ee_pose.translation());
  Eigen::Matrix3d _R_hand(ee_pose.linear());
  Eigen::Vector3d _ee_rot(CustomMath::GetBodyRotationAngle(_R_hand));
  Eigen::VectorXd _x_hand(6); _x_hand<<_ee_position, _ee_rot;
  Eigen::VectorXd _xdot_hand(6); _xdot_hand.setZero(6);

  Eigen::VectorXd _x_goal_hand(6);
  Eigen::VectorXd _xdot_goal_hand(6); _xdot_goal_hand.setZero(6);
  Eigen::Matrix3d goal_rotation;
  goal_rotation << 0.9839500605991979, 0.17205434566608574, 0.04712034895306263, 0.1720841410986481,
      -0.9850664264302305, 0.0034540995243698414, 0.047011871713080355, 0.0047100940254425905,
      -0.9988832258739279;
  _x_goal_hand.head(3) << 0.5719940302730065, 0.08550089664276542, 0.39738780129846685;
  _x_goal_hand.tail(3) << CustomMath::GetBodyRotationAngle(goal_rotation);

  HandTrajectory.reset_initial(elapsed_time_.toSec(), _x_hand, _xdot_hand);
  HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, elapsed_time_.toSec() + 5.0);
}

void OperationalSpaceController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  
  elapsed_time_ += period;

  // Get Franka's model information
  const franka::RobotState& robot_state_ = state_handle_->getRobotState();

  ///////////////////////////////////////
  // Get robot model
  // Jaobian / EE pose /
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 49> mass_array = model_handle_->getMass();

  const std::array<double, 42>& jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> _J_hands(jacobian_array.data());

  Eigen::Map<const Eigen::Matrix<double, 7, 1>> _q(robot_state_.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> _qd(robot_state_.dq.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> _mass(mass_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> _coriolis(coriolis_array.data());

  Eigen::Affine3d ee_pose(Eigen::Matrix4d::Map(robot_state_.O_T_EE.data()));
  Eigen::Vector3d _x_hand(ee_pose.translation());
  Eigen::Matrix3d _R_hand(ee_pose.linear());

  Eigen::VectorXd _xdot_hand(_J_hands * _qd);
  Eigen::Matrix3d _Rdot_hand(
      CustomMath::GetBodyRotationMatrix(_xdot_hand(3), _xdot_hand(4), _xdot_hand(5)));

  // double radius = 0.15;
  // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  // double delta_x = radius * std::sin(angle);
  // double delta_z = radius * (std::cos(angle) - 1);
  // Eigen::Vector3d new_pos = ee_position;
  // new_pos[0] += delta_x;
  // new_pos[2] += delta_z;
  // if (elapsed_time_.toSec() < 10.0)
  //   gripper_close = true;
  // else{
  //   gripper_open =true;
  // }

  // Gripper
  if (gripper_close) {
    epsilon.inner = 0.05;  // 0.005;
    epsilon.outer = 0.05;  // 0.005;
    close_goal.speed = 0.1;
    close_goal.width = 0.005;  // 0.003;
    close_goal.force = 100.0;
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

  // Get desired trajectory
  HandTrajectory.update_time(elapsed_time_.toSec());

  Eigen::Vector3d _x_des_hand(HandTrajectory.position_cubicSpline());
  Eigen::Matrix3d _R_des_hand(HandTrajectory.rotationCubic());

  Eigen::Vector3d _xdot_des_hand(HandTrajectory.velocity_cubicSpline());
  Eigen::Vector3d _xdot_des_hand_r(HandTrajectory.rotationCubicDot());
  Eigen::Matrix3d _Rdot_des_hand(CustomMath::GetBodyRotationMatrix(
      _xdot_des_hand_r(0), _xdot_des_hand_r(1), _xdot_des_hand_r(2)));

  // Operational space control

  Eigen::Matrix<double, 7, 6> _J_bar_hands(CustomMath::pseudoInverseQR(_J_hands));
  Eigen::Matrix<double, 6, 6> _lambda(CustomMath::pseudoInverseQR(_J_hands.transpose()) * _mass *
                                      _J_bar_hands);

  Eigen::VectorXd _x_err_hand(6), _xdot_err_hand(6), _force(6), _torque(7);
  

  _x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);
  _x_err_hand.segment(3, 3) = -CustomMath::getPhi(_R_hand, _R_des_hand);

  _xdot_err_hand.segment(0, 3) = _xdot_des_hand.head(3) - _xdot_hand.head(3);
  _xdot_err_hand.segment(3, 3) = -CustomMath::getPhi(_Rdot_hand, _Rdot_des_hand);

  _force = k_gains_ * _x_err_hand + d_gains_ * _xdot_err_hand;

  _torque = _J_hands.transpose() * _lambda * _force + _coriolis;

  std::array<double, 7> _tau_d;
  for (int i = 0; i < 7; ++i) {
    _tau_d[i] = _torque(i);
  }
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(_tau_d, robot_state_.tau_J_d);

  // CLIK
  //   int _x_kp = 20;
  //   HandTrajectory.update_time(elapsed_time_.toSec());
  //   _x_des_hand.head(3) =
  //       HandTrajectory.position_cubicSpline();  // 이거까지는 맞게 출력되는거 확인 했음.
  //   _R_des_hand = HandTrajectory.rotationCubic();
  //   _x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(
  //       _R_des_hand);  // x desired 값은 잘 나온다. 그럼 jacobian 문제?
  //   _xdot_des_hand.head(3) = HandTrajectory.velocity_cubicSpline();
  //   _xdot_des_hand.segment<3>(3) = HandTrajectory.rotationCubicDot();
  //   _x_err_hand.segment(0, 3) = _x_des_hand.head(3) - ee_position;
  //   _R_des_hand = CustomMath::GetBodyRotationMatrix(_x_des_hand(3), _x_des_hand(4),
  //   _x_des_hand(5)); _x_err_hand.segment(3, 3) = -CustomMath::getPhi(ee_rotation, _R_des_hand);
  //   _J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);
  //   _qdot_des = _J_bar_hands * (_xdot_des_hand + _x_kp * (_x_err_hand));
  //   // _qdot_des = _J_bar_hands*(_xdot_des_hand);
  //   _q_des = _q_des + period.toSec() * _qdot_des;  //_q 를 넣으면 진동..

  if (HandTrajectory.check_trajectory_complete() == 0) {
    if (elapsed_time_.toSec() - printtime > timeinterval) {
      printtime = elapsed_time_.toSec();
    }
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d_saturated[i]);
    // joint_handles_[i].setCommand(coriolis_array[i]);
      std::cout << tau_d_saturated[i] << "  ";
    }
  }
  else{
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(coriolis_array[i]);
    }
  }
}

void OperationalSpaceController::gripperCloseCallback(const std_msgs::Bool& msg) {
  gripper_close = msg.data;
  gripper_open = false;
}

void OperationalSpaceController::gripperOpenCallback(const std_msgs::Bool& msg) {
  gripper_open = msg.data;
  gripper_close = false;
}

std::array<double, 7> OperationalSpaceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_valve

PLUGINLIB_EXPORT_CLASS(franka_valve::OperationalSpaceController,
                       controller_interface::ControllerBase)