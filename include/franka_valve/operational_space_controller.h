// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <Eigen/Dense>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

#include <std_msgs/Bool.h>

#include <franka_valve/trajectory.h>


namespace franka_valve {

class OperationalSpaceController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  void gripperCloseCallback(const std_msgs::Bool& msg);
  void gripperOpenCallback(const std_msgs::Bool& msg);
  

 private:
// Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);
  static constexpr double kDeltaTauMax{1.0};

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Duration elapsed_time_;

  double k_gains_{400.0};
  double d_gains_{40.0};
  double coriolis_factor_{1.0};

  
  franka_gripper::GraspGoal close_goal;
  franka_gripper::MoveGoal open_goal;
  franka_gripper::GraspEpsilon epsilon;


  bool gripper_close;
  bool gripper_open;

  ros::Subscriber gripper_close_sub_;
  ros::Subscriber gripper_open_sub_;

  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_ac_close
  {"/franka_gripper/grasp", true};
  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_open
  {"/franka_gripper/move", true};

  HTrajectory HandTrajectory;

  // Eigen::VectorXd _q, _qdot, _q_des, _qdot_des, _q_init; 
  // Eigen::VectorXd _q_goal, _qdot_goal;
  // Eigen::VectorXd _x_des_hand, _xdot_des_hand;
  // Eigen::VectorXd _x_goal_hand, _xdot_goal_hand;

  // Eigen::MatrixXd _J_hands; // jacobian matrix
  // Eigen::MatrixXd _J_bar_hands; // pseudo invere jacobian matrix
  // Eigen::MatrixXd _lambda; // pseudo invere jacobian matrix

  // Eigen::VectorXd _x_hand, _xdot_hand; // End-effector

  // Eigen::VectorXd _x_err_hand;
  // Eigen::Matrix3d _R_des_hand;
  float printtime, timeinterval;



};

}  // namespace franka_valve
