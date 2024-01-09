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

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/franka_gripper.h>

#include <std_msgs/Bool.h>

#include <franka_valve/trajectory.h>
// #include <torch/torch.h>

using namespace std;
using namespace Eigen;

namespace franka_valve {

class VisionController : public controller_interface::MultiInterfaceController<
                                      franka_hw::FrankaModelInterface,
                                      hardware_interface::EffortJointInterface,
                                      franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  void gripperCloseCallback(const std_msgs::Bool& msg);
  void gripperOpenCallback(const std_msgs::Bool& msg);
  void gripperStopCallback(const std_msgs::Bool& msg);

 private:
  struct Target {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    int gripper;
    double gripper_width;
    double time;
    string state;
    array<double, 7> q_goal;
  };
  struct Robot {
    Vector3d pos;
    double zrot;      // frame rotation according to Z-axis
    double ee_align;  // gripper and body align angle
  };

  struct Objects {
    const char* name;
    Vector3d o_margin;  // (x,y,z)residual from frame origin to rotation plane
    Vector3d r_margin;  // (x,y,z)radius of the object (where we first grab)
    Vector3d grab_dir;  // grabbing direction. r_margin vs o_margin x r_margin
    Vector3d pos;
    Vector3d grab_vector;
    Vector3d normal_vector;
    double radius;
  };

  void InitMotionPlan();
  void MotionPlan(Target target);
  void UpdateStates();

  array<double, 7> OperationalSpaceControl();
  array<double, 7> saturateTorqueRate(const array<double, 7>& tau_d_calculated,
                                      const array<double, 7>& tau_J_d);
  array<double, 7> JointControl();
  void writeToCSVfile(string filename, MatrixXd input_matrix, int cnt) 
  Target TargetTransformMatrix(Objects obj, Robot robot, double angle);
  Matrix3d R3D(Objects obj, Vector3d unitVec, double angle);
  void writeToCSVfile(string filename, MatrixXd qd, MatrixXd q, MatrixXd xd, MatrixXd x);

  static constexpr double kDeltaTauMax{1.0};

  unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  vector<hardware_interface::JointHandle> effort_handles_;

  ros::Duration _elapsed_time_;

  // std::vector<double> k_gains_;
  // std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  VectorXd _k_gains, _d_gains;
  VectorXd _xk_gains, _xd_gains;

  // franka_gripper::GraspGoal close_goal;

  franka_gripper::GraspGoal close_goal;
  
  franka_gripper::MoveGoal open_goal;
  franka_gripper::StopGoal stop_goal;
  franka_gripper::GraspEpsilon epsilon;

  bool gripper_close;
  bool gripper_open;
  bool gripper_stop;
  bool gripper_home;

  ros::Subscriber gripper_close_sub_;
  ros::Subscriber gripper_open_sub_;
  ros::Subscriber gripper_stop_sub;
  ros::Subscriber gripper_home_sub;
  

  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_ac_close{
      "/franka_gripper/grasp", true};
  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_open{"/franka_gripper/move",
                                                                            true};

  actionlib::SimpleActionClient<franka_gripper::StopAction> gripper_ac_stop{"/franka_gripper/stop",
                                                                            true};
                                                                          
  CTrajectory JointTrajectory;  // joint space trajectory
  HTrajectory HandTrajectory;   // task space trajectory
  RTrajectory CircularTrajectory;
  vector<Target> _target_plan;
  array<bool, 30> _bool_plan;
  int _cnt_plan;

  double _gripper_open, _gripper_close;

  array<double, 7> _torque_des;
  MatrixXd _J_hands, _mass, _coriolis;
  VectorXd _q, _qdot, _q_des, _qdot_des;
  VectorXd _x_hand, _xdot_hand, _x_des_hand, _xdot_des_hand;
  Matrix3d _R_des_hand, _R_hand, _Rdot_des_hand, _Rdot_hand;
  double _init_theta, _goal_theta, _theta_des;
  Objects _obj;
  Robot _robot;
  MatrixXd _Tvr;
  ros::Time _operation_time;
  float _printtime, _timeinterval, _t, _dt;

  bool _motion_done;
  MatrixXd _q_plot, _qdes_plot, _x_plot, _xdes_plot;
  int _cnt_plot;
  int _cnt;
  double _accum_time;
};

}  // namespace franka_valve
