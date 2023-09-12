// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>


#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <std_msgs/Bool.h>
#include "zmq.hpp"
#include <thread>


using namespace std;
using namespace Eigen;

namespace franka_valve {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::EffortJointInterface,franka_hw::FrankaModelInterface,franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  VectorXd _q, _qdot;
  // unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  // hardware_interface::PositionJointInterface* position_joint_interface_;
  // std::vector<hardware_interface::JointHandle> position_joint_handles_;


  unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  vector<hardware_interface::JointHandle> effort_handles_;
  unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
   static constexpr double kDeltaTauMax{1.0};
  
  array<double, 7> saturateTorqueRate(const array<double, 7>& tau_d_calculated,
                                      const array<double, 7>& tau_J_d);
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
  void ZMQ();
  void controller();

  void ZMQ_receive();
  zmq::context_t context{14};  // joint pos 7 + vel 7 = 14
zmq::context_t context2{7};  // torque
zmq::socket_t socket{context, zmq::socket_type::push};
zmq::socket_t socket2{context2, zmq::socket_type::pull};
std::array<float, 7> Buffer_Robot;  // mat3x3 * 4 = 36 + 1

std::array<float, 14> Buffer;  // mat3x3 * 4 = 36 + 1
zmq::message_t request;
zmq::message_t reply;
bool zmq_start;
int zmq_cnt;
int _cnt_recv;
MatrixXd _q_traj, _qdot_traj;
VectorXd _q_tmp, _qdot_tmp, _q_mppi, _qdot_mppi, _torque_des;
VectorXd _q_err, _qdot_err;
MatrixXd _mass;
VectorXd _coriolis;

VectorXd _old_q_mppi;
VectorXd _old_qdot_mppi;
VectorXd _q_delta;
VectorXd _qdot_delta;
  // Eigen::VectorXd _q(7), _qdot(7);
};

}  // namespace franka_example_controllers