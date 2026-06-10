// include/dockpilot_control/mpc_controller.hpp
#ifndef MOTION_CONTROL__MPC_CONTROLLER_HPP_
#define MOTION_CONTROL__MPC_CONTROLLER_HPP_

#include "dockpilot_control/controller_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>

namespace dockpilot_control
{

class MPCController : public ControllerInterface
{
public:
  explicit MPCController(const rclcpp::Node::SharedPtr &node);
  geometry_msgs::msg::Twist compute(const geometry_msgs::msg::Pose &error, double dt) override;

private:
  // Horizon
  int N_;

  // Time step
  double dt_;

  // Input limits
  Eigen::Vector3d u_max_, u_min_;
  Eigen::Vector3d du_max_, du_min_;

  // Cost matrices
  Eigen::Matrix3d Q_, R_, QN_;

  // State and input
  Eigen::Vector3d u_prev_;

  // Dynamics matrices
  Eigen::Matrix3d A_, B_;
};

}  // namespace dockpilot_control

#endif  // MOTION_CONTROL__MPC_CONTROLLER_HPP_