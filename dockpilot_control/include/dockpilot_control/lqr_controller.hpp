#ifndef MOTION_CONTROL__LQR_CONTROLLER_HPP_
#define MOTION_CONTROL__LQR_CONTROLLER_HPP_

#include "dockpilot_control/controller_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>

namespace dockpilot_control
{

class LQRController : public ControllerInterface
{
public:
  explicit LQRController(const rclcpp::Node::SharedPtr &node);
  geometry_msgs::msg::Twist compute(const geometry_msgs::msg::Pose &error, double dt) override;

private:
  // Helper functions
  static double clamp(double value, double min_val, double max_val);
  static double slew(double desired, double prev, double max_delta);
  static double wrapAngle(double angle);

  // Discrete Algebraic Riccati Equation (DARE) solver
  static Eigen::MatrixXd solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                   const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                                   int max_iters = 100, double eps = 1e-9);

  // LQR gain matrix
  Eigen::Matrix3d K_;
      double filter_alpha_;

  // System matrices and weighting matrices
  Eigen::Matrix3d A_, B_, Q_, R_;

  // Control time step
  double dt_;

  // Velocity command limits
  double max_linear_x_, max_linear_y_, max_angular_;

  // Slew rate limits
  double max_delta_v_, max_delta_omega_;

  // Previous commands (for slew limiting)
  double prev_cmd_x_, prev_cmd_y_, prev_cmd_yaw_;

    double wheel_radius_;
  double lx_;
  double ly_;
  double wheel_max_speed_;

  // Declare the mecanum kinematics functions:
  Eigen::Vector4d inverseKinematics(double vx, double vy, double wz);
  Eigen::Vector3d forwardKinematics(const Eigen::Vector4d &wheel_speeds);
};

}  // namespace dockpilot_control

#endif  // MOTION_CONTROL__LQR_CONTROLLER_HPP_
