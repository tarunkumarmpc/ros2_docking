#include "dockpilot_control/lqr_controller.hpp"
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>

namespace dockpilot_control
{

double LQRController::clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

double LQRController::slew(double desired, double prev, double max_delta)
{
  double delta = desired - prev;
  if (delta > max_delta) return prev + max_delta;
  if (delta < -max_delta) return prev - max_delta;
  return desired;
}

double LQRController::wrapAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

Eigen::MatrixXd LQRController::solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                         const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                                         int max_iters, double eps)
{
  Eigen::MatrixXd X = Q;
  Eigen::MatrixXd X_prev;
  for (int i = 0; i < max_iters; ++i)
  {
    X_prev = X;
    Eigen::MatrixXd K = (R + B.transpose() * X * B).ldlt().solve(B.transpose() * X * A);
    X = Q + A.transpose() * X * A - A.transpose() * X * B * K;
    if ((X - X_prev).norm() < eps) break;
  }
  return X;
}

// ---------- Constructor ----------
LQRController::LQRController(const rclcpp::Node::SharedPtr &node)
  : prev_cmd_x_(0.0),
    prev_cmd_y_(0.0),
    prev_cmd_yaw_(0.0)
{
  // LQR Weights (original values preserved)
  Eigen::Vector3d q_diag(
    node->declare_parameter("lqr.q_x", 3.0),
    node->declare_parameter("lqr.q_y", 3.0),
    node->declare_parameter("lqr.q_yaw", 3.0));
  Q_ = q_diag.asDiagonal();

  Eigen::Vector3d r_diag(
    node->declare_parameter("lqr.r_vx", 0.5),
    node->declare_parameter("lqr.r_vy", 0.5),
    node->declare_parameter("lqr.r_w", 1.0));
  R_ = r_diag.asDiagonal();

  dt_ = node->declare_parameter("lqr.dt", 0.1);

  // Robot Physical Dimensions (configurable via ROS parameters)
  wheel_radius_ = node->declare_parameter("lqr.wheel_radius", 0.05); // [m]
  lx_ = node->declare_parameter("lqr.lx", 0.305);                   // [m]
  ly_ = node->declare_parameter("lqr.ly", 0.161);                   // [m]
  wheel_max_speed_ = node->declare_parameter("lqr.wheel_max_speed", 2.0); // [rad/s] Caps linear velocity at 0.1 m/s

  // Velocity and slew rate limits (reduced for smooth transitions)
  max_delta_v_ = node->declare_parameter("lqr.max_delta_v", 0.01);     // [m/s]
  max_delta_omega_ = node->declare_parameter("lqr.max_delta_omega", 0.010); // [rad/s]

  // State-space model for integrator dynamics
  A_ = Eigen::Matrix3d::Identity();
  B_ = Eigen::Matrix3d::Identity() * dt_;

  // Solve Riccati equation and compute gain K
  Eigen::MatrixXd X = solveDARE(A_, B_, Q_, R_);
  K_ = (R_ + B_.transpose() * X * B_).ldlt().solve(B_.transpose() * X * A_);
}

// ---------- Mecanum Kinematics ----------
Eigen::Vector4d LQRController::inverseKinematics(double vx, double vy, double wz)
{
  double L = lx_ + ly_;
  Eigen::Matrix<double,4,3> invJ;
  invJ <<
    1, -1, -L,
    1,  1,  L,
    1,  1, -L,
    1, -1,  L;
  
  return (1.0 / wheel_radius_) * invJ * Eigen::Vector3d(vx, vy, wz);
}

Eigen::Vector3d LQRController::forwardKinematics(const Eigen::Vector4d &wheel_speeds)
{
  double L = lx_ + ly_;
  Eigen::Matrix<double,3,4> J;
  J <<
    1, 1, 1, 1,
   -1, 1, 1,-1,
   -1/L, 1/L, -1/L, 1/L;
  
  return (wheel_radius_ / 4.0) * (J * wheel_speeds);
}

// ---------- Compute Command ----------
geometry_msgs::msg::Twist LQRController::compute(const geometry_msgs::msg::Pose &error_pose, double /*dt*/)
{
  geometry_msgs::msg::Twist cmd;

  // Extract error
  double ex = -error_pose.position.x;
  double ey = -error_pose.position.y;

  tf2::Quaternion q(
    error_pose.orientation.x,
    error_pose.orientation.y,
    error_pose.orientation.z,
    error_pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double eyaw = wrapAngle(yaw);

  // State vector
  Eigen::Vector3d x(ex, ey, eyaw);

  // LQR feedback law
  Eigen::Vector3d u = -K_ * x;

  // Slew rate control on chassis velocities
  u(0) = slew(u(0), prev_cmd_x_, max_delta_v_);
  u(1) = slew(u(1), prev_cmd_y_, max_delta_v_);
  u(2) = slew(u(2), prev_cmd_yaw_, max_delta_omega_);

  // Convert to wheel speeds
  Eigen::Vector4d wheel_speeds = inverseKinematics(u(0), u(1), u(2));

  // Clamp each wheel speed
  for (int i = 0; i < 4; ++i)
    wheel_speeds(i) = clamp(wheel_speeds(i), -wheel_max_speed_, wheel_max_speed_);

  // Back to achievable chassis velocity
  Eigen::Vector3d achievable_vel = forwardKinematics(wheel_speeds);

  // Assign output with explicit velocity capping
  cmd.linear.x = clamp(achievable_vel(0), -0.1, 0.1);   // Cap at 0.1 m/s
  cmd.linear.y = clamp(achievable_vel(1), -0.1, 0.1);   // Cap at 0.1 m/s
  cmd.angular.z = clamp(achievable_vel(2), -0.07, 0.07); // Cap at 0.07 rad/s

  // NaN check
  if (!std::isfinite(cmd.linear.x) || !std::isfinite(cmd.linear.y) || !std::isfinite(cmd.angular.z))
  {
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
  }

  // Store for next time step
  prev_cmd_x_ = cmd.linear.x;
  prev_cmd_y_ = cmd.linear.y;
  prev_cmd_yaw_ = cmd.angular.z;

  return cmd;
}

} // namespace dockpilot_control