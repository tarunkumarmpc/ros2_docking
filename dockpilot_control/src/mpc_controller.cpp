// src/mpc_controller.cpp
#include "dockpilot_control/mpc_controller.hpp"
#include <cmath>
#include <algorithm>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/Cholesky>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
namespace dockpilot_control
{

// Helper: clamp value
inline double clamp(double value, double min_val, double max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

// Helper: wrap angle to [-π, π]
inline double wrap_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

MPCController::MPCController(const rclcpp::Node::SharedPtr &node)
  : u_prev_(Eigen::Vector3d::Zero())
{
  N_  = node->declare_parameter("mpc.N", 8);
  dt_ = node->declare_parameter("mpc.dt", 0.1);

  // Velocity limits (tuned for Mecanum)
  u_max_ << node->declare_parameter("mpc.max_vx", 0.3),
            node->declare_parameter("mpc.max_vy", 0.3),
            node->declare_parameter("mpc.max_w",  0.5);
  u_min_ = -u_max_;

  // Acceleration limits (smooth motion)
  du_max_ << node->declare_parameter("mpc.max_acc_x", 0.05),
              node->declare_parameter("mpc.max_acc_y", 0.05),
              node->declare_parameter("mpc.max_acc_w", 0.1);
  du_min_ = -du_max_;

  // Cost matrices
  Eigen::Vector3d q_diag;
  q_diag << node->declare_parameter("mpc.q_x", 10.0),
            node->declare_parameter("mpc.q_y", 10.0),
            node->declare_parameter("mpc.q_yaw", 20.0);
  Q_ = q_diag.asDiagonal();

  Eigen::Vector3d r_diag;
  r_diag << node->declare_parameter("mpc.r_vx", 0.1),
            node->declare_parameter("mpc.r_vy", 0.1),
            node->declare_parameter("mpc.r_w", 0.1);
  R_ = r_diag.asDiagonal();
  QN_ = Q_;  // Terminal cost

  // Initialize
  u_prev_.setZero();
}

geometry_msgs::msg::Twist MPCController::compute(const geometry_msgs::msg::Pose &error, double /*dt*/)
{
  // Extract positional error
  double ex = error.position.x;
  double ey = error.position.y;

  // Yaw error from quaternion (proper, instead of approx)
  tf2::Quaternion q(
      error.orientation.x,
      error.orientation.y,
      error.orientation.z,
      error.orientation.w);
  double eyaw = tf2::getYaw(q);
  eyaw = wrap_angle(eyaw);

  // State error vector
  Eigen::Vector3d x0(ex, ey, eyaw);

  // Discrete dynamics: x(k+1) = A x(k) + B u(k)
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d B = Eigen::Matrix3d::Identity() * dt_;

  int n = 3;   // state dim
  int m = 3;   // control dim
  int N = N_;  // horizon

  // Build stacked prediction matrices
  Eigen::MatrixXd Sx(n*N, n);
  Eigen::MatrixXd Su(n*N, m*N);
  Sx.setZero(); Su.setZero();

  Eigen::Matrix3d Ak = A;
  for (int i = 0; i < N; ++i) {
    // Sx: contribution of x0
    Sx.block(i*n, 0, n, n) = Ak;
    // Su: contribution of controls
    for (int j = 0; j <= i; ++j) {
      Eigen::Matrix3d Aij = Eigen::Matrix3d::Identity();
      for (int k = 0; k < i-j; k++)
        Aij *= A;
      Su.block(i*n, j*m, n, m) = Aij * B;
    }
    Ak *= A;
  }

  // Block diagonal Q and R for horizon cost
  Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(n*N, n*N);
  Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(m*N, m*N);
  for (int i = 0; i < N; i++) {
    Qbar.block(i*n, i*n, n, n) = Q_;
    Rbar.block(i*m, i*m, m, m) = R_;
  }
  // Terminal cost
  Qbar.block((N-1)*n, (N-1)*n, n, n) = QN_;

  // Quadratic cost: J = 0.5 U^T H U + g^T U
  Eigen::MatrixXd H = Su.transpose() * Qbar * Su + Rbar;
  Eigen::VectorXd g = Su.transpose() * Qbar * Sx * x0;

  // Solve unconstrained MPC: U* = -H⁻¹ g
  Eigen::VectorXd U_opt = -H.ldlt().solve(g);

  // First control action
  Eigen::Vector3d u = U_opt.head<3>();

  // Apply rate limits
  Eigen::Vector3d du = u - u_prev_;
  du = du.cwiseMin(du_max_).cwiseMax(du_min_);
  u = u_prev_ + du;
  u_prev_ = u;

  // Clamp within bounds
  u = u.cwiseMin(u_max_).cwiseMax(u_min_);

  // Debug print
  std::cout << "MPC debug: ex=" << ex << ", ey=" << ey 
            << ", eyaw=" << eyaw << ", u=[" 
            << u(0) << ", " << u(1) << ", " << u(2) << "]" << std::endl;

  // ROS Twist output
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = u(0);
  cmd.linear.y  = u(1);  // Mecanum allows lateral motion
  cmd.angular.z = u(2);
  return cmd;
}

}  // namespace dockpilot_control
