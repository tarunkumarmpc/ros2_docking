#include "dockpilot_control/pure_pursuit_controller.hpp"
#include <cmath>

namespace dockpilot_control
{

PurePursuitController::PurePursuitController(const rclcpp::Node::SharedPtr& node)
{
  wheel_radius_ = node->declare_parameter("wheel_radius", 0.05);
  lx_           = node->declare_parameter("lx", 0.305);
  ly_           = node->declare_parameter("ly", 0.161);

  max_wheel_speed_ = node->declare_parameter("max_wheel_speed", 2.0);
  max_linear_      = node->declare_parameter("max_linear", 0.1);
  max_angular_     = node->declare_parameter("max_angular", 0.5);
  max_dv_          = node->declare_parameter("max_dv", 0.02);
  max_dw_          = node->declare_parameter("max_dw", 0.05);
  k_lookahead_     = node->declare_parameter("pp.curvature_gain", 2.0);
}

double PurePursuitController::wrapAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}


geometry_msgs::msg::Twist
PurePursuitController::compute(const geometry_msgs::msg::Pose& error,
                               double /*dt*/)
{
  /* 1. translational error */
  double ex = -error.position.x;
  double ey = -error.position.y;
  double dist = std::sqrt(ex * ex + ey * ey);

  /* 2. desired linear speed (proportional to distance) */
  double v = clamp(dist, 0.0, max_linear_);

  /* 3. direction */
  double vx = v * (ex / (dist + 1e-6));
  double vy = v * (ey / (dist + 1e-6));

  tf2::Quaternion q(
    error.orientation.x,
    error.orientation.y,
    error.orientation.z,
    error.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double eyaw = wrapAngle(yaw);
  double omega = clamp(k_lookahead_ * eyaw, -max_angular_, max_angular_);

  /* 5. slew limits */
  vx  = slew(vx, prev_vx_, max_dv_);
  vy  = slew(vy, prev_vy_, max_dv_);
  omega = slew(omega, prev_w_, max_dw_);

  /* 6. mecanum wheel saturation */
  const double L = lx_ + ly_;
  double w1 = (vx - vy - omega * L) / wheel_radius_;
  double w2 = (vx + vy + omega * L) / wheel_radius_;
  double w3 = (vx + vy - omega * L) / wheel_radius_;
  double w4 = (vx - vy + omega * L) / wheel_radius_;
  double w_max = std::max({std::fabs(w1), std::fabs(w2),
                           std::fabs(w3), std::fabs(w4)});
  if (w_max > max_wheel_speed_)
  {
    double scale = max_wheel_speed_ / w_max;
    vx *= scale; vy *= scale; omega *= scale;
  }

  prev_vx_ = vx;
  prev_vy_ = vy;
  prev_w_  = omega;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = vx;
  cmd.linear.y  = vy;
  cmd.angular.z = omega;
  return cmd;
}

}  // namespace dockpilot_control