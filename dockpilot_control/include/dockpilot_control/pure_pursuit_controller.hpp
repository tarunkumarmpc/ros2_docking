#ifndef MOTION_CONTROL__PURE_PURSUIT_CONTROLLER_HPP_
#define MOTION_CONTROL__PURE_PURSUIT_CONTROLLER_HPP_

#include "dockpilot_control/controller_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>

namespace dockpilot_control
{

class PurePursuitController : public ControllerInterface
{
public:
  explicit PurePursuitController(const rclcpp::Node::SharedPtr& node);
  geometry_msgs::msg::Twist compute(const geometry_msgs::msg::Pose& error,
                                    double dt) override;

private:
  inline double clamp(double v, double lo, double hi) const
  { return std::max(lo, std::min(v, hi)); }

  inline double slew(double desired, double prev, double max_delta) const
  {
    double delta = desired - prev;
    return (delta > max_delta)  ? prev + max_delta :
           (delta < -max_delta) ? prev - max_delta : desired;
  }
    static double wrapAngle(double angle);


  /* ---------- parameters ---------- */
  double wheel_radius_, lx_, ly_;
  double max_wheel_speed_;
  double max_linear_, max_angular_;
  double max_dv_, max_dw_;
  double k_lookahead_;   // curvature gain for yaw
  double prev_vx_{0}, prev_vy_{0}, prev_w_{0};
};

}  // namespace dockpilot_control
#endif  // MOTION_CONTROL__PURE_PURSUIT_CONTROLLER_HPP_