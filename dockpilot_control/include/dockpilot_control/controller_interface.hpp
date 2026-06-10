#ifndef MOTION_CONTROL__CONTROLLER_INTERFACE_HPP_
#define MOTION_CONTROL__CONTROLLER_INTERFACE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace dockpilot_control
{
class ControllerInterface
{
public:
  virtual ~ControllerInterface() = default;
  virtual geometry_msgs::msg::Twist compute(const geometry_msgs::msg::Pose &error,
                                            double dt) = 0;
};
}  // namespace dockpilot_control

#endif  // MOTION_CONTROL__CONTROLLER_INTERFACE_HPP_