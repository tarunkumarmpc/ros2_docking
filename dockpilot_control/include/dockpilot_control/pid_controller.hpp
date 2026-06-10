// pid_controller.hpp
#ifndef MOTION_CONTROL__PID_CONTROLLER_HPP_
#define MOTION_CONTROL__PID_CONTROLLER_HPP_

#include "dockpilot_control/controller_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace dockpilot_control
{

class PIDController : public ControllerInterface
{
public:
  explicit PIDController(const rclcpp::Node::SharedPtr &node);
  geometry_msgs::msg::Twist compute(const geometry_msgs::msg::Pose &error, double dt) override;

private:
    // PID gains
    double kp_x_, ki_x_, kd_x_;
    double kp_y_, ki_y_, kd_y_;
    double kp_yaw_, ki_yaw_, kd_yaw_;

    // Integrals
    double ix_, iy_, iyaw_;

    // Previous errors
    double prev_x_, prev_y_, prev_yaw_;

    // Previous commands (for slew)
    double prev_cmd_x_, prev_cmd_y_, prev_cmd_yaw_;

    // Velocity limits
    double max_linear_x_;
    double max_linear_y_;
    double max_angular_;
    double final_dist_;
    double final_linear_x_;
    double final_linear_y_;
    double final_angular_;

    // Slew rates
    double max_delta_v_;
    double max_delta_omega_;

    //filter 
  double filtered_x_{0.0};
  double filtered_y_{0.0};
  double filtered_yaw_{0.0};
  double lowpass_alpha_;

  // Adaptive gain scheduling members
double large_error_thr_;
double small_error_thr_;

double kp_large_mult_;
double kd_large_mult_;
double ki_large_mult_;

double kp_small_mult_;
double kd_small_mult_;
double ki_small_mult_;

double wheel_radius_;
double lx_;
double ly_;


  void applyLowpassFilter(double &current, double &filtered);
  inline double clamp(double value, double min_val, double max_val);
  inline double slew(double desired, double prev, double max_delta);
   double wrapAngle(double angle) ;

    double interpolateGain(double error_abs, double small_thr, double large_thr, double gain_small, double gain_large);

       // Deadband and minimum velocity thresholds
    double min_linear_velocity_;
    double min_angular_velocity_;
    double deadband_distance_;
    double deadband_angle_;
    bool use_filter_;
    
};

}  // namespace dockpilot_control

#endif  // MOTION_CONTROL__PID_CONTROLLER_HPP_