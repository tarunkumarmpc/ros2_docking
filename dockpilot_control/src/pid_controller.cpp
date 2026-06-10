#include "dockpilot_control/pid_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace dockpilot_control {

// Helper functions
inline double PIDController::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

inline double PIDController::slew(double desired, double prev, double max_delta) {
    double delta = desired - prev;
    if (delta > max_delta)
        return prev + max_delta;
    if (delta < -max_delta)
        return prev - max_delta;
    return desired;
}

inline double PIDController::wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline void PIDController::applyLowpassFilter(double &current, double &filtered)
{
    filtered = lowpass_alpha_ * current + (1.0 - lowpass_alpha_) * filtered;
    current = filtered;
}

// Smooth interpolation for gain scheduling
inline double PIDController::interpolateGain(double error_abs, double small_thr, double large_thr, double gain_small, double gain_large)
{
    if (error_abs <= small_thr) return gain_small;
    else if (error_abs >= large_thr) return gain_large;
    else {
        double ratio = (error_abs - small_thr) / (large_thr - small_thr);
        return gain_small + ratio * (gain_large - gain_small);
    }
}

PIDController::PIDController(const rclcpp::Node::SharedPtr &node)
{
    // Base PID gains
    kp_x_ = node->declare_parameter("pid.kp_x", 1.0);
    ki_x_ = node->declare_parameter("pid.ki_x", 0.0);
    kd_x_ = node->declare_parameter("pid.kd_x", 0.5);

    kp_y_ = node->declare_parameter("pid.kp_y", 1.0);
    ki_y_ = node->declare_parameter("pid.ki_y", 0.0);
    kd_y_ = node->declare_parameter("pid.kd_y", 0.5);

    kp_yaw_ = node->declare_parameter("pid.kp_yaw", 2.0);
    ki_yaw_ = node->declare_parameter("pid.ki_yaw", 0.0);
    kd_yaw_ = node->declare_parameter("pid.kd_yaw", 0.5);

    // Gain scheduling parameters
    large_error_thr_ = node->declare_parameter("pid.large_error_thr", 0.18);
    small_error_thr_ = node->declare_parameter("pid.small_error_thr", 0.07);
    kp_large_mult_ = node->declare_parameter("pid.kp_large_mult", 1.4);
    kd_large_mult_ = node->declare_parameter("pid.kd_large_mult", 1.9);
    ki_large_mult_ = node->declare_parameter("pid.ki_large_mult", 0.4);
    kp_small_mult_ = node->declare_parameter("pid.kp_small_mult", 0.6);
    kd_small_mult_ = node->declare_parameter("pid.kd_small_mult", 0.5);
    ki_small_mult_ = node->declare_parameter("pid.ki_small_mult", 0.7);

    // Robot geometry params
    wheel_radius_ = node->declare_parameter("lqr.wheel_radius", 0.05);
    lx_ = node->declare_parameter("lqr.lx", 0.305);
    ly_ = node->declare_parameter("lqr.ly", 0.161);

    max_linear_x_ = node->declare_parameter("pid.max_linear_x", 0.1);
    max_linear_y_ = node->declare_parameter("pid.max_linear_y", 0.1);
    max_angular_ = node->declare_parameter("pid.max_angular", 0.07);

    max_delta_v_ = node->declare_parameter("pid.max_delta_v", 0.05);
    max_delta_omega_ = node->declare_parameter("pid.max_delta_omega", 0.07);

    lowpass_alpha_ = node->declare_parameter("pid.lowpass_alpha", 0.13);

    // Zero minimum velocity deadzones for no stuck behavior
    min_linear_velocity_ = 0.0;
    min_angular_velocity_ = 0.0;

    ix_ = iy_ = iyaw_ = 0.0;
    prev_x_ = prev_y_ = prev_yaw_ = 0.0;
    prev_cmd_x_ = prev_cmd_y_ = prev_cmd_yaw_ = 0.0;
    filtered_x_ = 0.0;
    filtered_y_ = 0.0;
    filtered_yaw_ = 0.0;
}

geometry_msgs::msg::Twist PIDController::compute(const geometry_msgs::msg::Pose &error, double dt)
{
    geometry_msgs::msg::Twist cmd;

    double ex = error.position.x;
    double ey = error.position.y;

    tf2::Quaternion q(
        error.orientation.x,
        error.orientation.y,
        error.orientation.z,
        error.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double eyaw = -1 * wrapAngle(yaw);

    applyLowpassFilter(ex, filtered_x_);
    applyLowpassFilter(ey, filtered_y_);
    applyLowpassFilter(eyaw, filtered_yaw_);

    // Smoothly interpolate gains based on error magnitude
    double kp_x = interpolateGain(std::abs(ex), small_error_thr_, large_error_thr_, kp_x_ * kp_small_mult_, kp_x_ * kp_large_mult_);
    double kd_x = interpolateGain(std::abs(ex), small_error_thr_, large_error_thr_, kd_x_ * kd_small_mult_, kd_x_ * kd_large_mult_);
    double ki_x = interpolateGain(std::abs(ex), small_error_thr_, large_error_thr_, ki_x_ * ki_small_mult_, ki_x_ * ki_large_mult_);

    double kp_y = interpolateGain(std::abs(ey), small_error_thr_, large_error_thr_, kp_y_ * kp_small_mult_, kp_y_ * kp_large_mult_);
    double kd_y = interpolateGain(std::abs(ey), small_error_thr_, large_error_thr_, kd_y_ * kd_small_mult_, kd_y_ * kd_large_mult_);
    double ki_y = interpolateGain(std::abs(ey), small_error_thr_, large_error_thr_, ki_y_ * ki_small_mult_, ki_y_ * ki_large_mult_);

    double kp_yaw = interpolateGain(std::abs(eyaw), small_error_thr_, large_error_thr_, kp_yaw_ * kp_small_mult_, kp_yaw_ * kp_large_mult_);
    double kd_yaw = interpolateGain(std::abs(eyaw), small_error_thr_, large_error_thr_, kd_yaw_ * kd_small_mult_, kd_yaw_ * kd_large_mult_);
    double ki_yaw = interpolateGain(std::abs(eyaw), small_error_thr_, large_error_thr_, ki_yaw_ * ki_small_mult_, ki_yaw_ * ki_large_mult_);

    constexpr double max_integral = 0.35;

    ix_ = clamp(ix_ + ex * dt, -max_integral, max_integral);
    iy_ = clamp(iy_ + ey * dt, -max_integral, max_integral);
    iyaw_ = clamp(iyaw_ + eyaw * dt, -max_integral, max_integral);

    double dx = (dt > 1e-6) ? (ex - prev_x_) / dt : 0.0;
    double dy = (dt > 1e-6) ? (ey - prev_y_) / dt : 0.0;
    double dyaw_raw = eyaw - prev_yaw_;
    double dyaw = (dt > 1e-6) ? wrapAngle(dyaw_raw) / dt : 0.0;

    // PID computed velocities before kinematics
    double vx = kp_x * ex + ki_x * ix_ + kd_x * dx;
    double vy = kp_y * ey + ki_y * iy_ + kd_y * dy;
    double omega = kp_yaw * eyaw + ki_yaw * iyaw_ + kd_yaw * dyaw;

    vx = clamp(vx, -max_linear_x_, max_linear_x_);
    vy = clamp(vy, -max_linear_y_, max_linear_y_);
    omega = clamp(omega, -max_angular_, max_angular_);

    // Inverse kinematics for mecanum wheels
    double R = wheel_radius_;
    double Lx = lx_;
    double Ly = ly_;
    double wheel_FL = (1 / R) * (vx - vy - (Lx + Ly) * omega);
    double wheel_FR = (1 / R) * (vx + vy + (Lx + Ly) * omega);
    double wheel_RL = (1 / R) * (vx + vy - (Lx + Ly) * omega);
    double wheel_RR = (1 / R) * (vx - vy + (Lx + Ly) * omega);

    // You can publish or command these wheel speeds to the hardware here

    // Output robot frame velocities (for compatibility)
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = omega;

    // No minimum velocity threshold zeroing here to prevent sticking

    // Slew rate limiting to smooth commands
    cmd.linear.x = slew(cmd.linear.x, prev_cmd_x_, max_delta_v_);
    cmd.linear.y = slew(cmd.linear.y, prev_cmd_y_, max_delta_v_);
    cmd.angular.z = slew(cmd.angular.z, prev_cmd_yaw_, max_delta_omega_);

    if (!std::isfinite(cmd.linear.x) || !std::isfinite(cmd.linear.y) || !std::isfinite(cmd.angular.z)) {
        cmd = geometry_msgs::msg::Twist();
    }

    prev_x_ = ex;
    prev_y_ = ey;
    prev_yaw_ = eyaw;

    prev_cmd_x_ = cmd.linear.x;
    prev_cmd_y_ = cmd.linear.y;
    prev_cmd_yaw_ = cmd.angular.z;

    // Debug print
    std::cout << "Errors: Ex=" << ex << ", Ey=" << ey << ", Eyaw=" << eyaw << std::endl;
    std::cout << "PID Gains: KP_x=" << kp_x << ", KI_x=" << ki_x << ", KD_x=" << kd_x << std::endl;
    std::cout << "PID Gains: KP_y=" << kp_y << ", KI_y=" << ki_y << ", KD_y=" << kd_y << std::endl;
    std::cout << "PID Gains: KP_yaw=" << kp_yaw << ", KI_yaw=" << ki_yaw << ", KD_yaw=" << kd_yaw << std::endl;
    std::cout << "Cmd velocities: vx=" << cmd.linear.x << ", vy=" << cmd.linear.y << ", omega=" << cmd.angular.z << std::endl;
    std::cout << "Wheel Speeds (rad/s): FL=" << wheel_FL << ", FR=" << wheel_FR << ", RL=" << wheel_RL << ", RR=" << wheel_RR << std::endl;

    return cmd;
}

}  // namespace dockpilot_control
