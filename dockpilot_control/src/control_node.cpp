#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <mutex>
#include <atomic>
#include <memory>
#include <stdexcept>
#include "dockpilot_control/controller_interface.hpp"
#include "dockpilot_control/pid_controller.hpp"
#include "dockpilot_control/pure_pursuit_controller.hpp"
#include "dockpilot_control/mpc_controller.hpp"
#include "dockpilot_control/lqr_controller.hpp"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

namespace dockpilot_control
{

// Helper: Compute shortest angular distance
static double shortest_angular_distance(double from, double to) {
    double error = to - from;
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;
    return error;
}

// Helper: Wrap angle in radians to [-π, π]
inline double wrapAngleRad(double yaw_rad) {
    double current = std::fmod(yaw_rad + M_PI, 2 * M_PI);
    if (current < 0.0) current += 2 * M_PI;
    return current - M_PI;
}

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
        : Node("control_node"), active_(false)
    {
        declare_parameter<std::string>("controller_type", "pid");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<double>("marker_timeout_sec", 1.0);

        base_frame_ = get_parameter("base_frame").as_string();
        marker_timeout_ = get_parameter("marker_timeout_sec").as_double();

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                goal_ = *msg;
            });

        marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/filtered_tag_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                marker_ = *msg;
            });

        active_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/docking_active", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                active_ = msg->data;
                if (!active_) {
                    geometry_msgs::msg::Twist stop;
                    cmd_pub_->publish(stop);
                }
            });

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),   // 20 Hz — matches docking server loop rate
            [this]() {
                init_controller_once();
                controlLoop();
            });

        RCLCPP_INFO(get_logger(), "ControlNode initialized - base_frame=%s", base_frame_.c_str());
    }

private:
    void init_controller_once()
    {
        if (controller_) return;
        const std::string type = get_parameter("controller_type").as_string();

        if (type == "pid") {
            controller_ = std::make_unique<PIDController>(shared_from_this());
            RCLCPP_INFO(get_logger(), "PID controller loaded");
        } else if (type == "pure_pursuit") {
            controller_ = std::make_unique<PurePursuitController>(shared_from_this());
            RCLCPP_INFO(get_logger(), "Pure-Pursuit controller loaded");
        } else if (type == "lqr") {
            controller_ = std::make_unique<LQRController>(shared_from_this());
            RCLCPP_INFO(get_logger(), "LQR controller loaded");
        } else if (type == "mpc") {
            controller_ = std::make_unique<MPCController>(shared_from_this());
            RCLCPP_INFO(get_logger(), "MPC controller loaded");
        } else {
            RCLCPP_FATAL(get_logger(), "Unknown controller_type: %s", type.c_str());
            throw std::runtime_error("Unknown controller_type");
        }
    }

    void controlLoop()
    {
        if (!active_ || !controller_) return;

        geometry_msgs::msg::PoseStamped goal, marker;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            goal = goal_;
            marker = marker_;
        }
        if (goal.header.frame_id.empty() || marker.header.frame_id.empty()) {
            RCLCPP_WARN(get_logger(), "Empty frame_id in goal or marker pose. Skipping control loop.");
            return;
        }

        // Check for stale marker pose
        rclcpp::Time now = this->now();
        if ((now - marker.header.stamp).seconds() > marker_timeout_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Marker pose is stale (age: %.2f sec > timeout: %.2f sec). Stopping robot.",
                                 (now - marker.header.stamp).seconds(), marker_timeout_);
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            return;
        }

        // Transform marker to base_frame
        geometry_msgs::msg::PoseStamped marker_robot = marker;
        if (marker.header.frame_id != base_frame_) {
            try {
                if (tf_buffer_->canTransform(base_frame_, marker.header.frame_id, marker.header.stamp, tf2::durationFromSec(0.1))) {
                    marker_robot = tf_buffer_->transform(marker, base_frame_, tf2::durationFromSec(0.1));
                } else {
                    geometry_msgs::msg::PoseStamped marker_latest = marker;
                    marker_latest.header.stamp = rclcpp::Time(0); // latest transform time
                    marker_robot = tf_buffer_->transform(marker_latest, base_frame_, tf2::durationFromSec(0.1));
                    RCLCPP_WARN(get_logger(), "Falling back to latest transform for marker from %s to %s",
                                marker.header.frame_id.c_str(), base_frame_.c_str());
                }
            } catch (const tf2::TransformException &e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF2 Transform failed: %s", e.what());
                return;
            }
        }

        double ex = marker_robot.pose.position.x - goal.pose.position.x;
        double ey = marker_robot.pose.position.y - goal.pose.position.y;

        RCLCPP_DEBUG(this->get_logger(), "Marker position: x=%.3f, y=%.3f", marker_robot.pose.position.x, marker_robot.pose.position.y);
        RCLCPP_DEBUG(this->get_logger(), "Goal position: x=%.3f, y=%.3f", goal.pose.position.x, goal.pose.position.y);
        RCLCPP_DEBUG(this->get_logger(), "Error ex: %.3f, ey: %.3f", ex, ey);

        // Compute yaw angles and wrap them
        tf2::Quaternion q_goal, q_now;
        tf2::fromMsg(goal.pose.orientation, q_goal);
        tf2::fromMsg(marker_robot.pose.orientation, q_now);

        tf2::Matrix3x3 m_goal(q_goal), m_now(q_now);
        double roll_g, pitch_g, yaw_g;
        double roll_n, pitch_n, yaw_n;
        m_goal.getRPY(roll_g, pitch_g, yaw_g);
        m_now.getRPY(roll_n, pitch_n, yaw_n);

        // Check for invalid yaw values
        if (std::isnan(yaw_g) || std::isnan(yaw_n)) {
            RCLCPP_WARN(get_logger(), "Invalid yaw from quaternion. Stopping.");
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            return;
        }

        // Wrap yaw angles to [-π, π]
        yaw_g = wrapAngleRad(yaw_g);
        yaw_n = wrapAngleRad(yaw_n);

        RCLCPP_DEBUG(this->get_logger(), "Wrapped yaw_g: %.3f rad (%.3f deg), yaw_n: %.3f rad (%.3f deg)",
                    yaw_g, RAD2DEG(yaw_g), yaw_n, RAD2DEG(yaw_n));

        double yaw_error = shortest_angular_distance(yaw_n, yaw_g);
        RCLCPP_DEBUG(this->get_logger(), "Yaw error: %.3f rad (%.3f deg)", yaw_error, RAD2DEG(yaw_error));

        // Pack into Pose for Controller
        geometry_msgs::msg::Pose error;
        error.position.x = ex;   // motion in X: forward/backward
        error.position.y = ey;   // motion in Y: left/right
        error.position.z = 0.0;

        // Encode yaw error as small rotation around Z
        tf2::Quaternion q_err;
        q_err.setRPY(0, 0, yaw_error);
        error.orientation = tf2::toMsg(q_err);

        // Publish Command
        auto cmd = controller_->compute(error, 0.1);

        // Safety: Clamp Velocities
        const double max_linear = 0.15;
        const double max_angular = 0.3;
        cmd.linear.x = std::clamp(cmd.linear.x, -max_linear, max_linear);
        cmd.linear.y = std::clamp(cmd.linear.y, -max_linear, max_linear);
        cmd.angular.z = std::clamp(cmd.angular.z, -max_angular, max_angular);

        // Safety: Check for NaN
        if (!std::isfinite(cmd.linear.x) || !std::isfinite(cmd.linear.y) || !std::isfinite(cmd.angular.z)) {
            RCLCPP_ERROR(get_logger(), "NaN detected in cmd_vel! Stopping.");
            cmd = geometry_msgs::msg::Twist{};
        }

        cmd_pub_->publish(cmd);
    }

    std::unique_ptr<ControllerInterface> controller_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PoseStamped goal_, marker_;
    std::mutex mtx_;
    std::atomic<bool> active_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool smooth_undocking_ = false;  // reserved for future smooth undock motion
    std::string base_frame_;
    double marker_timeout_;
};

} // namespace dockpilot_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dockpilot_control::ControlNode>());
    rclcpp::shutdown();
    return 0;
}