#ifndef DOCKING_SERVER_HPP
#define DOCKING_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <unordered_map>
#include "dockpilot_interfaces/action/dock.hpp"
#include "dockpilot_interfaces/action/undock.hpp"

using Dock = dockpilot_interfaces::action::Dock;
using Undock = dockpilot_interfaces::action::Undock;
using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;
using GoalHandleUndock = rclcpp_action::ServerGoalHandle<Undock>;

struct DockConfig {
    geometry_msgs::msg::PoseStamped dock_pose;
    geometry_msgs::msg::PoseStamped undock_pose;
};

class DockPoseManager {
public:
    explicit DockPoseManager(rclcpp::Node &node,
                             tf2_ros::Buffer &tf_buffer,
                             const std::string &base_frame);

    void load(const std::string &file_path);
    bool getDockConfig(const std::string &dock_id, DockConfig &out) const;

private:
    void setDefault();
    geometry_msgs::msg::PoseStamped parsePose(const YAML::Node &n, const std::string &default_frame) const;
    geometry_msgs::msg::PoseStamped transformToBase(const geometry_msgs::msg::PoseStamped &in) const;

    rclcpp::Node &node_;
    tf2_ros::Buffer &tf_buffer_;
    const std::string base_frame_;
    std::unordered_map<std::string, DockConfig> configs_;
    DockConfig default_;
};

enum class ServerState { IDLE, DOCKING, UNDOCKING, ERROR };

struct DiagData {
    std::atomic<double> ex{0}, ey{0}, eyaw{0}, distance{0};
    std::atomic<double> current_y_tol{0}, current_yaw_tol{0};
    std::atomic<bool> marker_timeout{false};
    std::atomic<size_t> cycle{0};
    std::atomic<ServerState> server_state{ServerState::IDLE};
};

class DockingServer : public rclcpp::Node {
public:
    DockingServer();
    ~DockingServer();

private:
    // Helper functions
    static double getYawFromQuaternion(const tf2::Quaternion& q);
    void computePoseErrors(const geometry_msgs::msg::PoseStamped &current,
                           const geometry_msgs::msg::PoseStamped &target,
                           double &ex, double &ey, double &eyaw);
    std::pair<double, double> calculateDynamicTolerances(double distance) const;
    void publishStopGoalPose();

    // Action execution
    void executeDock(std::shared_ptr<GoalHandleDock> gh);
    void executeUndock(std::shared_ptr<GoalHandleUndock> gh);
    void diagnosticsLoop();

    // Parameters
    double y_tol_, yaw_tol_, x_tol_;
    double lookahead_, dock_timeout_sec_, undock_timeout_sec_;
    bool use_stages_;
    bool smooth_docking_;
    bool smooth_undocking_;
    double max_yaw_rate_;
    double max_y_tol_, min_y_tol_;
    double max_yaw_tol_, min_yaw_tol_;
    double tolerance_transition_dist_;
    std::string base_frame_;

    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_dock_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_active_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
    rclcpp_action::Server<Dock>::SharedPtr dock_action_server_;
    rclcpp_action::Server<Undock>::SharedPtr undock_action_server_;
    double straight_approach_distance_; // NEW: Added member variable

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Components
    std::unique_ptr<DockPoseManager> dock_pose_manager_;
    std::mutex marker_mutex_;
    geometry_msgs::msg::PoseStamped latest_marker_;
    rclcpp::Time last_marker_stamp_;
    DiagData diag_;
    std::thread diagnostics_thread_;
};

#endif // DOCKING_SERVER_HPP
