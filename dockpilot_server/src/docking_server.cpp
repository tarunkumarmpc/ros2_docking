// docking_server.cpp
// ROS 2 Humble / Iron / Jazzy compatible
// 3-stage docking with strict fine-mode entry < 10 cm
// All previous bug-fixes included.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "dockpilot_interfaces/action/dock.hpp"
#include "dockpilot_interfaces/action/undock.hpp"

#include <yaml-cpp/yaml.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <deque>
#include <numeric>
#include <cmath>
#include <unordered_map>
#include <exception>

#define RAD2DEG(x) ((x)*180.0 / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180.0)

/* ------------------------------------------------------------------ */
/* 1.  Helpers                                                        */
/* ------------------------------------------------------------------ */
double wrapAngleDeg(double yaw_deg)
{
  double current = std::fmod(yaw_deg + 180.0, 360.0);
  if (current < 0.0) current += 360.0;
  return current - 180.0;
}
double wrapAngleRad(double yaw_rad)
{
  return std::atan2(std::sin(yaw_rad), std::cos(yaw_rad));
}

/* ------------------------------------------------------------------ */
/* 2.  Rolling standard deviation                                     */
/* ------------------------------------------------------------------ */
struct RollingSigma
{
  std::deque<double> buf;
  const size_t capacity = 50;
  void push(double v)
  {
    buf.push_back(v);
    if (buf.size() > capacity) buf.pop_front();
  }
  double sigma() const
  {
    if (buf.empty()) return 1e-3;
    double mean = std::accumulate(buf.begin(), buf.end(), 0.0) / buf.size();
    double var = 0.0;
    for (double v : buf) var += (v - mean) * (v - mean);
    return std::sqrt(var / buf.size());
  }
};

/* ------------------------------------------------------------------ */
/* 3.  DockConfig                                                     */
/* ------------------------------------------------------------------ */
struct DockConfig
{
  geometry_msgs::msg::PoseStamped dock_pose;
  geometry_msgs::msg::PoseStamped undock_pose;
};

/* ------------------------------------------------------------------ */
/* 3.1 DockPoseManager                                                */
/* ------------------------------------------------------------------ */
class DockPoseManager
{
public:
  DockPoseManager(rclcpp::Node &node, tf2_ros::Buffer &tf_buffer,
                  const std::string &base_frame);

  void load(const std::string &file_path);
  bool get(const std::string &dock_id, DockConfig &out) const;

private:
  geometry_msgs::msg::PoseStamped parsePose(const YAML::Node &n,
                                            const std::string &def_frame) const;
  geometry_msgs::msg::PoseStamped transformToBase(
      const geometry_msgs::msg::PoseStamped &in) const;

  rclcpp::Node &node_;
  tf2_ros::Buffer &tf_buffer_;
  std::string base_frame_;
  std::unordered_map<std::string, DockConfig> configs_;
  DockConfig default_;
};

DockPoseManager::DockPoseManager(rclcpp::Node &node,
                                 tf2_ros::Buffer &tf_buffer,
                                 const std::string &base_frame)
    : node_(node), tf_buffer_(tf_buffer), base_frame_(base_frame)
{
  default_.dock_pose.header.frame_id = base_frame_;
  default_.dock_pose.pose.position.x = 0.10;
  default_.dock_pose.pose.orientation.w = 1.0;
  default_.undock_pose.header.frame_id = base_frame_;
  default_.undock_pose.pose.position.x = -0.30;
  default_.undock_pose.pose.orientation.w = 1.0;
}

void DockPoseManager::load(const std::string &file_path)
{
  if (file_path.empty())
  {
    RCLCPP_WARN(node_.get_logger(), "dock_config_file empty; using default.");
    return;
  }
  try
  {
    YAML::Node root = YAML::LoadFile(file_path);
    if (!root.IsMap()) {
      throw std::runtime_error("YAML root is not a map");
    }
    if (root["docks"]) {
      for (const auto &kv : root["docks"]) {
        std::string tag = kv.first.as<std::string>();
        DockConfig cfg;
        if (kv.second["dock_pose"] && kv.second["undock_pose"]) {
          cfg.dock_pose = transformToBase(parsePose(kv.second["dock_pose"], "camera_link"));
          cfg.undock_pose = transformToBase(parsePose(kv.second["undock_pose"], "camera_link"));
          configs_.emplace(tag, cfg);
        } else {
          RCLCPP_WARN(node_.get_logger(), "Invalid dock config for '%s'", tag.c_str());
        }
      }
    }
    if (root["default_dock"]) {
      default_.dock_pose =
          transformToBase(parsePose(root["default_dock"]["dock_pose"], "camera_link"));
      default_.undock_pose =
          transformToBase(parsePose(root["default_dock"]["undock_pose"], "camera_link"));
    }
    RCLCPP_INFO(node_.get_logger(), "Loaded %zu dock configurations", configs_.size());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node_.get_logger(), "YAML load failed: %s", e.what());
  }
}

bool DockPoseManager::get(const std::string &dock_id, DockConfig &out) const
{
  auto it = configs_.find(dock_id);
  if (it != configs_.end())
  {
    out = it->second;
    return true;
  }
  out = default_;
  RCLCPP_WARN(node_.get_logger(), "Using default dock for unknown ID '%s'", dock_id.c_str());
  return false;
}

geometry_msgs::msg::PoseStamped DockPoseManager::parsePose(
    const YAML::Node &n, const std::string &def_frame) const
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id =
      n["header"]["frame_id"] ? n["header"]["frame_id"].as<std::string>() : def_frame;
  const auto &pos = n["pose"]["position"];
  p.pose.position.x = pos["x"].as<double>(0.0);
  p.pose.position.y = pos["y"].as<double>(0.0);
  p.pose.position.z = pos["z"].as<double>(0.0);

  if (n["pose"]["orientation"])
  {
    const auto &q = n["pose"]["orientation"];
    p.pose.orientation.x = q["x"].as<double>(0.0);
    p.pose.orientation.y = q["y"].as<double>(0.0);
    p.pose.orientation.z = q["z"].as<double>(0.0);
    p.pose.orientation.w = q["w"].as<double>(1.0);
  }
  else if (n["pose"]["yaw"])
  {
    double yaw_deg = n["pose"]["yaw"].as<double>(0.0);
    double yaw_rad = wrapAngleDeg(yaw_deg) * M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    p.pose.orientation = tf2::toMsg(q);
  }
  else
  {
    p.pose.orientation.w = 1.0;
  }

  tf2::Quaternion q;
  tf2::fromMsg(p.pose.orientation, q);
  if (q.length() < 1e-6) {
    RCLCPP_WARN(node_.get_logger(), "Invalid quaternion in pose; normalizing to identity");
    q.setRPY(0.0, 0.0, 0.0);
  } else {
    q.normalize();
  }
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

geometry_msgs::msg::PoseStamped
DockPoseManager::transformToBase(const geometry_msgs::msg::PoseStamped &in) const
{
  if (in.header.frame_id == base_frame_) return in;
  geometry_msgs::msg::PoseStamped out;
  try
  {
    tf_buffer_.transform(in, out, base_frame_, tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(node_.get_logger(), "TF transform failed: %s", ex.what());
    return in;
  }
  return out;
}

/* ------------------------------------------------------------------ */
/* 4.  Action typedefs                                                */
/* ------------------------------------------------------------------ */
using Dock = dockpilot_interfaces::action::Dock;
using Undock = dockpilot_interfaces::action::Undock;
using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;
using GoalHandleUndock = rclcpp_action::ServerGoalHandle<Undock>;

/* ------------------------------------------------------------------ */
/* 5.  Distance-banded staged controller                              */
/* ------------------------------------------------------------------ */
struct BandCtl
{
  double x, y, yaw;
  bool ok;
};

BandCtl bandStep(double ex, double ey, double eyaw, const double *max_step)
{
  BandCtl out;
  out.x   = std::copysign(std::min(max_step[0], 0.7 * std::fabs(ex)), ex);
  out.y   = std::copysign(std::min(max_step[1], 0.7 * std::fabs(ey)), ey);
  out.yaw = std::copysign(std::min(max_step[2], 0.7 * std::fabs(eyaw)), eyaw);
  out.ok  = (std::fabs(ex) < 1e-6 && std::fabs(ey) < 1e-6 && std::fabs(eyaw) < 1e-6);
  return out;
}

/* ------------------------------------------------------------------ */
/* 6.  Stages                                                         */
/* ------------------------------------------------------------------ */
enum class DockingStage {
  ALIGN_Y = 0,
  ALIGN_YAW = 1,
  ALIGN_X = 2,
  FINAL_APPROACH = 3
};

enum class FineSubStage {
  NONE = 0,
  COARSE_FINAL = 1,
  FINE_Y = 2,
  FINE_YAW = 3,
  FINE_X = 4
};

/* ------------------------------------------------------------------ */
/* 7.  Main server                                                    */
/* ------------------------------------------------------------------ */
class DockingServer : public rclcpp::Node
{
public:
  DockingServer();
  ~DockingServer();

private:
  /* ---------- parameters ---------- */
  double final_x_tol_{0.003};
  double final_y_tol_{0.003};
  double final_yaw_tol_{0.003};
  double dock_timeout_sec_{0.0};
  double undock_timeout_sec_{0.0};
  double kDwellTime_{1.0};
  std::string base_frame_{"base_link"};
  double fine_align_dist_{0.20};
  double hysteresis_y_{0.0005};
  double hysteresis_yaw_{0.005};
  double yaw_drift_threshold_{0.005};

  /* ---------- tf ---------- */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* ---------- dock poses ---------- */
  std::unique_ptr<DockPoseManager> dock_pose_manager_;

  /* ---------- pubs / subs ---------- */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_dock_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_active_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;

  /* ---------- action servers ---------- */
  rclcpp_action::Server<Dock>::SharedPtr dock_action_server_;
  rclcpp_action::Server<Undock>::SharedPtr undock_action_server_;

  /* ---------- diagnostics ---------- */
  struct Diag
  {
    std::atomic<double> ex{0}, ey{0}, eyaw{0}, distance{0};
    std::atomic<double> x_tol{0}, y_tol{0}, yaw_tol{0};
    std::atomic<bool> marker_timeout{false};
    std::atomic<uint64_t> cycle{0};
    std::atomic<int> stage{0};
    std::atomic<int> sub_stage{0};
  } diag_;

  std::thread diagnostics_thread_;

  /* ---------- marker state ---------- */
  std::mutex marker_mutex_;
  geometry_msgs::msg::PoseStamped latest_marker_;
  rclcpp::Time last_marker_stamp_;

  /* ---------- rolling stats ---------- */
  RollingSigma roll_x_, roll_y_, roll_yaw_;

  /* ---------- methods ---------- */
  std::tuple<double, double, double> adaptiveTolerance(double d, double ex, double ey,
                                                       double eyaw);
  double getYaw(const geometry_msgs::msg::Quaternion &q);
  void computeErrors(const geometry_msgs::msg::PoseStamped &curr,
                     const geometry_msgs::msg::PoseStamped &tgt,
                     double &ex, double &ey, double &eyaw);
  void publishStop();

  bool isPoseValid(const geometry_msgs::msg::PoseStamped &pose);
  void handleAlignY(double ex, double ey, double eyaw,
                    const geometry_msgs::msg::PoseStamped &base_pose,
                    const geometry_msgs::msg::PoseStamped &target,
                    const double *max_step, geometry_msgs::msg::PoseStamped &wp);
  void handleAlignYaw(double ex, double ey, double eyaw,
                      const geometry_msgs::msg::PoseStamped &base_pose,
                      const geometry_msgs::msg::PoseStamped &target,
                      const double *max_step, geometry_msgs::msg::PoseStamped &wp);
  void handleAlignX(double ex, double ey, double eyaw,
                    const geometry_msgs::msg::PoseStamped &base_pose,
                    const geometry_msgs::msg::PoseStamped &target,
                    const double *max_step, geometry_msgs::msg::PoseStamped &wp);
  void handleCoarseFinal(double ex, double ey, double eyaw,
                         const geometry_msgs::msg::PoseStamped &base_pose,
                         const geometry_msgs::msg::PoseStamped &target,
                         const double *max_step, geometry_msgs::msg::PoseStamped &wp,
                         int &fine_sub_stage, bool &in_fine_phase);
  void handleFinePhase(double ex, double ey, double eyaw, double x_tol, double y_tol, double yaw_tol,
                       const geometry_msgs::msg::PoseStamped &base_pose,
                       const geometry_msgs::msg::PoseStamped &target,
                       const double *max_step, geometry_msgs::msg::PoseStamped &wp,
                       int &fine_sub_stage, double &last_ey, double &last_eyaw, bool &in_fine_phase);
  void executeDock(std::shared_ptr<GoalHandleDock> gh);
  void executeUndock(std::shared_ptr<GoalHandleUndock> gh);
  void diagnosticsLoop();
};

/* ------------------------------------------------------------------ */
DockingServer::DockingServer()
    : Node("docking_server")
{
  declare_parameter("final_x_tolerance", final_x_tol_);
  declare_parameter("final_y_tolerance", final_y_tol_);
  declare_parameter("final_yaw_tolerance", final_yaw_tol_);
  declare_parameter("dock_timeout", dock_timeout_sec_);
  declare_parameter("undock_timeout", undock_timeout_sec_);
  declare_parameter("base_frame", base_frame_);
  declare_parameter("dock_config_file", std::string(""));
  declare_parameter("dwell_time", kDwellTime_);
  declare_parameter("fine_align_distance", fine_align_dist_);
  declare_parameter("hysteresis_y", hysteresis_y_);
  declare_parameter("hysteresis_yaw", hysteresis_yaw_);
  declare_parameter("yaw_drift_threshold", yaw_drift_threshold_);

  get_parameter("final_x_tolerance", final_x_tol_);
  get_parameter("final_y_tolerance", final_y_tol_);
  get_parameter("final_yaw_tolerance", final_yaw_tol_);
  get_parameter("dock_timeout", dock_timeout_sec_);
  get_parameter("undock_timeout", undock_timeout_sec_);
  get_parameter("base_frame", base_frame_);
  get_parameter("dwell_time", kDwellTime_);
  get_parameter("fine_align_distance", fine_align_dist_);
  get_parameter("hysteresis_y", hysteresis_y_);
  get_parameter("hysteresis_yaw", hysteresis_yaw_);
  get_parameter("yaw_drift_threshold", yaw_drift_threshold_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string dock_config_file;
  get_parameter("dock_config_file", dock_config_file);
  dock_pose_manager_ = std::make_unique<DockPoseManager>(*this, *tf_buffer_, base_frame_);
  dock_pose_manager_->load(dock_config_file);

  active_dock_pub_ = create_publisher<std_msgs::msg::String>("/active_dock_id", 10);
  docking_active_pub_ = create_publisher<std_msgs::msg::Bool>("/docking_active", 10);
  goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

  marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/filtered_tag_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        latest_marker_ = *msg;
        last_marker_stamp_ = now();
      });

  dock_action_server_ = rclcpp_action::create_server<Dock>(
      this, "dock",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Dock::Goal>)
      { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](const std::shared_ptr<GoalHandleDock>)
      { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandleDock> gh)
      {
        std::thread{&DockingServer::executeDock, this, gh}.detach();
      });

  undock_action_server_ = rclcpp_action::create_server<Undock>(
      this, "undock",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Undock::Goal>)
      { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](const std::shared_ptr<GoalHandleUndock>)
      { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandleUndock> gh)
      {
        std::thread{&DockingServer::executeUndock, this, gh}.detach();
      });

  diagnostics_thread_ = std::thread(&DockingServer::diagnosticsLoop, this);

  RCLCPP_INFO(get_logger(),
              "DockingServer ready: final (x,y,yaw) = (%.3f m, %.3f m, %.3f rad)",
              final_x_tol_, final_y_tol_, final_yaw_tol_);
}

DockingServer::~DockingServer()
{
  if (diagnostics_thread_.joinable()) diagnostics_thread_.join();
}

/* ------------------------------------------------------------------ */
std::tuple<double, double, double>
DockingServer::adaptiveTolerance(double distance, double ex, double ey, double eyaw)
{
  roll_x_.push(std::fabs(ex));
  roll_y_.push(std::fabs(ey));
  roll_yaw_.push(std::fabs(eyaw));

  double scale = std::clamp(distance / 0.40, 0.0, 1.0);
  double x_tol = std::max(final_x_tol_, 2.0 * roll_x_.sigma() * scale);
  double y_tol = std::max(final_y_tol_, 1.5 * roll_y_.sigma() * scale);
  double yaw_tol = std::max(final_yaw_tol_, 3.0 * roll_yaw_.sigma() * scale);

  diag_.x_tol.store(x_tol);
  diag_.y_tol.store(y_tol);
  diag_.yaw_tol.store(yaw_tol);
  return {x_tol, y_tol, yaw_tol};
}

double DockingServer::getYaw(const geometry_msgs::msg::Quaternion &q)
{
  tf2::Quaternion qq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(qq);
  double r, p, y;
  m.getRPY(r, p, y);
  return wrapAngleRad(y);
}

void DockingServer::computeErrors(const geometry_msgs::msg::PoseStamped &curr,
                                  const geometry_msgs::msg::PoseStamped &tgt,
                                  double &ex, double &ey, double &eyaw)
{
  ex = tgt.pose.position.x - curr.pose.position.x;
  ey = tgt.pose.position.y - curr.pose.position.y;

  tf2::Quaternion qc, qt;
  tf2::fromMsg(curr.pose.orientation, qc);
  tf2::fromMsg(tgt.pose.orientation, qt);
  eyaw = qc.angleShortestPath(qt);
}

void DockingServer::publishStop()
{
  geometry_msgs::msg::PoseStamped stop;
  stop.header.frame_id = base_frame_;
  stop.header.stamp = now();
  stop.pose.orientation.w = 1.0;
  goal_pose_pub_->publish(stop);
}

bool DockingServer::isPoseValid(const geometry_msgs::msg::PoseStamped &pose)
{
  if (pose.header.frame_id.empty() || std::isnan(pose.pose.position.x) || std::isnan(pose.pose.position.y)) {
    return false;
  }
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  return q.length() > 1e-6;
}

/* ------------------------------------------------------------------ */
void DockingServer::handleAlignY(double ex, double ey, double eyaw,
                                 const geometry_msgs::msg::PoseStamped &base_pose,
                                 const geometry_msgs::msg::PoseStamped &target,
                                 const double *max_step, geometry_msgs::msg::PoseStamped &wp)
{
  (void)ex;      // unused in Y-alignment stage
  (void)eyaw;    // unused in Y-alignment stage
  (void)target;  // unused in Y-alignment stage (consistent handler signature)
  BandCtl b = bandStep(0.0, ey, 0.0, max_step);
  wp.pose.position.x = base_pose.pose.position.x;
  wp.pose.position.y = base_pose.pose.position.y + b.y;
  wp.pose.orientation = base_pose.pose.orientation;
  RCLCPP_DEBUG(get_logger(), "ALIGN_Y: Correcting y=%.3f", b.y);
  diag_.sub_stage.store(static_cast<int>(FineSubStage::NONE));
}

void DockingServer::handleAlignYaw(double ex, double ey, double eyaw,
                                   const geometry_msgs::msg::PoseStamped &base_pose,
                                   const geometry_msgs::msg::PoseStamped &target,
                                   const double *max_step, geometry_msgs::msg::PoseStamped &wp)
{
  (void)ex;
  (void)ey;
  BandCtl b = bandStep(0.0, 0.0, eyaw, max_step);
  wp.pose.position = base_pose.pose.position;
  tf2::Quaternion q, qg;
  tf2::fromMsg(base_pose.pose.orientation, q);
  tf2::fromMsg(target.pose.orientation, qg);
  tf2::Quaternion q_step = q.slerp(qg, b.yaw / (std::fabs(eyaw) + 1e-6));
  wp.pose.orientation = tf2::toMsg(q_step);
  RCLCPP_DEBUG(get_logger(), "ALIGN_YAW: Correcting yaw=%.3f", b.yaw);
  diag_.sub_stage.store(static_cast<int>(FineSubStage::NONE));
}

void DockingServer::handleAlignX(double ex, double ey, double eyaw,
                                 const geometry_msgs::msg::PoseStamped &base_pose,
                                 const geometry_msgs::msg::PoseStamped &target,
                                 const double *max_step, geometry_msgs::msg::PoseStamped &wp)
{
  BandCtl b = bandStep(ex, 0.0, 0.0, max_step);
  wp.pose.position.x = base_pose.pose.position.x + b.x;
  wp.pose.position.y = base_pose.pose.position.y;
  wp.pose.orientation = base_pose.pose.orientation;
  if (std::fabs(ey) > hysteresis_y_) {
    double small_y_step = std::copysign(std::min(max_step[1] * 0.2, 0.3 * std::fabs(ey)), ey);
    wp.pose.position.y += small_y_step;
    RCLCPP_DEBUG(get_logger(), "ALIGN_X: Applied small y correction %.3f due to drift", small_y_step);
  }
  if (std::fabs(eyaw) > yaw_drift_threshold_) {
    double small_yaw_step = std::copysign(std::min(max_step[2] * 0.2, 0.3 * std::fabs(eyaw)), eyaw);
    tf2::Quaternion q, qg;
    tf2::fromMsg(base_pose.pose.orientation, q);
    tf2::fromMsg(target.pose.orientation, qg);
    tf2::Quaternion q_step = q.slerp(qg, small_yaw_step / (std::fabs(eyaw) + 1e-6));
    wp.pose.orientation = tf2::toMsg(q_step);
    RCLCPP_DEBUG(get_logger(), "ALIGN_X: Applied small yaw correction %.3f due to drift", small_yaw_step);
  }
  RCLCPP_DEBUG(get_logger(), "ALIGN_X: Correcting x=%.3f", b.x);
  diag_.sub_stage.store(static_cast<int>(FineSubStage::NONE));
}

void DockingServer::handleCoarseFinal(double ex, double ey, double eyaw,
                                      const geometry_msgs::msg::PoseStamped &base_pose,
                                      const geometry_msgs::msg::PoseStamped &target,
                                      const double *max_step, geometry_msgs::msg::PoseStamped &wp,
                                      int &fine_sub_stage, bool &in_fine_phase)
{
  in_fine_phase = false;
  fine_sub_stage = static_cast<int>(FineSubStage::NONE);
  BandCtl b = bandStep(ex, ey, eyaw, max_step);
  wp.pose.position.x = base_pose.pose.position.x + b.x;
  wp.pose.position.y = base_pose.pose.position.y + b.y;
  tf2::Quaternion q, qg;
  tf2::fromMsg(base_pose.pose.orientation, q);
  tf2::fromMsg(target.pose.orientation, qg);
  tf2::Quaternion q_step = q.slerp(qg, b.yaw / (std::fabs(eyaw) + 1e-6));
  wp.pose.orientation = tf2::toMsg(q_step);
  RCLCPP_DEBUG(get_logger(), "COARSE_FINAL: Simultaneous corr x=%.3f y=%.3f yaw=%.3f", b.x, b.y, b.yaw);
  diag_.sub_stage.store(static_cast<int>(FineSubStage::COARSE_FINAL));
}

void DockingServer::handleFinePhase(double ex, double ey, double eyaw, double x_tol, double y_tol, double yaw_tol,
                                    const geometry_msgs::msg::PoseStamped &base_pose,
                                    const geometry_msgs::msg::PoseStamped &target,
                                    const double *max_step, geometry_msgs::msg::PoseStamped &wp,
                                    int &fine_sub_stage, double &last_ey, double &last_eyaw, bool &in_fine_phase)
{
  in_fine_phase = true;
  BandCtl b;
  switch (fine_sub_stage) {
    case static_cast<int>(FineSubStage::FINE_Y): {
      b = bandStep(0.0, ey, 0.0, max_step);
      wp.pose.position.x = base_pose.pose.position.x;
      wp.pose.position.y = base_pose.pose.position.y + b.y;
      wp.pose.orientation = base_pose.pose.orientation;
      if (std::fabs(eyaw) > yaw_drift_threshold_) {
        double small_yaw_step = std::copysign(std::min(max_step[2] * 0.2, 0.3 * std::fabs(eyaw)), eyaw);
        tf2::Quaternion q, qg;
        tf2::fromMsg(base_pose.pose.orientation, q);
        tf2::fromMsg(target.pose.orientation, qg);
        tf2::Quaternion q_step = q.slerp(qg, small_yaw_step / (std::fabs(eyaw) + 1e-6));
        wp.pose.orientation = tf2::toMsg(q_step);
        RCLCPP_DEBUG(get_logger(), "FINE_Y: Applied small yaw correction %.3f due to drift", small_yaw_step);
      }
      if (std::fabs(ey) < y_tol && std::fabs(ey - last_ey) < hysteresis_y_) {
        fine_sub_stage = static_cast<int>(FineSubStage::FINE_YAW);
        RCLCPP_DEBUG(get_logger(), "Transition to FINE_YAW: ey=%.6f, y_tol=%.6f", ey, y_tol);
      }
      last_ey = ey;
      RCLCPP_DEBUG(get_logger(), "FINE_Y: Correcting y=%.3f", b.y);
      diag_.sub_stage.store(static_cast<int>(FineSubStage::FINE_Y));
      break;
    }
    case static_cast<int>(FineSubStage::FINE_YAW): {
      b = bandStep(0.0, 0.0, eyaw, max_step);
      wp.pose.position = base_pose.pose.position;
      tf2::Quaternion q, qg;
      tf2::fromMsg(base_pose.pose.orientation, q);
      tf2::fromMsg(target.pose.orientation, qg);
      tf2::Quaternion q_step = q.slerp(qg, b.yaw / (std::fabs(eyaw) + 1e-6));
      wp.pose.orientation = tf2::toMsg(q_step);
      if (std::fabs(eyaw) < yaw_tol && std::fabs(eyaw - last_eyaw) < hysteresis_yaw_) {
        fine_sub_stage = static_cast<int>(FineSubStage::FINE_X);
        RCLCPP_DEBUG(get_logger(), "Transition to FINE_X: eyaw=%.6f, yaw_tol=%.6f", eyaw, yaw_tol);
      }
      last_eyaw = eyaw;
      RCLCPP_DEBUG(get_logger(), "FINE_YAW: Correcting yaw=%.3f", b.yaw);
      diag_.sub_stage.store(static_cast<int>(FineSubStage::FINE_YAW));
      break;
    }
    case static_cast<int>(FineSubStage::FINE_X): {
      b = bandStep(ex, 0.0, 0.0, max_step);
      wp.pose.position.x = base_pose.pose.position.x + b.x;
      wp.pose.position.y = base_pose.pose.position.y;
      wp.pose.orientation = base_pose.pose.orientation;
      if (std::fabs(ex) < x_tol) {
        fine_sub_stage = static_cast<int>(FineSubStage::FINE_Y);
        RCLCPP_DEBUG(get_logger(), "Transition to FINE_Y: ex=%.6f, x_tol=%.6f", ex, x_tol);
      }
      RCLCPP_DEBUG(get_logger(), "FINE_X: Advancing x=%.3f", b.x);
      diag_.sub_stage.store(static_cast<int>(FineSubStage::FINE_X));
      break;
    }
  }
  if (std::fabs(ey) > 1.5 * y_tol || std::fabs(eyaw) > 1.5 * yaw_tol) {
    fine_sub_stage = static_cast<int>(FineSubStage::FINE_Y);
    RCLCPP_DEBUG(get_logger(), "Reset to FINE_Y: ey=%.6f, eyaw=%.6f", ey, eyaw);
  }
}

/* ------------------------------------------------------------------ */
void DockingServer::executeDock(std::shared_ptr<GoalHandleDock> gh)
{
  const auto goal = gh->get_goal();
  RCLCPP_INFO(get_logger(), "Docking started for '%s'", goal->dock_id.c_str());

  auto result = std::make_shared<Dock::Result>();
  rclcpp::Rate loop(20);
  rclcpp::Time start_time = now();

  std_msgs::msg::String dock_id_msg;
  dock_id_msg.data = goal->dock_id;
  active_dock_pub_->publish(dock_id_msg);

  std_msgs::msg::Bool active_msg;
  active_msg.data = true;
  docking_active_pub_->publish(active_msg);

  DockConfig cfg;
  if (!dock_pose_manager_->get(goal->dock_id, cfg))
  {
    RCLCPP_ERROR(get_logger(), "Unknown dock_id '%s'", goal->dock_id.c_str());
    result->success = false;
    result->message = "unknown dock";
    gh->abort(result);
    publishStop();
    active_msg.data = false;
    docking_active_pub_->publish(active_msg);
    return;
  }
  const geometry_msgs::msg::PoseStamped target = cfg.dock_pose;

  if (!isPoseValid(target)) {
    RCLCPP_ERROR(get_logger(), "Invalid target pose for dock '%s'", goal->dock_id.c_str());
    result->success = false;
    result->message = "invalid pose";
    gh->abort(result);
    publishStop();
    active_msg.data = false;
    docking_active_pub_->publish(active_msg);
    return;
  }

  /* ---------- reset all state variables ---------- */
  DockingStage stage = DockingStage::ALIGN_Y;
  int fine_sub_stage = static_cast<int>(FineSubStage::NONE);
  bool in_fine_phase = false;
  bool strict_fine_mode = false;   // NEW: guarantees fine-only after <10 cm
  bool inside_tol = false;
  rclcpp::Time tol_enter;
  double last_ey = 0.0;
  double last_eyaw = 0.0;

  static constexpr double kCoarse[3] = {0.30, 0.30, 0.50};
  static constexpr double kMed[3]    = {0.15, 0.15, 0.25};
  static constexpr double kSeqFine[3] = {0.02, 0.02, 0.05};

  while (rclcpp::ok())
  {
    if (gh->is_canceling())
    {
      result->success = false;
      result->message = "canceled";
      gh->canceled(result);
      publishStop();
      active_msg.data = false;
      docking_active_pub_->publish(active_msg);
      diag_.stage.store(static_cast<int>(DockingStage::ALIGN_Y));
      diag_.sub_stage.store(static_cast<int>(FineSubStage::NONE));
      return;
    }
    if (dock_timeout_sec_ > 0.0 && (now() - start_time).seconds() > dock_timeout_sec_)
    {
      result->success = false;
      result->message = "timeout";
      gh->abort(result);
      publishStop();
      active_msg.data = false;
      docking_active_pub_->publish(active_msg);
      diag_.stage.store(static_cast<int>(DockingStage::ALIGN_Y));
      diag_.sub_stage.store(static_cast<int>(FineSubStage::NONE));
      return;
    }

    geometry_msgs::msg::PoseStamped latest;
    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      latest = latest_marker_;
    }
    if (latest.header.frame_id.empty() || (now() - last_marker_stamp_).seconds() > 0.5)
    {
      diag_.marker_timeout.store(true);
      loop.sleep();
      continue;
    }
    diag_.marker_timeout.store(false);

    geometry_msgs::msg::PoseStamped base_pose;
    try
    {
      tf_buffer_->transform(latest, base_pose, base_frame_, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF2: %s", ex.what());
      loop.sleep();
      continue;
    }

    if (!isPoseValid(base_pose)) {
      RCLCPP_WARN(get_logger(), "Invalid base pose; skipping cycle");
      loop.sleep();
      continue;
    }

    double ex, ey, eyaw;
    computeErrors(base_pose, target, ex, ey, eyaw);
    double distance = std::hypot(ex, ey);
    double x_tol, y_tol, yaw_tol;
    if (stage == DockingStage::FINAL_APPROACH) {
      x_tol = final_x_tol_;
      y_tol = final_y_tol_;
      yaw_tol = final_yaw_tol_;
    } else {
      std::tie(x_tol, y_tol, yaw_tol) = adaptiveTolerance(distance, ex, ey, eyaw);
    }

    /* ---------- strict fine-mode entry ---------- */
    if (!strict_fine_mode && distance < 0.10) {
      stage = DockingStage::FINAL_APPROACH;
      fine_sub_stage = static_cast<int>(FineSubStage::FINE_Y);
      in_fine_phase = true;
      strict_fine_mode = true;
      RCLCPP_INFO(get_logger(), "Entered strict fine-mode (distance < 0.10 m)");
    }

    /* ---------- stage selection (only if not in strict fine-mode) ---------- */
    if (!strict_fine_mode) {
      if (std::fabs(eyaw) > 3.0 * yaw_tol) {
        stage = DockingStage::ALIGN_YAW;
      } else if (std::fabs(ey) > 3.0 * y_tol) {
        stage = DockingStage::ALIGN_Y;
      } else if (std::fabs(ex) > 3.0 * x_tol && distance > fine_align_dist_) {
        stage = DockingStage::ALIGN_X;
      } else {
        stage = DockingStage::FINAL_APPROACH;
      }
    }

    diag_.ex.store(ex);
    diag_.ey.store(ey);
    diag_.eyaw.store(eyaw);
    diag_.distance.store(distance);
    diag_.x_tol.store(x_tol);
    diag_.y_tol.store(y_tol);
    diag_.yaw_tol.store(yaw_tol);
    diag_.stage.store(static_cast<int>(stage));

    RCLCPP_DEBUG(get_logger(), "Stage: %d, Sub-stage: %d, Distance: %.3f m, "
                "Ex: %.6f m, Ey: %.6f m, Eyaw: %.6f rad, "
                "x_tol: %.6f m, y_tol: %.6f m, yaw_tol: %.6f rad",
                static_cast<int>(stage), fine_sub_stage, distance, ex, ey, eyaw, x_tol, y_tol, yaw_tol);

    /* ---------- select max step ---------- */
    const double *max_step;
    if (strict_fine_mode || distance <= fine_align_dist_) {
      max_step = kSeqFine;
    } else if (distance > 0.40) {
      max_step = kCoarse;
    } else {
      max_step = kMed;
    }

    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = base_frame_;
    wp.header.stamp = now();

    if (stage == DockingStage::ALIGN_Y) {
      handleAlignY(ex, ey, eyaw, base_pose, target, max_step, wp);
    } else if (stage == DockingStage::ALIGN_YAW) {
      handleAlignYaw(ex, ey, eyaw, base_pose, target, max_step, wp);
    } else if (stage == DockingStage::ALIGN_X) {
      handleAlignX(ex, ey, eyaw, base_pose, target, max_step, wp);
    } else {
      if (!in_fine_phase) {
        handleCoarseFinal(ex, ey, eyaw, base_pose, target, max_step, wp, fine_sub_stage, in_fine_phase);
      } else {
        handleFinePhase(ex, ey, eyaw, x_tol, y_tol, yaw_tol, base_pose, target, max_step, wp,
                        fine_sub_stage, last_ey, last_eyaw, in_fine_phase);
      }
    }

    diag_.cycle.fetch_add(1);
    goal_pose_pub_->publish(wp);

    auto fb = std::make_shared<Dock::Feedback>();
    fb->current_marker_pose = base_pose;
    fb->current_robot_pose = base_pose;
    fb->distance_to_goal = distance;
    fb->angle_error_to_goal = std::fabs(eyaw);
    gh->publish_feedback(fb);

    if (stage == DockingStage::FINAL_APPROACH &&
        std::fabs(ex) < x_tol && std::fabs(ey) < y_tol && std::fabs(eyaw) < yaw_tol)
    {
      if (!inside_tol)
      {
        inside_tol = true;
        tol_enter = now();
        RCLCPP_DEBUG(get_logger(), "Entered tolerance zone at time: %.3f s", tol_enter.seconds());
      }
      if ((now() - tol_enter).seconds() >= kDwellTime_)
      {
        RCLCPP_INFO(get_logger(),
                    "Docking SUCCESS: achieved within x_tol=%.6f m, y_tol=%.6f m, yaw_tol=%.6f rad",
                    x_tol, y_tol, yaw_tol);
        result->success = true;
        result->message = "success";
        gh->succeed(result);
        publishStop();
        active_msg.data = false;
        docking_active_pub_->publish(active_msg);
        diag_.stage.store(static_cast<int>(DockingStage::ALIGN_Y));
        diag_.sub_stage.store(static_cast<int>(FineSubStage::NONE));
        return;
      }
    }
    else
    {
      if (inside_tol) {
        RCLCPP_DEBUG(get_logger(), "Exited tolerance zone");
      }
      inside_tol = false;
    }

    loop.sleep();
  }
}

/* ------------------------------------------------------------------ */
void DockingServer::executeUndock(std::shared_ptr<GoalHandleUndock> gh)
{
  const auto goal = gh->get_goal();
  RCLCPP_INFO(get_logger(), "Undocking started");
  auto result = std::make_shared<Undock::Result>();
  rclcpp::Rate loop(20);
  rclcpp::Time start_time = now();

  std_msgs::msg::Bool active_msg;
  active_msg.data = true;
  docking_active_pub_->publish(active_msg);

  DockConfig cfg;
  if (!dock_pose_manager_->get(goal->dock_id, cfg))
  {
    result->success = false;
    result->message = "unknown dock";
    gh->abort(result);
    publishStop();
    active_msg.data = false;
    docking_active_pub_->publish(active_msg);
    return;
  }
  const geometry_msgs::msg::PoseStamped target = cfg.undock_pose;

  if (!isPoseValid(target)) {
    RCLCPP_ERROR(get_logger(), "Invalid undock pose for '%s'", goal->dock_id.c_str());
    result->success = false;
    result->message = "invalid pose";
    gh->abort(result);
    publishStop();
    active_msg.data = false;
    docking_active_pub_->publish(active_msg);
    return;
  }

  static constexpr double kCoarse[3] = {0.30, 0.30, 0.50};
  static constexpr double kMed[3]    = {0.15, 0.15, 0.25};
  static constexpr double kFine[3]   = {0.02, 0.02, 0.10};

  while (rclcpp::ok())
  {
    if (gh->is_canceling())
    {
      result->success = false;
      result->message = "canceled";
      gh->canceled(result);
      publishStop();
      active_msg.data = false;
      docking_active_pub_->publish(active_msg);
      return;
    }
    if (undock_timeout_sec_ > 0.0 && (now() - start_time).seconds() > undock_timeout_sec_)
    {
      result->success = false;
      result->message = "timeout";
      gh->abort(result);
      publishStop();
      active_msg.data = false;
      docking_active_pub_->publish(active_msg);
      return;
    }

    geometry_msgs::msg::PoseStamped latest;
    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      latest = latest_marker_;
    }
    if (latest.header.frame_id.empty() || (now() - last_marker_stamp_).seconds() > 0.5)
    {
      loop.sleep();
      continue;
    }

    geometry_msgs::msg::PoseStamped base_pose;
    try
    {
      tf_buffer_->transform(latest, base_pose, base_frame_, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF2: %s", ex.what());
      loop.sleep();
      continue;
    }

    double ex, ey, eyaw;
    computeErrors(base_pose, target, ex, ey, eyaw);
    double distance = std::hypot(ex, ey);

    const double *max_step = (distance > 0.40) ? kCoarse :
                             (distance > 0.10) ? kMed   : kFine;

    BandCtl b = bandStep(ex, ey, eyaw, max_step);

    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = base_frame_;
    wp.header.stamp = now();
    wp.pose.position.x = base_pose.pose.position.x + b.x;
    wp.pose.position.y = base_pose.pose.position.y + b.y;
    tf2::Quaternion q, qg;
    tf2::fromMsg(base_pose.pose.orientation, q);
    tf2::fromMsg(target.pose.orientation, qg);
    tf2::Quaternion q_step = q.slerp(qg, b.yaw / (std::fabs(eyaw) + 1e-6));
    wp.pose.orientation = tf2::toMsg(q_step);

    goal_pose_pub_->publish(wp);

    auto fb = std::make_shared<Undock::Feedback>();
    fb->current_robot_pose = base_pose;
    fb->distance_remaining = distance;
    gh->publish_feedback(fb);

    if (distance < 0.05 && std::fabs(eyaw) < final_yaw_tol_)
    {
      RCLCPP_INFO(get_logger(), "Undocking SUCCESS");
      result->success = true;
      result->message = "success";
      gh->succeed(result);
      publishStop();
      active_msg.data = false;
      docking_active_pub_->publish(active_msg);
      return;
    }
    loop.sleep();
  }
}

/* ------------------------------------------------------------------ */
void DockingServer::diagnosticsLoop()
{
  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    diagnostic_msgs::msg::DiagnosticArray da;
    da.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    ds.name = "docking_server";

    auto add = [&](const char *key, double v)
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = std::to_string(v);
      ds.values.emplace_back(kv);
    };
    auto add_str = [&](const char *key, const std::string &v)
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = v;
      ds.values.emplace_back(kv);
    };

    add("error_x", diag_.ex.load());
    add("error_y", diag_.ey.load());
    add("error_yaw_rad", diag_.eyaw.load());
    add("error_yaw_deg", wrapAngleDeg(diag_.eyaw.load() * 180.0 / M_PI));
    add("distance", diag_.distance.load());
    add("x_tolerance", diag_.x_tol.load());
    add("y_tolerance", diag_.y_tol.load());
    add("yaw_tolerance", diag_.yaw_tol.load());
    add("marker_timeout", diag_.marker_timeout.load());
    add("cycle", static_cast<int>(diag_.cycle.load()));

    std::string stage_str;
    switch (static_cast<DockingStage>(diag_.stage.load()))
    {
      case DockingStage::ALIGN_Y: stage_str = "ALIGN_Y"; break;
      case DockingStage::ALIGN_YAW: stage_str = "ALIGN_YAW"; break;
      case DockingStage::ALIGN_X: stage_str = "ALIGN_X"; break;
      case DockingStage::FINAL_APPROACH: stage_str = "FINAL_APPROACH"; break;
      default: stage_str = "UNKNOWN";
    }
    add_str("stage", stage_str);

    std::string sub_stage_str;
    switch (static_cast<FineSubStage>(diag_.sub_stage.load()))
    {
      case FineSubStage::NONE: sub_stage_str = "NONE"; break;
      case FineSubStage::COARSE_FINAL: sub_stage_str = "COARSE_FINAL"; break;
      case FineSubStage::FINE_Y: sub_stage_str = "FINE_Y"; break;
      case FineSubStage::FINE_YAW: sub_stage_str = "FINE_YAW"; break;
      case FineSubStage::FINE_X: sub_stage_str = "FINE_X"; break;
      default: sub_stage_str = "UNKNOWN";
    }
    add_str("sub_stage", sub_stage_str);

    da.status.emplace_back(ds);
    diagnostics_pub_->publish(da);
    rate.sleep();
  }
}

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingServer>());
  rclcpp::shutdown();
  return 0;
}