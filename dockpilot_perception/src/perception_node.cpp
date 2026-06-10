//working

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <opencv2/opencv.hpp>
#include "dockpilot_perception/tag_ekf.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

inline double computeYawFromXY(double x, double y) { return std::atan2(y, x); }

double wrapAngle(double yaw_deg) {
    double current = std::fmod(yaw_deg + 180.0, 360.0);
    if (current < 0.0) current += 360.0;
    return current - 180.0;
}

struct AdaptiveParams {
  double basePos = 0.02;
  double kPos = 15.0;
  double baseYaw = 1.5;
  double kYaw = 10.0;
  double minPosLimit = 0.01;
  double maxPosLimit = 0.2;
  double minYawLimit = 5.0;
  double maxYawLimit = 20.0;
};

struct PoseSample {
    rclcpp::Time timestamp;
    double x, y;
    geometry_msgs::msg::Pose pose;
    double reprojErr;
    std::string method;
    int solution_index;
};

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {
    declare_parameter<bool>("use_arctan_yaw", false);
    declare_parameter<bool>("use_wrapped_yaw", true);
    declare_parameter<int>("target_id", 71);
    declare_parameter<double>("tag_size", 0.06);
    declare_parameter<bool>("debug", false);
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("tag_offset_yaw", 0.0);
    declare_parameter<double>("tag_offset_x", 0.0);
    declare_parameter<double>("tag_offset_y", 0.0);
    declare_parameter<double>("detection_timeout",0.5);
    declare_parameter<double>("max_orientation_jump", 0.15);
    declare_parameter<double>("orientation_smoothing", 0.3);
    declare_parameter<int>("required_detections", 20);
    declare_parameter<double>("reprojection_error_threshold", 5.0);
    declare_parameter<bool>("enable_epnp", true);
    declare_parameter<double>("motion_threshold", 0.5);
    declare_parameter<int>("pose_smoothing_window", 5);
    declare_parameter<double>("min_pos_limit_default", 0.01);
    declare_parameter<double>("max_pos_limit_default", 0.2);
    declare_parameter<double>("min_yaw_limit_default", 5.0);
    declare_parameter<double>("max_yaw_limit_default", 20.0);
    declare_parameter<double>("k_pos_stationary", 2.0);
    declare_parameter<double>("k_pos_moving", 10.0);
    declare_parameter<double>("k_yaw_stationary", 2.0);
    declare_parameter<double>("k_yaw_moving", 10.0);
    declare_parameter<int>("fallback_buffer_size", 5);

    updateParams();
    last_detection_time_ = now();

    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/dockpilot/front_camera/color/camera_info", qos,
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(cam_info_mutex_);
          cam_info_ = *msg;
          if (!cam_ok_) {
            RCLCPP_INFO(get_logger(), "Camera info received: fx=%f fy=%f cx=%f cy=%f",
                        cam_info_.k[0], cam_info_.k[4], cam_info_.k[2], cam_info_.k[5]);
            cam_ok_ = true;
          }
        });

    detection_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", qos,
        std::bind(&PerceptionNode::detectionCallback, this, std::placeholders::_1));

    active_dock_sub_ = create_subscription<std_msgs::msg::String>(
        "/active_dock_id", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          try {
            target_id_ = std::stoi(msg->data);
            resetState();
          } catch (...) {
            RCLCPP_WARN(get_logger(), "Invalid dock ID received: %s", msg->data.c_str());
          }
        });

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/marker_pose", 10);
    filtered_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/filtered_tag_pose", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                              [this]() { publishFilteredPose(); });

    RCLCPP_INFO(get_logger(), "Perception node initialized. Target ID: %d", target_id_);
  }

private:
  void updateParams() {
    use_arctan_yaw_ = get_parameter("use_arctan_yaw").as_bool();
    use_wrapped_yaw_ = get_parameter("use_wrapped_yaw").as_bool();
    target_id_ = get_parameter("target_id").as_int();
    tag_size_ = get_parameter("tag_size").as_double();
    if (tag_size_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid tag_size: %f, setting to default 0.06", tag_size_);
      tag_size_ = 0.06;
    }
    debug_ = get_parameter("debug").as_bool();
    base_frame_ = get_parameter("base_frame").as_string();
    tag_offset_yaw_ = get_parameter("tag_offset_yaw").as_double();
    tag_offset_x_ = get_parameter("tag_offset_x").as_double();
    tag_offset_y_ = get_parameter("tag_offset_y").as_double();
    detection_timeout_ = get_parameter("detection_timeout").as_double();
    if (detection_timeout_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid detection_timeout: %f, setting to default 20.0", detection_timeout_);
      detection_timeout_ = 20.0;
    }
    max_orientation_jump_ = get_parameter("max_orientation_jump").as_double();
    orientation_smoothing_ = get_parameter("orientation_smoothing").as_double();
    if (orientation_smoothing_ < 0.0 || orientation_smoothing_ > 1.0) {
      RCLCPP_WARN(get_logger(), "Invalid orientation_smoothing: %f, clamping to [0.0, 1.0]", orientation_smoothing_);
      orientation_smoothing_ = std::clamp(orientation_smoothing_, 0.0, 1.0);
    }
    required_detections_ = get_parameter("required_detections").as_int();
    if (required_detections_ <= 0) {
      RCLCPP_WARN(get_logger(), "Invalid required_detections: %d, setting to default 20", required_detections_);
      required_detections_ = 20;
    }
    reprojection_error_threshold_ = get_parameter("reprojection_error_threshold").as_double();
    enable_epnp_ = get_parameter("enable_epnp").as_bool();
    motion_threshold_ = get_parameter("motion_threshold").as_double();
    pose_smoothing_window_ = get_parameter("pose_smoothing_window").as_int();
    min_pos_limit_default_ = get_parameter("min_pos_limit_default").as_double();
    max_pos_limit_default_ = get_parameter("max_pos_limit_default").as_double();
    min_yaw_limit_default_ = get_parameter("min_yaw_limit_default").as_double();
    max_yaw_limit_default_ = get_parameter("max_yaw_limit_default").as_double();
    k_pos_stationary_ = get_parameter("k_pos_stationary").as_double();
    k_pos_moving_ = get_parameter("k_pos_moving").as_double();
    k_yaw_stationary_ = get_parameter("k_yaw_stationary").as_double();
    k_yaw_moving_ = get_parameter("k_yaw_moving").as_double();
    fallback_buffer_size_ = get_parameter("fallback_buffer_size").as_int();
    if (fallback_buffer_size_ <= 0) {
      RCLCPP_WARN(get_logger(), "Invalid fallback_buffer_size: %d, setting to default 5", fallback_buffer_size_);
      fallback_buffer_size_ = 5;
    }
    orientation_offset_.setRPY(0.0, 0.0, DEG2RAD(tag_offset_yaw_));
  }

  void resetState() {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    ekfs_.clear();
    ekf_initialized_.clear();
    prev_yaw_filtered_.clear();
    pose_buffers_.clear();
    fallback_buffers_.clear();
    filtered_pose_history_.clear();
    recentPosJumps.clear();
    recentYawJumps.clear();
    marker_based_speed_ = 0.0;
    RCLCPP_INFO(get_logger(), "Reset state for new active dock ID: %d", target_id_);
  }

  tf2::Quaternion computeFinalOrientation(const geometry_msgs::msg::Pose& p) {
    tf2::Quaternion q_raw;
    tf2::fromMsg(p.orientation, q_raw);
    if (use_arctan_yaw_) {
      double yaw = computeYawFromXY(p.position.x, p.position.y);
      tf2::Quaternion q; q.setRPY(0.0, 0.0, yaw); return q;
    }
    if (use_wrapped_yaw_) {
      double roll, pitch, yaw_pnp;
      tf2::Matrix3x3(q_raw).getRPY(roll, pitch, yaw_pnp);
      double yaw_deg = RAD2DEG(yaw_pnp);
      double wrapped = wrapAngle(yaw_deg);
      if (prev_yaw_filtered_.count(target_id_)) {
        wrapped = (1.0 - orientation_smoothing_) * wrapped + orientation_smoothing_ * prev_yaw_filtered_[target_id_];
      }
      prev_yaw_filtered_[target_id_] = wrapped;
      tf2::Quaternion q; q.setRPY(roll, pitch, DEG2RAD(wrapped)); return q;
    }
    return q_raw;
  }

  geometry_msgs::msg::Pose computeFallbackPose(const std::vector<PoseSample>& buffers, double reference_yaw_rad) {
    if (buffers.empty()) {
      geometry_msgs::msg::Pose default_pose;
      default_pose.position.x = default_pose.position.y = default_pose.position.z = 0.0;
      default_pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
      RCLCPP_WARN(get_logger(), "No buffered poses for fallback, returning default pose");
      return default_pose;
    }

    const double yaw_consistency_weight = 0.5;
    const double yaw_consistency_sigma = 15.0;

    double median_yaw_rad = reference_yaw_rad;

    double total_weight = 0.0;
    double sum_x = 0.0, sum_y = 0.0;
    tf2::Vector3 sum_quat_axis(0, 0, 0);
    double sum_quat_angle = 0.0;
    std::vector<double> weights(buffers.size());
    std::vector<double> reproj_scores(buffers.size());
    std::vector<double> yaw_scores(buffers.size());
    double avg_reproj = 0.0;

    for (size_t i = 0; i < buffers.size(); ++i) {
      const auto& sample = buffers[i];
      double reproj = sample.reprojErr;
      double norm_reproj = std::min(reproj / reprojection_error_threshold_, 1.0);
      reproj_scores[i] = 1.0 - norm_reproj;

      double yaw_rad = getYawFromPose(sample.pose);
      double yaw_deg = RAD2DEG(yaw_rad);
      double d_yaw_deg = wrapAngle(yaw_deg - RAD2DEG(median_yaw_rad));
      yaw_scores[i] = std::exp(-0.5 * std::pow(d_yaw_deg / yaw_consistency_sigma, 2));

      weights[i] = (1.0 - yaw_consistency_weight) * reproj_scores[i] +
                   yaw_consistency_weight * yaw_scores[i];

      sum_x += weights[i] * sample.x;
      sum_y += weights[i] * sample.y;
      avg_reproj += weights[i] * reproj;

      tf2::Quaternion q;
      tf2::fromMsg(sample.pose.orientation, q);
      tf2::Vector3 axis = q.getAxis();
      double angle = q.getAngle();
      sum_quat_axis += weights[i] * angle * axis;
      sum_quat_angle += weights[i] * angle;
      total_weight += weights[i];

      if (debug_) {
        RCLCPP_DEBUG(get_logger(),
                     "Fallback Pose %zu (%s Sol=%d): x=%.3f y=%.3f yaw=%.2f° reproj=%.3f yawDev=%.2f° reproj_score=%.4f yaw_score=%.4f weight=%.4f",
                     i, sample.method.c_str(), sample.solution_index,
                     sample.x, sample.y, yaw_deg, reproj, d_yaw_deg,
                     reproj_scores[i], yaw_scores[i], weights[i]);
      }
    }

    if (total_weight < 1e-6) {
      RCLCPP_WARN(get_logger(), "No valid weights in fallback buffer, using lowest reproj pose");
      auto min_it = std::min_element(buffers.begin(), buffers.end(),
                                     [](const PoseSample& a, const PoseSample& b) {
                                       return a.reprojErr < b.reprojErr;
                                     });
      geometry_msgs::msg::Pose pose = min_it->pose;
      pose.position.z = 0.0;
      avg_reproj_ = min_it->reprojErr;
      RCLCPP_INFO(get_logger(),
                  "Selected fallback pose for tag %d: Method=%s Sol=%d x=%.3f y=%.3f yaw=%.2f° reproj=%.3f",
                  target_id_, min_it->method.c_str(), min_it->solution_index,
                  pose.position.x, pose.position.y, RAD2DEG(getYawFromPose(pose)),
                  min_it->reprojErr);
      return pose;
    }

    geometry_msgs::msg::Pose fallback_pose;
    fallback_pose.position.x = sum_x / total_weight;
    fallback_pose.position.y = sum_y / total_weight;
    fallback_pose.position.z = 0.0;

    tf2::Vector3 avg_axis = sum_quat_axis * (1.0 / sum_quat_angle);
    double avg_angle = sum_quat_angle / total_weight;
    tf2::Quaternion avg_quat(avg_axis, avg_angle);
    avg_quat.normalize();
    fallback_pose.orientation = tf2::toMsg(avg_quat);

    double final_yaw = getYawFromPose(fallback_pose);
    double final_yaw_deg = RAD2DEG(final_yaw);
    double d_yaw_deg = wrapAngle(final_yaw_deg - RAD2DEG(median_yaw_rad));
    avg_reproj_ = avg_reproj / total_weight;

    RCLCPP_INFO(get_logger(),
                "Selected fallback pose for tag %d: x=%.3f y=%.3f yaw=%.2f° avg_reproj=%.3f yawDev=%.2f°",
                target_id_, fallback_pose.position.x, fallback_pose.position.y,
                final_yaw_deg, avg_reproj_, d_yaw_deg);

    return fallback_pose;
  }

  void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    if (!cam_ok_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Camera info not received");
      return;
    }

    rclcpp::Time stamp(msg->header.stamp);
    static rclcpp::Time last_processed_stamp(0, 0, RCL_ROS_TIME);
    if (stamp <= last_processed_stamp) return;
    last_processed_stamp = stamp;

    for (const auto& det : msg->detections) {
      int id = static_cast<int>(det.id);
      if (id != target_id_) continue;

      double dt = (now() - last_detection_time_).seconds();
      if (dt > detection_timeout_) {
        resetState();
        RCLCPP_WARN(get_logger(), "EKF reset for tag %d after %.1f s timeout", id, dt);
      }

      std::lock_guard<std::mutex> lock(ekf_mutex_);
      if (!ekfs_.count(id)) {
        ekfs_.emplace(id, TagEKF());
        RCLCPP_INFO(get_logger(), "Initialized EKF for tag ID: %d", id);
      }
      TagEKF& ekf = ekfs_[id];

      std::vector<cv::Point2f> img;
      for (const auto& p : det.corners) img.emplace_back(p.x, p.y);
      if (img.size() != 4) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Invalid corner count for tag %d: %zu", id, img.size());
        continue;
      }

      const float s = static_cast<float>(tag_size_) / 2.0f;
      std::vector<cv::Point3f> obj{{-s, s, 0.0f}, {s, s, 0.0f}, {s, -s, 0.0f}, {-s, -s, 0.0f}};

      cv::Mat camera_matrix, dist_coeffs;
      {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);
        camera_matrix = (cv::Mat_<double>(3, 3) << cam_info_.k[0], 0, cam_info_.k[2],
                         0, cam_info_.k[4], cam_info_.k[5], 0, 0, 1);
        dist_coeffs = cam_info_.d.empty() ? cv::Mat::zeros(4, 1, CV_64F) : cv::Mat(cam_info_.d);
      }

      std::vector<cv::Mat> rvecs, tvecs;
      std::vector<double> reprojErrs, posJumps, yawJumps;
      std::vector<std::pair<std::string, int>> solution_methods;

      std::vector<std::string> method_names;
      std::vector<int> methods;

      if (enable_epnp_) {
        method_names = {"ITERATIVE", "EPNP", "IPPE", "IPPE_SQUARE"};
        methods = {cv::SOLVEPNP_ITERATIVE, cv::SOLVEPNP_EPNP, cv::SOLVEPNP_IPPE, cv::SOLVEPNP_IPPE_SQUARE};
      } else {
        method_names = {"ITERATIVE", "IPPE", "IPPE_SQUARE"};
        methods = {cv::SOLVEPNP_ITERATIVE, cv::SOLVEPNP_IPPE, cv::SOLVEPNP_IPPE_SQUARE};
      }

      geometry_msgs::msg::Pose prev_pose;
      double filtered_yaw_rad = 0.0;
      if (ekf_initialized_.count(id) && ekf_initialized_[id]) {
        auto stamped = ekf.getPose(base_frame_, stamp);
        prev_pose = stamped.pose.pose;
        filtered_yaw_rad = getYawFromPose(prev_pose);
      } else {
        prev_pose.position.x = prev_pose.position.y = prev_pose.position.z = 0.0;
        prev_pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
      }

      if (debug_) {
        RCLCPP_INFO(get_logger(), "Evaluating PnP solutions for tag %d:", id);
      }

      for (size_t i = 0; i < methods.size(); ++i) {
        int method = methods[i];
        std::vector<cv::Mat> rvecs_method, tvecs_method;
        int nSolutions = cv::solvePnPGeneric(obj, img, camera_matrix, dist_coeffs, rvecs_method, tvecs_method, false, static_cast<cv::SolvePnPMethod>(method));
        for (int sol = 0; sol < nSolutions; ++sol) {
          cv::Mat rvec = rvecs_method[sol];
          cv::Mat tvec = tvecs_method[sol];

          double reprojErr = computeReprojectionError(obj, img, camera_matrix, dist_coeffs, rvec, tvec);
          reprojErrs.push_back(reprojErr);

          geometry_msgs::msg::PoseStamped tag_cam;
          tag_cam.header = msg->header;
          tag_cam.pose.position.x = tvec.at<double>(0) + tag_offset_x_;
          tag_cam.pose.position.y = tvec.at<double>(1) + tag_offset_y_;
          tag_cam.pose.position.z = tvec.at<double>(2);

          cv::Mat rot_mat;
          cv::Rodrigues(rvec, rot_mat);
          tf2::Matrix3x3 mat(
              rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
              rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
              rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));
          tf2::Quaternion q_cam;
          mat.getRotation(q_cam);
          tag_cam.pose.orientation = tf2::toMsg(q_cam);

          geometry_msgs::msg::PoseStamped tag_base;
          geometry_msgs::msg::TransformStamped tf;
          try {
            tf = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, stamp,
                                             tf2::durationFromSec(0.2));
          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed: %s", ex.what());
            tf = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
          }
          tf2::doTransform(tag_cam, tag_base, tf);

          geometry_msgs::msg::Pose unified = tag_base.pose;
          unified.position.z = 0.0;

          double raw_yaw_rad;
          {
            tf2::Quaternion q_raw;
            tf2::fromMsg(unified.orientation, q_raw);
            double roll, pitch, yaw_pnp;
            tf2::Matrix3x3(q_raw).getRPY(roll, pitch, yaw_pnp);
            raw_yaw_rad = yaw_pnp;
          }
          double prev_yaw_rad = getYawFromPose(prev_pose);
          double prev_yaw_deg = RAD2DEG(prev_yaw_rad);
          double raw_yaw_deg = RAD2DEG(raw_yaw_rad);
          double d_yaw_deg = wrapAngle(raw_yaw_deg - prev_yaw_deg);
          double d_yaw_rad = std::abs(DEG2RAD(d_yaw_deg));

          tf2::Quaternion final_q = computeFinalOrientation(unified);
          unified.orientation = tf2::toMsg(orientation_offset_ * final_q);

          double pos_jump = std::hypot(unified.position.x - prev_pose.position.x,
                                       unified.position.y - prev_pose.position.y);
          posJumps.push_back(pos_jump);
          yawJumps.push_back(d_yaw_rad);
          rvecs.push_back(rvec);
          tvecs.push_back(tvec);
          solution_methods.push_back({method_names[i], sol});

          if (!ekf_initialized_[id]) {
            PoseSample sample;
            sample.timestamp = stamp;
            sample.x = unified.position.x;
            sample.y = unified.position.y;
            sample.pose = unified;
            sample.reprojErr = reprojErr;
            sample.method = method_names[i];
            sample.solution_index = sol;
            pose_buffers_[id].push_back(sample);
          }

          if (reprojErr <= 4 * reprojection_error_threshold_) {
            PoseSample sample;
            sample.timestamp = stamp;
            sample.x = unified.position.x;
            sample.y = unified.position.y;
            sample.pose = unified;
            sample.reprojErr = reprojErr;
            sample.method = method_names[i];
            sample.solution_index = sol;
            fallback_buffers_[id].push_back(sample);
            while (fallback_buffers_[id].size() > static_cast<size_t>(fallback_buffer_size_)) {
              fallback_buffers_[id].erase(fallback_buffers_[id].begin());
            }
          }

          if (debug_) {
            RCLCPP_INFO(get_logger(),
                        "  Method=%s Sol=%d x=%.3f y=%.3f yaw=%.2f° reproj=%.3f posJump=%.3f yawJump=%.2f°",
                        method_names[i].c_str(), sol,
                        unified.position.x, unified.position.y,
                        RAD2DEG(getYawFromPose(unified)), reprojErr, pos_jump, RAD2DEG(d_yaw_rad));
          }
        }
      }

      if (rvecs.empty()) {
        RCLCPP_WARN(get_logger(), "No valid PnP solutions for tag %d", id);
        continue;
      }

      updateAdaptiveParams();

      double pos_std = computeStdDev(recentPosJumps);
      double yaw_std = computeStdDev(recentYawJumps);
      double posLimit = ekf_initialized_[id] ?
                        std::clamp(adaptiveParams.basePos + adaptiveParams.kPos * pos_std,
                                   adaptiveParams.minPosLimit, adaptiveParams.maxPosLimit) : 2.0;
      double yawLimit = ekf_initialized_[id] ?
                        std::clamp(adaptiveParams.baseYaw + adaptiveParams.kYaw * yaw_std,
                                   adaptiveParams.minYawLimit, adaptiveParams.maxYawLimit) : 180.0;

      if (debug_) {
        RCLCPP_INFO(get_logger(),
                    "Adaptive: pos=[%.3f, %.3f] yaw=[%.2f, %.2f] (kPos=%.1f, kYaw=%.1f, motion_factor=%.2f, marker_speed=%.3f m/s, meanPos=%.3f, stdPos=%.3f, meanYaw=%.2f°, stdYaw=%.2f°)",
                    adaptiveParams.minPosLimit, adaptiveParams.maxPosLimit,
                    adaptiveParams.minYawLimit, adaptiveParams.maxYawLimit,
                    adaptiveParams.kPos, adaptiveParams.kYaw,
                    std::min(std::max((marker_based_speed_ - motion_threshold_) / motion_threshold_, 0.0), 1.0),
                    marker_based_speed_, adaptiveParams.basePos, pos_std, adaptiveParams.baseYaw, yaw_std);
        RCLCPP_INFO(get_logger(),
                    "Adaptive Limits: posLimit=%.3f (min=%.3f, max=%.3f), yawLimit=%.2f° (min=%.2f°, max=%.2f°), meanPos=%.3f, stdPos=%.3f, meanYaw=%.2f°, stdYaw=%.2f°)",
                    posLimit, adaptiveParams.minPosLimit, adaptiveParams.maxPosLimit,
                    yawLimit, adaptiveParams.minYawLimit, adaptiveParams.maxYawLimit,
                    adaptiveParams.basePos, pos_std, adaptiveParams.baseYaw, yaw_std);
      }

      std::vector<size_t> validIdx;
      for (size_t i = 0; i < posJumps.size(); ++i) {
        if (posJumps[i] <= posLimit && RAD2DEG(yawJumps[i]) <= yawLimit) {
          validIdx.push_back(i);
          if (debug_) {
            RCLCPP_DEBUG(get_logger(),
                         "Solution %zu (%s Sol=%d) accepted: posJump=%.3f (limit=%.3f), yawJump=%.2f° (limit=%.2f°)",
                         i, solution_methods[i].first.c_str(), solution_methods[i].second,
                         posJumps[i], posLimit, RAD2DEG(yawJumps[i]), yawLimit);
          }
        } else {
          if (debug_) {
            RCLCPP_DEBUG(get_logger(),
                         "Solution %zu (%s Sol=%d) rejected: posJump=%.3f (limit=%.3f), yawJump=%.2f° (limit=%.2f°)",
                         i, solution_methods[i].first.c_str(), solution_methods[i].second,
                         posJumps[i], posLimit, RAD2DEG(yawJumps[i]), yawLimit);
          }
        }
      }

      bool skip_update = false;
      geometry_msgs::msg::Pose selectedPose;
      size_t selectedIdx = -1;
      if (validIdx.empty()) {
        std::vector<double> yaws;
        for (size_t i = 0; i < rvecs.size(); ++i) {
          if (reprojErrs[i] > 4 * reprojection_error_threshold_) continue;
          geometry_msgs::msg::Pose pose = computePoseFromIdx(i, rvecs, tvecs, msg->header);
          pose.position.x += tag_offset_x_;
          pose.position.y += tag_offset_y_;
          pose.position.z = 0.0;
          pose.orientation = tf2::toMsg(orientation_offset_ * computeFinalOrientation(pose));
          yaws.push_back(getYawFromPose(pose));
        }
        if (!yaws.empty()) {
          std::sort(yaws.begin(), yaws.end());
          filtered_yaw_rad = yaws[yaws.size() / 2];
        }

        selectedPose = computeFallbackPose(fallback_buffers_[id], filtered_yaw_rad);
        if (selectedPose.position.x == 0.0 && selectedPose.position.y == 0.0 &&
            selectedPose.orientation.w == 1.0) {
          RCLCPP_WARN(get_logger(), "No valid fallback pose; skipping EKF update");
          skip_update = true;
        } else {
          RCLCPP_WARN(get_logger(), "All solutions rejected; using intelligent fallback for tag %d", id);
            if (ekf_initialized_[id]) {
            double fallback_yaw_rad = getYawFromPose(selectedPose);
            double prev_yaw_rad = getYawFromPose(prev_pose);
            double prev_yaw_deg = RAD2DEG(prev_yaw_rad);
            double fallback_yaw_deg = RAD2DEG(fallback_yaw_rad);
            double d_yaw_deg = wrapAngle(fallback_yaw_deg - prev_yaw_deg);
            selectedYawJump = std::abs(DEG2RAD(d_yaw_deg));
            selectedPosJump = std::hypot(selectedPose.position.x - prev_pose.position.x,
                                         selectedPose.position.y - prev_pose.position.y);
            // Set noise covariance for fallback pose (slightly higher than stationary)
            double pos_noise = marker_based_speed_ > motion_threshold_ ? 0.05 : 0.01; // Increased for fallback
            double yaw_noise = marker_based_speed_ > motion_threshold_ ? DEG2RAD(5.0) : DEG2RAD(1.0); // Increased for fallback
            ekf.setProcessNoiseCovariance(pos_noise, yaw_noise);
            // Update EKF with fallback pose
            ekf.update(selectedPose, stamp);
            RCLCPP_INFO(get_logger(), "EKF updated with fallback pose for tag %d: x=%.3f y=%.3f yaw=%.2f° posJump=%.3f yawJump=%.2f° posNoise=%.3f yawNoise=%.2f°",
                        id, selectedPose.position.x, selectedPose.position.y,
                        fallback_yaw_deg, selectedPosJump, d_yaw_deg, pos_noise, RAD2DEG(yaw_noise));
        }
        }
      } else {
        std::vector<double> scores(validIdx.size());
        for (size_t i = 0; i < validIdx.size(); ++i) {
          size_t idx = validIdx[i];
          double norm_reproj = std::min(reprojErrs[idx] / reprojection_error_threshold_, 1.0);
          double score_reproj = 1.0 - norm_reproj;
          double norm_yaw_jump = std::min(RAD2DEG(yawJumps[idx]) / max_yaw_limit_default_, 1.0);
          double score_yaw_jump = 1.0 - norm_yaw_jump;
          double norm_pos_jump = std::min(posJumps[idx] / max_pos_limit_default_, 1.0);
          double score_pos_jump = 1.0 - norm_pos_jump;
          scores[i] = 0.5 * score_reproj + 0.5 * score_yaw_jump + 0.0 * score_pos_jump;
          if (debug_) {
            RCLCPP_DEBUG(get_logger(),
                         "Solution %zu (%s Sol=%d): score=%.4f (reproj=%.4f, yaw_jump=%.4f, pos_jump=%.4f)",
                         idx, solution_methods[idx].first.c_str(), solution_methods[i].second,
                         scores[i], score_reproj, score_yaw_jump, score_pos_jump);
          }
        }

        auto bestIt = std::max_element(scores.begin(), scores.end());
        selectedIdx = validIdx[std::distance(scores.begin(), bestIt)];
        selectedPose = computePoseFromIdx(selectedIdx, rvecs, tvecs, msg->header);
        selectedPose.position.x += tag_offset_x_;
        selectedPose.position.y += tag_offset_y_;
        selectedPose.position.z = 0.0;
        selectedPose.orientation = tf2::toMsg(orientation_offset_ * computeFinalOrientation(selectedPose));
      }

      if (skip_update) continue;

      geometry_msgs::msg::PoseStamped raw_pose;
      raw_pose.header = msg->header;
      raw_pose.header.frame_id = base_frame_;
      raw_pose.pose = selectedPose;
      pose_pub_->publish(raw_pose);

      if (selectedIdx != static_cast<size_t>(-1)) {
        RCLCPP_INFO(get_logger(),
                    "[SELECTED] ID=%d Method=%s Sol=%d x=%.3f y=%.3f yaw=%.2f° reproj=%.3f posJump=%.3f yawJump=%.2f°",
                    id, solution_methods[selectedIdx].first.c_str(), solution_methods[selectedIdx].second,
                    selectedPose.position.x, selectedPose.position.y,
                    RAD2DEG(getYawFromPose(selectedPose)),
                    reprojErrs[selectedIdx], posJumps[selectedIdx], RAD2DEG(yawJumps[selectedIdx]));
      } else {
        RCLCPP_INFO(get_logger(),
                    "[SELECTED] ID=%d Method=FALLBACK Sol=0 x=%.3f y=%.3f yaw=%.2f° reproj=%.3f posJump=%.3f yawJump=%.2f°",
                    id, selectedPose.position.x, selectedPose.position.y,
                    RAD2DEG(getYawFromPose(selectedPose)),
                    avg_reproj_, 0.0, 0.0);
      }

      PoseSample new_sample{msg->header.stamp, selectedPose.position.x, selectedPose.position.y, selectedPose,
                           selectedIdx != static_cast<size_t>(-1) ? reprojErrs[selectedIdx] : avg_reproj_,
                           selectedIdx != static_cast<size_t>(-1) ? solution_methods[selectedIdx].first : "FALLBACK",
                           selectedIdx != static_cast<size_t>(-1) ? solution_methods[selectedIdx].second : 0};
      {
        std::lock_guard<std::mutex> hist_lock(history_mutex_);
        pose_history_.push_back(new_sample);
        while (pose_history_.size() > static_cast<size_t>(pose_smoothing_window_)) {
          pose_history_.pop_front();
        }
      }

      updateHistoricalData(selectedIdx != static_cast<size_t>(-1) ? posJumps[selectedIdx] : 0.0,
                           selectedIdx != static_cast<size_t>(-1) ? yawJumps[selectedIdx] : 0.0);
      double pos_noise = marker_based_speed_ > motion_threshold_ ? 0.05 : 0.005;
      double yaw_noise = marker_based_speed_ > motion_threshold_ ? DEG2RAD(5.0) : DEG2RAD(0.5);
      ekf.setProcessNoiseCovariance(pos_noise, yaw_noise);
      //RCLCPP_INFO(get_logger(), "Updated process noise: pos=%.6f, yaw=%.6f", pos_noise, yaw_noise);

      if (!ekf_initialized_[id]) {
        if (pose_buffers_[id].size() >= static_cast<size_t>(required_detections_)) {
          geometry_msgs::msg::Pose initial_pose = computeInitialPose(pose_buffers_[id]);
          ekf.update(initial_pose, stamp);
          ekf_initialized_[id] = true;
          pose_buffers_[id].clear();
          RCLCPP_INFO(get_logger(), "EKF initialized intelligently for tag %d using %d buffered detections", id, required_detections_);
        } else {
          RCLCPP_INFO(get_logger(), "Buffering detection %zu/%d for tag %d", pose_buffers_[id].size(), required_detections_, id);
          last_detection_time_ = now();
          continue;
        }
      }
      ekf.update(selectedPose, stamp);
      last_detection_time_ = now();
    }
  }

  void publishFilteredPose() {
    double dt = (now() - last_detection_time_).seconds();
    if (dt > detection_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "No detection for %.1f s – stopping EKF output", dt);
      return;
    }

    std::lock_guard<std::mutex> lock(ekf_mutex_);
    if (!ekfs_.count(target_id_)) return;
    if (!ekf_initialized_.count(target_id_) || !ekf_initialized_[target_id_]) return;

    auto stamped_pose = ekfs_[target_id_].getPose(base_frame_, now());
    if (stamped_pose.header.stamp.sec == 0 && stamped_pose.header.stamp.nanosec == 0) return;

    geometry_msgs::msg::PoseStamped filtered_msg;
    filtered_msg.header = stamped_pose.header;
    filtered_msg.pose = stamped_pose.pose.pose;
    filtered_pose_pub_->publish(filtered_msg);

    // Store filtered pose for speed calculation
    {
      std::lock_guard<std::mutex> hist_lock(history_mutex_);
      PoseSample sample;
      sample.timestamp = stamped_pose.header.stamp;
      sample.x = stamped_pose.pose.pose.position.x;
      sample.y = stamped_pose.pose.pose.position.y;
      sample.pose = stamped_pose.pose.pose;
      sample.reprojErr = 0.0; // Not used for filtered poses
      sample.method = "FILTERED";
      sample.solution_index = 0;
      filtered_pose_history_.push_back(sample);
      while (filtered_pose_history_.size() > static_cast<size_t>(pose_smoothing_window_)) {
        filtered_pose_history_.pop_front();
      }

      // Compute speed using filtered poses
      if (filtered_pose_history_.size() >= 2) {
        double total_speed = 0.0;
        size_t valid_pairs = 0;
        for (size_t i = 1; i < filtered_pose_history_.size(); ++i) {
          double dt = (filtered_pose_history_[i].timestamp - filtered_pose_history_[i-1].timestamp).seconds();
          if (dt <= 0.0 || dt > 0.5) continue;
          double dx = filtered_pose_history_[i].x - filtered_pose_history_[i-1].x;
          double dy = filtered_pose_history_[i].y - filtered_pose_history_[i-1].y;
          double distance = std::hypot(dx, dy);
          double speed = distance / dt;
          total_speed += speed;
          valid_pairs++;
        }
        marker_based_speed_ = valid_pairs > 0 ? total_speed / valid_pairs : 0.0;
      } else {
        marker_based_speed_ = 0.0;
      }

      if (debug_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Marker-based speed (filtered): %.3f m/s", marker_based_speed_);
      }
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header = stamped_pose.header;
    tf.child_frame_id = "tag_" + std::to_string(target_id_);
    tf.transform.translation.x = stamped_pose.pose.pose.position.x;
    tf.transform.translation.y = stamped_pose.pose.pose.position.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = stamped_pose.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    if (debug_) {
      RCLCPP_INFO(get_logger(),
                  "[FILTER] x=%.3f y=%.3f yaw=%.2f°",
                  stamped_pose.pose.pose.position.x,
                  stamped_pose.pose.pose.position.y,
                  RAD2DEG(getYawFromPose(stamped_pose.pose.pose)));
    }
  }

  void updateHistoricalData(double posJump, double yawJump) {
    double t = now().seconds();
    recentPosJumps.emplace_back(t, posJump);
    recentYawJumps.emplace_back(t, yawJump);
    while (recentPosJumps.size() > maxHistorySize) recentPosJumps.pop_front();
    while (recentYawJumps.size() > maxHistorySize) recentYawJumps.pop_front();
    while (!recentPosJumps.empty() && t - recentPosJumps.front().first > 5.0) recentPosJumps.pop_front();
    while (!recentYawJumps.empty() && t - recentYawJumps.front().first > 5.0) recentYawJumps.pop_front();
  }

  void updateAdaptiveParams() {
    double meanPos = recentPosJumps.empty() ? 0.02 : computeMean(recentPosJumps);
    double meanYaw = recentYawJumps.empty() ? 1.5 : computeMean(recentYawJumps);
    double stdPos = recentPosJumps.size() < 2 ? 0.005 : computeStdDev(recentPosJumps);
    double stdYaw = recentYawJumps.size() < 2 ? 0.5 : computeStdDev(recentYawJumps);

    adaptiveParams.basePos = meanPos;
    adaptiveParams.baseYaw = RAD2DEG(meanYaw);

    double motion_factor = std::min(std::max((marker_based_speed_ - motion_threshold_) / motion_threshold_, 0.0), 1.0);
    adaptiveParams.kPos = k_pos_stationary_ + (k_pos_moving_ - k_pos_stationary_) * motion_factor;
    adaptiveParams.kYaw = k_yaw_stationary_ + (k_yaw_moving_ - k_yaw_stationary_) * motion_factor;

    adaptiveParams.minPosLimit = min_pos_limit_default_;
    adaptiveParams.maxPosLimit = max_pos_limit_default_;
    adaptiveParams.minYawLimit = min_yaw_limit_default_;
    adaptiveParams.maxYawLimit = max_yaw_limit_default_;

    if (debug_) {
      RCLCPP_INFO(get_logger(),
                  "Adaptive: pos=[%.3f, %.3f] yaw=[%.2f, %.2f] (kPos=%.1f, kYaw=%.1f, motion_factor=%.2f, marker_speed=%.3f m/s, meanPos=%.3f, stdPos=%.3f, meanYaw=%.2f°, stdYaw=%.2f°)",
                  adaptiveParams.minPosLimit, adaptiveParams.maxPosLimit,
                  adaptiveParams.minYawLimit, adaptiveParams.maxYawLimit,
                  adaptiveParams.kPos, adaptiveParams.kYaw, motion_factor, marker_based_speed_,
                  meanPos, stdPos, RAD2DEG(meanYaw), RAD2DEG(stdYaw));
    }
  }

  double computeMean(const std::deque<std::pair<double, double>>& data) {
    if (data.empty()) return 0.0;
    double sum = 0.0;
    for (const auto& v : data) sum += v.second;
    return sum / data.size();
  }

  double computeStdDev(const std::deque<std::pair<double, double>>& data) {
    if (data.size() < 2) return 0.0;
    double mean = computeMean(data);
    double var = 0.0;
    for (const auto& v : data) var += std::pow(v.second - mean, 2);
    return std::sqrt(var / (data.size() - 1));
  }

  geometry_msgs::msg::Pose computePoseFromIdx(size_t idx,
                                              const std::vector<cv::Mat>& rvecs,
                                              const std::vector<cv::Mat>& tvecs,
                                              const std_msgs::msg::Header& header) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = tvecs[idx].at<double>(0);
    pose.position.y = tvecs[idx].at<double>(1);
    pose.position.z = tvecs[idx].at<double>(2);

    cv::Mat rot_mat;
    cv::Rodrigues(rvecs[idx], rot_mat);
    tf2::Matrix3x3 mat(
        rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
        rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
        rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));
    tf2::Quaternion q;
    mat.getRotation(q);
    pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseStamped pose_cam, pose_base;
    pose_cam.header = header;
    pose_cam.pose = pose;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, header.frame_id,
                                       header.stamp, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed: %s", ex.what());
      tf = tf_buffer_->lookupTransform(base_frame_, header.frame_id, tf2::TimePointZero);
    }
    tf2::doTransform(pose_cam, pose_base, tf);
    return pose_base.pose;
  }

  double computeReprojectionError(const std::vector<cv::Point3f>& objPoints,
                                 const std::vector<cv::Point2f>& imgPoints,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs,
                                 const cv::Mat& rvec,
                                 const cv::Mat& tvec) {
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(objPoints, rvec, tvec, cameraMatrix, distCoeffs, reprojected);
    double err = 0.0;
    for (size_t i = 0; i < imgPoints.size(); ++i) {
      cv::Point2f d = imgPoints[i] - reprojected[i];
      err += std::sqrt(d.x * d.x + d.y * d.y);
    }
    return err / imgPoints.size();
  }

  double getYawFromPose(const geometry_msgs::msg::Pose& pose) {
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    if (std::isnan(yaw)) {
      RCLCPP_WARN(get_logger(), "Invalid yaw in pose, returning 0.0");
      return 0.0;
    }
    return yaw;
  }

    geometry_msgs::msg::Pose computeInitialPose(const std::vector<PoseSample>& buffers) {
    if (buffers.empty()) {
      geometry_msgs::msg::Pose default_pose;
      default_pose.position.x = default_pose.position.y = default_pose.position.z = 0.0;
      default_pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
      RCLCPP_WARN(get_logger(), "No buffered poses for initialization, returning default pose");
      return default_pose;
    }

    // Local weighting parameters
    const double yaw_consistency_weight = 0.5; // Equal balance between reproj and yaw
    const double yaw_consistency_sigma = 15.0; // Gaussian sigma for yaw deviation (degrees)

    // Compute median yaw for robustness against outliers
    std::vector<double> yaws;
    for (const auto& sample : buffers) {
      if (sample.reprojErr > 4 * reprojection_error_threshold_) continue;
      yaws.push_back(getYawFromPose(sample.pose));
    }
    if (yaws.empty()) {
      RCLCPP_WARN(get_logger(), "No valid poses for yaw calculation, using lowest reproj pose");
      auto min_it = std::min_element(buffers.begin(), buffers.end(),
                                     [](const PoseSample& a, const PoseSample& b) {
                                       return a.reprojErr < b.reprojErr;
                                     });
      geometry_msgs::msg::Pose pose = min_it->pose;
      pose.position.z = 0.0;
      RCLCPP_INFO(get_logger(),
                  "Selected initial pose for tag %d: Method=%s Sol=%d x=%.3f y=%.3f yaw=%.2f° reproj=%.3f",
                  target_id_, min_it->method.c_str(), min_it->solution_index,
                  pose.position.x, pose.position.y, RAD2DEG(getYawFromPose(pose)),
                  min_it->reprojErr);
      return pose;
    }
    std::sort(yaws.begin(), yaws.end());
    double median_yaw_rad = yaws[yaws.size() / 2];

    // Compute weighted average pose
    double total_weight = 0.0;
    double sum_x = 0.0, sum_y = 0.0;
    tf2::Vector3 sum_quat_axis(0, 0, 0);
    double sum_quat_angle = 0.0;
    std::vector<double> weights(buffers.size());
    std::vector<double> reproj_scores(buffers.size());
    std::vector<double> yaw_scores(buffers.size());
    double avg_reproj = 0.0;

    for (size_t i = 0; i < buffers.size(); ++i) {
      const auto& sample = buffers[i];
      double reproj = sample.reprojErr;
      double norm_reproj = std::min(reproj / reprojection_error_threshold_, 1.0);
      reproj_scores[i] = 1.0 - norm_reproj;

      double yaw_rad = getYawFromPose(sample.pose);
      double yaw_deg = RAD2DEG(yaw_rad);
      double d_yaw_deg = wrapAngle(yaw_deg - RAD2DEG(median_yaw_rad));
      yaw_scores[i] = std::exp(-0.5 * std::pow(d_yaw_deg / yaw_consistency_sigma, 2));

      weights[i] = (1.0 - yaw_consistency_weight) * reproj_scores[i] +
                   yaw_consistency_weight * yaw_scores[i];

      sum_x += weights[i] * sample.x;
      sum_y += weights[i] * sample.y;
      avg_reproj += weights[i] * reproj;

      tf2::Quaternion q;
      tf2::fromMsg(sample.pose.orientation, q);
      tf2::Vector3 axis = q.getAxis();
      double angle = q.getAngle();
      sum_quat_axis += weights[i] * angle * axis;
      sum_quat_angle += weights[i] * angle;
      total_weight += weights[i];

      if (debug_) {
        RCLCPP_DEBUG(get_logger(),
                     "Init Pose %zu (%s Sol=%d): x=%.3f y=%.3f yaw=%.2f° reproj=%.3f yawDev=%.2f° reproj_score=%.4f yaw_score=%.4f weight=%.4f",
                     i, sample.method.c_str(), sample.solution_index,
                     sample.x, sample.y, yaw_deg, reproj, d_yaw_deg,
                     reproj_scores[i], yaw_scores[i], weights[i]);
      }
    }

    if (total_weight < 1e-6) {
      RCLCPP_WARN(get_logger(), "No valid weights, using lowest reproj pose");
      auto min_it = std::min_element(buffers.begin(), buffers.end(),
                                     [](const PoseSample& a, const PoseSample& b) {
                                       return a.reprojErr < b.reprojErr;
                                     });
      geometry_msgs::msg::Pose pose = min_it->pose;
      pose.position.z = 0.0;
      RCLCPP_INFO(get_logger(),
                  "Selected initial pose for tag %d: Method=%s Sol=%d x=%.3f y=%.3f yaw=%.2f° reproj=%.3f",
                  target_id_, min_it->method.c_str(), min_it->solution_index,
                  pose.position.x, pose.position.y, RAD2DEG(getYawFromPose(pose)),
                  min_it->reprojErr);
      return pose;
    }

    geometry_msgs::msg::Pose init_pose;
    init_pose.position.x = sum_x / total_weight;
    init_pose.position.y = sum_y / total_weight;
    init_pose.position.z = 0.0;

    tf2::Vector3 avg_axis = sum_quat_axis * (1.0 / sum_quat_angle);
    double avg_angle = sum_quat_angle / total_weight;
    tf2::Quaternion avg_quat(avg_axis, avg_angle);
    avg_quat.normalize();
    init_pose.orientation = tf2::toMsg(avg_quat);

    double final_yaw = getYawFromPose(init_pose);
    double final_yaw_deg = RAD2DEG(final_yaw);
    double d_yaw_deg = wrapAngle(final_yaw_deg - RAD2DEG(median_yaw_rad));
    avg_reproj /= total_weight;

    RCLCPP_INFO(get_logger(),
                "Selected initial pose for tag %d: x=%.3f y=%.3f yaw=%.2f° avg_reproj=%.3f yawDev=%.2f°",
                target_id_, init_pose.position.x, init_pose.position.y,
                final_yaw_deg, avg_reproj, d_yaw_deg);

    return init_pose;
  }

  bool use_arctan_yaw_ = false;
  bool use_wrapped_yaw_ = true;
  bool debug_ = true;
  bool enable_epnp_ = true;
  int target_id_ = 71;
  int required_detections_ = 20;
  int pose_smoothing_window_ = 5;
  int fallback_buffer_size_ = 5;
  double tag_size_ = 0.06;
  double tag_offset_yaw_ = 0.0;
  double tag_offset_x_ = 0.0;
  double tag_offset_y_ = 0.0;
  double detection_timeout_ = 20.0;
  double max_orientation_jump_ = 0.15;
  double orientation_smoothing_ = 0.3;
  double reprojection_error_threshold_ = 5.0;
  double motion_threshold_ = 0.05;
  double min_pos_limit_default_ = 0.01;
  double max_pos_limit_default_ = 0.2;
  double min_yaw_limit_default_ = 5.0;
  double max_yaw_limit_default_ = 20.0;
  double k_pos_stationary_ = 2.0;
  double k_pos_moving_ = 10.0;
  double k_yaw_stationary_ = 2.0;
  double k_yaw_moving_ = 10.0;
  double marker_based_speed_ = 0.0;
  double avg_reproj_ = 0.0;
  std::deque<PoseSample> pose_history_;
  std::deque<PoseSample> filtered_pose_history_;
  std::string base_frame_ = "base_link";
  tf2::Quaternion orientation_offset_;
   double selectedYawJump;  // To store the yaw jump for the selected pose
    double selectedPosJump;  // To store the position jump for the selected pose

  std::unordered_map<int, TagEKF> ekfs_;
  std::unordered_map<int, bool> ekf_initialized_;
  std::unordered_map<int, double> prev_yaw_filtered_;
  std::unordered_map<int, std::vector<PoseSample>> pose_buffers_;
  std::unordered_map<int, std::vector<PoseSample>> fallback_buffers_;

  sensor_msgs::msg::CameraInfo cam_info_;
  bool cam_ok_ = false;
  std::mutex cam_info_mutex_;
  std::mutex ekf_mutex_;
  std::mutex history_mutex_;

  rclcpp::Time last_detection_time_;
  std::deque<std::pair<double, double>> recentPosJumps;
  std::deque<std::pair<double, double>> recentYawJumps;
  const size_t maxHistorySize = 20;
  AdaptiveParams adaptiveParams;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_dock_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PerceptionNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_node"), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}