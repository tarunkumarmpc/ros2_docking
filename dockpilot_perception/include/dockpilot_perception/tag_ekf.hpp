#ifndef MOTION_PERCEPTION__TAG_EKF_HPP_
#define MOTION_PERCEPTION__TAG_EKF_HPP_

#include <cmath>
#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <rclcpp/rclcpp.hpp>

class TagEKF {
public:
  TagEKF(double process_noise_pos = 0.05,      // Increased for more trust in motion model
       double process_noise_vel = 0.1,       // Increased for velocity
       double process_noise_yaw = 0.05,      // Increased for yaw
       double measurement_noise_pos = 0.1,    // Reduced for more trust in measurements
       double measurement_noise_yaw = 0.2,   // Reduced for yaw
       double vel_smoothing_factor = 0.2,    // Increased for faster velocity response
       double chi_squared_threshold = 10.0,  //
         bool   use_velocity_model   = false)
    : process_noise_pos_(process_noise_pos),
      process_noise_vel_(process_noise_vel),
      process_noise_yaw_(process_noise_yaw),
      measurement_noise_pos_(measurement_noise_pos),
      measurement_noise_yaw_(measurement_noise_yaw),
      vel_smoothing_factor_(vel_smoothing_factor),
      chi_squared_threshold_(chi_squared_threshold),
      use_velocity_model_(use_velocity_model) {

    x_.setZero();
    P_.setIdentity();
    P_ *= 1.0;

    Q_.setZero();
    Q_(0, 0) = process_noise_pos_;  // x
    Q_(1, 1) = process_noise_pos_;  // y
    Q_(2, 2) = process_noise_pos_;  // z
    Q_(3, 3) = process_noise_yaw_;  // yaw
    Q_(4, 4) = process_noise_vel_;  // vx
    Q_(5, 5) = process_noise_vel_;  // vy

    R_.setZero();
    R_(0, 0) = measurement_noise_pos_;  // x
    R_(1, 1) = measurement_noise_pos_;  // y
    R_(2, 2) = measurement_noise_pos_;  // z
    R_(3, 3) = measurement_noise_yaw_;  // yaw

    last_vel_.setZero();
    initialized_ = false;
    last_msg_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
void setProcessNoiseCovariance(double pos_noise, double yaw_noise) {
    process_noise_pos_ = pos_noise;
    process_noise_yaw_ = yaw_noise;
    Q_(0, 0) = process_noise_pos_;  // x
    Q_(1, 1) = process_noise_pos_;  // y
    Q_(2, 2) = process_noise_pos_;  // z
    Q_(3, 3) = process_noise_yaw_;  // yaw
   
  }
  /* ---------- runtime switch ---------- */
  void setUseVelocityModel(bool on) noexcept { use_velocity_model_ = on; }
  bool useVelocityModel() const noexcept      { return use_velocity_model_; }

  bool initialized() const { return initialized_; }

  void reset() {
    initialized_ = false;
    x_.setZero();
    P_.setIdentity();
    P_ *= 1.0;
    last_vel_.setZero();
    last_msg_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_INFO(rclcpp::get_logger("TagEKF"), "Filter reset.");
  }

  /* ---------- dynamic predict ---------- */
  void predict(double vx, double vy, double w, const rclcpp::Time &msg_stamp) {
    if (!initialized_) {
      last_msg_stamp_ = msg_stamp;
      RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
                  "Filter not initialized, skipping prediction.");
      return;
    }

    double dt = (msg_stamp - last_msg_stamp_).seconds();
    last_msg_stamp_ = msg_stamp;

    if (dt <= 0.0 || std::isnan(dt)) {

RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
            "Non-positive dt %.3f s, skipping predict", dt);
      return;
    }
    if (dt > 0.5) {
      RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
                  "Large time step: %.3f s, capping at 0.1 s.", dt);
      dt = 0.1;
    }

    /* velocity smoothing */
    if (std::isnan(vx) || std::isnan(vy) || std::isnan(w)) {
      vx = last_vel_(0);
      vy = last_vel_(1);
      w  = last_vel_(2);
    } else {
      last_vel_(0) = vel_smoothing_factor_ * vx + (1.0 - vel_smoothing_factor_) * last_vel_(0);
      last_vel_(1) = vel_smoothing_factor_ * vy + (1.0 - vel_smoothing_factor_) * last_vel_(1);
      last_vel_(2) = vel_smoothing_factor_ * w  + (1.0 - vel_smoothing_factor_) * last_vel_(2);
    }

    /* skip if velocity model disabled */
    if (!use_velocity_model_) {
      Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
      P_ = F * P_ * F.transpose() + Q_ * dt;
      RCLCPP_DEBUG(rclcpp::get_logger("TagEKF"), "Static prediction (velocity model OFF)");
      return;
    }

    /* stationary check */
    const double kStationaryThresh = 0.01;
    if (std::abs(last_vel_(0)) < kStationaryThresh &&
        std::abs(last_vel_(1)) < kStationaryThresh &&
        std::abs(last_vel_(2)) < kStationaryThresh) {
      RCLCPP_DEBUG(rclcpp::get_logger("TagEKF"), "Stationary: skipping dynamic prediction");
      return;
    }

    /* motion model */
    const double th  = x_(3);
    const double cth = std::cos(th);
    const double sth = std::sin(th);

    x_(0) += (last_vel_(0) * cth - last_vel_(1) * sth) * dt;
    x_(1) += (last_vel_(0) * sth + last_vel_(1) * cth) * dt;
    x_(3) += last_vel_(2) * dt;
    x_(3) = std::remainder(x_(3), 2.0 * M_PI);

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = (-last_vel_(0) * sth - last_vel_(1) * cth) * dt;
    F(1, 3) = ( last_vel_(0) * cth - last_vel_(1) * sth) * dt;
    F(0, 4) =  cth * dt;
    F(1, 4) =  sth * dt;
    F(0, 5) = -sth * dt;
    F(1, 5) =  cth * dt;
    F(3, 5) =  dt;

    P_ = F * P_ * F.transpose() + Q_ * dt;
    RCLCPP_DEBUG(rclcpp::get_logger("TagEKF"), "Predicted state: x=%f, y=%f, yaw=%f",
                 x_(0), x_(1), x_(3));
  }

  /* ---------- static predict (no velocity) ---------- */
  void predict(const rclcpp::Time &msg_stamp) {
    if (!initialized_) {
      last_msg_stamp_ = msg_stamp;
      RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
                  "Filter not initialized, skipping static prediction.");
      return;
    }

    double dt = (msg_stamp - last_msg_stamp_).seconds();
    last_msg_stamp_ = msg_stamp;

    if (dt <= 0.0 || std::isnan(dt)) {
//RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
           // "Non-positive dt %.3f s, skipping static predict", dt);
      return;
    }
    if (dt > 0.5) {
      RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
                  "Large time step: %.3f s, capping at 0.1 s.", dt);
      dt = 0.1;
    }

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    P_ = F * P_ * F.transpose() + Q_ * dt;
    RCLCPP_DEBUG(rclcpp::get_logger("TagEKF"), "Static prediction (velocity model OFF)");
  }

  /* ---------- measurement update ---------- */
  void update(const geometry_msgs::msg::Pose &pose, const rclcpp::Time &msg_stamp) {
    if (std::isnan(pose.position.x) || std::isnan(pose.position.y) ||
        std::isnan(pose.position.z) || std::isnan(pose.orientation.w)) {
      RCLCPP_WARN(rclcpp::get_logger("TagEKF"), "Invalid pose measurement, skipping update.");
      return;
    }

    if (!initialized_) {
      x_.setZero();
      x_(0) = pose.position.x;
      x_(1) = pose.position.y;
      x_(2) = pose.position.z;
      tf2::Quaternion q;
      tf2::fromMsg(pose.orientation, q);
      x_(3) = yawFromQuat(q);
      P_.setIdentity();
      P_ *= 1.0;
      initialized_ = true;
      last_msg_stamp_ = msg_stamp;
      RCLCPP_INFO(rclcpp::get_logger("TagEKF"),
                  "Filter initialized with x=%f, y=%f, z=%f, yaw=%f",
                  x_(0), x_(1), x_(2), x_(3));
      return;
    }

    predict(msg_stamp);

    Eigen::Vector4d z;
    z(0) = pose.position.x;
    z(1) = pose.position.y;
    z(2) = pose.position.z;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    z(3) = yawFromQuat(q);

    Eigen::Vector4d y = z - x_.head<4>();
    y(3) = std::remainder(y(3), 2.0 * M_PI);

    Eigen::Matrix<double, 4, 6> H = Eigen::Matrix<double, 4, 6>::Zero();
    H(0, 0) = 1.0; H(1, 1) = 1.0; H(2, 2) = 1.0; H(3, 3) = 1.0;

    Eigen::Matrix4d S = H * P_ * H.transpose() + R_;
    double d2 = y.transpose() * S.inverse() * y;
    if (d2 > chi_squared_threshold_) {
      RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
                  "Measurement rejected, chi-squared: %f > %f", d2, chi_squared_threshold_);
      last_msg_stamp_ = msg_stamp;
      return;
    }

    Eigen::Matrix<double, 6, 4> K = P_ * H.transpose() * S.inverse();
    x_ += K * y;
    x_(3) = std::remainder(x_(3), 2.0 * M_PI);
    P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P_;
    last_msg_stamp_ = msg_stamp;

    RCLCPP_DEBUG(rclcpp::get_logger("TagEKF"),
                 "Updated state: x=%f, y=%f, yaw=%f, chi-squared=%f",
                 x_(0), x_(1), x_(3), d2);
  }

  /* ---------- pose getter ---------- */
  geometry_msgs::msg::PoseWithCovarianceStamped
  getPose(const std::string &frame_id, const rclcpp::Time &query_time) const {
    geometry_msgs::msg::PoseWithCovarianceStamped p;

    if (!initialized_) {
      RCLCPP_WARN(rclcpp::get_logger("TagEKF"),
                  "Filter not initialized, returning empty pose.");
      return p;
    }

    if (query_time.nanoseconds() > 0 && query_time >= last_msg_stamp_ - rclcpp::Duration::from_seconds(0.1)) {
      p.header.stamp = query_time;
    } else {
      p.header.stamp = last_msg_stamp_;
    }
    p.header.frame_id = frame_id;

    p.pose.pose.position.x = x_(0);
    p.pose.pose.position.y = x_(1);
    p.pose.pose.position.z = x_(2);

    tf2::Quaternion q;
    q.setRPY(0, 0, x_(3));
    p.pose.pose.orientation = tf2::toMsg(q);

    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        p.pose.covariance[i * 6 + j] = P_(i, j);

    return p;
  }

private:
  static double yawFromQuat(const tf2::Quaternion &q) {
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    if (std::isnan(y)) {
      RCLCPP_ERROR(rclcpp::get_logger("TagEKF"), "Invalid yaw from quaternion, returning 0.0");
      return 0.0;
    }
    return y;
  }

  Eigen::Matrix<double, 6, 1> x_;        // [x y z yaw vx vy]
  Eigen::Matrix<double, 6, 6> P_, Q_;
  Eigen::Matrix4d R_;
  Eigen::Vector3d last_vel_;             // [vx vy w]
  rclcpp::Time last_msg_stamp_{0, 0, RCL_ROS_TIME};
  bool initialized_{false};

  /* tunable parameters */
  double process_noise_pos_;
  double process_noise_vel_;
  double process_noise_yaw_;
  double measurement_noise_pos_;
  double measurement_noise_yaw_;
  double vel_smoothing_factor_;
  double chi_squared_threshold_;
  bool   use_velocity_model_{false};
};

#endif  // MOTION_PERCEPTION__TAG_EKF_HPP_