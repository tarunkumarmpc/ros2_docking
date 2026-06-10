#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include <opencv2/imgproc.hpp>

extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
}

class AprilTagDetectorNode : public rclcpp::Node
{
public:
    AprilTagDetectorNode()
    : Node("apriltag_detector_node"),
      cam_info_received_(false)
    {
        // Subscriptions
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/dockpilot/front_camera/color/image_raw", 10,
            std::bind(&AprilTagDetectorNode::imageCallback, this, std::placeholders::_1));
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/dockpilot/front_camera/color/camera_info", 10,
            std::bind(&AprilTagDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

        // Publishers
        detection_array_pub_ = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
            "detections", 10);
        annotated_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "apriltag_annotated_image", 10);

        // Initialize AprilTag detector
        tf_family_ = tag36h11_create();
        if (!tf_family_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create tag36h11 family");
            throw std::runtime_error("Failed to create tag36h11 family");
        }

        detector_ = apriltag_detector_create();
        if (!detector_) {
            tag36h11_destroy(tf_family_);
            RCLCPP_ERROR(this->get_logger(), "Failed to create AprilTag detector");
            throw std::runtime_error("Failed to create AprilTag detector");
        }

        apriltag_detector_add_family(detector_, tf_family_);

        // Detector tuning for robustness
        detector_->quad_decimate = 1.0;  // Downsample for better detection
        detector_->quad_sigma = 0.8;    // Gaussian blur to reduce noise
        detector_->nthreads = 1;
        detector_->refine_edges = 1;

        RCLCPP_INFO(this->get_logger(), "AprilTag detector node started");
    }

    ~AprilTagDetectorNode() override
    {
        if (detector_) {
            apriltag_detector_destroy(detector_);
        }
        if (tf_family_) {
            tag36h11_destroy(tf_family_);
        }
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (cam_info_received_) {
            return;
        }

        if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0 || msg->k[2] < 0.0 || msg->k[5] < 0.0 ||
            std::isnan(msg->k[0]) || std::isnan(msg->k[4]) || std::isnan(msg->k[2]) || std::isnan(msg->k[5])) {
            RCLCPP_ERROR(this->get_logger(), "Invalid camera info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                         msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
            return;
        }

        try {
            cam_model_.fromCameraInfo(msg);
            cam_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera info received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                        cam_model_.fx(), cam_model_.fy(), cam_model_.cx(), cam_model_.cy());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process camera info: %s", e.what());
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!cam_info_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for camera info - skipping image processing.");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Received image: encoding=%s, width=%u, height=%u",
                     msg->encoding.c_str(), msg->width, msg->height);

        // Convert image to mono8
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            if (msg->encoding == "mono8") {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Converting image from %s to mono8", msg->encoding.c_str());
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Validate image
        if (!cv_ptr || cv_ptr->image.empty() || !cv_ptr->image.data || cv_ptr->image.cols <= 0 || cv_ptr->image.rows <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid or empty image: cv_ptr=%s, empty=%d, data=%p, cols=%d, rows=%d",
                         cv_ptr ? "valid" : "null", cv_ptr->image.empty(), static_cast<void*>(cv_ptr->image.data),
                         cv_ptr->image.cols, cv_ptr->image.rows);
            return;
        }

        // Validate image dimensions against camera info
        if (cv_ptr->image.cols != static_cast<int>(cam_model_.fullResolution().width) ||
            cv_ptr->image.rows != static_cast<int>(cam_model_.fullResolution().height)) {
            RCLCPP_WARN(this->get_logger(), "Image dimensions (%d x %d) do not match camera info (%d x %d)",
                        cv_ptr->image.cols, cv_ptr->image.rows,
                        cam_model_.fullResolution().width, cam_model_.fullResolution().height);
        }

        // Prepare AprilTag image struct
        image_u8_t apriltag_img = {
            static_cast<int32_t>(cv_ptr->image.cols),
            static_cast<int32_t>(cv_ptr->image.rows),
            static_cast<int32_t>(cv_ptr->image.cols),
            cv_ptr->image.data
        };

        RCLCPP_DEBUG(this->get_logger(), "AprilTag image prepared: width=%d, height=%d, stride=%d",
                     apriltag_img.width, apriltag_img.height, apriltag_img.stride);

        // Run detection
        zarray_t *detections = nullptr;
        try {
            RCLCPP_DEBUG(this->get_logger(), "Starting AprilTag detection");
            detections = apriltag_detector_detect(detector_, &apriltag_img);
            if (!detections) {
                RCLCPP_ERROR(this->get_logger(), "AprilTag detection returned null");
                return;
            }
            RCLCPP_DEBUG(this->get_logger(), "Detected %d AprilTags", zarray_size(detections));
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "AprilTag detection failed: %s", e.what());
            return;
        }

        // Prepare annotated image if needed
        cv::Mat annotated_img;
        bool need_annotated = annotated_img_pub_->get_subscription_count() > 0;
        if (need_annotated) {
            try {
                RCLCPP_DEBUG(this->get_logger(), "Converting image to BGR for annotation");
                cv::cvtColor(cv_ptr->image, annotated_img, cv::COLOR_GRAY2BGR);
            } catch (const cv::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "OpenCV color conversion failed: %s", e.what());
                apriltag_detections_destroy(detections);
                return;
            }
        }

        // Prepare detection array
        apriltag_msgs::msg::AprilTagDetectionArray detection_array;
        detection_array.header = msg->header;
        detection_array.header.frame_id = msg->header.frame_id;

        // Process detections
        for (int i = 0; i < zarray_size(detections); ++i) {
            apriltag_detection_t *det = nullptr;
            zarray_get(detections, i, &det);
            if (!det || !det->H) {
                RCLCPP_ERROR(this->get_logger(), "Invalid detection at index %d: det=%p, H=%p",
                             i, static_cast<void*>(det), det ? static_cast<void*>(det->H) : nullptr);
                continue;
            }

            // Validate corner points
            bool valid_corners = true;
            std::vector<cv::Point2f> image_points;
            for (int j = 0; j < 4; ++j) {
                double x = det->p[j][0];
                double y = det->p[j][1];
                if (std::isnan(x) || std::isnan(y) ||
                    x <= 1.0 || x >= cv_ptr->image.cols - 1.0 ||
                    y <= 1.0 || y >= cv_ptr->image.rows - 1.0) {
                    RCLCPP_WARN(this->get_logger(), "Invalid corner point %d for tag %d: x=%.2f, y=%.2f",
                                j, det->id, x, y);
                    valid_corners = false;
                    break;
                }
                image_points.emplace_back(x, y);
            }
            if (!valid_corners || image_points.size() != 4) {
                RCLCPP_WARN(this->get_logger(), "Skipping tag %d due to invalid corners", det->id);
                continue;
            }

            // Log corner points
            RCLCPP_DEBUG(this->get_logger(), "Tag %d corners: [0]=(%.2f,%.2f), [1]=(%.2f,%.2f), [2]=(%.2f,%.2f), [3]=(%.2f,%.2f)",
                         det->id,
                         det->p[0][0], det->p[0][1],
                         det->p[1][0], det->p[1][1],
                         det->p[2][0], det->p[2][1],
                         det->p[3][0], det->p[3][1]);

            // Validate center point
            if (std::isnan(det->c[0]) || std::isnan(det->c[1]) ||
                det->c[0] <= 1.0 || det->c[0] >= cv_ptr->image.cols - 1.0 ||
                det->c[1] <= 1.0 || det->c[1] >= cv_ptr->image.rows - 1.0) {
                RCLCPP_WARN(this->get_logger(), "Invalid center point for tag %d: x=%.2f, y=%.2f",
                            det->id, det->c[0], det->c[1]);
                continue;
            }

            // Validate homography matrix
            if (det->H->nrows != 3 || det->H->ncols != 3) {
                RCLCPP_WARN(this->get_logger(), "Invalid homography matrix for tag %d: nrows=%d, ncols=%d",
                            det->id, det->H->nrows, det->H->ncols);
                continue;
            }
            bool valid_homography = true;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    double value = matd_get(det->H, r, c);
                    if (std::isnan(value) || std::isinf(value)) {
                        RCLCPP_WARN(this->get_logger(), "Invalid homography element (%d,%d) for tag %d: value=%.2f",
                                    r, c, det->id, value);
                        valid_homography = false;
                        break;
                    }
                }
                if (!valid_homography) break;
            }
            if (!valid_homography) {
                continue;
            }

            RCLCPP_DEBUG(this->get_logger(), "Processing tag ID %d, decision_margin=%.2f", det->id, det->decision_margin);

            // Create detection message
            apriltag_msgs::msg::AprilTagDetection detection;
            detection.family = "tag36h11";
            detection.id = det->id;
            detection.hamming = det->hamming;
            detection.goodness = 0.0; // Not available in apriltag_detection_t
            detection.decision_margin = det->decision_margin;
            detection.centre.x = det->c[0];
            detection.centre.y = det->c[1];
            for (int j = 0; j < 4; ++j) {
                detection.corners[j].x = det->p[j][0];
                detection.corners[j].y = det->p[j][1];
            }
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    detection.homography[r * 3 + c] = matd_get(det->H, r, c);
                }
            }

            detection_array.detections.push_back(detection);

            // Draw annotations if needed
            if (need_annotated) {
                RCLCPP_DEBUG(this->get_logger(), "Drawing annotations for tag %d", det->id);
                drawDetection(annotated_img, det);
            }
        }

        // Publish detection array
        if (!detection_array.detections.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Publishing %zu AprilTag detections", detection_array.detections.size());
            detection_array_pub_->publish(detection_array);
        }

        // Publish annotated image
        if (need_annotated && !annotated_img.empty()) {
            try {
                RCLCPP_DEBUG(this->get_logger(), "Publishing annotated image");
                auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated_img).toImageMsg();
                annotated_img_pub_->publish(*out_msg);
            } catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to publish annotated image: %s", e.what());
            }
        }

        // Clean up detections
        RCLCPP_DEBUG(this->get_logger(), "Cleaning up detections");
        apriltag_detections_destroy(detections);
    }

    void drawDetection(cv::Mat &img, apriltag_detection_t *det)
    {
        if (!det) {
            RCLCPP_ERROR(this->get_logger(), "Null detection in drawDetection");
            return;
        }

        for (int i = 0; i < 4; i++) {
            double x = det->p[i][0];
            double y = det->p[i][1];
            if (std::isnan(x) || std::isnan(y) ||
                x <= 1.0 || x >= img.cols - 1.0 ||
                y <= 1.0 || y >= img.rows - 1.0) {
                RCLCPP_WARN(this->get_logger(), "Invalid corner point %d for tag %d: x=%.2f, y=%.2f",
                            i, det->id, x, y);
                continue;
            }
            cv::line(img,
                     cv::Point(static_cast<int>(x), static_cast<int>(y)),
                     cv::Point(static_cast<int>(det->p[(i + 1) % 4][0]), static_cast<int>(det->p[(i + 1) % 4][1])),
                     cv::Scalar(0, 255, 0), 2);
        }

        std::string id_str = std::to_string(det->id);
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 0.5;
        int thickness = 2;

        cv::Size text_size = cv::getTextSize(id_str, font, scale, thickness, nullptr);
        cv::Point text_org(static_cast<int>(det->p[0][0]), static_cast<int>(det->p[0][1] - 5));
        if (text_org.y < 0) {
            text_org.y = static_cast<int>(det->p[0][1] + text_size.height + 5);
        }
        if (text_org.x >= 0 && text_org.y >= 0 && text_org.x < img.cols && text_org.y < img.rows) {
            cv::putText(img, id_str, text_org, font, scale, cv::Scalar(0, 0, 255), thickness);
        } else {
            RCLCPP_WARN(this->get_logger(), "Text position out of bounds for tag %d: x=%d, y=%d",
                        det->id, text_org.x, text_org.y);
        }
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_array_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_img_pub_;

    image_geometry::PinholeCameraModel cam_model_;
    bool cam_info_received_;

    apriltag_family_t *tf_family_;
    apriltag_detector_t *detector_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<AprilTagDetectorNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("apriltag_detector_node"), "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}