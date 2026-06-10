# DockPilot Perception ROS2 Package

## Overview

The `dockpilot_perception` package is a ROS2-based system for real-time detection and pose estimation of AprilTags using camera inputs. It processes raw camera images to detect AprilTags (tag36h11 family), computes 3D poses relative to the robot's base frame, and applies an Extended Kalman Filter (EKF) for robust pose estimation. This package is ideal for robotics applications like autonomous docking, navigation, or object tracking requiring precise marker-based localization.

### Key Features
- **AprilTag Detection**: Uses the AprilTag library for efficient tag detection.
- **Pose Estimation**: Computes 3D poses with reprojection error handling and multiple solvers (PnP, EPNP).
- **EKF Filtering**: Smooths noisy measurements with adaptive noise parameters.
- **Dynamic Target Switching**: Supports runtime changes to target tag ID via a topic.
- **Timeout and Fallback**: Handles detection timeouts with buffered pose fallback.
- **TF Broadcasting**: Publishes tag pose transforms.
- **Configurable**: Extensive parameters for tuning to specific environments.

## Dependencies

- **ROS2**: jazzy or later recommended.
- **ROS2 Packages**: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `std_msgs`.
- **External Libraries**:
  - AprilTag C library (`apriltag`)
  - OpenCV (`opencv2`)
  - Eigen (for EKF matrix operations)
  - `cv_bridge`
- **Custom Messages**: `apriltag_msgs` (install separately or include as dependency).
- **System Requirements**: Ubuntu 20.04 or 22.04, C++17 compiler, CMake 3.16+.

## Build and Installation Instructions

### Prerequisites
1.  ROS2:

2. Install dependencies:
   ```bash
   sudo apt install libopencv-dev libeigen3-dev ros-$distro-cv-bridge
   ```
3. Install AprilTag library:
   ```bash
   sudo apt install libapriltag-dev
   ```
   Or build from source:
   ```bash
   git clone https://github.com/AprilRobotics/apriltag.git
   cd apriltag
   mkdir build && cd build
   cmake .. && make -j$(nproc)
   sudo make install
   ```
4. Ensure `apriltag_msgs` is available. If not in your ROS distro, clone and build:
   ```bash
   git clone https://github.com/christian-mattos/apriltag_msgs.git ~/ros2_ws/src/apriltag_msgs
   ```

### Building the Package

Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select dockpilot_perception
   ```
 Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### Installation
- After building, the package is installed in `~/ros2_ws/install/dockpilot_perception`.
- No additional installation steps are required beyond sourcing the workspace.

## Usage

### Launching
1. Ensure your camera is publishing to:
   - `/dockpilot/front_camera/color/image_raw` (`sensor_msgs/msg/Image`)
   - `/dockpilot/front_camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
2. Launch the package:
   ```bash
   ros2 launch dockpilot_perception perception.launch.py
   ```
3. Monitor outputs:
   - **Detections**: `/detections` (`apriltag_msgs/msg/AprilTagDetectionArray`)
   - **Raw Pose**: `/marker_pose` (`geometry_msgs/msg/PoseStamped`)
   - **Filtered Pose**: `/filtered_tag_pose` (`geometry_msgs/msg/PoseStamped`)
   - **Annotated Image** (optional): `/apriltag_annotated_image` (`sensor_msgs/msg/Image`)
   - **TF Transforms**: Tag pose relative to `base_link` (configurable).

### Configuration
- Parameters are defined in `dockpilot_perception/config/perception.yaml`.
- Key parameters include:
  - `target_id` (default: 71): Target AprilTag ID.
  - `tag_size` (default: 0.06): Tag size in meters.
  - `debug` (default: false): Enable verbose logging.
  - `detection_timeout` (default: 2.0): Timeout for fallback.
  - See `perception.yaml` for full list and defaults.
- Edit parameters:
   ```bash
   nano ~/ros2_ws/src/dockpilot_perception/config/perception.yaml
   ```
- Dynamically change target ID:
   ```bash
   ros2 topic pub /active_dock_id std_msgs/msg/String "data: '72'"
   ```

### Debugging
- Enable `debug: true` in `perception.yaml` for detailed logs.
- Visualize annotated images in RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```
   Add an `Image` display for `/apriltag_annotated_image`.
- Monitor TF in RViz or via:
   ```bash
   ros2 run tf2_tools view_frames
   ```

## Nodes

### AprilTag Detector Node (`apriltag_detector_node`)
- Detects AprilTags in images using the tag36h11 family.
- Publishes detections and optional annotated images.
- Tuned for robustness with Gaussian blur and edge refinement.

### Perception Node (`perception_node`)
- Computes 3D poses from detections using PnP/EPNP solvers.
- Filters poses with EKF, adaptive noise, and fallback mechanisms.
- Supports dynamic target switching and TF broadcasting.

## Limitations
- Single-camera support (multi-camera could be added).
- Assumes planar tags (z=0); extendable for 3D.
- Limited to one active target ID at a time.
- Performance may require tuning for high-speed motion.

