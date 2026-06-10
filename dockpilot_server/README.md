# Docking Server for ROS 2

This repository contains a ROS 2 node (`docking_server`) that implements a 3-stage docking system with strict fine-mode entry for a robot, achieving precise docking within 10 cm of a target dock. The node supports docking and undocking actions using marker-based pose estimation.

## Overview

The `docking_server` node, part of the `dockpilot_server` package, provides:
- **Docking Action**: Aligns the robot in Y, yaw, and X directions, with a strict fine-mode for precision within 10 cm.
- **Undocking Action**: Moves the robot to a predefined undock pose.
- **Configuration**: Loads dock and undock poses from a YAML file.
- **Diagnostics**: Publishes error, tolerance, and stage information.

## Dependencies

- **ROS 2**: Humble, Iron, or Rolling.
- **Packages**: `rclcpp`, `rclcpp_action`, `geometry_msgs`, `std_msgs`, `diagnostic_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `yaml-cpp`.
- **Custom Interfaces**: `dockpilot_interfaces` package with `Dock` and `Undock` actions.
- **System**: C++17, CMake, ament_cmake.

## Build Instructions

1. **Clone the Repository**:
  
2. **Install Dependencies**:

   sudo apt install ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-rclcpp-action \
                    ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-std-msgs \
                    ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-tf2 \
                    ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-geometry-msgs \
                    libyaml-cpp-dev
   ```

3. **Build the Package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select dockpilot_server
   source install/setup.bash
   ```

## Run Instructions

1. **Configure the Node**:
   - Ensure `docks_server.yaml` and `dock_configs.yaml` are in `dockpilot_server/config/`.
   - Example `dock_configs.yaml`:
     ```yaml
     docks:
       dock1:
         dock_pose:
           header:
             frame_id: "camera_link"
           pose:
             position:
               x: 1.0
               y: 0.0
               z: 0.0
             yaw: 90.0
         undock_pose:
           header:
             frame_id: "camera_link"
           pose:
             position:
               x: -0.5
               y: 0.0
               z: 0.0
             yaw: 90.0
     default_dock:
       dock_pose:
         header:
           frame_id: "base_link"
         pose:
           position:
             x: 0.10
             y: 0.0
             z: 0.0
           orientation:
             w: 1.0
       undock_pose:
         header:
           frame_id: "base_link"
         pose:
           position:
             x: -0.30
             y: 0.0
             z: 0.0
           orientation:
             w: 1.0
     ```
   - Example `docks_server.yaml`:
     ```yaml
     docking_server:
       ros__parameters:
         final_x_tolerance: 0.003
         final_y_tolerance: 0.003
         final_yaw_tolerance: 0.003
         dock_timeout: 60.0
         undock_timeout: 30.0
         base_frame: "base_link"
         dwell_time: 1.0
         fine_align_distance: 0.20
         hysteresis_y: 0.0005
         hysteresis_yaw: 0.005
         yaw_drift_threshold: 0.005
     ```

2. **Launch the Node**:
   Use the associated launch file (e.g., `docking_server_launch.py`):
   ```bash
   ros2 launch dockpilot_server docking_server_launch.py
   ```

3. **Trigger Docking**:
   ```bash
   ros2 action send_goal /dock dockpilot_interfaces/action/Dock "{dock_id: 'dock1'}"
   ```

4. **Trigger Undocking**:
   ```bash
   ros2 action send_goal /undock dockpilot_interfaces/action/Undock "{dock_id: 'dock1'}"
   ```

5. **Monitor Diagnostics**:
   ```bash
   ros2 topic echo /diagnostics
   ```

## Notes

- Ensure a TF tree is available (e.g., `base_link` to `camera_link`).
- The node expects `/filtered_tag_pose` for marker-based localization.
- The `dockpilot_interfaces` package must define `Dock` and `Undock` actions.
