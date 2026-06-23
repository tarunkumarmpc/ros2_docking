
**Autonomous Docking Stack for ROS 2**

DockPilot is a  ROS 2 stack for autonomous robot docking. It provides full-stack docking capabilities — from visual fiducial detection and pose estimation through to closed-loop velocity control — all built on standard ROS 2 interfaces and designed  and tested in  real-world deployment.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron%20%7C%20Jazzy-blue)](https://docs.ros.org)
[![Build](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/tarunkumarmpc/ros2_docking)
[![GitHub Stars](https://img.shields.io/github/stars/tarunkumarmpc/ros2_docking?style=social)](https://github.com/tarunkumarmpc/ros2_docking/stargazers)
[![GitHub Issues](https://img.shields.io/github/issues/tarunkumarmpc/ros2_docking)](https://github.com/tarunkumarmpc/ros2_docking/issues)

---

## Overview

DockPilot provides a complete autonomous docking pipeline:

```
Camera → AprilTag Detection → EKF Pose Estimation → Docking Server → Velocity Controller → /cmd_vel
```

### Packages

| Package | Description |
|---|---|
| [`dockpilot_interfaces`](dockpilot_interfaces/) | ROS 2 action definitions: `Dock` and `Undock` |
| [`dockpilot_perception`](dockpilot_perception/) | AprilTag detector (tag36h11) + EKF-based pose estimator |
| [`dockpilot_server`](dockpilot_server/) | High-level docking action server (state machine, multi-dock config) |
| [`dockpilot_control`](dockpilot_control/) | Velocity controller: PID, Pure Pursuit, LQR, MPC |

---

## Features

**AprilTag-based docking** — tag36h11 family, robust detection at range
**EKF pose fusion** — smooth, noise-robust 6-DOF marker pose estimation
**Multi-dock support** — configure multiple named docking stations in YAML
**Pluggable controllers** — swap between PID, Pure Pursuit, LQR, or MPC at launch
**Safety features** — slew-rate limiting, timeout watchdogs, graceful abort
**Standard interfaces** — all communication via standard `geometry_msgs` and `nav_msgs`
**Configurable** — all parameters exposed via ROS 2 parameter server

---

## Requirements

| Dependency | Version |
|---|---|
| ROS 2 | Humble / Iron / Jazzy |
| OpenCV | ≥ 4.x |
| apriltag (C library) | ≥ 3.x |
| apriltag_msgs | ROS 2 package |
| Eigen3 | ≥ 3.3 |
| yaml-cpp | ≥ 0.7 |

Install system dependencies:
```bash
sudo apt-get install libapriltag-dev libeigen3-dev libyaml-cpp-dev
```

Install ROS 2 dependencies:
```bash
sudo apt-get install ros-$ROS_DISTRO-apriltag-msgs \
                     ros-$ROS_DISTRO-cv-bridge \
                     ros-$ROS_DISTRO-image-geometry \
                     ros-$ROS_DISTRO-tf2-ros \
                     ros-$ROS_DISTRO-tf2-geometry-msgs
```

---

## Installation

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/tarunkumarmpc/ros2_docking.git

# Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build (interfaces must be built first)
colcon build --packages-select \
  dockpilot_interfaces \
  dockpilot_perception \
  dockpilot_control \
  dockpilot_server

# Source
source install/setup.bash
```

---

## Quick Start

### 1. Configure your docking stations

Edit [`dockpilot_server/config/dock_configs.yaml`](dockpilot_server/config/dock_configs.yaml) to define your dock locations and AprilTag IDs.

### 2. Launch perception

```bash
ros2 launch dockpilot_perception perception.launch.py
```

Ensure your camera publishes to:
- `/dockpilot/front_camera/color/image_raw`
- `/dockpilot/front_camera/color/camera_info`

### 3. Launch the docking server

```bash
ros2 launch dockpilot_server docking_server.launch.py
```

### 4. Launch the velocity controller

```bash
ros2 launch dockpilot_control control.launch.py controller_type:=pid
```

Available controller types: `pid`, `pure_pursuit`, `lqr`, `mpc`

### 5. Send a dock action goal

```bash
ros2 action send_goal /dock dockpilot_interfaces/action/Dock "{dock_id: 'dock1'}"
```

### 6. Send an undock action goal

```bash
ros2 action send_goal /undock dockpilot_interfaces/action/Undock "{dock_id: 'dock1'}"
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                       DockPilot Stack                    │
│                                                         │
│  ┌──────────────────┐    ┌──────────────────────────┐  │
│  │ dockpilot_       │    │   dockpilot_server        │  │
│  │ perception       │───▶│                           │  │
│  │                  │    │  /dock  (action server)   │  │
│  │ AprilTag detect  │    │  /undock (action server)  │  │
│  │ + EKF estimator  │    │  State machine            │  │
│  └──────────────────┘    └──────────┬───────────────┘  │
│                                     │                   │
│                          ┌──────────▼───────────────┐  │
│                          │   dockpilot_control       │  │
│                          │                           │  │
│                          │  PID / Pure Pursuit /     │  │
│                          │  LQR / MPC                │  │
│                          │  ──▶ /cmd_vel             │  │
│                          └───────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## ROS 2 Topics & Actions

### Subscribed Topics

| Topic | Type | Node |
|---|---|---|
| `/dockpilot/front_camera/color/image_raw` | `sensor_msgs/Image` | `dockpilot_perception` |
| `/dockpilot/front_camera/color/camera_info` | `sensor_msgs/CameraInfo` | `dockpilot_perception` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `dockpilot_control` |
| `/marker_pose` | `geometry_msgs/PoseStamped` | `dockpilot_control` |

### Published Topics

| Topic | Type | Node |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `dockpilot_control` |
| `detections` | `apriltag_msgs/AprilTagDetectionArray` | `dockpilot_perception` |
| `apriltag_annotated_image` | `sensor_msgs/Image` | `dockpilot_perception` |

### Actions

| Action | Type | Description |
|---|---|---|
| `/dock` | `dockpilot_interfaces/action/Dock` | Command robot to dock at a named station |
| `/undock` | `dockpilot_interfaces/action/Undock` | Command robot to undock from a station |

---

## Configuration

All parameters are exposed via YAML config files and the ROS 2 parameter server.

- **Docking stations**: `dockpilot_server/config/dock_configs.yaml`
- **Server settings**: `dockpilot_server/config/docks_server.yaml`
- **Controller gains**: `dockpilot_control/config/control_params.yaml`
- **Perception settings**: `dockpilot_perception/config/perception.yaml`

---

## Controller Selection

DockPilot ships with four interchangeable velocity controllers:

| Controller | Best For | Notes |
|---|---|---|
| `pid` | General purpose, easy to tune | Default, recommended for most users |
| `pure_pursuit` | Smooth curved approach paths | Good for non-holonomic robots |
| `lqr` | Optimal linear control, fast approach | Requires accurate robot model |
| `mpc` | Constrained optimal control, mecanum | Computationally heavier |

Switch at launch time:
```bash
ros2 launch dockpilot_control control.launch.py controller_type:=lqr
```

---

## Contributing

Contributions are welcome! Please open issues and pull requests on GitHub.

Please open a [GitHub Issue](https://github.com/tarunkumarmpc/ros2_docking/issues) to report bugs or request features.

---

## License

This project is licensed under the **Apache License 2.0** — see the [LICENSE](LICENSE) file for details.

---

## Acknowledgements
Thank you to ROS2 ecosystem 
---
