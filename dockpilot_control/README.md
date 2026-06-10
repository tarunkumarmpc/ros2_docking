# DockPilot Control ROS 2 Package

## Overview

The `dockpilot_control` package is a ROS 2 node designed to control a robot's motion by computing velocity commands (`geometry_msgs/Twist`) to drive the robot toward a specified goal pose. The node supports multiple control algorithms (PID, Pure Pursuit, LQR, MPC), with the PID controller set as the default. It subscribes to goal and marker poses, calculates position and orientation errors, and publishes velocity commands to the `/cmd_vel` topic. The package uses TF2 for coordinate frame transformations and includes safety features to ensure robust operation.

## Features

- **Modular Controller Interface**: Supports PID (default), Pure Pursuit, LQR, and MPC controllers, selectable via the `controller_type` ROS parameter.
- **Pose Tracking**: Tracks a marker pose (e.g., from a filtered tag) and aligns the robot with a goal pose.
- **PID Controller Features**:
  - **Gain Scheduling**: Dynamically adjusts PID gains based on error magnitude for improved performance across large and small errors.
  - **Low-Pass Filtering**: Smooths position and yaw errors to reduce noise sensitivity.
  - **Slew Rate Limiting**: Limits the rate of change of velocity commands to ensure smooth motion.
  - **Mecanum Wheel Kinematics**: Computes wheel speeds for mecanum drive robots based on desired linear and angular velocities.
  - Configurable PID gains, velocity limits, and other parameters via ROS parameters.
- **Safety Mechanisms**:
  - Stops the robot if the marker pose is stale (configurable timeout).
  - Clamps linear and angular velocities to safe limits.
  - Checks for invalid yaw or NaN values in computations.
- **TF2 Integration**: Transforms marker poses to the robot's base frame for accurate error calculation.
- **Configurable Parameters**: Controller type, base frame, marker timeout, and PID-specific parameters are configurable via a YAML file.

## Dependencies

- **ROS 2**: Tested with ROS 2 Humble or later.
- **C++ Libraries**:
  - `rclcpp`: ROS 2 C++ client library.
  - `geometry_msgs`: For `Twist` and `PoseStamped` messages.
  - `std_msgs`: For `Bool` messages.
  - `tf2`, `tf2_ros`, `tf2_geometry_msgs`: For coordinate frame transformations.
  - `algorithm`, `cmath`, `iostream`: For PID controller utilities.
- **Build Tools**:
  - `colcon`: For building the ROS 2 workspace.
  - `ament_cmake`: For CMake-based ROS 2 package configuration.
- **Custom Headers**:
  - `dockpilot_control/controller_interface.hpp`
  - `dockpilot_control/pid_controller.hpp`
  - `dockpilot_control/pure_pursuit_controller.hpp`
  - `dockpilot_control/mpc_controller.hpp`
  - `dockpilot_control/lqr_controller.hpp`

Ensure these dependencies are installed and available in your ROS 2 workspace.

## Installation



1. **Build the Workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select dockpilot_control
   ```

2. **Source the Workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Configuration

The node uses a YAML configuration file (`control_params.yaml`) located in the `config` directory of the package. Below is an example configuration including PID-specific parameters:

```yaml
control_node:
  ros__parameters:
    # General parameters
    controller_type: "pid"  # Options: pid (default), pure_pursuit, lqr, mpc
    base_frame: "base_link"
    marker_timeout_sec: 1.0
    # PID controller parameters
    pid:
      kp_x: 1.0
      ki_x: 0.0
      kd_x: 0.5
      kp_y: 1.0
      ki_y: 0.0
      kd_y: 0.5
      kp_yaw: 2.0
      ki_yaw: 0.0
      kd_yaw: 0.5
      large_error_thr: 0.18
      small_error_thr: 0.07
      kp_large_mult: 1.4
      kd_large_mult: 1.9
      ki_large_mult: 0.4
      kp_small_mult: 0.6
      kd_small_mult: 0.5
      ki_small_mult: 0.7
      max_linear_x: 0.1
      max_linear_y: 0.1
      max_angular: 0.07
      max_delta_v: 0.05
      max_delta_omega: 0.07
      lowpass_alpha: 0.13
    # Robot geometry (used by PID and LQR controllers)
    lqr:
      wheel_radius: 0.05
      lx: 0.305
      ly: 0.161
```

Place the configuration file in `<package_share_directory>/config/control_params.yaml`. Adjust PID gains and other parameters based on your robot's dynamics and requirements.

## Launching the Node

The provided launch file (`control.launch.py`) starts the `control_node` with configurable parameters and topic remappings. To launch the node:

```bash
ros2 launch dockpilot_control control.launch.py controller_type:=pid
```

### Launch File Details

- **File**: `control.launch.py`
- **Parameters**:
  - `controller_type` (default: `pid`): Specifies the control algorithm (`pid`, `pure_pursuit`, `lqr`, `mpc`).
  - Loads additional parameters from `config/control_params.yaml`.
- **Topic Remappings**:
  - `/goal_pose`: Subscribes to the goal pose (`geometry_msgs/PoseStamped`).
  - `/marker_pose`: Subscribes to the marker pose (`geometry_msgs/PoseStamped`).
  - `/docking_active`: Subscribes to the activation flag (`std_msgs/Bool`).
  - `/cmd_vel`: Publishes velocity commands (`geometry_msgs/Twist`).

### Example Launch Commands

- To use the default PID controller:
  ```bash
  ros2 launch dockpilot_control control.launch.py controller_type:=pid
  ```

- To use the Pure Pursuit controller:
  ```bash
  ros2 launch dockpilot_control control.launch.py controller_type:=pure_pursuit
  ```

## Usage

1. **Ensure Dependencies Are Running**:
   - A TF2 transform tree must be available (e.g., from a robot localization node).
   - Publish marker poses to `/marker_pose` (e.g., from a vision system).
   - Publish goal poses to `/goal_pose`.
   - Publish the activation flag to `/docking_active` (`true` to enable control, `false` to stop).

2. **Monitor Output**:
   - Velocity commands are published to `/cmd_vel`.
   - Use `ros2 topic echo /cmd_vel` to inspect commands.
   - The PID controller prints debug information (errors, gains, velocities, wheel speeds) to the console.
   - Check ROS logs for additional debugging information.

3. **Visualize in RViz**:
   - Add `PoseStamped` displays for `/goal_pose` and `/marker_pose`.
   - Add a `Twist` display for `/cmd_vel` to visualize velocity commands.

## PID Controller Details

The `PIDController` class implements a PID control algorithm with the following features:

- **Error Calculation**:
  - Computes errors in x (forward/backward), y (left/right), and yaw (orientation) from the input `geometry_msgs/Pose`.
  - Applies a low-pass filter to smooth noisy inputs (`lowpass_alpha` parameter).
- **Gain Scheduling**:
  - Dynamically adjusts PID gains (`kp`, `ki`, `kd`) based on error magnitude using `small_error_thr` and `large_error_thr`.
  - Uses multipliers (`kp_small_mult`, `kp_large_mult`, etc.) to scale gains for small and large errors.
- **Slew Rate Limiting**:
  - Limits the rate of change of velocity commands (`max_delta_v`, `max_delta_omega`) to ensure smooth motion.
- **Mecanum Wheel Kinematics**:
  - Converts linear (`vx`, `vy`) and angular (`omega`) velocities to individual wheel speeds for a mecanum drive robot.
  - Uses robot geometry parameters (`wheel_radius`, `lx`, `ly`) for accurate calculations.
- **Safety Features**:
  - Clamps velocities to configurable limits (`max_linear_x`, `max_linear_y`, `max_angular`).
  - Clamps integral terms to prevent windup (`max_integral = 0.35`).
  - Checks for NaN values and returns zero velocities if detected.
  - No minimum velocity deadzones to prevent the robot from getting stuck.

The controller outputs a `geometry_msgs/Twist` message with `linear.x`, `linear.y`, and `angular.z`, which can be used directly or converted to wheel speeds for mecanum drive robots.

## Safety Notes

- The node stops the robot if:
  - The `docking_active` flag is `false`.
  - The marker pose is stale (older than `marker_timeout_sec`).
  - Invalid yaw or NaN values are detected.
- Velocity commands are clamped to:
  - Linear: ±0.1 m/s (configurable via `max_linear_x`, `max_linear_y`).
  - Angular: ±0.07 rad/s (configurable via `max_angular`).

## Extending the Package

To add a new controller:
1. Create a new class inheriting from `ControllerInterface` in `dockpilot_control/<new_controller>.hpp`.
2. Implement the `compute` method to return a `geometry_msgs::msg::Twist` based on the error pose.
3. Update `ControlNode::init_controller_once` to support the new controller type.

To extend the PID controller:
1. Add new parameters to `control_params.yaml` (e.g., for additional tuning or features).
2. Modify `PIDController::compute` to incorporate new control logic or features.

## Debugging Tips

- **TF2 Issues**: Use `ros2 run tf2_tools view_frames` to inspect the TF tree.
- **Stale Poses**: Adjust `marker_timeout_sec` if marker updates are infrequent.
- **PID Tuning**: Adjust `kp`, `ki`, `kd`, and gain scheduling parameters in `control_params.yaml` to optimize performance.
- **Verbose Output**: The PID controller prints errors, gains, velocities, and wheel speeds to the console. Enable debug logging with:
  ```bash
  ros2 run dockpilot_control control_node --ros-args --log-level debug
  ```




