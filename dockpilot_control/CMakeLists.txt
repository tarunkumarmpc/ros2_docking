cmake_minimum_required(VERSION 3.8)
project(dockpilot_control)

# Dependencies (add 'Eigen3' if using any linear algebra in mpc)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# If using Eigen for MPC math, uncomment:
 find_package(Eigen3 REQUIRED)


# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}  # Uncomment if using Eigen3
    angles::angles
)

# PID Controller library
add_library(pid_controller src/pid_controller.cpp)
ament_target_dependencies(pid_controller rclcpp geometry_msgs tf2_geometry_msgs)

# Pure Pursuit controller library
add_library(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller rclcpp geometry_msgs)

# MPC Controller library ---
add_library(mpc_controller src/mpc_controller.cpp)
ament_target_dependencies(mpc_controller rclcpp geometry_msgs)
target_link_libraries(mpc_controller Eigen3::Eigen) # if using Eigen

# LQR Controller library 
add_library(lqr_controller src/lqr_controller.cpp)
ament_target_dependencies(lqr_controller rclcpp geometry_msgs)
target_link_libraries(lqr_controller Eigen3::Eigen) # if using Eigen

# Control Node executable
add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node
  pid_controller
  pure_pursuit_controller
  mpc_controller
  lqr_controller
)
ament_target_dependencies(control_node
  rclcpp geometry_msgs std_msgs tf2_ros tf2_geometry_msgs
)

# Install targets
install(TARGETS
  control_node
  pid_controller
  pure_pursuit_controller
  mpc_controller
  lqr_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Export include directory
ament_export_include_directories(include)

# Export libraries
ament_export_libraries(
  pid_controller
  pure_pursuit_controller
  mpc_controller
  lqr_controller
)

# Install launch files (ignore trailing 'f' if not needed)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install include directory
install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
