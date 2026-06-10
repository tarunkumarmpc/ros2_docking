cmake_minimum_required(VERSION 3.8)
project(dockpilot_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(dockpilot_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# ------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(docking_server src/docking_server.cpp)

ament_target_dependencies(docking_server
  rclcpp rclcpp_action geometry_msgs std_msgs tf2 tf2_geometry_msgs
  dockpilot_interfaces diagnostic_msgs
)

target_link_libraries(docking_server yaml-cpp::yaml-cpp)

install(TARGETS docking_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()