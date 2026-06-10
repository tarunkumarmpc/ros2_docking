cmake_minimum_required(VERSION 3.8)
project(dockpilot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_path(APRILTAG_INCLUDE_DIRS apriltag.h PATH_SUFFIXES apriltag REQUIRED)
find_library(APRILTAG_LIBRARIES apriltag REQUIRED)


add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
target_include_directories(apriltag_detector_node PRIVATE ${APRILTAG_INCLUDE_DIRS})
ament_target_dependencies(apriltag_detector_node
  rclcpp sensor_msgs geometry_msgs apriltag_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_geometry)
target_link_libraries(apriltag_detector_node ${APRILTAG_LIBRARIES} ${OpenCV_LIBS})

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
  rclcpp sensor_msgs geometry_msgs apriltag_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs)
target_link_libraries(perception_node ${OpenCV_LIBS} ${angles_LIBRARIES})


install(TARGETS
  apriltag_detector_node
  perception_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
ament_package()