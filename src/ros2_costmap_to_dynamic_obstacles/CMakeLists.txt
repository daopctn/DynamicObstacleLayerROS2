cmake_minimum_required(VERSION 3.8)
project(ros2_costmap_to_dynamic_obstacles)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
  include/ros2_costmap_to_dynamic_obstacles
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(costmap_converter_node src/costmap_converter_node.cpp src/background_subtractor.cpp src/blob_detector.cpp src/costmap_to_dynamic_obstacles.cpp)
ament_target_dependencies(costmap_converter_node rclcpp 
                                                 sensor_msgs
                                                 nav2_costmap_2d
                                                 OpenCV 
                                                 cv_bridge 
                                                 unique_identifier_msgs
                                                 nav2_dynamic_msgs
                                                 tf2_ros
                                                 nav2_util
                          )

# Install executable
install(TARGETS 
  costmap_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Export package
ament_package()
