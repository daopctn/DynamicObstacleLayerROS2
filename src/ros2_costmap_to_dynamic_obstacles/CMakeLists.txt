cmake_minimum_required(VERSION 3.8)
project(ros2_costmap_to_dynamic_obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)

add_executable(costmap_converter_node src/costmap_converter_node.cpp)
ament_target_dependencies(costmap_converter_node rclcpp nav2_dynamic_msgs)

install(TARGETS
  costmap_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
