cmake_minimum_required(VERSION 3.8)
project(ros2_costmap_to_dynamic_obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(costmap_converter_node src/costmap_converter_node.cpp)

# Đảm bảo target có include OpenCV
target_include_directories(costmap_converter_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Liên kết OpenCV với node
target_link_libraries(costmap_converter_node ${OpenCV_LIBS})

ament_target_dependencies(costmap_converter_node rclcpp nav2_dynamic_msgs)

install(TARGETS
  costmap_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
