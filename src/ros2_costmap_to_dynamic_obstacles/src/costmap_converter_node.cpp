#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include "costmap_to_dynamic_obstacles.h"
#include <memory>

class CustomCostmapNode : public nav2_util::LifecycleNode {
public:
  CustomCostmapNode() : LifecycleNode("custom_costmap_node") {
    RCLCPP_INFO(this->get_logger(), "[Constructor] CustomCostmapNode created.");

    // Lifecycle-aware publisher to publish ObstacleArray on the /detect topic
    obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("/detect", 10);
    
    // Timer to run the callback every 150 ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(150), std::bind(&CustomCostmapNode::publishCallback, this));
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "[on_configure] Configuring the node...");

    // Initialize Costmap2DROS with the required name
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("custom_local_costmap");
    RCLCPP_INFO(this->get_logger(), "[on_configure] Initialized Costmap2DROS.");

    // Initialize CostmapToDynamicObstacles with suitable parameters
    costmap_to_dynamic_obstacles_ = std::make_shared<CostmapToDynamicObstacles>();
    costmap_to_dynamic_obstacles_->initialize();
    RCLCPP_INFO(this->get_logger(), "[on_configure] CostmapToDynamicObstacles initialized.");

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "[on_activate] Activating the node...");

    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());
    obstacle_pub_->on_activate();  // Activate the lifecycle publisher

    costmap = costmap_ros_->getCostmap();
    if (costmap) {
      unsigned int width = costmap->getSizeInCellsX();
      unsigned int height = costmap->getSizeInCellsY();
      RCLCPP_INFO(this->get_logger(), "[on_activate] Costmap2D activated with size: width = %u, height = %u", width, height);
    } else {
      RCLCPP_ERROR(this->get_logger(), "[on_activate] Failed to get Costmap2D instance.");
      return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "[on_deactivate] Deactivating the node...");
    costmap_ros_->on_deactivate(rclcpp_lifecycle::State());
    obstacle_pub_->on_deactivate();  // Deactivate the publisher
    costmap = nullptr;
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "[on_cleanup] Cleaning up the node...");
    costmap_ros_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  void publishCallback() {
    RCLCPP_INFO(this->get_logger(), "[publishCallback] Timer triggered.");

    if (!costmap) {
      RCLCPP_ERROR(this->get_logger(), "[publishCallback] Costmap2D is not available.");
      return;
    }

    // Debugging log for costmap properties
    RCLCPP_DEBUG(this->get_logger(), "[publishCallback] Costmap size: width = %u, height = %u",
                 costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

    // Convert Costmap2D to OpenCV Mat
    cv::Mat costmap_mat;
    costmap_to_dynamic_obstacles_->setCostmap2D(costmap, costmap_mat);
    RCLCPP_DEBUG(this->get_logger(), "[publishCallback] Costmap2D converted to cv::Mat.");

    // Create and populate the ObstacleArray
    nav2_dynamic_msgs::msg::ObstacleArray obstacle_array;
    costmap_to_dynamic_obstacles_->compute();

    // Log obstacle detection process
    RCLCPP_INFO(this->get_logger(), "[publishCallback] Number of dynamic obstacles detected: %lu", obstacle_array.obstacles.size());

    if (!obstacle_array.obstacles.empty()) {
      for (size_t i = 0; i < obstacle_array.obstacles.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "[publishCallback] Obstacle %lu: Position (x: %.2f, y: %.2f), Size (x: %.2f, y: %.2f)",
                     i, obstacle_array.obstacles[i].position.x, obstacle_array.obstacles[i].position.y,
                     obstacle_array.obstacles[i].size.x, obstacle_array.obstacles[i].size.y);
      }
    }

    // Publish the obstacle array
    obstacle_pub_->publish(obstacle_array);
    RCLCPP_DEBUG(this->get_logger(), "[publishCallback] Published ObstacleArray to /detect.");
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<CostmapToDynamicObstacles> costmap_to_dynamic_obstacles_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav2_costmap_2d::Costmap2D* costmap = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomCostmapNode>();

  // Handle lifecycle state transitions with debug logs
  node->configure();
  node->activate();
  rclcpp::spin(node->get_node_base_interface());
  node->deactivate();
  node->cleanup();
  rclcpp::shutdown();
  return 0;
}
