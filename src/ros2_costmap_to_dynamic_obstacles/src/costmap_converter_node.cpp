#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <memory>

class CustomCostmapNode : public nav2_util::LifecycleNode {
public:
  CustomCostmapNode() : LifecycleNode("custom_costmap_node") {
    RCLCPP_INFO(this->get_logger(), "CustomCostmapNode created.");
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "Configuring...");

    // Khởi tạo Costmap2DROS với tên riêng để tránh xung đột
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "custom_local_costmap");

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "Activating...");

    // Kích hoạt Costmap2DROS
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());

    // Lấy con trỏ tới Costmap2D và in kích thước
    nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (costmap) {
      unsigned int width = costmap->getSizeInCellsX();
      unsigned int height = costmap->getSizeInCellsY();
      RCLCPP_INFO(this->get_logger(), "Custom costmap size: width = %u, height = %u", width, height);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to get Costmap2D instance.");
      return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "Deactivating...");
    costmap_ros_->on_deactivate(rclcpp_lifecycle::State());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(this->get_logger(), "Cleaning up...");
    costmap_ros_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomCostmapNode>();

  // Đưa node vào trạng thái active
  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());
  node->deactivate();
  node->cleanup();
  rclcpp::shutdown();
  return 0;
}