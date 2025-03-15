#include <rclcpp/rclcpp.hpp>
#include "costmap_to_dynamic_obstacles.hpp"

class CostmapConversionNode : public rclcpp::Node {
public:
    CostmapConversionNode() : Node("costmap_converter_node") {
        detection_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("/detection", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(150), std::bind(&CostmapConversionNode::publishCallback, this));
        costmap_converter_ = std::make_unique<CostmapToDynamicObstacle>();
        costmap_converter_->initialize();
    }
private:
    std::unique_ptr<CostmapToDynamicObstacle> costmap_converter_;
    rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr detection_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishCallback() {
        cv::Mat fg_mask;
        costmap_converter_->compute(fg_mask);
        nav2_dynamic_msgs::msg::ObstacleArray obstacles_msg;
        costmap_converter_->fillObstacleArrayMsg(obstacles_msg);
        detection_pub_->publish(obstacles_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapConversionNode>());
    rclcpp::shutdown();
    return 0;
}
