#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>
#include <opencv2/opencv.hpp>
#include "costmap_to_dynamic_obstacles.hpp"

class CostmapConversionNode : public rclcpp::Node {
public:
    CostmapConversionNode() : Node("costmap_converter_node") {
        // Tạo costmap riêng
        costmap_ros_ = std::make_shared<costmap_2d::Costmap2DROS>("local_costmap", tf_buffer_, "map");

        // Tạo publisher cho topic /detection
        detection_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("/detection", 10);

        // Tạo bộ hẹn giờ gọi publishCallback mỗi 150ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(150),
            std::bind(&CostmapConversionNode::publishCallback, this)
        );

        // Khởi tạo bộ xử lý costmap
        costmap_converter_ = std::make_unique<CostmapToDynamicObstacle>();
        costmap_converter_->initialize();
    }

private:
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<CostmapToDynamicObstacle> costmap_converter_;
    rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr detection_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;

    void publishCallback() {
        // Chuyển đổi costmap sang ảnh OpenCV
        cv::Mat costmap_image;
        costmap_converter_->setCostmap2D(costmap_image);

        // Xử lý tách vật cản động
        cv::Mat fg_mask;
        costmap_converter_->compute(fg_mask);

        // Tạo ObstacleArrayMsg để xuất dữ liệu
        nav2_dynamic_msgs::msg::ObstacleArray obstacles_msg;
        costmap_converter_->fillObstacleArrayMsg(obstacles_msg);

        // Xuất dữ liệu lên topic /detection
        detection_pub_->publish(obstacles_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapConversionNode>());
    rclcpp::shutdown();
    return 0;
}
