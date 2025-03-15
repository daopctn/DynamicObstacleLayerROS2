#include "costmap_to_dynamic_obstacles.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "nav2_dynamic_msgs/msg/obstacle.hpp"

CostmapToDynamicObstacle::CostmapToDynamicObstacle() {}

void CostmapToDynamicObstacle::initialize() {
    // Khởi tạo cần thiết, nếu có
}

void CostmapToDynamicObstacle::setCostmap2D(const nav2_costmap_2d::Costmap2DROS &costmap_ros) {
    unsigned int width = costmap_ros.getCostmap()->getSizeInCellsX();
    unsigned int height = costmap_ros.getCostmap()->getSizeInCellsY();
    costmap_mat_ = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    
    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            unsigned char cost = costmap_ros.getCostmap()->getCost(j, i);
            costmap_mat_.at<uchar>(i, j) = cost;
        }
    }
}

void CostmapToDynamicObstacle::compute() {
    detectBlobs();
}

void CostmapToDynamicObstacle::detectBlobs() {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(costmap_mat_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    obstacle_array_.obstacles.clear();
    for (const auto &contour : contours) {
        cv::Rect bounding_box = cv::boundingRect(contour);
        
        nav2_dynamic_msgs::msg::Obstacle obstacle;
        obstacle.position.x = bounding_box.x;
        obstacle.position.y = bounding_box.y;
        obstacle.size.x = bounding_box.width;
        obstacle.size.y = bounding_box.height;
        
        obstacle_array_.obstacles.push_back(obstacle);
    }
}

nav2_dynamic_msgs::msg::ObstacleArray CostmapToDynamicObstacle::getObstacles() const {
    return obstacle_array_;
}