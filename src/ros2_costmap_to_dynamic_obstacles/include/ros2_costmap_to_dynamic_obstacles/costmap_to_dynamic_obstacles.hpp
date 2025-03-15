#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <opencv2/opencv.hpp>
#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"

class CostmapToDynamicObstacle
{
public:
    CostmapToDynamicObstacle();
    void initialize();
    void setCostmap2D(const nav2_costmap_2d::Costmap2DROS &costmap_ros);
    void compute();
    nav2_dynamic_msgs::msg::ObstacleArray getObstacles() const;

private:
    cv::Mat costmap_mat_;
    nav2_dynamic_msgs::msg::ObstacleArray obstacle_array_;
    void detectBlobs();
};

#endif // COSTMAP_TO_DYNAMIC_OBSTACLES_H