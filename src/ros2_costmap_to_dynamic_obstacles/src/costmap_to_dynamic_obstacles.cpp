#include "costmap_to_dynamic_obstacles.hpp"

CostmapToDynamicObstacle::CostmapToDynamicObstacle() {}

void CostmapToDynamicObstacle::initialize() {}

void CostmapToDynamicObstacle::setCostmap2D(const cv::Mat& costmap) {}

void CostmapToDynamicObstacle::compute(cv::Mat& fg_mask) {
    bg_subtractor_.apply(costmap, fg_mask);
    blob_detector_.detect(fg_mask, keypoints, contours);
}

void CostmapToDynamicObstacle::fillObstacleArrayMsg(nav2_dynamic_msgs::msg::ObstacleArray& obstacles_msg) {
    // Convert keypoints to obstacles messages
}
