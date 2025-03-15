#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_HPP
#define COSTMAP_TO_DYNAMIC_OBSTACLES_HPP

#include <opencv2/opencv.hpp>
#include "background_subtractor.hpp"
#include "blob_detector.hpp"

class CostmapToDynamicObstacle {
public:
    CostmapToDynamicObstacle();
    void initialize();
    void setCostmap2D(const cv::Mat& costmap);
    void compute(cv::Mat& fg_mask);
    void fillObstacleArrayMsg(nav2_dynamic_msgs::msg::ObstacleArray& obstacles_msg);
private:
    BackgroundSubtractor bg_subtractor_;
    BlobDetector blob_detector_;
};

#endif // COSTMAP_TO_DYNAMIC_OBSTACLES_HPP
