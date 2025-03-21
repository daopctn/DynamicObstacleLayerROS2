#ifndef COSTMAP_TO_DYNAMIC_OBSTACLE_HPP
#define COSTMAP_TO_DYNAMIC_OBSTACLE_HPP

#include <opencv2/opencv.hpp>
#include "ros2_costmap_to_dynamic_obstacles/background_subtractor.hpp"
#include "ros2_costmap_to_dynamic_obstacles/blob_detector.hpp"
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>
#include <costmap_2d/costmap_2d.hpp>

class CostmapToDynamicObstacles {
public:
    // Initialize method with detector and background subtractor parameters
    void initialize(const BlobDetectorParams& blob_params, const BackgroundSubtractorParams& bg_params);

    // Method to set costmap and convert it into a cv::Mat object
    void setCostmap2D(costmap_2d::Costmap2D* costmap, cv::Mat& costmap_mat);

    // Compute method to detect and update dynamic obstacles
    void compute(const cv::Mat& costmap_mat, nav2_dynamic_msgs::msg::ObstacleArray& obstacle_array);

private:
    std::shared_ptr<BackgroundSubtractor> background_subtractor_;
    std::shared_ptr<BlobDetector> blob_detector_;

    cv::Mat costmap_mat_;                                 // Holds the converted costmap as cv::Mat
    std::vector<cv::KeyPoint> keypoints_;                  // Stores keypoints detected by BlobDetector
    std::vector<std::vector<cv::Point>> contours_;         // Stores contours of detected blobs
};

#endif // COSTMAP_TO_DYNAMIC_OBSTACLE_HPP
