#ifndef COSTMAP_TO_DYNAMIC_OBSTACLE_H
#define COSTMAP_TO_DYNAMIC_OBSTACLE_H

#include <opencv2/opencv.hpp>
#include "ros2_costmap_to_dynamic_obstacles/background_subtractor.h"
#include "ros2_costmap_to_dynamic_obstacles/blob_detector.h"
#include <vector>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>

class CostmapToDynamicObstacles {
    public:
        // Initialize method with detector and background subtractor parameters
        void initialize();
    
        // Method to set costmap and convert it into a cv::Mat object
        void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap, cv::Mat& costmap_mat);
    
        // Compute method to detect and update dynamic obstacles
        void compute(nav2_costmap_2d::Costmap2D* costmap, const cv::Mat& costmap_mat, nav2_dynamic_msgs::msg::ObstacleArray& obstacle_array);
    
    private:
    
        std::shared_ptr<BackgroundSubtractor> background_subtractor_;
        std::shared_ptr<BlobDetector> blob_detector_;
        cv::Mat fg_mask;  // Ma trận mask foreground
        cv::Mat costmap_mat;  // Ma trận OpenCV đại diện cho costmap
        std::vector<cv::KeyPoint> keypoints_;  // Danh sách keypoints từ blob detector
        std::vector<std::vector<cv::Point>> contours_;  // Danh sách contours từ blob detector
    };
    
#endif // COSTMAP_TO_DYNAMIC_OBSTACLE_H
