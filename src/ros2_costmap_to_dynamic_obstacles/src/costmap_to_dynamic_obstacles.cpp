#include "costmap_to_dynamic_obstacles.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_dynamic_msgs/msg/obstacle.hpp>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

void CostmapToDynamicObstacles::initialize() {
    // Initialize background_subtractor with parameters
    BackgroundSubtractor::Params params;
    params.alpha_fast = 0.7;
    params.alpha_slow = 0.1;
    params.beta = 0.8;
    params.min_occupancy_probability = 100;
    params.min_sep_between_fast_and_slow_filter = 40;
    params.max_occupancy_neighbors = 100;
    params.morph_size = 0;
    background_subtractor_ = std::make_shared<BackgroundSubtractor>(params);

    BlobDetector::Params blob_params;
    blob_params.filterByColor = true;
    blob_params.blobColor = 255;
    blob_params.minDistBetweenBlobs = 10.0f;
    blob_params.filterByArea = true;
    blob_params.minArea = 3.0f;
    blob_params.maxArea = 1000.0f;
    blob_params.filterByCircularity = true;
    blob_params.minCircularity = 0.0f;
    blob_params.maxCircularity = 1.0f;
    blob_params.filterByInertia = true;
    blob_params.minInertiaRatio = 0.0f;
    blob_params.maxInertiaRatio = 1.0f;
    blob_params.filterByConvexity = true;
    blob_params.minConvexity = 0.0f;
    blob_params.maxConvexity = 1.0f;
    blob_detector_ = BlobDetector::create(blob_params);
}

void CostmapToDynamicObstacles::setCostmap2D(nav2_costmap_2d::Costmap2D* costmap, cv::Mat& costmap_mat) {
    if (costmap) {
        // Convert Costmap2D to OpenCV Mat
        unsigned int width = costmap->getSizeInCellsX();
        unsigned int height = costmap->getSizeInCellsY();

        costmap_mat = cv::Mat(height, width, CV_8UC1);
        for (unsigned int y = 0; y < height; ++y) {
            unsigned char* row_ptr = costmap_mat.ptr<unsigned char>(y);
            for (unsigned int x = 0; x < width; ++x) {
                row_ptr[x] = costmap->getCost(x, y);
            }
        }
        cv::flip(costmap_mat, costmap_mat, 0);  // Flip matrix vertically
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("costmap_to_dynamic_obstacles"), "Failed to get Costmap2D instance.");
    }
}

void CostmapToDynamicObstacles::compute(nav2_costmap_2d::Costmap2D* costmap, const cv::Mat& costmap_mat, nav2_dynamic_msgs::msg::ObstacleArray& obstacle_array) {
    if (!costmap) {
        RCLCPP_ERROR(rclcpp::get_logger("costmap_to_dynamic_obstacles"), "Failed to get Costmap2D from Costmap2DROS.");
        return;
    }

    fg_mask = cv::Mat(costmap_mat.size(), CV_8UC1, cv::Scalar(0));

    int origin_x = static_cast<int>(costmap->getOriginX() / costmap->getResolution());
    int origin_y = static_cast<int>(costmap->getOriginY() / costmap->getResolution());

    background_subtractor_->apply(costmap_mat, fg_mask, origin_x, origin_y);
    std::vector<cv::KeyPoint> keypoints;
    blob_detector_->detect(fg_mask, keypoints);

    obstacle_array.obstacles.clear();

    for (size_t i = 0; i < keypoints.size(); ++i) {
        nav2_dynamic_msgs::msg::Obstacle obstacle;
        // Tạo UUID ngẫu nhiên cho mỗi obstacle
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        unique_identifier_msgs::msg::UUID ros_uuid;
        std::copy(uuid.begin(), uuid.end(), ros_uuid.uuid.begin());  // Copy UUID từ Boost sang ROS2

        obstacle.id = ros_uuid;  // Gán UUID cho obstacle


        obstacle.position.x = keypoints[i].pt.x;
        obstacle.position.y = keypoints[i].pt.y;

        
        float radius = contours_[i].size() / 2;
        obstacle.size.x = radius * 2;
        obstacle.size.y = radius * 2;

        obstacle.header.stamp = rclcpp::Clock().now();
        obstacle.header.frame_id = "map";

        obstacle_array.obstacles.push_back(obstacle);
    }

    RCLCPP_INFO(rclcpp::get_logger("costmap_to_dynamic_obstacles"), "Detected %lu obstacles", keypoints.size());
}