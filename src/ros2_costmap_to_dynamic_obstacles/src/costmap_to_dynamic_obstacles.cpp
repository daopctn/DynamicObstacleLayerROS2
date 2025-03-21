#include "costmap_to_dynamic_obstacles.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

void CostmapToDynamicObstacles::initialize(const BlobDetectorParams& blob_params, const BackgroundSubtractorParams& bg_params) {
    // Initialize background subtractor and blob detector with parameters
    background_subtractor_ = std::make_shared<BackgroundSubtractor>(bg_params);
    blob_detector_ = std::make_shared<BlobDetector>(blob_params);
}

void CostmapToDynamicObstacles::setCostmap2D(costmap_2d::Costmap2D* costmap, cv::Mat& costmap_mat) {
    int width = costmap->getSizeInCellsX();
    int height = costmap->getSizeInCellsY();
    costmap_mat = cv::Mat(height, width, CV_8UC1);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            unsigned char cost = costmap->getCost(x, y);
            costmap_mat.at<uchar>(y, x) = (cost == costmap_2d::LETHAL_OBSTACLE) ? 255 : 0;
        }
    }
}

void CostmapToDynamicObstacles::compute(const cv::Mat& costmap_mat, nav2_dynamic_msgs::msg::ObstacleArray& obstacle_array) {
    cv::Mat foreground_mask;
    
    // Apply background subtraction to isolate dynamic obstacles
    background_subtractor_->apply(costmap_mat, foreground_mask);

    // Perform blob detection to identify dynamic objects and their contours
    std::vector<cv::KeyPoint> keypoints;
    std::vector<std::vector<cv::Point>> contours;
    blob_detector_->detect(foreground_mask, keypoints, contours);

    // Clear the obstacle array before populating new detected obstacles
    obstacle_array.obstacles.clear();

    // Fill the ObstacleArray message
    for (size_t i = 0; i < keypoints.size(); ++i) {
        nav2_dynamic_msgs::msg::Obstacle obstacle;

        // Generate a unique UUID for each obstacle
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        obstacle.uuid = boost::uuids::to_string(uuid);

        // Assign position from keypoints (centroids)
        obstacle.position.x = keypoints[i].pt.x;
        obstacle.position.y = keypoints[i].pt.y;

        // Calculate bounding rectangle for the contour
        cv::Rect bounding_box = cv::boundingRect(contours[i]);
        obstacle.size_x = bounding_box.width;
        obstacle.size_y = bounding_box.height;

        // Fill header with timestamp and reference frame
        obstacle.header.stamp = rclcpp::Clock().now();
        obstacle.header.frame_id = "/map";

        obstacle_array.obstacles.push_back(obstacle);
    }

    RCLCPP_INFO(rclcpp::get_logger("costmap_to_dynamic_obstacles"), "Detected %lu obstacles", keypoints.size());
}
