#include "ros2_costmap_to_dynamic_obstacles/blob_detector.hpp"


BlobDetector::BlobDetector(float min_area, float max_area, float min_circularity, float min_inertia_ratio) {
    params_.filterByArea = true;
    params_.minArea = min_area;
    params_.maxArea = max_area;
    
    params_.filterByCircularity = true;
    params_.minCircularity = min_circularity;
    
    params_.filterByInertia = true;
    params_.minInertiaRatio = min_inertia_ratio;
    
    detector_ = cv::SimpleBlobDetector::create(params_);
}

void BlobDetector::detect(const cv::Mat& binary_image, std::vector<cv::KeyPoint>& keypoints) {
    detector_->detect(binary_image, keypoints);
}