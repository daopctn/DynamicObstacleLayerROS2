#include "blob_detector.hpp"

BlobDetector::BlobDetector() {
    params_.filterByArea = true;
    params_.minArea = 10;
}

void BlobDetector::detect(const cv::Mat& fg_mask, std::vector<cv::KeyPoint>& keypoints, std::vector<std::vector<cv::Point>>& contours) {
    cv::SimpleBlobDetector detector(params_);
    detector.detect(fg_mask, keypoints);
    cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}
