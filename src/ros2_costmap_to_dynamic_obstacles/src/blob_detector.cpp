#include "blob_detector.hpp"

BlobDetector::BlobDetector() {
    configureDetector();
}

void BlobDetector::configureDetector() {
    cv::SimpleBlobDetector::Params params;
    
    params.filterByArea = true;
    params.minArea = 50;
    params.maxArea = 5000;
    
    params.filterByCircularity = true;
    params.minCircularity = 0.1;
    
    params.filterByInertia = true;
    params.minInertiaRatio = 0.1;
    
    detector = cv::SimpleBlobDetector::create(params);
}

void BlobDetector::detectBlobs(const cv::Mat& inputImage, std::vector<cv::KeyPoint>& keypoints) {
    detector->detect(inputImage, keypoints);
}
