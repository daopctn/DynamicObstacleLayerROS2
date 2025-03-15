#ifndef BLOB_DETECTOR_HPP
#define BLOB_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class BlobDetector {
public:
    BlobDetector();
    void detect(const cv::Mat& fg_mask, std::vector<cv::KeyPoint>& keypoints, std::vector<std::vector<cv::Point>>& contours);
private:
    cv::SimpleBlobDetector::Params params_;
};

#endif // BLOB_DETECTOR_HPP
