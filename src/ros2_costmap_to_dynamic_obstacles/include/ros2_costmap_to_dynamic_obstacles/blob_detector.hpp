#ifndef BLOB_DETECTOR_HPP
#define BLOB_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class BlobDetector {
public:
    BlobDetector(float min_area, float max_area, float min_circularity, float min_inertia_ratio);
    void detect(const cv::Mat& binary_image, std::vector<cv::KeyPoint>& keypoints);

private:
    cv::SimpleBlobDetector::Params params_;
    cv::Ptr<cv::SimpleBlobDetector> detector_;
};

#endif // BLOB_DETECTOR_HPP