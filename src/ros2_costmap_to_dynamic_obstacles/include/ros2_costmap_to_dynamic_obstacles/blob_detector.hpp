// blob_detector.hpp
#ifndef BLOB_DETECTOR_HPP
#define BLOB_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class BlobDetector {
public:
    BlobDetector();
    void detectBlobs(const cv::Mat& inputImage, std::vector<cv::KeyPoint>& keypoints);

private:
    cv::Ptr<cv::SimpleBlobDetector> detector;
    void configureDetector();
};

#endif // BLOB_DETECTOR_HPP
