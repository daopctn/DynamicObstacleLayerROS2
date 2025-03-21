#ifndef BLOBDETECTOR_HPP
#define BLOBDETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class BlobDetector {
public:
    BlobDetector(float min_area = 10.0f, float max_area = 1000.0f, 
                 int min_threshold = 128, int max_threshold = 255);
    // Phát hiện blob, trả về keypoints (centroids) và contours
    void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints_, std::vector<std::vector<cv::Point>>& contours);

private:
    cv::Ptr<cv::SimpleBlobDetector> detector_;  // Pointer to OpenCV's SimpleBlobDetector
};

#endif // BLOBDETECTOR_HPP