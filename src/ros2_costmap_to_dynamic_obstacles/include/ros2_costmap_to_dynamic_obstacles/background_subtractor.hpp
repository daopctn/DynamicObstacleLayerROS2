#ifndef BACKGROUND_SUBTRACTOR_HPP
#define BACKGROUND_SUBTRACTOR_HPP

#include <opencv2/opencv.hpp>

class BackgroundSubtractor {
public:
    BackgroundSubtractor();
    void apply(const cv::Mat& input, cv::Mat& fg_mask);
private:
    cv::Ptr<cv::BackgroundSubtractorMOG2> bg_subtractor_;
};

#endif // BACKGROUND_SUBTRACTOR_HPP
