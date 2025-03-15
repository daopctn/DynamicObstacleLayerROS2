#include "ros2_costmap_to_dynamic_obstacles/background_subtractor.hpp"

BackgroundSubtractor::BackgroundSubtractor() {
    bg_subtractor_ = cv::createBackgroundSubtractorMOG2(500, 16, true);
}

void BackgroundSubtractor::apply(const cv::Mat& input, cv::Mat& fg_mask) {
    if (input.empty()) return;
    bg_subtractor_->apply(input, fg_mask);
    cv::erode(fg_mask, fg_mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(fg_mask, fg_mask, cv::Mat(), cv::Point(-1, -1), 2);
}
