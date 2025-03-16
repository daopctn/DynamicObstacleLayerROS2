#include "ros2_costmap_to_dynamic_obstacles/background_subtractor.hpp"


BackgroundSubtractor::BackgroundSubtractor(float alpha_fast, float alpha_slow, float beta, float threshold1, float threshold2)
    : alpha_fast_(alpha_fast), alpha_slow_(alpha_slow), beta_(beta), threshold1_(threshold1), threshold2_(threshold2) {}

void BackgroundSubtractor::apply(const cv::Mat& costmap, cv::Mat& fg_mask) {
    if (fast_filter_.empty()) {
        costmap.convertTo(fast_filter_, CV_32F);
        costmap.convertTo(slow_filter_, CV_32F);
    }

    // Cập nhật bộ lọc nhanh và chậm
    fast_filter_ = (1 - alpha_fast_) * fast_filter_ + alpha_fast_ * costmap;
    slow_filter_ = (1 - alpha_slow_) * slow_filter_ + alpha_slow_ * costmap;

    // Tính toán mặt nạ chướng ngại vật động
    cv::Mat diff;
    cv::absdiff(fast_filter_, slow_filter_, diff);
    fg_mask = (fast_filter_ > threshold1_) & (diff > threshold2_);
}