#include "ros2_costmap_to_dynamic_obstacles/background_subtractor.hpp"


BackgroundSubtractor::BackgroundSubtractor() {
    // Khởi tạo bộ lọc nhanh và bộ lọc chậm với ma trận rỗng
    fast_filter_ = cv::Mat();
    slow_filter_ = cv::Mat();
}

void BackgroundSubtractor::apply(const cv::Mat& costmap, cv::Mat& fg_mask) {
    if (fast_filter_.empty() || slow_filter_.empty()) {
        // Khởi tạo bộ lọc với cùng kích thước costmap
        costmap.convertTo(fast_filter_, CV_32F);
        costmap.convertTo(slow_filter_, CV_32F);
    }

    // Chuyển đổi costmap sang float
    cv::Mat costmap_float;
    costmap.convertTo(costmap_float, CV_32F);

    // Cập nhật bộ lọc nhanh và chậm
    fast_filter_ = beta_ * ((1 - alpha_fast_) * fast_filter_ + alpha_fast_ * costmap_float) + 
                   (1 - beta_) * cv::blur(fast_filter_, cv::Size(3, 3));
    slow_filter_ = beta_ * ((1 - alpha_slow_) * slow_filter_ + alpha_slow_ * costmap_float) + 
                   (1 - beta_) * cv::blur(slow_filter_, cv::Size(3, 3));

    // Tạo mặt nạ foreground bằng cách lấy hiệu giữa bộ lọc nhanh và chậm
    cv::Mat diff;
    cv::absdiff(fast_filter_, slow_filter_, diff);

    // Ngưỡng để xác định các điểm động
    fg_mask = (fast_filter_ > threshold1_) & (diff > threshold2_);
}
