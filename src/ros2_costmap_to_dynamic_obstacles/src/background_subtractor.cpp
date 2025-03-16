#include "ros2_costmap_to_dynamic_obstacles/background_subtractor.hpp"

BackgroundSubtractor::BackgroundSubtractor() {
    // Khởi tạo bộ lọc nhanh và bộ lọc chậm với ma trận rỗng
    fast_filter_ = cv::Mat();
    slow_filter_ = cv::Mat();

    // Gán giá trị cho các biến thành viên
    alpha_fast_ = 0.4;  // Hệ số cập nhật của bộ lọc nhanh
    alpha_slow_ = 0.1;  // Hệ số cập nhật của bộ lọc chậm
    beta_ = 0.8;        // Ảnh hưởng của hàng xóm gần nhất
    threshold1_ = 50;   // Ngưỡng kích hoạt bộ lọc nhanh
    threshold2_ = 30;   // Ngưỡng chênh lệch giữa bộ lọc nhanh và chậm
}

void BackgroundSubtractor::apply(const cv::Mat& costmap, cv::Mat& fg_mask) {
    if (fast_filter_.empty() || slow_filter_.empty()) {
        // Khởi tạo bộ lọc với cùng kích thước costmap
        costmap.convertTo(fast_filter_, CV_32F);
        costmap.convertTo(slow_filter_, CV_32F);
        fg_mask = cv::Mat::zeros(costmap.size(), CV_8UC1); // Đảm bảo fg_mask khởi tạo đúng
        return;
    }

    // Chuyển đổi costmap sang float
    cv::Mat costmap_float;
    costmap.convertTo(costmap_float, CV_32F);

    // Cập nhật bộ lọc nhanh và chậm với sự làm mịn
    cv::Mat fast_blur, slow_blur;
    cv::GaussianBlur(fast_filter_, fast_blur, cv::Size(3, 3), 0);
    cv::GaussianBlur(slow_filter_, slow_blur, cv::Size(3, 3), 0);

    fast_filter_ = beta_ * ((1 - alpha_fast_) * fast_filter_ + alpha_fast_ * costmap_float) + (1 - beta_) * fast_blur;
    slow_filter_ = beta_ * ((1 - alpha_slow_) * slow_filter_ + alpha_slow_ * costmap_float) + (1 - beta_) * slow_blur;

    // Tạo mặt nạ foreground bằng cách lấy hiệu giữa bộ lọc nhanh và chậm
    cv::Mat diff;
    cv::absdiff(fast_filter_, slow_filter_, diff);

    // Ngưỡng để xác định các điểm động
    fg_mask = (fast_filter_ > threshold1_) & (diff > threshold2_);
}
