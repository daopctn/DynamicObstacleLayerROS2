#include "background_subtractor.hpp"

// Constructor mặc định
BackgroundSubtractor::BackgroundSubtractor()
    : alpha_fast_(0.1f),    // Hệ số cập nhật nhanh mặc định
      alpha_slow_(0.01f),   // Hệ số cập nhật chậm mặc định
      beta_(0.5f),          // Ảnh hưởng của hàng xóm mặc định
      threshold1_(10.0f),   // Ngưỡng kích hoạt bộ lọc nhanh mặc định
      threshold2_(5.0f)     // Ngưỡng chênh lệch giữa hai bộ lọc mặc định
{
    // Các ma trận fast_filter_ và slow_filter_ sẽ được khởi tạo khi có costmap đầu tiên
}

// Constructor có tham số
BackgroundSubtractor::BackgroundSubtractor(float alpha_fast, float alpha_slow, float beta, float threshold1, float threshold2)
    : alpha_fast_(alpha_fast),
      alpha_slow_(alpha_slow),
      beta_(beta),
      threshold1_(threshold1),
      threshold2_(threshold2)
{
    // Các ma trận fast_filter_ và slow_filter_ sẽ được khởi tạo khi có costmap đầu tiên
}

void BackgroundSubtractor::apply(const cv::Mat& costmap, cv::Mat& fg_mask)
{
    // Kiểm tra xem costmap đầu vào có hợp lệ không
    if (costmap.empty()) {
        fg_mask = cv::Mat();
        return;
    }

    // Nếu đây là lần đầu tiên, khởi tạo các bộ lọc với costmap ban đầu
    if (fast_filter_.empty() || slow_filter_.empty()) {
        costmap.copyTo(fast_filter_);
        costmap.copyTo(slow_filter_);
        fg_mask = cv::Mat::zeros(costmap.size(), CV_8UC1); // Ban đầu không có tiền cảnh
        return;
    }

    // Đảm bảo kích thước của các ma trận phù hợp với costmap đầu vào
    CV_Assert(costmap.size() == fast_filter_.size() && costmap.size() == slow_filter_.size());

    // Cập nhật bộ lọc nhanh và chậm bằng công thức trung bình chạy
    fast_filter_ = (1.0f - alpha_fast_) * fast_filter_ + alpha_fast_ * costmap;
    slow_filter_ = (1.0f - alpha_slow_) * slow_filter_ + alpha_slow_ * costmap;

    // Tính toán sự khác biệt giữa costmap hiện tại và bộ lọc nhanh
    cv::Mat diff_fast;
    cv::absdiff(costmap, fast_filter_, diff_fast);

    // Tính toán sự khác biệt giữa bộ lọc nhanh và chậm
    cv::Mat diff_filters;
    cv::absdiff(fast_filter_, slow_filter_, diff_filters);

    // Tạo mặt nạ tiền cảnh ban đầu dựa trên ngưỡng threshold1_
    fg_mask = cv::Mat::zeros(costmap.size(), CV_8UC1);
    fg_mask.setTo(255, diff_fast > threshold1_);

    // Tinh chỉnh mặt nạ bằng cách kiểm tra sự khác biệt giữa hai bộ lọc
    cv::Mat refine_mask;
    refine_mask = (diff_filters > threshold2_);
    fg_mask &= refine_mask; // Chỉ giữ những điểm thỏa mãn cả hai điều kiện

    // Áp dụng ảnh hưởng của hàng xóm (spatial smoothing) bằng beta_
    if (beta_ > 0.0f) {
        cv::Mat smoothed_mask;
        cv::GaussianBlur(fg_mask, smoothed_mask, cv::Size(5, 5), 0); // Lọc Gaussian để làm mịn
        fg_mask = (1.0f - beta_) * fg_mask + beta_ * smoothed_mask; // Kết hợp với mặt nạ gốc
        fg_mask.convertTo(fg_mask, CV_8UC1); // Chuyển về định dạng nhị phân
        cv::threshold(fg_mask, fg_mask, 127, 255, cv::THRESH_BINARY); // Nhị phân hóa lại
    }
}