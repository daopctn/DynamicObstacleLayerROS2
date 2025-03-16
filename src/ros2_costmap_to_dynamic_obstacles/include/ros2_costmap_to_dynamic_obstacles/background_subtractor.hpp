#ifndef BACKGROUND_SUBTRACTOR_HPP
#define BACKGROUND_SUBTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class BackgroundSubtractor {
public:
    BackgroundSubtractor();
    void apply(const cv::Mat& costmap, cv::Mat& fg_mask);

private:
    const float alpha_fast_ = 0.4;  // Hệ số cập nhật của bộ lọc nhanh
    const float alpha_slow_ = 0.1;  // Hệ số cập nhật của bộ lọc chậm
    const float beta_ = 0.8;        // Ảnh hưởng của hàng xóm gần nhất
    const float threshold1_ = 50;   // Ngưỡng kích hoạt bộ lọc nhanh
    const float threshold2_ = 30;   // Ngưỡng chênh lệch giữa bộ lọc nhanh và chậm

    cv::Mat fast_filter_; // Lưu trạng thái bộ lọc nhanh
    cv::Mat slow_filter_; // Lưu trạng thái bộ lọc chậm
};

#endif // BACKGROUND_SUBTRACTOR_HPP
