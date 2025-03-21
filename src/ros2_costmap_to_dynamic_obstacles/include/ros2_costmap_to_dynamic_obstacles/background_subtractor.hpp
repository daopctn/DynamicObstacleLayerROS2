#ifndef BACKGROUND_SUBTRACTOR_HPP
#define BACKGROUND_SUBTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class BackgroundSubtractor {
public:
    // Constructor mặc định
    BackgroundSubtractor();
    // Constructor có tham số
    BackgroundSubtractor(float alpha_fast, float alpha_slow, float beta, float threshold1, float threshold2);
    
    void apply(const cv::Mat& costmap, cv::Mat& fg_mask);

private:
    float alpha_fast_;  // Hệ số cập nhật của bộ lọc nhanh
    float alpha_slow_;  // Hệ số cập nhật của bộ lọc chậm
    float beta_;        // Ảnh hưởng của hàng xóm gần nhất
    float threshold1_;  // Ngưỡng kích hoạt bộ lọc nhanh
    float threshold2_;  // Ngưỡng chênh lệch giữa bộ lọc nhanh và chậm

    cv::Mat fast_filter_; // Lưu trạng thái bộ lọc nhanh
    cv::Mat slow_filter_; // Lưu trạng thái bộ lọc chậm
};

#endif // BACKGROUND_SUBTRACTOR_HPP