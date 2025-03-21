#include "blob_detector.hpp"

BlobDetector::BlobDetector(float min_area, float max_area, int min_threshold, int max_threshold) {
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = min_area;
    params.maxArea = max_area;
    params.minThreshold = min_threshold;
    params.maxThreshold = max_threshold;

    detector_ = cv::SimpleBlobDetector::create(params);
}

void BlobDetector::detect(const cv::Mat& image, 
                          std::vector<cv::KeyPoint>& keypoints_, 
                          std::vector<std::vector<cv::Point>>& contours) {
    if (!detector_ || image.empty()) {
        return; // Thoát nếu detector hoặc ảnh không hợp lệ
    }

    // Bước 1: Dùng SimpleBlobDetector để lấy keypoints (centroids)
    detector_->detect(image, keypoints_);

    // Bước 2: Chuẩn bị ảnh nhị phân để tìm contours
    cv::Mat binary_image;
    cv::threshold(image, binary_image, (min_threshold + max_threshold) / 2, 255, cv::THRESH_BINARY);

    // Bước 3: Tìm contours
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Bước 4 (tùy chọn): Lọc contours để khớp với keypoints
    // Lưu ý: Số lượng contours có thể không khớp với số lượng keypoints
    // Ở đây, ta có thể lọc contours dựa trên vị trí gần với keypoints
    std::vector<std::vector<cv::Point>> filtered_contours;
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Moments m = cv::moments(contours[i]);
        if (m.m00 != 0) {
            float cx = m.m10 / m.m00; // Tọa độ x của trung tâm contour
            float cy = m.m01 / m.m00; // Tọa độ y của trung tâm contour

            // Kiểm tra xem contour này có gần với bất kỳ keypoint nào không
            for (const auto& kp : keypoints_) {
                float dist = cv::norm(cv::Point2f(cx, cy) - kp.pt);
                if (dist < kp.size / 2) { // Nếu trung tâm contour gần với keypoint
                    filtered_contours.push_back(contours[i]);
                    break;
                }
            }
        }
    }
    contours = filtered_contours; // Gán lại contours đã lọc
}