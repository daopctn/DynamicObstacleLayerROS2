#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <memory>
#include <chrono>  // Để sử dụng timer
#include <opencv2/opencv.hpp>
#include <background_subtractor.h>
#include <blob_detector.h>
class CostmapConverterNode : public rclcpp::Node {
public:
  CostmapConverterNode() : Node("custom_costmap_node") {
    RCLCPP_INFO(this->get_logger(), "CustomCostmapNode created.");

    // Khởi tạo Costmap2DROS
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("custom_local_costmap");

    // Cấu hình Costmap2DROS
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());

    // Lấy Costmap2D và in kích thước
    auto costmap = costmap_ros_->getCostmap();
    if (costmap) {
      unsigned int width = costmap->getSizeInCellsX();
      unsigned int height = costmap->getSizeInCellsY();
      RCLCPP_INFO(this->get_logger(), "Custom costmap size: width = %u, height = %u", width, height);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to get Costmap2D instance.");
    }

    // Khởi tạo background_subtractor với tham số mặc định
    BackgroundSubtractor::Params params;
    params.alpha_fast = 0.7;   // Decrease: Fast grid reacts more quickly to moving objects.
    params.alpha_slow = 0.1;   // Decrease: Slow grid becomes more stable and better differentiates slow and fast movements.
    params.beta = 0.8;         // Slightly decrease: Reduce bias toward the neighbor mean to prevent "blurring" small objects.
    params.min_occupancy_probability = 100;  // Lower: Detect smaller moving objects by reducing the static obstacle threshold.
    params.min_sep_between_fast_and_slow_filter = 40;  // Lower: Increase sensitivity to smaller differences between fast and slow grids.
    params.max_occupancy_neighbors = 100;  // Increase: Allow slightly larger local variations in the slow occupancy grid.
    params.morph_size = 0;  // Decrease: No morphological dilation/erosion to prevent masking out small obstacles.
    background_subtractor = std::make_shared<BackgroundSubtractor>(params);
    BlobDetector::Params blob_params;
    blob_params.filterByColor = true;
    blob_params.blobColor = 255;
    blob_params.minDistBetweenBlobs = 10.0f;
    blob_params.filterByArea = true;
    blob_params.minArea = 3.0f;
    blob_params.maxArea = 1000.0f;
    blob_params.filterByCircularity = true;
    blob_params.minCircularity = 0.0f;
    blob_params.maxCircularity = 1.0f;
    blob_params.filterByInertia = true;
    blob_params.minInertiaRatio = 0.0f;
    blob_params.maxInertiaRatio = 1.0f;
    blob_params.filterByConvexity = true;
    blob_params.minConvexity = 0.0f;
    blob_params.maxConvexity = 1.0f;
    blob_det_ = BlobDetector::create(blob_params);
    // Tạo timer để xử lý Costmap2D và tính toán foreground mask mỗi 150ms
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(150), 
      [this]() {
        auto costmap = costmap_ros_->getCostmap();
        if (costmap) {
          // Chuyển Costmap2D thành ảnh OpenCV (với ma trận costmap_mat)
          unsigned int width = costmap->getSizeInCellsX();
          unsigned int height = costmap->getSizeInCellsY();
          
          costmap_mat = cv::Mat(height, width, CV_8UC1);
          for (unsigned int y = 0; y < height; ++y) {
            unsigned char* row_ptr = costmap_mat.ptr<unsigned char>(y);
            for (unsigned int x = 0; x < width; ++x) {
              row_ptr[x] = costmap->getCost(x, y);  // Lấy giá trị cost và gán vào ma trận OpenCV
            }
          }

          cv::flip(costmap_mat, costmap_mat, 0);  // Lật ma trận theo trục X để tương thích hình ảnh
          
          // Khởi tạo fg_mask với cùng kích thước
          fg_mask = cv::Mat(costmap_mat.size(), CV_8UC1, cv::Scalar(0));
          
          // Tính toán origin trong đơn vị cell (nếu cần)
          int origin_x = static_cast<int>(costmap->getOriginX() / costmap->getResolution());
          int origin_y = static_cast<int>(costmap->getOriginY() / costmap->getResolution());
          
          // Áp dụng Background Subtraction
          background_subtractor->apply(costmap_mat, fg_mask, origin_x, origin_y);
          blob_det_->detect(fg_mask, keypoints_);
          // Hiển thị foreground mask và bản đồ costmap
          cv::namedWindow("Costmap", cv::WINDOW_NORMAL);
          cv::resizeWindow("Costmap", 800, 600);
          cv::imshow("Costmap", costmap_mat);

          cv::namedWindow("Foreground Mask", cv::WINDOW_NORMAL);
          cv::resizeWindow("Foreground Mask", 800, 600);
          cv::imshow("Foreground Mask", fg_mask);

          cv::namedWindow("Blob Detector", cv::WINDOW_NORMAL);
          cv::resizeWindow("Blob Detector", 800, 600);
          cv::Mat fg_mask_color;
          cv::cvtColor(fg_mask, fg_mask_color, cv::COLOR_GRAY2BGR);
          cv::drawKeypoints(fg_mask_color, keypoints_, fg_mask_color, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          cv::drawContours(fg_mask_color, blob_det_->getContours(), -1, cv::Scalar(0, 255, 0), 2);
          cv::imshow("Blob Detector", fg_mask_color);
          cv::waitKey(30);  // Giảm lag và cho phép vẽ cửa sổ
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to get Costmap2D instance.");
        }
      }
    );
  }

  ~CostmapConverterNode() {
    // Reset Costmap khi node tắt
    costmap_ros_.reset();
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::TimerBase::SharedPtr timer_;  // Con trỏ tới timer
  std::shared_ptr<BackgroundSubtractor> background_subtractor;  // Con trỏ đến bộ trích lọc background
  cv::Mat fg_mask;  // Ma trận mask foreground
  cv::Mat costmap_mat;  // Ma trận OpenCV đại diện cho costmap
  std::vector<cv::KeyPoint> keypoints_;  // Danh sách keypoints từ blob detector
  std::vector<std::vector<cv::Point>> contours_;  // Danh sách contours từ blob detector
  std::shared_ptr<BlobDetector> blob_det_;  // Con trỏ đến blob detector
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CostmapConverterNode>();

  rclcpp::spin(node);  // Giữ cho node hoạt động
  rclcpp::shutdown();

  return 0;
}
