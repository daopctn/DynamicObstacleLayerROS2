#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <memory>
#include <chrono>  // Để sử dụng timer
#include <opencv2/opencv.hpp>
#include <background_subtractor.h>
#include <blob_detector.h>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>
#include <nav2_dynamic_msgs/msg/obstacle.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>  // Thêm thư viện UUID
#include <random>  // Thêm thư viện random
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
class CostmapConverterNode : public rclcpp::Node {
public:
  CostmapConverterNode() : Node("custom_costmap_node") {
    RCLCPP_INFO(this->get_logger(), "CustomCostmapNode created.");

    // Khởi tạo Costmap2DROS
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("custom_local_costmap");

    // Cấu hình Costmap2DROS
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());

    // Thiết lập thông số cho Background Subtractor
    BackgroundSubtractor::Params bg_params;
    bg_params.alpha_fast = 0.7;
    bg_params.alpha_slow = 0.1;
    bg_params.beta = 0.8;
    bg_params.min_occupancy_probability = 100;
    bg_params.min_sep_between_fast_and_slow_filter = 40;
    bg_params.max_occupancy_neighbors = 100;
    bg_params.morph_size = 0;
    
    background_subtractor_ = std::make_shared<BackgroundSubtractor>(bg_params);

    // Thiết lập thông số cho Blob Detector
    BlobDetector::Params blob_params;
    blob_params.filterByColor = true;
    blob_params.blobColor = 255;
    blob_params.minDistBetweenBlobs = 10.0f;
    blob_params.filterByArea = true;
    blob_params.minArea = 3.0f;
    blob_params.maxArea = 100.0f;
    blob_params.filterByCircularity = true;
    blob_params.minCircularity = 0.0f;
    blob_params.maxCircularity = 1.0f;
    blob_params.filterByInertia = true;
    blob_params.minInertiaRatio = 0.0f;
    blob_params.maxInertiaRatio = 1.0f;
    blob_params.filterByConvexity = true;
    blob_params.minConvexity = 0.0f;
    blob_params.maxConvexity = 1.0f;
    
    blob_detector_ = BlobDetector::create(blob_params);

    // Tạo timer để xử lý Costmap2D và tính toán foreground mask mỗi 150ms
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(150), 
      std::bind(&CostmapConverterNode::publishCallback, this)
    );
    // Tạo publisher cho visualization marker
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);
    // Tạo publisher cho obstacle array
    obstacle_pub_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("detection", 10);
  }

  ~CostmapConverterNode() {
    costmap_ros_.reset();
  }

private:
  void publishCallback()
  {
    // Lấy Costmap2D từ Costmap2DROS
    costmap_ = costmap_ros_->getCostmap();
    if (!costmap_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get Costmap2D from Costmap2DROS.");
      return;
    }

    // Chuyển đổi Costmap2D thành cv::Mat
    cv::Mat costmap_mat(costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX(), CV_8UC1);
    for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); ++y) {
      unsigned char* row_ptr = costmap_mat.ptr<unsigned char>(y);
      for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); ++x) {
        row_ptr[x] = costmap_->getCost(x, y);
      }
    }
    // cv::flip(costmap_mat, costmap_mat, 0);  // Lật ma trận theo trục X

    // Khởi tạo fg_mask với cùng kích thước
    cv::Mat fg_mask(costmap_mat.size(), CV_8UC1, cv::Scalar(0));

    // Tính toán origin trong đơn vị cell
    int origin_x = static_cast<int>(costmap_->getOriginX() / costmap_->getResolution());
    int origin_y = static_cast<int>(costmap_->getOriginY() / costmap_->getResolution());

    // Áp dụng Background Subtraction
    background_subtractor_->apply(costmap_mat, fg_mask, origin_x, origin_y);

    // Phát hiện blobs
    std::vector<cv::KeyPoint> keypoints;
    blob_detector_->detect(fg_mask, keypoints);
    auto& contours = blob_detector_->getContours();  // Lấy contours đã tính toán

    // Hiển thị foreground mask và kết quả blob detection
    cv::Mat fg_mask_color;
    cv::cvtColor(fg_mask, fg_mask_color, cv::COLOR_GRAY2BGR);
    cv::drawKeypoints(fg_mask_color, keypoints, fg_mask_color, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    for (const auto& contour : contours) {
      cv::drawContours(fg_mask_color, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
    }
    // Hiển thị kết quả
    cv::imshow("Costmap", costmap_mat);
    cv::imshow("Foreground Mask", fg_mask);
    cv::imshow("Blob Detector", fg_mask_color);
    cv::waitKey(30);
    // Chuyển đổi keypoints thành ObstacleArray
    obstacle_array_.header.frame_id = costmap_ros_->getGlobalFrameID();
    obstacle_array_.header.stamp = now();
    obstacle_array_.obstacles.clear();
    for (const auto& keypoint : keypoints) {
      nav2_dynamic_msgs::msg::Obstacle obstacle;
      obstacle.position.x = keypoint.pt.x * costmap_->getResolution() + costmap_->getOriginX();
      obstacle.position.y = keypoint.pt.y * costmap_->getResolution() + costmap_->getOriginY();
      obstacle.size.x = keypoint.size * costmap_->getResolution();
      obstacle.size.y = keypoint.size * costmap_->getResolution();
      //gans UUID cho obstacle
      obstacle.id = generateRandomUUID();
      
      obstacle_array_.obstacles.push_back(obstacle);
    }
    //publish obstacle array
    obstacle_pub_->publish(obstacle_array_);
    //publish marker
    publishAsMarker("map", obstacle_array_);
    

  }
  // Hàm tạo UUID ngẫu nhiên
  unique_identifier_msgs::msg::UUID generateRandomUUID()
  {
    unique_identifier_msgs::msg::UUID uuid;

    // Sử dụng random engine và phân phối để tạo các byte ngẫu nhiên cho UUID
    std::random_device rd;  // Thiết lập thiết bị sinh số ngẫu nhiên
    std::mt19937 gen(rd());  // Khởi tạo random engine
    std::uniform_int_distribution<int> dist(0, 255);  // Phân phối từ 0 đến 255

    // Gán các byte ngẫu nhiên cho UUID
    for (int i = 0; i < 16; ++i) {
        uuid.uuid[i] = dist(gen);  // Gán giá trị ngẫu nhiên từ phân phối
    }

    return uuid;
  }
  void publishAsMarker(
    const std::string &frame_id,
    const nav2_dynamic_msgs::msg::ObstacleArray &obstacles) {
    visualization_msgs::msg::MarkerArray box;
    for (auto obstacle : obstacles.obstacles) {
    box.markers.emplace_back();
    box.markers.back().header.frame_id = frame_id;
    box.markers.back().header.stamp = now();
    box.markers.back().ns = "Obstacles Markers";
    box.markers.back().type =
    visualization_msgs::msg::Marker::CUBE;
    box.markers.back().action =
    visualization_msgs::msg::Marker::ADD;
    box.markers.back().pose.orientation.w = 1.0;
    box.markers.back().pose.position = obstacle.position;
    box.markers.back().scale = obstacle.size;
    box.markers.back().scale.z = 0.01;
    box.markers.back().color.r = 0.0;
    box.markers.back().color.g = 1.0;
    box.markers.back().color.b = 0.0;
    box.markers.back().color.a = 0.5;
    }
    marker_pub_->publish(box);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;
  //create obstacle array
  nav2_dynamic_msgs::msg::ObstacleArray obstacle_array_;
  //create background subtractor and blob detector
  std::shared_ptr<BackgroundSubtractor> background_subtractor_;
  std::shared_ptr<BlobDetector> blob_detector_;
  //create marker publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  //obstacle array publisher
  rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CostmapConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}