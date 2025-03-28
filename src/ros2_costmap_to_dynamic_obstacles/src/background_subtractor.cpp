#include <opencv2/opencv.hpp>  
#include <background_subtractor.h>

BackgroundSubtractor::BackgroundSubtractor(const Params &parameters): params_(parameters)
{
}

void BackgroundSubtractor::apply(const cv::Mat& image, cv::Mat& fg_mask, int shift_x, int shift_y)
{
  // std::cout << "[INFO] Applying background subtraction..." << std::endl;
  current_frame_ = image;
  // visualize("1. Original Input Frame", current_frame_);

  // Initialize occupancy grids on the first frame
  if (occupancy_grid_fast_.empty() && occupancy_grid_slow_.empty())
  {
    // std::cout << "[INFO] Initializing occupancy grids..." << std::endl;
    occupancy_grid_fast_ = current_frame_;
    occupancy_grid_slow_ = current_frame_;
    previous_shift_x_ = shift_x;
    previous_shift_y_ = shift_y;
    // visualize("2. Initialized Occupancy Grid", occupancy_grid_fast_);
    return;
  }

  // Compute relative shift
  int shift_relative_x = shift_x - previous_shift_x_;
  int shift_relative_y = shift_y - previous_shift_y_;
  previous_shift_x_ = shift_x;
  previous_shift_y_ = shift_y;

  if (shift_relative_x != 0 || shift_relative_y != 0)
  {
    // std::cout << "[INFO] Translating occupancy grids by (" << shift_relative_x << ", " << shift_relative_y << ")" << std::endl;
    transformToCurrentFrame(shift_relative_x, shift_relative_y);
    // visualize("3. Transformed Occupancy Grids (After Shift)", occupancy_grid_fast_);
  }

  // Calculate mean of nearest neighbors using box filter
  int size = 3;  // Kernel size for neighbor mean computation
  cv::Mat nearest_mean_fast, nearest_mean_slow;
  cv::boxFilter(occupancy_grid_fast_, nearest_mean_fast, -1, cv::Size(size, size));
  cv::boxFilter(occupancy_grid_slow_, nearest_mean_slow, -1, cv::Size(size, size));
  // visualize("4. Nearest Neighbor Mean (Fast Grid)", nearest_mean_fast);
  // visualize("5. Nearest Neighbor Mean (Slow Grid)", nearest_mean_slow);

  // Update the occupancy grids using addWeighted
  // std::cout << "[INFO] Updating occupancy grids..." << std::endl;
  cv::addWeighted(current_frame_, params_.alpha_fast, occupancy_grid_fast_, (1 - params_.alpha_fast), 0, occupancy_grid_fast_);
  cv::addWeighted(occupancy_grid_fast_, params_.beta, nearest_mean_fast, (1 - params_.beta), 0, occupancy_grid_fast_);
  cv::addWeighted(current_frame_, params_.alpha_slow, occupancy_grid_slow_, (1 - params_.alpha_slow), 0, occupancy_grid_slow_);
  cv::addWeighted(occupancy_grid_slow_, params_.beta, nearest_mean_slow, (1 - params_.beta), 0, occupancy_grid_slow_);
  // visualize("6. Updated Fast Occupancy Grid", occupancy_grid_fast_);
  // visualize("7. Updated Slow Occupancy Grid", occupancy_grid_slow_);

  // Apply thresholds for obstacle detection
  cv::threshold(occupancy_grid_fast_, occupancy_grid_fast_, params_.min_occupancy_probability, 0, cv::THRESH_TOZERO);
  // visualize("8. Thresholded Fast Grid (Static Obstacle Detection)", occupancy_grid_fast_);

  cv::threshold(occupancy_grid_fast_ - occupancy_grid_slow_, fg_mask, params_.min_sep_between_fast_and_slow_filter, 255, cv::THRESH_BINARY);
  // visualize("9. Foreground Mask (Fast - Slow Grid Difference)", fg_mask);

  cv::threshold(nearest_mean_slow, nearest_mean_slow, params_.max_occupancy_neighbors, 255, cv::THRESH_BINARY_INV);
  // visualize("10. Inverse Threshold (Filter Static Obstacles)", nearest_mean_slow);

  cv::bitwise_and(nearest_mean_slow, fg_mask, fg_mask);
  // visualize("11. Final Foreground Mask (After Filtering Static Obstacles)", fg_mask);

  // Set image border to zero to handle boundary artifacts
  int border = 5;
  cv::Mat border_mask(current_frame_.size(), CV_8UC1, cv::Scalar(0));
  border_mask(cv::Rect(border, border, current_frame_.cols - 2 * border, current_frame_.rows - 2 * border)) = 255;
  cv::bitwise_and(border_mask, fg_mask, fg_mask);
  // visualize("12. Foreground Mask (After Border Cropping)", fg_mask);

  // Apply closing (morphological operations)
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                              cv::Size(2 * params_.morph_size + 1, 2 * params_.morph_size + 1),
                                              cv::Point(params_.morph_size, params_.morph_size));
  cv::dilate(fg_mask, fg_mask, element);
  cv::dilate(fg_mask, fg_mask, element);
  cv::erode(fg_mask, fg_mask, element);
  // visualize("13. Foreground Mask (After Morphological Closing)", fg_mask);
}

void BackgroundSubtractor::transformToCurrentFrame(int shift_x, int shift_y)
{
  cv::Mat temp_fast, temp_slow;
  cv::Mat translation_mat = (cv::Mat_<double>(2, 3, CV_64F) << 1, 0, -shift_x, 0, 1, -shift_y);
  cv::warpAffine(occupancy_grid_fast_, temp_fast, translation_mat, occupancy_grid_fast_.size());
  cv::warpAffine(occupancy_grid_slow_, temp_slow, translation_mat, occupancy_grid_slow_.size());

  occupancy_grid_fast_ = temp_fast;
  occupancy_grid_slow_ = temp_slow;
}

// Debug visualization function (commented by default)
void BackgroundSubtractor::visualize(const std::string& name, const cv::Mat& image)
{
  if (!image.empty())
  {
    std::cout << "[DEBUG] Visualizing: " << name << std::endl;
    cv::Mat im = image.clone();
    cv::flip(im, im, 0);  // Flip image vertically
    cv::imshow(name, im);
    cv::waitKey(1);
  }
}

void BackgroundSubtractor::updateParameters(const Params &parameters)
{
  params_ = parameters;
}
