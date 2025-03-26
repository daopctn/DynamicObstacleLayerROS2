#ifndef BACKGROUND_SUBTRACTOR_H_
#define BACKGROUND_SUBTRACTOR_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class BackgroundSubtractor
{
public:
  struct Params
  {
    double alpha_slow; //!< Filter constant (learning rate) of the slow filter part
    double alpha_fast; //!< Filter constant (learning rate) of the fast filter part
    double beta;
    double min_sep_between_fast_and_slow_filter;
    double min_occupancy_probability;
    double max_occupancy_neighbors;
    int morph_size;

    Params() : alpha_slow(0.1), alpha_fast(0.9), beta(1.0),
               min_sep_between_fast_and_slow_filter(0.1),
               min_occupancy_probability(0.5),
               max_occupancy_neighbors(2.0),
               morph_size(3) {}
  };

  //! Constructor that accepts a specific parameter configuration
  BackgroundSubtractor(const Params& parameters = Params());

  /**
   * @brief Computes a foreground mask
   * @param[in]  image    Next video frame
   * @param[out] fg_mask  Foreground mask as an 8-bit binary image
   * @param[in]  shift_x  Translation along the x axis between the current and previous image
   * @param[in]  shift_y  Translation along the y axis between the current and previous image
   */
  void apply(const cv::Mat& image, cv::Mat& fg_mask, int shift_x = 0, int shift_y = 0);

  /**
   * @brief OpenCV Visualization
   * @param name  Id/name of the opencv window
   * @param image Image to be visualized
   */
  void visualize(const std::string& name, const cv::Mat& image);

  /**
   * @brief Export vector of matrices to yaml file
   * @remarks This method is intended for debugging purposes
   * @param filename   Desired filename including path and excluding file suffix
   * @param mat_vec    Vector of cv::Mat to be exported
   */
  

  //! Update internal parameters
  void updateParameters(const Params& parameters);

private:
  void transformToCurrentFrame(int shift_x, int shift_y);

  cv::Mat occupancy_grid_fast_;
  cv::Mat occupancy_grid_slow_;
  cv::Mat current_frame_;

  int previous_shift_x_;
  int previous_shift_y_;

  Params params_;
};

#endif // BACKGROUND_SUBTRACTOR_H_