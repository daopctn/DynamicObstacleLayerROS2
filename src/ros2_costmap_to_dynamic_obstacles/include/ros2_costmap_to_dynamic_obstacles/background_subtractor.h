
 #ifndef BACKGROUNDSUBTRACTOR_H_
 #define BACKGROUNDSUBTRACTOR_H_
 
#include <opencv2/opencv.hpp>
 /**
  * @class BackgroundSubtractor
  * @brief Perform background subtraction to extract the "moving" foreground
  *
  * This class is based on OpenCV's background subtraction class cv::BackgroundSubtractor.
  * It has been modified in order to incorporate a specialized bandpass filter.
  *
  * See http://docs.opencv.org/3.2.0/d7/df6/classcv_1_1BackgroundSubtractor.html for the original class.
  */
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
   };
 
   //! Constructor that accepts a specific parameter configuration
   BackgroundSubtractor(const Params& parameters);
 
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
   void writeMatToYAML(const std::string& filename, const std::vector<cv::Mat>& mat_vec);
 
   //! Update internal parameters
   void updateParameters(const Params& parameters);
 
 private:
   //! Transform/shift all internal matrices/grids according to a given translation vector
   void transformToCurrentFrame(int shift_x, int shift_y);
 
   cv::Mat occupancy_grid_fast_;
   cv::Mat occupancy_grid_slow_;
   cv::Mat current_frame_;
 
   int previous_shift_x_;
   int previous_shift_y_;
 
   Params params_;
 };
 
 #endif // BACKGROUNDSUBTRACTOR_H_

