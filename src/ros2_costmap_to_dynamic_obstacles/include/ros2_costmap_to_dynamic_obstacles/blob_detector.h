#ifndef BLOBDETECTOR_H_
#define BLOBDETECTOR_H_

// Basically the OpenCV SimpleBlobDetector, extended with getContours()

#include <opencv2/opencv.hpp>
#include <vector>
/**
 * @class BlobDetector
 * @brief Detect blobs in image (specialized for dynamic obstacles in the costmap)
 *
 * This class is based on OpenCV's blob detector cv::SimpleBlobDetector.
 * It has been modified and specialized for dynamic obstacle tracking in the costmap:
 * -> The modified version also returns contours of the blob.
 *
 * See http://docs.opencv.org/trunk/d0/d7a/classcv_1_1SimpleBlobDetector.html for the original class.
 */
class BlobDetector : public cv::SimpleBlobDetector
{
public:
  // Sử dụng Params từ cv::SimpleBlobDetector nhưng thêm constructor mặc định
  using Params = cv::SimpleBlobDetector::Params;

  //! Default constructor which optionally accepts custom parameters
  BlobDetector(const Params& parameters = Params());

  //! Create shared instance of the blob detector with given parameters
  static cv::Ptr<BlobDetector> create(const Params& params);

  /**
   * @brief Detects keypoints in an image and extracts contours
   *
   * In contrast to the original detect method, this extended version
   * also extracts contours. Contours can be accessed by getContours()
   * after invoking this method.
   *
   * @todo The mask option is currently not implemented.
   *
   * @param image     image
   * @param keypoints The detected keypoints.
   * @param mask      Mask specifying where to look for keypoints (optional). It must be a 8-bit integer
   *                  matrix with non-zero values in the region of interest.
   */
  virtual void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                      const cv::Mat& mask = cv::Mat());

  /**
   * @brief Access contours extracted during detection stage
   * @return Read-only reference to the contours set of the previous detect() run
   */
  const std::vector<std::vector<cv::Point>>& getContours() { return contours_; }

  //! Update internal parameters
  void updateParameters(const Params& parameters);

protected:
  struct Center
  {
    cv::Point2d location;
    double radius;
    double confidence;
  };

  virtual void findBlobs(const cv::Mat& image, const cv::Mat& binary_image, std::vector<Center>& centers,
                         std::vector<std::vector<cv::Point>>& cur_contours) const;

  std::vector<std::vector<cv::Point>> contours_;
  Params params_;
};

#endif // BLOBDETECTOR_H_