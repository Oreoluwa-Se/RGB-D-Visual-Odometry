#ifndef IMAGE_FUNCS_HPP
#define IMAGE_FUNCS_HPP

// image extraction, feature detection, and matching in some sense
#include <common.hpp>
#include <my_slam/utils/poission_filter.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

namespace utils { // basic components for image, loading, feature
                  // detection,
class ImageFunctions {
public:
  static std::shared_ptr<ImageFunctions> Ptr;
  static cv::Mat load_image(const std::string &loc,
                            const std::string &load_type);

  // applying histogram equalization
  static cv::Mat apply_histeq(cv::Mat &&img);

  // combines feature extraction with randomly selected features
  static void feature_extraction(std::vector<vector<2>> &initial_kps,
                                 const cv::Mat &img, cv::Mat &desc,
                                 size_t max_points, const double kp_radius,
                                 const double &boarder, const double scale_f,
                                 const int lvl_pyramid, bool return_desc);

  // extracts randomly selected features
  static void rand_points_selected(const cv::Mat &img, size_t points,
                                   int boarder, const double kp_radius,
                                   std::vector<vector<2>> &out);

  // converts eigen to cv keypoints
  static std::vector<cv::KeyPoint> eig_2_cvkp(const std::vector<vector<2>> &v);

  // bilinear interpolation
  static float GetPixelValue(const cv::Mat &img, float x, float y);

  static void build();

  // destructor
  ~ImageFunctions();

private:
  ImageFunctions() {}
};
} // namespace utils

#endif