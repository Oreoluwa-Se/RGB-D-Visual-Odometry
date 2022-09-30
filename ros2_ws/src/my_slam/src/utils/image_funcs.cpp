#include <helper_modules.hpp>
#include <iostream>
#include <my_slam/utils/image_funcs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/core/core.hpp>

namespace utils {

std::shared_ptr<ImageFunctions> ImageFunctions::Ptr = nullptr;

cv::Mat ImageFunctions::load_image(const std::string &loc,
                                   const std::string &load_type) {
  build();

  auto loc_check = loc != "" || loc != " ";
  error_check(loc_check, "Empty image location provided", ErrorType::FATAL);
  auto l_type = load_type;
  std::transform(l_type.begin(), l_type.end(), l_type.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  cv::Mat img;
  if (l_type == "gray" || l_type == "grey") {
    img = cv::imread(loc, cv::IMREAD_GRAYSCALE);
  } else if (l_type == "color" || l_type == "colour") {
    img = cv::imread(loc, cv::IMREAD_COLOR);
  } else if (l_type == "gray_alt" || l_type == "grey_alt") {
    cv::Mat img_dup;
    cv::Mat img_split[3];
    // ---------------- //
    img = cv::imread(loc, cv::IMREAD_COLOR);
    cv::cvtColor(img, img_dup, cv::COLOR_BGR2Lab);
    cv::split(img_dup, img_split);
    return img_split[0];
  }

  return img;
}

cv::Mat ImageFunctions::apply_histeq(cv::Mat &&img) {
  build();

  cv::Mat dp;

  auto clahe = createCLAHE(5.0, cv::Size(8, 8));

  if (img.channels() == 3) {
    cv::Mat img_split[3], img_dup;
    cv::cvtColor(img, img_dup, cv::COLOR_BGR2Lab);
    cv::split(img_dup, img_split);
    clahe->apply(img_split[0], dp);

  } else if (img.channels() == 1) {
    clahe->apply(img, dp);
  }

  return dp;
}

void ImageFunctions::feature_extraction(
    std::vector<vector<2>> &initial_kps, const cv::Mat &img, cv::Mat &desc,
    size_t max_points, const double kp_radius, const double &boarder,
    const double scale_f, const int lvl_pyramid, bool return_desc) {
  build();

  max_points = max_points == 0 ? 500 : max_points;

  /*-- PHASE ONE - FEATURE EXTRACTION --*/
  std::vector<cv::KeyPoint> key_pts;
  auto detector = cv::ORB::create(int(max_points), scale_f, lvl_pyramid);
  // compute keypoints
  detector->detect(img, key_pts);

  // SAMPLING FILTER
  utils::PoissionFilter<2> filter(kp_radius);
  filter.preset_points(initial_kps); // load previous
  /* ------------------ */
  if (key_pts.size() > 0) {
    std::sort(key_pts.begin(), key_pts.end(), [](const auto &a, const auto &b) {
      return a.response > b.response;
    });

    std::vector<vector<2>> new_pts;
    for (size_t i = 0; i < key_pts.size(); ++i)
      new_pts.emplace_back(key_pts[i].pt.x, key_pts[i].pt.y);

    filter.insert_points(new_pts);
    new_pts.erase(std::remove_if(new_pts.begin(), new_pts.end(),
                                 [&](const auto &kps) {
                                   return kps.x() < boarder ||
                                          kps.y() < boarder ||
                                          kps.x() > img.cols - boarder ||
                                          kps.y() > img.rows - boarder;
                                 }),
                  new_pts.end());

    initial_kps.insert(initial_kps.end(), new_pts.begin(), new_pts.end());
  }

  if (initial_kps.size() < max_points) {
    /*-- PHASE TWO - ADDING RANDOM POINTS --*/
    std::vector<vector<2>> rem;
    ImageFunctions::rand_points_selected(img, max_points, boarder, kp_radius,
                                         rem);
    filter.insert_points(rem);
    initial_kps.insert(initial_kps.end(), rem.begin(), rem.end());
  }

  if (initial_kps.size() > max_points)
    initial_kps.assign(initial_kps.begin(), initial_kps.begin() + max_points);

  if (return_desc) {
    auto kps = eig_2_cvkp(initial_kps);
    detector->compute(img, kps, desc);
  }
}

std::vector<cv::KeyPoint>
ImageFunctions::eig_2_cvkp(const std::vector<vector<2>> &v) {
  if (Ptr == nullptr)
    Ptr = std::shared_ptr<ImageFunctions>(new ImageFunctions);
  std::vector<cv::KeyPoint> r(v.size());
  for (size_t i = 0; i < v.size(); ++i) {
    r[i].pt.x = v[i].x();
    r[i].pt.y = v[i].y();
  }
  return r;
}

void ImageFunctions::rand_points_selected(const cv::Mat &img, size_t num_points,
                                          int boarder, const double kp_radius,
                                          std::vector<vector<2>> &out) {
  build();

  cv::RNG rng;
  // select points and calculate gradient
  std::vector<vector<3>> points;
  std::vector<vector<3>> new_points;
  for (size_t i = 0; i < 2 * num_points; i++) {
    int x = rng.uniform(boarder, img.cols - boarder);
    int y = rng.uniform(boarder, img.rows - boarder);
    auto dx = GetPixelValue(img, x + 1, y) - GetPixelValue(img, x - 1, y);
    auto dy = GetPixelValue(img, x, y + 1) - GetPixelValue(img, x, y - 1);
    points.emplace_back(x, y, abs(dy / dx));
  }

  std::sort(points.begin(), points.end(),
            [](const auto &a, const auto &b) { return a.z() > b.z(); });

  utils::PoissionFilter<3> filter(kp_radius);
  filter.insert_points(points);
  points.erase(std::remove_if(points.begin(), points.end(),
                              [&](const auto &kps) { return kps.z() < 0.7; }),
               points.end());

  new_points.insert(new_points.end(), points.begin(), points.end());

  for (size_t i = 0; i < new_points.size(); i++) {
    out.emplace_back(new_points[i].x(), new_points[i].y());
  }
}

float ImageFunctions::GetPixelValue(const cv::Mat &img, float x, float y) {
  build();

  if (x < 0)
    x = 0;
  if (y < 0)
    y = 0;
  if (x >= img.cols - 1)
    x = img.cols - 2;
  if (y >= img.rows - 1)
    y = img.rows - 2;

  float xx = x - floor(x);
  float yy = y - floor(y);
  int x_a1 = std::min(img.cols - 1, int(x) + 1);
  int y_a1 = std::min(img.rows - 1, int(y) + 1);
  auto out = (1 - xx) * (1 - yy) * img.at<uchar>(y, x) +
             xx * (1 - yy) * img.at<uchar>(y, x_a1) +
             (1 - xx) * yy * img.at<uchar>(y_a1, x) +
             xx * yy * img.at<uchar>(y_a1, x_a1);
  return double(out);
}

// build image_func
void ImageFunctions::build() {
  if (Ptr == nullptr)
    Ptr = std::shared_ptr<ImageFunctions>(new ImageFunctions);
}

ImageFunctions::~ImageFunctions() {}
} // namespace utils
