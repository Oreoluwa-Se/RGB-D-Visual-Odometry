#include <helper_modules.hpp>
#include <my_slam/sensor/camera.hpp>

namespace sensor {
Camera::Camera(const std::vector<double> &calib_coeff,
               const std::vector<double> &distort_coeff, double baseline,
               const SE3 &pose)
    : _baseline(baseline), _width(0.0), _height(0.0), _pose(pose),
      _pose_inv(pose.inverse())

{
  /* ----------------------------------------- */
  for (size_t i = 0; i < calib_coeff.size(); ++i) {
    _calib_coeff[i] = calib_coeff[i];
  }

  if (!distort_coeff.empty()) {
    for (size_t i = 0; i < distort_coeff.size(); ++i) {
      _distort_coeff[i] = distort_coeff[i];
    }
  }

  // assumes image is distorted if we pass in coordinates with distortion
  // coefficients
  auto check = std::count(distort_coeff.begin(), distort_coeff.end(), 0) ==
               int(distort_coeff.size());
  if (check && !distort_coeff.empty())
    _distorted_image = true;
}

vector<2> Camera::add_intrinsic(const vector<2> &point) {
  double u_dist = point.x() * _calib_coeff[0] + _calib_coeff[2];
  double v_dist = point.y() * _calib_coeff[1] + _calib_coeff[3];

  return vector<2>(u_dist, v_dist);
}

vector<2> Camera::remove_intrinsic(const vector<2> &point) {
  // fx, fy, cx, cy
  double x = (point.x() - _calib_coeff[2]) / _calib_coeff[0];
  double y = (point.y() - _calib_coeff[3]) / _calib_coeff[1];

  return vector<2>(x, y);
}

bool Camera::undistort_pixel(const vector<2> &point, vector<2> &un_dist_point) {
  if (!_distorted_image) {
    un_dist_point = point;
    return true;
  }
  /*Steps:
      - pixel location is distorted
      - convert to image plane and find the actual location [where would be with
     distortion].
      - convert to pixel location
  */
  auto distorted = point;
  distorted = Camera::remove_intrinsic(distorted);
  distorted = Camera::distort(distorted);
  un_dist_point = Camera::add_intrinsic(point);
  if (isValidPixel(un_dist_point))
    return true;

  return false;
}

vector<2> Camera::distort(const vector<2> &point) {
  double K1 = _distort_coeff[0];
  double K2 = _distort_coeff[1];
  double T1 = 0.0, T2 = 0.0;

  auto radius = sqrt(pow(point.x(), 2) + pow(point.y(), 2));
  auto x_dist = point.x() * (1 + K1 * pow(radius, 2) + K2 * pow(radius, 4)) +
                2 * T1 * point.x() * point.y() +
                T2 * (pow(radius, 2) + 2 * point.x() * point.x());
  auto y_dist = point.y() * (1 + K1 * pow(radius, 2) + K2 * pow(radius, 4)) +
                T1 * (pow(radius, 2) + 2 * point.y() * point.y()) +
                2 * T2 * point.x() * point.y();

  return vector<2>(x_dist, y_dist);
}

bool Camera::isValidPixel(const vector<2> &point) {
  if (_width < 0 || _height < 0) {
    error_check(_width > 0, "Image width should be greater than zero",
                ErrorType::FATAL);
    error_check(_height > 0, "Image height should be greater than zero",
                ErrorType::FATAL);
  }
  int x = std::round(point.x()), y = std::round(point.y());
  return x >= 0 && x < _width && y >= 0 && y < _height;
}

// coordinate transforms: world, camera, pixel
vector<3> Camera::world2camera(const vector<3> &p_w, const SE3 &T_f_w) {
  return _pose * T_f_w * p_w;
}

vector<3> Camera::camera2world(const vector<3> &p_c, const SE3 &T_f_w) {
  return T_f_w.inverse() * _pose_inv * p_c;
}

bool Camera::camera2pixel(
    const vector<3> &p_c,
    vector<2> &p_p) { // p_c here is the projected point after using extrinsics
  auto point = vector<2>(p_c.x() / p_c.z(), p_c.y() / p_c.z());
  Camera::add_intrinsic(point);

  return isValidPixel(point);
}

bool Camera::pixel2camera(const vector<2> &p_p, vector<3> &ray) {
  vector<2> pixel;
  if (Camera::undistort_pixel(p_p, pixel)) {
    pixel = Camera::remove_intrinsic(pixel);
    ray.x() = pixel.x();
    ray.y() = pixel.y();
    ray.z() = 1;
    return true;
  }
  return false;
}

bool Camera::world2pixel(const vector<3> &p_w, const SE3 &T_f_w,
                         vector<2> &p_p) {
  auto img_plane = world2camera(p_w, T_f_w);
  img_plane /= img_plane.z();

  if (_distorted_image) {
    auto dist = distort(vector<2>(img_plane.x(), img_plane.y()));
    img_plane.x(), img_plane.y() = dist.x(), dist.y();
  }

  return camera2pixel(img_plane, p_p);
}

bool Camera::pixel2world(const vector<2> &p_p, const SE3 &T_f_w,
                         vector<3> &world_point, double depth) {
  vector<3> ray;
  if (!pixel2camera(p_p, ray))
    return false;

  ray *= depth;
  world_point = camera2world(ray, T_f_w);

  return true;
}

} // namespace sensor
