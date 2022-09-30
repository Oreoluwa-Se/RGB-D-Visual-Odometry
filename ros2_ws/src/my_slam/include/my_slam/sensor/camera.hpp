#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <common.hpp>

namespace sensor {
class Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Camera> Ptr;

  Camera(const std::vector<double> &calib_coeff,
         const std::vector<double> &distort_coeff, double baseline,
         const SE3 &pose);

  // functions
  matrix<3> K() const { // fx, fy, cx, cy
    // returns the intrinsic matrix
    matrix<3> K = matrix<3>::Zero();
    K(0, 0) = _calib_coeff[0];
    K(1, 1) = _calib_coeff[1];
    K(0, 2) = _calib_coeff[2];
    K(1, 2) = _calib_coeff[3];
    K(2, 2) = 1;

    return K;
  }

  vector<2> distort(const vector<2> &point);
  bool undistort_pixel(const vector<2> &point, vector<2> &un_dist_point);
  vector<2> remove_intrinsic(const vector<2> &point);
  vector<2> add_intrinsic(const vector<2> &point);
  bool isValidPixel(const vector<2> &point);

  // coordinate transforms: world, camera, pixel
  vector<3> world2camera(const vector<3> &p_w, const SE3 &T_c_w);
  vector<3> camera2world(const vector<3> &p_c, const SE3 &T_c_w);
  bool camera2pixel(const vector<3> &p_c, vector<2> &p_p);
  bool pixel2camera(const vector<2> &p_p, vector<3> &ray);
  bool world2pixel(const vector<3> &p_w, const SE3 &T_c_w, vector<2> &p_p);
  bool pixel2world(const vector<2> &p_p, const SE3 &T_c_w,
                   vector<3> &world_point, double depth = 1);

  void setWidthHeight(int width, int height) {
    _width = width;
    _height = height;
  }

  int GetWidth() const { return _width; }

  SE3 Pose() const { return _pose; }

  SE3 PoseInv() const { return _pose_inv; }

private:
  /*-----------------------------------------------------------------*/
  // fx, fy, cx, cy
  std::unique_ptr<double[]> _calib_coeff = std::make_unique<double[]>(4);
  // K1, K2, T1, T2
  std::unique_ptr<double[]> _distort_coeff = std::make_unique<double[]>(2);
  // double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
  bool _distorted_image = false;
  double _baseline = 0.0;
  // image attributes
  int _width, _height;

  /*
      - Defined as T_c_f [from frame to single camera]
      - Defines camera installed location on device
      - Extrinsic parameters
   */
  SE3 _pose;
  SE3 _pose_inv;
};
} // namespace sensor
#endif