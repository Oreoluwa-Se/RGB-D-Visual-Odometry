#ifndef FEATURE_HPP
#define FEATURE_HPP

#include <common.hpp>

namespace structures {
struct Frame;
struct MapPoint;

struct Feature {
  enum struct Status {
    NEW,
    TRACKED,
    FAILED_FLOW,
    FAILED_TRIANGULATE,
    RANSAC_OUTLIER
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Feature> Ptr;

  std::weak_ptr<Frame> _frame;        // assosciated frame
  std::weak_ptr<MapPoint> _map_point; // assosciated map point [3d]
  cv::KeyPoint _position;             // 2d coordinate

  Status _status = Status::NEW;  // current feature point status
  bool _is_on_left_image = true; // if feature is on left or right image

  Feature() {}
  Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
      : _frame(frame), _position(kp) {}
};

} // namespace structures

#endif
