#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP

#include "rclcpp/rate.hpp"
#include <common.hpp>
#include <helper_modules.hpp>
#include <my_slam/control/backend.hpp>
#include <my_slam/control/frontend.hpp>
#include <my_slam/control/viewer.hpp>
#include <my_slam/dataset/reader.hpp>
#include <my_slam/structures/frame.hpp>
#include <my_slam/structures/map.hpp>

namespace my_slam {
using dataset::Reader;
using structures::Map;

class VisualOdom : public rclcpp::Node {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<VisualOdom> Ptr;

  VisualOdom(const rclcpp::NodeOptions &options);

  bool Init(Frontend::Ptr frontend, Reader::Ptr dataset, Viewer::Ptr viewer,
            const std::string &path);
  void ParameterSetup();
  void SectionSetup(const std::string &path);
  void Run();
  bool Step();

  SystemStatus GetSystemStatus() const { return _frontend->GetStatus(); }
  Frontend::Ptr _frontend = nullptr;
  Reader::Ptr _data_reader = nullptr;

private:
  bool _inited = false;
  Backend::Ptr _backend = nullptr;
  Map::Ptr _map = nullptr;
  Viewer::Ptr _viewer = nullptr;

  //   _parameters for system
  int _number_of_active_frame = 7;
  double _freq_hz = 10;
  int _feat_opt_number = 4;
  bool _loop_closure_act = false;
  int _num_occurences_for_visible = 10;
  int _chi_idx = 2;
  bool _show_point_cloud = false;
  int _num_features = 200;
  int _num_features_init = 100;
  int _num_features_tracking = 50;
  int _num_features_tracking_bad = 20;
  int _num_features_needed_for_keyframe = 80;
  double _max_depth = 3.5;
  double _min_depth = 0.1;
  size_t _num_detected_feats = 1000;
  double _filter_radius = 1.0;
  double _scale_factor = 1.2;
  int _lvl_pyramid = 8;
  int _border = 10;
  double _min_dist_loop_closure = 0.5;
  int _g2o_ransac_iter = 10;
  double _cv_ransac_thresh = 1;
  double _cv_ransac_percent = 0.99;
  int _cv_ransac_maxIter = 100;
  int _half_patch_size = 11;
  int _opt_flow_ransac_num = 30;
};
} // namespace my_slam

#endif