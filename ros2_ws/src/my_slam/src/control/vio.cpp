#include "my_slam/control/viewer.hpp"
#include "my_slam/dataset/reader.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <helper_modules.hpp>
#include <iostream>
#include <my_slam/control/vio.hpp>
#include <my_slam/dataset/config.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <unistd.h>

namespace my_slam {
using dataset::Config;
using dataset::Reader;

VisualOdom::VisualOdom(const rclcpp::NodeOptions &options)
    : Node("visual_odom_node", options) {}

bool VisualOdom::Init(Frontend::Ptr frontend, Reader::Ptr dataset,
                      Viewer::Ptr viewer, const std::string &path) {
  {
    Config::build();
    Config::SetParameterFile(path);
  }
  // get required information
  ParameterSetup();
  // reading parameters from config file
  _frontend = frontend;
  _data_reader = dataset;
  _viewer = viewer;
  SectionSetup(path);
  return true;
}

void VisualOdom::ParameterSetup() {
  _number_of_active_frame = Config::Get<int>("num_active_frames");
  _feat_opt_number = Config::Get<int>("final_feats_opt_number");
  _freq_hz = Config::Get<double>("sleep_time");
  _loop_closure_act = (Config::Get<int>("loop_closure") == 0) ? false : true;
  _num_occurences_for_visible = Config::Get<int>("num_occur_for_visible");
  _chi_idx = Config::Get<int>("chi_dof");
  _show_point_cloud =
      (Config::Get<int>("show_point_cloud") == 0) ? false : true;
  _num_features_init = Config::Get<int>("num_features_init");
  _num_features = Config::Get<int>("num_features");
  _num_features_tracking = Config::Get<int>("num_features_tracking");
  _num_features_tracking_bad = Config::Get<int>("num_features_tracking_bad");
  _num_features_needed_for_keyframe =
      Config::Get<int>("num_features_key_frame");
  _max_depth = Config::Get<double>("camera_max_depth");
  _min_depth = Config::Get<double>("camera_min_depth");
  _num_detected_feats = size_t(Config::Get<int>("detected_features"));
  _filter_radius = Config::Get<double>("radius");
  _border = Config::Get<int>("boarder");
  _scale_factor = Config::Get<double>("scale_factor");
  _lvl_pyramid = Config::Get<int>("level_pyramid");
  _min_dist_loop_closure = Config::Get<int>("min_dist_for_loop_closure");
  _g2o_ransac_iter = Config::Get<int>("final_ransac_iter");
  _cv_ransac_thresh = Config::Get<double>("ransac_thresh");
  _cv_ransac_percent = Config::Get<double>("ransac_percent");
  _cv_ransac_maxIter = Config::Get<int>("ransac_maxIter");
  _half_patch_size = Config::Get<int>("half_patch_size");
  _min_dist_loop_closure = Config::Get<double>("min_dist_for_loop_closure");
  _opt_flow_ransac_num = Config::Get<int>("iterations");
}

void VisualOdom::SectionSetup(const std::string &path) {

  _map = Map::Ptr(new Map(_number_of_active_frame));
  _backend = Backend::Ptr(new Backend);

  _data_reader->SetParameters(path);
  _data_reader->Init();

  _frontend->SetMap(_map);
  _frontend->SetParameters(
      _num_features_init, _num_features, _num_features_tracking,
      _num_features_tracking_bad, _num_features_needed_for_keyframe,
      _loop_closure_act, _max_depth, _min_depth, _num_detected_feats,
      _filter_radius, _scale_factor, _lvl_pyramid, _border, _freq_hz,
      _g2o_ransac_iter, _feat_opt_number, _chi_idx, _cv_ransac_thresh,
      _cv_ransac_percent, _cv_ransac_maxIter, _half_patch_size,
      _min_dist_loop_closure, _opt_flow_ransac_num);

  _frontend->SetBackend(_backend);
  _frontend->SetCameras(_data_reader->GetCamera(0));
  auto _odom_frames = _data_reader->GetOdomFrameInfo();
  _frontend->SetFramesAndPosition(_data_reader->GetCameraFrameID(),
                                  _odom_frames.first, _odom_frames.second,
                                  _data_reader->_init_position);

  _backend->SetParametersAndRun(_freq_hz, _feat_opt_number, _g2o_ransac_iter,
                                _loop_closure_act, _num_occurences_for_visible,
                                _chi_idx);
  _backend->SetCameras(_data_reader->GetCamera(0));
  _backend->SetMap(_map);

  _viewer->SetMap(_map);
  _viewer->SetCameraFrame(_data_reader->GetCameraFrameID());
  _viewer->Init(_show_point_cloud, _freq_hz);
  _frontend->SetViewer(_viewer);
}

void VisualOdom::Run() {
  rclcpp::Rate rate(_freq_hz);

  while (rclcpp::ok()) {
    if (Step() == false) {
      break;
    }
    rate.sleep();
  }

  _backend->Stop();
  if (_viewer)
    _viewer->Stop();
}

bool VisualOdom::Step() {
  Frame::Ptr new_frame = _data_reader->NextFrame();
  auto t1 = std::chrono::steady_clock::now();
  bool success = _frontend->AddFrame(std::move(new_frame));
  auto t2 = std::chrono::steady_clock::now();
  auto time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";

  return success;
}
} // namespace my_slam
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_slam::VisualOdom)
