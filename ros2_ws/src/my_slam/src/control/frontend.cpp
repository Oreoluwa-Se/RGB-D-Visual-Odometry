#include <algorithm>
#include <chrono>

#include <future>
#include <geometry_msgs/msg/transform_stamped.h>
#include <my_slam/control/frontend.hpp>
#include <my_slam/control/g2o_types.hpp>
#include <my_slam/structures/feature.hpp>
#include <my_slam/structures/map_point.hpp>
#include <my_slam/utils/image_funcs.hpp>
#include <my_slam/utils/triangulate.hpp>

using structures::Feature;
using structures::MapPoint;
using utils::ImageFunctions;
using namespace std::chrono_literals;
namespace my_slam {

void Frontend::SetParameters(
    int num_features_init, int num_features, int num_features_tracking,
    int num_features_tracking_bad, int num_features_needed_for_keyframe,
    bool use_loop_closure, double max_depth, double min_depth,
    size_t num_detected_feats, double filter_radius, double scale_factor,
    int lvl_pyramid, int border, double freq_hq, int g2o_ransac_iter,
    int feat_opt_number, int chi_dof, double cv_ransac_thresh,
    double cv_ransac_percent, int cv_ransac_maxIter, int half_patch_size,
    double min_dist_loop_closure, int opt_flow_ransac_num) {

  _num_features_init = num_features_init;
  _num_features = num_features;
  _num_features_tracking = num_features_tracking;
  _num_features_tracking_bad = num_features_tracking_bad;
  _num_features_needed_for_keyframe = num_features_needed_for_keyframe;
  _use_loop_closure = use_loop_closure;
  _max_depth = max_depth;
  _min_depth = min_depth;
  _num_detected_feats = num_detected_feats;
  _filter_radius = filter_radius;
  _scale_factor = scale_factor;
  _lvl_pyramid = lvl_pyramid;
  _border = border;
  _freq_hq = freq_hq;
  _g2o_ransac_iter = g2o_ransac_iter;
  _feat_opt_number = feat_opt_number;
  _chi_dof = chi_dof;
  _cv_ransac_thresh = cv_ransac_thresh;
  _cv_ransac_percent = cv_ransac_percent;
  _cv_ransac_maxIter = cv_ransac_maxIter;
  _half_patch_size = half_patch_size;
  _min_dist_loop_closure = min_dist_loop_closure;
  _opt_flow_ransac_num = opt_flow_ransac_num;

  _timer =
      this->create_wall_timer(50ms, std::bind(&Frontend::SetCameraFrame, this));
}

int Frontend::DetectFeatures() {
  // want to keep features already being tracked
  std::vector<vector<2>> init_features;
  std::vector<Feature::Ptr> tracked;
  for (size_t i = 0; i < _current_frame->_features_left.size(); ++i) {
    init_features.emplace_back(
        _current_frame->_features_left[i]->_position.pt.x,
        _current_frame->_features_left[i]->_position.pt.y);
    tracked.emplace_back(_current_frame->_features_left[i]);
  }
  assert(init_features.size() == _current_frame->_features_left.size());

  ImageFunctions::feature_extraction(
      init_features, _current_frame->_left_img, _current_frame->_desc_left,
      _num_detected_feats, _filter_radius, _border, _scale_factor, _lvl_pyramid,
      _use_loop_closure);

  // convert new added features
  auto new_feats_cv = ImageFunctions::eig_2_cvkp(
      slice(init_features, int(tracked.size()), init_features.size()));

  //  make feature points
  int cnt_detected = 0;
  for (size_t i = 0; i < new_feats_cv.size(); i++) {
    tracked.emplace_back(
        Feature::Ptr(new Feature(_current_frame, new_feats_cv[i])));
    cnt_detected++;
  }

  // contains tracked from previous and new parts
  _current_frame->_features_left.clear();
  std::vector<cv::KeyPoint> cv_out;
  for (size_t i = 0; i < tracked.size(); ++i) {
    _current_frame->_features_left.emplace_back(std::move(tracked[i]));
    cv_out.emplace_back(_current_frame->_features_left[i]->_position);
  }

  cv::drawKeypoints(_current_frame->_left_img, cv_out,
                    _current_frame->_detected_feat_img, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DEFAULT);

  return cnt_detected;
}

bool Frontend::AddFrame(Frame::Ptr &&frame) {
  _current_frame = std::move(frame);

  // blocks during map update -> mostly for loop closure [not implemented
  // yet]
  std::unique_lock<std::mutex> lock(_map->_map_update);

  switch (_status) {
  case SystemStatus::INIT:
    RGBDInit();
    break;
  case SystemStatus::TRACKING_GOOD:
  case SystemStatus::TRACKING_BAD:
    Track();
    break;
  case SystemStatus::LOST:
    Reset();
    break;
  }

  _viewer->AddFrame(_current_frame);
  _viewer->PlotImage(std::move(_current_frame->_detected_feat_img),
                     std::move(_current_frame->_tracked_feat_img));
  _map->InsertFramePose(_current_frame->_id, _current_frame->Pose());
  _last_frame = _current_frame;

  return true;
}

bool Frontend::RGBDInit() {

  DetectFeatures();
  if (BuildInitMap() > _num_features_init) {
    _status = SystemStatus::TRACKING_GOOD;

    return true;
  }

  return false;
}

bool Frontend::Track() {
  if (_last_frame)
    _current_frame->SetPose(_relative_motion * _last_frame->Pose());

  TrackLastFrame();
  _tracking_inliers = EstimateCurrentPose();

  if (_tracking_inliers > _num_features_tracking) {
    // tracking good
    _status = SystemStatus::TRACKING_GOOD;
  } else if (_tracking_inliers > _num_features_tracking_bad) {
    // tracking bad
    _status = SystemStatus::TRACKING_BAD;
  } else
    _status = SystemStatus::LOST;

  InsertKeyframe();
  _relative_motion = _current_frame->Pose() * _last_frame->Pose().inverse();

  return true;
}

int Frontend::BuildInitMap() {
  size_t cnt_init_landmarks = 0;

  for (size_t i = 0; i < _current_frame->_features_left.size();
       ++i) { // needed to add the status check
    if (_current_frame->_features_left[i]->_status !=
            Feature::Status::TRACKED &&
        _current_frame->_features_left[i]->_status != Feature::Status::NEW)
      continue;

    // convert pixel to ray coordinates
    vector<3> ray;
    auto check_1 = _camera_left->pixel2camera(
        vector<2>(_current_frame->_features_left[i]->_position.pt.x,
                  _current_frame->_features_left[i]->_position.pt.y),
        ray);

    if (!(check_1))
      continue;

    auto depth_idx = size_t(_current_frame->_features_left[i]->_position.pt.x +
                            _current_frame->_features_left[i]->_position.pt.y *
                                _camera_left->GetWidth());

    // depth is a cv:: matrix
    auto depth_val = _current_frame->_depth[depth_idx];

    if (!std::isnan(depth_val) &&
        (depth_val >= _min_depth && depth_val <= _max_depth)) {

      auto new_mp = MapPoint::CreateNewMapPoint();
      auto p_world = ray * depth_val;
      new_mp->SetPos(p_world);
      new_mp->AddObservation(_current_frame->_features_left[i]);
      _current_frame->_features_left[i]->_map_point = new_mp;
      cnt_init_landmarks++;
      _map->InsertMapPoint(new_mp);
      _current_frame->_features_left[i]->_status = Feature::Status::TRACKED;
    } else {
      _current_frame->_features_left[i]->_status =
          Feature::Status::FAILED_TRIANGULATE;
    }
  }

  _current_frame->SetKeyFrame();
  _map->InsertKeyFrame(_current_frame);
  _backend->UpdateMap(_current_frame);
  return int(cnt_init_landmarks);
}

int Frontend::TriangulateNewPoints() {

  SE3 current_frame_pose = _current_frame->Pose().inverse(); // T_w_f
  int cnt_triangulated_pts = 0;

  for (size_t i = 0; i < _current_frame->_features_left.size();
       ++i) { // skip if feature has a map point, or not tracking feature
    if (!_current_frame->_features_left[i]->_map_point.expired())
      continue;

    vector<3> ray;
    auto check_1 = _camera_left->pixel2camera(
        vector<2>(_current_frame->_features_left[i]->_position.pt.x,
                  _current_frame->_features_left[i]->_position.pt.y),
        ray);

    if (!check_1)
      continue;

    auto depth_idx = size_t(_current_frame->_features_left[i]->_position.pt.x +
                            _current_frame->_features_left[i]->_position.pt.y *
                                _camera_left->GetWidth());

    // depth is a cv:: matrix
    auto depth_val = _current_frame->_depth[depth_idx];

    if (!std::isnan(depth_val) &&
        (depth_val >= _min_depth && depth_val <= _max_depth)) {
      auto new_mp = MapPoint::CreateNewMapPoint();

      auto p_world = current_frame_pose * (ray * depth_val);
      new_mp->SetPos(p_world);
      new_mp->AddObservation(_current_frame->_features_left[i]);
      _current_frame->_features_left[i]->_map_point = new_mp;
      cnt_triangulated_pts++;
      _map->InsertMapPoint(new_mp);
      _current_frame->_features_left[i]->_status = Feature::Status::TRACKED;
    } else {
      _current_frame->_features_left[i]->_status =
          Feature::Status::FAILED_TRIANGULATE;
    }
  }

  return cnt_triangulated_pts;
}

void Frontend::PublishCameraOdometry(const matrix<4> &&pose) {
  quaternion rot;
  vector<3> translation;
  {
    matrix<3> rot_;
    auto transform_mat = pose;

    rot_(0, 0) = transform_mat(0, 0);
    rot_(0, 1) = transform_mat(0, 1);
    rot_(0, 2) = transform_mat(0, 2);
    rot_(1, 0) = transform_mat(1, 0);
    rot_(1, 1) = transform_mat(1, 1);
    rot_(1, 2) = transform_mat(1, 2);
    rot_(2, 0) = transform_mat(2, 0);
    rot_(2, 1) = transform_mat(2, 1);
    rot_(2, 2) = transform_mat(2, 2);

    translation(0, 0) = transform_mat(0, 3);
    translation(1, 0) = transform_mat(1, 3);
    translation(2, 0) = transform_mat(2, 3);

    quaternion q(rot_);
    rot = std::move(q);
  }

  Odometry out_pose;
  out_pose.header.frame_id = _header_link;
  out_pose.header.stamp = this->get_clock()->now();
  out_pose.child_frame_id = _base_link;
  out_pose.pose.pose.position.x = translation.x() + _initial_position[0];
  out_pose.pose.pose.position.y = translation.y() + _initial_position[1];
  out_pose.pose.pose.position.z = translation.z() + _initial_position[2];
  out_pose.pose.pose.orientation.w = rot.w();
  out_pose.pose.pose.orientation.x = rot.x();
  out_pose.pose.pose.orientation.y = rot.y();
  out_pose.pose.pose.orientation.z = rot.z();

  _pub_camera->publish(std::move(out_pose));
}

void Frontend::SetCameraFrame() {
  if (!_camera_to_base_bool) {
    // get relationship between base link and camera
    geometry_msgs::msg::TransformStamped cam_to_base;

    try {
      // frame order(target, source)
      cam_to_base =
          _buff->lookupTransform(_base_link, _cam_link, tf2::TimePointZero);

    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  _cam_link.c_str(), _base_link.c_str(), ex.what());
      return;
    }

    quaternion q(
        cam_to_base.transform.rotation.w, cam_to_base.transform.rotation.x,
        cam_to_base.transform.rotation.y, cam_to_base.transform.rotation.z);
    q.normalize();

    // convert to homogeneous
    _camera_to_base.block<3, 3>(0, 0) = q.toRotationMatrix();

    _camera_to_base_bool = true;

    _timer->cancel();
  }
}

void Frontend::OptFlow(const cv::Mat &img1, const cv::Mat &img2,
                       std::vector<cv::Point2f> &kps_1,
                       std::vector<cv::Point2f> &kps_2,
                       std::vector<uchar> &status) {
  std::vector<cv::Mat> pyr1, pyr2;
  cv::buildOpticalFlowPyramid(img1, pyr1,
                              cv::Size(_half_patch_size, _half_patch_size),
                              _lvl_pyramid, true);

  cv::buildOpticalFlowPyramid(img2, pyr2,
                              cv::Size(_half_patch_size, _half_patch_size),
                              _lvl_pyramid, true);

  cv::Mat err;
  cv::calcOpticalFlowPyrLK(
      pyr1, pyr2, kps_1, kps_2, status, err,
      cv::Size(_half_patch_size, _half_patch_size), _lvl_pyramid,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                       _opt_flow_ransac_num, 0.01));
}

void Frontend::MatchedFeatures(const cv::Mat &img1, const cv::Mat &img2,
                               std::vector<Feature::Ptr> &feats,
                               Camera::Ptr &cam, const SE3 &curr_pose,
                               std::vector<cv::KeyPoint> &survived,
                               std::vector<int> &survived_idx) {
  std::vector<cv::Point2f> kps_feats, kps_other;
  for (auto &kp : feats) {
    bool added = false;
    if (kp->_map_point.lock()) {
      auto mp = kp->_map_point.lock();
      vector<2> px;
      if (cam->world2pixel(mp->position(), curr_pose, px)) {
        kps_other.emplace_back(cv::Point2f(px.x(), px.y()));
        kps_feats.emplace_back(kp->_position.pt);
        added = true;
      }
    }
    if (!added) {
      kps_other.emplace_back(kp->_position.pt);
      kps_feats.emplace_back(kp->_position.pt);
    }
  }
  assert(kps_other.size() == kps_feats.size());

  // OPTICAL FLOW MATCHING
  std::vector<uchar> status;
  OptFlow(img1, img2, kps_feats, kps_other, status);

  // inliers by distance
  std::vector<double> kp_dist, iqr_values;
  std::vector<size_t> trackIndex_quart, trackIndex;
  for (size_t i = 0; i < kps_feats.size(); i++) {
    if (status[i]) {
      trackIndex_quart.push_back(i);
      double xx = (kps_feats[i].x - kps_other[i].x);
      double yy = (kps_feats[i].y - kps_other[i].y);
      kp_dist.push_back(sqrt(xx * xx + yy * yy));
    } else
      feats[i]->_status = Feature::Status::RANSAC_OUTLIER;
  }

  std::vector<cv::Point2f> kps_feat_fund, kps_current_fund;
  IQR<double>(kp_dist, iqr_values);
  auto low_check = iqr_values[0] - 1.5 * iqr_values[2];
  auto high_check = iqr_values[1] + 1.5 * iqr_values[2];

  // Final stage of outlier removal.
  for (size_t i = 0; i < trackIndex_quart.size(); i++) {
    if ((kp_dist[i] >= low_check) && (kp_dist[i] <= high_check)) {
      trackIndex.push_back(trackIndex_quart[i]);
      kps_feat_fund.emplace_back(kps_feats[trackIndex_quart[i]]);
      kps_current_fund.emplace_back(kps_other[trackIndex_quart[i]]);
    } else
      feats[trackIndex_quart[i]]->_status = Feature::Status::FAILED_FLOW;
  }

  // FUNDAMENTAL MATRIX CHECK
  cv::Mat mask;
  cv::findFundamentalMat(kps_feat_fund, kps_current_fund, cv::FM_RANSAC,
                         _cv_ransac_thresh, _cv_ransac_percent,
                         _cv_ransac_maxIter, mask);

  // FINAL OUTLIER REJECTION -> INTERQUARTILE
  kp_dist.clear();
  trackIndex_quart.clear();
  for (size_t i = 0; i < trackIndex.size(); i++) {
    if (mask.at<unsigned char>((int)i) != 0) {
      trackIndex_quart.push_back(trackIndex[i]);
      double xx = (kps_feats[trackIndex[i]].x - kps_other[trackIndex[i]].x);
      double yy = (kps_feats[trackIndex[i]].y - kps_other[trackIndex[i]].y);
      kp_dist.push_back(sqrt(xx * xx + yy * yy));
    } else
      feats[trackIndex[i]]->_status = Feature::Status::RANSAC_OUTLIER;
  }

  iqr_values.clear();
  IQR<double>(kp_dist, iqr_values);
  low_check = iqr_values[0] - 1.5 * iqr_values[2];
  high_check = iqr_values[1] + 1.5 * iqr_values[2];

  // Final stage of outlier removal.
  for (size_t i = 0; i < trackIndex_quart.size(); i++) {
    if ((kp_dist[i] >= low_check) && (kp_dist[i] <= high_check)) {
      feats[trackIndex_quart[i]]->_status = Feature::Status::TRACKED;
      survived.emplace_back(kps_other[trackIndex_quart[i]], 1.f);
      survived_idx.push_back(trackIndex_quart[i]);
    } else
      feats[trackIndex_quart[i]]->_status = Feature::Status::RANSAC_OUTLIER;
  }
}

int Frontend::TrackLastFrame() {
  std::vector<cv::KeyPoint> found_kps;
  std::vector<int> idxs_from_prev_img;
  MatchedFeatures(_last_frame->_left_img, _current_frame->_left_img,
                  _last_frame->_features_left, _camera_left,
                  _current_frame->Pose(), found_kps, idxs_from_prev_img);

  cv::cvtColor(_current_frame->_left_img, _current_frame->_tracked_feat_img,
               cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < idxs_from_prev_img.size(); i++) {
    // keep points with 3d reference from previous image
    if (_last_frame->_features_left[idxs_from_prev_img[i]]
            ->_map_point.expired())
      continue;

    Feature::Ptr feat(new Feature(_current_frame, std::move(found_kps[i])));
    feat->_status = Feature::Status::TRACKED;
    feat->_map_point =
        _last_frame->_features_left[idxs_from_prev_img[i]]->_map_point;

    // show tracked path
    cv::circle(_current_frame->_tracked_feat_img, feat->_position.pt, 2,
               cv::Scalar(0, 250, 0), 2);
    cv::line(_current_frame->_tracked_feat_img,
             _last_frame->_features_left[idxs_from_prev_img[i]]->_position.pt,
             feat->_position.pt, cv::Scalar(0, 250, 0));

    // add to  feature list
    _current_frame->_features_left.emplace_back(std::move(feat));
  }

  LOG(INFO) << "Found " << _current_frame->_features_left.size()
            << " from last image.";

  return int(_current_frame->_features_left.size());
}

int Frontend::EstimateCurrentPose() {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  // vertex
  VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
  vertex_pose->setId(0);
  vertex_pose->setEstimate(_current_frame->Pose());
  optimizer.addVertex(vertex_pose);

  matrix<3> K = _camera_left->K();

  // edges
  int index = 1;
  std::vector<EdgeProjectionPoseOnly *> edges;
  std::vector<Feature::Ptr> features;
  for (size_t i = 0; i < _current_frame->_features_left.size(); ++i) {
    auto mp = _current_frame->_features_left[i]->_map_point.lock();
    if (mp) {
      features.emplace_back(_current_frame->_features_left[i]);
      EdgeProjectionPoseOnly *edge =
          new EdgeProjectionPoseOnly(mp->position(), K);
      edge->setId(index++);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(
          vector<2>(_current_frame->_features_left[i]->_position.pt.x,
                    _current_frame->_features_left[i]->_position.pt.y));
      edge->setInformation(matrix<2>::Identity());
      edge->setRobustKernel(new g2o::RobustKernelHuber);
      edges.emplace_back(edge);
      optimizer.addEdge(edge);
    }
  }

  // estimate the Pose the determine the outliers
  const double chi2_th = chi2inv95[_chi_dof];
  int cnt_outlier = 0;

  std::vector<double> errors;
  std::vector<int> idx;
  int tracked_idx_count = 0;
  for (int iteration = 0; iteration < _g2o_ransac_iter; ++iteration) {
    optimizer.initializeOptimization();
    optimizer.optimize(_feat_opt_number);
    cnt_outlier = 0;

    // count the outliers
    for (size_t i = 0; i < edges.size(); ++i) {
      auto e = edges[i];
      if (features[i]->_status != Feature::Status::TRACKED)
        e->computeError();

      if (e->chi2() > chi2_th) {
        features[i]->_status = Feature::Status::RANSAC_OUTLIER;
        e->setLevel(1);
        cnt_outlier++;
      } else {
        features[i]->_status = Feature::Status::TRACKED;
        e->setLevel(0);
        if (iteration == _g2o_ransac_iter - 1) {
          errors.emplace_back(e->chi2());
          idx.emplace_back(i);
          tracked_idx_count++;
        }
      }

      if (iteration == _g2o_ransac_iter - 2)
        e->setRobustKernel(nullptr);
    }
  }

  { // refine errors
    std::vector<double> iqr_values;
    IQR<double>(errors, iqr_values);
    auto low_check = iqr_values[0] - 1.5 * iqr_values[2];
    auto high_check = iqr_values[1] + 1.5 * iqr_values[2];
    for (size_t i = 0; i < errors.size(); i++) {
      if ((errors[i] >= low_check) && (errors[i] <= high_check))
        continue;

      features[idx[i]]->_status = Feature::Status::RANSAC_OUTLIER;
      if (!features[i]->_map_point.expired())
        _map->AddOutlierPoint(features[i]->_map_point.lock()->_id);
    }
  }

  for (auto &feat : features) {
    if (feat->_status != Feature::Status::TRACKED) {
      feat->_map_point.reset();
      // can still be useful in triangulation
      feat->_status = Feature::Status::NEW;
    }
  }

  // Returns pose of camera wrt world-> T_cw
  _current_frame->SetPose(vertex_pose->estimate());
  // need to orient it it to base frame coordinates

  rclcpp::Rate rate(_freq_hq);
  while (!_camera_to_base_bool) {
    rate.sleep();
  }

  auto pub_pose = _current_frame->Pose().inverse().matrix();
  {
    auto temp = _current_frame->Pose().inverse().matrix();
    temp.block<3, 4>(0, 0) = _camera_to_base.matrix() * temp.block<3, 4>(0, 0);

    pub_pose(0, 3) = temp(0, 3);
    pub_pose(1, 3) = temp(1, 3);
    pub_pose(2, 3) = temp(2, 3);
  }

  //   LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
  //             << features.size() - cnt_outlier;
  LOG(INFO) << "Current Pose " << _current_frame->_id << " = \n "
            << _current_frame->Pose().inverse().matrix();
  std::cout << " " << std::endl;
  LOG(INFO) << "Current Pose base link oriented" << _current_frame->_id
            << " = \n " << pub_pose.matrix();

  PublishCameraOdometry(std::move(pub_pose));

  return features.size() - cnt_outlier;
}

void Frontend::SetObservationsForKeyFrame() {
  for (auto &feat : _current_frame->_features_left) {
    auto mp = feat->_map_point.lock();
    if (mp)
      mp->AddObservation(feat);
  }
}

bool Frontend::InsertKeyframe() {
  auto T_diff = (_use_loop_closure)
                    ? _last_keyframe_pose.inverse() * _current_frame->Pose()
                    : _last_loopframe_pose.inverse() * _current_frame->Pose();

  auto rot_dist = SO3(T_diff.rotationMatrix()).log().norm();

  if (!_use_loop_closure && abs(rot_dist) < 0.25 &&
      _tracking_inliers >= _num_features_needed_for_keyframe) {
    return false;
  } else if ((_use_loop_closure && abs(rot_dist) < 0.25) &&
             abs(T_diff.log().norm()) <= _min_dist_loop_closure &&
             _tracking_inliers >= _num_features_needed_for_keyframe) {
    return false;
  }

  // current frame is a new keyframe
  _current_frame->SetKeyFrame();
  _map->InsertKeyFrame(_current_frame);
  LOG(INFO) << "Set frame " << _current_frame->_id << " as keyframe "
            << _current_frame->_keyframe_id;
  SetObservationsForKeyFrame();
  DetectFeatures();
  TriangulateNewPoints();

  if (_use_loop_closure)
    _last_loopframe_pose = _current_frame->Pose();
  else
    _last_keyframe_pose = _current_frame->Pose();

  _backend->UpdateMap(_current_frame);

  return true;
}

bool Frontend::Reset() { // resent when we don't have enough features
                         //   LOG(INFO) << "Resetting.. Not Enough Features ";
  _current_frame->SetKeyFrame();
  _map->InsertKeyFrame(_current_frame);

  SetObservationsForKeyFrame();
  DetectFeatures();
  TriangulateNewPoints();

  _backend->UpdateMap(_current_frame);
  return true;
}
} // namespace my_slam
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_slam::Frontend)