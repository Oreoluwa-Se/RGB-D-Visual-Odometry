#include <helper_modules.hpp>
#include <map>
#include <my_slam/control/backend.hpp>
#include <my_slam/control/g2o_types.hpp>
#include <my_slam/structures/feature.hpp>
#include <my_slam/structures/map_point.hpp>
#include <unistd.h>

namespace my_slam {
using structures::Feature;
using structures::MapPoint;
// constructor starts thread for backend process
Backend::Backend() {
  _backend_running.store(true);
  _backend_paused.store(false);
  _backend_has_paused.store(false);
}

void Backend::SetParametersAndRun(double freq_hz, int feat_opt_number,
                                  int final_ransac_iter, bool loop_closure_act,
                                  int num_occurences_for_visible, int chi_idx) {
  _sleep_time = 1 / freq_hz;
  _feat_opt_number = feat_opt_number;
  _final_ransac_iter = final_ransac_iter;
  _loop_closure_act = loop_closure_act;
  _num_occurences_for_visible = num_occurences_for_visible;
  _chi_idx = chi_idx;
  /* _______________________________________________________________ */
  _backend_thread = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap(Frame::Ptr frame) {
  std::unique_lock<std::mutex> lock(_map_mutex);
  _curr_frame = frame;
  _map_update.notify_one();
}

void Backend::Stop() {
  _backend_running.store(false);
  _map_update.notify_one();
  _backend_thread.join();
}

void Backend::PauseBackend() { _backend_paused.store(true); }

bool Backend::PauseCheck() {
  return _backend_paused.load() && _backend_has_paused.load();
}

void Backend::Resume() { _backend_paused.store(false); }

void Backend::BackendLoop() {
  while (_backend_running.load()) {
    while (_backend_paused.load()) {
      _backend_has_paused.store(true);
      sleep(_sleep_time);
    }
    _backend_has_paused.store(false);

    std::unique_lock<std::mutex> lock(_data_mutex);
    _map_update.wait(lock);

    structures::Map::KeyframesType active_kfs = _map->GetActiveKeyFrames();
    structures::Map::LandmarksType active_landmarks =
        _map->GetActiveMapPoints();

    Optimize(active_kfs, active_landmarks);

    // get connections
    if (_loop_closure_act) {
      _curr_frame->UpdateCovisibleConnects(_num_occurences_for_visible);
      //   _loop_closure->Detect(_curr_frame);
    }

    sleep(_sleep_time);
  }
}

void Backend::Optimize(structures::Map::KeyframesType &keyframes,
                       structures::Map::LandmarksType &landmarks) {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  std::map<unsigned long, VertexPose *> vertices;
  unsigned long max_kf_id = 0;

  for (auto &keyframe : keyframes) {
    auto kf = keyframe.second;
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(kf->_keyframe_id);
    vertex_pose->setEstimate(kf->Pose());
    optimizer.addVertex(vertex_pose);
    if (kf->_keyframe_id > max_kf_id)
      max_kf_id = kf->_keyframe_id;

    vertices.insert({kf->_keyframe_id, vertex_pose});
  }

  std::map<unsigned long, VertexXYZ *> vertices_lndmrks;
  SE3 left_ext = _cam_left->Pose();
  int index = 1;
  int chi_idx = _chi_idx;
  double chi_ = chi2inv95[chi_idx];
  std::map<EdgeProjection *, Feature::Ptr> edges_and_feats;
  matrix<3> K_left = _cam_left->K();

  for (auto &landmark : landmarks) {
    if (landmark.second->_is_outlier)
      continue;

    unsigned long landmark_id = landmark.second->_id;
    auto observations = landmark.second->GetObs();
    for (auto &obs : observations) {
      if (obs.expired())
        continue;
      auto feat = obs.lock();

      if (feat->_frame.expired() || feat->_status != Feature::Status::TRACKED)
        continue;

      auto frame = feat->_frame.lock();

      if (!vertices.count(frame->_keyframe_id))
        break;

      EdgeProjection *edge = nullptr;
      edge = new EdgeProjection(K_left, left_ext);

      if (vertices_lndmrks.find(landmark_id) == vertices_lndmrks.end()) {
        VertexXYZ *v = new VertexXYZ;
        v->setEstimate(landmark.second->position());
        v->setId(landmark_id + max_kf_id + 1);
        v->setMarginalized(true);

        // if the first observation is not in active map -> added as a
        // constraint
        auto kf_id = observations.front().lock()->_frame.lock()->_keyframe_id;
        if (keyframes.find(kf_id) == keyframes.end())
          v->setFixed(true);
        else
          v->setFixed(false);

        vertices_lndmrks.insert({landmark_id, v});
        optimizer.addVertex(v);
      }

      edge->setId(index++);
      edge->setVertex(0, vertices.at(frame->_keyframe_id)); // pose
      edge->setVertex(1, vertices_lndmrks.at(landmark_id)); // landmark
      edge->setMeasurement(
          vector<2>(feat->_position.pt.x, feat->_position.pt.y));
      edge->setInformation(matrix<2>::Identity());
      auto rk = new g2o::RobustKernelHuber();
      rk->setDelta(chi_);
      edge->setRobustKernel(rk);
      edges_and_feats.insert({edge, feat});
      optimizer.addEdge(edge);
    }
  }

  // do optimization and eliminate the outliers
  optimizer.initializeOptimization();
  optimizer.optimize(_feat_opt_number);

  int iteration = 0;
  int cnt_outlier = 0, cnt_inlier = 0;
  while (iteration < _final_ransac_iter) {
    cnt_outlier = 0, cnt_inlier = 0;
    // determine if we want to adjust the outlier threshold
    for (auto &ef : edges_and_feats) {
      if (ef.first->chi2() > chi_)
        cnt_outlier++;
      else
        cnt_inlier++;
    }
    double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
    if (inlier_ratio > 0.5)
      break;
    else {
      chi_ = chi2inv95[chi_idx++];
      iteration++;
    }
  }

  for (auto &ef : edges_and_feats) {
    if (ef.first->chi2() > chi_) {
      ef.second->_status = Feature::Status::RANSAC_OUTLIER;
      // remove the observation
      if (!ef.second->_map_point.expired()) {
        ef.second->_map_point.lock()->RemoveLoopObservation(ef.second);
        if (ef.second->_map_point.lock()->GetObs().empty())
          _map->AddOutlierPoint(ef.second->_map_point.lock()->_id);
      }
    } else
      ef.second->_status = Feature::Status::TRACKED;
  }

  {
    std::unique_lock<std::mutex> lock(_map->_map_update);
    for (auto &v : vertices) {
      keyframes.at(v.first)->SetPose(v.second->estimate());
      _map->InsertFramePose(keyframes.at(v.first)->_id, v.second->estimate());
    }

    for (auto &v : vertices_lndmrks)
      landmarks.at(v.first)->SetPos(v.second->estimate());

    _map->DeleteOldActivePoints();
    _map->RemoveOutlierPoints();
  }
}
} // namespace my_slam