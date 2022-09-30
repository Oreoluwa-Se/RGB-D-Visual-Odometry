#include <my_slam/structures/map.hpp>

namespace structures {

void Map::InsertKeyFrame(Frame::Ptr frame) {

  std::unique_lock<std::mutex> lck(_data_mutex);
  _current_frame = frame;
  if (_keyframes.find(frame->_keyframe_id) == _keyframes.end()) {
    _keyframes.insert(std::make_pair(frame->_keyframe_id, frame));
    _active_keyframes.insert(std::make_pair(frame->_keyframe_id, frame));
  } else {
    _keyframes[frame->_keyframe_id] = frame;
    _active_keyframes[frame->_keyframe_id] = frame;
  }

  if (int(_active_keyframes.size()) > _num_active_frames)
    RemoveOldKeyFrame();
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
  //   std::unique_lock<std::mutex> lck(_data_mutex);
  if (_landmarks.find(map_point->_id) == _landmarks.end()) {
    _landmarks.insert(std::make_pair(map_point->_id, map_point));
    _active_landmarks.insert(std::make_pair(map_point->_id, map_point));
  } else {
    _landmarks[map_point->_id] = map_point;
    _active_landmarks[map_point->_id] = map_point;
  }
}

void Map::RemoveOldKeyFrame() {
  if (_current_frame == nullptr)
    return;

  double max_dis = 0, min_dis = 999999;
  double max_kf_id = 0, min_kf_id = 0;
  auto Twf = _current_frame->Pose().inverse();
  for (auto &kf : _active_keyframes) {
    if (kf.second == _current_frame)
      continue;
    auto dist = (kf.second->Pose() * Twf).log().norm();
    if (dist > max_dis) {
      max_dis = dist;
      max_kf_id = kf.first;
    }
    if (dist < min_dis) {
      min_dis = dist;
      min_kf_id = kf.first;
    }
  }

  /* ========================== */
  const double min_dis_th = 0.2;
  Frame::Ptr frame_to_remove = nullptr;
  if (min_dis < min_dis_th) {
    frame_to_remove = _keyframes.at(min_kf_id);
  } else {
    frame_to_remove = _keyframes.at(max_kf_id);
  }

  _active_keyframes.erase(frame_to_remove->_keyframe_id);
  for (auto feat : frame_to_remove->_features_left) {
    if (feat->_map_point.lock()) {
      auto mp = feat->_map_point.lock();
      mp->RemoveObservation(feat);
    }
  }

  CleanMap();
}

void Map::CleanMap() {
  int cnt_landmark_removed = 0;

  if (_active_landmarks.empty())
    return;

  for (auto iter = _active_landmarks.begin();
       iter != _active_landmarks.end();) {
    if (iter->second->GetActiveObsCount() == 0) {
      iter = _active_landmarks.erase(iter);
      cnt_landmark_removed++;
    } else
      ++iter;
  }
}

Map::KeyframesType Map::GetAllNAKeyFrames() {
  std::unique_lock<std::mutex> lck(_data_mutex);
  KeyframesType nonactive;
  for (auto &k : _keyframes)
    if (_active_keyframes.find(k.first) != _active_keyframes.end())
      nonactive.insert({k.first, k.second});
  // ------------ //
  return nonactive;
}

void Map::DeleteOldActivePoints() {
  std::unique_lock<std::mutex> lck(_map_point_mutex);
  int cnt_landmark_removed = 0;

  for (auto iter = _active_landmarks.begin();
       iter != _active_landmarks.end();) {
    if (iter->second == nullptr)
      continue;

    if (iter->second->_observed_times == 0) {
      iter = _active_landmarks.erase(iter);
      cnt_landmark_removed++;
    } else
      ++iter;
  }
}

void Map::RemoveOutlierPoints() {
  std::unique_lock<std::mutex> lck(_map_point_mutex);
  for (auto iter = outlierIdx.begin(); iter != outlierIdx.end(); iter++) {
    _landmarks.erase(*iter);
    _active_landmarks.erase(*iter);
  }
  outlierIdx.clear();
}
} // namespace structures
