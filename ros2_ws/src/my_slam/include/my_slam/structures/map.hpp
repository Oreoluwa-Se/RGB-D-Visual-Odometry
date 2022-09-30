#ifndef MAP_HPP
#define MAP_HPP

#include <common.hpp>
#include <my_slam/structures/frame.hpp>
#include <my_slam/structures/map_point.hpp>

namespace structures {

struct IndexHash {
  // to reduce collisions that arises for hashing map keys -> good when
  // expecting large data
  std::size_t operator()(const unsigned long &idx) const {
    size_t seed = 0;

    // 0x9e3779b9 -> irrational number hierachy... We stan an irrational king
    // (seed << 6) + (seed >> 2) -> for shuffling bits | ^-XOR function
    seed ^= std::hash<int>{}(idx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

class Map {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Map> Ptr;
  typedef std::unordered_map<unsigned long, MapPoint::Ptr, IndexHash>
      LandmarksType;
  typedef std::unordered_map<unsigned long, Frame::Ptr, IndexHash>
      KeyframesType;
  typedef std::unordered_map<unsigned long, SE3, IndexHash> PoseType;

  Map(int num_active_keyframes = 7) : _num_active_frames(num_active_keyframes) {
    _keyframes.max_load_factor(0.25);
    _active_keyframes.max_load_factor(0.25);
    _landmarks.max_load_factor(0.25);
    _active_landmarks.max_load_factor(0.25);
    _framePose.max_load_factor(0.25);
  }
  // mutex when updating loop
  std::mutex _map_update;

  // used to calculate RMSE
  void InsertFramePose(long id, SE3 twf) {
    std::unique_lock<std::mutex> lck(_data_mutex);
    if (_framePose.find(id) == _framePose.end())
      _framePose.insert(std::make_pair(id, twf));
    else
      _framePose[id] = twf;
  }

  void InsertKeyFrame(Frame::Ptr frame);

  void InsertMapPoint(MapPoint::Ptr map_point);

  void AddOutlierPoint(unsigned long idx) {
    std::unique_lock<std::mutex> lck(_map_point_mutex);
    outlierIdx.push_back(idx);
  }

  LandmarksType GetAllMapPoints() {
    std::unique_lock<std::mutex> lck(_data_mutex);
    return _landmarks;
  }

  KeyframesType GetAllKeyFrames() {
    std::unique_lock<std::mutex> lck(_data_mutex);
    return _keyframes;
  }

  LandmarksType GetActiveMapPoints() {
    std::unique_lock<std::mutex> lck(_data_mutex);
    return _active_landmarks;
  }

  KeyframesType GetActiveKeyFrames() {
    std::unique_lock<std::mutex> lck(_data_mutex);
    return _active_keyframes;
  }

  PoseType GetFrameTrail() {
    std::unique_lock<std::mutex> lck(_data_mutex);
    return _framePose;
  }

  void RemoveOutlierPoints();

  void DeleteOldActivePoints();

  KeyframesType GetAllNAKeyFrames();

  void CleanMap();

private:
  void RemoveOldKeyFrame();

  std::mutex _data_mutex;
  std::mutex _map_point_mutex;

  LandmarksType _landmarks;
  LandmarksType _active_landmarks;
  KeyframesType _keyframes;
  KeyframesType _active_keyframes;
  PoseType _framePose;
  std::list<unsigned long> outlierIdx;

  Frame::Ptr _current_frame = nullptr;
  int _num_active_frames;
};
} // namespace structures
#endif