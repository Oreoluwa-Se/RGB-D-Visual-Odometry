#ifndef MAP_POINT_HPP
#define MAP_POINT_HPP

#include <common.hpp>
#include <my_slam/structures/feature.hpp>

namespace structures {
struct MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<MapPoint> Ptr;

  unsigned long _id = 0;
  int _observed_times = 0;
  bool _is_outlier = false;

  MapPoint() {}
  MapPoint(long id, vector<3> position) : _id(id), _position(position) {}

  vector<3> position() {
    std::unique_lock<std::mutex> lock(_data_mutex);
    return _position;
  }

  void SetPos(const vector<3> &pos) {
    std::unique_lock<std::mutex> lck(_data_mutex);
    _position = pos;
  };

  void AddObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lock(_data_mutex);
    _observations.emplace_back(feat);
    _loop_observations.emplace_back(feat);
  }

  // for observations in active keyframes
  void RemoveObservation(std::shared_ptr<Feature> feat);

  // for general observations
  void RemoveLoopObservation(std::shared_ptr<Feature> feat);

  std::list<std::weak_ptr<Feature>> GetObs() {
    std::unique_lock<std::mutex> lock(_data_mutex);
    return _observations;
  }

  int GetActiveObsCount() {
    std::unique_lock<std::mutex> lock(_data_mutex);
    return _observations.size();
  }

  int GetObsCount() {
    std::unique_lock<std::mutex> lock(_data_mutex);
    return _loop_observations.size();
  }

  std::list<std::weak_ptr<Feature>> GetLoopObs() {
    std::unique_lock<std::mutex> lock(_data_mutex);
    return _loop_observations;
  }

  static MapPoint::Ptr CreateNewMapPoint();

private:
  vector<3> _position = vector<3>::Zero();
  std::list<std::weak_ptr<Feature>> _loop_observations;
  std::list<std::weak_ptr<Feature>> _observations;
  std::mutex _data_mutex;
};
} // namespace structures
#endif