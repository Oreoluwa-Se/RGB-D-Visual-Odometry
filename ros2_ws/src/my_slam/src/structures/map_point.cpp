#include <my_slam/structures/map_point.hpp>
namespace structures {

MapPoint::Ptr MapPoint::CreateNewMapPoint() {

  static long fatctory_id = 0;
  MapPoint::Ptr new_mp(new MapPoint);
  new_mp->_id = fatctory_id++;
  return new_mp;
}

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
  std::unique_lock<std::mutex> lock(_data_mutex);
  for (auto iter = _observations.begin(); iter != _observations.end(); iter++) {
    if (iter->expired())
      continue;
    if (iter->lock() == feat) {
      _observations.erase(iter);
      break;
    }
  }
}

void MapPoint::RemoveLoopObservation(std::shared_ptr<Feature> feat) {
  std::unique_lock<std::mutex> lock(_data_mutex);
  for (auto iter = _loop_observations.begin(); iter != _loop_observations.end();
       iter++) {
    if (iter->expired())
      continue;
    if (iter->lock() == feat) {
      _loop_observations.erase(iter);
      feat->_map_point.reset();

      break;
    }
  }

  if (_loop_observations.size() == 0)
    _is_outlier = true;

  RemoveObservation(feat);
}
} // namespace structures