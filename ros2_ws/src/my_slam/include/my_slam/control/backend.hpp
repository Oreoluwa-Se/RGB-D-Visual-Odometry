#ifndef SLAM_BACKEND_HPP
#define SLAM_BACKEND_HPP

#include <atomic>
#include <common.hpp>
#include <condition_variable>
#include <my_slam/structures/frame.hpp>
#include <my_slam/structures/map.hpp>
#include <thread>

namespace my_slam {
using sensor::Camera;
using structures::Frame;
using structures::Map;

class Map;
class Backend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Backend> Ptr;

  Backend();

  void SetParametersAndRun(double freq_hz, int feat_opt_number,
                           int final_ransac_iter, bool loop_closure_act,
                           int num_occurences_for_visible, int chi_idx);

  void SetCameras(Camera::Ptr left) { _cam_left = left; }

  void SetMap(const std::shared_ptr<Map> &map) { _map = map; }

  void UpdateMap(Frame::Ptr frame);

  void PauseBackend();

  bool PauseCheck();

  void Resume();

  void Stop();

private:
  void BackendLoop();

  void Optimize(structures::Map::KeyframesType &kf,
                structures::Map::LandmarksType &lndmrks);

  std::shared_ptr<Map> _map;
  std::thread _backend_thread;
  std::condition_variable _map_update;

  std::atomic<bool> _backend_running;
  std::atomic<bool> _backend_paused;
  std::atomic<bool> _backend_has_paused;

  std::mutex _data_mutex;
  std::mutex _map_mutex;

  Camera::Ptr _cam_left = nullptr;
  Frame::Ptr _curr_frame = nullptr;
  double _sleep_time = 0.1;
  int _feat_opt_number = 0;
  int _final_ransac_iter = 0;
  bool _loop_closure_act = false;
  int _num_occurences_for_visible = 10;
  int _chi_idx = 2;
};
} // namespace my_slam
#endif