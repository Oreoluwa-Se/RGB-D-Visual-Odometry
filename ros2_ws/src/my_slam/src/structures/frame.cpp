
#include <my_slam/structures/feature.hpp>
#include <my_slam/structures/frame.hpp>
#include <my_slam/structures/map_point.hpp>

namespace structures {

Frame::Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &left)
    : _id(id), _time_stamp(time_stamp), _left_img(left), _pose(pose) {}

Frame::Ptr Frame::CreateFrame() {
  static long factory_id = 0;
  Frame::Ptr new_frame(new Frame);
  new_frame->_id = factory_id++;
  return new_frame;
}

void Frame::SetKeyFrame() {
  static long keyframe_factory_id = 0;
  _is_keyframe = true;
  _keyframe_id = keyframe_factory_id++;
}

void Frame::UpdateCovisibleConnects(int occurences) {
  // finds keyframes that share the same map_point
  std::unique_lock<std::mutex> lock(_frame_connections_mutex);
  std::unordered_map<Frame::Ptr, int> frame_count;

  for (const auto &feat : this->_features_left)
    if (!feat->_map_point.expired()) {
      auto mp = feat->_map_point.lock();
      for (const auto &obs : mp->GetLoopObs()) {
        if (!obs.expired()) {
          auto ob_frame = obs.lock()->_frame.lock();

          if (ob_frame && ob_frame->_keyframe_id != this->_keyframe_id)
            frame_count[ob_frame]++;
        }
      }
    }

  if (frame_count.empty())
    return;

  _kf_connected.clear();

  // in situaation where we dont pass criteria.. select maximum
  int max_count{0};
  Frame::Ptr max_count_kf;

  for (auto &kf : frame_count) {
    if (kf.second > occurences) {
      _kf_connected[kf.first] = kf.second;
      kf.first->AddConnection(this->shared_from_this(), kf.second);
    }

    if (kf.second > max_count) {
      max_count = kf.second;
      max_count_kf = kf.first;
    }
  }

  if (_kf_connected.empty() && max_count > 0) {
    _kf_connected[max_count_kf] = max_count;
    max_count_kf->AddConnection(this->shared_from_this(), max_count);
  } else
    return;

  SortConnectedKeyFrames();
}

void Frame::AddConnection(Frame::Ptr frame, int weight) {
  std::unique_lock<std::mutex> lock(_frame_connections_mutex);
  _kf_connected[frame] = weight;
}

void Frame::SortConnectedKeyFrames() {

  std::vector<std::pair<int, Frame::Ptr>> connected_kf;
  std::transform(_kf_connected.begin(), _kf_connected.end(),
                 std::back_inserter(connected_kf), [](const auto &kf) {
                   return std::make_pair(kf.second, kf.first);
                 });

  std::sort(connected_kf.begin(), connected_kf.end(),
            [](const auto &a, const auto &b) { return a.first > b.first; });

  _ordered_kf.clear();
  std::transform(connected_kf.begin(), connected_kf.end(),
                 std::back_inserter(_ordered_kf),
                 [](const auto &kf) { return kf.second; });
}

// Get the ordered connected keyframes vector with size
std::vector<Frame::Ptr> Frame::GetOrderedConnectedKFs(unsigned int size) {
  std::unique_lock<std::mutex> lock(_frame_connections_mutex);

  if (_ordered_kf.size() <= size)
    return _ordered_kf;
  else
    return std::vector<Frame::Ptr>(_ordered_kf.begin(),
                                   _ordered_kf.begin() + size);
}

std::set<Frame::Ptr> Frame::GetConnectedKFsSet() {
  std::unique_lock<std::mutex> lock(_frame_connections_mutex);
  std::set<Frame::Ptr> tmp;
  for (auto &kf : _kf_connected)
    tmp.insert(kf.first);

  return tmp;
}

int Frame::GetNumTrackedFeatures() {
  std::unique_lock<std::mutex> lock(_frame_connections_mutex);
  return std::count_if(_features_left.begin(), _features_left.end(),
                       [](const auto &feat) {
                         return feat->_status == Feature::Status::TRACKED;
                       });
}
} // namespace structures