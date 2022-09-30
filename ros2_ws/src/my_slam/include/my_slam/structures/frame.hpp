#ifndef FRAME_HPP
#define FRAME_HPP

#include <common.hpp>
#include <my_slam/sensor/camera.hpp>

namespace structures {
struct Feature;

struct Frame : std::enable_shared_from_this<Frame> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Frame> Ptr;

  unsigned long _id = 0;
  unsigned long _keyframe_id = 0;
  bool _is_keyframe = false;
  double _time_stamp;
  bool _tracked_img_populated = false;

  // image attributes and descriptors
  cv::Mat _left_img, _desc_left, _detected_feat_img, _tracked_feat_img;

  // extracted features in left image
  std::vector<std::shared_ptr<Feature>> _features_left;

  // depth information
  std::vector<double> _depth;

  Frame() {}

  Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &left);

  // set and get pose, thread safe
  SE3 Pose() {
    std::unique_lock<std::mutex> lck(_pose_mutex);
    return _pose;
  }

  void SetPose(const SE3 &pose) {
    std::unique_lock<std::mutex> lck(_pose_mutex);
    _pose = pose;
  }

  void SetKeyFrame();

  // updates the connections
  void UpdateCovisibleConnects(int occurences);

  int GetNumTrackedFeatures();

  // Get the ordered connected keyframes vector with size
  std::vector<Frame::Ptr> GetOrderedConnectedKFs(unsigned int size);

  std::set<Frame::Ptr> GetConnectedKFsSet();

  std::unordered_map<Frame::Ptr, int> GetConnectedKFs() {
    std::unique_lock<std::mutex> lock(_frame_connections_mutex);
    return _kf_connected;
  }

  std::vector<std::shared_ptr<Feature>> GetFeatures() {
    std::unique_lock<std::mutex> lock(_frame_connections_mutex);
    return _features_left;
  }

  static std::shared_ptr<Frame> CreateFrame();

private:
  void AddConnection(Frame::Ptr frame, int weight);
  void SortConnectedKeyFrames();
  std::mutex _pose_mutex;
  SE3 _pose; // Tfw

  // tracks keyframes that have a lot same observed points
  std::unordered_map<Frame::Ptr, int> _kf_connected;
  std::mutex _frame_connections_mutex;
  // Orders connected keyframes from large weight to small
  std::vector<Frame::Ptr> _ordered_kf;
};

} // namespace structures

#endif