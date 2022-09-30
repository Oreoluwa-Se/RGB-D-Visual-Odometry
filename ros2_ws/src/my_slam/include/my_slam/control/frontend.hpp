#ifndef FRONTEND_HPP
#define FRONTEND_HPP

#include "my_slam/structures/feature.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include <common.hpp>
#include <helper_modules.hpp>
#include <my_slam/control/backend.hpp>
#include <my_slam/control/viewer.hpp>
#include <my_slam/structures/feature.hpp>
#include <my_slam/structures/frame.hpp>
#include <my_slam/structures/map.hpp>
#include <my_slam/visibility_control.h>

using sensor::Camera;
using structures::Feature;
using structures::Frame;
using structures::Map;

using nav_msgs::msg::Odometry;
using tf2_ros::Buffer;
using tf2_ros::TransformListener;

namespace my_slam {
using std::size_t;

class Frontend : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Frontend> Ptr;

  Frontend(const rclcpp::NodeOptions &options)
      : Node("frontend_node", options) {
    _pub_camera = this->create_publisher<Odometry>("predicted_odom", 1);
    _buff = std::make_unique<Buffer>(this->get_clock());
    _listener = std::make_shared<TransformListener>(*_buff);
  }

  void SetParameters(int num_features_init, int num_features,
                     int num_features_tracking, int num_features_tracking_bad,
                     int num_features_needed_for_keyframe,
                     bool use_loop_closure, double max_depth, double min_depth,
                     size_t num_detected_feats, double filter_radius,
                     double scale_factor, int lvl_pyramid, int border,
                     double freq_hq, int g2o_ransac_iter, int feat_opt_number,
                     int chi_dof, double cv_ransac_thresh,
                     double cv_ransac_percent, int cv_ransac_maxIter,
                     int half_patch_size, double min_dist_loop_closure,
                     int opt_flow_ransac_num);

  bool AddFrame(Frame::Ptr &&frame);

  void SetMap(Map::Ptr map) { _map = map; }

  void SetBackend(Backend::Ptr backend) { _backend = backend; }

  void SetViewer(Viewer::Ptr viewer) { _viewer = viewer; }

  SystemStatus GetStatus() const { return _status; }

  void SetCameras(Camera::Ptr left) { _camera_left = left; }

  void PublishCameraOdometry(const matrix<4> &&pose);

  void SetCameraFrame();

  void SetFramesAndPosition(const std::string &cam_link,
                            const std::string &header_frame_id,
                            const std::string &base_frame,
                            const double arr[3]) {
    _cam_link = cam_link;
    _base_link = base_frame;
    _header_link = header_frame_id;
    _initial_position[0] = arr[0];
    _initial_position[1] = arr[1];
    _initial_position[2] = arr[2];
  }

private:
  /**
   * Track in normal mode
   * @return true if success
   */
  bool Track();

  /**
   * Reset when lost
   * @return true if success
   */
  bool Reset();

  /**
   * Track with last frame
   * @return num of tracked points
   */
  int TrackLastFrame();

  /**
   * estimate current frame's pose
   * @return num of inliers
   */
  int EstimateCurrentPose();

  /**
   * set current frame as a keyframe and insert it into backend
   * @return true if success
   */
  bool InsertKeyframe();

  /**
   * Try init the frontend with stereo images saved in current_frame_
   * @return true if success
   */
  bool RGBDInit();

  /**
   * Detect features in left image in current_frame_
   * keypoints will be saved in current_frame_
   * @return
   */
  int DetectFeatures();

  /**
   * Build the initial map with single image
   * @return true if succeed
   */
  int BuildInitMap();

  /**
   * Triangulate the 2D points in current frame
   * @return num of triangulated points
   */
  int TriangulateNewPoints();

  /**
   * Set the features in keyframe as new observation of the map points
   */
  void SetObservationsForKeyFrame();

  /**
   * @brief For matching pixels between images
   */
  void OptFlow(const cv::Mat &img1, const cv::Mat &img2,
               std::vector<cv::Point2f> &kps_1, std::vector<cv::Point2f> &kps_2,
               std::vector<uchar> &status);

  /**
   * @brief Overhead function for matching images. Uses OptFlow function under
   * hood.
   *  - Implements outlier methods to streamline points.
   * @param survived: returns keypoint of matched images that survived the
   * purge
   * @param survived_idx: original index of survived keypoints..[for keypoint
   * assosciation]
   */
  void MatchedFeatures(const cv::Mat &img1, const cv::Mat &img2,
                       std::vector<Feature::Ptr> &feats, Camera::Ptr &cam,
                       const SE3 &curr_pose,
                       std::vector<cv::KeyPoint> &survived,
                       std::vector<int> &survived_idx);

  SystemStatus _status = SystemStatus::INIT;

  Frame::Ptr _current_frame = nullptr;
  Frame::Ptr _last_frame = nullptr;
  Camera::Ptr _camera_left = nullptr;
  Viewer::Ptr _viewer = nullptr;

  Map::Ptr _map = nullptr;
  std::shared_ptr<Backend> _backend = nullptr;
  bool _use_loop_closure = false;
  SE3 _relative_motion;
  SE3 _last_loopframe_pose;
  SE3 _last_keyframe_pose;

  int _tracking_inliers = 0; // inliers, used for testing new keyframes

  // params
  int _num_features = 200;
  int _num_features_init = 100;
  int _num_features_tracking = 50;
  int _num_features_tracking_bad = 20;
  int _num_features_needed_for_keyframe = 80;

  rclcpp::Publisher<Odometry>::SharedPtr _pub_camera = nullptr;
  std::shared_ptr<TransformListener> _listener = nullptr;
  rclcpp::TimerBase::SharedPtr _timer = nullptr;
  std::unique_ptr<Buffer> _buff = nullptr;
  matrix<3> _camera_to_base;
  bool _camera_to_base_bool = false;

  std::string _cam_link;
  std::string _header_link;
  std::string _base_link;
  bool _frame_set_check = false;

  // function parameters
  double _max_depth = 3.5;
  double _min_depth = 0.1;
  size_t _num_detected_feats = 1000;
  double _filter_radius = 1.0;
  double _scale_factor = 1.2;
  int _lvl_pyramid = 8;
  int _border = 10;
  double _freq_hq = 10; // to include
  int _g2o_ransac_iter = 10;
  int _feat_opt_number = 4;
  int _chi_dof = 2;
  double _cv_ransac_thresh = 1;
  double _cv_ransac_percent = 0.99;
  int _cv_ransac_maxIter = 100;
  int _half_patch_size = 11;
  double _min_dist_loop_closure = 0.5;
  int _opt_flow_ransac_num = 30;
  double _initial_position[3];
};
} // namespace my_slam

#endif