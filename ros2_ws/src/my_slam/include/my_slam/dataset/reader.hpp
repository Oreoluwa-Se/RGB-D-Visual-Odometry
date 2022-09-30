#ifndef READER_HPP
#define READER_HPP

#include "my_slam/visibility_control.h"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <common.hpp>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <my_slam/sensor/camera.hpp>
#include <my_slam/structures/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <utility>

using nav_msgs::msg::Odometry;
using sensor::Camera;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::PointCloud2;
using structures::Frame;

namespace dataset {
class Reader : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Reader> Ptr;

  explicit Reader(const rclcpp::NodeOptions &options);

  Camera::Ptr GetCamera(int cam_id) const { return _cameras.at(cam_id); }

  void SetParameters(const std::string &file);

  void InsertImage(const cv::Mat &&img);

  void InsertDepth(const std::vector<double> &&depth);

  Frame::Ptr NextFrame();

  std::string GetCameraFrameID() { return _frame_id; }

  std::pair<std::string, std::string> GetOdomFrameInfo() {
    return std::make_pair(_odom_frame_id, _odom_child_id);
  }
  bool Init();

  double _init_position[3];

private:
  void DepthCallback(const PointCloud2::SharedPtr msg);
  void ImageCallback(const CompressedImage::SharedPtr msg);
  void CameraInfoCallback(const CameraInfo::SharedPtr msg);
  void OdometryInfo(const Odometry::SharedPtr msg);
  void GetImageAndDepth(cv::Mat &img, std::vector<double> &vec);

  rclcpp::Subscription<CameraInfo>::SharedPtr _cam_info;
  rclcpp::Subscription<CompressedImage>::SharedPtr _img_sub;
  rclcpp::Subscription<PointCloud2>::SharedPtr _depth_sub;
  rclcpp::Subscription<Odometry>::SharedPtr _odom_sub;

  std::deque<cv::Mat> _rgb_images_left;
  std::deque<std::vector<double>> _depth_data;

  std::array<float, 12> _proj_data;
  std::vector<Camera::Ptr> _cameras;
  bool _proj_data_filled = false;
  int _current_image_index;
  std::string _frame_id;
  bool _cam_size_set = false;

  std::string _odom_frame_id;
  std::string _odom_child_id;
  bool _frame_info_set = false;

  std::mutex _image_mutex;
  std::mutex _depth_mutex;

  double _freq_hq = 10;
  size_t _image_trail = 5;
};
} // namespace dataset
#endif