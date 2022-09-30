#ifndef VIEWER_HPP
#define VIEWER_HPP

#include "image_transport/publisher.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <common.hpp>
#include <cv_bridge/cv_bridge.h>
#include <future>
#include <image_transport/image_transport.hpp>
#include <my_slam/structures/frame.hpp>
#include <my_slam/structures/map.hpp>
#include <my_slam/structures/map_point.hpp>
#include <my_slam/visibility_control.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
namespace my_slam {
// using image_transport::ImageTransport;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::PointCloud2;
using structures::Frame;
using structures::Map;

class Viewer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Viewer> Ptr;

  Viewer(const rclcpp::NodeOptions &options);

  void Init(bool show_point_cloud, double freq_hq);

  void SetMap(Map::Ptr map) { _map = map; }

  void SetCameraFrame(const std::string &camera_frame) {
    _camera_frame = camera_frame;
  }

  void AddFrame(Frame::Ptr current_frame);

  void Stop();

  void PlotImage(cv::Mat &&detected_image, cv::Mat &&tracked_image) {

    std::async(std::launch::async, &Viewer::PlotFrameImage, this,
               std::move(detected_image));

    std::async(std::launch::async, &Viewer::PlotFrameTracked, this,
               std::move(tracked_image));
  }

private:
  void ThreadLoop();

  void DrawFrame(Frame::Ptr frame, const float *color);

  void DrawMapPoints();

  void PointCloudPublisher(pcl::PointCloud<pcl::PointXYZRGB> &&pub);

  void PublishImage(cv::Mat &&image, std::string &topic,
                    rclcpp::Publisher<CompressedImage>::SharedPtr pub);

  void PlotFrameImage(cv::Mat &&image);

  void PlotFrameTracked(cv::Mat &&image);

  rclcpp::Publisher<CompressedImage>::SharedPtr _pub_image_detected = nullptr;
  rclcpp::Publisher<CompressedImage>::SharedPtr _pub_image_matched = nullptr;
  rclcpp::Publisher<PointCloud2>::SharedPtr _pub_pointCloud = nullptr;
  PointCloud2::SharedPtr _pc_msg;

  std::thread _viewer_thread;
  Map::Ptr _map = nullptr;
  Frame::Ptr _curr_frame = nullptr;

  bool _map_updated = false;
  bool _viewer_running = true;
  std::string _camera_frame;
  std::mutex _viewer_mutex;
  double _freq_hq = 0.0;

  bool _show_point_cloud = false;
};

} // namespace my_slam
#endif