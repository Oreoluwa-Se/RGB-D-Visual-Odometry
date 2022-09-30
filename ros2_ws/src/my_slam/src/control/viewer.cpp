#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/types.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <iterator>
#include <my_slam/control/viewer.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace my_slam {

Viewer::Viewer(const rclcpp::NodeOptions &options)
    : Node("slam_viewer_node", options) {}

void Viewer::Init(bool show_point_cloud, double freq_hq) {
  _freq_hq = freq_hq;

  // for publishing images
  _pub_image_detected = this->create_publisher<CompressedImage>(
      "/RGB_image_viewer/detected/compressed", 1);
  _pub_image_matched = this->create_publisher<CompressedImage>(
      "/RGB_image_viewer/matched/compressed", 1);

  // for viewing point clouds
  if (show_point_cloud) {
    _pub_pointCloud =
        this->create_publisher<PointCloud2>("features_point_cloud", 1);
    _viewer_thread = std::thread(std::bind(&Viewer::ThreadLoop, this));
    _show_point_cloud = true;
  }
}

void Viewer::Stop() {
  _viewer_running = false;
  _viewer_thread.join();
  rclcpp::shutdown();
}

void Viewer::AddFrame(Frame::Ptr curr_frame) {
  if (!_show_point_cloud)
    return;

  std::unique_lock<std::mutex> lock(_viewer_mutex);
  _curr_frame = curr_frame;
}

void Viewer::ThreadLoop() {
  rclcpp::Rate rate(_freq_hq);

  while (rclcpp::ok() && _viewer_running) {

    std::unique_lock<std::mutex> lock(_viewer_mutex);
    if (_map && _show_point_cloud)
      DrawMapPoints();

    rate.sleep();
  }
}

void Viewer::PlotFrameImage(cv::Mat &&image) {
  if (image.empty())
    return;

  cv::Mat outimg;
  if (!_show_point_cloud)
    cv::resize(image, outimg, cv::Size(), 0.6, 0.6, cv::INTER_LINEAR);
  else
    cv::resize(image, outimg, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

  std::string topic_name = "/RGB_image_viewer/detected/compressed";
  PublishImage(std::move(outimg), topic_name, _pub_image_detected);
}

void Viewer::PlotFrameTracked(cv::Mat &&image) {
  if (image.empty())
    return;

  cv::Mat outimg;

  if (!_show_point_cloud)
    cv::resize(image, outimg, cv::Size(), 0.6, 0.6, cv::INTER_LINEAR);
  else
    cv::resize(image, outimg, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

  std::string topic_name = "/RGB_image_viewer/matched/compressed";
  PublishImage(std::move(outimg), topic_name, _pub_image_matched);
}

void Viewer::PublishImage(cv::Mat &&image, std::string &topic,
                          rclcpp::Publisher<CompressedImage>::SharedPtr pub) {

  CompressedImage out;
  std::vector<uchar> buffer;
  // parameter for decoding image into png
  std::vector<int> params;
  params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  params.push_back(9);
  cv::imencode(".png", image, buffer, params);
  // convert type
  std::vector<uint8_t> arr;
  std::transform(buffer.begin(), buffer.end(), std::back_inserter(arr),
                 [](const auto &elem) { return uint8_t(elem); });

  // setup image compression mesage stuff
  out.header.stamp = this->get_clock()->now();
  out.header.frame_id = topic;
  out.format = "png";
  out.data = arr;

  pub->publish(out);
}

void Viewer::DrawMapPoints() {
  auto active_map_points = _map->GetActiveMapPoints();

  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
  // split active, current, and previous frames
  for (auto &mp : _map->GetAllMapPoints()) {
    auto pos = mp.second->position();
    pcl::PointXYZRGB pt;

    if (pos[2] > 0.05) {
      // check if in active frame
      if (active_map_points.count(mp.second->_id))
        pt = pcl::PointXYZRGB(44, 58, 71);
      // for other frames
      else
        pt = pcl::PointXYZRGB(202, 211, 200);

      pt.x = pos[0];
      pt.y = pos[1];
      pt.z = pos[2];

      cloud_.points.push_back(pt);
    }
  }

  PointCloudPublisher(std::move(cloud_));
}

void Viewer::PointCloudPublisher(pcl::PointCloud<pcl::PointXYZRGB> &&pub) {
  _pc_msg = std::make_shared<PointCloud2>();
  pcl::toROSMsg(pub, *_pc_msg);

  _pc_msg->header.frame_id = _camera_frame;
  _pc_msg->header.stamp = this->get_clock()->now();
  _pub_pointCloud->publish(*_pc_msg);
}

} // namespace my_slam