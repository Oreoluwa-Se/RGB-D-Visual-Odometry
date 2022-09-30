#include <common.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <my_slam/dataset/config.hpp>
#include <opencv4/opencv2/highgui.hpp>

using std::placeholders::_1;

using dataset::Config;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

namespace trying {
class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    Config::SetParameterFile(
        "/home/user/ros2_ws/src/my_slam/config/params.yaml");

    rclcpp::QoS cam_info_qos(Config::Get<int>("cam_info_queue"));
    cam_info_qos.keep_last(Config::Get<int>("cam_info_queue"));
    cam_info_qos.reliable();
    cam_info_qos.durability_volatile();

    _cam_info = this->create_subscription<CameraInfo>(
        "/robot/front_rgbd_camera/rgb/camera_info", cam_info_qos,
        std::bind(&MinimalSubscriber::topic_callback, this, _1));

    // get quality of service for depth image
    rclcpp::QoS image_qos(Config::Get<int>("image_queue"));
    image_qos.keep_last(Config::Get<int>("image_queue"));
    image_qos.reliable();
    image_qos.durability_volatile();

    _img_sub = this->create_subscription<CompressedImage>(
        "/robot/front_rgbd_camera/rgb/image_raw/compressed", image_qos,
        std::bind(&MinimalSubscriber::ImageCallback, this, _1));

    // get quality of service for depth image
    rclcpp::QoS depth_qos(Config::Get<int>("image_queue"));
    depth_qos.keep_last(Config::Get<int>("image_queue"));
    depth_qos.reliable();
    depth_qos.durability_volatile();

    _depth_sub = this->create_subscription<PointCloud2>(
        "/robot/front_rgbd_camera/depth/points", depth_qos,
        std::bind(&MinimalSubscriber::DepthCallback, this, _1));
  }

private:
  // variables
  rclcpp::Subscription<CompressedImage>::SharedPtr _img_sub;
  rclcpp::Subscription<PointCloud2>::SharedPtr _depth_sub;
  rclcpp::Subscription<CameraInfo>::SharedPtr _cam_info;
  std::deque<std::vector<double>> _depth_data;
  std::deque<cv::Mat> _rgb_images_left;
  std::array<float, 12> _proj_data;
  bool _proj_data_filled = false;

  // functions
  void topic_callback(const CameraInfo::SharedPtr msg) {
    if (!_proj_data_filled) {
      // _frame_id = msg->header.frame_id;
      size_t msg_size = sizeof(msg->p) / sizeof(msg->p[0]);

      for (size_t i = 0; i < msg_size; i++) {
        _proj_data[i] = msg->p[i];
      }

      _proj_data_filled = true;
    }
    RCLCPP_INFO(this->get_logger(), "filled");
  }

  void ImageCallback(const CompressedImage::SharedPtr msg) {

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    _rgb_images_left.emplace_back(std::move(cv_ptr->image));
    RCLCPP_INFO(this->get_logger(), "image ptr");
  }

  void DepthCallback(const PointCloud2::SharedPtr msg) {
    std::vector<double> depth;

    for (size_t u = 0; u < size_t(msg->height); u++) {
      for (size_t v = 0; v < size_t(msg->width); v++) {
        size_t array_pos = size_t(u * msg->row_step + v * msg->point_step);
        size_t x = array_pos + size_t(msg->fields[0].offset);
        size_t y = array_pos + size_t(msg->fields[1].offset);
        size_t z = array_pos + size_t(msg->fields[2].offset);

        float X = 0.0, Y = 0.0, Z = 0.0;
        std::memcpy(&X, &msg->data[x], sizeof(float));
        std::memcpy(&Y, &msg->data[y], sizeof(float));
        std::memcpy(&Z, &msg->data[z], sizeof(float));

        // distance from depth
        depth.push_back(sqrt(X * X + Y * Y + Z * Z));
      }
    }
    _depth_data.emplace_back(std::move(depth));
    RCLCPP_INFO(this->get_logger(), "Depth pushed");
  }
};

} // namespace trying
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trying::MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}