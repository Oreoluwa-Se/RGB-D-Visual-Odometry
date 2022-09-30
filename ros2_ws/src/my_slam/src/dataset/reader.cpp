#include "cv_bridge/cv_bridge.h"
#include <helper_modules.hpp>
#include <mutex>
#include <my_slam/dataset/config.hpp>
#include <my_slam/dataset/reader.hpp>
#include <my_slam/utils/image_funcs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>

using std::size_t;
using std::placeholders::_1;
using utils::ImageFunctions;

namespace dataset {

Reader::Reader(const rclcpp::NodeOptions &options)
    : Node("data_reader_node", options), _current_image_index(0) {
  Config::build();
}

void Reader::SetParameters(const std::string &file) {

  Config::SetParameterFile(file);

  _image_trail = size_t(Config::Get<int>("image_trail"));
  _freq_hq = Config::Get<double>("sleep_time");

  rclcpp::QoS cam_info_qos(Config::Get<int>("cam_info_queue"));
  cam_info_qos.keep_last(Config::Get<int>("cam_info_queue"));
  cam_info_qos.reliable();
  cam_info_qos.durability_volatile();
  _cam_info = this->create_subscription<CameraInfo>(
      "/robot/front_rgbd_camera/rgb/camera_info", cam_info_qos,
      std::bind(&Reader::CameraInfoCallback, this, _1));

  // get quality of service for image
  rclcpp::QoS image_qos(Config::Get<int>("image_queue"));
  image_qos.keep_last(Config::Get<int>("image_queue"));
  image_qos.reliable();
  image_qos.durability_volatile();
  _img_sub = this->create_subscription<CompressedImage>(
      "/robot/front_rgbd_camera/rgb/image_raw/compressed", image_qos,
      std::bind(&Reader::ImageCallback, this, _1));

  // get quality of service for depth points
  rclcpp::QoS depth_qos(Config::Get<int>("image_queue"));
  depth_qos.keep_last(Config::Get<int>("image_queue"));
  depth_qos.reliable();
  depth_qos.durability_volatile();
  _depth_sub = this->create_subscription<PointCloud2>(
      "/robot/front_rgbd_camera/depth/points", depth_qos,
      std::bind(&Reader::DepthCallback, this, _1));

  // get quality of service for depth points
  rclcpp::QoS nav_qos(1);
  nav_qos.keep_last(1);
  nav_qos.best_effort();
  nav_qos.durability_volatile();
  _odom_sub = this->create_subscription<Odometry>(
      "/odom", nav_qos, std::bind(&Reader::OdometryInfo, this, _1));

  rclcpp::Rate rate(_freq_hq);
  while (!_proj_data_filled) {
    rate.sleep();
  }
}

void Reader::OdometryInfo(const Odometry::SharedPtr msg) {
  if (!_frame_info_set) {
    _init_position[0] = msg->pose.pose.position.x;
    _init_position[1] = msg->pose.pose.position.y;
    _init_position[2] = msg->pose.pose.position.z;
    _odom_frame_id = msg->header.frame_id;
    _odom_child_id = msg->child_frame_id;
    _frame_info_set = true;
    _odom_sub.reset();
  }
}

void Reader::CameraInfoCallback(const CameraInfo::SharedPtr msg) {

  if (!_proj_data_filled) {
    _frame_id = msg->header.frame_id;
    size_t msg_size = sizeof(msg->p) / sizeof(msg->p[0]);
    for (size_t i = 0; i < msg_size; i++) {
      _proj_data[i] = msg->p[i];
    }

    _proj_data_filled = true;
    _cam_info.reset();
  }
}

void Reader::ImageCallback(const CompressedImage::SharedPtr msg) {
  rclcpp::Rate rate(_freq_hq);
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  std::unique_lock<std::mutex> lock(_image_mutex);
  if (_rgb_images_left.size() >= _image_trail) {
    _rgb_images_left.pop_front();
  }

  _rgb_images_left.emplace_back(std::move(cv_ptr->image));
  rate.sleep();
}

void Reader::DepthCallback(const PointCloud2::SharedPtr msg) {
  std::vector<double> depth;
  rclcpp::Rate rate(_freq_hq);

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

  std::unique_lock<std::mutex> lock(_depth_mutex);
  if (_depth_data.size() >= _image_trail)
    _depth_data.pop_front();

  _depth_data.emplace_back(std::move(depth));
  rate.sleep();
}

bool Reader::Init() {
  matrix<3> K;
  vector<3> t;

  K << _proj_data[0], _proj_data[1], _proj_data[2], _proj_data[4],
      _proj_data[5], _proj_data[6], _proj_data[8], _proj_data[9],
      _proj_data[10];

  (t << _proj_data[3], _proj_data[7], _proj_data[11]).finished();
  t = K.inverse() * t; // normalized plane coordinates

  std::vector<double> coeff = {K(0, 0), K(1, 1), K(0, 2), K(1, 2)};
  std::vector<double> dist;
  auto pose = SE3(SO3(), t);

  Camera::Ptr new_camera(new Camera(coeff, dist, t.norm(), pose));
  _cameras.push_back(new_camera);

  _current_image_index = 0;
  return true;
}

void Reader::GetImageAndDepth(cv::Mat &img, std::vector<double> &depth) {
  rclcpp::Rate rate(_freq_hq);
  bool occupied = false;

  while (!occupied) {
    std::unique_lock<std::mutex> l1(_image_mutex);
    std::unique_lock<std::mutex> l2(_depth_mutex);

    occupied = !_rgb_images_left.empty() && !_depth_data.empty();
    l1.unlock();
    l2.unlock();

    if (occupied)
      break;
    else {
      rate.sleep();
    }
  }

  std::unique_lock<std::mutex> l1(_image_mutex);
  std::unique_lock<std::mutex> l2(_depth_mutex);

  /* ================================== */
  img = std::move(_rgb_images_left.front());
  depth = std::move(_depth_data.front());

  _depth_data.pop_front();
  _rgb_images_left.pop_front();
}

Frame::Ptr Reader::NextFrame() {
  cv::Mat img;
  std::vector<double> depth;

  GetImageAndDepth(img, depth);
  auto new_frame = Frame::CreateFrame();

  if (!_cam_size_set) {
    for (auto &cam : _cameras)
      cam->setWidthHeight(img.cols, img.rows);
    _cam_size_set = true;
  }

  new_frame->_left_img = ImageFunctions::apply_histeq(std::move(img));
  new_frame->_depth = depth;
  _current_image_index++;

  return new_frame;
}

} // namespace dataset
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dataset::Reader)
