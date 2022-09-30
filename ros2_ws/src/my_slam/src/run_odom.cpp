#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <common.hpp>
#include <my_slam/control/frontend.hpp>
#include <my_slam/control/viewer.hpp>
#include <my_slam/control/vio.hpp>
#include <my_slam/dataset/reader.hpp>
#include <my_slam/structures/frame.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <thread>

const std::string path = "/home/user/ros2_ws/src/my_slam/config/params.yaml";

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  const rclcpp::NodeOptions options;

  auto data_reader = dataset::Reader::Ptr(new dataset::Reader(options));
  exec.add_node(data_reader);

  auto front_end = my_slam::Frontend::Ptr(new my_slam::Frontend(options));
  exec.add_node(front_end);

  auto vio = my_slam::VisualOdom::Ptr(new my_slam::VisualOdom(options));
  exec.add_node(vio);

  auto viewer = my_slam::Viewer::Ptr(new my_slam::Viewer(options));
  exec.add_node(viewer);

  std::thread exec_thread([&]() { exec.spin(); });

  //   vio->Init(front_end, data_reader, viewer, path);
  vio->Init(front_end, data_reader, viewer, path);
  vio->Run();
  exec_thread.join();
  rclcpp::shutdown();
  std::cout << "End of Program" << std::endl;
}