#include <helper_modules.hpp>
#include <my_slam/dataset/config.hpp>
#include <opencv4/opencv2/core/persistence.hpp>
#include <sys/stat.h>

bool IsPathExist(const std::string &s) {
  struct stat buffer;
  return (stat(s.c_str(), &buffer) == 0);
}

namespace dataset {
std::shared_ptr<Config> Config::config_ = nullptr;
bool Config::SetParameterFile(const std::string &filename) {
  if (config_ == nullptr)
    config_ = std::shared_ptr<Config>(new Config);

  if (IsPathExist(filename)) {
    config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
      config_->file_.release();
      error_check(config_->file_.isOpened() == false, "Cannot open config file",
                  ErrorType::FATAL);
    }
  }
  return true;
}

void Config::build() {
  if (Config::config_ == nullptr)
    Config::config_ = std::shared_ptr<Config>(new Config);
}

Config::~Config() {
  if (file_.isOpened())
    file_.release();
}

} // namespace dataset