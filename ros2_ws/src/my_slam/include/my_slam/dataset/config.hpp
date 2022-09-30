#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <common.hpp>
namespace dataset {
class Config {
private:
  Config() {}
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;

public:
  ~Config();
  static bool SetParameterFile(const std::string &filename);

  template <typename T> static T Get(const std::string &key) {
    return T(Config::config_->file_[key]);
  }

  static void build();
};
} // namespace dataset
#endif