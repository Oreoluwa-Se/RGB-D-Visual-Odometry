#ifndef HELPER_MODULES_HPP
#define HELPER_MODULES_HPP

#include <common.hpp>

enum struct ErrorType : int { INFO, WARN, FATAL, ERROR };

enum class SystemStatus { INIT, TRACKING_GOOD, TRACKING_BAD, LOST };

// for logging information
inline void error_check(bool cond, const std::string &msg, ErrorType type) {

  if (!cond) {
    switch (type) {
    case ErrorType::INFO:
      LOG(INFO) << msg;
      break;
    case ErrorType::WARN:
      LOG(WARNING) << msg;
      break;
    case ErrorType::FATAL:
      LOG(FATAL) << msg;
    case ErrorType::ERROR:
      LOG(ERROR) << msg;
    }
  }
}

template <typename T>
std::vector<T> slice(std::vector<T> const &v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n;

  std::vector<T> vec(first, last);
  return vec;
}

template <typename T> inline double quad(std::vector<T> &a) {
  double q;
  int n = a.size();

  if (n % 2 == 0)
    q = (double)(a[n / 2 - 1] + a[n / 2]) / 2;
  else
    q = (double)a[n / 2];

  return q;
}

template <typename T>
inline void IQR(std::vector<T> &a, std::vector<double> &values) {
  // remove duplicates
  std::set<T> val;
  for (size_t i = 0; i < a.size(); i++)
    val.insert(a[i]);

  if (a.size() == 1 || val.size() == 1) {
    values.push_back(0);
    values.push_back(a[0]);
    values.push_back(a[0]);
    return;
  }

  // convert set to vector
  std::vector<T> target(val.size());
  std::copy(val.begin(), val.end(), target.begin());

  std::vector<T> l(target.begin(), target.begin() + int(target.size() / 2));
  std::vector<T> h;

  if (int(target.size()) % 2 == 0)
    h.insert(h.end(), target.begin() + int(target.size() / 2), target.end());
  else
    h.insert(h.end(), target.begin() + int(target.size() / 2) + 1,
             target.end());

  values.push_back(quad(l));
  values.push_back(quad(h));
  values.push_back(values[1] - values[0]);
}
#endif