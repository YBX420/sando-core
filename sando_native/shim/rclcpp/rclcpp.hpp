// de-ROS shim for <rclcpp/rclcpp.hpp> — minimal stand-ins so the native SANDO core compiles WITHOUT ROS.
// Only what the compiled core actually uses: Time/Duration/Clock, get_logger/Logger, RCLCPP_ERROR/WARN.
#pragma once
#include <cstdio>
#include <string>
#include <chrono>
#include <iomanip>   // native SANDO relies on ROS headers transitively pulling these
#include <sstream>
#include <numeric>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <limits>

namespace rclcpp {

struct Time {
  double t_{0.0};
  Time() = default;
  Time(double s) : t_(s) {}
  double seconds() const { return t_; }
  double nanoseconds() const { return t_ * 1e9; }
};

struct Duration {
  double d_{0.0};
  Duration() = default;
  explicit Duration(double s) : d_(s) {}
  static Duration from_seconds(double s) { return Duration(s); }
  double seconds() const { return d_; }
};

struct Clock {
  Clock() = default;
  explicit Clock(int) {}
  Time now() const {
    auto n = std::chrono::steady_clock::now().time_since_epoch();
    return Time(std::chrono::duration<double>(n).count());
  }
};

struct Logger {
  std::string name_;
  const char* get_name() const { return name_.c_str(); }
};
inline Logger get_logger(const std::string& n) { return Logger{n}; }

}  // namespace rclcpp

// logging macros -> stderr (dead viz/diagnostic paths). variadic, printf-style.
#define RCLCPP_ERROR(logger, ...)  do { std::fprintf(stderr, "[ERROR] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define RCLCPP_WARN(logger, ...)   do { std::fprintf(stderr, "[WARN] ");  std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define RCLCPP_INFO(logger, ...)   do { } while (0)
#define RCLCPP_DEBUG(logger, ...)  do { } while (0)
