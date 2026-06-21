// de-ROS shim for tf2::Quaternion — only ctor + accessors + setRPY used by quaternion2Euler.
#pragma once
#include <cmath>
namespace tf2 {
class Quaternion {
 public:
  double x_{0}, y_{0}, z_{0}, w_{1};
  Quaternion() = default;
  Quaternion(double x, double y, double z, double w) : x_(x), y_(y), z_(z), w_(w) {}
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double w() const { return w_; }
  void setRPY(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    w_ = cr * cp * cy + sr * sp * sy;
    x_ = sr * cp * cy - cr * sp * sy;
    y_ = cr * sp * cy + sr * cp * sy;
    z_ = cr * cp * sy - sr * sp * cy;
  }
  void normalize() {
    double n = std::sqrt(x_*x_ + y_*y_ + z_*z_ + w_*w_);
    if (n > 0) { x_ /= n; y_ /= n; z_ /= n; w_ /= n; }
  }
};
}  // namespace tf2
