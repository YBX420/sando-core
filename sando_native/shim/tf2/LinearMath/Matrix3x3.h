// de-ROS shim for tf2::Matrix3x3(q).getRPY(r,p,y) — the only usage in the native SANDO core.
#pragma once
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
namespace tf2 {
class Matrix3x3 {
 public:
  Quaternion q_;
  Matrix3x3() = default;
  explicit Matrix3x3(const Quaternion& q) : q_(q) {}
  void getRPY(double& roll, double& pitch, double& yaw, unsigned int = 1) const {
    const double x = q_.x_, y = q_.y_, z = q_.z_, w = q_.w_;
    // standard quaternion -> roll/pitch/yaw (XYZ)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    double sinp = 2.0 * (w * y - z * x);
    pitch = std::fabs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }
};
}  // namespace tf2
