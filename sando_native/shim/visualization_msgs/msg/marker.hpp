// de-ROS shim for visualization_msgs::msg::Marker — fields + type/action constants the core's
// (dead, un-rendered) viz helpers touch. We never render; this just lets that code compile.
#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace visualization_msgs { namespace msg {
struct Marker {
  // marker types
  static constexpr int32_t ARROW = 0, CUBE = 1, SPHERE = 2, CYLINDER = 3,
      LINE_STRIP = 4, LINE_LIST = 5, CUBE_LIST = 6, SPHERE_LIST = 7,
      POINTS = 8, TEXT_VIEW_FACING = 9;
  // actions
  static constexpr int32_t ADD = 0, MODIFY = 0, DELETE = 2, DELETEALL = 3;

  std_msgs::msg::Header header;
  std::string ns;
  int32_t id = 0;
  int32_t type = 0;
  int32_t action = 0;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Vector3 scale;
  std_msgs::msg::ColorRGBA color;
  rclcpp::Duration lifetime{0.0};
  bool frame_locked = false;
  std::vector<geometry_msgs::msg::Point> points;
  std::vector<std_msgs::msg::ColorRGBA> colors;
  std::string text;
  int32_t mesh_use_embedded_materials = 0;
};
}}  // namespace visualization_msgs::msg
