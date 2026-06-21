#pragma once
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <string>
namespace geometry_msgs{namespace msg{struct TransformStamped{std_msgs::msg::Header header;std::string child_frame_id;Transform transform;};}}