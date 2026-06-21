#pragma once
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/header.hpp>
namespace geometry_msgs{namespace msg{struct TwistStamped{std_msgs::msg::Header header;Twist twist;};}}