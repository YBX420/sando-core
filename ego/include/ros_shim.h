// Minimal ROS-1 shim so EGO-Planner's pure-algorithm files compile WITHOUT ROS.
#pragma once
#include <cstdio>
#include <cstdarg>
#include <chrono>
namespace ros {
struct Duration { double s{0}; Duration(){} Duration(double x):s(x){} double toSec() const {return s;} Duration operator+(const Duration& o) const {return Duration(s+o.s);} };
struct Time {
  double s{0}; Time(){} Time(double x):s(x){}
  static Time now(){ using namespace std::chrono;
    return Time(duration<double>(steady_clock::now().time_since_epoch()).count()); }
  double toSec() const {return s;}
  Duration operator-(const Time& o) const {return Duration(s-o.s);}
};
struct NodeHandle { // no-op param: standalone sets params via structs, not the param server
  NodeHandle(){} NodeHandle(const NodeHandle&, const char*){}
  template<class T> void param(const std::string&, T& v, const T& def) const { v = def; }
  template<class T> void param(const char*, T& v, const T& def) const { v = def; }
};
}
inline void _ego_log(const char* lvl, const char* fmt, ...){ va_list a; va_start(a,fmt);
  fprintf(stderr,"[ego %s] ",lvl); vfprintf(stderr,fmt,a); fprintf(stderr,"\n"); va_end(a); }
#define ROS_ERROR(...) _ego_log("E", __VA_ARGS__)
#define ROS_WARN(...)  _ego_log("W", __VA_ARGS__)
#define ROS_INFO(...)  _ego_log("I", __VA_ARGS__)
#define ROS_DEBUG(...) do{}while(0)
#define ROS_ERROR_STREAM(x) do{}while(0)
#define ROS_WARN_STREAM(x)  do{}while(0)
#define ROS_INFO_STREAM(x)  do{}while(0)
