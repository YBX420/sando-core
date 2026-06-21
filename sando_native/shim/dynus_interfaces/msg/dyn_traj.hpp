#pragma once
#include <vector>
#include <string>
namespace dynus_interfaces{namespace msg{struct DynTraj{int id=0;bool is_agent=false;std::vector<std::string> function;std::vector<double> bbox;double time_received=0;};}}