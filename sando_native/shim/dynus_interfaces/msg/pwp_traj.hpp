#pragma once
#include <vector>
#include <dynus_interfaces/msg/coeff_poly3.hpp>
namespace dynus_interfaces{namespace msg{struct PWPTraj{std::vector<double> times;std::vector<CoeffPoly3> coeff_x,coeff_y,coeff_z;};}}