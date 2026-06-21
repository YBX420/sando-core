// de-ROS shim: real decomp_rviz_plugins is ROS viz, but it transitively pulled the decomp_util
// geometry types (LinearConstraint3D, Polyhedron) that gurobi_solver.hpp / sando use. Pull those.
// Also pull common std headers the ROS chain used to provide transitively.
#pragma once
#include <numeric>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>