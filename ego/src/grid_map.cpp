// Standalone GridMap: occupancy fed directly from a depth-FOV point cloud (no ROS, no probabilistic fusion).
#include "plan_env/grid_map.h"

void GridMap::update_point_cloud(const std::vector<Eigen::Vector3d>& cloud, const Eigen::Vector3d& /*cam_pos*/) {
  std::fill(buffer_inflate_.begin(), buffer_inflate_.end(), 0);   // fresh local map each update (no memory)
  const int inf = std::max(0, (int)std::ceil(inflate_ * res_inv_));
  for (const auto& p : cloud) {
    if (!isInMap(p)) continue;
    Eigen::Vector3i c; posToIndex(p, c);
    for (int dx = -inf; dx <= inf; ++dx)
      for (int dy = -inf; dy <= inf; ++dy)
        for (int dz = -inf; dz <= inf; ++dz) {
          Eigen::Vector3i id(c[0] + dx, c[1] + dy, c[2] + dz);
          if (isInMap(id)) buffer_inflate_[toAddress(id)] = 1;
        }
  }
}
