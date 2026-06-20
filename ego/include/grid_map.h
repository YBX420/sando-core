// Minimal STANDALONE GridMap for EGO-Planner core (NO ROS). Same class name + the query API the optimizer
// and A* use (getInflateOccupancy / getResolution / posToIndex / isInMap / setOccupancy ...). Occupancy is
// fed DIRECTLY from a point cloud (MetaUrban depth-FOV) via update_point_cloud() — no ROS subscribers, no
// probabilistic raycast fusion (we trust the fed cloud as occupied + inflate by obstacles_inflation_).
#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

struct GridMapConfig {
  Eigen::Vector3d map_origin{-50, -50, -1};
  Eigen::Vector3d map_size{100, 100, 8};
  double resolution{0.15};
  double obstacles_inflation{0.199};
};

class GridMap {
public:
  typedef std::shared_ptr<GridMap> Ptr;
  GridMap() {}

  void initMapFromConfig(const GridMapConfig& c) {
    origin_ = c.map_origin; size_ = c.map_size; res_ = c.resolution; res_inv_ = 1.0 / c.resolution;
    inflate_ = c.obstacles_inflation;
    for (int i = 0; i < 3; ++i) vox_[i] = (int)std::ceil(size_[i] / res_);
    min_b_ = origin_; max_b_ = origin_ + size_;
    buffer_inflate_.assign((size_t)vox_[0] * vox_[1] * vox_[2], 0);
  }
  void initMap() { initMapFromConfig(GridMapConfig()); }   // legacy call sites

  double getResolution() { return res_; }
  Eigen::Vector3d mapSize() { return size_; }
  Eigen::Vector3d mapOrigin() { return origin_; }

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id[i] = (int)std::floor((pos[i] - origin_[i]) * res_inv_);
  }
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) pos[i] = (id[i] + 0.5) * res_ + origin_[i];
  }
  inline int toAddress(const Eigen::Vector3i& id) { return id[0] * vox_[1] * vox_[2] + id[1] * vox_[2] + id[2]; }
  inline int toAddress(int x, int y, int z) { return x * vox_[1] * vox_[2] + y * vox_[2] + z; }
  inline bool isInMap(const Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) if (pos[i] < min_b_[i] + 1e-4 || pos[i] > max_b_[i] - 1e-4) return false;
    return true;
  }
  inline bool isInMap(const Eigen::Vector3i& idx) {
    for (int i = 0; i < 3; ++i) if (idx[i] < 0 || idx[i] >= vox_[i]) return false;
    return true;
  }
  inline void boundIndex(Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id[i] = std::max(0, std::min(id[i], vox_[i] - 1));
  }
  inline void setOccupied(const Eigen::Vector3d& pos) {
    if (!isInMap(pos)) return;
    Eigen::Vector3i id; posToIndex(pos, id);
    if (isInMap(id)) buffer_inflate_[toAddress(id)] = 1;
  }
  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1) { if (occ > 0.5) setOccupied(pos); }
  inline int getOccupancy(Eigen::Vector3d pos) { return getInflateOccupancy(pos); }
  inline int getInflateOccupancy(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return 0;
    Eigen::Vector3i id; posToIndex(pos, id);
    if (!isInMap(id)) return 0;
    return (int)buffer_inflate_[toAddress(id)];
  }
  // feed the drone's depth-FOV point cloud (world coords): clear, mark occupied + inflate.
  void update_point_cloud(const std::vector<Eigen::Vector3d>& cloud, const Eigen::Vector3d& cam_pos);

private:
  Eigen::Vector3d origin_, size_, min_b_, max_b_;
  Eigen::Vector3i vox_{0, 0, 0};
  double res_{0.15}, res_inv_{1.0 / 0.15}, inflate_{0.199};
  std::vector<char> buffer_inflate_;
};
