// Yaw-smoothness test for the C2 yaw governor (minco_yaw_c2_smooth).
// Drives the REAL SANDO::get_next_goal over a plan with a sharp 90-deg velocity-direction change
// and measures the executed yaw-rate stream's max single-tick |d(dyaw)| (a yaw-ACCELERATION proxy).
//   OFF: the first-order rate-limited filter steps dyaw 0->~w_max in one tick (large yaw-accel spike).
//   ON : the jerk-limited governor bounds |d(dyaw)| <= minco_yaw_accel_max*dc (smooth) and yaw stays C1.
// Asserts: ON bound respected AND ON < OFF.  (Yaw is cert-orthogonal -> safety unaffected.)
#include "sando_cpp/planner.hpp"
#include "sando_cpp/types.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>
using namespace sando;

static double run(bool smooth, double accel_max) {
  Parameters par; par.sim_env = "rviz_only"; par.local_solver = "minco";
  par.minco_yaw_c2_smooth = smooth; par.minco_yaw_accel_max = accel_max;
  SANDO s(par);
  const double dc = par.dc, sp = 2.0;

  RobotState init; init.pos = Eigen::Vector3d(0, 0, 1.5); s.update_state(init);
  RobotState goal; goal.pos = Eigen::Vector3d(40, 40, 1.5); s.set_terminal_goal(goal);
  s.update_occupancy_map_ptr(std::vector<Eigen::Vector3d>{});   // initialise the map (empty) -> ready
  s.change_drone_status(DroneStatus::TRAVELING);

  // plan: 16 setpoints, velocity +x for the first 8 then +y for the last 8 (a 90-deg heading step)
  Eigen::Vector3d p(0, 0, 1.5);
  for (int i = 0; i < 16; ++i) {
    RobotState r;
    Eigen::Vector3d v = (i < 8) ? Eigen::Vector3d(sp, 0, 0) : Eigen::Vector3d(0, sp, 0);
    p += v * dc; r.pos = p; r.vel = v; r.accel = Eigen::Vector3d::Zero(); r.t = i * dc;
    s.plan.push_back(r);
  }

  double prev_dyaw = 0.0, max_ddyaw = 0.0; bool first = true;
  for (int k = 0; k < 15; ++k) {
    auto ng = s.get_next_goal();
    if (!ng.ok) break;
    if (!first) max_ddyaw = std::max(max_ddyaw, std::fabs(ng.goal.dyaw - prev_dyaw));
    prev_dyaw = ng.goal.dyaw; first = false;
  }
  return max_ddyaw;
}

int main() {
  Parameters par;                         // read defaults for the bound
  const double dc = par.dc, accel_max = 6.0;
  double off = run(false, accel_max);
  double on  = run(true,  accel_max);
  const double bound = accel_max * dc;
  std::printf("max |d(dyaw)|/tick (yaw-accel proxy):  OFF=%.3f   ON=%.3f   (ON bound=%.3f rad/s/tick)\n",
              off, on, bound);

  int fails = 0;
  if (on > bound + 1e-6) { std::printf("  [BOUND] ON exceeds the yaw-accel limit\n"); ++fails; }
  if (!(on < off - 1e-6)) { std::printf("  [SMOOTH] ON not smoother than OFF\n"); ++fails; }
  std::printf("\n[yaw_smooth] fails=%d\n", fails);
  if (fails == 0) std::printf("ALL PASS\n");
  return fails == 0 ? 0 : 1;
}
