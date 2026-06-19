// Closed-loop recovery test for the smooth recovery brake (minco_recovery_smooth_brake).
// Drives the REAL SANDO::recovery_yield() in two boxed scenarios that force the BRAKE path,
// executing the front setpoint each tick (closed loop), and recording the EXECUTED velocity stream
// (max single-tick |dv| = a jerk proxy) + min clearance.
//   S1 head-on (threat AHEAD): smoothing a decel would COAST INTO the threat -> the clearance guard
//      must make smooth-brake FALL BACK to the instant stop (safety). Expect ON == OFF, both safe.
//   S2 lateral (threat to the SIDE, forward open): a smooth forward decel is safe -> smooth-brake
//      ACTIVATES. Expect ON jerk << OFF jerk (OFF = instant vel 2.5->0 spike), both safe.
// This validates BOTH the safety fallback and the smoothing-when-applicable, and never a regression.
#include "sando_cpp/planner.hpp"
#include "sando_cpp/types.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>
using namespace sando;

struct Result { bool fired; double max_dv; double min_clr; };

static Result run(bool smooth, bool head_on) {
  Parameters par; par.sim_env = "rviz_only"; par.local_solver = "minco";
  par.recovery_enabled = true; par.minco_recovery_smooth_brake = smooth;
  SANDO s(par);
  const double dc = par.dc, HR = 0.3;

  RobotState A; A.pos = Eigen::Vector3d(0, 0, 1.5);
  A.vel = Eigen::Vector3d(2.5, 0, 0); A.accel = Eigen::Vector3d::Zero();
  RobotState G; G.pos = Eigen::Vector3d(10, 0, 1.5); s.set_G(G);

  // human: AHEAD (head-on) or to the SIDE (lateral, forward stays open)
  Eigen::Vector3d hpos = head_on ? Eigen::Vector3d(1.1, 0, 1.5) : Eigen::Vector3d(0.0, 0.95, 1.5);
  const Eigen::Vector3d hvel = head_on ? Eigen::Vector3d(-1.0, 0, 0) : Eigen::Vector3d(0, -1.0, 0);

  Result R{false, 0.0, 1e18};
  double prev_v = A.vel.norm(), t = 0.0;
  for (int tick = 0; tick < 14; ++tick) {
    s.set_A(A); s.set_A_time(t);
    const Eigen::Vector3d c = A.pos;
    // box LEFT/RIGHT/BEHIND so yield_target cannot improve -> BRAKE. (No wall AHEAD: in S2 forward is open.)
    s.obst_pos   = { hpos, c + Eigen::Vector3d(0, 0.75, 0), c + Eigen::Vector3d(0, -0.75, 0), c + Eigen::Vector3d(-0.75, 0, 0) };
    s.obst_bbox  = { Eigen::Vector3d(0.6,0.6,0.6), Eigen::Vector3d(3,0.3,3), Eigen::Vector3d(3,0.3,3), Eigen::Vector3d(0.3,3,3) };
    s.obst_vel   = { hvel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
    s.obst_accel = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };
    s.obst_class = { "human", "wall", "wall", "wall" };
    s.obst_snapshot_time_ = t;

    bool ok = false;
    try { ok = s.recovery_yield(); } catch (const std::exception& e) { std::printf("  THREW: %s\n", e.what()); break; }
    if (!ok || s.goal_setpoints.empty()) { A.pos += A.vel * dc; }
    else {
      R.fired = true;
      const RobotState& sp = s.goal_setpoints.front();
      double v = sp.vel.norm();
      R.max_dv = std::max(R.max_dv, std::fabs(v - prev_v));
      prev_v = v;
      A.pos = sp.pos; A.vel = sp.vel; A.accel = sp.accel;
    }
    R.min_clr = std::min(R.min_clr, (A.pos - hpos).norm() - HR);
    hpos += hvel * dc; t += dc;
  }
  return R;
}

int main() {
  int fails = 0;
  struct { const char* name; bool head_on; bool expect_smooth; } S[] = {
    {"S1 head-on (must fall back, safe)", true,  false},
    {"S2 lateral (must smooth)",          false, true},
  };
  for (auto& sc : S) {
    Result off = run(false, sc.head_on);
    Result on  = run(true,  sc.head_on);
    std::printf("%-34s OFF[fired=%d dv=%.3f clr=%.3f]  ON[fired=%d dv=%.3f clr=%.3f]\n",
                sc.name, off.fired, off.max_dv, off.min_clr, on.fired, on.max_dv, on.min_clr);
    if (!off.fired || !on.fired) { std::printf("   [SCENARIO] brake did not fire\n"); ++fails; continue; }
    if (off.min_clr < -1e-6 || on.min_clr < -1e-6) { std::printf("   [SAFETY] collision\n"); ++fails; }
    if (on.max_dv > off.max_dv + 1e-6) { std::printf("   [REGRESSION] ON jerk > OFF jerk\n"); ++fails; }
    if (sc.expect_smooth && !(on.max_dv < off.max_dv - 0.3)) { std::printf("   [SMOOTH] ON did not smooth vs OFF\n"); ++fails; }
  }
  std::printf("\n[recovery_closedloop] fails=%d\n", fails);
  if (fails == 0) std::printf("ALL PASS\n");
  return fails == 0 ? 0 : 1;
}
