// Seam C2-from-exec-state (A4) — honest re-anchoring of the new-plan start state.
//
// The drone physically lags the committed A by a steady-state tracking bias (finite a_max). Today the
// new MINCO solve (and the S3 certificate computed on it) is anchored at the plan-PREDICTED A, so the
// certified trajectory is one the drone never flies — the flown path sits ~bias toward the obstacle, an
// unquantified OPTIMISTIC clearance. A4 re-anchors at A_exec = A + LPF(get_state() - plan.front()), the
// drone-side DUAL of the obstacle-side dt_pred advance. set_A(A_exec) runs inside generate_global_path
// BEFORE the local solve, so get_A() reports A_exec even when the solve itself is a no-op.
//
// Two decisive properties:
//   (1) GOLDEN-SAFE: flag ON but ZERO tracking error -> A_exec == A exactly (== OFF). This is why the
//       perfect-replay golden suite is byte-identical with the flag compiled in.
//   (2) HONEST: a deliberate offset delta = get_state() - plan.front() shifts A toward the MEASURED
//       state by (1-alpha)*delta after one LPF step, and converges to delta in closed loop. OFF leaves
//       A at the plan-predicted value. The shift is toward where the drone ACTUALLY is — never away.
#include "sando_cpp/planner.hpp"
#include "sando_cpp/types.hpp"
#include <cstdio>
#include <cmath>
using namespace sando;

// SANDO seeded at st0=(0,0,1.5), aiming far, ready to replan. plan.front()==st0 after seeding.
static SANDO make(bool seam_on, double alpha) {
  Parameters par;
  par.sim_env = "rviz_only";
  par.local_solver = "minco";
  par.seam_c2_from_state = seam_on;
  par.seam_bias_alpha = alpha;
  SANDO s(par);
  RobotState st0;
  st0.pos = Eigen::Vector3d(0, 0, 1.5);
  s.update_state(st0);                                 // seeds plan=[st0] AND state=st0
  s.update_occupancy_map_ptr({});                      // open space
  RobotState gt;
  gt.pos = Eigen::Vector3d(10, 0, 1.5);                // far goal -> need_replan() fires
  s.set_terminal_goal(gt);
  s.change_drone_status(DroneStatus::TRAVELING);
  return s;
}

// inject a measured state offset from plan.front() (update_state with state_initialized=true touches
// only `state`, never `plan`), then replan so generate_global_path runs set_A(A_exec). Returns get_A().
static RobotState replan_with_offset(SANDO& s, const Eigen::Vector3d& dpos) {
  RobotState meas;
  meas.pos = Eigen::Vector3d(0, 0, 1.5) + dpos;        // plan.front()==st0=(0,0,1.5)
  s.update_state(meas);
  s.replan(0.0, 0.0);
  return s.get_A();
}

static double vmax_abs(const Eigen::Vector3d& v) { return v.cwiseAbs().maxCoeff(); }

int main() {
  int fails = 0;
  const double alpha = 0.8;
  const Eigen::Vector3d st0(0, 0, 1.5);

  // (1) GOLDEN-SAFE: zero tracking error -> ON == OFF, both == predicted A == st0.
  {
    SANDO on = make(true, alpha), off = make(false, alpha);
    RobotState a_on = replan_with_offset(on, Eigen::Vector3d::Zero());
    RobotState a_off = replan_with_offset(off, Eigen::Vector3d::Zero());
    double d_on_off = vmax_abs(a_on.pos - a_off.pos);
    double d_off_pred = vmax_abs(a_off.pos - st0);
    double bias = vmax_abs(on.get_seam_bias().pos);
    std::printf("(1) zero-error:   |A_on-A_off|=%.2e  |A_off-pred|=%.2e  seam_bias=%.2e\n",
                d_on_off, d_off_pred, bias);
    if (d_on_off > 1e-12) { std::printf("   [GOLDEN] ON != OFF at zero error\n"); ++fails; }
    if (d_off_pred > 1e-12) { std::printf("   [BASELINE] OFF moved A off the predicted state\n"); ++fails; }
    if (bias > 1e-12) { std::printf("   [BIAS] nonzero bias at zero error\n"); ++fails; }
  }

  // (2) HONEST: a 0.2 m lateral offset -> ON shifts A toward the measurement by (1-alpha)*delta after
  //     ONE LPF step; OFF stays at the predicted state.
  {
    const Eigen::Vector3d delta(0.0, 0.2, 0.0);
    SANDO on = make(true, alpha), off = make(false, alpha);
    RobotState a_on = replan_with_offset(on, delta);
    RobotState a_off = replan_with_offset(off, delta);
    Eigen::Vector3d expect_shift = (1.0 - alpha) * delta;   // (0, 0.04, 0)
    double shift_err = vmax_abs((a_on.pos - a_off.pos) - expect_shift);
    double off_moved = vmax_abs(a_off.pos - st0);
    // shift must be TOWARD the measurement (same sign as delta), never away
    bool toward = (a_on.pos(1) - a_off.pos(1)) * delta(1) >= 0.0;
    std::printf("(2) offset 0.2m:  A_on-A_off=(%.3f,%.3f,%.3f)  expect (%.3f,%.3f,%.3f)  err=%.2e\n",
                a_on.pos(0) - a_off.pos(0), a_on.pos(1) - a_off.pos(1), a_on.pos(2) - a_off.pos(2),
                expect_shift(0), expect_shift(1), expect_shift(2), shift_err);
    if (shift_err > 1e-9) { std::printf("   [HONEST] shift != (1-alpha)*delta\n"); ++fails; }
    if (off_moved > 1e-12) { std::printf("   [BASELINE] OFF moved A off the predicted state\n"); ++fails; }
    if (!toward) { std::printf("   [DIRECTION] shift is AWAY from the measured state\n"); ++fails; }
  }

  // (3) LPF FIDELITY: over several replans the implemented bias follows the EXACT spec recurrence
  //     bias_k = alpha*bias_{k-1} + (1-alpha)*(measured - plan.front()), recomputed from the ACTUAL
  //     per-tick front (robust to however the plan evolves). The measured state holds a persistent
  //     0.2 m offset, so the bias tracks the real execution state rather than the plan prediction.
  {
    const Eigen::Vector3d delta(0.0, 0.2, 0.0);
    SANDO on = make(true, alpha);
    Eigen::Vector3d bias_ref = Eigen::Vector3d::Zero();
    double max_err = 0.0;
    for (int k = 0; k < 8; ++k) {
      Eigen::Vector3d front = on.get_plan_front().pos;          // front the upcoming replan will read
      Eigen::Vector3d e = (st0 + delta) - front;               // injected tracking error this tick
      bias_ref = alpha * bias_ref + (1.0 - alpha) * e;          // spec recurrence
      replan_with_offset(on, delta);                            // implementation applies its own LPF
      max_err = std::max(max_err, vmax_abs(on.get_seam_bias().pos - bias_ref));
    }
    std::printf("(3) lpf fidelity: max|bias_impl - bias_spec| over 8 replans = %.2e\n", max_err);
    if (max_err > 1e-9) { std::printf("   [LPF] implemented filter != spec recurrence\n"); ++fails; }
  }

  std::printf("\n[seam_exec_state] fails=%d\n", fails);
  if (fails == 0) std::printf("ALL PASS\n");
  return fails == 0 ? 0 : 1;
}
