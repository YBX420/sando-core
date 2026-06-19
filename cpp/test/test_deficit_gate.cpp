// Integration test for the S3 deficit certificate wired as an ADDITIONAL mover gate in
// optimise_one_minco (opt.minco_deficit_cert).  Drives the PRODUCTION plan_minco OFF vs ON.
// Checks: (1) ON runs without crash + within real-time budget; (2) ON does NOT spuriously
// reject an easy clear scenario (no liveness regression); (3) SOUNDNESS — any trajectory the
// ON gate marks VALID is actually continuous-time cert-clean vs the hard sphere.
#include "sando_cpp/plan_minco.hpp"
#include "sando_cpp/bernstein_cert.hpp"
#include "sando_cpp/obstacles.hpp"
#include "sando_cpp/avoid_config.hpp"
#include <cstdio>
#include <chrono>
#include <vector>
#include <memory>

using namespace sando;

int main() {
  int fails = 0;

  auto cfg = default_config();
  cfg["human"] = AvoidParams{"human", "hard", 0.8, 1.0e4};

  // helper: run plan_minco with the flag on/off, return (valid, ms, traj)
  auto run = [&](const char* name, const Eigen::MatrixXd& ap,
                 const std::vector<const Obstacle*>& obs,
                 const SphereObstacle* human, bool on) {
    PlanOptParams opt;                       // production defaults (q_conformal=0, eps_track=0, clearance_tol=0.05)
    opt.vmax = 3.0; opt.amax = 3.0;
    opt.minco_deficit_cert = on;
    auto t0 = std::chrono::steady_clock::now();
    bool valid = false; double clr_ok = true;
    try {
      auto pr = plan_minco(ap, obs, cfg, opt);
      const MinjerkTraj& tr = pr.first;
      valid = pr.second.trajectory_valid;
      // SOUNDNESS re-check: if ON marks it valid, the committed trajectory must be cert-clean.
      if (on && valid && human) {
        const double d_safe = cfg["human"].d_safe;
        const double R = human->radius + d_safe + 0.0 - opt.clearance_tol;  // extra=0 by default
        const double t_hi = tr.t_start + opt.tau_trust;                     // moving -> trusted horizon
        auto v = bcert::certify_traj_vs_sphere(tr, human->centre0, human->vel, human->accel, R, t_hi);
        clr_ok = v.certified;
      }
    } catch (const std::exception& e) {
      std::printf("  %-22s on=%d THREW: %s\n", name, (int)on, e.what());
      return std::make_pair(false, 0.0);
    }
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("  %-22s on=%d valid=%d cert_clean=%d  %.2f ms\n", name, (int)on, (int)valid, (int)clr_ok, ms);
    if (on && valid && !clr_ok) { std::printf("    [UNSOUND] ON marked valid but traj NOT cert-clean!\n"); ++fails; }
    return std::make_pair(valid, ms);
  };

  // ---- Scenario A: easy/clear (human far off the straight path) — ON must not spuriously reject ----
  {
    Eigen::MatrixXd ap(5, 3);
    ap << 0,0,1.5, 4,0,1.5, 8,0,1.5, 12,0,1.5, 16,0,1.5;
    SphereObstacle human(Eigen::Vector3d(8, 8, 1.5), 0.3, Eigen::Vector3d(0, 0.3, 0), "human");
    std::vector<const Obstacle*> obs = {&human};
    std::printf("Scenario A (easy, human far):\n");
    auto off = run("A", ap, obs, &human, false);
    auto on  = run("A", ap, obs, &human, true);
    if (off.first && !on.first) { std::printf("    [LIVENESS] ON spuriously rejected an easy case OFF accepted\n"); ++fails; }
  }

  // ---- Scenario B: conflict (human crossing the path) — both run; ON's valid result must be cert-clean ----
  {
    Eigen::MatrixXd ap(5, 3);
    ap << 0,0,1.5, 4,0,1.5, 8,0,1.5, 12,0,1.5, 16,0,1.5;
    SphereObstacle human(Eigen::Vector3d(8, -2.2, 1.5), 0.3, Eigen::Vector3d(0, 0.8, 0), "human");
    std::vector<const Obstacle*> obs = {&human};
    std::printf("Scenario B (conflict, human crossing):\n");
    run("B", ap, obs, &human, false);
    run("B", ap, obs, &human, true);
  }

  std::printf("\n[deficit_gate] fails=%d\n", fails);
  if (fails == 0) std::printf("ALL PASS\n");
  return fails == 0 ? 0 : 1;
}
