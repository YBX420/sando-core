// Self-contained cross-check of bernstein_cert.hpp (no golden file).
// Builds a real MINCO quintic, certifies it vs sphere obstacles with the exact
// continuous-time Bernstein deficit (adaptive de Casteljau subdivision), and
// cross-checks against a dense brute-force min-distance.
//   * SOUNDNESS (load-bearing): a CERTIFIED verdict must never contradict a sampled
//     collision (no false-certify).
//   * NON-VACUOUS: clearly-safe cases (>=0.3 m true clearance margin) must certify
//     (this is what de Casteljau subdivision buys vs a single loose hull).
//   * UNSAFE cases (R > true clearance) must NOT certify.
#include "sando_cpp/minjerk_traj.hpp"
#include "sando_cpp/bernstein_cert.hpp"
#include <cstdio>
#include <Eigen/Dense>
#include <cmath>
#include <limits>

using namespace sando;

static double brute_min(const MinjerkTraj& tr, const Eigen::Vector3d& c0,
                        const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, int N) {
  double m = std::numeric_limits<double>::max();
  for (int i = 0; i <= N; ++i) {
    const double t = tr.t_start + (tr.t_end - tr.t_start) * (double(i) / N);
    const Eigen::Vector3d p = tr.eval(t);
    const Eigen::Vector3d c = c0 + vel * t + 0.5 * acc * (t * t);
    const double d = (p - c).norm();
    if (d < m) m = d;
  }
  return m;
}

int main() {
  Eigen::MatrixXd wp(4, 3);
  wp << 0, 0, 1.5,   10, 2, 1.5,   20, -2, 1.5,   30, 0, 1.5;   // curved 3-segment path
  Eigen::VectorXd T(3); T << 3, 3, 3;
  MinjerkTraj tr(wp, T);

  const int N = 400000;
  int ncase = 0, fails = 0;

  // mode SAFE: R = bmin - margin (true clearance exceeds R by `margin`).
  // mode UNSAFE: R = bmin + over  (R exceeds true clearance => must be rejected).
  auto run = [&](const char* name, Eigen::Vector3d c0, Eigen::Vector3d vel,
                 Eigen::Vector3d acc, bool safe, double amt) {
    const double bmin = brute_min(tr, c0, vel, acc, N);
    const double R = safe ? (bmin - amt) : (bmin + amt);
    const auto v = bcert::certify_traj_vs_sphere(tr, c0, vel, acc, R);
    ++ncase;
    bool ok = true; const char* why = "";
    if (v.certified && bmin < R - 1e-6) { ok = false; why = "FALSE-CERTIFY(unsound!)"; }
    else if (safe && R >= 0.5 && bmin >= 1.0 && !v.certified) { ok = false; why = "VACUOUS(safe not certified)"; }
    else if (!safe && v.certified) { ok = false; why = "CERTIFIED-UNSAFE(unsound!)"; }
    if (!ok) ++fails;
    std::printf("%-26s bmin=%8.4f R=%8.4f cert=%d margin=%+.3e %s%s\n",
                name, bmin, R, (int)v.certified, v.margin, ok ? "OK" : "FAIL ", why);
  };

  const Eigen::Vector3d Z(0, 0, 0);
  // ---- SAFE: obstacles genuinely off the path (real clearance) ----
  run("far-y safe",        {15,  9, 1.5}, Z, Z, true, 0.5);
  run("behind-start safe", {-4,  0, 1.5}, Z, Z, true, 0.5);
  run("past-goal safe",    {34,  0, 1.5}, Z, Z, true, 0.5);
  run("above-z safe",      {15,  0, 4.5}, Z, Z, true, 0.5);   // ~vertical clearance (de Casteljau must tighten)
  run("side seg1 safe",    { 5,  5, 1.5}, Z, Z, true, 0.5);
  run("tight-but-safe",    {15,  9, 1.5}, Z, Z, true, 0.3);   // only 0.3 m margin -> needs subdivision
  // ---- UNSAFE: R exceeds true clearance -> must be rejected ----
  run("on-path unsafe",    {15,  0, 1.5}, Z, Z, false, 0.6);
  run("at-waypoint unsafe",{10,  2, 1.5}, Z, Z, false, 0.6);
  run("far-y too-big-R",   {15,  9, 1.5}, Z, Z, false, 0.6);  // R = clearance+0.6 -> unsafe
  run("above-z too-big-R", {15,  0, 4.5}, Z, Z, false, 0.6);
  // ---- movers (CV / CA): the case the shipped halfspace gate cannot soundly handle ----
  run("mover CV safe",     { 6,  9, 1.5}, {1.0, -1.0, 0}, Z, true, 0.5);
  run("mover CV unsafe",   { 6,  9, 1.5}, {1.0, -1.0, 0}, Z, false, 0.6);
  run("mover CA safe",     {-2,  7, 1.5}, {2.0, -0.6, 0}, {0.1, -0.15, 0}, true, 0.5);
  run("mover CA unsafe",   {-2,  7, 1.5}, {2.0, -0.6, 0}, {0.1, -0.15, 0}, false, 0.6);

  // ---- planner-AGNOSTIC core cross-validation: the any-degree certify_segments_vs_sphere on the SAME
  // MINCO trajectory (via minco_to_segments, degree 5) must give the SAME certified verdict as the
  // dedicated deg-5 certify_traj_vs_sphere. This validates the general core that EGO-Planner also uses.
  {
    auto segs = bcert::minco_to_segments(tr);
    struct XC { Eigen::Vector3d c0, vel, acc; double R; };
    XC xs[] = {
      {{15, 9, 1.5}, Z, Z, 5.0}, {{15, 0, 1.5}, Z, Z, 1.05}, {{15, 0, 4.5}, Z, Z, 1.8},
      {{6, 9, 1.5}, {1, -1, 0}, Z, 3.0}, {{6, 9, 1.5}, {1, -1, 0}, Z, 4.5},
      {{-2, 7, 1.5}, {2.0, -0.6, 0}, {0.1, -0.15, 0}, 4.5},
    };
    for (auto& x : xs) {
      auto v5 = bcert::certify_traj_vs_sphere(tr, x.c0, x.vel, x.acc, x.R);
      auto vg = bcert::certify_segments_vs_sphere(segs, x.c0, x.vel, x.acc, x.R);
      ++ncase;
      bool match = (v5.certified == vg.certified);
      if (!match) ++fails;
      std::printf("xval R=%5.2f  deg5[cert=%d m=%+.2e]  general[cert=%d m=%+.2e]  %s\n",
                  x.R, (int)v5.certified, v5.margin, (int)vg.certified, vg.margin, match ? "OK" : "FAIL mismatch");
    }
  }

  std::printf("\n[bernstein_cert] %d cases, %d fail\n", ncase, fails);
  if (fails == 0) std::printf("ALL PASS\n");
  return fails == 0 ? 0 : 1;
}
