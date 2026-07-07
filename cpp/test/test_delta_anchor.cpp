// Numerical witness for bug① (τ anchor = t_obs + δ) in bernstein_cert.hpp.
// A δ-honoring implementation (ρ(t_traj)=R+v_eff*(t_traj+δ), t_traj global) must produce
// EXACTLY the verdicts asserted below; an implementation that anchors τ at trajectory
// start WITHOUT δ, or re-anchors the tube per-segment (local s instead of global t),
// flips specific verdicts. Drone hovers at origin -> S(t)=d^2 constant, so the deficit
// max is analytic: max_t b = (R+v_eff*(t*+δ))^2 - d^2 at t* = window end.
#include "sando_cpp/bernstein_cert.hpp"
#include <cstdio>
#include <cmath>

using namespace sando;
using namespace sando::bcert;

static std::vector<BSeg> hover(int nseg, double dur_each) {
  std::vector<BSeg> segs(nseg);
  for (int i = 0; i < nseg; ++i) {
    segs[i].t0 = i * dur_each;
    segs[i].dur = dur_each;
    segs[i].bern = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}; // deg-2 hover
  }
  return segs;
}

static int fails = 0;
static void check(const char* name, bool got_cert, double got_margin,
                  bool want_cert, double want_margin, double tol) {
  const bool ok = (got_cert == want_cert) && std::fabs(got_margin - want_margin) <= tol;
  printf("%-34s certified=%d margin=%+.9f   (expect certified=%d margin=%+.6f)  %s\n",
         name, (int)got_cert, got_margin, (int)want_cert, want_margin, ok ? "PASS" : "FAIL");
  if (!ok) ++fails;
}

int main() {
  const Eigen::Vector3d z = Eigen::Vector3d::Zero();

  // ---- W1: δ inflation must be honored (single segment, static obstacle) ----
  // R=1, v_eff=1, δ=0.5, t∈[0,1]. Correct tube end radius ρ(1)=1+1*(1+0.5)=2.5.
  // δ-dropping tube end radius = 2.0.
  {
    auto segs = hover(1, 1.0);
    // W1a: d=2.2 sits BETWEEN 2.0 and 2.5 -> correct: REFUSE, worst b=(2.5^2-2.2^2)=+1.41 (margin=-1.41);
    //      δ-dropping bug: max ρ=2.0<2.2 -> would CERTIFY with margin 2.2^2-2.0^2=+0.84.
    Eigen::Vector3d c0(2.2, 0, 0);
    auto v = certify_segments_vs_sphere(segs, c0, z, z, 1.0,
                                        std::numeric_limits<double>::infinity(), 16,
                                        /*v_eff=*/1.0, /*delta=*/0.5);
    check("W1a d=2.2 (delta must refute)", v.certified, v.margin, false, -1.41, 1e-6);
    // W1b: d=2.6 > 2.5 -> correct: CERTIFY, margin = 2.6^2-2.5^2 = 0.51 (non-vacuity of the δ tube).
    Eigen::Vector3d c1(2.6, 0, 0);
    auto v2 = certify_segments_vs_sphere(segs, c1, z, z, 1.0,
                                         std::numeric_limits<double>::infinity(), 16, 1.0, 0.5);
    check("W1b d=2.6 (delta cert non-vacuous)", v2.certified, v2.margin, true, 0.51, 1e-6);
    // W1c: same d=2.2 with delta=0 -> must CERTIFY margin 2.2^2-2.0^2=0.84
    //      (isolates that W1a's refusal comes from δ alone).
    auto v3 = certify_segments_vs_sphere(segs, c0, z, z, 1.0,
                                         std::numeric_limits<double>::infinity(), 16, 1.0, 0.0);
    check("W1c d=2.2 delta=0 control", v3.certified, v3.margin, true, 0.84, 1e-6);
  }

  // ---- W2: tube anchored in GLOBAL trajectory time, not per-segment local time ----
  // Two segments dur=1 each (t0=0,1), R=1, v_eff=1, δ=0. Global anchor: ρ(2)=3.
  // Per-segment-local re-anchor bug: each segment restarts the tube, max ρ=2.
  {
    auto segs = hover(2, 1.0);
    Eigen::Vector3d c0(2.5, 0, 0); // between 2 and 3
    auto v = certify_segments_vs_sphere(segs, c0, z, z, 1.0,
                                        std::numeric_limits<double>::infinity(), 16, 1.0, 0.0);
    // correct: REFUSE, worst b = 3^2-2.5^2 = +2.75 (margin=-2.75); local-anchor bug: CERTIFY.
    check("W2  d=2.5 global-time anchor", v.certified, v.margin, false, -2.75, 1e-6);
  }

  // ---- W3: δ survives the t_hi left-subcurve clipping (clip acts on assembled b) ----
  // R=1, v_eff=1, δ=0.5, t_hi=0.4 inside a dur=1 segment. Correct window-end radius
  // ρ(0.4)=1+1*(0.4+0.5)=1.9; δ-dropping: 1.4. d=1.6 in between.
  {
    auto segs = hover(1, 1.0);
    Eigen::Vector3d c0(1.6, 0, 0);
    auto v = certify_segments_vs_sphere(segs, c0, z, z, 1.0, /*t_hi=*/0.4, 16, 1.0, 0.5);
    // correct: REFUSE, worst b = 1.9^2-1.6^2 = +1.05 (margin=-1.05); δ-dropping: CERTIFY (1.6>1.4).
    check("W3  d=1.6 t_hi=0.4 clip keeps d", v.certified, v.margin, false, -1.05, 1e-6);
    // W3b control: same but d=2.0 > 1.9 -> CERTIFY margin = 2^2-1.9^2 = 0.39.
    Eigen::Vector3d c1(2.0, 0, 0);
    auto v2 = certify_segments_vs_sphere(segs, c1, z, z, 1.0, 0.4, 16, 1.0, 0.5);
    check("W3b d=2.0 t_hi=0.4 cert", v2.certified, v2.margin, true, 0.39, 1e-6);
  }

  // ---- W4: above-plane floor also carries δ: z_clear(t)=z0+v_eff_z*(t+δ) ----
  // Hover at z=0? use hover at z: make a level flight at z=1.6, floor z0=1, v_eff_z=1, δ=0.5, t_hi=0.4.
  // Correct floor at window end: 1+1*(0.4+0.5)=1.9 > 1.6 -> REFUSE (worst b = 1.9-1.6 = +0.3).
  // δ-dropping floor: 1.4 < 1.6 -> would CERTIFY.
  {
    std::vector<BSeg> segs(1);
    segs[0].t0 = 0.0; segs[0].dur = 1.0;
    segs[0].bern = {Eigen::Vector3d(0,0,1.6), Eigen::Vector3d(0,0,1.6), Eigen::Vector3d(0,0,1.6)};
    auto v = certify_segments_above_plane(segs, 1.0, /*t_hi=*/0.4, 16, /*v_eff_z=*/1.0, /*delta=*/0.5);
    check("W4  plane z=1.6 vs growing floor", v.certified, v.margin, false, -0.3, 1e-6);
  }

  // ---- W5: brute-force cross-check of W1a's analytic deficit ----
  {
    double worst = -1e300;
    const double d = 2.2, R = 1.0, ve = 1.0, del = 0.5;
    for (int i = 0; i <= 400000; ++i) {
      const double t = i / 400000.0;                 // one segment, [0,1]
      const double rho = R + ve * (t + del);
      const double b = rho * rho - d * d;            // S(t)=d^2 (hover)
      if (b > worst) worst = b;
    }
    printf("W5  brute-force max deficit = %+.9f (analytic +1.41)  %s\n",
           worst, std::fabs(worst - 1.41) < 1e-9 ? "PASS" : "FAIL");
    if (std::fabs(worst - 1.41) >= 1e-9) ++fails;
  }

  printf(fails == 0 ? "\nALL WITNESS CHECKS PASS\n" : "\n%d WITNESS CHECKS FAILED\n", fails);
  return fails == 0 ? 0 : 1;
}
