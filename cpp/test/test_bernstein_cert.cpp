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

  // ---- regression: TIME-GROWING tube  rho(t) = R + v_eff*(t + delta)  (the reachability/conformal dial) ----
  {
    const double INF = std::numeric_limits<double>::infinity();
    auto segs = bcert::minco_to_segments(tr);
    const Eigen::Vector3d cstat(15, 9, 1.5);          // far static obstacle: constant-R certifies w/ healthy margin
    const double R0 = 2.0;
    auto base = bcert::certify_segments_vs_sphere(segs, cstat, Z, Z, R0);                     // v_eff=0 (default)
    auto v0   = bcert::certify_segments_vs_sphere(segs, cstat, Z, Z, R0, INF, 16, 0.0, 0.0);  // explicit v_eff=0
    ++ncase;
    bool eq = (base.certified == v0.certified) && std::abs(base.margin - v0.margin) < 1e-12;
    if (!eq) ++fails;
    std::printf("tube v_eff=0 == constant-R : base[%d %+.3e] explicit[%d %+.3e]  %s\n",
                (int)base.certified, base.margin, (int)v0.certified, v0.margin, eq ? "OK" : "FAIL");

    // growing the tube can only TIGHTEN (margin non-increasing); enough growth flips certified -> false
    auto vg1 = bcert::certify_segments_vs_sphere(segs, cstat, Z, Z, R0, INF, 16, 1.0, 0.0);
    auto vg2 = bcert::certify_segments_vs_sphere(segs, cstat, Z, Z, R0, INF, 16, 3.0, 0.0);
    ++ncase;
    bool mono = (v0.margin >= vg1.margin - 1e-12) && (vg1.margin >= vg2.margin - 1e-12);
    bool flips = base.certified && !vg2.certified;
    if (!(mono && flips)) ++fails;
    std::printf("tube grows -> tightens : m(0)=%+.3e >= m(1)=%+.3e >= m(3)=%+.3e ; cert %d->%d  %s\n",
                v0.margin, vg1.margin, vg2.margin, (int)base.certified, (int)vg2.certified,
                (mono && flips) ? "OK" : "FAIL");

    // RISKY-BUT-FAST = short trust window: enforce separation only over the next slice we actually fly before
    // re-planning, so a SHORT t_hi certifies (margin no worse) where the FULL-horizon grown tube fails.
    auto vshort = bcert::certify_segments_vs_sphere(segs, cstat, Z, Z, R0, 0.3, 16, 3.0, 0.0);
    ++ncase;
    bool risk = (vshort.margin >= vg2.margin - 1e-12) && vshort.certified;
    if (!risk) ++fails;
    std::printf("risky-fast short t_hi : m_short(0.3s)=%+.3e cert=%d  >=  m_full=%+.3e cert=%d  %s\n",
                vshort.margin, (int)vshort.certified, vg2.margin, (int)vg2.certified, risk ? "OK" : "FAIL");
  }

  // ---- CYLINDER disjunction: 2-D horizontal AROUND (n_axes=2)  +  vertical fly-OVER  ----
  {
    const double INF = std::numeric_limits<double>::infinity();
    auto segs = bcert::minco_to_segments(tr);            // tr is flat at z=1.5, passes through wp (10,2,1.5)
    // (a) THE SOUNDNESS GAP the cylinder fixes: obstacle directly UNDER the path (horizontally on it, z far
    //     below). The 3-D sphere CLEARS it (big z gap); the 2-D horizontal cert REJECTS it (drone passes
    //     through the cylinder footprint). For a tall cylinder the sphere verdict would be UNSOUND.
    const Eigen::Vector3d c_under(10, 2, -2.0);          // under wp (10,2,1.5): horiz~0, 3-D dist~3.5
    auto sph  = bcert::certify_segments_vs_sphere(segs, c_under, Z, Z, 1.0);                       // n_axes=3
    auto horz = bcert::certify_segments_vs_sphere(segs, c_under, Z, Z, 1.0, INF, 16, 0.0, 0.0, 2); // n_axes=2
    ++ncase;
    bool gap = sph.certified && !horz.certified;
    if (!gap) ++fails;
    std::printf("cylinder AROUND : sphere clears(%d) but 2-D horizontal rejects(%d) obstacle-under-path  %s\n",
                (int)sph.certified, (int)horz.certified, gap ? "OK" : "FAIL");

    // (b) horizontal cert certifies a genuinely far obstacle (z is ignored entirely)
    const Eigen::Vector3d c_side(15, 9, 99.0);
    auto horz_far = bcert::certify_segments_vs_sphere(segs, c_side, Z, Z, 2.0, INF, 16, 0.0, 0.0, 2);
    ++ncase;
    if (!horz_far.certified) ++fails;
    std::printf("cylinder AROUND : 2-D horizontal ignores z, certifies far obstacle cert=%d margin=%+.3e  %s\n",
                (int)horz_far.certified, horz_far.margin, horz_far.certified ? "OK" : "FAIL");

    // (c) fly-OVER vertical cert: flat z=1.5 path is ABOVE a low plane, BELOW a high plane; raising tightens
    auto above_lo = bcert::certify_segments_above_plane(segs, 1.0);   // 1.5 >= 1.0 -> certified
    auto above_hi = bcert::certify_segments_above_plane(segs, 2.6);   // 1.5 <  2.6 -> rejected
    ++ncase;
    bool ab = above_lo.certified && !above_hi.certified && (above_lo.margin > above_hi.margin);
    if (!ab) ++fails;
    std::printf("fly-OVER vert   : above z=1.0 cert=%d(m=%+.3e)  above z=2.6 cert=%d(m=%+.3e)  %s\n",
                (int)above_lo.certified, above_lo.margin, (int)above_hi.certified, above_hi.margin,
                ab ? "OK" : "FAIL");

    // (d) SOUNDNESS (load-bearing): a CERTIFIED vertical verdict must never contradict a dense z-min sample
    double zmin = std::numeric_limits<double>::max();
    for (int i = 0; i <= N; ++i) {
      const double t = tr.t_start + (tr.t_end - tr.t_start) * (double(i) / N);
      const double pz = tr.eval(t)(2);
      if (pz < zmin) zmin = pz;
    }
    ++ncase;
    bool snd = !above_lo.certified || (zmin >= 1.0 - 1e-6);
    if (!snd) ++fails;
    std::printf("fly-OVER sound  : certified-above(z=1.0) -> dense min p_z=%.4f >= 1.0  %s\n",
                zmin, snd ? "OK" : "FAIL");

    // (e) bez_pad widens p_z outward -> margin can only SHRINK (more conservative), never grow
    auto above_pad = bcert::certify_segments_above_plane(segs, 1.0, INF, 16, 0.0, 0.0, 1e-6);
    ++ncase;
    bool pad = above_pad.margin <= above_lo.margin + 1e-12;
    if (!pad) ++fails;
    std::printf("fly-OVER bezpad : padded margin %+.3e <= unpadded %+.3e  %s\n",
                above_pad.margin, above_lo.margin, pad ? "OK" : "FAIL");
  }

  // ---- deg-1 (straight-line) graft: low-degree segments must be ELEVATED and truly checked, NOT skipped.
  // Regression for the false-certify where an all-deg-1 trajectory skipped every segment -> worst_hi=-inf
  // -> {certified=true, margin=+inf} (a straight line through an obstacle "certified" with infinite margin).
  {
    bcert::BSeg seg;
    seg.bern = { Eigen::Vector3d(0, 0, 1.5), Eigen::Vector3d(30, 0, 1.5) };  // deg-1: two endpoints on [0,3]
    seg.t0 = 0.0; seg.dur = 3.0;
    std::vector<bcert::BSeg> line = { seg };
    // UNSAFE: obstacle sphere sits ON the line (centre 15,0,1.5) -> must NOT certify (was false-certified).
    auto thru = bcert::certify_segments_vs_sphere(line, Eigen::Vector3d(15, 0, 1.5), Z, Z, 1.0);
    ++ncase;
    bool ok_thru = !thru.certified;
    if (!ok_thru) ++fails;
    std::printf("deg1 line THROUGH obstacle : cert=%d margin=%+.3e (must be cert=0)  %s\n",
                (int)thru.certified, thru.margin, ok_thru ? "OK" : "FAIL false-certify");
    // SAFE: obstacle 9 m off the line -> must certify with positive margin (elevation kept it non-vacuous).
    auto clear = bcert::certify_segments_vs_sphere(line, Eigen::Vector3d(15, 9, 1.5), Z, Z, 1.0);
    ++ncase;
    bool ok_clear = clear.certified && clear.margin > 0.0;
    if (!ok_clear) ++fails;
    std::printf("deg1 line CLEAR of obstacle : cert=%d margin=%+.3e (must be cert=1)  %s\n",
                (int)clear.certified, clear.margin, ok_clear ? "OK" : "FAIL vacuous");
    // GUARD: no segment enforced (empty trajectory) -> nothing proven -> must NOT certify.
    std::vector<bcert::BSeg> none;
    auto empty = bcert::certify_segments_vs_sphere(none, Z, Z, Z, 1.0);
    ++ncase;
    bool ok_empty = !empty.certified;
    if (!ok_empty) ++fails;
    std::printf("empty traj (nothing enforced) : cert=%d (must be cert=0)  %s\n",
                (int)empty.certified, ok_empty ? "OK" : "FAIL");
  }

  std::printf("\n[bernstein_cert] %d cases, %d fail\n", ncase, fails);
  if (fails == 0) std::printf("ALL PASS\n");
  return fails == 0 ? 0 : 1;
}
