// bernstein_cert.hpp — exact continuous-time collision certificate on a committed
// MINCO quintic, via the Bernstein convex-hull deficit. (Core component of the S3
// "continuous-time conformal-Bernstein deficit" contribution.)
//
// IDEA (one segment, one sphere obstacle whose centre c(t) is a polynomial in t):
//   relative position w(t) = p(t) - c(t) is a vector polynomial;  S(t) = ||w(t)||^2
//   is a degree-10 polynomial.  Deficit  D(t) = R^2 - S(t).  Write D in the
//   degree-10 Bernstein basis on the segment, coeffs b_k = R^2 - S_k.  Because the
//   Bernstein basis is a partition of unity of nonnegative functions,
//        D(s) <= max_k b_k   for all s in [0,1].
//   Hence  max_k b_k <= 0  ==>  ||p(t)-c(t)|| >= R  for ALL continuous t (no K-sampling).
//   The obstacle radius R folds in the conformal tube radius: R = r_obs + r_body
//   + d_safe + q_conformal (+ eps_track).  Geometry is deterministic+exact here; the
//   1-eps coverage lives entirely in q_conformal (the S3 split).
//
// SOUNDNESS: every coefficient is carried as an outward-rounded floating-point
// interval (round-to-nearest then one nextafter outward), so a COMPUTED upper bound
// b_hi_k <= 0 rigorously implies the TRUE real coefficient b_k <= 0 — i.e.
// "CERTIFIED" is never a floating-point lie.  (Vs the shipped per-control-point
// halfspace gate, which is unsound for movers, and the K>=200 dense-sampling gate,
// which misses between samples.)
//
// This header is self-contained and touches no existing file: nothing calls it yet,
// so golden ctest is unaffected.  Wiring it in as the mover GATE is the next step.
#pragma once
#include "sando_cpp/minjerk_traj.hpp"
#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>

namespace sando {
namespace bcert {

// ---- outward-rounded floating-point interval --------------------------------
inline double rdown(double x) { return std::nextafter(x, -std::numeric_limits<double>::infinity()); }
inline double rup(double x) { return std::nextafter(x, std::numeric_limits<double>::infinity()); }

struct Iv {
  double lo, hi;
};

inline Iv iv_pt(double x) { return {x, x}; }                 // a double is exact
inline Iv iv_rat(long num, long den) {                       // exact rational n/den, outward
  double q = static_cast<double>(num) / static_cast<double>(den);
  return {rdown(q), rup(q)};
}
inline Iv iv_add(const Iv& a, const Iv& b) { return {rdown(a.lo + b.lo), rup(a.hi + b.hi)}; }
inline Iv iv_sub(const Iv& a, const Iv& b) { return {rdown(a.lo - b.hi), rup(a.hi - b.lo)}; }
inline Iv iv_mul(const Iv& a, const Iv& b) {
  const double p1 = a.lo * b.lo, p2 = a.lo * b.hi, p3 = a.hi * b.lo, p4 = a.hi * b.hi;
  return {rdown(std::min(std::min(p1, p2), std::min(p3, p4))),
          rup(std::max(std::max(p1, p2), std::max(p3, p4)))};
}

// power -> Bernstein for the quintic position map, as exact integer/60 ratios
// (mirrors C2B_matrix() in minjerk_traj.hpp, kept in integer form for soundness).
inline const std::array<std::array<long, 6>, 6>& C2B_int() {
  static const std::array<std::array<long, 6>, 6> C = {{
      {{60, 0, 0, 0, 0, 0}},
      {{60, 12, 0, 0, 0, 0}},
      {{60, 24, 6, 0, 0, 0}},
      {{60, 36, 18, 6, 0, 0}},
      {{60, 48, 36, 24, 12, 0}},
      {{60, 60, 60, 60, 60, 60}}}};
  return C;
}

// degree elevation n -> n+1 of a Bernstein coeff list (size n+1 -> n+2).
inline std::vector<Iv> elevate(const std::vector<Iv>& b) {
  const int n = static_cast<int>(b.size()) - 1;
  std::vector<Iv> out(n + 2, Iv{0.0, 0.0});
  for (int k = 0; k <= n + 1; ++k) {
    Iv term{0.0, 0.0};
    if (k >= 1) term = iv_mul(iv_rat(k, n + 1), b[k - 1]);            // (k/(n+1)) b_{k-1}
    Iv rest{0.0, 0.0};
    if (k <= n) rest = iv_mul(iv_rat((n + 1) - k, n + 1), b[k]);      // (1-k/(n+1)) b_k
    out[k] = iv_add(term, rest);
  }
  return out;
}

// {certified, margin}.  margin is the continuous-time deficit slack (units: distance^2):
// margin = -max_k b_hi_k over all segments;  certified <=> margin >= 0.
struct Verdict {
  bool certified;
  double margin;
};

// midpoint de Casteljau split of a degree-10 Bernstein coeff vector b on [0,1] into
// L (on [0,1/2]) and Rr (on [1/2,1]).  All averages are sound interval ops.
inline void subdiv10(const std::array<Iv, 11>& b, std::array<Iv, 11>& L, std::array<Iv, 11>& Rr) {
  std::array<Iv, 11> cur = b;
  L[0] = cur[0];
  Rr[10] = cur[10];
  for (int lvl = 1; lvl <= 10; ++lvl) {
    for (int k = 0; k <= 10 - lvl; ++k) cur[k] = iv_mul(iv_rat(1, 2), iv_add(cur[k], cur[k + 1]));
    L[lvl] = cur[0];
    Rr[10 - lvl] = cur[10 - lvl];
  }
}

// Left sub-curve on [0,u] of a degree-10 Bernstein coeff vector (de Casteljau at u, left edge):
// used to enforce a MOVING obstacle only up to the trusted horizon t_hi.
inline std::array<Iv, 11> left_subcurve(const std::array<Iv, 11>& b, double u) {
  std::array<Iv, 11> cur = b, L;
  const Iv U = iv_pt(u), Um = iv_sub(iv_pt(1.0), U);   // 1-u, outward-rounded
  L[0] = cur[0];
  for (int lvl = 1; lvl <= 10; ++lvl) {
    for (int k = 0; k <= 10 - lvl; ++k)
      cur[k] = iv_add(iv_mul(Um, cur[k]), iv_mul(U, cur[k + 1]));
    L[lvl] = cur[0];
  }
  return L;
}

// Worst deficit upper bound over a sub-interval, ADAPTIVELY subdividing (de Casteljau)
// until the Bernstein hull certifies the sub-interval (hull<=0) or maxdepth is hit.
// Subdivision tightens the hull to the true polynomial (2^{-2r} convergence), so a
// genuinely-safe segment certifies after a few splits; an unsafe one stays >0.
inline double seg_worst(const std::array<Iv, 11>& S, const Iv& R2, int depth, int maxdepth) {
  double hull = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < 11; ++k) {
    const double h = rup(R2.hi - S[k].lo);   // upper bound of deficit coeff R^2 - S_k
    if (h > hull) hull = h;
  }
  if (hull <= 0.0 || depth >= maxdepth) return hull;
  std::array<Iv, 11> L, Rr;
  subdiv10(S, L, Rr);
  return std::max(seg_worst(L, R2, depth + 1, maxdepth), seg_worst(Rr, R2, depth + 1, maxdepth));
}

// Whole committed trajectory vs ONE spherical obstacle whose centre is the analytic
// polynomial  c(t) = c0 + vel*t + 0.5*acc*t^2  (t in the trajectory's own time frame;
// vel=acc=0 => static).  R = total inflated radius (incl. r_body + d_safe + q_conformal).
// t_hi_in = enforce only up to this absolute trajectory time (MOVING obstacle -> t_start+tau_trust;
// static / default +inf -> whole trajectory).  maxdepth = max de Casteljau subdivision per segment.
inline Verdict certify_traj_vs_sphere(const MinjerkTraj& tr, const Eigen::Vector3d& c0,
                                      const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                                      double R, double t_hi_in = std::numeric_limits<double>::infinity(),
                                      int maxdepth = 16) {
  static const long C5[6] = {1, 5, 10, 10, 5, 1};
  static const long C10[11] = {1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1};
  const auto& C2B = C2B_int();
  const Iv R2 = iv_mul(iv_pt(R), iv_pt(R));
  const double t_hi = std::min(t_hi_in, tr.t_end);

  double worst_hi = -std::numeric_limits<double>::infinity();  // max over segs,k of b_hi_k
  bool certified = true;

  for (int i = 0; i < tr.M; ++i) {
    const double Ti = tr.T(i), u0 = tr.cum(i);
    const double seg_hi = u0 + Ti;
    if (u0 >= t_hi) continue;   // segment entirely beyond the trusted horizon -> not enforced
    // interval powers D[j] = Ti^j, j=0..5
    std::array<Iv, 6> D;
    D[0] = iv_pt(1.0);
    for (int j = 1; j < 6; ++j) D[j] = iv_mul(D[j - 1], iv_pt(Ti));

    // S_k (deg-10) accumulated over the 3 coordinates
    std::array<Iv, 11> S;
    for (int k = 0; k < 11; ++k) S[k] = Iv{0.0, 0.0};

    for (int coord = 0; coord < 3; ++coord) {
      // P: deg-5 Bernstein control points of p(coord) on this segment.
      // a_j = c[6i+j] * Ti^j ; P_k = sum_j (C2B_int[k][j]/60) * a_j.
      std::array<Iv, 6> a;
      for (int j = 0; j < 6; ++j) a[j] = iv_mul(iv_pt(tr.c(6 * i + j, coord)), D[j]);
      std::array<Iv, 6> P;
      for (int k = 0; k < 6; ++k) {
        Iv acc_iv{0.0, 0.0};
        for (int j = 0; j < 6; ++j)
          if (C2B[k][j] != 0) acc_iv = iv_add(acc_iv, iv_mul(iv_rat(C2B[k][j], 60), a[j]));
        P[k] = acc_iv;
      }

      // obstacle centre as power poly in s on this segment: t = u0 + s*Ti
      //   g0 = c0 + vel*u0 + 0.5*acc*u0^2 ; g1 = Ti*(vel + acc*u0) ; g2 = 0.5*acc*Ti^2
      const double c0c = c0(coord), vc = vel(coord), ac = acc(coord);
      Iv g0 = iv_add(iv_add(iv_pt(c0c), iv_mul(iv_pt(vc), iv_pt(u0))),
                     iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(u0), iv_pt(u0))));
      Iv g1 = iv_mul(iv_pt(Ti), iv_add(iv_pt(vc), iv_mul(iv_pt(ac), iv_pt(u0))));
      Iv g2 = iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(Ti), iv_pt(Ti)));
      // deg-2 Bernstein: o0=g0, o1=g0+g1/2, o2=g0+g1+g2
      std::vector<Iv> o = {g0, iv_add(g0, iv_mul(iv_rat(1, 2), g1)),
                           iv_add(iv_add(g0, g1), g2)};
      o = elevate(o);  // 2->3
      o = elevate(o);  // 3->4
      o = elevate(o);  // 4->5  (now size 6, matches P)

      // relative position w = P - o  (deg-5), then square (Chu-Vandermonde) -> deg-10
      std::array<Iv, 6> w;
      for (int k = 0; k < 6; ++k) w[k] = iv_sub(P[k], o[k]);
      for (int k = 0; k < 11; ++k) {
        Iv acc_iv{0.0, 0.0};
        const int imin = std::max(0, k - 5), imax = std::min(5, k);
        for (int ii = imin; ii <= imax; ++ii) {
          const int jj = k - ii;
          Iv coeff = iv_rat(C5[ii] * C5[jj], C10[k]);
          acc_iv = iv_add(acc_iv, iv_mul(coeff, iv_mul(w[ii], w[jj])));
        }
        S[k] = iv_add(S[k], acc_iv);
      }
    }

    // adaptive de Casteljau subdivision tightens the deg-10 hull to the true deficit;
    // clip the straddling segment to the trusted horizon (moving obstacles enforced only to t_hi).
    std::array<Iv, 11> Suse = S;
    if (seg_hi > t_hi) {
      double s_cut = (t_hi - u0) / Ti;
      if (s_cut > 1.0) s_cut = 1.0; else if (s_cut < 0.0) s_cut = 0.0;
      Suse = left_subcurve(S, s_cut);
    }
    const double sw = seg_worst(Suse, R2, 0, maxdepth);
    if (sw > worst_hi) worst_hi = sw;
  }
  certified = (worst_hi <= 0.0);
  return {certified, -worst_hi};
}

// ============================================================================
// PLANNER-AGNOSTIC core (any-degree piecewise Bernstein).  ANY planner feeds its
// committed trajectory as per-segment position Bernstein control points + timing;
// the SAME continuous-time conformal deficit certificate applies.  MINCO (quintic)
// and EGO-Planner (cubic uniform B-spline -> per-segment Bezier) both use this —
// the literal demonstration of the planner-agnostic certified safety layer.
// ============================================================================
struct BSeg {                       // position Bernstein control points (degree = bern.size()-1)
  std::vector<Eigen::Vector3d> bern;
  double t0;                        // wall-clock segment start (trajectory's own time frame)
  double dur;                       // segment duration
};

inline long g_binom(int n, int k) {
  if (k < 0 || k > n) return 0;
  long r = 1; for (int i = 0; i < k; ++i) r = r * (n - i) / (i + 1); return r;
}
inline std::vector<Iv> g_elevate(const std::vector<Iv>& b) {            // deg d -> d+1
  const int d = static_cast<int>(b.size()) - 1;
  std::vector<Iv> o(d + 2, Iv{0.0, 0.0});
  for (int k = 0; k <= d + 1; ++k) {
    Iv t{0.0, 0.0}, r{0.0, 0.0};
    if (k >= 1) t = iv_mul(iv_rat(k, d + 1), b[k - 1]);
    if (k <= d) r = iv_mul(iv_rat(d + 1 - k, d + 1), b[k]);
    o[k] = iv_add(t, r);
  }
  return o;
}
inline std::vector<Iv> g_square(const std::vector<Iv>& w) {             // deg n -> deg 2n (Chu-Vandermonde)
  const int n = static_cast<int>(w.size()) - 1;
  std::vector<Iv> S(2 * n + 1, Iv{0.0, 0.0});
  for (int k = 0; k <= 2 * n; ++k) {
    Iv acc{0.0, 0.0};
    for (int i = std::max(0, k - n); i <= std::min(n, k); ++i) {
      Iv coeff = iv_rat(g_binom(n, i) * g_binom(n, k - i), g_binom(2 * n, k));
      acc = iv_add(acc, iv_mul(coeff, iv_mul(w[i], w[k - i])));
    }
    S[k] = acc;
  }
  return S;
}
inline void g_subdiv(const std::vector<Iv>& b, std::vector<Iv>& L, std::vector<Iv>& R) {  // midpoint, deg m
  const int m = static_cast<int>(b.size()) - 1;
  std::vector<Iv> cur = b; L.assign(m + 1, Iv{0.0, 0.0}); R.assign(m + 1, Iv{0.0, 0.0});
  L[0] = cur[0]; R[m] = cur[m];
  for (int lvl = 1; lvl <= m; ++lvl) {
    for (int k = 0; k <= m - lvl; ++k) cur[k] = iv_mul(iv_rat(1, 2), iv_add(cur[k], cur[k + 1]));
    L[lvl] = cur[0]; R[m - lvl] = cur[m - lvl];
  }
}
inline std::vector<Iv> g_left_subcurve(const std::vector<Iv>& b, double u) {              // [0,u], deg m
  const int m = static_cast<int>(b.size()) - 1;
  std::vector<Iv> cur = b, L(m + 1, Iv{0.0, 0.0});
  const Iv U = iv_pt(u), Um = iv_sub(iv_pt(1.0), U);
  L[0] = cur[0];
  for (int lvl = 1; lvl <= m; ++lvl) {
    for (int k = 0; k <= m - lvl; ++k) cur[k] = iv_add(iv_mul(Um, cur[k]), iv_mul(U, cur[k + 1]));
    L[lvl] = cur[0];
  }
  return L;
}
inline double g_seg_worst(const std::vector<Iv>& S, const Iv& R2, int depth, int maxdepth) {
  double hull = -std::numeric_limits<double>::infinity();
  for (const auto& s : S) { const double h = rup(R2.hi - s.lo); if (h > hull) hull = h; }
  if (hull <= 0.0 || depth >= maxdepth) return hull;
  std::vector<Iv> L, R; g_subdiv(S, L, R);
  return std::max(g_seg_worst(L, R2, depth + 1, maxdepth), g_seg_worst(R, R2, depth + 1, maxdepth));
}

// Whole committed PIECEWISE-BERNSTEIN trajectory (any degree per segment) vs ONE sphere obstacle whose
// centre is c(t)=c0+vel*t+0.5*acc*t^2.  R = total inflated radius (incl. r_body+d_safe+q_conformal).
// CERTIFIED => ||p(t)-c(t)|| >= R for ALL continuous t in [0, t_hi].
inline Verdict certify_segments_vs_sphere(const std::vector<BSeg>& segs, const Eigen::Vector3d& c0,
                                          const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                                          double R, double t_hi_in = std::numeric_limits<double>::infinity(),
                                          int maxdepth = 16) {
  const Iv R2 = iv_mul(iv_pt(R), iv_pt(R));
  double t_end = 0.0;
  for (const auto& sg : segs) t_end = std::max(t_end, sg.t0 + sg.dur);
  const double t_hi = std::min(t_hi_in, t_end);
  double worst_hi = -std::numeric_limits<double>::infinity();
  for (const auto& sg : segs) {
    const int n = static_cast<int>(sg.bern.size()) - 1;
    if (n < 2) continue;                                  // need degree >= 2 for the deg-2 obstacle poly
    const double t0 = sg.t0, dur = sg.dur, seg_hi = t0 + dur;
    if (t0 >= t_hi || dur <= 0.0) continue;
    std::vector<Iv> S(2 * n + 1, Iv{0.0, 0.0});
    for (int coord = 0; coord < 3; ++coord) {
      const double c0c = c0(coord), vc = vel(coord), ac = acc(coord);
      Iv g0 = iv_add(iv_add(iv_pt(c0c), iv_mul(iv_pt(vc), iv_pt(t0))),
                     iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(t0), iv_pt(t0))));
      Iv g1 = iv_mul(iv_pt(dur), iv_add(iv_pt(vc), iv_mul(iv_pt(ac), iv_pt(t0))));
      Iv g2 = iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(dur), iv_pt(dur)));
      std::vector<Iv> o = {g0, iv_add(g0, iv_mul(iv_rat(1, 2), g1)), iv_add(iv_add(g0, g1), g2)};  // deg 2
      while (static_cast<int>(o.size()) - 1 < n) o = g_elevate(o);     // elevate obstacle poly to deg n
      std::vector<Iv> w(n + 1);
      for (int k = 0; k <= n; ++k) w[k] = iv_sub(iv_pt(sg.bern[k](coord)), o[k]);
      std::vector<Iv> Sc = g_square(w);                                // deg 2n
      for (int k = 0; k <= 2 * n; ++k) S[k] = iv_add(S[k], Sc[k]);
    }
    std::vector<Iv> Suse = S;
    if (seg_hi > t_hi) {
      double s_cut = (t_hi - t0) / dur;
      if (s_cut > 1.0) s_cut = 1.0; else if (s_cut < 0.0) s_cut = 0.0;
      Suse = g_left_subcurve(S, s_cut);
    }
    const double sw = g_seg_worst(Suse, R2, 0, maxdepth);
    if (sw > worst_hi) worst_hi = sw;
  }
  return {worst_hi <= 0.0, -worst_hi};
}

// Convenience: build BSeg list from a MinjerkTraj (degree-5) for the planner-agnostic core.
inline std::vector<BSeg> minco_to_segments(const MinjerkTraj& tr) {
  auto cps = tr.control_points();          // M blocks of 6x3 (degree-5 Bernstein)
  std::vector<BSeg> segs(tr.M);
  for (int i = 0; i < tr.M; ++i) {
    BSeg s; s.t0 = tr.cum(i); s.dur = tr.T(i); s.bern.resize(6);
    for (int k = 0; k < 6; ++k) s.bern[k] = cps[i].row(k).transpose();
    segs[i] = s;
  }
  return segs;
}

}  // namespace bcert
}  // namespace sando
