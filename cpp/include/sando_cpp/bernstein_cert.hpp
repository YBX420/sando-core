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
#include <cstdio>
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
  if (std::isfinite(t_hi_in) && t_hi_in > tr.t_end + 1e-6) {
    // WINDOW-COVERAGE contract, MINCO entry too (07-21 ruling): a finite requested window the
    // trajectory does not reach must FAIL-CLOSED -- the silent clip certified time never proved.
    // plan_minco already clips its request explicitly (min(tr.t_end, ...)), so this never fires
    // there; any other caller now gets the loud refusal instead of a partial 'certified'.
    std::fprintf(stderr, "[bcert] certify_traj_vs_sphere: window %.3f beyond trajectory end %.3f "
                 "-> FAIL-CLOSED\n", t_hi_in, tr.t_end);
    return Verdict{false, -std::numeric_limits<double>::infinity()};
  }
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
  if (worst_hi == -std::numeric_limits<double>::infinity())
    return {false, 0.0};   // no segment enforced (tr.M==0 or all beyond t_hi) -> nothing proven -> NOT certified
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
// Worst deficit upper bound when the deficit b = rho^2 - S is ALREADY assembled (deg-2n Bernstein).
// Needed once the tube radius rho(t) grows with time (deg-2 rho^2) so R^2 is no longer a constant:
// the deficit must be formed BEFORE de Casteljau subdivision (subdividing a fixed R^2 against S is only
// valid for a constant tube).  hull = max_k b_hi_k; subdivide b (sound convex combos) and recurse.
inline double g_seg_worst_deficit(const std::vector<Iv>& b, int depth, int maxdepth, bool* refuted = nullptr) {
  // hull = max(coeff.hi) is an UPPER bound on the segment's sup deficit; lo_min = min(coeff.lo) is a guaranteed
  // LOWER bound on every value (Bernstein convex-hull property: value(t) in [min coeff, max coeff]).
  double hull = -std::numeric_limits<double>::infinity();
  double lo_min = std::numeric_limits<double>::infinity();
  for (const auto& s : b) { if (s.hi > hull) hull = s.hi; if (s.lo < lo_min) lo_min = s.lo; }
  // early-exit, SOUND both ways: hull<=0 => sup<=0 (this segment is safe); lo_min>0 => the deficit is provably
  // POSITIVE everywhere => sup>0 (NOT certifiable, e.g. a ground candidate vs a fly-OVER plane) -> stop now
  // instead of subdividing to maxdepth (this is what made certify_above ~2000x slower than certify_horizontal).
  // lo_min>0 on this (sub-)interval is also the REFUTATION WITNESS for the three-valued verdict: the deficit is
  // provably positive on a whole sub-interval => the proof obligation is genuinely violated ("truly blocked"),
  // vs. hull>0 with no witness = envelope too loose / budget exhausted (UNKNOWN). Optional out-param only;
  // refuted=nullptr keeps every existing caller byte-identical.
  if (lo_min > 0.0) { if (refuted) *refuted = true; return hull; }
  if (hull <= 0.0 || depth >= maxdepth) return hull;
  std::vector<Iv> L, R; g_subdiv(b, L, R);
  return std::max(g_seg_worst_deficit(L, depth + 1, maxdepth, refuted),
                  g_seg_worst_deficit(R, depth + 1, maxdepth, refuted));
}

// WINDOW-COVERAGE contract (07-21 ruling): a FINITE requested window must be covered by a
// contiguous tiling that starts at t<=0+eps and reaches t_hi_in. The old silent clip
// t_hi = min(t_hi_in, t_end) let a short trajectory (or a gapped tiling) return 'certified'
// for time it never proved -- an E1-theorem poison: events that belong to U/D could book as
// certified. Callers holding a SOUND completion for the remainder (the hover-tail law) must
// CLIP THEIR REQUEST EXPLICITLY; this core never shrinks a window on its own again.
inline bool g_window_covered(const std::vector<BSeg>& segs, double t_hi_in, const char* who) {
  if (!std::isfinite(t_hi_in)) return true;      // whole-trajectory request: no window contract
  std::vector<std::pair<double, double>> iv;
  for (const auto& sg : segs) if (sg.dur > 0.0) iv.emplace_back(sg.t0, sg.t0 + sg.dur);
  std::sort(iv.begin(), iv.end());
  double cover = 0.0;
  bool ok = !iv.empty() && iv.front().first <= 1e-6;
  for (const auto& p : iv) {
    if (!ok) break;
    if (p.first > cover + 1e-6) { ok = false; break; }   // gap in the tiling
    cover = std::max(cover, p.second);
  }
  if (!ok || cover + 1e-6 < t_hi_in) {
    std::fprintf(stderr, "[bcert] %s: requested window %.3f NOT contiguously covered "
                 "(tiled to %.3f) -> FAIL-CLOSED\n", who, t_hi_in, cover);
    return false;
  }
  return true;
}

// FAIL-CLOSED input guard for the generic piecewise-Bernstein entries (2026-07-20 audit finding #7).
// Two silent-lie modes it closes, LOUDLY (never a quiet fallback):
//   - a NaN/Inf coefficient poisons every downstream comparison (NaN compares false, so the deficit
//     hull stays at -inf and the trajectory FALSE-CERTIFIES);
//   - segment degree > 30 overflows the 64-bit long binomials in g_square (binom(2n,k) at 2n>=62),
//     producing garbage coefficients with no error. Internal EGO cubic / MINCO quintic are far below.
inline bool g_segs_guard(const std::vector<BSeg>& segs, const char* who) {
  for (const auto& sg : segs) {
    const int n = static_cast<int>(sg.bern.size()) - 1;
    if (n > 30) {
      std::fprintf(stderr, "[bcert] %s: segment degree %d > 30 (long-binomial overflow) -> FAIL-CLOSED\n", who, n);
      return false;
    }
    if (!std::isfinite(sg.t0) || !std::isfinite(sg.dur)) {
      std::fprintf(stderr, "[bcert] %s: non-finite segment timing -> FAIL-CLOSED\n", who);
      return false;
    }
    for (const auto& p : sg.bern)
      if (!p.allFinite()) {
        std::fprintf(stderr, "[bcert] %s: non-finite control point -> FAIL-CLOSED\n", who);
        return false;
      }
  }
  return true;
}

// Whole committed PIECEWISE-BERNSTEIN trajectory (any degree per segment) vs ONE sphere obstacle whose
// centre is c(t)=c0+vel*t+0.5*acc*t^2.  R = total inflated radius (incl. r_body+d_safe+q_conformal).
// CERTIFIED => ||p(t)-c(t)|| >= R for ALL continuous t in [0, t_hi].
// v_eff/delta upgrade the constant radius R into a TIME-GROWING tube rho(t) = R + v_eff*(t + delta):
//   - v_eff = the reachability/conformal growth rate (max-speed hard floor, or the conformal residual
//     quantile). delta = perception->commit latency (the tube must already be inflated by v_eff*delta at
//     trajectory t=0, since the obstacle was observed delta seconds before this committed trajectory starts).
//   - v_eff=0 (default) => rho=R constant => byte-identical to the old constant-R certificate.
// The deficit b = rho^2 - S is now a genuine deg-2n Bernstein polynomial (rho^2 is deg-2 in t, elevated to
// 2n) assembled BEFORE de Casteljau subdivision — subdividing a fixed R^2 against S would be unsound here.
// n_axes selects the obstacle GEOMETRY: 3 (default) sums x,y,z -> a SPHERE ||p-c||>=rho (byte-identical to
// the old cert); 2 sums only x,y -> a VERTICAL-CYLINDER horizontal-separation proof sqrt(dx^2+dy^2)>=rho
// (the AROUND half of the cylinder disjunction; z is ignored, so it is SOUND for a full-height cylinder and
// must NOT be replaced by the 3-D sphere, which would falsely clear a drone hovering low and horizontally
// inside the cylinder by counting the z gap to the centre).
inline Verdict certify_segments_vs_sphere(const std::vector<BSeg>& segs, const Eigen::Vector3d& c0,
                                          const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                                          double R, double t_hi_in = std::numeric_limits<double>::infinity(),
                                          int maxdepth = 16, double v_eff = 0.0, double delta = 0.0,
                                          int n_axes = 3, bool* refuted = nullptr) {
  if (!g_segs_guard(segs, "certify_segments_vs_sphere")
      || !g_window_covered(segs, t_hi_in, "certify_segments_vs_sphere"))
    return Verdict{false, -std::numeric_limits<double>::infinity()};
  if (!c0.allFinite() || !vel.allFinite() || !acc.allFinite()
      || !std::isfinite(R) || !std::isfinite(v_eff) || !std::isfinite(delta)) {
    std::fprintf(stderr, "[bcert] certify_segments_vs_sphere: non-finite obstacle/radius -> FAIL-CLOSED\n");
    return Verdict{false, -std::numeric_limits<double>::infinity()};
  }
  // rho^2(t) = A t^2 + Bp t + Cp, all carried as outward-rounded intervals for soundness.
  const Iv r0_iv = iv_pt(R), v_iv = iv_pt(v_eff), d_iv = iv_pt(delta), two = iv_pt(2.0);
  const Iv A_iv  = iv_mul(v_iv, v_iv);                                   // v_eff^2
  const Iv B_iv  = iv_mul(two, iv_mul(r0_iv, v_iv));                     // 2 R v_eff
  const Iv Bp_iv = iv_add(B_iv, iv_mul(two, iv_mul(A_iv, d_iv)));        // 2 R v_eff + 2 v_eff^2 delta
  const Iv Cp_iv = iv_add(iv_add(iv_mul(r0_iv, r0_iv), iv_mul(B_iv, d_iv)),
                          iv_mul(A_iv, iv_mul(d_iv, d_iv)));             // (R + v_eff*delta)^2
  double t_end = 0.0;
  for (const auto& sg : segs) t_end = std::max(t_end, sg.t0 + sg.dur);
  const double t_hi = std::min(t_hi_in, t_end);
  double worst_hi = -std::numeric_limits<double>::infinity();
  for (const auto& sg : segs) {
    const int n = static_cast<int>(sg.bern.size()) - 1;
    if (n < 0) continue;                                  // empty segment: no control points
    const double t0 = sg.t0, dur = sg.dur, seg_hi = t0 + dur;
    if (t0 >= t_hi || dur <= 0.0) continue;
    const int m = n < 2 ? 2 : n;                          // working degree >= 2 for the deg-2 obstacle poly.
                                                          // Low-degree segments (a straight-line deg-1 graft)
                                                          // are ELEVATED soundly via g_elevate, not skipped:
                                                          // skipping left worst_hi=-inf -> false certify+inf.
    std::vector<Iv> S(2 * m + 1, Iv{0.0, 0.0});
    for (int coord = 0; coord < n_axes; ++coord) {
      const double c0c = c0(coord), vc = vel(coord), ac = acc(coord);
      Iv g0 = iv_add(iv_add(iv_pt(c0c), iv_mul(iv_pt(vc), iv_pt(t0))),
                     iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(t0), iv_pt(t0))));
      Iv g1 = iv_mul(iv_pt(dur), iv_add(iv_pt(vc), iv_mul(iv_pt(ac), iv_pt(t0))));
      Iv g2 = iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(dur), iv_pt(dur)));
      std::vector<Iv> o = {g0, iv_add(g0, iv_mul(iv_rat(1, 2), g1)), iv_add(iv_add(g0, g1), g2)};  // deg 2
      while (static_cast<int>(o.size()) - 1 < m) o = g_elevate(o);     // elevate obstacle poly to deg m
      std::vector<Iv> w(n + 1);                                        // position control points as intervals
      for (int k = 0; k <= n; ++k) w[k] = iv_pt(sg.bern[k](coord));
      while (static_cast<int>(w.size()) - 1 < m) w = g_elevate(w);     // elevate position to deg m (sound)
      for (int k = 0; k <= m; ++k) w[k] = iv_sub(w[k], o[k]);          // relative position P - o (deg m)
      std::vector<Iv> Sc = g_square(w);                                // deg 2m
      for (int k = 0; k <= 2 * m; ++k) S[k] = iv_add(S[k], Sc[k]);
    }
    // tube rho^2 as a deg-2 Bernstein poly on this segment (t = t0 + s*dur), elevated to deg 2m
    const Iv t0_iv = iv_pt(t0), dur_iv = iv_pt(dur);
    const Iv q0 = iv_add(iv_add(iv_mul(A_iv, iv_mul(t0_iv, t0_iv)), iv_mul(Bp_iv, t0_iv)), Cp_iv); // rho^2(t0)
    const Iv q1 = iv_mul(iv_add(iv_mul(two, iv_mul(A_iv, t0_iv)), Bp_iv), dur_iv);                 // (2A t0 + Bp) dur
    const Iv q2 = iv_mul(A_iv, iv_mul(dur_iv, dur_iv));                                            // A dur^2
    std::vector<Iv> Q = {q0, iv_add(q0, iv_mul(iv_rat(1, 2), q1)), iv_add(iv_add(q0, q1), q2)};    // deg 2
    while (static_cast<int>(Q.size()) - 1 < 2 * m) Q = g_elevate(Q);  // elevate tube poly to deg 2m
    // deficit b = rho^2 - S (deg 2m), ASSEMBLED before subdivision; clip to trusted horizon, hull
    std::vector<Iv> b(2 * m + 1);
    for (int k = 0; k <= 2 * m; ++k) b[k] = iv_sub(Q[k], S[k]);
    if (seg_hi > t_hi) {
      double s_cut = (t_hi - t0) / dur;
      if (s_cut > 1.0) s_cut = 1.0; else if (s_cut < 0.0) s_cut = 0.0;
      b = g_left_subcurve(b, s_cut);
    }
    const double sw = g_seg_worst_deficit(b, 0, maxdepth, refuted);
    if (sw > worst_hi) worst_hi = sw;
  }
  if (worst_hi == -std::numeric_limits<double>::infinity())
    return {false, 0.0};   // no segment enforced in [0,t_hi] -> nothing proven -> NOT certified (sound)
  return {worst_hi <= 0.0, -worst_hi};
}

// FLY-OVER half of the cylinder disjunction: proves p_z(t) >= z_clear(t) for ALL continuous t in [0, t_hi].
// SOUND because vertical separation alone lower-bounds the distance to a vertical cylinder: once the drone is
// above z_clear = head_top + reach_pad + r_body + d_safe_v it clears the cylinder regardless of (x,y).
// Combine per-obstacle with certify_segments_vs_sphere(n_axes=2) as a WHOLE-WINDOW disjunction (horizontal OR
// vertical) and AND across obstacles. NEVER a guessed near-window [t_a,t_b] — the proof must hold over the
// ENTIRE trusted horizon [0,t_hi] (clipped by g_left_subcurve exactly as the sphere cert clips S).
//   v_eff_z grows a vertical floor z_clear(t) = z_clear + v_eff_z*(t+delta); v_eff_z=0 (default) = constant
//     plane, valid because the KF pins the obstacle to vz=az=0 (height-bounded rigid body + reach_pad).
//   bez_pad = outward bound on the EGO B-spline->Bezier (Mb/6) rounding of each p_z control point, so the
//     convex-hull lower bound is rigorous at ULP scale (0 = trust the control points exactly).
// Deficit b_k = z_clear(t) - p_z,k (deg n); certified iff worst deficit <= 0 (p_z >= floor everywhere). The
// deficit is assembled BEFORE g_seg_worst_deficit subdivides, so a time-growing floor stays sound.
inline Verdict certify_segments_above_plane(const std::vector<BSeg>& segs, double z_clear,
                                            double t_hi_in = std::numeric_limits<double>::infinity(),
                                            int maxdepth = 16, double v_eff_z = 0.0,
                                            double delta = 0.0, double bez_pad = 0.0,
                                            bool* refuted = nullptr) {
  if (!g_segs_guard(segs, "certify_segments_above_plane")
      || !g_window_covered(segs, t_hi_in, "certify_segments_above_plane"))
    return Verdict{false, -std::numeric_limits<double>::infinity()};
  if (!std::isfinite(z_clear) || !std::isfinite(v_eff_z) || !std::isfinite(delta) || !std::isfinite(bez_pad)) {
    std::fprintf(stderr, "[bcert] certify_segments_above_plane: non-finite plane params -> FAIL-CLOSED\n");
    return Verdict{false, -std::numeric_limits<double>::infinity()};
  }
  double t_end = 0.0;
  for (const auto& sg : segs) t_end = std::max(t_end, sg.t0 + sg.dur);
  const double t_hi = std::min(t_hi_in, t_end);
  const Iv ze = iv_pt(z_clear), ve = iv_pt(v_eff_z), de = iv_pt(delta);
  double worst_hi = -std::numeric_limits<double>::infinity();
  for (const auto& sg : segs) {
    const int n = static_cast<int>(sg.bern.size()) - 1;
    if (n < 1) continue;                                  // need degree >= 1 (EGO/MINCO are cubic/quintic)
    const double t0 = sg.t0, dur = sg.dur, seg_hi = t0 + dur;
    if (t0 >= t_hi || dur <= 0.0) continue;
    // p_z control points as outward-rounded intervals (absorb the B-spline->Bezier Mb/6 rounding via bez_pad)
    std::vector<Iv> pz(n + 1);
    for (int k = 0; k <= n; ++k) { const double bz = sg.bern[k](2); pz[k] = Iv{rdown(bz - bez_pad), rup(bz + bez_pad)}; }
    // vertical floor z_clear(t) = z_clear + v_eff_z*(t+delta): deg-1 Bernstein on [t0, t0+dur], elevate to deg n
    const Iv f0 = iv_add(ze, iv_mul(ve, iv_add(iv_pt(t0), de)));
    const Iv f1 = iv_add(ze, iv_mul(ve, iv_add(iv_pt(seg_hi), de)));
    std::vector<Iv> F = {f0, f1};
    while (static_cast<int>(F.size()) - 1 < n) F = g_elevate(F);
    // deficit b = floor - p_z (deg n), assembled before subdivision; clip straddling segment to t_hi
    std::vector<Iv> b(n + 1);
    for (int k = 0; k <= n; ++k) b[k] = iv_sub(F[k], pz[k]);
    if (seg_hi > t_hi) {
      double s_cut = (t_hi - t0) / dur;
      if (s_cut > 1.0) s_cut = 1.0; else if (s_cut < 0.0) s_cut = 0.0;
      b = g_left_subcurve(b, s_cut);
    }
    const double sw = g_seg_worst_deficit(b, 0, maxdepth, refuted);
    if (sw > worst_hi) worst_hi = sw;
  }
  if (worst_hi == -std::numeric_limits<double>::infinity())
    return {false, 0.0};   // no segment enforced (empty / all n<1 / dur<=0 / beyond t_hi) -> NOT certified (sound)
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
