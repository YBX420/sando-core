// bug2 witness against the REAL header: is the shipped subdivision order sound,
// and is the counterfactual "frozen R^2, subdivide S only" order unsound?
#include "sando_cpp/bernstein_cert.hpp"
#include <cstdio>
#include <random>
#include <cmath>
using namespace sando::bcert;

static Eigen::Vector3d bez(const std::vector<Eigen::Vector3d>& b, double s) {
  std::vector<Eigen::Vector3d> c = b;
  for (size_t lvl = 1; lvl < b.size(); ++lvl)
    for (size_t k = 0; k + lvl < b.size(); ++k) c[k] = (1 - s) * c[k] + s * c[k + 1];
  return c[0];
}

// Counterfactual BUGGY order: identical S assembly to the shipped core (same iv_* primitives,
// same elevations), but rho^2 FROZEN to a scalar and only S subdivided (old seg_worst pattern).
// at_start=true  -> freeze at rho^2(t0)           (the naive upgrade: UNSOUND claim under test)
// at_start=false -> freeze at rho^2(t_end) hull   (conservative freeze: sound-but-incomplete claim)
static Verdict frozen_certify(const std::vector<BSeg>& segs, const Eigen::Vector3d& c0,
                              const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                              double R, double v_eff, double delta, bool at_start,
                              int maxdepth = 16) {
  double t_end = 0.0;
  for (const auto& sg : segs) t_end = std::max(t_end, sg.t0 + sg.dur);
  double worst = -std::numeric_limits<double>::infinity();
  for (const auto& sg : segs) {
    const int n = static_cast<int>(sg.bern.size()) - 1;
    if (n < 0) continue;
    const double t0 = sg.t0, dur = sg.dur;
    if (dur <= 0.0) continue;
    const int m = n < 2 ? 2 : n;
    std::vector<Iv> S(2 * m + 1, Iv{0.0, 0.0});
    for (int coord = 0; coord < 3; ++coord) {
      const double c0c = c0(coord), vc = vel(coord), ac = acc(coord);
      Iv g0 = iv_add(iv_add(iv_pt(c0c), iv_mul(iv_pt(vc), iv_pt(t0))),
                     iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(t0), iv_pt(t0))));
      Iv g1 = iv_mul(iv_pt(dur), iv_add(iv_pt(vc), iv_mul(iv_pt(ac), iv_pt(t0))));
      Iv g2 = iv_mul(iv_pt(0.5 * ac), iv_mul(iv_pt(dur), iv_pt(dur)));
      std::vector<Iv> o = {g0, iv_add(g0, iv_mul(iv_rat(1, 2), g1)), iv_add(iv_add(g0, g1), g2)};
      while (static_cast<int>(o.size()) - 1 < m) o = g_elevate(o);
      std::vector<Iv> w(n + 1);
      for (int k = 0; k <= n; ++k) w[k] = iv_pt(sg.bern[k](coord));
      while (static_cast<int>(w.size()) - 1 < m) w = g_elevate(w);
      for (int k = 0; k <= m; ++k) w[k] = iv_sub(w[k], o[k]);
      std::vector<Iv> Sc = g_square(w);
      for (int k = 0; k <= 2 * m; ++k) S[k] = iv_add(S[k], Sc[k]);
    }
    const double rho_frozen = at_start ? R + v_eff * (t0 + delta) : R + v_eff * (t_end + delta);
    const Iv R2 = iv_mul(iv_pt(rho_frozen), iv_pt(rho_frozen));
    const double sw = g_seg_worst(S, R2, 0, maxdepth);
    if (sw > worst) worst = sw;
  }
  if (worst == -std::numeric_limits<double>::infinity()) return {false, 0.0};
  return {worst <= 0.0, -worst};
}

// dense-sampled ground truth: min over t of (|p(t)-c(t)| - rho(t)); negative => real violation
static double ground_truth(const std::vector<BSeg>& segs, const Eigen::Vector3d& c0,
                           const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                           double R, double v_eff, double delta, double* t_at = nullptr) {
  double worst = std::numeric_limits<double>::infinity();
  for (const auto& sg : segs) {
    for (int i = 0; i <= 200000; ++i) {
      const double s = i / 200000.0, t = sg.t0 + s * sg.dur;
      const Eigen::Vector3d p = bez(sg.bern, s);
      const Eigen::Vector3d c = c0 + vel * t + 0.5 * acc * t * t;
      const double m = (p - c).norm() - (R + v_eff * (t + delta));
      if (m < worst) { worst = m; if (t_at) *t_at = t; }
    }
  }
  return worst;
}

int main() {
  const Eigen::Vector3d zero(0, 0, 0);

  // ---- W1: drone parked at distance 1.5, tube rho(t)=1+t on [0,1] (truly violated for t>0.5)
  {
    BSeg s; s.t0 = 0.0; s.dur = 1.0;
    s.bern = {Eigen::Vector3d(1.5, 0, 0), Eigen::Vector3d(1.5, 0, 0)};
    std::vector<BSeg> segs{s};
    double t_at = 0;
    const double gt = ground_truth(segs, zero, zero, zero, 1.0, 1.0, 0.0, &t_at);
    bool refuted = false;
    Verdict v = certify_segments_vs_sphere(segs, zero, zero, zero, 1.0,
                                           std::numeric_limits<double>::infinity(),
                                           16, /*v_eff*/1.0, /*delta*/0.0, 3, &refuted);
    Verdict f = frozen_certify(segs, zero, zero, zero, 1.0, 1.0, 0.0, /*at_start*/true);
    printf("W1 unsafe case: ground truth min(dist-rho) = %+.4f at t=%.3f  -> %s\n",
           gt, t_at, gt < 0 ? "TRUE VIOLATION" : "safe");
    printf("  shipped (group-then-split): certified=%d  margin=%+.6g  refuted=%d   %s\n",
           (int)v.certified, v.margin, (int)refuted,
           !v.certified ? "[correctly refuses]" : "[FALSE CERTIFY !!]");
    printf("  frozen rho^2(t0) + subdiv S only: certified=%d  margin=%+.6g   %s\n",
           (int)f.certified, f.margin, f.certified ? "[FALSE CERTIFY -> UNSOUND ORDER]" : "[refuses]");
  }

  // ---- W2: drone flying away p(t)=1.6+2t, same tube (truly safe, min slack in distance = 0.6 at t=0)
  {
    BSeg s; s.t0 = 0.0; s.dur = 1.0;
    s.bern = {Eigen::Vector3d(1.6, 0, 0), Eigen::Vector3d(3.6, 0, 0)};
    std::vector<BSeg> segs{s};
    const double gt = ground_truth(segs, zero, zero, zero, 1.0, 1.0, 0.0);
    Verdict v = certify_segments_vs_sphere(segs, zero, zero, zero, 1.0,
                                           std::numeric_limits<double>::infinity(),
                                           16, 1.0, 0.0, 3, nullptr);
    Verdict fh = frozen_certify(segs, zero, zero, zero, 1.0, 1.0, 0.0, /*at_start*/false); // hull freeze
    printf("\nW2 safe case: ground truth min(dist-rho) = %+.4f (truly safe)\n", gt);
    printf("  shipped (group-then-split): certified=%d  margin=%+.6g   [certifies, correct]\n",
           (int)v.certified, v.margin);
    printf("  frozen at hull rho^2(t_end) + subdiv S only: certified=%d  margin=%+.6g   %s\n",
           (int)fh.certified, fh.margin,
           fh.certified ? "[certifies]" : "[cannot certify -> sound but strictly INCOMPLETE]");
  }

  // ---- regression sanity: v_eff=0 -> frozen and shipped must agree (constant tube is the legal case)
  {
    BSeg s; s.t0 = 0.0; s.dur = 1.0;
    s.bern = {Eigen::Vector3d(1.5, 0, 0), Eigen::Vector3d(1.5, 0, 0)};
    std::vector<BSeg> segs{s};
    Verdict v = certify_segments_vs_sphere(segs, zero, zero, zero, 1.0,
                                           std::numeric_limits<double>::infinity(), 16, 0.0, 0.0, 3, nullptr);
    Verdict f = frozen_certify(segs, zero, zero, zero, 1.0, 0.0, 0.0, true);
    printf("\nconstant tube (v_eff=0): shipped certified=%d margin=%.9g | frozen certified=%d margin=%.9g  %s\n",
           (int)v.certified, v.margin, (int)f.certified, f.margin,
           (v.certified == f.certified && std::abs(v.margin - f.margin) < 1e-12)
               ? "[agree -> freeze is exact ONLY for constant R^2]" : "[DISAGREE]");
  }

  // ---- randomized soundness sweep: shipped must never false-certify; count frozen false-certifies
  {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    int n_cases = 400, shipped_fc = 0, frozen_fc = 0, shipped_cert = 0, frozen_at_risk = 0;
    for (int c = 0; c < n_cases; ++c) {
      BSeg s;
      s.t0 = 2.0 * std::abs(U(rng));            // nonzero anchor too
      s.dur = 0.5 + std::abs(U(rng));
      const int deg = 1 + (c % 4);              // deg 1..4
      s.bern.resize(deg + 1);
      for (int k = 0; k <= deg; ++k)
        s.bern[k] = Eigen::Vector3d(3.0 * U(rng), 3.0 * U(rng), 3.0 * U(rng));
      std::vector<BSeg> segs{s};
      const Eigen::Vector3d c0(2.0 * U(rng), 2.0 * U(rng), 2.0 * U(rng));
      const Eigen::Vector3d vel(0.5 * U(rng), 0.5 * U(rng), 0.5 * U(rng));
      const Eigen::Vector3d acc(0.2 * U(rng), 0.2 * U(rng), 0.2 * U(rng));
      const double R = 0.3 + 0.7 * std::abs(U(rng));
      const double v_eff = 0.2 + 0.8 * std::abs(U(rng));
      const double delta = 0.2 * std::abs(U(rng));
      const double gt = ground_truth(segs, c0, vel, acc, R, v_eff, delta);
      Verdict v = certify_segments_vs_sphere(segs, c0, vel, acc, R,
                                             std::numeric_limits<double>::infinity(),
                                             16, v_eff, delta, 3, nullptr);
      Verdict f = frozen_certify(segs, c0, vel, acc, R, v_eff, delta, true);
      if (v.certified) ++shipped_cert;
      if (v.certified && gt < -1e-7) ++shipped_fc;
      if (f.certified && gt < -1e-7) { ++frozen_fc; }
      if (f.certified && !v.certified) ++frozen_at_risk;
    }
    printf("\nrandom sweep (%d cases, deg 1-4, t0>0, moving obstacles, v_eff/delta > 0):\n", n_cases);
    printf("  shipped: certified %d/%d, FALSE CERTIFIES vs dense sampling: %d   %s\n",
           shipped_cert, n_cases, shipped_fc, shipped_fc == 0 ? "[sound]" : "[UNSOUND !!]");
    printf("  frozen-at-start: FALSE CERTIFIES: %d  (certified-but-shipped-refuses: %d)  %s\n",
           frozen_fc, frozen_at_risk, frozen_fc > 0 ? "[UNSOUND, as predicted]" : "[no violation found]");
  }

  // ---- interval order-equivalence at the leaf: assemble-then-split vs split-then-assemble
  {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-5.0, 5.0);
    double maxd = 0.0;
    for (int c = 0; c < 200; ++c) {
      const int m = 4 + (c % 5);
      std::vector<Iv> Q(m + 1), S(m + 1);
      for (int k = 0; k <= m; ++k) { Q[k] = iv_pt(U(rng)); S[k] = iv_pt(U(rng)); }
      std::vector<Iv> b(m + 1);
      for (int k = 0; k <= m; ++k) b[k] = iv_sub(Q[k], S[k]);
      std::vector<Iv> bL, bR, qL, qR, sL, sR;
      g_subdiv(b, bL, bR); g_subdiv(Q, qL, qR); g_subdiv(S, sL, sR);
      for (int k = 0; k <= m; ++k) {
        const Iv altL = iv_sub(qL[k], sL[k]), altR = iv_sub(qR[k], sR[k]);
        maxd = std::max({maxd, std::abs(bL[k].lo - altL.lo), std::abs(bL[k].hi - altL.hi),
                               std::abs(bR[k].lo - altR.lo), std::abs(bR[k].hi - altR.hi)});
      }
    }
    printf("\ninterval order equivalence (200 random deg-4..8): max endpoint diff "
           "assemble-then-split vs split-then-assemble = %.3e (ulp scale; both outward-sound)\n", maxd);
  }
  return 0;
}
