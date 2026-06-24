// cert_capi — PLANNER-AGNOSTIC certificate entry point. Certifies an ARBITRARY committed trajectory given as
// piecewise position Bezier/Bernstein control points (BSeg), decoupled from any planner. Any external planner
// (EGO B-spline, GCOPTER/MINCO, min-snap polynomial, MINVO, native Bezier, ...) wraps our continuous-time
// Bernstein collision certificate by converting its trajectory to (control_points, deg, t0, dur) and calling here.
//
// Build (hand, like the other capis):
//   g++ -O2 -shared -fPIC -std=c++17 -o cert_capi.so cert_capi.cpp -I ../include -I ../third_party/eigen
#include "sando_cpp/bernstein_cert.hpp"
#include <vector>

using namespace sando::bcert;

static std::vector<BSeg> build_segs(const double* cp, int n_seg, int deg,
                                    const double* t0s, const double* durs) {
  const int npts = deg + 1;
  std::vector<BSeg> segs(n_seg);
  for (int i = 0; i < n_seg; ++i) {
    segs[i].t0 = t0s[i];
    segs[i].dur = durs[i];
    segs[i].bern.resize(npts);
    for (int k = 0; k < npts; ++k) {
      const double* p = cp + (static_cast<long>(i) * npts + k) * 3;   // segment-major, control-point, xyz
      segs[i].bern[k] = Eigen::Vector3d(p[0], p[1], p[2]);
    }
  }
  return segs;
}

extern "C" {

// AROUND half of the cylinder disjunction (n_axes=2 = horizontal) OR the full 3-D sphere (n_axes=3).
int cert_horizontal_bseg(const double* cp, int n_seg, int deg, const double* t0s, const double* durs,
                         const double* c0, const double* vel, const double* acc, double R, double t_hi,
                         double v_eff, double delta, int n_axes, double* margin_out) {
  if (n_seg <= 0 || deg < 1) { if (margin_out) *margin_out = -1.0; return 0; }
  auto segs = build_segs(cp, n_seg, deg, t0s, durs);
  const Eigen::Vector3d C(c0[0], c0[1], c0[2]), V(vel[0], vel[1], vel[2]), A(acc[0], acc[1], acc[2]);
  const double th = (t_hi > 0.0) ? t_hi : std::numeric_limits<double>::infinity();
  auto v = certify_segments_vs_sphere(segs, C, V, A, R, th, /*maxdepth*/16, v_eff, delta, n_axes);
  if (margin_out) *margin_out = v.margin;
  return v.certified ? 1 : 0;
}

// OVER half of the cylinder disjunction: p_z(t) >= z_clear for all t.
int cert_above_bseg(const double* cp, int n_seg, int deg, const double* t0s, const double* durs,
                    double z_clear, double t_hi, double v_eff_z, double delta, double bez_pad, double* margin_out) {
  if (n_seg <= 0 || deg < 1) { if (margin_out) *margin_out = -1.0; return 0; }
  auto segs = build_segs(cp, n_seg, deg, t0s, durs);
  const double th = (t_hi > 0.0) ? t_hi : std::numeric_limits<double>::infinity();
  auto v = certify_segments_above_plane(segs, z_clear, th, /*maxdepth*/16, v_eff_z, delta, bez_pad);
  if (margin_out) *margin_out = v.margin;
  return v.certified ? 1 : 0;
}

}  // extern "C"
