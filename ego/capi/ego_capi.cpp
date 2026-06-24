// C-API for the standalone (de-ROS'd) EGO-Planner core, for ctypes from the MetaUrban loop.
// Mirrors the sando_capi pattern. Feed: gridmap config + a depth-FOV point cloud + start/goal -> get B-spline traj.
#include "plan_manage/planner_manager.h"
#include "plan_env/grid_map.h"
#include "sando_cpp/bernstein_cert.hpp"   // planner-AGNOSTIC S3 continuous-time conformal deficit certificate
#include <vector>
#include <limits>
#include <Eigen/Eigen>

using ego_planner::EGOPlannerManager;   // namespace if present; fallback below

// Convert EGO's committed uniform-cubic B-spline into per-segment Bezier (Bernstein) control points for the
// continuous-time deficit certificate. READS m->local_data_, which every reboundReplan OVERWRITES, so the
// caller MUST snapshot each verdict before issuing the next replan. Returns empty if no committed trajectory.
static std::vector<sando::bcert::BSeg> build_segs(EGOPlannerManager* m) {
  std::vector<sando::bcert::BSeg> segs;
  if (m->local_data_.duration_ <= 1e-6) return segs;
  Eigen::MatrixXd cp = m->local_data_.position_traj_.get_control_points();   // 3 x Ncols (each col a ctrl pt)
  const double dt = m->local_data_.position_traj_.getInterval();
  const int N = (int)cp.cols(), p = 3;                                       // EGO position B-spline = cubic
  static const double Mb[4][4] = {{1,4,1,0},{0,4,2,0},{0,2,4,0},{0,1,4,1}};  // uniform-cubic B-spline -> Bezier (/6)
  for (int j = 0; j + p < N; ++j) {                                          // segments j = 0 .. N-4
    sando::bcert::BSeg s; s.t0 = j * dt; s.dur = dt; s.bern.resize(4);
    for (int k = 0; k < 4; ++k) {
      Eigen::Vector3d b(0, 0, 0);
      for (int l = 0; l < 4; ++l) b += (Mb[k][l] / 6.0) * cp.col(j + l);
      s.bern[k] = b;
    }
    segs.push_back(s);
  }
  return segs;
}

extern "C" {

void* ego_create() {
  auto* m = new EGOPlannerManager();
  ros::NodeHandle nh;
  m->initPlanModules(nh);                 // shim nh -> params are -1 defaults; override next:
  // sane EGO defaults (typical ego-planner launch yaml)
  m->pp_.max_vel_ = 3.0; m->pp_.max_acc_ = 6.0; m->pp_.max_jerk_ = 4.0;
  m->pp_.ctrl_pt_dist = 0.5; m->pp_.planning_horizen_ = 7.5; m->pp_.feasibility_tolerance_ = 0.05;
  m->setOptParams(/*l_smooth*/1.0, /*l_collision*/0.5, /*l_feasibility*/0.1, /*l_fitness*/1.0,
                  /*dist0*/0.5, /*max_vel*/3.0, /*max_acc*/6.0, /*order*/3);
  GridMapConfig gc; m->grid_map_->initMapFromConfig(gc);
  return (void*)m;
}

void ego_set_gridmap(void* h, double* origin, double* size, double res, double inflation) {
  auto* m = (EGOPlannerManager*)h; GridMapConfig gc;
  gc.map_origin = Eigen::Vector3d(origin[0], origin[1], origin[2]);
  gc.map_size = Eigen::Vector3d(size[0], size[1], size[2]);
  gc.resolution = res; gc.obstacles_inflation = inflation;
  m->grid_map_->initMapFromConfig(gc);
}

void ego_set_params(void* h, double max_vel, double max_acc, double max_jerk, double ctrl_pt_dist,
                    double horizon, double l1, double l2, double l3, double l4, double dist0, int order) {
  auto* m = (EGOPlannerManager*)h;
  m->pp_.max_vel_ = max_vel; m->pp_.max_acc_ = max_acc; m->pp_.max_jerk_ = max_jerk;
  m->pp_.ctrl_pt_dist = ctrl_pt_dist; m->pp_.planning_horizen_ = horizon;
  m->setOptParams(l1, l2, l3, l4, dist0, max_vel, max_acc, order);
}

void ego_update_cloud(void* h, double* pts, int n, double* cam) {
  auto* m = (EGOPlannerManager*)h;
  std::vector<Eigen::Vector3d> cloud; cloud.reserve(n);
  for (int i = 0; i < n; ++i) cloud.emplace_back(pts[3*i], pts[3*i+1], pts[3*i+2]);
  m->grid_map_->update_point_cloud(cloud, Eigen::Vector3d(cam[0], cam[1], cam[2]));
}

int ego_replan(void* h, double* sp, double* sv, double* sa, double* gp, double* gv,
               int poly_init, int random_poly) {
  auto* m = (EGOPlannerManager*)h;
  bool ok = m->reboundReplan(Eigen::Vector3d(sp[0],sp[1],sp[2]), Eigen::Vector3d(sv[0],sv[1],sv[2]),
                             Eigen::Vector3d(sa[0],sa[1],sa[2]), Eigen::Vector3d(gp[0],gp[1],gp[2]),
                             Eigen::Vector3d(gv[0],gv[1],gv[2]), poly_init != 0, random_poly != 0);
  return ok ? 1 : 0;
}

double ego_traj_duration(void* h) { return ((EGOPlannerManager*)h)->local_data_.duration_; }

// sample the committed B-spline at local time t -> out[0..2]=pos, [3..5]=vel, [6..8]=acc
int ego_traj_eval(void* h, double t, double* out) {
  auto* m = (EGOPlannerManager*)h;
  if (m->local_data_.duration_ <= 1e-6) return 0;
  Eigen::VectorXd p = m->local_data_.position_traj_.evaluateDeBoorT(t);
  Eigen::VectorXd v = m->local_data_.velocity_traj_.evaluateDeBoorT(t);
  Eigen::VectorXd a = m->local_data_.acceleration_traj_.evaluateDeBoorT(t);
  for (int i = 0; i < 3; ++i) { out[i] = p[i]; out[3+i] = v.size()>i? v[i]:0.0; out[6+i] = a.size()>i? a[i]:0.0; }
  return 1;
}

int ego_check_occ(void* h, double x, double y, double z) {
  return ((EGOPlannerManager*)h)->grid_map_->getInflateOccupancy(Eigen::Vector3d(x, y, z));
}

// ===== OUR safety envelope ported ONTO EGO (planner-agnostic certified safety layer) =====
// Certify EGO's committed cubic B-spline against ONE sphere obstacle whose centre is the analytic
// polynomial c(t)=c0+vel*t+0.5*acc*t^2, with total inflated radius R (= r_obs+r_body+d_safe+q_conformal).
// Converts each uniform-cubic B-spline segment to its Bezier (Bernstein) control points and runs the
// SAME continuous-time conformal deficit certificate used for MINCO -> P(collision)<=eps, no sampling.
// Returns 1 if CERTIFIED (||p(t)-c(t)||>=R for all t in [0,t_hi]); writes the deficit margin to *margin_out.
// v_eff = tube growth rate (risk dial): 0 = trust the prediction over [0,t_hi] exactly (tightest/fastest);
//   >0 = inflate the tube by v_eff*(t+delta) to cover reachable / conformal prediction drift (safer/slower).
// delta = perception->commit latency (tube already inflated by v_eff*delta at t=0). t_hi = trust window.
int ego_certify(void* h, double* c0, double* vel, double* acc, double R, double t_hi,
                double v_eff, double delta, double* margin_out) {
  auto* m = (EGOPlannerManager*)h;
  auto segs = build_segs(m);
  if (segs.empty()) { if (margin_out) *margin_out = -1.0; return 0; }
  const double th = (t_hi > 0.0) ? t_hi : std::numeric_limits<double>::infinity();
  auto v = sando::bcert::certify_segments_vs_sphere(
      segs, Eigen::Vector3d(c0[0], c0[1], c0[2]), Eigen::Vector3d(vel[0], vel[1], vel[2]),
      Eigen::Vector3d(acc[0], acc[1], acc[2]), R, th, /*maxdepth*/16, v_eff, delta);
  if (margin_out) *margin_out = v.margin;
  return v.certified ? 1 : 0;
}

// AROUND half of the cylinder disjunction: 2-D HORIZONTAL separation sqrt(dx^2+dy^2) >= R against the moving
// cylinder axis c(t)=c0+vel*t+0.5*acc*t^2 (z ignored). SOUND for a full-height cylinder where the 3-D sphere
// is NOT (the sphere would falsely clear a low hover sitting horizontally inside the footprint). Same tube knobs.
int ego_certify_horizontal(void* h, double* c0, double* vel, double* acc, double R, double t_hi,
                           double v_eff, double delta, double* margin_out) {
  auto* m = (EGOPlannerManager*)h;
  auto segs = build_segs(m);
  if (segs.empty()) { if (margin_out) *margin_out = -1.0; return 0; }
  const double th = (t_hi > 0.0) ? t_hi : std::numeric_limits<double>::infinity();
  auto v = sando::bcert::certify_segments_vs_sphere(
      segs, Eigen::Vector3d(c0[0], c0[1], c0[2]), Eigen::Vector3d(vel[0], vel[1], vel[2]),
      Eigen::Vector3d(acc[0], acc[1], acc[2]), R, th, /*maxdepth*/16, v_eff, delta, /*n_axes*/2);
  if (margin_out) *margin_out = v.margin;
  return v.certified ? 1 : 0;
}

// OVER half of the cylinder disjunction: VERTICAL clearance p_z(t) >= z_clear for all t in [0,t_hi]. Once
// above z_clear (= head_top+reach_pad+r_body+d_safe_v) the drone clears the cylinder regardless of (x,y).
// Per obstacle the loop ORs horizontal-vs-vertical (each a whole-window proof) and ANDs across obstacles.
int ego_certify_above(void* h, double z_clear, double t_hi, double v_eff_z, double delta,
                      double bez_pad, double* margin_out) {
  auto* m = (EGOPlannerManager*)h;
  auto segs = build_segs(m);
  if (segs.empty()) { if (margin_out) *margin_out = -1.0; return 0; }
  const double th = (t_hi > 0.0) ? t_hi : std::numeric_limits<double>::infinity();
  auto v = sando::bcert::certify_segments_above_plane(segs, z_clear, th, /*maxdepth*/16, v_eff_z, delta, bez_pad);
  if (margin_out) *margin_out = v.margin;
  return v.certified ? 1 : 0;
}

void ego_destroy(void* h) { delete (EGOPlannerManager*)h; }

}  // extern C
