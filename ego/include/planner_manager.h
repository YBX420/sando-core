#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include "ros_shim.h"

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(ros::NodeHandle &nh);
    void setOptParams(double l1, double l2, double l3, double l4, double d0, double mv, double ma, int order) {
      bspline_optimizer_rebound_->setParamsManual(l1, l2, l3, l4, d0, mv, ma, order);
    }
    void setMovingObstacles(const std::vector<BsplineOptimizer::MovingObs> &obs, double lambda) {
      bspline_optimizer_rebound_->setMovingObstacles(obs, lambda);
    }
    // GUIDE PATH (north-star arm, 2026-07-22): a polyline reference the NEXT reboundReplan uses as
    // its INITIAL point set (replacing the polynomial / previous-trajectory init). The rebound
    // optimizer then deforms it only where the (static) occupancy demands -- "plan along this
    // KF-corrected reference line" instead of steering EGO by walls and sub-goal carrots.
    // Empty vector = cleared = byte-identical legacy init. Persists until replaced or cleared.
    void setGuidePath(const std::vector<Eigen::Vector3d> &pts) { guide_path_ = pts; }
    // guide-attraction dial (0 = init-only guide, byte-identical legacy behaviour)
    void setGuideAttract(double lambda, double tol) { guide_lambda_ = lambda; guide_tol_ = tol; }
    // plan-to-plan consistency dial (0 = legacy) + the per-tick snapshot of the FLOWN trajectory.
    // snapshotPrev is called ONCE at tick start: several replans may run inside one decision tick
    // (primary / retry / rollback) and every one must be tied to the trajectory the executor is
    // actually flying, not to a sibling candidate solved a millisecond earlier.
    void setConsistency(double lambda, double tau) { cons_lambda_ = lambda; cons_tau_ = tau; }
    void setPhysSmooth(double la, double ac) { pacc_lambda_ = la; pacc_th_ = ac; }
    void snapshotPrev(double t_shift) {
      if (cons_lambda_ > 0.0 && local_data_.duration_ > 1e-3)
        bspline_optimizer_rebound_->snapshotPrevTraj(local_data_.position_traj_, t_shift,
                                                     local_data_.duration_);
      else
        bspline_optimizer_rebound_->clearPrevTraj();
    }

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    /* main planning algorithms & modules */

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    std::vector<Eigen::Vector3d> guide_path_;   // north-star guide (empty = legacy init)
    double guide_lambda_{0.0}, guide_tol_{0.1}; // optimizer attraction toward guide_path_
    double cons_lambda_{0.0}, cons_tau_{0.5};   // plan-to-plan consistency dial
    double pacc_lambda_{0.0}, pacc_th_{6.0};  // physical accel comfort-hinge dial

    int continous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif