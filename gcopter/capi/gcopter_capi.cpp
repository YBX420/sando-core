// gcopter_capi — a NON-ROS C ABI around ZJU GCOPTER (Wang et al.), so it can be driven headless from Python the
// same way ego_capi drives EGO-Planner. Replicates the example node global_planning.cpp's plan() pipeline:
//   point cloud -> voxel_map (setOccupied + dilate) -> sfc_gen::planPath (our A*, OMPL removed) -> FIRI convexCover
//   -> shortCut -> gcopter::GCOPTER_PolytopeSFC.setup + optimize -> MINCO Trajectory<5>.
// The committed trajectory is returned as per-piece duration + the 3x6 DESCENDING-power CoefficientMat (col0 = t^5)
// that cert_bridge.minco_descending_to_bseg already consumes -> our continuous-time Bernstein cert certifies it.
#include <cstdint>   // quickhull.hpp uses std::uint8_t but doesn't include it (newer libstdc++ needs it explicit)
// include gcopter's geometry deps BEFORE gcopter.hpp (it uses geo_utils::/firi:: at definition, non-dependent names)
#include "gcopter/geo_utils.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/sdlp.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <cmath>

extern "C" {

// Returns n_pieces (>0 on success, 0 on failure). out_coeffs must hold max_pieces*18 doubles (3x6 row-major per
// piece, descending power), out_durs max_pieces doubles.
int gcopter_plan(const double *cloud, int n, const double *start, const double *goal,
                 const double *mapbound, double voxelWidth, double dilateRadius,
                 double vmax, double weightT, int max_pieces,
                 double *out_coeffs, double *out_durs)
{
    const Eigen::Vector3i xyz(int((mapbound[1] - mapbound[0]) / voxelWidth),
                              int((mapbound[3] - mapbound[2]) / voxelWidth),
                              int((mapbound[5] - mapbound[4]) / voxelWidth));
    if (xyz(0) <= 0 || xyz(1) <= 0 || xyz(2) <= 0) return 0;
    const Eigen::Vector3d offset(mapbound[0], mapbound[2], mapbound[4]);
    voxel_map::VoxelMap voxelMap(xyz, offset, voxelWidth);
    for (int i = 0; i < n; i++)
    {
        const double x = cloud[3 * i], y = cloud[3 * i + 1], z = cloud[3 * i + 2];
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
            voxelMap.setOccupied(Eigen::Vector3d(x, y, z));
    }
    voxelMap.dilate(int(std::ceil(dilateRadius / voxelMap.getScale())));

    const Eigen::Vector3d s(start[0], start[1], start[2]), g(goal[0], goal[1], goal[2]);
    // degenerate guard: a near-zero / near-vertical start->goal (e.g. the harness "climb" sub-goal) makes the SFC
    // a flat polytope and crashes GCOPTER's optimiser with an Eigen negative-dim assertion -> bail cleanly.
    if ((g - s).norm() < 3.0 * voxelWidth || (g.head<2>() - s.head<2>()).norm() < 2.0 * voxelWidth) return 0;
    std::vector<Eigen::Vector3d> route;
    sfc_gen::planPath<voxel_map::VoxelMap>(s, g, voxelMap.getOrigin(), voxelMap.getCorner(), &voxelMap, 0.02, route);
    const bool dbg = std::getenv("GCOPTER_DEBUG") != nullptr;
    if (dbg) std::fprintf(stderr, "[gcap] cloud=%d route=%zu\n", n, route.size());
    if (route.size() < 2) return 0;

    // line-of-sight simplify: the grid A* route is jagged (voxel-center steps); convexCover seeds a polytope per
    // waypoint, and jagged seeds make thin / non-overlapping corridors that diverge the optimisation. Greedily keep
    // only the farthest collision-free-straight-reachable waypoint -> a sparse smooth polyline (start, corners, goal).
    {
        auto losClear = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
            const double L = (b - a).norm();
            const int K = std::max(2, int(L / (voxelWidth * 0.5)));
            for (int i = 0; i <= K; i++)
                if (voxelMap.query(Eigen::Vector3d(a + (b - a) * (double(i) / K)))) return false;
            return true;
        };
        std::vector<Eigen::Vector3d> simp;
        simp.push_back(route.front());
        size_t i = 0;
        while (i + 1 < route.size())
        {
            size_t j = route.size() - 1;
            for (; j > i + 1; --j)
                if (losClear(route[i], route[j])) break;
            simp.push_back(route[j]);
            i = j;
        }
        route = simp;
        // FIRI's convexCover can hit a degenerate (negative-dim) state on a SINGLE straight segment (route == 2
        // points); densify to >=3 waypoints so each polytope is seeded from a proper sub-segment.
        if (route.size() == 2)
        {
            const Eigen::Vector3d a = route[0], b = route[1];
            route.clear();
            route.push_back(a);
            route.push_back(0.5 * (a + b));
            route.push_back(b);
        }
        if (dbg) std::fprintf(stderr, "[gcap] route simplified -> %zu\n", route.size());
    }

    std::vector<Eigen::MatrixX4d> hPolys;
    std::vector<Eigen::Vector3d> pc;
    voxelMap.getSurf(pc);
    sfc_gen::convexCover(route, pc, voxelMap.getOrigin(), voxelMap.getCorner(), 7.0, 3.0, hPolys);
    sfc_gen::shortCut(hPolys);
    if (dbg) std::fprintf(stderr, "[gcap] surf=%zu hPolys=%zu\n", pc.size(), hPolys.size());
    if (hPolys.empty()) return 0;

    Eigen::Matrix3d iniState, finState;
    iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();   // cols = [pos, vel, acc]
    finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    gcopter::GCOPTER_PolytopeSFC gc;
    Eigen::VectorXd magBounds(5), penWeights(5), physParams(6);
    magBounds << vmax, 2.1, 1.05, 2.0, 12.0;          // [v_max, omg_max, theta_max, thrust_min, thrust_max]
    penWeights << 1.0e4, 1.0e4, 1.0e4, 1.0e4, 1.0e5;  // ChiVec (default config)
    physParams << 0.61, 9.8, 0.70, 0.80, 0.01, 1.0e-4;

    Trajectory<5> traj;
    if (dbg) std::fprintf(stderr, "[gcap] setup...\n");
    if (!gc.setup(weightT, iniState, finState, hPolys, INFINITY, 1.0e-2, 16, magBounds, penWeights, physParams))
        return 0;
    if (dbg) std::fprintf(stderr, "[gcap] setup ok, optimize...\n");
    if (std::isinf(gc.optimize(traj, 1.0e-5))) return 0;
    if (dbg) std::fprintf(stderr, "[gcap] optimize ok\n");

    const int np = traj.getPieceNum();
    if (np <= 0 || np > max_pieces) return 0;
    for (int i = 0; i < np; i++)
    {
        const double dur = traj[i].getDuration();
        if (!std::isfinite(dur) || dur <= 0.0) return 0;
        out_durs[i] = dur;
        const Eigen::Matrix<double, 3, 6> &C = traj[i].getCoeffMat();   // descending power, col0 = t^5
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 6; c++)
            {
                if (!std::isfinite(C(r, c))) return 0;                  // reject any NaN/Inf coeff
                out_coeffs[i * 18 + r * 6 + c] = C(r, c);
            }
    }
    return np;
}

}  // extern "C"
