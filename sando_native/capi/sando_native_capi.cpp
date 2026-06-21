// sando_native_capi.cpp — flat extern "C" ABI over the de-ROS'd native MIT-ACL SANDO planner.
// Mirrors the MINCO sando_capi so sando_native_bridge.py can drive it via ctypes (no ROS).
// Params come from native SANDO's own config/sando.yaml (loaded with yaml-cpp).
#include "sando/sando_type.hpp"
#include "sando/sando.hpp"
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <memory>
#include <cstdio>

#define API extern "C" __attribute__((visibility("default")))

// ---------------- Parameters from native SANDO's sando.yaml ----------------
API void* sn_params_from_yaml(const char* path) {
  try {
    Parameters* pp = new Parameters();
    Parameters& p = *pp;
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node y = root["sando_node"] ? root["sando_node"]["ros__parameters"] : root;
    if (!y) y = root;
  if (y["sim_env"]) p.sim_env = y["sim_env"].as<std::string>();
  if (y["use_global_pc"]) p.use_global_pc = y["use_global_pc"].as<bool>();
  if (y["vehicle_type"]) p.vehicle_type = y["vehicle_type"].as<std::string>();
  if (y["provide_goal_in_global_frame"]) p.provide_goal_in_global_frame = y["provide_goal_in_global_frame"].as<bool>();
  if (y["use_hardware"]) p.use_hardware = y["use_hardware"].as<bool>();
  if (y["flight_mode"]) p.flight_mode = y["flight_mode"].as<std::string>();
  if (y["visual_level"]) p.visual_level = y["visual_level"].as<int>();
  if (y["global_planner"]) p.global_planner = y["global_planner"].as<std::string>();
  if (y["global_planner_verbose"]) p.global_planner_verbose = y["global_planner_verbose"].as<bool>();
  if (y["factor_hgp"]) p.factor_hgp = y["factor_hgp"].as<double>();
  if (y["inflation_hgp"]) p.inflation_hgp = y["inflation_hgp"].as<double>();
  if (y["x_min"]) p.x_min = y["x_min"].as<double>();
  if (y["x_max"]) p.x_max = y["x_max"].as<double>();
  if (y["y_min"]) p.y_min = y["y_min"].as<double>();
  if (y["y_max"]) p.y_max = y["y_max"].as<double>();
  if (y["z_min"]) p.z_min = y["z_min"].as<double>();
  if (y["z_max"]) p.z_max = y["z_max"].as<double>();
  if (y["hgp_timeout_duration_ms"]) p.hgp_timeout_duration_ms = y["hgp_timeout_duration_ms"].as<int>();
  if (y["max_num_expansion"]) p.max_num_expansion = y["max_num_expansion"].as<int>();
  if (y["use_free_start"]) p.use_free_start = y["use_free_start"].as<bool>();
  if (y["free_start_factor"]) p.free_start_factor = y["free_start_factor"].as<double>();
  if (y["use_free_goal"]) p.use_free_goal = y["use_free_goal"].as<bool>();
  if (y["free_goal_factor"]) p.free_goal_factor = y["free_goal_factor"].as<double>();
  if (y["max_dist_vertexes"]) p.max_dist_vertexes = y["max_dist_vertexes"].as<double>();
  if (y["w_unknown"]) p.w_unknown = y["w_unknown"].as<double>();
  if (y["w_align"]) p.w_align = y["w_align"].as<double>();
  if (y["decay_len_cells"]) p.decay_len_cells = y["decay_len_cells"].as<double>();
  if (y["w_side"]) p.w_side = y["w_side"].as<double>();
  if (y["heat_weight"]) p.heat_weight = y["heat_weight"].as<double>();
  if (y["use_heat_map"]) p.use_heat_map = y["use_heat_map"].as<bool>();
  if (y["dynamic_heat_enabled"]) p.dynamic_heat_enabled = y["dynamic_heat_enabled"].as<bool>();
  if (y["dynamic_as_occupied_current"]) p.dynamic_as_occupied_current = y["dynamic_as_occupied_current"].as<bool>();
  if (y["dynamic_as_occupied_future"]) p.dynamic_as_occupied_future = y["dynamic_as_occupied_future"].as<bool>();
  if (y["heat_alpha0"]) p.heat_alpha0 = y["heat_alpha0"].as<double>();
  if (y["heat_alpha1"]) p.heat_alpha1 = y["heat_alpha1"].as<double>();
  if (y["heat_p"]) p.heat_p = y["heat_p"].as<int>();
  if (y["heat_q"]) p.heat_q = y["heat_q"].as<int>();
  if (y["heat_tau_ratio"]) p.heat_tau_ratio = y["heat_tau_ratio"].as<double>();
  if (y["heat_gamma"]) p.heat_gamma = y["heat_gamma"].as<double>();
  if (y["heat_Hmax"]) p.heat_Hmax = y["heat_Hmax"].as<double>();
  if (y["dyn_base_inflation_m"]) p.dyn_base_inflation_m = y["dyn_base_inflation_m"].as<double>();
  if (y["dyn_heat_tube_radius_m"]) p.dyn_heat_tube_radius_m = y["dyn_heat_tube_radius_m"].as<double>();
  if (y["heat_num_samples"]) p.heat_num_samples = y["heat_num_samples"].as<int>();
  if (y["static_heat_enabled"]) p.static_heat_enabled = y["static_heat_enabled"].as<bool>();
  if (y["static_heat_alpha"]) p.static_heat_alpha = y["static_heat_alpha"].as<double>();
  if (y["static_heat_p"]) p.static_heat_p = y["static_heat_p"].as<int>();
  if (y["static_heat_Hmax"]) p.static_heat_Hmax = y["static_heat_Hmax"].as<double>();
  if (y["static_heat_rmax_m"]) p.static_heat_rmax_m = y["static_heat_rmax_m"].as<double>();
  if (y["static_heat_boundary_only"]) p.static_heat_boundary_only = y["static_heat_boundary_only"].as<bool>();
  if (y["static_heat_apply_on_unknown"]) p.static_heat_apply_on_unknown = y["static_heat_apply_on_unknown"].as<bool>();
  if (y["static_heat_exclude_dynamic"]) p.static_heat_exclude_dynamic = y["static_heat_exclude_dynamic"].as<bool>();
  if (y["use_soft_cost_obstacles"]) p.use_soft_cost_obstacles = y["use_soft_cost_obstacles"].as<bool>();
  if (y["obstacle_soft_cost"]) p.obstacle_soft_cost = y["obstacle_soft_cost"].as<double>();
  if (y["los_cells"]) p.los_cells = y["los_cells"].as<int>();
  if (y["min_len"]) p.min_len = y["min_len"].as<double>();
  if (y["min_turn"]) p.min_turn = y["min_turn"].as<double>();
  if (y["use_state_update"]) p.use_state_update = y["use_state_update"].as<bool>();
  if (y["environment_assumption"]) p.environment_assumption = y["environment_assumption"].as<std::string>();
  if (y["sfc_size"]) p.sfc_size = y["sfc_size"].as<std::vector<double>>();
  if (y["min_dist_from_agent_to_traj"]) p.min_dist_from_agent_to_traj = y["min_dist_from_agent_to_traj"].as<double>();
  if (y["use_shrinked_box"]) p.use_shrinked_box = y["use_shrinked_box"].as<bool>();
  if (y["shrinked_box_size"]) p.shrinked_box_size = y["shrinked_box_size"].as<double>();
  if (y["map_buffer"]) p.map_buffer = y["map_buffer"].as<double>();
  if (y["center_shift_factor"]) p.center_shift_factor = y["center_shift_factor"].as<double>();
  if (y["initial_wdx"]) p.initial_wdx = y["initial_wdx"].as<double>();
  if (y["initial_wdy"]) p.initial_wdy = y["initial_wdy"].as<double>();
  if (y["initial_wdz"]) p.initial_wdz = y["initial_wdz"].as<double>();
  if (y["min_wdx"]) p.min_wdx = y["min_wdx"].as<double>();
  if (y["min_wdy"]) p.min_wdy = y["min_wdy"].as<double>();
  if (y["min_wdz"]) p.min_wdz = y["min_wdz"].as<double>();
  if (y["sando_map_res"]) p.res = y["sando_map_res"].as<double>();
  if (y["use_comm_delay_inflation"]) p.use_comm_delay_inflation = y["use_comm_delay_inflation"].as<bool>();
  if (y["comm_delay_inflation_alpha"]) p.comm_delay_inflation_alpha = y["comm_delay_inflation_alpha"].as<double>();
  if (y["comm_delay_inflation_max"]) p.comm_delay_inflation_max = y["comm_delay_inflation_max"].as<double>();
  if (y["comm_delay_filter_alpha"]) p.comm_delay_filter_alpha = y["comm_delay_filter_alpha"].as<double>();
  if (y["depth_camera_depth_max"]) p.depth_camera_depth_max = y["depth_camera_depth_max"].as<double>();
  if (y["fov_visual_depth"]) p.fov_visual_depth = y["fov_visual_depth"].as<double>();
  if (y["fov_visual_x_deg"]) p.fov_visual_x_deg = y["fov_visual_x_deg"].as<double>();
  if (y["fov_visual_y_deg"]) p.fov_visual_y_deg = y["fov_visual_y_deg"].as<double>();
  if (y["horizon"]) p.horizon = y["horizon"].as<double>();
  if (y["dc"]) p.dc = y["dc"].as<double>();
  if (y["dynamic_constraint_type"]) p.dynamic_constraint_type = y["dynamic_constraint_type"].as<std::string>();
  if (y["v_max"]) p.v_max = y["v_max"].as<double>();
  if (y["a_max"]) p.a_max = y["a_max"].as<double>();
  if (y["j_max"]) p.j_max = y["j_max"].as<double>();
  if (y["drone_bbox"]) p.drone_bbox = y["drone_bbox"].as<std::vector<double>>();
  if (y["goal_radius"]) p.goal_radius = y["goal_radius"].as<double>();
  if (y["goal_seen_radius"]) p.goal_seen_radius = y["goal_seen_radius"].as<double>();
  if (y["num_P"]) p.num_P = y["num_P"].as<int>();
  if (y["num_N"]) p.num_N = y["num_N"].as<int>();
  if (y["use_dynamic_factor"]) p.use_dynamic_factor = y["use_dynamic_factor"].as<bool>();
  if (y["dynamic_factor_k_radius"]) p.dynamic_factor_k_radius = y["dynamic_factor_k_radius"].as<double>();
  if (y["dynamic_factor_initial_mean"]) p.dynamic_factor_initial_mean = y["dynamic_factor_initial_mean"].as<double>();
  if (y["factor_initial"]) p.factor_initial = y["factor_initial"].as<double>();
  if (y["factor_final"]) p.factor_final = y["factor_final"].as<double>();
  if (y["factor_constant_step_size"]) p.factor_constant_step_size = y["factor_constant_step_size"].as<double>();
  if (y["obst_max_vel"]) p.obst_max_vel = y["obst_max_vel"].as<double>();
  if (y["obst_position_error"]) p.obst_position_error = y["obst_position_error"].as<double>();
  if (y["inflate_unknown_boundary"]) p.inflate_unknown_boundary = y["inflate_unknown_boundary"].as<bool>();
  if (y["max_gurobi_comp_time_sec"]) p.max_gurobi_comp_time_sec = y["max_gurobi_comp_time_sec"].as<double>();
  if (y["jerk_smooth_weight"]) p.jerk_smooth_weight = y["jerk_smooth_weight"].as<double>();
  if (y["traj_lifetime"]) p.traj_lifetime = y["traj_lifetime"].as<double>();
  if (y["num_replanning_before_adapt"]) p.num_replanning_before_adapt = y["num_replanning_before_adapt"].as<int>();
  if (y["default_k_value"]) p.default_k_value = y["default_k_value"].as<int>();
  if (y["alpha_k_value_filtering"]) p.alpha_k_value_filtering = y["alpha_k_value_filtering"].as<double>();
  if (y["k_value_factor"]) p.k_value_factor = y["k_value_factor"].as<double>();
  if (y["alpha_filter_dyaw"]) p.alpha_filter_dyaw = y["alpha_filter_dyaw"].as<double>();
  if (y["w_max"]) p.w_max = y["w_max"].as<double>();
  if (y["w_max_yawing"]) p.w_max_yawing = y["w_max_yawing"].as<double>();
  if (y["skip_initial_yawing"]) p.skip_initial_yawing = y["skip_initial_yawing"].as<bool>();
  if (y["yaw_spinning_threshold"]) p.yaw_spinning_threshold = y["yaw_spinning_threshold"].as<int>();
  if (y["yaw_spinning_dyaw"]) p.yaw_spinning_dyaw = y["yaw_spinning_dyaw"].as<double>();
  if (y["force_goal_z"]) p.force_goal_z = y["force_goal_z"].as<bool>();
  if (y["default_goal_z"]) p.default_goal_z = y["default_goal_z"].as<double>();
  if (y["debug_verbose"]) p.debug_verbose = y["debug_verbose"].as<bool>();
  if (y["ignore_other_trajs"]) p.ignore_other_trajs = y["ignore_other_trajs"].as<bool>();
  if (y["hover_avoidance_enabled"]) p.hover_avoidance_enabled = y["hover_avoidance_enabled"].as<bool>();
  if (y["hover_avoidance_2d"]) p.hover_avoidance_2d = y["hover_avoidance_2d"].as<bool>();
  if (y["hover_avoidance_d_trigger"]) p.hover_avoidance_d_trigger = y["hover_avoidance_d_trigger"].as<double>();
  if (y["hover_avoidance_h"]) p.hover_avoidance_h = y["hover_avoidance_h"].as<double>();
    if (p.drone_bbox.size() >= 1) p.drone_radius = p.drone_bbox[0] / 2.0;  // derived (node does this)
    return pp;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[sn_capi] yaml load failed: %s\n", e.what()); return nullptr;
  }
}
API void sn_params_destroy(void* h) { delete static_cast<Parameters*>(h); }

// override a single double field after yaml load (bridge tweaks v_max/map bounds/cruise from metaurban yaml)
API void sn_params_set_double(void* h, const char* name, double v) {
  Parameters& p = *static_cast<Parameters*>(h); std::string k = name;
#define D(f) else if (k == #f) p.f = v
  if (k == "v_max") p.v_max = v;
  D(a_max); D(j_max); D(x_min); D(x_max); D(y_min); D(y_max); D(z_min); D(z_max);
  D(goal_radius); D(drone_radius); D(horizon); D(res); D(default_goal_z);
  else std::fprintf(stderr, "[sn_capi] set_double unknown '%s'\n", name);
#undef D
}

// ---------------- SANDO lifecycle + loop API (camelCase native methods) ----------------
API void* sn_create(void* params_h) {
  try { return new SANDO(*static_cast<Parameters*>(params_h)); }
  catch (const std::exception& e) { std::fprintf(stderr, "[sn_capi] create failed: %s\n", e.what()); return nullptr; }
}
API void sn_destroy(void* h) { delete static_cast<SANDO*>(h); }

API void sn_update_state(void* h, const double* pos3, const double* vel3, const double* acc3, double yaw) {
  try {
    RobotState st;
    st.pos = Eigen::Vector3d(pos3[0], pos3[1], pos3[2]);
    if (vel3) st.vel = Eigen::Vector3d(vel3[0], vel3[1], vel3[2]);
    if (acc3) st.accel = Eigen::Vector3d(acc3[0], acc3[1], acc3[2]);
    st.yaw = yaw;
    static_cast<SANDO*>(h)->updateState(st);
  } catch (...) {}
}

API void sn_set_terminal_goal(void* h, const double* pos3) {
  try { RobotState G; G.pos = Eigen::Vector3d(pos3[0], pos3[1], pos3[2]);
    static_cast<SANDO*>(h)->setTerminalGoal(G); } catch (...) {}
}

// add_traj: native takes shared_ptr<DynTraj>; build it from analytic expr strings + bbox.
API void sn_add_traj(void* h, int id, double bx, double by, double bz,
                     const char* tx, const char* ty, const char* tz,
                     const char* vx, const char* vy, const char* vz, int is_agent, double t) {
  try {
    auto d = std::make_shared<DynTraj>();
    d->id = id; d->mode = DynTraj::Mode::Analytic;
    d->bbox = Eigen::Vector3d(bx, by, bz);
    d->traj_x = tx; d->traj_y = ty; d->traj_z = tz;
    d->traj_vx = vx ? vx : ""; d->traj_vy = vy ? vy : ""; d->traj_vz = vz ? vz : "";
    d->is_agent = (is_agent != 0); d->time_received = t;
    d->compileAnalytic();
    static_cast<SANDO*>(h)->addTraj(d, t);
  } catch (...) {}
}
API void sn_clean_old_trajs(void* h, double t) { try { static_cast<SANDO*>(h)->cleanUpOldTrajs(t); } catch (...) {} }

// occupancy: build a pcl cloud from Nx3, set ptr + process at time t.
API void sn_update_occupancy(void* h, const double* pts, int n, double t) {
  try {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(n);
    for (int i = 0; i < n; ++i)
      cloud->push_back(pcl::PointXYZ((float)pts[3*i], (float)pts[3*i+1], (float)pts[3*i+2]));
    auto* s = static_cast<SANDO*>(h);
    s->updateOccupancyMapPtr(cloud);
    s->updateOccupancyMap(t);
  } catch (...) {}
}

// replan returns tuple<bool,bool>(success, attempted) -> bit0|bit1
API int sn_replan(void* h, double last_rt, double t) {
  try { auto r = static_cast<SANDO*>(h)->replan(last_rt, t);
    return (std::get<0>(r) ? 1 : 0) | (std::get<1>(r) ? 2 : 0);
  } catch (...) { return 0; }
}

// out9 = [pos(3), vel(3), accel(3)]; 1 if a setpoint is available
API int sn_get_next_goal(void* h, double* out9) {
  try {
    RobotState ng;
    if (!static_cast<SANDO*>(h)->getNextGoal(ng)) return 0;
    for (int i = 0; i < 3; ++i) { out9[i] = ng.pos[i]; out9[3+i] = ng.vel[i]; out9[6+i] = ng.accel[i]; }
    return 1;
  } catch (...) { return 0; }
}

API int sn_get_drone_status(void* h) { try { return static_cast<SANDO*>(h)->getDroneStatus(); } catch (...) { return 0; } }

API int sn_get_global_path(void* h, double* out, int max_pts) {
  try {
    vec_Vecf<3> gp; static_cast<SANDO*>(h)->getGlobalPath(gp);
    int n = (int)gp.size(); if (n > max_pts) n = max_pts;
    for (int i = 0; i < n; ++i) { out[3*i] = gp[i][0]; out[3*i+1] = gp[i][1]; out[3*i+2] = gp[i][2]; }
    return n;
  } catch (...) { return 0; }
}

// committed trajectory setpoints (for drawing the chosen path)
API int sn_get_setpoints(void* h, double* out, int max_pts) {
  try {
    std::vector<RobotState> sp; static_cast<SANDO*>(h)->retrieveGoalSetpoints(sp);
    int n = (int)sp.size(); if (n > max_pts) n = max_pts;
    for (int i = 0; i < n; ++i) { out[3*i] = sp[i].pos[0]; out[3*i+1] = sp[i].pos[1]; out[3*i+2] = sp[i].pos[2]; }
    return n;
  } catch (...) { return 0; }
}
