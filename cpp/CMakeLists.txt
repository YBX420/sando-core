cmake_minimum_required(VERSION 3.16)
project(sando_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Header-only Eigen (vendored) + our headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

enable_testing()

# ---- module tests (golden-verified against Python) ----
add_executable(test_minjerk test/test_minjerk.cpp)
add_test(NAME minjerk
         COMMAND test_minjerk ${CMAKE_CURRENT_SOURCE_DIR}/golden/minco_cases.txt)

add_executable(test_obstacles test/test_obstacles.cpp)
add_test(NAME obstacles
         COMMAND test_obstacles ${CMAKE_CURRENT_SOURCE_DIR}/golden/obstacles_cases.txt)

add_executable(test_cost test/test_cost.cpp)
add_test(NAME cost COMMAND test_cost ${CMAKE_CURRENT_SOURCE_DIR}/golden/cost_cases.txt)

add_executable(test_localopt test/test_localopt.cpp)
add_test(NAME localopt COMMAND test_localopt ${CMAKE_CURRENT_SOURCE_DIR}/golden/localopt_cases.txt)

add_executable(test_types test/test_types.cpp)
add_test(NAME types COMMAND test_types ${CMAKE_CURRENT_SOURCE_DIR}/golden/types_cases.txt)

add_executable(test_voxel_map test/test_voxel_map.cpp)
add_test(NAME voxel_map COMMAND test_voxel_map ${CMAKE_CURRENT_SOURCE_DIR}/golden/voxel_map_cases.txt)

add_executable(test_local_opt_terms test/test_local_opt_terms.cpp)
add_test(NAME local_opt_terms COMMAND test_local_opt_terms ${CMAKE_CURRENT_SOURCE_DIR}/golden/local_opt_terms_cases.txt)

add_executable(test_voxel_map_helpers test/test_voxel_map_helpers.cpp)
add_test(NAME voxel_map_helpers COMMAND test_voxel_map_helpers ${CMAKE_CURRENT_SOURCE_DIR}/golden/voxel_map_helpers_cases.txt)

add_executable(test_graph_search test/test_graph_search.cpp)
add_test(NAME graph_search COMMAND test_graph_search ${CMAKE_CURRENT_SOURCE_DIR}/golden/graph_search_cases.txt)

add_executable(test_local_opt_hardalm test/test_local_opt_hardalm.cpp)
add_test(NAME local_opt_hardalm COMMAND test_local_opt_hardalm ${CMAKE_CURRENT_SOURCE_DIR}/golden/local_opt_hardalm_cases.txt)

add_executable(test_minco_cost_grad test/test_minco_cost_grad.cpp)
add_test(NAME minco_cost_grad COMMAND test_minco_cost_grad ${CMAKE_CURRENT_SOURCE_DIR}/golden/minco_cost_grad_cases.txt)

add_executable(test_hgp_planner test/test_hgp_planner.cpp)
add_test(NAME hgp_planner COMMAND test_hgp_planner ${CMAKE_CURRENT_SOURCE_DIR}/golden/hgp_planner_cases.txt)

add_executable(test_plan_minco test/test_plan_minco.cpp)
add_test(NAME plan_minco COMMAND test_plan_minco ${CMAKE_CURRENT_SOURCE_DIR}/golden/plan_minco_cases.txt)

add_executable(test_hgp_manager test/test_hgp_manager.cpp)
add_test(NAME hgp_manager COMMAND test_hgp_manager ${CMAKE_CURRENT_SOURCE_DIR}/golden/hgp_manager_cases.txt)

add_executable(test_planner test/test_planner.cpp)
add_test(NAME planner COMMAND test_planner ${CMAKE_CURRENT_SOURCE_DIR}/golden/planner_cases.txt)

add_executable(test_bspline test/test_bspline.cpp)
add_test(NAME bspline COMMAND test_bspline ${CMAKE_CURRENT_SOURCE_DIR}/golden/bspline_cases.txt)

add_executable(test_utils_math test/test_utils_math.cpp)
add_test(NAME utils_math COMMAND test_utils_math ${CMAKE_CURRENT_SOURCE_DIR}/golden/utils_math_cases.txt)

add_executable(test_basis_converter test/test_basis_converter.cpp)
add_test(NAME basis_converter COMMAND test_basis_converter ${CMAKE_CURRENT_SOURCE_DIR}/golden/basis_converter_cases.txt)

add_executable(test_obstacle_tracker test/test_obstacle_tracker.cpp)
add_test(NAME obstacle_tracker COMMAND test_obstacle_tracker ${CMAKE_CURRENT_SOURCE_DIR}/golden/obstacle_tracker_cases.txt)

# DynTraj conformal label-set primitive (human_in_set truth table) — self-contained, no golden file.
add_executable(test_dyntraj_labelset test/test_dyntraj_labelset.cpp)
add_test(NAME dyntraj_labelset COMMAND test_dyntraj_labelset)

# S3 exact continuous-time conformal-Bernstein deficit certificate (bernstein_cert.hpp) —
# self-contained: cross-checks the certificate vs dense brute-force, asserts no false-certify.
add_executable(test_bernstein_cert test/test_bernstein_cert.cpp)
add_test(NAME bernstein_cert COMMAND test_bernstein_cert)

# S3 deficit certificate wired as the mover GATE (opt.minco_deficit_cert) — OFF vs ON integration:
# real-time, no liveness regression on easy cases, and ON-valid trajectories are cert-clean.
add_executable(test_deficit_gate test/test_deficit_gate.cpp)
add_test(NAME deficit_gate COMMAND test_deficit_gate)

# Smooth recovery brake (minco_recovery_smooth_brake) — closed-loop on the real SANDO::recovery_yield:
# head-on falls back to instant stop (safe), lateral/forward-open smooths (jerk 2.5->~0), never regresses.
add_executable(test_recovery_closedloop test/test_recovery_closedloop.cpp)
add_test(NAME recovery_closedloop COMMAND test_recovery_closedloop)
