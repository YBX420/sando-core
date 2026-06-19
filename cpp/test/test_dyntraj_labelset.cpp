// Unit test: DynTraj conformal label-set primitives — human_in_set() truth table AND the full
// hard/soft gating decision derived_class() (label-set primary; empty -> legacy id heuristic).
// Pure logic, no golden file. The ROS2 end-to-end path (DynTraj.msg -> sando_node -> planner ->
// obst_class_codes readback) is covered by cpp/ros2/test_labelset_ros2.py.
// Mondrian class codes: 0=HUMAN, 1=VEHICLE_LIKE, 2=OTHER.
#include "sando_cpp/types.hpp"
#include <cstdio>
#include <string>
#include <vector>

using sando::DynTraj;

static int fails = 0;
static void check(const char* name, bool got, bool want) {
  bool ok = (got == want);
  if (!ok) ++fails;
  std::printf("  %-34s got=%d want=%d  %s\n", name, (int)got, (int)want, ok ? "PASS" : "FAIL");
}
static void check_cls(const char* name, const std::string& got, const char* want) {
  bool ok = (got == want);
  if (!ok) ++fails;
  std::printf("  %-34s got=%-6s want=%-6s %s\n", name, got.c_str(), want, ok ? "PASS" : "FAIL");
}

static DynTraj mk(int id, const std::vector<int>& labels) {
  DynTraj t; t.id = id; t.label_set = labels; return t;
}

int main() {
  // --- human_in_set() truth table ---
  DynTraj t;
  check("empty -> no human",  t.human_in_set(), false);
  t.label_set = {0};           check("[0] -> human",        t.human_in_set(), true);
  t.label_set = {1};           check("[1] -> no human",     t.human_in_set(), false);
  t.label_set = {2};           check("[2] -> no human",     t.human_in_set(), false);
  t.label_set = {1, 2};        check("[1,2] -> no human",   t.human_in_set(), false);
  t.label_set = {1, 0, 2};     check("[1,0,2] -> human",    t.human_in_set(), true);
  t.label_set = {2, 0};        check("[2,0] -> human",      t.human_in_set(), true);
  t.label_set = {0, 0};        check("[0,0] -> human",      t.human_in_set(), true);

  // copy semantics: label_set survives a DynTraj copy (planner snapshot does `DynTraj t = traj;`)
  DynTraj a = mk(7, {0, 2});
  DynTraj b = a;
  check("copied [0,2] -> human", b.human_in_set(), true);

  // --- derived_class(): the shipped per-class gating decision (4-class conformal refinement) ---
  // label-set primary (overrides id heuristic, both directions). human(0) dominates; non-human maps
  // OTHER(2)->animal, VEHICLE_LIKE(1)->vehicle, failing toward the larger clearance (animal) when ambiguous.
  check_cls("id=5  [1]   -> vehicle(hard)", mk(5,   {1}).derived_class(),    "vehicle"); // overrides id<200
  check_cls("id=250 [0]  -> human(hard)",   mk(250, {0}).derived_class(),    "human");   // overrides id>=200
  check_cls("id=5  [2]   -> animal(hard)",  mk(5,   {2}).derived_class(),    "animal");
  check_cls("id=250 [2,0]-> human(hard)",   mk(250, {2, 0}).derived_class(), "human");   // human dominates
  check_cls("id=5  [1,2] -> animal(hard)",  mk(5,   {1, 2}).derived_class(), "animal");  // ambiguous -> larger clearance
  // empty set -> legacy id heuristic, including the deliberate open-ended id>=200 (boundaries + >=300)
  check_cls("id=5   [] -> human(hard)", mk(5,   {}).derived_class(),  "human");
  check_cls("id=199 [] -> human(hard)", mk(199, {}).derived_class(),  "human");  // boundary just below
  check_cls("id=200 [] -> wall(soft)",  mk(200, {}).derived_class(),  "wall");   // boundary at 200
  check_cls("id=300 [] -> wall(soft)",  mk(300, {}).derived_class(),  "wall");   // >=300 stays SOFT
  check_cls("id=350 [] -> wall(soft)",  mk(350, {}).derived_class(),  "wall");   // (regression to [200,300) would FAIL)

  std::printf("\ndyntraj_labelset: %d fail\n", fails);
  std::printf("%s\n", fails == 0 ? "ALL PASS" : "FAILED");
  return fails == 0 ? 0 : 1;
}
