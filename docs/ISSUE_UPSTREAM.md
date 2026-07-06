# Upstream issue draft: EGO-Planner BsplineOptimizer UB (use-of-uninitialized + inconsistent flag/base_point)

**Repo**: ZJU-FAST-Lab/ego-planner (rebound optimizer, both check_collision_and_rebound & initControlPoints)
**Found**: 2026-07-02, deterministic SIGSEGV on a crossing-pedestrians scene; root-caused with ASan.

1. `got_intersection_id` carries a STALE value across the `for j` loop: when iteration j finds no
   intersection, the subsequent `if (got_intersection_id >= 0)` block reads an `intersection_point`
   that was never written THIS iteration (garbage length / NaN) — use of uninitialized memory.
2. `flag_temp[j] = true` is set BEFORE the `length > 1e-5` check: a degenerate intersection marks the
   segment as having a base point while pushing nothing — later `.back()` on the empty vector in the
   step-3 chain copy is heap UB (the observed crash site).
Fix (upstreamable): per-iteration `fresh_intersection` flag; bind `flag_temp` atomically to the push;
guard step-3 empty-neighbor copies. Zero behavior change on healthy paths (bit-identical outputs on
our regression scenes); ASan clean afterwards.
Patch: sando-core `ego/src/bspline_optimizer.cpp` (both code paths), commit-ready.
