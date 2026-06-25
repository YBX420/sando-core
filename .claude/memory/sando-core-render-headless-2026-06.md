---
name: sando-core-render-headless-2026-06
description: --headless makes render+headless one code path; honest 100-scenario MetaUrban A/B reveals ours's real (modest) edge
metadata:
  type: project
---

2026-06-25: resolved the long-standing "headless looks good, render looks bad" discrepancy AND got an honest
benchmark out of it.

**Why render != headless (diagnosed, NOT what was assumed):** not PX4, not dynamics (real-quad dynamics actually
makes native ego collide LESS: 11%->3% — inertia stops it cutting corners). The two real causes: (1) the RENDER
has a full 3-D STATIC scene (buildings/curbs, "static field 136762 voxels") that the harvested-GT headless harness
(replay_core / planner_safety_matrix) completely lacks -> EGO's A* hits infeasible/timeout situations the clean
corridors never do; (2) param/logic drift between two safety-layer codebases (now partly unified, see
[[safety_layer]] commit 292e782 — but the ESCAPE legitimately differs: render needs climb, headless evade, because
fleeing into static crashes the drone).

**The fix that mattered — `render_3d_video.py --headless`:** skip grab_views/compose/draw overlays + build the env
without a camera. SAME MetaUrban sim + planner + safety + REAL-QUAD dynamics, no pixels. seed 3 lap-done is
BYTE-IDENTICAL to the rendered run (11.1s, 0-collision, clr 0.42, climb4/over8/straight99) in ~34s vs ~3-4min, and
CPU-only so it parallelises ~10x. **This makes render and the headless metric ONE code path (render optional) =
consistency by construction**, and it inherently runs the real quad dynamics the user wanted.

**Honest 100-scenario MetaUrban A/B (`render_screen.py`, seeds 0-99 ours vs native, --headless, P=10, ~15 min;
`out/conformal/render_screen.csv`):**
- ours: reach 70/100, collide **26**/100 ; native EGO: reach 98/100, collide 29/100.
- head-to-head: both-safe 59, **ours-wins (only native collides) 15**, **ours-loses (only ours collides) 12**,
  both-collide 14. ours is only MARGINALLY safer (26 vs 29) and reaches MUCH less (70 vs 98, more conservative).
- of ours's 26 collisions only 1 is A*-blocked (astar>500); the other 25 ours PLANS fine yet still collides
  (maneuver flies into static / doesn't clear) -> a real algorithmic gap, NOT just A* timeout.

**STATIC FIX (the ours-loses solve, big win):** ALL 12 ours-loses collided with STATIC (the cert guards only
movers; `fov_cloud` was FOV-CONE-limited so it hid buildings beside/behind the drone). Fix in
`ego_maneuver_replan` (render_3d_video.py): (1) feed EGO the full 360-degree local static map (STATIC_CLOUD within
EGO_HOR+4, not the cone — static is a KNOWN map, a real drone has it; only movers stay perception-limited);
(2) add around-L/R +-25/50 ground-weave candidates (the render's tournament was only straight/over/climb);
(3) a STATIC CLEARANCE GATE: reject any candidate whose committed B-spline comes within MAN_STATIC_MARGIN=0.55
(drone radius + tracking standoff) of the local static cloud (16 samples over [0,TAU], min 3-D dist).
**Result on the same 100 MetaUrban scenarios: ours reach 70->95, collide 26->10; native unchanged 98/29;
head-to-head ours-wins 15->24, ours-loses 12->5.** So ours now CLEARLY beats native EGO (10 vs 29 collisions,
95 vs 98 reach). The 5 residual losses [15,33,46,50,83] are mostly SHALLOW static grazes (-0.004 to -0.14, only
33 is -0.37) = quad TRACKING OVERSHOOT past the planned clearance; a larger margin or flown-path gate would catch
them but risks reach. Commit after the b3b85ed honest baseline.

**Key takeaway / honesty correction:** the earlier harvested-GT-harness "ours 0 collisions" (planner_safety_matrix,
compare3) was OPTIMISTIC — no static scene. On the realistic MetaUrban scenario ours started at 26% collisions; the
static gate brought it to 10% and a clear win over native. Tools: `render_screen.py` (ThreadPool concurrency cap;
bash wait/jobs/xargs left un-killable orphans in nohup; the driver itself needs python on PATH -> activate conda
BEFORE nohup). Related: [[sando-core-d435i-fix-2026-06]] [[sando-core-planner-safety-matrix-2026-06]] [[sando-core-win-ego-2026-06]].
