---
name: sando-core-planner-safety-matrix-2026-06
description: Headless planner-agnostic A/B — 4 planners with vs without our safety layer; safety=on is 0-collision for all
metadata:
  type: project
---

2026-06-25: built `metaurban/planner_safety_matrix.py` — a headless (pure CPU, NO d435i, NO render) A/B that flies
SEVERAL planners through the SAME harvested-GT crossing corridors (replay_core episodes, 20 seeds) WITH vs WITHOUT
our certified safety layer. Demonstrates the layer is **planner-agnostic**: the continuous-time Bernstein cylinder
cert + conformal keep-out + fastest-safe maneuver tournament gates whatever trajectory the planner commits.

**Planners (each its own representation, ONE cert):** `ego` (ZJU EGO-Planner, cubic B-spline + ESDF avoidance —
the real closed loop; for ego the cert uses ego.certify_*), `quintic` (RapidQuad min-jerk quintic to goal, no
avoidance), `septic` (min-snap deg-7), `bspline` (plain cubic B-spline) — the non-EGO three have NO avoidance
front-end, so with safety=off they fly the smooth arc straight through movers; the cert is the ONLY thing keeping
them safe (cleanest planner-agnostic claim). Graft cert via `cert_bridge`. Safety=on = tournament (straight /
around ±25/50 / over / climb) + cert-gate + **evade fallback** (flee nearest mover; never freeze).

**Result (speeds {2,3,4} m/s × 20 seeds × 6 ep = 360/cell, point-mass; `out/conformal/planner_safety.csv`, 2880 rows):**
```
planner safety  n    collide     reach    clr_med
ego     off    360  44 (12%)    99%       0.90
ego     on     360   0 ( 0%)   100%       1.46
quintic off    360  33 ( 9%)   100%       0.52
quintic on     360   0 ( 0%)   100%       1.78
septic  off    360  45 (12%)   100%       0.34
septic  on     360   0 ( 0%)    68%       1.66
bspline off    360  24 ( 7%)   100%       1.82
bspline on     360   0 ( 0%)   100%       2.01
```
**Every safety=on cell is 0-collision** at every speed (SANITY PASS) and clearance rises to 1.5–2.0 m; every
safety=off planner collides 7–12%. So the safety layer makes ANY planner collision-free.

**Honest caveats:** (1) `septic on` reach drops to 68% — entirely the vmax=2.0 stall (15/120 reach at 2.0 vs
91%/98% at 3.0/4.0): a weak min-snap-from-rest planner can't out-pace the crossing at low speed, so the
conservative layer HOLDs/evades rather than risk it (0 collisions bought partly by not completing). (2) Point-mass
(dynamics=off, optimistic "planned" path) — a `--dynamics` flag runs real quad tracking but wasn't swept here.
(3) **A first run without the evade fallback had ego/on collide 19/480** — a stationary HOLD lets a mover walk into
the drone; adding the flee-nearest evade (mirroring replay_core) fixed it to 0. (4) These are DESIGNED contested
corridors (build_episodes times the drone onto the anchor crosser), not free flight.

**Run:** `bash metaurban/psm_run.sh 2,3,4 0-19 6` (8 cells parallel, fresh process each → no CSV append race + EGO
grid-leak isolation, then merges + prints the table). Single cell: `python metaurban/planner_safety_matrix.py
--planner ego --safety on --seeds 0-19 --n_ep 6 --speeds 2,3,4 --csv ...`. Needs conda env `sando` (ego_capi +
cert_capi, NO GUROBI/metaurban). Built with a Workflow (8 parallel cells + an adversarial verify agent that caught
the missing-evade bug). SANDO not included (different GUROBI pipeline; see compare3.py). Related:
[[sando-core-conformal-2026-06]] [[sando-core-planner-agnostic-plan.md]] [[sando-core-d435i-fix-2026-06]].
