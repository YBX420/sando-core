---
name: sando-core-zero-collision-2026-06
description: Drive-to-zero-collision (2026-06-27) — 100-scene benchmark reached 0 person/animal/static crashes, 1 residual vehicle
metadata:
  type: project
---

2026-06-27: drove the 100-scene MetaUrban benchmark (real quad dynamics, crash=stop, IDENTICAL perception both sides) toward the user's goal "0撞0坠" (0 collision 0 crash). **Final: ours 安全到达 93/100, 撞人 0, 撞车 1, 撞动物 0, 坠机 0 (1 total collision) vs native EGO 69/100, 3+3+1 mover + 22 static = 29.** Same perception for both → the only variable is the safety layer.

**The fixes that got there (each found by diagnosing the actual collision seeds), in order of impact:**
1. **Static gate horizon** `MAN_STATIC_HZ` 0.75→**1.20 s** (`flown_samples`): the drone reacts to static sooner so inertia doesn't carry it in. Broad win: mover 3→1, static 9→8, reach 89.
2. **Known static MAP for BOTH** (`EGO_STATIC_MAP=1`, default ON): `_local_static` (360° within EGO_HOR+4) replaces forward-cone for STATIC only; movers stay forward-cone. Realistic (a drone has a static map / doesn't forget walls) + fair (both get it). Fixed the cone-blind side-static grazes. static 8→4, reach 93. KEY: forward-cone-only static makes 0-crash physically impossible (side-blind).
3. **Cylinder-SDF static gate** (`static_clear` rewrite + `MAN_STATIC_BUF`): gate the FLOWN path against static obstacle CYLINDERS (same SDF as GT clearance()) instead of sparse cloud points → catches grazes of large buildings the cloud-point gate missed; the drone HOLDs rather than grazes. **static 4→0.** BUG fixed: `loc_obs` must filter by SURFACE distance (center_dist − radius), NOT center distance — a 40 m building's centre is 20 m away while its wall is beside the drone (seed-55 graze).
4. **Mover forward-sim gate** (`mover_clear_flown`): the analytic `cert_clear` has NO forward-sim, so a fast climb-over / tracking overshoot grazed a tall mover's roof. Forward-sim the flown path vs KF-predicted mover cylinders (horizontal OR vertical disjunction). Fixed seed 15 (animal) + 18 (ped) → 撞人 0, 撞动物 0.
   (Plus the earlier code-review fixes still in: climb→HOLD soundness, live commit hysteresis, δ_track per-flight 0.45, DNF transparency. See [[sando-core-hctd-tracking-tube-2026-06]].)

**The 1 residual = seed 56 (vehicle).** A ~7 m/s drone approaches a 2.8 m-tall van detected late by the forward cone (movers stay cone); momentum needs ~4 m to brake but the van is ~1.3 m away → grazes the roof (−0.11). The ONLY thing that fixes it is the dynamic speed-FOV cap (`EGO_FOVCAP=1`) or a global v_max drop — both confirmed to **kill ~10 reaches** (the warp over-slows / never reaches; v_max=5 doesn't even cap the actual flown speed which the B-spline drives to 7). Not worth trading 10 reach for 1 vehicle collision. = the FUNDAMENTAL "can't stop in time + cone-limited mover" limit; literal 0/100 needs wider mover sensing or an RTA braking-distance invariant (a different, bigger method).

**Fundamental finding (the user asked for it):** 0-collision + forward-cone-only + cruise-speed is OVER-CONSTRAINED (a trilemma). The certificate only protects PERCEIVED+CERTIFIED obstacles; relax one axis to get 0. We relaxed STATIC→known map (legit, both) → 0 crash; movers stay cone → the 1 fast-van residual. Also: **stopping ≠ safe** (a held drone can be walked into by a mover — frozen-robot), so HOLD is not a mover guarantee; and tuning churns collisions around (~plateau) rather than eliminating, because residuals are at the reactive scheme's resolution limit.

Config (env, defaults baked in): EGO_STATIC_MAP=1, EGO_STATICM=0.70, EGO_STATIC_BUF=0.45, EGO_STATIC_HZ=1.20, EGO_TRACK=0.45, EGO_FOVCAP=0. Figure `out/conformal/fig_collision_final.png`. Doc `docs/algorithm-comparisons-2026-06.md` §2 + `docs/code-review-2026-06-27.md`.
