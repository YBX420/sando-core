---
name: sando-core-fov-audit-2026-06
description: The "FOV go-around fails / worse than 360" symptom is mostly a measurement artifact (unfair A/B + a diagnostic bug), not realized collisions; 11 root causes + 5-step fix order
metadata:
  type: project
---

**2026-06-30 multi-agent audit of `render_3d_video.py --maneuver` (the ONLY FOV path; `ego_maneuver.py` headless harness has NO cone = 360°/12m and CANNOT exhibit the FOV problem).**

Empirical, headless, 14 seeds, SHIPPED DEFAULT (`EGO_STATIC_MAP=1`): **0/14 collisions, pedestrian clearance always >1.1m.** So "FOV worse than 360" is **latent, not a realized collision gap.** It comes from three non-collision sources:
- **Unfair A/B on the dynamic channel:** `feed_native` feeds native every mover within 30m as an exact-GT analytic DynTraj (no cone); ours' `kf_movers` gates the SAME movers by the 8m/±45° cone on NOISY detections (`KF_MEAS_NOISE=0.10`). Native is omniscient+noise-free on movers, ours is FOV+noise-limited. The in-code comment claiming both see the same cone is FALSE for movers.
- **Diagnostic bug (`colldbg-masks-cone-blindness`):** on a collision, COLLDBG re-queries `kf_movers` with the 30m OMNISCIENT sensor → cone-blind collisions get mislabeled as benign small KF error → the FOV root cause is systematically under-reported.
- The only **realized** collision: `EGO_STATIC_MAP=0` + seed5 → drone at 4.6m/s outran its 8m cone into an unseen **STATIC tree** (clr=-0.003m), NOT a mover. The 360 known-static map (`EGO_STATIC_MAP=1`) is the legit crutch carrying 0-collision — KEEP IT ON (forward-cone-only statics make side-grazes physically unavoidable).

**11 root causes; 5-step fix order** (each has a ship-it eng version): (1) symmetric `VMAX_EFF=min(v_base, √(2a(fov_range-1.2)))` fed to BOTH ours+native (native ignores `EGO_VMAX` today = an ours-only hole) + cone = true D435 87°; (2) yaw-to-path via `yaw_ref` (do NOT overwrite `cam_heading` — keep the honest bolted cam), mirror into `feed_native` + fix the COLLDBG requery; (3) anti-thrash commit-lock = the [[sando-core-fov-novelty-2026-06]] CCF; (4) harden the already-merged track memory (TTL taper, non-mutating no-cone requeries); (5) offline conformal age-bucket coverage table.

**Why:** 塔菲大人 worried we should drop the 360 panoramic map / switch to D435; the audit shows the real issues are A/B fairness + camera aim + tournament thrash, NOT the sensor model — and that strict forward-cone statics make side-grazes physically unavoidable, so dropping the known-static map is exactly wrong.
**How to apply:** DO NOT set `EGO_STATIC_MAP=0`; DO NOT flip `EGO_FOVCAP` default this pass; DO NOT treat HOLD as a safe backstop (hover-at-current-pos overshoots ~2-3m from cruise); DO NOT widen the cone to fix the headline; DO NOT cite default 0/14 or `ego_maneuver.py` as proof "FOV is fine" (category error). Line numbers in all fix designs are drifted 30-180 lines — locate every edit by code CONTENT. Verification needs a cone-EXPOSED matched A/B (`--fov_deg 30`, NOT disabling the static map) + an adversarial fast side-crosser scenario (default denominator is 0). **Decided 2026-06-30: A/B = cone-aligned native + `predict=False` ablation (double control); headline = CCF.**
