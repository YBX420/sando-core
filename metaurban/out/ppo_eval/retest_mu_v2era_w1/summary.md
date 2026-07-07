# shield ON/OFF campaign  (2026-07-07 16:35)  model=out/ppo_planner_mu  n=100/arm  seed=8777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 91/100  (91.0%)
collided: 6/100  (6.00%)   Wilson95 [2.78%, 12.48%]
timeout : 3/100
min_clr : median 9.06  p10 0.46  min -1.16 m
attribution: in_track=2  hole_in_cone=0  hole_blind=4  by_cls={'vehicle': 6}  med_speed=8.33
shield  : {'ticks': 7259, 'passed': 7021, 'projected': 46, 'brake': 192, 'uncert_frac': 0.0264}

## arm: bare (no shield)   (n=100)
reached : 80/100  (80.0%)
collided: 16/100  (16.00%)   Wilson95 [10.10%, 24.42%]
timeout : 4/100
min_clr : median 8.58  p10 -0.12  min -2.08 m
attribution: in_track=12  hole_in_cone=0  hole_blind=4  by_cls={'static': 8, 'vehicle': 8}  med_speed=0.015

Fisher exact (collision, shielded vs bare): p = 0.0400
