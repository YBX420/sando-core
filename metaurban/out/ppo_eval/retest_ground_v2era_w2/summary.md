# shield ON/OFF campaign  (2026-07-07 17:18)  model=out/ppo_planner_ground  n=100/arm  seed=9777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 88/100  (88.0%)
collided: 11/100  (11.00%)   Wilson95 [6.25%, 18.63%]
timeout : 1/100
min_clr : median 5.71  p10 -0.04  min -1.61 m
attribution: in_track=9  hole_in_cone=1  hole_blind=1  by_cls={'vehicle': 11}  med_speed=8.33
shield  : {'ticks': 9184, 'passed': 8902, 'projected': 45, 'brake': 237, 'uncert_frac': 0.0258}

## arm: bare (no shield)   (n=100)
reached : 82/100  (82.0%)
collided: 16/100  (16.00%)   Wilson95 [10.10%, 24.42%]
timeout : 2/100
min_clr : median 7.38  p10 -0.36  min -1.66 m
attribution: in_track=10  hole_in_cone=1  hole_blind=5  by_cls={'static': 3, 'vehicle': 13}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.4083
