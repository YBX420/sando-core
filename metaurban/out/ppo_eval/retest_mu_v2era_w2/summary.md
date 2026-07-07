# shield ON/OFF campaign  (2026-07-07 16:34)  model=out/ppo_planner_mu  n=100/arm  seed=9777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 88/100  (88.0%)
collided: 11/100  (11.00%)   Wilson95 [6.25%, 18.63%]
timeout : 1/100
min_clr : median 8.80  p10 -0.02  min -1.99 m
attribution: in_track=2  hole_in_cone=1  hole_blind=8  by_cls={'vehicle': 11}  med_speed=8.33
shield  : {'ticks': 6655, 'passed': 6522, 'projected': 56, 'brake': 77, 'uncert_frac': 0.0116}

## arm: bare (no shield)   (n=100)
reached : 82/100  (82.0%)
collided: 17/100  (17.00%)   Wilson95 [10.89%, 25.55%]
timeout : 1/100
min_clr : median 8.60  p10 -0.18  min -0.86 m
attribution: in_track=9  hole_in_cone=0  hole_blind=8  by_cls={'static': 9, 'vehicle': 8}  med_speed=0.0

Fisher exact (collision, shielded vs bare): p = 0.3083
