# shield ON/OFF campaign  (2026-07-08 10:20)  model=out/ppo_planner_ground  n=100/arm  seed=7777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 93/100  (93.0%)
collided: 6/100  (6.00%)   Wilson95 [2.78%, 12.48%]
timeout : 1/100
min_clr : median 8.78  p10 0.54  min -1.55 m
attribution: in_track=5  hole_in_cone=0  hole_blind=1  by_cls={'vehicle': 6}  med_speed=8.33
shield  : {'ticks': 9847, 'passed': 9724, 'projected': 64, 'brake': 59, 'uncert_frac': 0.006}

## arm: bare (no shield)   (n=100)
reached : 88/100  (88.0%)
collided: 11/100  (11.00%)   Wilson95 [6.25%, 18.63%]
timeout : 1/100
min_clr : median 8.98  p10 -0.08  min -1.06 m
attribution: in_track=9  hole_in_cone=1  hole_blind=1  by_cls={'static': 5, 'vehicle': 6}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.3106
