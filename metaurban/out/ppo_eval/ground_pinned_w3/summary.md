# shield ON/OFF campaign  (2026-07-08 10:22)  model=out/ppo_planner_ground  n=100/arm  seed=10777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 95/100  (95.0%)
collided: 4/100  (4.00%)   Wilson95 [1.57%, 9.84%]
timeout : 1/100
min_clr : median 7.93  p10 0.45  min -2.24 m
attribution: in_track=3  hole_in_cone=0  hole_blind=1  by_cls={'vehicle': 4}  med_speed=8.33
shield  : {'ticks': 10319, 'passed': 9923, 'projected': 67, 'brake': 329, 'uncert_frac': 0.0319}

## arm: bare (no shield)   (n=100)
reached : 90/100  (90.0%)
collided: 10/100  (10.00%)   Wilson95 [5.52%, 17.44%]
timeout : 0/100
min_clr : median 7.94  p10 0.07  min -1.44 m
attribution: in_track=9  hole_in_cone=0  hole_blind=1  by_cls={'static': 7, 'vehicle': 3}  med_speed=0.0

Fisher exact (collision, shielded vs bare): p = 0.1640
