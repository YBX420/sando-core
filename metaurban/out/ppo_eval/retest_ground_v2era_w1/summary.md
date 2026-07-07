# shield ON/OFF campaign  (2026-07-07 17:20)  model=out/ppo_planner_ground  n=100/arm  seed=8777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 93/100  (93.0%)
collided: 4/100  (4.00%)   Wilson95 [1.57%, 9.84%]
timeout : 3/100
min_clr : median 8.21  p10 0.56  min -1.46 m
attribution: in_track=4  hole_in_cone=0  hole_blind=0  by_cls={'vehicle': 4}  med_speed=8.33
shield  : {'ticks': 9901, 'passed': 9666, 'projected': 43, 'brake': 192, 'uncert_frac': 0.0194}

## arm: bare (no shield)   (n=100)
reached : 89/100  (89.0%)
collided: 9/100  (9.00%)   Wilson95 [4.81%, 16.23%]
timeout : 2/100
min_clr : median 7.52  p10 0.43  min -1.50 m
attribution: in_track=8  hole_in_cone=0  hole_blind=1  by_cls={'static': 3, 'vehicle': 6}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.2507
