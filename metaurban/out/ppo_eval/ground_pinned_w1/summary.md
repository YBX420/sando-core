# shield ON/OFF campaign  (2026-07-08 10:20)  model=out/ppo_planner_ground  n=100/arm  seed=8777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 89/100  (89.0%)
collided: 9/100  (9.00%)   Wilson95 [4.81%, 16.23%]
timeout : 2/100
min_clr : median 7.47  p10 0.36  min -1.32 m
attribution: in_track=5  hole_in_cone=2  hole_blind=2  by_cls={'vehicle': 9}  med_speed=8.33
shield  : {'ticks': 9893, 'passed': 9739, 'projected': 65, 'brake': 89, 'uncert_frac': 0.009}

## arm: bare (no shield)   (n=100)
reached : 83/100  (83.0%)
collided: 16/100  (16.00%)   Wilson95 [10.10%, 24.42%]
timeout : 1/100
min_clr : median 4.61  p10 -0.32  min -1.58 m
attribution: in_track=15  hole_in_cone=0  hole_blind=1  by_cls={'static': 8, 'vehicle': 8}  med_speed=2.935

Fisher exact (collision, shielded vs bare): p = 0.1989
