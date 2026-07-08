# shield ON/OFF campaign  (2026-07-08 10:20)  model=out/ppo_planner_ground  n=100/arm  seed=9777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 91/100  (91.0%)
collided: 8/100  (8.00%)   Wilson95 [4.11%, 15.00%]
timeout : 1/100
min_clr : median 7.35  p10 0.40  min -1.56 m
attribution: in_track=8  hole_in_cone=0  hole_blind=0  by_cls={'vehicle': 8}  med_speed=8.33
shield  : {'ticks': 9908, 'passed': 9489, 'projected': 86, 'brake': 333, 'uncert_frac': 0.0336}

## arm: bare (no shield)   (n=100)
reached : 83/100  (83.0%)
collided: 17/100  (17.00%)   Wilson95 [10.89%, 25.55%]
timeout : 0/100
min_clr : median 6.68  p10 -0.29  min -1.42 m
attribution: in_track=15  hole_in_cone=0  hole_blind=2  by_cls={'static': 9, 'vehicle': 8}  med_speed=0.0

Fisher exact (collision, shielded vs bare): p = 0.0856
