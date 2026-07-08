# shield ON/OFF campaign  (2026-07-08 09:36)  model=out/ppo_planner_mu  n=100/arm  seed=10777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 83/100  (83.0%)
collided: 11/100  (11.00%)   Wilson95 [6.25%, 18.63%]
timeout : 6/100
min_clr : median 7.69  p10 -0.29  min -0.92 m
attribution: in_track=3  hole_in_cone=1  hole_blind=7  by_cls={'vehicle': 11}  med_speed=8.33
shield  : {'ticks': 8079, 'passed': 7974, 'projected': 38, 'brake': 67, 'uncert_frac': 0.0083}

## arm: bare (no shield)   (n=100)
reached : 73/100  (73.0%)
collided: 22/100  (22.00%)   Wilson95 [15.00%, 31.07%]
timeout : 5/100
min_clr : median 7.33  p10 -0.46  min -2.22 m
attribution: in_track=11  hole_in_cone=2  hole_blind=9  by_cls={'static': 7, 'vehicle': 15}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.0556
