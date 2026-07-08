# shield ON/OFF campaign  (2026-07-08 09:36)  model=out/ppo_planner_mu  n=100/arm  seed=9777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 91/100  (91.0%)
collided: 6/100  (6.00%)   Wilson95 [2.78%, 12.48%]
timeout : 3/100
min_clr : median 8.61  p10 0.48  min -1.68 m
attribution: in_track=0  hole_in_cone=2  hole_blind=4  by_cls={'vehicle': 6}  med_speed=8.33
shield  : {'ticks': 8210, 'passed': 8130, 'projected': 62, 'brake': 18, 'uncert_frac': 0.0022}

## arm: bare (no shield)   (n=100)
reached : 84/100  (84.0%)
collided: 13/100  (13.00%)   Wilson95 [7.76%, 20.98%]
timeout : 3/100
min_clr : median 8.22  p10 -0.30  min -1.49 m
attribution: in_track=8  hole_in_cone=1  hole_blind=4  by_cls={'static': 6, 'vehicle': 7}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.1464
