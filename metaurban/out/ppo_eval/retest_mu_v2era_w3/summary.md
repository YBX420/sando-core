# shield ON/OFF campaign  (2026-07-07 16:37)  model=out/ppo_planner_mu  n=100/arm  seed=10777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 91/100  (91.0%)
collided: 6/100  (6.00%)   Wilson95 [2.78%, 12.48%]
timeout : 3/100
min_clr : median 7.04  p10 0.42  min -1.80 m
attribution: in_track=3  hole_in_cone=1  hole_blind=2  by_cls={'vehicle': 6}  med_speed=8.33
shield  : {'ticks': 7379, 'passed': 7119, 'projected': 93, 'brake': 167, 'uncert_frac': 0.0226}

## arm: bare (no shield)   (n=100)
reached : 75/100  (75.0%)
collided: 21/100  (21.00%)   Wilson95 [14.17%, 29.98%]
timeout : 4/100
min_clr : median 6.79  p10 -0.54  min -2.08 m
attribution: in_track=14  hole_in_cone=2  hole_blind=5  by_cls={'static': 7, 'vehicle': 14}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.0032
