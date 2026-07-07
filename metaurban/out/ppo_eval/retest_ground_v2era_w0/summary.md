# shield ON/OFF campaign  (2026-07-07 17:18)  model=out/ppo_planner_ground  n=100/arm  seed=7777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 92/100  (92.0%)
collided: 6/100  (6.00%)   Wilson95 [2.78%, 12.48%]
timeout : 2/100
min_clr : median 8.65  p10 0.70  min -1.59 m
attribution: in_track=5  hole_in_cone=1  hole_blind=0  by_cls={'vehicle': 6}  med_speed=8.33
shield  : {'ticks': 9391, 'passed': 9021, 'projected': 17, 'brake': 353, 'uncert_frac': 0.0376}

## arm: bare (no shield)   (n=100)
reached : 91/100  (91.0%)
collided: 8/100  (8.00%)   Wilson95 [4.11%, 15.00%]
timeout : 1/100
min_clr : median 8.77  p10 0.68  min -1.76 m
attribution: in_track=6  hole_in_cone=0  hole_blind=2  by_cls={'static': 1, 'vehicle': 7}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.7828
