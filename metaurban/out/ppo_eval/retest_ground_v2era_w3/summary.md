# shield ON/OFF campaign  (2026-07-07 17:20)  model=out/ppo_planner_ground  n=100/arm  seed=10777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 93/100  (93.0%)
collided: 5/100  (5.00%)   Wilson95 [2.15%, 11.18%]
timeout : 2/100
min_clr : median 6.98  p10 0.79  min -1.82 m
attribution: in_track=4  hole_in_cone=0  hole_blind=1  by_cls={'vehicle': 5}  med_speed=8.33
shield  : {'ticks': 9972, 'passed': 9683, 'projected': 26, 'brake': 263, 'uncert_frac': 0.0264}

## arm: bare (no shield)   (n=100)
reached : 93/100  (93.0%)
collided: 6/100  (6.00%)   Wilson95 [2.78%, 12.48%]
timeout : 1/100
min_clr : median 8.24  p10 0.71  min -1.39 m
attribution: in_track=5  hole_in_cone=0  hole_blind=1  by_cls={'static': 2, 'vehicle': 4}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 1.0000
