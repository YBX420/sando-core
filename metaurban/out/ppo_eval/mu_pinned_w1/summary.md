# shield ON/OFF campaign  (2026-07-08 09:33)  model=out/ppo_planner_mu  n=100/arm  seed=8777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 91/100  (91.0%)
collided: 5/100  (5.00%)   Wilson95 [2.15%, 11.18%]
timeout : 4/100
min_clr : median 8.43  p10 0.57  min -1.40 m
attribution: in_track=1  hole_in_cone=2  hole_blind=2  by_cls={'vehicle': 5}  med_speed=8.33
shield  : {'ticks': 7685, 'passed': 7585, 'projected': 52, 'brake': 48, 'uncert_frac': 0.0062}

## arm: bare (no shield)   (n=100)
reached : 80/100  (80.0%)
collided: 16/100  (16.00%)   Wilson95 [10.10%, 24.42%]
timeout : 4/100
min_clr : median 8.79  p10 -0.31  min -1.10 m
attribution: in_track=8  hole_in_cone=2  hole_blind=6  by_cls={'static': 6, 'vehicle': 10}  med_speed=8.095

Fisher exact (collision, shielded vs bare): p = 0.0192
