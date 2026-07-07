# shield ON/OFF campaign  (2026-07-07 16:35)  model=out/ppo_planner_mu  n=100/arm  seed=7777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 89/100  (89.0%)
collided: 8/100  (8.00%)   Wilson95 [4.11%, 15.00%]
timeout : 3/100
min_clr : median 9.53  p10 0.43  min -1.00 m
attribution: in_track=1  hole_in_cone=0  hole_blind=7  by_cls={'vehicle': 8}  med_speed=8.33
shield  : {'ticks': 7159, 'passed': 6960, 'projected': 106, 'brake': 93, 'uncert_frac': 0.013}

## arm: bare (no shield)   (n=100)
reached : 82/100  (82.0%)
collided: 14/100  (14.00%)   Wilson95 [8.53%, 22.14%]
timeout : 4/100
min_clr : median 9.66  p10 -0.12  min -2.18 m
attribution: in_track=7  hole_in_cone=0  hole_blind=7  by_cls={'static': 4, 'vehicle': 10}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.2582
