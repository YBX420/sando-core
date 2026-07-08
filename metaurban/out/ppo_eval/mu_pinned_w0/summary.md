# shield ON/OFF campaign  (2026-07-08 09:36)  model=out/ppo_planner_mu  n=100/arm  seed=7777 (same-seed stream, NOT strictly paired)
config: {"SHIELD_EPS": "", "SHIELD_PERCLASS": "", "DECIDE": "", "SMOOTH": "", "RADIUS_CONSIST": "", "CRET_GLIDE": "", "CALIB_V2": "1", "PERCEPT": "", "PERCEPT_SEED": "", "PRED_MODEL": "", "DYN_TRACK": ""}

## arm: shielded (certificate gate ON)   (n=100)
reached : 91/100  (91.0%)
collided: 2/100  (2.00%)   Wilson95 [0.55%, 7.00%]
timeout : 7/100
min_clr : median 9.52  p10 0.55  min -1.49 m
attribution: in_track=0  hole_in_cone=0  hole_blind=2  by_cls={'vehicle': 2}  med_speed=8.33
shield  : {'ticks': 8367, 'passed': 8268, 'projected': 61, 'brake': 38, 'uncert_frac': 0.0045}

## arm: bare (no shield)   (n=100)
reached : 81/100  (81.0%)
collided: 14/100  (14.00%)   Wilson95 [8.53%, 22.14%]
timeout : 5/100
min_clr : median 9.01  p10 -0.08  min -1.49 m
attribution: in_track=10  hole_in_cone=0  hole_blind=4  by_cls={'static': 8, 'vehicle': 6}  med_speed=0.0

Fisher exact (collision, shielded vs bare): p = 0.0029
