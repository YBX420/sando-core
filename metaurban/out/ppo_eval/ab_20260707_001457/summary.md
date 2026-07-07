# shield ON/OFF campaign  (2026-07-07 03:04)  model=out/ppo_planner_ground  n=400/arm  seed=7777 (paired scenarios)

## arm: shielded (certificate gate ON)   (n=400)
reached : 354/400  (88.5%)
collided: 30/400  (7.50%)   Wilson95 [5.30%, 10.50%]
timeout : 16/400
min_clr : median 7.11  p10 0.86  min -2.04 m
attribution: in_track=14  hole_in_cone=0  hole_blind=16  by_cls={'vehicle': 30}  med_speed=8.33
shield  : {'ticks': 39798, 'passed': 38478, 'projected': 308, 'brake': 1012, 'uncert_frac': 0.0254}

## arm: bare (no shield)   (n=400)
reached : 361/400  (90.2%)
collided: 33/400  (8.25%)   Wilson95 [5.93%, 11.36%]
timeout : 6/400
min_clr : median 8.13  p10 0.49  min -1.76 m
attribution: in_track=20  hole_in_cone=1  hole_blind=12  by_cls={'static': 5, 'vehicle': 28}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.7931
