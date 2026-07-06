# shield ON/OFF campaign  (2026-07-07 00:12)  model=out/ppo_planner_ground  n=400/arm  seed=7777 (paired scenarios)

## arm: shielded (certificate gate ON)   (n=400)
reached : 353/400  (88.2%)
collided: 27/400  (6.75%)   Wilson95 [4.68%, 9.64%]
timeout : 20/400
min_clr : median 7.37  p10 0.98  min -1.82 m
attribution: in_track=13  hole_in_cone=2  hole_blind=12  by_cls={'vehicle': 27}  med_speed=8.33
shield  : {'ticks': 42131, 'passed': 38572, 'projected': 2619, 'brake': 940, 'uncert_frac': 0.0223}

## arm: bare (no shield)   (n=400)
reached : 358/400  (89.5%)
collided: 37/400  (9.25%)   Wilson95 [6.79%, 12.49%]
timeout : 5/400
min_clr : median 8.13  p10 0.34  min -2.30 m
attribution: in_track=21  hole_in_cone=1  hole_blind=15  by_cls={'static': 6, 'vehicle': 31}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.2407
