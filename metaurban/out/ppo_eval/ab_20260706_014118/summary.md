# shield ON/OFF campaign  (2026-07-06 03:54)  model=out/ppo_planner_mu  n=400/arm  seed=7777 (paired scenarios)

## arm: shielded (certificate gate ON)   (n=400)
reached : 358/400  (89.5%)
collided: 27/400  (6.75%)   Wilson95 [4.68%, 9.64%]
timeout : 15/400
min_clr : median 8.48  p10 0.33  min -1.94 m
attribution: in_track=2  hole_in_cone=1  hole_blind=24  by_cls={'vehicle': 27}  med_speed=8.33
shield  : {'ticks': 29389, 'passed': 27925, 'projected': 1135, 'brake': 329, 'uncert_frac': 0.0112}

## arm: bare (no shield)   (n=400)
reached : 324/400  (81.0%)
collided: 57/400  (14.25%)   Wilson95 [11.16%, 18.02%]
timeout : 19/400
min_clr : median 8.37  p10 -0.11  min -2.13 m
attribution: in_track=33  hole_in_cone=2  hole_blind=22  by_cls={'static': 25, 'vehicle': 32}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.0007
