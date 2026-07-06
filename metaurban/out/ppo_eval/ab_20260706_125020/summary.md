# shield ON/OFF campaign  (2026-07-06 15:19)  model=out/ppo_planner_mu_bare  n=400/arm  seed=7777 (paired scenarios)

## arm: shielded (certificate gate ON)   (n=400)
reached : 329/400  (82.2%)
collided: 33/400  (8.25%)   Wilson95 [5.93%, 11.36%]
timeout : 38/400
min_clr : median 8.48  p10 0.68  min -1.84 m
attribution: in_track=9  hole_in_cone=0  hole_blind=24  by_cls={'vehicle': 33}  med_speed=8.33
shield  : {'ticks': 33753, 'passed': 32822, 'projected': 734, 'brake': 197, 'uncert_frac': 0.0058}

## arm: bare (no shield)   (n=400)
reached : 307/400  (76.8%)
collided: 58/400  (14.50%)   Wilson95 [11.39%, 18.29%]
timeout : 35/400
min_clr : median 8.21  p10 -0.30  min -2.07 m
attribution: in_track=28  hole_in_cone=1  hole_blind=29  by_cls={'static': 18, 'vehicle': 40}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.0073
