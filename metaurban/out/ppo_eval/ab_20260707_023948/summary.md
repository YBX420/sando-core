# shield ON/OFF campaign  (2026-07-07 05:27)  model=out/ppo_planner_ground  n=400/arm  seed=7777 (paired scenarios)

## arm: shielded (certificate gate ON)   (n=400)
reached : 362/400  (90.5%)
collided: 26/400  (6.50%)   Wilson95 [4.47%, 9.35%]
timeout : 12/400
min_clr : median 7.11  p10 0.65  min -1.82 m
attribution: in_track=10  hole_in_cone=2  hole_blind=14  by_cls={'vehicle': 26}  med_speed=8.33
shield  : {'ticks': 39214, 'passed': 38723, 'projected': 185, 'brake': 306, 'uncert_frac': 0.0078}

## arm: bare (no shield)   (n=400)
reached : 360/400  (90.0%)
collided: 35/400  (8.75%)   Wilson95 [6.36%, 11.93%]
timeout : 5/400
min_clr : median 8.13  p10 0.48  min -1.76 m
attribution: in_track=21  hole_in_cone=1  hole_blind=13  by_cls={'static': 6, 'vehicle': 29}  med_speed=8.33

Fisher exact (collision, shielded vs bare): p = 0.2865
