# shield ON/OFF campaign  (2026-07-06 15:10)  model=out/ppo_planner_mu_adapt  n=400/arm  seed=7777 (paired scenarios)

## arm: shielded (certificate gate ON)   (n=400)
reached : 350/400  (87.5%)
collided: 26/400  (6.50%)   Wilson95 [4.47%, 9.35%]
timeout : 24/400
min_clr : median 8.54  p10 1.00  min -1.91 m
attribution: in_track=5  hole_in_cone=0  hole_blind=21  by_cls={'vehicle': 26}  med_speed=8.33
shield  : {'ticks': 32229, 'passed': 30801, 'projected': 1358, 'brake': 70, 'uncert_frac': 0.0022}

## arm: bare (no shield)   (n=400)
reached : 313/400  (78.2%)
collided: 77/400  (19.25%)   Wilson95 [15.69%, 23.40%]
timeout : 10/400
min_clr : median 8.16  p10 -0.41  min -1.40 m
attribution: in_track=48  hole_in_cone=0  hole_blind=29  by_cls={'static': 38, 'vehicle': 39}  med_speed=7.07

Fisher exact (collision, shielded vs bare): p = 0.0000
