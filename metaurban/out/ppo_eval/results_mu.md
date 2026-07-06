# ppo_planner_mu A/B eval + attribution  (2026-07-06 01:26)

## arm: shielded (certificate gate ON)
reached : 51/60  (85.0%)
collided: 4/60  (6.7%)
timeout : 5/60
min_clr : median 10.49  p10 1.89  min -1.71 m
shield  : {'ticks': 4638, 'passed': 4591, 'projected': 42, 'brake': 5, 'uncert_frac': 0.0011}
attribution: coverage-hole 4/4, in-track 0/4
  ep11: cls=vehicle    r=2.87 in_track=False track_dist=5.55 n_ready=2 shield=passed
  ep15: cls=vehicle    r=2.87 in_track=False track_dist=None n_ready=0 shield=passed
  ep35: cls=vehicle    r=1.83 in_track=False track_dist=None n_ready=0 shield=passed
  ep55: cls=vehicle    r=1.83 in_track=False track_dist=None n_ready=0 shield=passed

## arm: bare policy (no shield)
reached : 51/60  (85.0%)
collided: 6/60  (10.0%)
timeout : 3/60
min_clr : median 10.05  p10 0.00  min -0.36 m
attribution: coverage-hole 4/6, in-track 2/6
  ep11: cls=vehicle    r=2.87 in_track=True track_dist=2.16 n_ready=1 shield=no-shield
  ep13: cls=static     r=0.36 in_track=True track_dist=0.03 n_ready=5 shield=no-shield
  ep15: cls=vehicle    r=2.87 in_track=False track_dist=None n_ready=0 shield=no-shield
  ep27: cls=vehicle    r=2.87 in_track=False track_dist=22.49 n_ready=2 shield=no-shield
  ep35: cls=vehicle    r=1.83 in_track=False track_dist=None n_ready=0 shield=no-shield
  ep55: cls=vehicle    r=1.83 in_track=False track_dist=None n_ready=0 shield=no-shield
