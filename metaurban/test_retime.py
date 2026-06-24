"""STEP 1 (BLOCKING): unit-test the SLIP re-timing identity.

Claim: flying the committed B-spline X(u) at constant speed-warp s means at real time tau the drone is at X(s*tau);
the mover is at c(tau)=c0+v*tau+0.5*a*tau^2. So certifying the FLOWN space-time path is EXACTLY:
    certify_horizontal(c0, R, obs_vel=v/s, obs_acc=a/s^2, t_hi=s*TAU)   (substitution, no C++ change).
This must match a brute-force sampled min horizontal distance of X(s*tau) vs c(tau) over real tau in [0,TAU], and
must be SOUND: cert==CERTIFIED  =>  brute min_dist >= R (never a false certify). Block all SLIP work until green.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from ego_bridge import EGOPlanner


def quiet_replan(ego, *a):
    old = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1)
    try:
        return ego.replan(*a)
    finally:
        os.dup2(old, 1); os.close(dn); os.close(old)


ego = EGOPlanner(map_origin=(-30, -30, -1), map_size=(80, 80, 6), res=0.2, inflation=0.3)
ego.set_params(max_vel=3.0, max_acc=6.0)
ego.update_cloud(np.zeros((0, 3)), [0, 0, 1.5])
quiet_replan(ego, [0, 0, 1.5], [0, 0, 0], [0, 0, 0], [12, 0, 1.5])
dur = ego.duration()
TAU = 0.75
R = 1.0
print(f"[retime] EGO duration={dur:.2f}s; testing re-timing cert vs brute-force space-time")
fails = 0
# drone flies +x along y=0. Movers that CROSS the path closely so the cert is exercised at the boundary and the
# verdict DEPENDS on s (slowing/speeding changes who-passes-first) — the heart of the slip-behind/ahead logic.
movers = [(np.array([2.0, 0.2, 1.5]), np.zeros(3), np.zeros(3)),                  # SITS on the path -> must REJECT
          (np.array([3.0, 0.4, 1.5]), np.array([0.0, -1.6, 0.0]), np.zeros(3)),   # crosses path ~x=3 — near
          (np.array([9.0, 0.8, 1.5]), np.array([0.0, -1.2, 0.0]), np.zeros(3)),   # crosses path ~x=9
          (np.array([4.0, 1.2, 1.5]), np.array([0.0, -1.4, 0.0]), np.zeros(3))]   # a clear-ish one
rejected = 0
for mi, (c0, v, a) in enumerate(movers):
    for s in (0.5, 0.8, 1.0, 1.2, 1.4):
        cert, margin = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=v / s, obs_acc=a / (s * s),
                                               t_hi=s * TAU, v_eff=0.0, delta=0.0)
        mind = 1e18
        for tau in np.linspace(0.0, TAU, 4000):
            u = s * tau
            if u > dur:
                break
            r = ego.eval(u)
            if r is None:
                continue
            p = np.asarray(r[0], float)
            c = c0 + v * tau + 0.5 * a * tau * tau
            mind = min(mind, float(np.hypot(p[0] - c[0], p[1] - c[1])))
        bf = mind >= R
        unsound = cert and not (mind >= R - 1e-6)            # CERTIFIED but actually closer than R = false certify
        if unsound:
            fails += 1
        if not cert:
            rejected += 1
        tag = "UNSOUND!!" if unsound else ("ok" if cert == bf else "conservative")  # cert=F,bf=T is fine (conservative)
        print(f"  mover{mi} s={s:.1f}: cert={int(cert)} margin={margin:+.3f} | brute min_d={mind:.3f} (R={R}) bf={int(bf)}  {tag}")
ok = (fails == 0 and rejected > 0)                           # sound AND non-vacuous (some genuine rejections seen)
print(f"\n[retime] {fails} UNSOUND case(s), {rejected} rejections (non-vacuous).  "
      f"{'STEP 1 PASS (sound + exercised)' if ok else ('SOUNDNESS BROKEN — STOP' if fails else 'VACUOUS — all certified, strengthen test')}")
sys.exit(0 if ok else 1)
