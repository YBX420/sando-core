"""cert_ablation — continuous-time Bernstein certificate vs fixed-rate discrete sampling.

The headline claim of the safety layer is that the certificate is SOUND in continuous time: it proves
|| p(t) - c(t) || >= rho(t) for ALL t in [0,TAU], not just at sample points. The usual cheap alternative --
sample the committed trajectory at N times and check separation there -- can TUNNEL: a fast obstacle slips
through the gap between samples, so the check says "safe" while the trajectory actually collides.

This quantifies it. For many (EGO committed B-spline, moving cylinder) cases drawn from real planning:
  truth      = dense 4000-sample min of (horizontal_sep(t) - rho(t)) over [0,TAU]  (collision iff < 0)
  continuous = certify_horizontal (the sound Bernstein deficit)                     -> verdict + margin
  discrete-N = min over N evenly-spaced samples of the SAME committed B-spline       -> verdict
We report, per method, the FALSE-SAFE rate (verdict=safe while truth=collision) -- the dangerous error.
Sound => 0 false-safe. Discrete-N => some false-safe (misses between samples).

Run:  python metaurban/cert_ablation.py --trials 400
"""
import os, sys, math, json, argparse, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ego_bridge import EGOPlanner

OUTDIR = os.path.join(os.path.dirname(HERE), "out", "conformal")
TAU = 0.75
DELTA = 0.30


@contextlib.contextmanager
def _quiet():
    fd = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1); os.close(dn)
    try:
        yield
    finally:
        os.dup2(fd, 1); os.close(fd)


def horiz_sep(p, c):
    return math.hypot(p[0] - c[0], p[1] - c[1])


def truth_min_deficit(ego, c0, vel, R, v_eff, n=4000):
    """Dense-sample the committed B-spline vs the moving cylinder axis; return min(sep - rho) over [0,TAU]."""
    dur = ego.duration()
    md = 1e18
    for t in np.linspace(0.0, min(TAU, dur), n):
        r = ego.eval(t)
        if r is None:
            continue
        p = r[0]
        c = c0 + vel * t
        rho = R + v_eff * (t + DELTA)
        md = min(md, horiz_sep(p, c) - rho)
    return md


def discrete_safe(ego, c0, vel, R, v_eff, n):
    """N-sample gate verdict: 'safe' iff all n samples clear the moving cylinder. Returns bool."""
    dur = ego.duration()
    for t in np.linspace(0.0, min(TAU, dur), max(2, n)):
        r = ego.eval(t)
        if r is None:
            continue
        p = r[0]
        c = c0 + vel * t
        if horiz_sep(p, c) < R + v_eff * (t + DELTA):
            return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=400)
    ap.add_argument("--samples", default="2,3,5,9", help="discrete N values to test")
    args = ap.parse_args()
    Ns = [int(x) for x in args.samples.split(",")]
    rng = np.random.default_rng(7)

    ego = EGOPlanner(map_origin=(-30, -30, -1), map_size=(60, 60, 6), res=0.2, inflation=0.2)
    ego.set_params(max_vel=4.0, max_acc=8.0, horizon=7.5)

    stats = {"continuous": dict(safe=0, false_safe=0, false_unsafe=0),
             **{f"discrete_{n}": dict(safe=0, false_safe=0, false_unsafe=0) for n in Ns}}
    n_collide = n_used = 0

    with _quiet():
        for _ in range(args.trials):
            # a short plan with a couple of static cloud points, then a FAST moving cylinder near the path
            gx = rng.uniform(6, 12)
            cloud = [[rng.uniform(2, gx), rng.uniform(-2, 2), 1.5] for _ in range(rng.integers(0, 4))]
            ego.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), [0, 0, 1.5])
            if not ego.replan([0, 0, 1.5], [rng.uniform(0, 2), rng.uniform(-1, 1), 0], [0, 0, 0],
                              [gx, rng.uniform(-2, 2), 1.5]) or ego.duration() < 1e-2:
                continue
            # sample a point on the path to aim the moving obstacle near it (so cases are non-trivial)
            taim = rng.uniform(0.1, min(TAU, ego.duration()))
            rr = ego.eval(taim)
            if rr is None:
                continue
            paim = rr[0]
            speed = rng.uniform(1.0, 6.0)                      # fast movers -> tunnelling risk
            ang = rng.uniform(0, 2 * math.pi)
            vel = np.array([speed * math.cos(ang), speed * math.sin(ang), 0.0])
            # place the cylinder so that at time taim it is offset from paim by a random small miss distance
            miss = rng.uniform(-1.2, 1.2)
            perp = np.array([-math.sin(ang), math.cos(ang), 0.0])
            c_at_aim = np.array([paim[0], paim[1], 1.5]) + miss * perp
            c0 = c_at_aim - vel * taim
            R = rng.uniform(0.4, 0.9)
            v_eff = rng.choice([0.0, 0.3])

            md = truth_min_deficit(ego, c0, vel, R, v_eff)
            truth_collide = md < 0.0
            n_used += 1; n_collide += int(truth_collide)

            cont_safe, margin = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=vel, t_hi=TAU, v_eff=v_eff, delta=DELTA)
            _tally(stats["continuous"], cont_safe, truth_collide)
            for n in Ns:
                _tally(stats[f"discrete_{n}"], discrete_safe(ego, c0, vel, R, v_eff, n), truth_collide)

    print(f"\n=== continuous-time cert vs discrete sampling  ({n_used} cases, {n_collide} truly colliding) ===")
    print(f"{'method':<14}{'verdict=safe':>13}{'FALSE-SAFE':>12}{'false-unsafe':>14}   (false-safe = MISSED collision)")
    for k, s in stats.items():
        print(f"{k:<14}{s['safe']:>13}{s['false_safe']:>12}{s['false_unsafe']:>14}")
    print("\nSOUNDNESS: continuous-time certificate false-safe =", stats["continuous"]["false_safe"],
          "(must be 0).  Discrete sampling tunnels -> misses collisions.")

    os.makedirs(OUTDIR, exist_ok=True)
    with open(os.path.join(OUTDIR, "cert_ablation.json"), "w") as f:
        json.dump(dict(n_cases=n_used, n_collide=n_collide, stats=stats, samples=Ns), f, indent=2)
    print(f"[abl] wrote {os.path.join(OUTDIR, 'cert_ablation.json')}")
    return 0


def _tally(s, verdict_safe, truth_collide):
    if verdict_safe and not truth_collide:
        s["safe"] += 1
    elif verdict_safe and truth_collide:
        s["safe"] += 1; s["false_safe"] += 1            # DANGEROUS: said safe, actually collides
    elif (not verdict_safe) and (not truth_collide):
        s["false_unsafe"] += 1                          # conservative: rejected a safe one (acceptable)


if __name__ == "__main__":
    sys.exit(main())
