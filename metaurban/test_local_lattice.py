"""test_local_lattice — Stage 1 property test: the 0-false-cert soundness gate.

For 10^4 random (intent primitive, cylinder set) scenarios: build the composite, certify. When the
certificate PASSES, brute-force sample the composite at 4000 wall-times and verify the SAME
disjunction the kernel claims -- (moving-clear AND frozen-clear) OR above -- holds per mover at
every sample. A cert-pass with a sample violating all disjuncts is a FALSE CERTIFICATE = fatal.
"""
import sys

import numpy as np

sys.path.insert(0, "/media/boxuan/Data2/projects/sando_py/sando-core/metaurban")
import local_lattice as LL

RNG = np.random.default_rng(20260708)
DELTA = 0.30


def random_primitive(v_max):
    # reachable-set parametrization (blueprint 2): terminal velocity near v0, terminal position
    # DERIVED from the velocity profile so the quintic stays kinematically smooth.
    p0 = np.array([0.0, 0.0, 1.0])
    sp0 = RNG.uniform(0, v_max)
    hd0 = RNG.uniform(-np.pi, np.pi)
    v0 = np.array([sp0 * np.cos(hd0), sp0 * np.sin(hd0), 0.0])
    a0 = np.concatenate([RNG.uniform(-1, 1, 2), [0.0]])
    dpsi = np.radians(RNG.choice([0, 15, -15, 30, -30, 50, -50, 75, -75]))
    vT = float(np.clip(sp0 + RNG.choice([1.2, 0.6, 0, -0.9, -1.8]), 0, v_max))
    hdT = hd0 + dpsi
    vTv = np.array([vT * np.cos(hdT), vT * np.sin(hdT), 0.0])
    pT = p0 + 0.5 * (v0 + vTv) * LL.T_P                 # trapezoidal displacement (consistent)
    pT[2] = RNG.choice([1.0, 3.0])
    return LL.quintic3(p0, v0, a0, pT, vTv, np.zeros(3), LL.T_P)


def random_cyl(n):
    out = []
    for _ in range(n):
        c0 = np.array([*RNG.uniform(-6, 6, 2), 0.0])
        vel = np.array([*RNG.uniform(-3, 3, 2), 0.0])
        R = RNG.uniform(0.8, 2.5)
        zc = RNG.choice([2.0, 3.5, 50.0])               # 50 = effectively unflyable-over
        veff = RNG.uniform(0.5, 1.6)
        out.append((c0, vel, np.zeros(3), R, zc, veff))
    return out


def brute_ok(segs, durs, cyl, t_cert, n=4000):
    """True iff every mover's disjunct holds at every sampled wall-time (kernel-matching)."""
    ts = np.linspace(0, t_cert, n)
    for (c0, vel, acc, R, zc, veff) in cyl:
        above_all = True
        horiz_all = True
        for t in ts:
            p = LL.composite_eval(segs, durs, t)
            tube = R + veff * (t + DELTA)
            cm = c0 + vel * t + 0.5 * acc * t * t
            d_move = np.hypot(p[0] - cm[0], p[1] - cm[1])
            d_froz = np.hypot(p[0] - c0[0], p[1] - c0[1])
            if not (p[2] >= zc):
                above_all = False
            if not (d_move >= tube and d_froz >= tube):
                horiz_all = False
            if not above_all and not horiz_all:
                return False                            # this mover's disjunct broken -> unsafe
    return True


def main():
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 10000
    v_max, a_max = 3.0, 6.0
    n_pass = n_false = n_feas = 0
    for i in range(N):
        prim = random_primitive(v_max)
        segs, durs, t_cert = LL.make_composite(prim, a_max)
        if not LL.feasible(segs, durs, v_max, a_max):
            continue
        n_feas += 1
        cyl = random_cyl(RNG.integers(1, 6))
        ok, _ = LL.certify_composite(segs, durs, cyl, t_cert, DELTA)
        if ok:
            n_pass += 1
            if not brute_ok(segs, durs, cyl, t_cert):
                n_false += 1
                if n_false <= 3:
                    print(f"  FALSE CERT #{i}: t_cert={t_cert:.2f} ncyl={len(cyl)}")
    print(f"scenarios={N} feasible={n_feas} cert_pass={n_pass} FALSE_CERTS={n_false}")
    print("PASS" if n_false == 0 else "FAIL")
    sys.exit(0 if n_false == 0 else 1)


if __name__ == "__main__":
    main()
