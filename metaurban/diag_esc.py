"""diag_esc — V3_ESC 预认证应急树 vs 单直刹的"悬崖"对照(同 diag_v3 合成几何)。
每个 (d, mover):基线=直刹复合体可证数;ESC=直刹失败后走逃生树(veer/hop)可救回数。
报:#cert 基线→ESC / hover 基线→ESC / 最大前进候选 基线→ESC。纯数学,无引擎。"""
import os, sys
_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
import local_lattice as LL

p = np.array([0., 0., 1.0]); v = np.zeros(3); a = np.zeros(3)
goal = np.array([12., 0., 1.0])
R = 0.30 + 0.45 + 0.125 + 0.473
VEFF = 0.61
VMX, AMX, V3DT, DELTA = 3.0, 6.0, 0.3, 0.1
ANGS = (45.0, 90.0); HOPS = (1.5, 3.0)

MOVERS = [("static", np.array([0., 0., 0.])),
          ("head-on(-x)", np.array([-1.3, 0., 0.])),
          ("cross(+y)", np.array([0., 1.3, 0.]))]

print(f"R={R:.2f}m veff={VEFF} v_max={VMX} a_max={AMX} commit={V3DT}s esc_ang={ANGS} hops={HOPS}")
print(f"{'d(m)':>5} {'mover':>12} | {'#cert b->e':>10} {'hover b->e':>11} {'maxfwd b->e':>13} {'esc_used':>8}")
for mtype, vped in MOVERS:
    for d in (6.0, 4.0, 3.0, 2.5, 2.0, 1.5, 1.2):
        cyl = [(np.array([d, 0., 1.0]), vped, np.zeros(3), R, 3.0, VEFF)]
        cands = LL.candidate_primitives(p, v, a, goal, 4.0, VMX)
        nb = ne = esc_used = 0
        hb = he = False
        fb = fe = 0.0
        for prim, tag in cands:
            segs, durs, tc = LL.make_composite(prim, AMX, V3DT)
            if not LL.feasible(segs, durs, VMX, AMX):
                continue
            ok, m, who = LL.certify_composite(segs, durs, cyl, tc, DELTA, want_who=True)
            ok_e = ok
            if not ok:
                r = LL.try_escapes(prim, AMX, V3DT, cyl, DELTA, who, ANGS, HOPS, VMX)
                if r is not None:
                    ok_e = True
                    esc_used += 1
            pint = LL._poly_eval(prim, LL.T_P, 0)[0]
            prog = float((pint[:2] - p[:2]) @ np.array([1.0, 0.0]))
            if ok:
                nb += 1
                hb = hb or tag == "hover"
                if tag != "hover":
                    fb = max(fb, prog)
            if ok_e:
                ne += 1
                he = he or tag == "hover"
                if tag != "hover":
                    fe = max(fe, prog)
        print(f"{d:5.1f} {mtype:>12} | {nb:4d}->{ne:4d} {str(hb):>5}->{str(he):>5} "
              f"{fb:5.2f}->{fe:5.2f} {esc_used:8d}")
