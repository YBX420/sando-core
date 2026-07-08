"""diag_v3 — 受控深度诊断:CPL-v3 复合体证书的"悬崖"。
合成卡住几何(无人机静止,正前方 d 米一个 mover),扫 d × mover 类型,报:
  能证过的候选数 / hover 是否唯一活的 / 最大前进候选(前进多少米、朝哪) / t_cert 窗长。
定位"为什么一碰 mover 就停"。纯数学,无引擎。"""
import os, sys
_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
import local_lattice as LL

p = np.array([0., 0., 1.0]); v = np.zeros(3); a = np.zeros(3)
goal = np.array([12., 0., 1.0])                        # 目标在正前方(+x)
R = 0.30 + 0.45 + 0.125 + 0.473                        # 渲染 ped cyl: r+MAN_DSAFE+q+MAN_TRACK
VEFF = 0.61
VMX, AMX, V3DT, DELTA = 3.0, 6.0, 0.3, 0.1

MOVERS = [("static", np.array([0., 0., 0.])),
          ("head-on(-x)", np.array([-1.3, 0., 0.])),
          ("cross(+y)", np.array([0., 1.3, 0.]))]

print(f"R={R:.2f}m veff={VEFF} v_max={VMX} a_max={AMX} commit={V3DT}s")
print(f"{'d(m)':>5} {'mover':>12} | {'#cert':>5} {'hover?':>6} {'#fwd':>4} {'maxfwd_prog':>11} {'best_tag':>10} {'t_cert':>6}")
for mtype, vped in MOVERS:
    for d in (6.0, 5.0, 4.0, 3.0, 2.5, 2.0, 1.5):
        cyl = [(np.array([d, 0., 1.0]), vped, np.zeros(3), R, 3.0, VEFF)]
        cands = LL.candidate_primitives(p, v, a, goal, 4.0, VMX)
        n_cert = 0; hover_ok = False; fwd = []; t_cert0 = 0.0
        for prim, tag in cands:
            segs, durs, tc = LL.make_composite(prim, AMX, V3DT); t_cert0 = tc
            if not LL.feasible(segs, durs, VMX, AMX):
                continue
            ok, m = LL.certify_composite(segs, durs, cyl, tc, DELTA)
            if not ok:
                continue
            n_cert += 1
            if tag == "hover":
                hover_ok = True
            pint = LL._poly_eval(prim, LL.T_P, 0)[0]
            prog = float((pint[:2] - p[:2]) @ np.array([1.0, 0.0]))   # goal-ward (+x) progress
            if prog > 0.15 and tag != "hover":
                fwd.append((prog, tag))
        fwd.sort(reverse=True)
        best = fwd[0] if fwd else (0.0, "-")
        print(f"{d:5.1f} {mtype:>12} | {n_cert:5d} {str(hover_ok):>6} {len(fwd):4d} {best[0]:11.2f} {best[1]:>10} {t_cert0:6.2f}")
