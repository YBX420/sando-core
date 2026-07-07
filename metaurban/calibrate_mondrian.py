"""calibrate_mondrian — per-CLASS conformal ranks on top of the canonical FS3C-R shapes.

The live calib's single shared q_hat couples classes: pedestrian tail flights push q_hat=1.487 and
vehicles get sentenced with it (TEST vehicle coverage 1.0 = plainly over-conservative; excess-evade
heatmap: vehicle scenarios = 36%+ of all superfluous maneuvers). Mondrian split-conformal gives each
class its own rank over per-flight per-class sups -- class-conditional guarantee, textbook-valid.
Shapes (b, v, sigma) are TAKEN AS-IS from the canonical calib (the stable estimators); only the
quantile layer changes. Classes whose flight count can't support the rank fall back to the shared q_hat.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
CAN = json.load(open(os.path.join("..", "out", "conformal", "calib_v2.json")))
B = np.load("out/conformal/harvest_foldB3_v2.npy")   # live fold of the canonical calib
AGE_MIN = 4
SHAPES = {}
for cls in ("pedestrian", "vehicle", "static"):
    lv = CAN["groups"][cls]["levels"]
    SHAPES[cls] = lv   # per-eps dict with q_conformal/v_eff; we need b,v,sigma -> recover from provenance
PROV = CAN.get("provenance", {}).get("shapes", {})


def cls_sups(D, cls, b, v, s):
    out = {}
    for key in {(str(x), int(e)) for x, e in zip(D["scn"], D["ep"])}:
        scn, epi = key
        m = ((D["scn"] == scn) & (D["ep"] == epi) & (D["cls"] == cls) & (D["qual"] == 1)
             & (D["age"] >= AGE_MIN))
        if not m.sum():
            continue
        r = (D["e"][m] - v * D["d"][m] - b) / s
        out[key] = float(np.max(r))
    return out


res = {}
for eps in (0.05, 0.10):
    res[str(eps)] = {}
    for cls in ("pedestrian", "vehicle", "static"):
        sh = PROV.get(cls)
        if not sh:
            print(f"[mondrian] {cls}: no shape provenance -> keep shared"); continue
        b, v, s = sh["b"], sh["v"], sh["sigma"]
        sups = cls_sups(B, cls, b, v, s)
        n = len(sups)
        k = int(np.ceil((n + 1) * (1 - eps)))
        if k > n:
            print(f"[mondrian] eps={eps} {cls}: n={n} k={k} UNDER -> keep shared q_hat")
            continue
        qc = float(np.sort(list(sups.values()))[k - 1])
        res[str(eps)][cls] = dict(n=n, k=k, qhat_c=round(qc, 4))
        print(f"[mondrian] eps={eps} {cls}: n={n} k={k} qhat_c={qc:.3f}")
json.dump(res, open(os.path.join("..", "out", "conformal", "mondrian_ranks.json"), "w"), indent=1)
print("[mondrian] wrote ../out/conformal/mondrian_ranks.json")
