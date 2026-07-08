"""metrics_social — TOP-5 standard social-nav comfort metrics from telemetry-v2 json files.
Formulas & citation traps: docs/comfort-metrics-adoption.md. Conventions: CPD = CENTRE distance;
radii (r_drone=0.25, r_ped=0.30) applied only where a surface distance is required; jerk from the
0.06s channel only (asserted); TTC vs nearest mover, 10s saturation; PSC on surface distance @0.5m.
Usage: python metrics_social.py out/telem_ours_A.json [more.json ...]
"""
import json
import sys

import numpy as np

R_DRONE, R_PED = 0.25, 0.30


def compute(path):
    T = np.array(json.load(open(path)))
    if T.shape[1] < 12:
        return dict(file=path, error="old telemetry format (need v2 rows)")
    t, spd = T[:, 0], T[:, 3]
    p = T[:, 4:6]
    dt = float(np.median(np.diff(t)))
    assert dt <= 0.12, f"jerk needs the fast telemetry channel, got dt={dt}"
    # NB: renderer telemetry runs ~0.1s; jerk numbers are comparable ONLY within the same rate
    # (Arena-style caveat, see docs/comfort-metrics-adoption.md citation traps)
    cpd = T[:, 7]
    rel = T[:, 8:10]
    relv = T[:, 10:12]
    surf = cpd - R_DRONE - R_PED
    # D1: accel / jerk (position 2nd/3rd differences, low-passed)
    v = np.diff(p, axis=0) / dt
    a = np.diff(v, axis=0) / dt
    j = np.diff(a, axis=0) / dt
    k = np.ones(8) / 8.0
    a_m = np.convolve(np.linalg.norm(a, axis=1), k, mode="valid")
    j_m = np.convolve(np.linalg.norm(j, axis=1), k, mode="valid")
    # B3: TTC vs nearest mover (closing-speed form, 10s saturation)
    ttcs = []
    for i in range(len(T)):
        d = float(np.hypot(*rel[i]))
        if d < 1e-6:
            continue
        closing = float(-(rel[i] @ relv[i]) / d)
        if closing > 0.05:
            ttcs.append(min((d - R_DRONE - R_PED) / closing, 10.0))
    return dict(
        file=path.split("/")[-1],
        path_len=round(float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))), 1),
        mean_speed=round(float(spd.mean()), 2),
        CPD_min=round(float(cpd.min()), 2), CPD_mean=round(float(cpd.mean()), 2),
        PSC_0p5=round(float(np.mean(surf >= 0.5)), 3),
        Disc_0p2=round(float(np.mean(surf < 0.2)), 3),
        TTC_min=round(float(min(ttcs)) if ttcs else 10.0, 2),
        accel_mean=round(float(a_m.mean()), 2),
        jerk_mean=round(float(j_m.mean()), 1),
        stalled_frac=round(float(np.mean(spd < 0.3)), 3),
    )


if __name__ == "__main__":
    for f in sys.argv[1:]:
        print(json.dumps(compute(f), ensure_ascii=False))
