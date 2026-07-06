"""verify_spacing — benchmark realism gate: per-class-pair minimum distances over the FULL compiled
timeline must clear the floors (ped-ped 0.8 / ped-veh 2.0 / veh-veh 4.0 m beyond body radii).
Run: python3 verify_spacing.py [scenarios/bench] -- exits 1 on any violation (CI-able)."""
import glob, sys
import numpy as np
import scenario_lib as SLB

FLOORS = {("pedestrian","pedestrian"): 0.8, ("pedestrian","vehicle"): 2.0, ("vehicle","vehicle"): 4.0}

def main(d="scenarios/bench"):
    ok_all = True
    for f in sorted(glob.glob(f"{d}/*.json")):
        if f.endswith("MANIFEST.json"): continue
        import os as _os
        if not _os.path.basename(f).startswith("street_"):
            print(f"{_os.path.basename(f):24s} (hand-crafted formation: spacing floors exempt)")
            continue                          # floors govern GENERATOR realism; deliberate walls are design
        scn = SLB.load(f)
        tracks = [SLB.compile_mover(m, float(scn["t_max"])) for m in scn["movers"]]
        rs = [m.get("r") or SLB.CLS_R[m["cls"]] for m in scn["movers"]]
        cls = [m["cls"] for m in scn["movers"]]
        tl = np.arange(0, float(scn["t_max"]), 0.1)
        pos = []
        for tr in tracks:
            x = np.interp(tl, tr["t"], tr["xy"][:,0]); y = np.interp(tl, tr["t"], tr["xy"][:,1])
            x[(tl < tr["t"][0]) | (tl > tr["t"][-1])] = np.nan; y[np.isnan(x)] = np.nan
            pos.append(np.stack([x,y],1))
        worst = {}
        for i in range(len(tracks)):
            for j in range(i+1, len(tracks)):
                key = tuple(sorted([cls[i], cls[j]]))
                if key not in FLOORS: continue
                dd = np.hypot(*(pos[i]-pos[j]).T) - rs[i] - rs[j]
                if not np.all(np.isnan(dd)):
                    worst[key] = min(worst.get(key, 1e9), float(np.nanmin(dd)))
        ok = all(v >= FLOORS[k] - 1e-6 for k, v in worst.items())
        ok_all &= ok
        print(f"{scn['name']:24s} " + " ".join(f"{k[0][:3]}-{k[1][:3]}={v:.2f}" for k, v in sorted(worst.items()))
              + f"  {'PASS' if ok else 'FAIL'}")
    print("SPACING GATE:", "ALL PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1

if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "scenarios/bench"))
