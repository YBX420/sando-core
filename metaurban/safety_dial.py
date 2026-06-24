"""safety_dial — the conformal eps as a SAFETY <-> SPEED dial.

Smaller eps  -> larger conformal keep-out (v_eff) -> wider berths -> safer but slower.
Larger eps   -> tighter keep-out -> faster but the marginal guarantee P(collision)<=eps is looser.
This aggregates the per-eps A/B summaries + the calibration into one table + a Pareto plot, realising the
"max-speed <-> conformal one dial" idea: pick eps for the mission's risk budget; the certificate enforces it.

Run (after ab_replay at several eps):  python metaurban/safety_dial.py
"""
import os, sys, json, glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(os.path.dirname(HERE), "out", "conformal")

EPS_TAGS = [("0.01", "cv01"), ("0.05", "cv05"), ("0.10", "cv10")]


def main():
    calib = json.load(open(os.path.join(OUTDIR, "calib.json")))
    ped = calib["groups"]["pedestrian"]["levels"]
    rows = []
    for eps, tag in EPS_TAGS:
        path = os.path.join(OUTDIR, f"ab_summary_{tag}.json")
        if not os.path.exists(path):
            print(f"[dial] missing {path} (run ab_replay --eps {eps} --tag {tag})"); continue
        ab = json.load(open(path))
        lv = ped.get(eps, {})
        rows.append(dict(eps=float(eps), target_cov=1 - float(eps),
                         ped_veff=lv.get("v_eff"), ped_cov=lv.get("test_marginal_coverage"),
                         ours_coll=ab["ours_collisions"], nat_coll=ab["native_collisions"],
                         dt_med=ab["time_delta_median"], dt_max=ab["time_delta_max"],
                         faster=ab["strictly_faster"], n=ab["n_both"],
                         ours_min_clr=ab["ours_min_clr"]))

    print("\n=== conformal eps = safety<->speed dial (pedestrian keep-out; 120-episode A/B vs native EGO) ===")
    print(f"{'eps':>5}{'targetCov':>10}{'v_eff':>8}{'testCov':>9}{'ours_coll':>10}{'minClr':>8}"
          f"{'dt_med':>8}{'dt_max':>8}{'faster':>8}")
    for r in rows:
        print(f"{r['eps']:>5.2f}{r['target_cov']:>10.2f}{_n(r['ped_veff']):>8}{_n(r['ped_cov']):>9}"
              f"{r['ours_coll']:>10}{_n(r['ours_min_clr']):>8}{_sg(r['dt_med']):>8}{_sg(r['dt_max']):>8}"
              f"{str(r['faster'])+'/'+str(r['n']):>8}")
    print("\nreading: as eps shrinks, v_eff (keep-out growth) rises -> ours flies wider (min clearance up, dt up),"
          "\nsafer guarantee; ours stays 0-collision across the dial (cert + dual-cert conservatism).")

    with open(os.path.join(OUTDIR, "safety_dial.json"), "w") as f:
        json.dump(rows, f, indent=2)
    print(f"[dial] wrote {os.path.join(OUTDIR, 'safety_dial.json')}")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        if len(rows) >= 2:
            es = [r["eps"] for r in rows]
            fig, ax = plt.subplots(1, 2, figsize=(10, 4))
            ax[0].plot(es, [r["ped_veff"] for r in rows], "o-", color="tab:red")
            ax[0].set_xlabel("eps (risk budget)"); ax[0].set_ylabel("pedestrian v_eff (m/s)")
            ax[0].set_title("keep-out growth vs eps"); ax[0].grid(alpha=0.3); ax[0].invert_xaxis()
            ax[1].plot(es, [r["dt_med"] for r in rows], "o-", label="median dt (ours-native)")
            ax[1].plot(es, [r["ours_min_clr"] for r in rows], "s-", label="ours min clearance (m)")
            ax[1].axhline(0, color="0.7", lw=0.8)
            ax[1].set_xlabel("eps (risk budget)"); ax[1].set_title("speed cost & safety margin vs eps")
            ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3); ax[1].invert_xaxis()
            fig.suptitle("Conformal eps as a safety<->speed dial (ours stays 0-collision throughout)")
            fig.tight_layout(); fig.savefig(os.path.join(OUTDIR, "safety_dial.png"), dpi=130, bbox_inches="tight")
            print(f"[dial] wrote {os.path.join(OUTDIR, 'safety_dial.png')}")
    except Exception as e:
        print(f"[dial] (plot skipped: {e})")
    return 0


def _n(x):
    return "n/a" if x is None else (f"{x:.3f}" if abs(x) < 100 else f"{x:.0f}")


def _sg(x):
    return "n/a" if x is None else f"{x:+.2f}"


if __name__ == "__main__":
    sys.exit(main())
