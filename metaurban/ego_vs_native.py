"""ego_vs_native — does KNOWING the future let us fly FASTER and stay SAFE?

A/B over a max-speed sweep on dynamic-crossing scenes:
  native = plain EGO reacting to the human's CURRENT position (no prediction, no certificate).
  ours   = KF-predicted + cylinder-certified fastest-safe maneuvering (ego_maneuver.run_episode).

Thesis (塔菲大人): "because I know how the human will move, I dare to fly faster but safe." So as the speed cap
rises, NATIVE — surprised by a human stepping into the path it planned through the now-empty gap — should start
COLLIDING (min_clr<0) or stall, while OURS stays safe (min_clr>=0) and reaches the goal in less time.

Run:  python metaurban/ego_vs_native.py
      python metaurban/ego_vs_native.py crossers head_on        # pick scenes
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ego_maneuver import run_episode

VELS = [float(v) for v in os.environ.get("EGO_VELS", "3,5,8,12").split(",")]
SCENES = sys.argv[1:] or ["crossers", "head_on"]


def _row(m):
    reached = "Y" if m["reached"] else "N"
    coll = "COLLIDE" if m["collided"] else "safe"
    return (f"  {m['mode']:7s} vmax={m['max_vel']:5.1f}  reached={reached}  "
            f"time={m['time_s']:5.1f}s  mean_v={m['mean_speed']:5.2f}  "
            f"min_clr={m['min_clr']:+.3f}  {coll}")


def main():
    print("=== ego_vs_native: faster-because-I-predict, A/B over a max-speed sweep ===")
    for scene in SCENES:
        print(f"\n--- scene '{scene}' ---")
        for v in VELS:
            nat = run_episode("native", max_vel=v, scene_name=scene)
            our = run_episode("ours", max_vel=v, scene_name=scene)
            print(_row(nat))
            print(_row(our))
            # head-to-head verdict at this speed cap
            if our["reached"] and not our["collided"]:
                if nat["collided"]:
                    verdict = "OURS safe, NATIVE collides"
                elif not nat["reached"]:
                    verdict = "OURS reaches, NATIVE stalls"
                elif our["time_s"] < nat["time_s"] - 1e-6:
                    verdict = f"OURS faster by {nat['time_s'] - our['time_s']:.1f}s, both safe"
                elif our["time_s"] > nat["time_s"] + 1e-6:
                    verdict = f"NATIVE faster by {our['time_s'] - nat['time_s']:.1f}s (both safe)"
                else:
                    verdict = "tie"
            else:
                verdict = "ours failed (investigate)"
            print(f"    -> {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
