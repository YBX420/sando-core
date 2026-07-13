#!/usr/bin/env python3
"""state_to_rviz — bridge live_view's /state JSON into RViz2 markers (SYSTEM python + rclpy).

Runs OUTSIDE the conda env on purpose: the replay stack (conda, live_view.py --serve) exposes
plain JSON over HTTP; this node polls it at 10 Hz and publishes ONE MarkerArray. No conda x ROS
ABI mixing, no rebuild of anything. Frame: everything in fixed frame "map" (MetaUrban world m).

  source /opt/ros/humble/setup.bash
  python3 state_to_rviz.py [--url http://localhost:8090/state]

Markers: red cylinders = conformal keep-outs (faint = trust-window ghost), blue/orange = GT movers,
green line = committed B-spline, orange dashed-ish line = pre-certified escape branch (red flash
for 1.2 s when the escape tree FIRES = brake_esc/brake_stale increments), coloured points = trail
by decision kind, white sphere + yellow arrow = drone + velocity, floating text = HUD.
"""
import argparse
import json
import time
import urllib.request

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

KC = {"straight": (0.35, 0.86, 0.47), "around_l": (0.31, 0.78, 0.90), "around_r": (0.31, 0.78, 0.90),
      "over": (0.47, 0.55, 1.0), "climb": (0.47, 0.55, 1.0), "brake": (1.0, 0.67, 0.24),
      "cret": (0.78, 0.78, 0.47), "evade": (0.94, 0.31, 0.31), "cpl": (0.86, 0.43, 0.94),
      "native": (0.63, 0.63, 0.67), "sando": (0.63, 0.63, 0.67)}


def col(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def pt(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


class Bridge(Node):
    def __init__(self, url):
        super().__init__("sando_state_to_rviz")
        self.url = url
        self.pub = self.create_publisher(MarkerArray, "/sando/markers", 1)
        self.prev_esc = 0
        self.fire_until = 0.0
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f"polling {url} -> /sando/markers (fixed frame: map)")

    def mk(self, mid, mtype, ns="sando"):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.nanosec = 500_000_000          # stale markers vanish in 0.5 s
        return m

    def cylinder(self, mid, x, y, r, h, c):
        m = self.mk(mid, Marker.CYLINDER)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(h) / 2
        m.scale.x = m.scale.y = 2 * float(r); m.scale.z = float(h)
        m.color = c
        return m

    def strip(self, mid, pts3, c, width):
        m = self.mk(mid, Marker.LINE_STRIP)
        m.scale.x = width; m.color = c
        m.points = [pt(*q) for q in pts3]
        return m

    def tick(self):
        try:
            s = json.loads(urllib.request.urlopen(self.url, timeout=1.0).read())
        except Exception:
            return
        arr, mid = MarkerArray(), 0

        counts = s.get("counts") or {}
        esc_n = counts.get("brake_esc", 0) + counts.get("brake_stale", 0)
        if esc_n > self.prev_esc:
            self.fire_until = time.time() + 1.2
        self.prev_esc = esc_n
        firing = time.time() < self.fire_until

        for c in (s.get("cyl") or []):                                   # keep-outs + ghosts
            cx, cy, vx, vy, r, veff = c
            arr.markers.append(self.cylinder(mid, cx, cy, r, 2.5, col(0.92, 0.31, 0.31, 0.28))); mid += 1
            tau = s.get("tau", 0.75)
            arr.markers.append(self.cylinder(mid, cx + vx * tau, cy + vy * tau,
                                             r + veff * tau, 2.5, col(0.55, 0.16, 0.16, 0.10))); mid += 1
        for mv in (s.get("movers") or []):                               # GT movers
            x, y, r, h, cls = mv
            c = col(1.0, 0.65, 0.35, 0.65) if cls == "vehicle" else col(0.43, 0.69, 1.0, 0.65)
            arr.markers.append(self.cylinder(mid, x, y, r, h, c)); mid += 1
        if s.get("traj"):
            arr.markers.append(self.strip(mid, s["traj"], col(0.31, 0.90, 0.51), 0.08)); mid += 1
        if s.get("esc"):
            c = col(1.0, 0.25, 0.25) if firing else col(1.0, 0.71, 0.24, 0.9)
            arr.markers.append(self.strip(mid, s["esc"], c, 0.14 if firing else 0.08)); mid += 1
        if s.get("trail"):
            m = self.mk(mid, Marker.POINTS); mid += 1
            m.scale.x = m.scale.y = 0.12
            for q in s["trail"]:
                m.points.append(pt(q[0], q[1], q[2]))
                m.colors.append(col(*KC.get(q[3], (0.8, 0.8, 0.8))))
            arr.markers.append(m)
        if s.get("p"):
            p, v = s["p"], s.get("v") or [0, 0, 0]
            m = self.mk(mid, Marker.SPHERE); mid += 1
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
            m.scale.x = m.scale.y = m.scale.z = 0.5; m.color = col(0.96, 0.96, 0.96)
            arr.markers.append(m)
            m = self.mk(mid, Marker.ARROW); mid += 1                     # velocity
            m.scale.x, m.scale.y, m.scale.z = 0.06, 0.14, 0.1
            m.points = [pt(*p), pt(p[0] + v[0] * 0.8, p[1] + v[1] * 0.8, p[2] + v[2] * 0.8)]
            m.color = col(0.98, 0.86, 0.35)
            arr.markers.append(m)
            m = self.mk(mid, Marker.TEXT_VIEW_FACING); mid += 1          # HUD
            m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
            m.pose.position.z = float(p[2]) + 2.2
            m.scale.z = 0.55
            cs = " ".join(f"{k}:{v}" for k, v in counts.items() if v)
            clr = s.get("clr")
            m.text = (f"t={s.get('t', 0):.1f}s {s.get('kind', '-')}"
                      f" clr={'--' if clr is None else f'{clr:.2f}m'}"
                      f" rt={s.get('rtf') or 0:.2f}x\n{cs}\n"
                      f"esc: {'FIRED!' if firing else ('ARMED' if s.get('esc') else 'none')}")
            m.color = col(1, 0.3, 0.3) if firing else col(0.82, 0.87, 0.92)
            arr.markers.append(m)
        # goal flag
        g = s.get("goal")
        if g:
            arr.markers.append(self.cylinder(mid, g[0], g[1], 0.8, 0.05, col(0.35, 0.86, 0.47, 0.7))); mid += 1
        self.pub.publish(arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8090/state")
    args = ap.parse_args()
    rclpy.init()
    rclpy.spin(Bridge(args.url))


if __name__ == "__main__":
    main()
