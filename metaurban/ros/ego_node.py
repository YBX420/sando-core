#!/usr/bin/env python3
"""ego_node — our de-ROS'd EGO-Planner flying MIGHTY's Gazebo sim (drop-in for mighty_node).

SYSTEM python3 + rclpy + ctypes (ego_capi.so via ego_bridge) — nothing to build. Speaks the
MIGHTY contract (namespace NX01, ROS_DOMAIN_ID=20):

  subs:  /NX01/state                 dynus_interfaces/State        (pos/vel/quat, frame map)
         /NX01/term_goal             geometry_msgs/PoseStamped     (mission goal, frame map)
         /NX01/mid360_PointCloud2    sensor_msgs/PointCloud2       (livox, frame NX01/NX01_livox -> TF to map)
  pubs:  /NX01/goal                  dynus_interfaces/Goal         (flat-output setpoint stream, 50 Hz)
         /NX01/ego_traj              nav_msgs/Path                 (committed B-spline, for RViz)

Loop: replan every 0.3 s from current state toward a horizon-capped local goal; stream setpoints
by evaluating the COMMITTED spline (anytime: a failed replan keeps flying the old commitment,
exhausted commitment -> hover). Far goals are chased EGO-style via local sub-goals on the line.

  ./ops/ego_mighty.sh          # after run_sim.py is up: kills mighty_node, starts this
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from dynus_interfaces.msg import Goal, State
import tf2_ros

MU = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, MU)
from ego_bridge import EGOPlanner                                   # noqa: E402  (ctypes, no deps)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class EgoNode(Node):
    def __init__(self):
        super().__init__("ego_node", namespace=os.environ.get("EGO_NS", "NX01"))
        self.declare_parameter("max_vel", 3.0)
        self.declare_parameter("max_acc", 6.0)
        self.declare_parameter("horizon", 7.5)
        self.declare_parameter("inflation", 0.4)
        self.declare_parameter("cruise_z", 3.0)
        self.declare_parameter("goal_x", float("nan"))   # set these to fly WITHOUT any term_goal
        self.declare_parameter("goal_y", float("nan"))   # publisher (standalone launch); a later
        self.declare_parameter("goal_z", float("nan"))   # term_goal message still overrides them
        gp = lambda k: float(self.get_parameter(k).value)
        # one big map covering the 105 m easy_forest corridor (char grid: ~28 MB, fine)
        self.ego = EGOPlanner(map_origin=(-15, -40, -0.5), map_size=(140, 80, 9),
                              res=0.15, inflation=gp("inflation"))
        self.ego.set_params(max_vel=gp("max_vel"), max_acc=gp("max_acc"), horizon=gp("horizon"))
        self.cruise_z = gp("cruise_z")
        self.horizon = gp("horizon")

        self.declare_parameter("accum_sec", 4.0)     # livox is a SPARSE rotating scan: one frame sees
        self.accum_sec = gp("accum_sec")             # a sliver of the forest. Keep a rolling window of
        self.accum = []                              # map-frame frames (plus the ACL mapper grid) so the
        self.occ_pts = None                          # planner's map is persistent, not last-slice-only.

        self.p = self.v = None
        self.goal = None
        if not math.isnan(gp("goal_x")):
            self.goal = np.array([gp("goal_x"), gp("goal_y"),
                                  gp("goal_z") if not math.isnan(gp("goal_z")) else self.cruise_z])
        self.cloud_msg = None
        self.a_est = np.zeros(3)
        self.commit_t = None
        self.traj_dur = 0.0
        self.yaw = 0.0
        self.reached = False

        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_subscription(State, "state", self.cb_state, 10)
        self.create_subscription(PoseStamped, "term_goal", self.cb_goal, 10)
        self.create_subscription(PointCloud2, "mid360_PointCloud2", self.cb_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "occupancy_grid", self.cb_occ, 5)
        self.pub_goal = self.create_publisher(Goal, "goal", 10)
        self.pub_path = self.create_publisher(Path, "ego_traj", 1)
        self.create_timer(0.30, self.replan)
        self.create_timer(0.02, self.command)
        self.get_logger().info("EGO node up: waiting for state + term_goal + cloud")

    def cb_state(self, m):
        self.p = np.array([m.pos.x, m.pos.y, m.pos.z])
        self.v = np.array([m.vel.x, m.vel.y, m.vel.z])

    def cb_goal(self, m):
        z = m.pose.position.z if m.pose.position.z > 0.2 else self.cruise_z
        g = np.array([m.pose.position.x, m.pose.position.y, z])
        if self.goal is None or np.linalg.norm(g - self.goal) > 0.5:
            self.get_logger().info(f"term_goal <- {np.round(g, 2).tolist()}")
            self.reached = False
        self.goal = g

    def _to_map(self, m):
        """PointCloud2 -> Nx3 map-frame points (identity if already in map), z/nan filtered."""
        try:
            tr = self.tfb.lookup_transform("map", m.header.frame_id, rclpy.time.Time())
        except Exception:
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        pts = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"),
                                             skip_nans=True).astype(np.float64)
        if pts.size == 0:
            return np.zeros((0, 3))
        pts = pts @ R.T + np.array([t.x, t.y, t.z])
        return pts[(pts[:, 2] > 0.3) & (pts[:, 2] < 7.5)]            # cut ground + canopy

    def cb_cloud(self, m):
        pts = self._to_map(m)
        if pts is None:
            return
        now = time.monotonic()
        self.accum.append((now, pts))
        while self.accum and now - self.accum[0][0] > self.accum_sec:
            self.accum.pop(0)

    def cb_occ(self, m):
        self.occ_pts = self._to_map(m)

    def cloud_in_map(self):
        parts = [f for _, f in self.accum]
        if self.occ_pts is not None:
            parts.append(self.occ_pts)
        if not parts:
            return None
        pts = np.concatenate(parts, axis=0)
        if self.p is not None:
            pts = pts[np.linalg.norm(pts[:, :2] - self.p[:2], axis=1) < 25.0]
        if len(pts):                                                  # 0.1 m voxel dedup, cap volume
            pts = np.unique(np.round(pts * 10.0).astype(np.int32), axis=0).astype(np.float64) / 10.0
        if len(pts) > 40000:
            pts = pts[:: len(pts) // 40000 + 1]
        return pts

    def replan(self):
        if self.p is None or self.goal is None or self.reached:
            return
        if np.linalg.norm((self.p - self.goal)[:2]) < 1.0:
            self.reached = True
            self.get_logger().info("GOAL REACHED — hovering")
            return
        pts = self.cloud_in_map()
        if pts is None:
            return                                                   # no cloud/TF yet
        self.ego.update_cloud(pts, cam_pos=self.p.tolist())
        d = self.goal - self.p
        dist = np.linalg.norm(d)
        local = self.goal if dist < self.horizon - 1 else self.p + d / dist * (self.horizon - 1)
        ok = self.ego.replan(self.p.tolist(), self.v.tolist(), self.a_est.tolist(), local.tolist())
        if ok:
            self.commit_t = time.monotonic()
            self.traj_dur = max(self.ego.duration() - 1e-3, 0.0)
            path = Path(); path.header.frame_id = "map"
            for k in range(21):
                r = self.ego.eval(self.traj_dur * k / 20.0)
                if r is None:
                    continue
                ps = PoseStamped(); ps.header.frame_id = "map"
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, r[0])
                path.poses.append(ps)
            self.pub_path.publish(path)
        else:
            self.get_logger().warn("replan FAILED — flying remaining commitment / hover",
                                   throttle_duration_sec=2.0)

    def command(self):
        if self.p is None:
            return
        g = Goal(); g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.power = True
        if self.reached and self.goal is not None:
            tgt, vel, acc = self.goal, np.zeros(3), np.zeros(3)
        elif self.commit_t is not None:
            te = min(time.monotonic() - self.commit_t, self.traj_dur)
            r = self.ego.eval(te)
            if r is None:
                tgt, vel, acc = self.p, np.zeros(3), np.zeros(3)
            else:
                tgt, vel, acc = (np.asarray(x, float) for x in r)
                self.a_est = acc
        else:
            tgt, vel, acc = self.p, np.zeros(3), np.zeros(3)         # no traj yet: hold
        if np.linalg.norm(vel[:2]) > 0.3:
            self.yaw = math.atan2(vel[1], vel[0])
        g.p.x, g.p.y, g.p.z = map(float, tgt)
        g.v.x, g.v.y, g.v.z = map(float, vel)
        g.a.x, g.a.y, g.a.z = map(float, acc)
        g.yaw = float(self.yaw)
        self.pub_goal.publish(g)


def main():
    rclpy.init()
    rclpy.spin(EgoNode())


if __name__ == "__main__":
    main()
