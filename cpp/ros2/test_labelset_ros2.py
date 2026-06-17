#!/usr/bin/env python3
"""ROS2 端到端测试: conformal label-set 经 DynTraj.msg -> sando_node -> planner 派生软硬。

需要 sando_node 在跑(用 run_labelset_test.sh 起)。本脚本发若干障碍(覆盖 label 覆盖 id 启发式、
空集合退回 id 启发式含 id 边界、集合成员、other-only),订阅 sando_node 发的 obst_class_codes
(诊断话题, flat [id,code,...], code 1=hard 0=soft),逐条断言派生结果。

完备覆盖:
  - label 覆盖 id 启发式(两个方向: 把 id 该硬的压成软、把 id 该软的提成硬)
  - 空集合 -> 退回 id 启发式(id<200 硬 / id>=200 软),含边界 199/200 与 >=300
  - 集合成员(human 混在多类里 -> 硬)、other-only -> 软

断言只认【单条完整快照消息】(latest 整条, 必须一条消息里集齐所有 id), 这样将来若有障碍被
漏掉/掉帧/错配, 不会被跨快照累积的旧值掩盖成假绿。
"""
import sys
import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import DynTraj, State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

# (id, label_set, pos, expect_code, why)
OBSTACLES = [
    (5,   [1],   (3.0,  0.0, 2.0), 0, "label[1]=vehicle 覆盖 id<200(本应硬) -> 软"),
    (250, [0],   (4.0,  1.5, 2.0), 1, "label[0]=human 覆盖 id>=200(本应软) -> 硬"),
    (7,   [],    (5.0, -1.5, 2.0), 1, "空集合 -> id 启发式 id<200 -> 硬"),
    (260, [],    (6.0,  1.5, 2.0), 0, "空集合 -> id 启发式 id>=200 -> 软"),
    (8,   [2, 0],(7.0, -1.5, 2.0), 1, "human 混在 [2,0] 里 -> 硬"),
    (9,   [2],   (4.5,  2.5, 2.0), 0, "other-only [2] -> 软"),
    (199, [],    (8.0,  0.0, 2.0), 1, "空集合 id=199 边界 -> 硬"),
    (200, [],    (8.5,  1.5, 2.0), 0, "空集合 id=200 边界 -> 软"),
    (350, [],    (9.0, -1.5, 2.0), 0, "空集合 id>=300 -> 软 (退回老[200,300)会判硬 -> 此条失败)"),
]
EXPECT = {oid: code for (oid, _, _, code, _) in OBSTACLES}


def make_traj(oid, label_set, pos):
    m = DynTraj()
    m.id = oid
    m.is_agent = False
    m.label_set = [int(x) for x in label_set]
    m.bbox = [1.0, 1.0, 1.0]
    m.function = [f"{pos[0]}", f"{pos[1]}", f"{pos[2]}"]   # static analytic
    m.velocity = ["0.0", "0.0", "0.0"]
    m.pos.x, m.pos.y, m.pos.z = pos
    return m


def main():
    rclpy.init()
    node = Node("labelset_e2e_test")
    pub_trajs = node.create_publisher(DynTraj, "trajs", 50)
    pub_state = node.create_publisher(State, "state", 10)
    pub_goal = node.create_publisher(PoseStamped, "term_goal", 10)

    # store the parsed dict of THE LATEST single obst_class_codes message (not accumulated across
    # snapshots) — a real coherent snapshot must contain every expected id in ONE message.
    box = {"msg": {}}

    def on_codes(msg):
        d = list(msg.data)
        m = {}
        for i in range(0, len(d) - 1, 2):
            m[d[i]] = d[i + 1]
        box["msg"] = m

    node.create_subscription(Int32MultiArray, "obst_class_codes", on_codes, 10)

    st = State()
    st.pos.x, st.pos.y, st.pos.z = 0.0, 0.0, 2.0
    st.quat.w = 1.0
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 15.0, 0.0, 2.0
    goal.pose.orientation.w = 1.0
    trajs = [make_traj(oid, ls, pos) for (oid, ls, pos, _, _) in OBSTACLES]

    deadline = node.get_clock().now().nanoseconds + int(20e9)   # 20s budget
    got_all = False
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        pub_state.publish(st)
        pub_goal.publish(goal)
        for m in trajs:
            pub_trajs.publish(m)
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(oid in box["msg"] for oid in EXPECT):   # ONE message carried every expected id
            got_all = True
            break

    last = dict(box["msg"])
    print("\n=== ROS2 label-set 端到端结果 ===", flush=True)
    if not got_all:
        missing = [oid for oid in EXPECT if oid not in last]
        print(f"FAIL — 超时没在【单条】消息里收齐所有障碍。最近一条: {last}  缺: {missing}", flush=True)
        node.destroy_node(); rclpy.shutdown()
        return 1

    fails = 0
    for (oid, ls, pos, code, why) in OBSTACLES:
        got = last.get(oid)
        ok = (got == code)
        if not ok:
            fails += 1
        hs = {1: "硬", 0: "软"}
        print(f"  id={oid:<3} label={str(ls):<6} -> {hs.get(got,'?')}(got={got}) "
              f"want={hs[code]}({code})  {'PASS' if ok else 'FAIL'}   [{why}]", flush=True)

    print(f"\n{'ALL PASS — label-set 在 ROS2 真路径端到端生效' if fails == 0 else f'FAILED ({fails})'}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
