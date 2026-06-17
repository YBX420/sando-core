#!/usr/bin/env bash
# Run the ROS2 end-to-end label-set test: start sando_node headless, run the rclpy checker, clean up.
# (manual env export because colcon setup.bash does not propagate on the /media mount — see
#  .claude/memory/sando-core-ros2-run.md. ROS2 uses SYSTEM python; do NOT activate conda here.)
# NOTE: no `set -u` — ROS2's setup.bash references unset vars (AMENT_TRACE_SETUP_FILES) and would abort.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# conda must not shadow system python for ROS2
type conda >/dev/null 2>&1 && conda deactivate >/dev/null 2>&1
unset PYTHONPATH
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$WS/install/sando_cpp:$WS/install/dynus_interfaces:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$WS/install/sando_cpp/lib:$WS/install/dynus_interfaces/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$WS/install/dynus_interfaces/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"

# pkill -x matches the process NAME exactly (sando_node), never a shell whose *command line*
# merely mentions "sando_node" (a -f pattern would self-kill the caller). Safe.
cleanup() { kill "$NODE_PID" 2>/dev/null; pkill -x -9 sando_node 2>/dev/null; }
trap cleanup EXIT

ros2 run sando_cpp sando_node > /tmp/sando_node_test.log 2>&1 &
NODE_PID=$!

python3 "$WS/test_labelset_ros2.py"
RC=$?
exit $RC
