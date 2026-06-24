#!/usr/bin/env bash
# px4_sitl.sh — robust start/stop/restart for PX4 SITL + jmavsim, baking in tonight's hard-won lessons:
#   * a STALE lockstep jmavsim holding :4560 corrupts every later run -> stop must kill the wrapper's whole
#     process GROUP (it owns the java child) + any leftover java + the -d px4 DAEMON (named 'px4', not 'bin/px4').
#   * px4 -d forks a daemon; check liveness with `pgrep -ax px4`, NOT a 'bin/px4' pattern (false negatives).
#   * after 'Ready for takeoff' the EKF/GPS still needs ~30-60 s to fully converge, else arming is denied.
#   * bracket-pgrep ([p]x4) so the script never matches its own command line.
#
# Usage:  bash metaurban/px4_sitl.sh {start|stop|restart|status|wait}
#   start   : bring up jmavsim then px4, wait for 'Ready for takeoff'
#   wait    : additionally block ~EKF_WAIT s so the EKF converges before you connect the bridge
#   restart : stop + start
set -u

PX4_DIR=/media/boxuan/Data2/projects/PX4-Autopilot
LOGDIR=/media/boxuan/Data2/projects/sando_py/sando-core/logs
JMAV_LOG=$LOGDIR/jmavsim.log
PX4_LOG=$LOGDIR/px4_bin.log
EKF_WAIT=${EKF_WAIT:-55}
mkdir -p "$LOGDIR"

_alive_px4() { pgrep -ax px4 | grep -c 'bin/px4'; }
_alive_jmav() { pgrep -f '[j]mavsim_run.jar' | wc -l; }
_port4560()  { (ss -lntu 2>/dev/null || true) | grep -c ':4560'; }

stop() {
  # kill jmavsim_run.sh wrappers by process GROUP (takes the java child with them), then any stragglers
  for pid in $(pgrep -f '[j]mavsim_run.sh'); do
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' '); [ -n "$pgid" ] && kill -9 -"$pgid" 2>/dev/null
  done
  pkill -9 -x java 2>/dev/null
  for pid in $(pgrep -ax px4 | grep 'bin/px4' | awk '{print $1}'); do kill -9 "$pid" 2>/dev/null; done
  sleep 3
  echo "[px4_sitl] stopped: java=$(pgrep -x java | wc -l) px4=$(_alive_px4) port4560=$(_port4560)"
}

start() {
  if [ "$(_port4560)" -ne 0 ]; then
    echo "[px4_sitl] :4560 busy -> stopping first"; stop
  fi
  echo "[px4_sitl] launching jmavsim ..."
  ( cd "$PX4_DIR" && HEADLESS=1 setsid ./Tools/simulation/jmavsim/jmavsim_run.sh > "$JMAV_LOG" 2>&1 & )
  for i in $(seq 1 20); do [ "$(_port4560)" -ne 0 ] && break; sleep 1; done
  sleep 3
  echo "[px4_sitl] jmavsim java=$(_alive_jmav) :4560=$(_port4560); launching px4 ..."
  : > "$PX4_LOG"
  ( cd "$PX4_DIR/build/px4_sitl_default" && HEADLESS=1 PX4_SIM_MODEL=jmavsim_iris \
      setsid ./bin/px4 -d -s etc/init.d-posix/rcS > "$PX4_LOG" 2>&1 & )
  for i in $(seq 1 40); do grep -q 'Ready for takeoff' "$PX4_LOG" 2>/dev/null && break; sleep 1; done
  if grep -q 'Ready for takeoff' "$PX4_LOG"; then
    echo "[px4_sitl] PX4 READY (px4=$(_alive_px4) java=$(_alive_jmav))"
  else
    echo "[px4_sitl] PX4 did NOT reach 'Ready for takeoff' (check $PX4_LOG)"; return 1
  fi
}

wait_ekf() {
  echo "[px4_sitl] waiting ${EKF_WAIT}s for EKF/GPS to converge (avoids 'Arming denied: health failures') ..."
  for i in $(seq 1 "$EKF_WAIT"); do
    [ "$(_alive_px4)" -eq 0 ] && { echo "[px4_sitl] WARN: px4 died during EKF wait (see $PX4_LOG)"; return 1; }
    sleep 1
  done
  echo "[px4_sitl] EKF wait done; px4=$(_alive_px4) java=$(_alive_jmav)"
}

status() {
  echo "[px4_sitl] px4_alive=$(_alive_px4) jmavsim_alive=$(_alive_jmav) port4560=$(_port4560) " \
       "ready=$(grep -c 'Ready for takeoff' "$PX4_LOG" 2>/dev/null)"
  grep -iE 'arming denied|failsafe|disarmed|battery warning|connection to ground' "$PX4_LOG" 2>/dev/null | tail -3
}

case "${1:-status}" in
  start)   start ;;
  stop)    stop ;;
  restart) stop; start ;;
  wait)    start && wait_ekf ;;
  status)  status ;;
  *) echo "usage: $0 {start|stop|restart|status|wait}"; exit 2 ;;
esac
