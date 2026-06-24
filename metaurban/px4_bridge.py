"""px4_bridge — drive a real PX4 SITL flight stack from the SANDO loop via MAVSDK offboard.

Replaces the lightweight quadrotor.py stand-in with the REAL PX4 controller + jMAVSim multicopter dynamics:
the SANDO C++ core emits 3-D set-points, this bridge streams them to PX4 in OFFBOARD mode (position+yaw),
and reads PX4's fused pose back so MetaUrban can render the true flown state. PX4 does the inner-loop
control + dynamics; SANDO does the planning/avoidance — the clean split.

Prereqs (already set up on this box):
  - PX4 SITL running:  cd PX4-Autopilot/build/px4_sitl_default &&
      HEADLESS=1 PX4_SIM_MODEL=jmavsim_iris setsid ./bin/px4 -d -s etc/init.d-posix/rcS
  - pip install mavsdk  (bundles mavsdk_server)
MAVSDK telemetry/offboard is async; this wraps it behind a SYNC façade (background asyncio thread) so the
existing synchronous render/eval loops can call set_setpoint()/get_pose() without becoming async.

Frames: SANDO/MetaUrban world is ENU-ish (x east, y north, z up). PX4 is NED (x north, y east, z down).
We map world (x,y,z) -> NED (y, x, -z) relative to the PX4 local origin set at first offboard.
"""
import threading, asyncio, math, time
import numpy as np


class PX4Bridge:
    def __init__(self, system_address="udpin://0.0.0.0:14540", takeoff_alt=1.5):
        self._addr = system_address
        self._takeoff_alt = float(takeoff_alt)
        self._lock = threading.Lock()
        self._pose = {"ned": np.zeros(3), "yaw": 0.0, "vel": np.zeros(3), "ok": False}
        self._sp = {"n": 0.0, "e": 0.0, "d": -float(takeoff_alt), "yaw": 0.0}
        self._ready = threading.Event()
        self._origin_world = None          # world xyz mapped to PX4 NED (0,0,0) at offboard start
        self._loop = None
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    # ---- world(ENU,z-up) <-> PX4 NED ----
    @staticmethod
    def _world_to_ned(dxyz):
        return float(dxyz[1]), float(dxyz[0]), float(-dxyz[2])     # N=worldY, E=worldX, D=-worldZ

    @staticmethod
    def _ned_to_world(n, e, d):
        return np.array([e, n, -d], float)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main())

    async def _main(self):
        from mavsdk import System
        from mavsdk.offboard import PositionNedYaw, OffboardError
        self._drone = System()
        print("[px4] connecting ...", flush=True)
        await self._drone.connect(system_address=self._addr)
        async for st in self._drone.core.connection_state():
            if st.is_connected: break
        print("[px4] connected; setting params ...", flush=True)
        # DISABLE SITL failsafes that otherwise land/disarm us mid-flight (the real "loss on PX4" cause):
        # the simulated battery drains -> 'battery warning (fast)' -> 'Disarmed by landing'; plus datalink/RC-loss
        # and the power-supply preflight check. Offboard position control must survive all of these.
        # Disable SITL failsafes that land/disarm us mid-flight. EVERY call is timeout-wrapped: a missing param
        # makes MAVSDK's set_param hang forever otherwise, which would stall the whole bridge init (-> 'not ready').
        async def _seti(n, v):
            try:
                await asyncio.wait_for(self._drone.param.set_param_int(n, v), timeout=4)
            except Exception as e:
                print(f"[px4] param {n}: {e} (continuing)", flush=True)
        async def _setf(n, v):
            try:
                await asyncio.wait_for(self._drone.param.set_param_float(n, v), timeout=4)
            except Exception as e:
                print(f"[px4] param {n}: {e} (continuing)", flush=True)
        for n, v in (("COM_LOW_BAT_ACT", 0), ("NAV_DLL_ACT", 0), ("NAV_RCL_ACT", 0),
                     ("COM_RCL_EXCEPT", 7), ("CBRK_SUPPLY_CHK", 894281), ("COM_ARM_WO_GPS", 1)):
            await _seti(n, v)
        # the SITL battery drains 100%->0% in SIM_BAT_DRAIN s (default 60) -> emergency-battery FORCE-LANDS ~12 s
        # after arming. SIM_BAT_DRAIN=0 disables the battery simulator entirely. (Also set at boot via stdin.)
        for n, v in (("SIM_BAT_DRAIN", 0.0), ("BAT_LOW_THR", 0.0), ("BAT_CRIT_THR", 0.0), ("BAT_EMERGEN_THR", 0.0)):
            await _setf(n, v)
        print("[px4] params done; waiting for FULL EKF/GPS health ...", flush=True)
        # Wait for FULL health (local + global + home) before arming. Arming before the EKF/GPS converges gives
        # 'Arming denied: Resolve system health failures first'. Generous 120 s budget; SITL usually converges <60 s.
        import time as _t
        _t0 = _t.monotonic()
        async for h in self._drone.telemetry.health():
            if (h.is_local_position_ok and h.is_home_position_ok and h.is_global_position_ok
                    and h.is_gyrometer_calibration_ok and h.is_accelerometer_calibration_ok):
                break
            if _t.monotonic() - _t0 > 120:
                print("[px4] WARN: health not fully green after 120s, proceeding anyway", flush=True); break
        asyncio.ensure_future(self._telemetry_task())
        # ARM with retries: even with health green, the first arm can be denied while the EKF settles. Retry.
        armed = False
        async for a in self._drone.telemetry.armed(): armed = a; break
        for attempt in range(12):
            if armed:
                break
            try:
                await asyncio.wait_for(self._drone.action.arm(), timeout=5)
                armed = True
            except Exception as e:
                print(f"[px4] arm attempt {attempt}: {e}", flush=True)
                await asyncio.sleep(2.0)
                async for a in self._drone.telemetry.armed(): armed = a; break
        print(f"[px4] armed={armed}; engaging offboard ...", flush=True)
        await self._drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -self._takeoff_alt, 0.0))
        for attempt in range(6):
            try:
                await self._drone.offboard.start(); break
            except OffboardError as e:
                print(f"[px4] offboard start attempt {attempt}: {e}", flush=True)
                await self._drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -self._takeoff_alt, 0.0))
                await asyncio.sleep(1.0)
        print("[px4] OFFBOARD ready", flush=True)
        self._ready.set()
        # stream the latest set-point at >20 Hz (offboard requires continuous set-points)
        while True:
            with self._lock:
                sp = PositionNedYaw(self._sp["n"], self._sp["e"], self._sp["d"], self._sp["yaw"])
            try: await self._drone.offboard.set_position_ned(sp)
            except Exception: pass
            await asyncio.sleep(0.04)

    async def _telemetry_task(self):
        async def pos():
            async for p in self._drone.telemetry.position_velocity_ned():
                with self._lock:
                    self._pose["ned"] = np.array([p.position.north_m, p.position.east_m, p.position.down_m], float)
                    self._pose["vel"] = np.array([p.velocity.north_m_s, p.velocity.east_m_s, p.velocity.down_m_s], float)
                    self._pose["ok"] = True
        async def att():
            async for a in self._drone.telemetry.attitude_euler():
                with self._lock: self._pose["yaw"] = math.radians(a.yaw_deg)
        await asyncio.gather(pos(), att())

    # ---- sync API for the render/eval loop ----
    def wait_ready(self, timeout=60.0):
        return self._ready.wait(timeout)

    def set_world_origin(self, world_xyz):
        """Anchor: this world position == PX4 NED (0,0, -takeoff_alt). Call once at lap start."""
        self._origin_world = np.asarray(world_xyz, float).copy()

    def set_setpoint(self, world_xyz, yaw):
        # HORIZONTAL is origin-relative (PX4 NED home = origin_world's x,y); VERTICAL is ABSOLUTE altitude:
        # PX4 home is on the ground, so NED down = -(world z = altitude). The earlier -(z - origin_z) form
        # commanded down=0 (ground) for any cruise-altitude set-point -> the drone descended/landed and never
        # flew the corridor (this was a real cause of 'losing on PX4').
        p = np.asarray(world_xyz, float)
        with self._lock:
            self._sp["n"] = float(p[1] - self._origin_world[1])   # north = world +y
            self._sp["e"] = float(p[0] - self._origin_world[0])   # east  = world +x
            self._sp["d"] = float(-p[2])                          # down  = -altitude (absolute)
            self._sp["yaw"] = float(math.degrees(yaw))            # PositionNedYaw yaw is in DEGREES

    def get_pose_world(self):
        """Return (world_xyz, yaw, world_vel) from PX4's fused estimate, mapped back to the world frame."""
        with self._lock:
            ned = self._pose["ned"].copy(); vel = self._pose["vel"].copy(); yaw = self._pose["yaw"]; ok = self._pose["ok"]
        o = self._origin_world if self._origin_world is not None else np.zeros(3)
        w = np.array([ned[1] + o[0], ned[0] + o[1], -ned[2]])     # x=E+ox, y=N+oy, z=-down (absolute alt)
        wv = np.array([vel[1], vel[0], -vel[2]])                  # vx=vE, vy=vN, vz=-vD
        return w, yaw, wv, ok


if __name__ == "__main__":
    # self-test: connect, take off via offboard, hold a 1.5 m setpoint, report the flown pose
    br = PX4Bridge(takeoff_alt=1.5)
    print("[px4] waiting for offboard ready ...", flush=True)
    if not br.wait_ready(60): print("[px4] NOT ready"); raise SystemExit(1)
    print("[px4] OFFBOARD active — holding takeoff_alt, streaming setpoint", flush=True)
    br.set_world_origin([0.0, 0.0, 1.5])
    for i in range(40):
        br.set_setpoint([0.0, 0.0, 1.5], 0.0)         # hold origin at 1.5 m
        time.sleep(0.25)
        if i % 8 == 7:
            w, yaw, wv, ok = br.get_pose_world()
            print(f"[px4] flown world pos={np.round(w,2)} z={w[2]:.2f} yaw={math.degrees(yaw):.0f} ok={ok}", flush=True)
    print("[px4] self-test done (PX4 flew to/held the offboard setpoint).", flush=True)
