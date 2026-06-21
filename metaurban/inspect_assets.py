"""Free-fly asset inspector for the MetaUrban × SANDO demo.

Spawns the custom assets (drone + dog/cow/sheep/cat) in a tidy row inside the real MetaUrban scene,
each with a GREEN forward-arrow (the +Y / heading direction the demo treats as "forward") and a RED
+X arrow, plus a world axis gnomon. You fly a FREE camera around them IN A BROWSER (WASD + R/F up-down
+ arrow keys to look) so you can check every asset's true orientation before we lock the config.

Why a browser: DISPLAY :1 here is software-GL (llvmpipe) — cv2/onscreen windows are black. The offscreen
RGB camera renders on the GPU fine, so we stream it as MJPEG over HTTP; the browser also gives us keyboard
input back to drive the free camera.

Run (metaurban env, from the metaurban repo root), then open http://localhost:8089/ :
  cd /media/boxuan/Data21/projects/metaurban
  DISPLAY=:1 ~/miniconda3/envs/metaurban/bin/python \
      /media/boxuan/Data21/projects/sando_py/sando-core/metaurban/inspect_assets.py --seed 3
"""
import os, sys, time, argparse, threading
import cv2  # import BEFORE panda3d/metaurban (GL/X symbol clash otherwise)
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from panda3d.core import LineSegs, NodePath

_HERE = os.path.dirname(os.path.abspath(__file__))
from metaurban import SidewalkDynamicMetaUrbanEnv
from metaurban.component.sensors.rgb_camera import RGBCamera
sys.path.insert(0, _HERE)
from custom_glb_object import make_glb_class, make_drone_class

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=3)
ap.add_argument("--w", type=int, default=960)
ap.add_argument("--h", type=int, default=600)
ap.add_argument("--port", type=int, default=8089)
args = ap.parse_args()

ASSETS = "/media/boxuan/Data21/projects/metaurban/custom_assets/"
# name -> (kind, glb_or_None, [w,l,h], hpr_fix)   kind: 'drone'=procedural quad, 'glb'=mesh
# hpr_fix (deg) stands each asset up facing +Y (heading 0). Tuned from the close-up inspector.
# Keep it simple: just the drone (procedural quad) + the cow (only animal, per request).
INSPECT = [
    ("drone", "drone", None,                 [1.1, 1.1, 0.3], (0.0, 0.0, 0.0)),
    ("cow",   "glb",   "cow_quaternius.glb", [0.9, 2.6, 1.6], (0.0, -90.0, 0.0)),
]

env_cfg = dict(
    crswalk_density=1, object_density=0.9, walk_on_all_regions=False,
    use_render=False, image_observation=True, sensors=dict(rgb_camera=(RGBCamera, args.w, args.h)),
    interface_panel=[], manual_control=False, map='X', daytime="12:00",
    default_expert=False, drivable_area_extension=55, height_scale=1,
    show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=100000,
    on_continuous_line_done=False, out_of_route_done=False,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                        show_dest_mark=False, enable_reverse=True),
    show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
    num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
    crash_vehicle_done=False, crash_object_done=False, crash_human_done=False, traffic_density=0.45,
    spawn_human_num=40, spawn_wheelchairman_num=3, spawn_edog_num=6, spawn_erobot_num=3,
    spawn_drobot_num=3, max_actor_num=80)

print("[inspect] constructing offscreen env ...", flush=True)
env = SidewalkDynamicMetaUrbanEnv(env_cfg)
env.reset(seed=args.seed)
for _ in range(8): env.step([0.0, 0.0])
eng = env.engine; ego = env.agent
cam = eng.get_sensor("rgb_camera")

# place the asset row near a pedestrian cluster (= a real, on-ground sidewalk spot)
peds = []
for _, o in eng.get_objects().items():
    try:
        p = np.asarray(o.position, float)
        if p.shape[0] >= 2 and np.all(np.isfinite(p)) and o is not ego: peds.append(p[:2])
    except Exception: pass
CTR = (np.mean(peds, axis=0) if peds else np.asarray(ego.position[:2], float))
ROW_DIR = np.array([1.0, 0.0])      # lay the row out along world +X
GAP = 4.0

spawned = []
for i, (name, kind, glb, size, hpr_fix) in enumerate(INSPECT):
    pos = CTR + ROW_DIR * (i - (len(INSPECT) - 1) / 2.0) * GAP
    if kind == "drone":
        cls = make_drone_class(name, span=float(size[1])); z = 1.0           # drone hovers
    else:
        cls = make_glb_class(name, ASSETS + glb, size[0], size[1], size[2], hpr_fix=hpr_fix); z = 0.0  # grounded
    obj = eng.spawn_object(cls, position=[float(pos[0]), float(pos[1])], heading_theta=0.0)  # heading 0 = facing +Y
    obj.set_position([float(pos[0]), float(pos[1]), float(z)])
    spawned.append((name, np.array([pos[0], pos[1], max(z, size[2] * 0.5)]), size))
    print(f"[inspect] spawned {name:6s} at {np.round(pos,1)} size={size} hpr_fix={hpr_fix}", flush=True)

# ---- overlay: per-asset forward arrows + world gnomon (heading 0 -> we treat +Y as forward) -------
ov = NodePath("inspect_ov"); ov.reparentTo(eng.render)
ov.setLightOff(1); ov.setShaderOff(1); ov.setDepthTest(False); ov.setDepthWrite(False); ov.setBin("fixed", 60)


def _line(pts, color, thick):
    ls = LineSegs(); ls.setThickness(thick); ls.setColor(*color)
    for i, p in enumerate(pts):
        (ls.moveTo if i == 0 else ls.drawTo)(float(p[0]), float(p[1]), float(p[2]))
    return ls.create()


def _arrow(base, vec, color, thick=5.0):
    b = np.asarray(base, float); v = np.asarray(vec, float); tip = b + v
    ov.attachNewNode(_line([b, tip], color, thick))
    # little chevron at the tip
    side = np.array([-v[1], v[0], 0.0]); side = side / (np.linalg.norm(side) + 1e-9) * 0.25 * np.linalg.norm(v)
    back = b + v * 0.75
    ov.attachNewNode(_line([tip, back + side], color, thick))
    ov.attachNewNode(_line([tip, back - side], color, thick))


for name, c, size in spawned:
    L = max(size) * 1.3 + 0.6
    _arrow([c[0], c[1], 0.1], [0.0, L, 0.0], (0.1, 1.0, 0.2, 1.0))   # GREEN = +Y = "forward" (heading 0)
    _arrow([c[0], c[1], 0.1], [L * 0.6, 0.0, 0.0], (1.0, 0.2, 0.2, 1.0))  # RED = +X
# world gnomon at row centre
_arrow([CTR[0], CTR[1], 0.05], [3.0, 0.0, 0.0], (1.0, 0.0, 0.0, 1.0), 7)  # X red
_arrow([CTR[0], CTR[1], 0.05], [0.0, 3.0, 0.0], (0.0, 1.0, 0.0, 1.0), 7)  # Y green
ov.attachNewNode(_line([(CTR[0], CTR[1], 0.05), (CTR[0], CTR[1], 3.0)], (0.3, 0.5, 1.0, 1.0), 7))  # Z blue

# ---- free camera state, driven by browser keys over HTTP -----------------------------------------
# start a few metres back (-Y) and up, looking forward (+Y) at the asset row
CAM = {"pos": [float(CTR[0]), float(CTR[1] - 10.0), 3.0], "h": 0.0, "p": -12.0}
KEYS = {"s": set(), "t": 0.0}
FONT = cv2.FONT_HERSHEY_SIMPLEX
_FRAME = {"jpg": None}; _LOCK = threading.Lock()
_PAGE = ("""<!doctype html><html><head><meta charset=utf-8><title>asset inspector</title>
<style>html,body{margin:0;background:#0b0b10;color:#ccc;font-family:sans-serif}
img{display:block;margin:0 auto;max-width:100vw}#hud{position:fixed;top:6px;left:8px;font-size:13px;
background:rgba(0,0,0,.55);padding:6px 9px;border-radius:6px}</style></head>
<body><div id=hud>WASD move &nbsp; R/F up·down &nbsp; arrows look &nbsp; <b>click image first</b></div>
<img id=v src="/stream" tabindex=0>
<script>
const keys=new Set();
const map={KeyW:'w',KeyA:'a',KeyS:'s',KeyD:'d',KeyR:'r',KeyF:'f',
ArrowUp:'up',ArrowDown:'down',ArrowLeft:'left',ArrowRight:'right',Space:'r',ShiftLeft:'f',
Digit0:'0',Digit1:'1',Digit2:'2',Digit3:'3',Digit4:'4',Digit5:'5'};
addEventListener('keydown',e=>{if(map[e.code]){keys.add(map[e.code]);e.preventDefault();}});
addEventListener('keyup',e=>{if(map[e.code]){keys.delete(map[e.code]);e.preventDefault();}});
setInterval(()=>{fetch('/ctrl?k='+[...keys].join(','))},50);
</script></body></html>""").encode()


class _H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            self.send_response(200); self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(_PAGE))); self.end_headers(); self.wfile.write(_PAGE); return
        if u.path == "/ctrl":
            q = parse_qs(u.query); k = q.get("k", [""])[0]
            with _LOCK:
                KEYS["s"] = set(x for x in k.split(",") if x); KEYS["t"] = time.perf_counter()
            self.send_response(204); self.end_headers(); return
        if u.path == "/snapshot.jpg":
            with _LOCK: jpg = _FRAME["jpg"]
            if jpg is None: self.send_response(503); self.end_headers(); return
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg))); self.end_headers(); self.wfile.write(jpg); return
        if u.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache"); self.end_headers()
            try:
                while True:
                    with _LOCK: jpg = _FRAME["jpg"]
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError, OSError): return
        self.send_response(404); self.end_headers()


httpd = ThreadingHTTPServer(("0.0.0.0", args.port), _H)
threading.Thread(target=httpd.serve_forever, daemon=True).start()
print(f"[inspect] >>> open  http://localhost:{args.port}/  — WASD move, R/F up/down, arrows look <<<", flush=True)


def apply_keys(dt):
    with _LOCK:
        ks = set(KEYS["s"]); fresh = (time.perf_counter() - KEYS["t"]) < 0.4
    if not fresh: ks = set()
    # number keys 1-5 = snap to a close front view of that asset; 0 = overview
    digit = next((d for d in ks if d in "012345"), None)
    if digit is not None:
        if digit == "0":
            CAM["pos"] = [float(CTR[0]), float(CTR[1] - 10.0), 3.0]; CAM["h"] = 0.0; CAM["p"] = -12.0
        else:
            i = int(digit) - 1
            if i < len(spawned):
                c = spawned[i][1]; sz = float(max(spawned[i][2]))
                dist = sz * 1.7 + 2.2; off = dist * 0.7      # 3/4 view: back-left-up, aimed at the asset
                CAM["pos"] = [float(c[0] - off), float(c[1] - off), float(c[2] + dist * 0.5)]
                CAM["h"] = -45.0; CAM["p"] = -26.0
        return
    spd, turn = 7.0 * dt, 75.0 * dt
    h = np.radians(CAM["h"]); p = np.radians(CAM["p"])
    fwd = np.array([-np.sin(h) * np.cos(p), np.cos(h) * np.cos(p), np.sin(p)])
    right = np.array([np.cos(h), np.sin(h), 0.0])
    pos = np.asarray(CAM["pos"], float)
    if "w" in ks: pos += fwd * spd
    if "s" in ks: pos -= fwd * spd
    if "d" in ks: pos += right * spd
    if "a" in ks: pos -= right * spd
    if "r" in ks: pos[2] += spd
    if "f" in ks: pos[2] = max(0.3, pos[2] - spd)
    CAM["pos"] = [float(pos[0]), float(pos[1]), float(pos[2])]
    if "left" in ks:  CAM["h"] += turn
    if "right" in ks: CAM["h"] -= turn
    if "up" in ks:    CAM["p"] = min(89.0, CAM["p"] + turn)
    if "down" in ks:  CAM["p"] = max(-89.0, CAM["p"] - turn)


def hud(frame):
    cv2.rectangle(frame, (0, frame.shape[0] - 46), (frame.shape[1], frame.shape[0]), (0, 0, 0), -1)
    cv2.putText(frame, "row (along +X):  " + "  |  ".join(n for n, _, _ in spawned),
                (10, frame.shape[0] - 26), FONT, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, f"cam {np.round(CAM['pos'],1)} h={CAM['h']:.0f} p={CAM['p']:.0f}   "
                       "GREEN arrow=+Y(forward/heading0)  RED=+X  BLUE=+Z",
                (10, frame.shape[0] - 8), FONT, 0.5, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


print("[inspect] rendering loop up. (Ctrl-C to stop)", flush=True)
last = time.perf_counter()
while True:
    now = time.perf_counter(); dt = min(0.1, now - last); last = now
    apply_keys(dt)
    try: env.step([0.0, 0.0])
    except Exception: pass
    a = np.asarray(cam.perceive(False, eng.render, tuple(CAM["pos"]), (CAM["h"], CAM["p"], 0.0)))
    if a.dtype != np.uint8:
        a = (a * 255).astype(np.uint8) if a.max() <= 1.01 else a.astype(np.uint8)
    frame = hud(np.ascontiguousarray(a))
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if ok:
        with _LOCK: _FRAME["jpg"] = buf.tobytes()
    time.sleep(0.005)
