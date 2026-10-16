import fcntl
import hashlib
import heapq
import json
import math
import os
import random
import re
import threading
import time
import webbrowser
import socket
import select
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import urllib.request
import urllib.parse

from diagnostics import Diagnostics
from arm_camera import CAMERA
from arm_remote import start_remote_access
from features import FEATURES, DATA_DIR, SOURCE_DIR
from workspace_map import sample_workspace
from boundary_model import BOUNDARY_YIELD, build_boundary_poly

# One authoritative geometry source.  robot_mapper.py owns controller I/O and
# mapper-count bookkeeping; kinematics.py owns coordinates, angles, linkage
# closure, and physical-home geometry.
from kinematics import (
    HOME_AC_DEGREES,
    HOME_QY_DEGREES,
    full_linkage_kinematics,
    inverse_side,
    member_angles,
    side_point_to_world,
    tool_is_outwardmost,
)

MOONRAKER = "http://127.0.0.1:7125"
HOST = "0.0.0.0"
HTTP_PORT = 8765
PAGE_TOKEN = str(time.time_ns())
DIAG = Diagnostics(MOONRAKER)
_workspace_cache = None
_workspace_lock = threading.Lock()

X_COUNT_MOVE = 0.50
Y_COUNT_MOVE = -0.50
Z_COUNT_MOVE = -0.10

X_HOME_STEP = -10.00
Y_HOME_STEP = +10.00
Z_HOME_STEP = +5.00
HOME_FEED_XY = 3000
HOME_FEED_Z = 1800
X_FINE_STEP = -0.25
Y_FINE_STEP = +0.25
Z_FINE_STEP = +0.05
FINE_FEED_XY = 300
FINE_FEED_Z = 180
MAX_ITERS = 3000

# Fitted X/Y joint-space envelope used for both display and motion limiting.
# Allowed motion stays BOUNDARY_YIELD counts INSIDE this fitted perimeter.
NORMAL_MOVE_FEED_DEFAULT = 2500
NORMAL_ACCEL_MM_S2 = 500.0
DANCE_ACCEL_DEFAULT_MM_S2 = 80.0
MAPPING_MODE = False

def _build_boundary_poly():
    return build_boundary_poly()

BOUNDARY_POLY = _build_boundary_poly()
WORKSPACE_CACHE_SCHEMA = 2

# Prevent two mapper instances from fighting over the same controller.
DATA_DIR.mkdir(parents=True, exist_ok=True)
_process_lock_file = open(DATA_DIR / "robot_mapper.lock", "w")
try:
    fcntl.flock(_process_lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError as exc:
    raise RuntimeError("Another Robot Joint-Space Mapper instance is already running") from exc

# Klipper owns the MCU USB connection. The original application keeps its
# exact motion, kinematics, homing, boundary, and UI logic; this adapter only
# translates the old Marlin transport into Moonraker/Klipper commands.
lock = threading.Lock()
home_request_lock = threading.Lock()
home_in_progress = False
stop_event = threading.Event()
_axes_registered = False
ROBOT_CORNER_VEL = 6.0
LINEAR_CORNER_VEL = 3.0
LINEAR_MODE_FILE = str(DATA_DIR / "linear_mode.json")
BOUNDARY_MODE_FILE = str(DATA_DIR / "boundary_mode.json")
PROGRAM_FILE = str(DATA_DIR / "programmed_moves.json")
PROGRAM_LIBRARY_DIR = str(DATA_DIR / "programs")
PROGRAM_DEFAULTS = {"speed": 80.0, "time": 0.0, "wait": 0.0}
SENSOR_CONFIG_FILE = str(DATA_DIR / "sensor_config.json")
SENSOR_PORT = os.environ.get(
    "ENDERARM_SENSOR_PORT",
    "/dev/serial/by-id/FRDM-KL02Z",
)
SENSOR = None

def _load_linear_mode():
    try:
        with open(LINEAR_MODE_FILE, "r") as f:
            return FEATURES['rail'] and bool(json.load(f).get("enabled", False))
    except Exception:
        return False

def _save_linear_mode(enabled):
    with open(LINEAR_MODE_FILE, "w") as f:
        json.dump({"enabled": bool(enabled)}, f)

def _load_boundary_mode():
    try:
        with open(BOUNDARY_MODE_FILE, "r") as f:
            return bool(json.load(f).get("enabled", True))
    except Exception:
        return True

def _save_boundary_mode(enabled):
    with open(BOUNDARY_MODE_FILE, "w") as f:
        json.dump({"enabled": bool(enabled)}, f)

def _program_settings(raw, defaults=None):
    """Normalize a waypoint's timing controls (seconds, degrees per second)."""
    defaults = defaults or PROGRAM_DEFAULTS
    values = {}
    for key, minimum, maximum in (("speed", 1.0, 220.0), ("time", 0.0, 120.0), ("wait", 0.0, 120.0)):
        value = float(raw.get(key, defaults[key]))
        if not math.isfinite(value) or not minimum <= value <= maximum:
            label = {"speed": "speed", "time": "move time", "wait": "wait"}[key]
            raise ValueError(f"Program point {label} must be between {minimum:g} and {maximum:g}")
        values[key] = round(value, 3)
    return values

def _program_point(raw, defaults=None):
    """Normalize one recorded joint-space waypoint without moving the arm."""
    point={}
    for key in ("a", "b", "c", "d"):
        value=float(raw.get(key, 0.0))
        if not math.isfinite(value):
            raise ValueError(f"Program point {key.upper()} must be a finite number")
        point[key]=round(value, 4)
    point.update(_program_settings(raw, defaults))
    return point

def _decode_program(raw):
    points=raw.get("points", [])
    if not isinstance(points, list):
        points=[]
    defaults=_program_settings(raw.get("defaults", {}), PROGRAM_DEFAULTS)
    return [_program_point(point, defaults) for point in points], defaults

def _load_program():
    try:
        with open(PROGRAM_FILE, "r") as f:
            return _decode_program(json.load(f))
    except Exception:
        return [], dict(PROGRAM_DEFAULTS)

def _save_program(points, defaults):
    # This is the user's active program, not a backup or generated copy.
    with open(PROGRAM_FILE, "w") as f:
        json.dump({"points": points, "defaults": defaults}, f, separators=(",", ":"))

def _program_name(value):
    name=str(value or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9 _-]{0,47}", name):
        raise ValueError("Use a program name of 1–48 letters, numbers, spaces, _ or -")
    return name

def _program_library_path(name):
    return os.path.join(PROGRAM_LIBRARY_DIR, _program_name(name)+".json")

def _saved_program_names():
    try:
        return sorted(
            [entry[:-5] for entry in os.listdir(PROGRAM_LIBRARY_DIR) if entry.endswith(".json")],
            key=str.lower,
        )
    except FileNotFoundError:
        return []

def _save_named_program(name, points, defaults):
    path=_program_library_path(name)
    os.makedirs(PROGRAM_LIBRARY_DIR, exist_ok=True)
    with open(path, "w") as f:
        json.dump({"points": points, "defaults": defaults}, f, separators=(",", ":"))

def _load_named_program(name):
    with open(_program_library_path(name), "r") as f:
        return _decode_program(json.load(f))

class MotionCancelled(RuntimeError):
    pass

state = {"x":0.0,"y":0.0,"z":0.0,"linear_y":0.0,"motors_enabled":None,"linear_hardware":_load_linear_mode(),"arm_speed":40.0,"arm_accel":200.0,"arm_jerk":6.0,"linear_speed":30.0,"linear_accel":150.0,"linear_jerk":3.0,"homed":False,"home_required":False,"busy":False,"message":"Connecting to controller...","home_x_counts":None,"home_y_counts":None,"home_z_counts":None,"boundary_enabled":_load_boundary_mode()}
boundary_test_session = {
    "id": 1,
    "started_at": time.time(),
    "points": [],
}

dance_thread = None
dance_thread_lock = threading.Lock()
program_points, program_defaults = _load_program()
program_active_name = None
program_thread = None
program_thread_lock = threading.Lock()

def _program_running():
    with program_thread_lock:
        return bool(program_thread and program_thread.is_alive())

# Mapper X/Y are already 90T OUTPUT-JOINT angles established by the measured
# workspace. They are NOT motor-shaft degrees. The 20T:90T ratio belongs only
# in motor-command conversion, never in the renderer/animation mapping.
#
# After precision homing, the measured post-release X/Y values are used as the
# output-angle anchors for the physical home pose.
HOME_RELEASE_X_COUNTS = 3.5
HOME_RELEASE_Y_COUNTS = 3.5

# Positive mapper count motion is currently defined to increase qX and qY.
# Keep these explicit so motor wiring/direction can be calibrated without
# changing the geometric coordinate convention.
X_JOINT_DIRECTION = +1.0
Y_JOINT_DIRECTION = +1.0


def joint_angles_from_mapper_counts(x_counts, y_counts):
    """Map mapper X/Y directly to geometric 90T output angles qX and qY.

    X/Y are already output-joint angular coordinates. The belt ratio is NOT
    applied here. Physical home supplies the angular offset only.
    """
    # Anchor the geometric home angles to the counts measured by THIS homing
    # cycle.  The 3.5-count constants are only startup fallbacks before homing.
    home_x = state.get("home_x_counts")
    home_y = state.get("home_y_counts")
    if home_x is None:
        home_x = HOME_RELEASE_X_COUNTS
    if home_y is None:
        home_y = HOME_RELEASE_Y_COUNTS

    qx = HOME_AC_DEGREES + X_JOINT_DIRECTION * (x_counts - home_x)
    qy = HOME_QY_DEGREES + Y_JOINT_DIRECTION * (y_counts - home_y)
    return qx, qy


def kinematic_state(x_counts, y_counts, z_counts):
    """Return the authoritative solved pose used by the browser animation."""
    qx, qy = joint_angles_from_mapper_counts(x_counts, y_counts)
    pivots = full_linkage_kinematics(qx, qy)
    angles = member_angles(pivots)

    # Z mapper motion is the base/yaw axis.  Preserve the existing mapper sign.
    home_z = state.get("home_z_counts")
    if home_z is None:
        home_z = 0.0
    yaw_degrees = +(float(z_counts) - float(home_z))

    def point_dict(p):
        w = side_point_to_world(p, yaw_degrees)
        return {"x": w.x, "y": w.y, "z": w.z}

    world = {
        name: point_dict(getattr(pivots, name.lower() if name != "T" else "tool"))
        for name in ("A", "B", "C", "D", "E", "F", "G", "H", "I", "T")
    }
    tool = world["T"]
    radius = math.hypot(tool["x"], tool["y"])
    reach = math.hypot(radius, tool["z"])

    return {
        "qX_deg": qx,
        "qY_deg": qy,
        "yaw_deg": yaw_degrees,
        "member_angles_deg": {
            "AC": angles.ac_degrees,
            "AB": angles.ab_degrees,
            "BF": angles.bf_degrees,
            "FCH": angles.fch_degrees,
        },
        "world": world,
        "tool": tool,
        "radius_mm": radius,
        "reach_mm": reach,
        "tool_is_outwardmost": tool_is_outwardmost(pivots),
    }


def _boundary_test_point():
    kin = kinematic_state(state["x"], state["y"], state["z"])
    return {
        "index": len(boundary_test_session["points"]) + 1,
        "time": time.time(),
        "x": state["x"],
        "y": state["y"],
        "z": state["z"],
        "q_lower_deg": kin["qX_deg"],
        "q_crank_deg": kin["qY_deg"],
        "base_deg": kin["yaw_deg"],
        "tool_x_mm": kin["tool"]["x"],
        "tool_y_mm": kin["tool"]["y"],
        "tool_z_mm": kin["tool"]["z"],
        "tool_is_outwardmost": kin["tool_is_outwardmost"],
    }


def _arm_robot_status():
    try:
        q=_moon_get("/printer/objects/query?arm_robot", timeout=2.0)
        return q["result"]["status"].get("arm_robot", {})
    except Exception:
        return {}

def _sync_state_from_arm_robot():
    st=_arm_robot_status()
    if not st:
        return st
    controller_homed=bool(st.get("homed",state.get("homed",False)))
    # A completed tilt correction deliberately requires an operator-initiated
    # Home, even though Klipper still has its prior coordinate reference.
    state["homed"]=controller_homed and not state.get("home_required",False)
    # A programmed run is queued by this mapper (rather than arm_robot), so
    # do not let an idle native Dance status erase its local busy state.
    state["busy"]=bool(st.get("dance_active",False)) or _program_running()
    state["x"]=float(st.get("A",state.get("x",0.0)))
    state["y"]=float(st.get("B",state.get("y",0.0)))
    state["z"]=float(st.get("C",state.get("z",0.0)))
    state["linear_y"]=float(st.get("D",state.get("linear_y",0.0)))
    state["linear_hardware"]=bool(st.get("linear_enabled",state.get("linear_hardware",False)))
    # During Dance, pending tuning belongs to the NEXT cycle. Keep the user's
    # requested values visible instead of replacing them with current-cycle limits.
    if not st.get("tune_pending",False):
        state["arm_speed"]=float(st.get("arm_velocity",state.get("arm_speed",40.0)))
        state["arm_accel"]=float(st.get("arm_accel",state.get("arm_accel",200.0)))
        state["arm_jerk"]=float(st.get("arm_corner",state.get("arm_jerk",6.0)))
        state["linear_speed"]=float(st.get("slide_velocity",state.get("linear_speed",30.0)))
        state["linear_accel"]=float(st.get("slide_accel",state.get("linear_accel",150.0)))
        state["linear_jerk"]=float(st.get("slide_corner",state.get("linear_jerk",3.0)))
    if state["homed"]:
        if state.get("home_x_counts") is None: state["home_x_counts"]=3.5
        if state.get("home_y_counts") is None: state["home_y_counts"]=3.5
        if state.get("home_z_counts") is None: state["home_z_counts"]=2.5
    return st

def state_payload():
    """UI state derived from the native Klipper robot object."""
    arm_st=_sync_state_from_arm_robot()
    payload = dict(state)
    payload["arm_robot"]=arm_st
    payload["boundary_test_session"] = boundary_test_session
    payload["boundary_poly"] = BOUNDARY_POLY
    with program_thread_lock:
        running=bool(program_thread and program_thread.is_alive())
    payload["program"]={
        "points": list(program_points), "defaults": dict(program_defaults),
        "saved": _saved_program_names(), "active_name": program_active_name,
        "running": running,
    }
    if not FEATURES["sensors"]:
        payload["sensor"]={"enabled":False}
    elif SENSOR is None:
        payload["sensor"]={"enabled":True,"connected":False,"fresh":False,"error":"sensor service not started"}
    else:
        payload["sensor"]=SENSOR.snapshot()
    try:
        payload["kinematics"] = kinematic_state(
            state["x"], state["y"], state["z"]
        )
    except ValueError as exc:
        payload["kinematics"] = None
        payload["kinematics_error"] = str(exc)
    return payload

def live_motion_payload():
    """Return Klipper planner live position for the three arm axes."""
    try:
        q=_moon_get("/printer/objects/query?motion_report", timeout=2.0)
        st=q["result"]["status"]["motion_report"]
        pos=st.get("live_position", [])
        vel=float(st.get("live_velocity", 0.0))
        if len(pos) < 8:
            raise RuntimeError(f"Unexpected motion_report position: {pos}")
        # Klipper extra-axis order is A=4, B=5, C=6.
        return {
            "x": float(pos[4]),
            "y": float(pos[5]),
            "z": float(pos[6]),
            "linear_y": float(pos[7]),
            "velocity": vel,
            "time": time.time(),
        }
    except Exception as exc:
        return {"error": str(exc), "time": time.time()}

def _workspace_cache_key():
    """Fingerprint every input that changes the displayed safe workspace."""
    sources = (
        str(SOURCE_DIR / "kinematics.py"),
        str(SOURCE_DIR / "workspace_map.py"),
    )
    source_hashes = {}
    for path in sources:
        try:
            with open(path, "rb") as f:
                source_hashes[path] = hashlib.sha256(f.read()).hexdigest()
        except OSError:
            source_hashes[path] = None
    payload = {
        "schema": WORKSPACE_CACHE_SCHEMA,
        "boundary_poly": BOUNDARY_POLY,
        "boundary_yield": BOUNDARY_YIELD,
        "sources": source_hashes,
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def workspace_payload():
    global _workspace_cache
    if _workspace_cache is None:
        with _workspace_lock:
            if _workspace_cache is None:
                cache_path=str(DATA_DIR / "workspace_cache.json")
                cache_key = _workspace_cache_key()
                try:
                    with open(cache_path,"r") as f:
                        cached=json.load(f)
                    # Cache is valid only for the measured current base/rail ranges.
                    if (cached.get("meta",{}).get("c_range")==[2.5,272.0]
                            and cached.get("meta",{}).get("d_range")==[0.0,235.0]
                            and cached.get("meta",{}).get("cache_key")==cache_key
                            and cached.get("side_samples")):
                        _workspace_cache=cached
                except Exception:
                    pass
                if _workspace_cache is None:
                    t0=time.perf_counter()
                    _workspace_cache=sample_workspace(BOUNDARY_POLY, yield_mm=BOUNDARY_YIELD)
                    _workspace_cache.setdefault("meta", {})["cache_key"] = cache_key
                    try:
                        with open(cache_path,"w") as f: json.dump(_workspace_cache,f,separators=(",",":"))
                    except Exception as exc:
                        DIAG.event("workspace_cache_error",{"error":str(exc)})
                    DIAG.latency("workspace_build",time.perf_counter()-t0)
                    DIAG.event("workspace_built",_workspace_cache.get("meta",{}))
    return _workspace_cache

def workspace_payload_for_mode():
    full=workspace_payload()
    if state.get("linear_hardware",False):
        out=dict(full)
        out["meta"]=dict(full.get("meta",{}))
        out["meta"]["linear_enabled"]=True
        return out

    # The cached map contains all D positions.  With the rail disabled the
    # workspace is the D=0 volume swept continuously by the base, not the
    # sparse 15-degree samples used to keep the full rail-on cache small.
    # Rebuild a denser visual point cloud from the already-safe arm poses.
    # This affects drawing only; target validation continues to use exact IK
    # and the same measured A/B boundary.
    side_samples=full.get("side_samples", [])
    c_min,c_max=(full.get("meta",{}).get("c_range") or [2.5,272.0])
    c_step=3.0
    cs=[]
    c=float(c_min)
    while c<=float(c_max)+1e-9:
        cs.append(c)
        c+=c_step
    if not cs or cs[-1]<float(c_max):
        cs.append(float(c_max))
    total=len(side_samples)*len(cs)
    stride=max(1, math.ceil(total/12000))
    pts=[]
    index=0
    for c in cs:
        yaw=math.radians(c-2.5)
        for r,z,_,_ in side_samples:
            if index % stride == 0:
                pts.append((round(float(r)*math.sin(yaw),2),
                            round(float(r)*math.cos(yaw),2),round(float(z),2),
                            round(c,2),0.0))
            index+=1
    out=dict(full)
    out["points"]=pts
    meta=dict(full.get("meta",{}))
    meta["linear_enabled"]=False
    meta["d_range"]=[0.0,0.0]
    meta["ui_samples"]=len(pts)
    meta["total_samples"]=total
    meta["base_display_step_deg"]=c_step
    out["meta"]=meta
    if pts:
        out["bounds"]={
            "x":[min(float(q[0]) for q in pts),max(float(q[0]) for q in pts)],
            "y":[min(float(q[1]) for q in pts),max(float(q[1]) for q in pts)],
            "z":[min(float(q[2]) for q in pts),max(float(q[2]) for q in pts)],
        }
    return out

def plan_world_target(tx,ty,tz,preferred_d=None, seed=None):
    """Solve one world Cartesian target with automatic rail selection.

    Uses the precomputed safe A/B workspace to derive rail candidates
    analytically, then runs exact linkage IK only on the best few candidates.
    """
    if not state.get("homed"):
        raise RuntimeError("Home first before planning world motion")
    tx=float(tx); ty=float(ty); tz=float(tz)
    if not state.get("linear_hardware",False):
        preferred_d=0.0
    # A route is solved in its actual order.  Later waypoints must use the
    # previous waypoint as their linkage-branch seed, not the pose that was
    # live when the user opened the editor.
    seed=seed or {}
    cur_a=float(seed.get("a",state.get("x",3.5)))
    cur_b=float(seed.get("b",state.get("y",3.5)))
    cur_c=float(seed.get("c",state.get("z",2.5)))
    cur_d=float(seed.get("d",state.get("linear_y",0.0)))
    qx0,qy0=joint_angles_from_mapper_counts(cur_a,cur_b)
    home_c=float(state.get("home_z_counts") if state.get("home_z_counts") is not None else 2.5)
    cmin=home_c; cmax=home_c+269.5
    boundary_on=state.get("boundary_enabled", True)

    # Keep an unchanged current Cartesian pose at the current joints/rail.
    # The released-home A/B pose is physically validated by its endstops but
    # intentionally lies inside the generic 5-degree workspace yield.
    try:
        curkin=kinematic_state(cur_a,cur_b,cur_c)
        ctool=curkin["tool"]
        curwx=float(ctool["x"])
        curwy=float(ctool["y"])+cur_d
        curwz=float(ctool["z"])
        if math.sqrt((tx-curwx)**2+(ty-curwy)**2+(tz-curwz)**2) <= 0.5:
            return {"reachable":True,"x":tx,"y":ty,"z":tz,
                    "D":cur_d,"A":cur_a,"B":cur_b,"C":cur_c,
                    "yaw_deg":float(curkin.get("yaw_deg",0.0)),
                    "radius_mm":float(curkin.get("radius_mm",0.0)),
                    "boundary_margin":boundary_score(cur_a,cur_b),
                    "cost":0.0,"current_pose":True}
    except Exception:
        pass

    def normalize_c(rel_y):
        radius=math.hypot(tx,rel_y)
        if radius <= 1e-6:
            return None
        yaw=math.degrees(math.atan2(tx,rel_y))
        c=home_c+yaw
        while c < cmin:
            c += 360.0
        while c > cmax and c-360.0 >= cmin:
            c -= 360.0
        if not (cmin-1e-6 <= c <= cmax+1e-6):
            return None
        return radius,c,(c-home_c)

    def exact(d):
        d=max(0.0,min(235.0,float(d)))
        rel_y=ty-d
        nc=normalize_c(rel_y)
        if nc is None:
            return None
        radius,c,yaw=nc
        try:
            qx,qy=inverse_side(radius,tz,qx0,qy0)
            qx=qx0+((qx-qx0+180.0)%360.0)-180.0
            qy=qy0+((qy-qy0+180.0)%360.0)-180.0
            piv=full_linkage_kinematics(qx,qy)
            if boundary_on and not tool_is_outwardmost(piv):
                return None
        except Exception:
            return None
        a=3.5+(qx-HOME_AC_DEGREES)
        b=3.5+(qy-HOME_QY_DEGREES)
        margin=boundary_score(a,b)
        # Boundary OFF means the measured A/B envelope and its path yield do
        # not participate in planning.  The linkage solve, base travel, rail
        # limits, and Klipper's physical motion limits still apply.
        if boundary_on:
            if margin < BOUNDARY_YIELD:
                return None
            # The endpoint is safe.  A later joint-space corridor planner
            # decides whether a direct segment is safe or needs to travel
            # around a concave portion of the measured envelope.
        cost=(abs(d-cur_d)*0.55 + abs(c-cur_c)*0.12
              + math.hypot(a-cur_a,b-cur_b)*0.10)
        return {"reachable":True,"x":tx,"y":ty,"z":tz,
                "D":d,"A":a,"B":b,"C":c,"yaw_deg":yaw,
                "radius_mm":radius,"boundary_margin":margin,"cost":cost}

    # Manual rail: one exact solve only.
    if preferred_d is not None:
        r=exact(preferred_d)
        if r is None:
            raise RuntimeError("Target is not reachable with the requested rail position")
        return r

    # With the measured boundary disabled, do not derive candidates from the
    # safe-workspace cache: that cache intentionally contains only Boundary
    # ON poses.  Probe the available rail travel with exact IK instead.
    if not boundary_on:
        rail_candidates=[cur_d]+[float(d) for d in range(0,236,2)]
        seen=set(); exact_results=[]
        for d in rail_candidates:
            key=round(d,2)
            if key in seen:
                continue
            seen.add(key)
            r=exact(d)
            if r is not None:
                exact_results.append(r)
        if not exact_results:
            raise RuntimeError("Target cannot be reached by the arm geometry and physical axis ranges")
        return min(exact_results,key=lambda r:r["cost"])

    # Prefer no rail motion if it already works.
    current=exact(cur_d)
    if current is not None:
        return current

    w=workspace_payload()
    samples=w.get("side_samples",[])
    if not samples:
        raise RuntimeError("Workspace planner cache is unavailable")

    # Find side-plane arm samples near the requested height.
    near=[]
    for tol in (1.5,3.0,5.0,8.0,12.0):
        near=[p for p in samples if abs(float(p[1])-tz)<=tol]
        if len(near)>=12:
            break
    if not near:
        raise RuntimeError("No modeled arm poses reach this Z height")

    # For each safe radial sample r, solve D analytically:
    #   r^2 = X^2 + (Y-D)^2
    candidates=[]
    ax=abs(tx)
    for r,zs,a0,b0 in near:
        r=float(r)
        if r+1e-6 < ax:
            continue
        root=math.sqrt(max(0.0,r*r-tx*tx))
        for rel_y in (root,-root):
            d=ty-rel_y
            if not (0.0 <= d <= 235.0):
                continue
            nc=normalize_c(rel_y)
            if nc is None:
                continue
            _,c,_=nc
            approx_cost=(abs(d-cur_d)*0.55 + abs(c-cur_c)*0.12
                         + math.hypot(float(a0)-cur_a,float(b0)-cur_b)*0.10
                         + abs(float(zs)-tz)*0.4)
            candidates.append((approx_cost,d))

    if not candidates:
        raise RuntimeError("World target is outside the modeled reachable workspace")

    # Exact IK only for the best unique rail candidates.
    candidates.sort(key=lambda x:x[0])
    seen=set(); exact_results=[]
    for _,d in candidates:
        key=round(d,1)
        if key in seen:
            continue
        seen.add(key)
        r=exact(d)
        if r is not None:
            exact_results.append(r)
            if len(exact_results)>=4:
                break
        if len(seen)>=12:
            break
    if not exact_results:
        raise RuntimeError("Target is near the workspace edge but no exact safe IK path was found")
    return min(exact_results,key=lambda r:r["cost"])

def check_stop():
    if stop_event.is_set():
        raise MotionCancelled("Motion stopped by user")

def _moon_get(path, timeout=5.0):
    t0=time.perf_counter()
    try:
        with urllib.request.urlopen(MOONRAKER + path, timeout=timeout) as r:
            return json.loads(r.read().decode())
    finally:
        DIAG.latency("moonraker_get",time.perf_counter()-t0)

def _moon_post(path, params=None, timeout=30.0):
    data=urllib.parse.urlencode(params or {}).encode()
    req=urllib.request.Request(MOONRAKER + path, data=data, method="POST")
    t0=time.perf_counter()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw=r.read().decode()
            return json.loads(raw) if raw else {}
    finally:
        DIAG.latency("moonraker_post",time.perf_counter()-t0)

def _gcode(script, timeout=120.0):
    t0=time.perf_counter()
    first=(script.strip().splitlines() or [""])[0][:160]
    try:
        out=_moon_post("/printer/gcode/script", {"script":script}, timeout=timeout)
        DIAG.event("gcode",{"command":first,"bytes":len(script.encode())})
        return out
    finally:
        DIAG.latency("gcode_submit",time.perf_counter()-t0,{"command":first})

def _printer_info():
    try:
        return _moon_get("/printer/info", timeout=3.0).get("result",{})
    except Exception:
        return {}

def wait_for_controller(timeout=15.0):
    deadline=time.time()+timeout
    last=None
    while time.time()<deadline:
        info=_printer_info()
        last=info
        if info.get("state")=="ready":
            return [f"Klipper {info.get('software_version','ready')}"]
        if info.get("state") in ("shutdown","error"):
            try:
                _gcode("FIRMWARE_RESTART", timeout=5)
            except Exception:
                pass
        time.sleep(0.4)
    raise RuntimeError(f"Controller never became ready: {last}")

def _axis_register_cmd(joint, axis):
    # Klipper's closest equivalent to Marlin jerk for these extra axes is
    # instantaneous corner velocity. Lower = softer corner transitions.
    return (f"MANUAL_STEPPER STEPPER={joint} GCODE_AXIS={axis} "
            f"LIMIT_VELOCITY=220 LIMIT_ACCEL=1800 "
            f"INSTANTANEOUS_CORNER_VELOCITY={ROBOT_CORNER_VEL:.3f}")

def _unregister_axes():
    global _axes_registered
    for joint in ("joint_x","joint_y","joint_z","joint_4"):
        try:
            _gcode(f"MANUAL_STEPPER STEPPER={joint} GCODE_AXIS=", timeout=8)
        except Exception:
            pass
    _axes_registered=False

def _register_axes():
    global _axes_registered
    _unregister_axes()
    _gcode(_axis_register_cmd("joint_x","A"), timeout=8)
    _gcode(_axis_register_cmd("joint_y","B"), timeout=8)
    _gcode(_axis_register_cmd("joint_z","C"), timeout=8)
    _gcode(f"MANUAL_STEPPER STEPPER=joint_4 GCODE_AXIS=D LIMIT_VELOCITY=100 LIMIT_ACCEL=500 INSTANTANEOUS_CORNER_VELOCITY={LINEAR_CORNER_VEL:.3f}", timeout=8)
    _axes_registered=True

def _enable_all(enable=True):
    # Manual-stepper enable requires them to be temporarily detached from
    # G-code axes.
    _unregister_axes()
    val=1 if enable else 0
    joints=["joint_x","joint_y","joint_z"]
    if state.get("linear_hardware",False):
        joints.append("joint_4")
    for joint in joints:
        _gcode(f"MANUAL_STEPPER STEPPER={joint} ENABLE={val}", timeout=8)
    if not state.get("linear_hardware",False):
        _gcode("MANUAL_STEPPER STEPPER=joint_4 ENABLE=0", timeout=8)
    if enable:
        _register_axes()

def _reset_axis_positions():
    # Exact equivalent of the original G92 X0 Y0 Z0 for this relative-motion
    # application: zero all three manual-stepper coordinates, then reattach.
    _unregister_axes()
    for joint in ("joint_x","joint_y","joint_z"):
        _gcode(f"MANUAL_STEPPER STEPPER={joint} SET_POSITION=0", timeout=8)
    _register_axes()

def _transform_g1(cmd):
    vals={}
    for axis,val in re.findall(r"([XYZF])\s*(-?\d+(?:\.\d+)?)",cmd,re.I):
        vals[axis.upper()]=float(val)

    # Original Marlin units -> original mapper output-joint degrees.
    da=vals.get("X",0.0)/X_COUNT_MOVE
    db=vals.get("Y",0.0)/Y_COUNT_MOVE
    dc=vals.get("Z",0.0)/Z_COUNT_MOVE

    parts=["G1"]
    if "X" in vals: parts.append(f"A{da:.6f}")
    if "Y" in vals: parts.append(f"B{db:.6f}")
    if "Z" in vals: parts.append(f"C{dc:.6f}")

    if "F" in vals:
        # Preserve the exact duration implied by the original Marlin command.
        # old duration = old path / old feed. Convert that duration to the
        # joint-angle vector so Klipper runs the same physical timing.
        old_path=math.sqrt(sum(vals.get(k,0.0)**2 for k in ("X","Y","Z")))
        joint_path=math.sqrt(da*da+db*db+dc*dc)
        if old_path>1e-12 and joint_path>1e-12:
            joint_feed=vals["F"]*joint_path/old_path
        else:
            joint_feed=max(60.0,vals["F"]*2.0)
        parts.append(f"F{joint_feed:.3f}")
    return " ".join(parts)

def send(cmd, timeout=15):
    c=cmd.strip()
    u=c.upper()

    # The old application uses only relative G1 motion. Keep its API intact.
    if u=="G91":
        _gcode("G91",timeout=timeout)
        return ["ok"]
    if u=="G90":
        _gcode("G90",timeout=timeout)
        return ["ok"]
    if u.startswith("G1"):
        if not _axes_registered:
            _register_axes()
        _gcode(_transform_g1(c),timeout=max(float(timeout),30.0))
        return ["ok"]
    if u.startswith("G92"):
        _reset_axis_positions()
        return ["ok"]
    if u.startswith("M400"):
        _gcode("M400",timeout=max(float(timeout),120.0))
        return ["ok"]
    if u.startswith("M17"):
        _enable_all(True)
        return ["ok"]
    if u.startswith("M18") or u.startswith("M84"):
        _enable_all(False)
        return ["ok"]
    if u.startswith("M211"):
        return ["ok"]
    if u.startswith("M115"):
        return wait_for_controller(timeout=max(2.0,float(timeout)))
    if u.startswith("M204"):
        m=re.search(r"(?:P|T)(-?\d+(?:\.\d+)?)",c,re.I)
        if m:
            old_accel=float(m.group(1))
            # Same conversion used for X/Y: approximately two joint-degrees
            # per old Marlin motion unit.
            joint_accel=max(20.0,min(1800.0,old_accel*2.0))
            _gcode(f"SET_VELOCITY_LIMIT ACCEL={joint_accel:.3f}",timeout=10)
        return ["ok"]
    if u.startswith("M503"):
        return read_config()

    # Pass through any future Klipper-compatible commands rather than silently
    # dropping them.
    _gcode(c,timeout=max(float(timeout),15.0))
    return ["ok"]

def read_config():
    return [
        "Transport: EnderArm Control -> Moonraker -> Klipper",
        "UI/kinematics/homing/dance/boundary logic: preserved pre-Klipper",
        "joint_x: original Ender X driver; mapper X conversion 0.50 old-unit/count",
        "joint_y: original Ender Y driver; mapper Y conversion -0.50 old-unit/count",
        "joint_z: original Ender Z driver; mapper Z conversion -0.10 old-unit/count",
    ]

def move(cmd):
    send(cmd,timeout=30)
    send("M400",timeout=120)

def endstops():
    # Klipper sections are already wired to the arm's PHYSICAL switch mapping:
    # joint_x -> PC3, joint_y -> PC4, joint_z -> PC2.
    _gcode("QUERY_ENDSTOPS",timeout=8)
    time.sleep(0.03)
    store=_moon_get("/server/gcode_store?count=16",timeout=5)["result"]["gcode_store"]
    for item in reversed(store):
        msg=item.get("message","")
        if item.get("type")!="response" or "manual_stepper joint_x:" not in msg:
            continue
        vals={}
        for name,status in re.findall(
            r"manual_stepper (joint_[xyz]):(open|TRIGGERED|triggered)",msg
        ):
            vals[name]=status.lower()=="triggered"
        if len(vals)==3:
            return vals["joint_x"],vals["joint_y"],vals["joint_z"]
    raise RuntimeError("Could not read all Klipper arm endstops")

def _home_xyz(step_x, step_y, step_z, feed_xy, feed_z, label):
    """Home all three physical joints concurrently.

    Each loop reads all three endstops, then commands only axes that have not
    triggered yet. As soon as one joint reaches home, that axis is omitted from
    subsequent G1 commands while the remaining joints continue.
    """
    send("G91")
    for _ in range(MAX_ITERS):
        check_stop()
        x,y,z=endstops()
        check_stop()
        if x and y and z:
            return

        parts=[]
        if not x:
            parts.append(f"X{step_x}")
        if not y:
            parts.append(f"Y{step_y}")
        if not z:
            parts.append(f"Z{step_z}")

        # G1 feed is the speed along the combined XYZ vector, not an individual
        # axis speed. Compute the fastest vector feed that keeps each active
        # axis component within its configured homing speed.
        active=[]
        if not x:
            active.append(("x", abs(step_x), feed_xy))
        if not y:
            active.append(("y", abs(step_y), feed_xy))
        if not z:
            active.append(("z", abs(step_z), feed_z))

        path=(sum(dist*dist for _,dist,_ in active))**0.5
        limits=[]
        for _,dist,axis_feed in active:
            if dist > 0:
                limits.append(axis_feed * path / dist)
        feed=min(limits) if limits else min(feed_xy, feed_z)
        move("G1 " + " ".join(parts) + f" F{feed:.0f}")
        check_stop()

    x,y,z=endstops()
    raise RuntimeError(
        f"{label} safety limit: X={'HOME' if x else 'open'}, "
        f"Y={'HOME' if y else 'open'}, Z={'HOME' if z else 'open'}"
    )

def first_home():
    """Coarse home: X/Y together at a steady medium speed, then Z."""
    send("M211 S0")
    send("M17")
    send("G91")

    # 2.5 Marlin units = about 5 displayed joint counts on X/Y.
    # F300 coarse X/Y homing: prior working profile.
    state["message"]="Homing — X/Y"
    for _ in range(MAX_ITERS):
        check_stop()
        x,y,_=endstops()
        check_stop()
        if x and y:
            break

        parts=[]
        if not x:
            parts.append("X-2.50")
        if not y:
            parts.append("Y2.50")
        move("G1 " + " ".join(parts) + " F300")
        check_stop()
    else:
        x,y,_=endstops()
        raise RuntimeError(
            f"XY coarse home safety limit: X={'HOME' if x else 'open'}, "
            f"Y={'HOME' if y else 'open'}"
        )

    # Z is separate because firmware caps Z at 5 units/s.
    # 0.50 Marlin units = about 5 displayed Z counts.
    state["message"]="Homing — Z"
    for _ in range(MAX_ITERS):
        check_stop()
        _,_,z=endstops()
        check_stop()
        if z:
            return
        move("G1 Z0.50 F120")
        check_stop()

    _,_,z=endstops()
    raise RuntimeError(
        f"Z coarse home safety limit: Z={'HOME' if z else 'open'}"
    )

def backoff():
    """Release cross-mapped home switches and return logical count offsets.

    The exact switch contact remains coordinate zero. We track every commanded
    release move so the UI reports the real positive position after release.
    """
    send("G91")
    moved_x = 0.0
    moved_y = 0.0
    moved_z = 0.0

    state["message"]="Homing — release X switch"
    for _ in range(30):
        check_stop()
        x,_,_=endstops()
        if not x:
            break
        move("G1 X0.50 F600")
        moved_x += 0.50
    else:
        raise RuntimeError("Could not release physical X home switch")
    move("G1 X0.75 F600")
    moved_x += 0.75

    state["message"]="Homing — release Y switch"
    for _ in range(30):
        check_stop()
        _,y,_=endstops()
        if not y:
            break
        move("G1 Y-0.50 F600")
        moved_y -= 0.50
    else:
        raise RuntimeError("Could not release physical Y home switch")
    move("G1 Y-0.75 F600")
    moved_y -= 0.75

    state["message"]="Homing — release Z switch"
    for _ in range(30):
        check_stop()
        _,_,z=endstops()
        if not z:
            break
        move("G1 Z-0.10 F300")
        moved_z -= 0.10
    else:
        raise RuntimeError("Could not release physical Z home switch")
    move("G1 Z-0.15 F300")
    moved_z -= 0.15

    # Convert Marlin-axis motion back to mapper joint counts.
    return (
        moved_x / X_COUNT_MOVE,
        moved_y / Y_COUNT_MOVE,
        moved_z / Z_COUNT_MOVE,
    )

def fine_home():
    """Slow precision re-home: X/Y together first, then Z."""
    send("G91")

    state["message"]="Homing — precision X/Y"
    for _ in range(MAX_ITERS):
        check_stop()
        x,y,_=endstops()
        check_stop()
        if x and y:
            break

        parts=[]
        if not x:
            parts.append("X-0.25")
        if not y:
            parts.append("Y0.25")
        move("G1 " + " ".join(parts) + " F120")
        check_stop()
    else:
        x,y,_=endstops()
        raise RuntimeError(
            f"XY fine home safety limit: X={'HOME' if x else 'open'}, "
            f"Y={'HOME' if y else 'open'}"
        )

    state["message"]="Homing — precision Z"
    for _ in range(MAX_ITERS):
        check_stop()
        _,_,z=endstops()
        check_stop()
        if z:
            break
        move("G1 Z0.05 F60")
        check_stop()
    else:
        _,_,z=endstops()
        raise RuntimeError(
            f"Z fine home safety limit: Z={'HOME' if z else 'open'}"
        )

    # Exact switch contact is the true origin.
    send("G92 X0 Y0 Z0")

    # Release the cross-mapped switches so normal + motion is not blocked.
    state["message"]="Homing — final switch release"
    ox,oy,oz = backoff()

    xh,yh,zh = endstops()
    if xh or yh or zh:
        raise RuntimeError(
            "Home release incomplete: "
            f"X={'TRIGGERED' if xh else 'open'}, "
            f"Y={'TRIGGERED' if yh else 'open'}, "
            f"Z={'TRIGGERED' if zh else 'open'}"
        )

    state.update(x=ox,y=oy,z=oz,homed=True,home_x_counts=ox,home_y_counts=oy,home_z_counts=oz)

def precise_home(clear_stop=True):
    """Original home definition executed locally by Klipper.

    Physical home remains the original page convention:
      X release = 3.5 counts
      Y release = 3.5 counts
      Z release = 2.5 counts
    Only the timing-critical endstop loop moved into Klipper.
    """
    if clear_stop:
        stop_event.clear()
    if _printer_info().get("state") in ("shutdown","error"):
        try:
            _gcode("FIRMWARE_RESTART", timeout=6)
        except Exception:
            pass
        wait_for_controller(10)
    # Conservative continuous home profile. The controller applies its own
    # acceleration-limited trajectory; no UI polling commands are injected.
    home_speed=30.0
    home_accel=100.0
    state["message"]=f"Homing — continuous Klipper pass ({home_speed:.0f}°/s)"
    _gcode(f"ARM_HOME SPEED={home_speed:.1f} ACCEL={home_accel:.1f}", timeout=180)
    ox,oy,oz=3.5,3.5,2.5
    state.update(
        x=ox,y=oy,z=oz,linear_y=0.0,homed=True,home_required=False,
        home_x_counts=ox,home_y_counts=oy,home_z_counts=oz
    )
    global _axes_registered
    _axes_registered=True
    # ARM_HOME already registered A/B/C/D at the correct released-home
    # coordinates. Re-registering here caused transient zero frames.
    _apply_runtime_tuning(force=True)
    state["message"]="Home complete"

def ensure_homed_for_motion():
    """Restore the physical reference before accepting a motion command."""
    _sync_state_from_arm_robot()
    if state.get("home_required",False):
        raise RuntimeError("Tilt recovery complete — press Home before normal motion")
    if state.get("homed", False):
        return
    state["message"]="Home reference lost — homing before motion"
    precise_home()
    _sync_state_from_arm_robot()
    if not state.get("homed", False):
        raise RuntimeError("Automatic home did not establish a valid reference")

def _point_in_poly(x, y, poly=BOUNDARY_POLY):
    inside=False
    j=len(poly)-1
    for i in range(len(poly)):
        xi,yi=poly[i]
        xj,yj=poly[j]
        if ((yi > y) != (yj > y)):
            x_cross=(xj-xi)*(y-yi)/(yj-yi)+xi
            if x < x_cross:
                inside=not inside
        j=i
    return inside

def _seg_dist(px,py,ax,ay,bx,by):
    vx,vy=bx-ax,by-ay
    wx,wy=px-ax,py-ay
    vv=vx*vx+vy*vy
    if vv <= 1e-12:
        return ((px-ax)**2+(py-ay)**2)**0.5
    t=max(0.0,min(1.0,(wx*vx+wy*vy)/vv))
    qx,qy=ax+t*vx,ay+t*vy
    return ((px-qx)**2+(py-qy)**2)**0.5

def boundary_score(x,y):
    """Signed distance-like safety score: positive inside, negative outside."""
    d=min(
        _seg_dist(x,y,*BOUNDARY_POLY[i],*BOUNDARY_POLY[(i+1)%len(BOUNDARY_POLY)])
        for i in range(len(BOUNDARY_POLY))
    )
    return d if _point_in_poly(x,y) else -d

def nearest_safe_arm_pose(target_y, target_z):
    """Resolve arm-local tool Y/Z to the nearest validated A/B pose.

    This intentionally searches the measured safe joint region instead of
    relying on analytic IK branch selection.  It therefore stays on the same
    physical outward-tool assembly represented by the plotted workspace.
    """
    target_y=float(target_y); target_z=float(target_z)
    amin=min(p[0] for p in BOUNDARY_POLY)
    amax=max(p[0] for p in BOUNDARY_POLY)
    bmin=min(p[1] for p in BOUNDARY_POLY)
    bmax=max(p[1] for p in BOUNDARY_POLY)

    best=None
    def consider(a,b):
        nonlocal best
        if boundary_score(a,b) < BOUNDARY_YIELD:
            return
        try:
            qx,qy=joint_angles_from_mapper_counts(a,b)
            piv=full_linkage_kinematics(qx,qy)
            if not tool_is_outwardmost(piv):
                return
            dy=float(piv.tool.y)-target_y
            dz=float(piv.tool.z)-target_z
            err2=dy*dy+dz*dz
            if best is None or err2 < best[0]:
                best=(err2,float(a),float(b),float(piv.tool.y),float(piv.tool.z))
        except Exception:
            return

    # Coarse search over the measured safe region.
    a=amin
    while a <= amax + 1e-9:
        b=bmin
        while b <= bmax + 1e-9:
            consider(a,b)
            b += 2.0
        a += 2.0

    if best is None:
        raise RuntimeError("No valid outward-tool pose found for this workspace target")

    # Fine local refinement around the best safe branch.
    _,ba,bb,_,_=best
    for step,span in ((0.5,3.0),(0.1,0.8)):
        a=ba-span
        while a <= ba+span+1e-9:
            b=bb-span
            while b <= bb+span+1e-9:
                consider(a,b)
                b += step
            a += step
        _,ba,bb,_,_=best

    err=math.sqrt(best[0])
    if err > 12.0:
        raise RuntimeError(
            "Clicked Y/Z is not on the validated arm branch; nearest safe pose is %.1f mm away"
            % err)
    return {
        "A":best[1],"B":best[2],
        "tool_y":best[3],"tool_z":best[4],
        "error_mm":err,
    }

def clip_xy_path(x0,y0,x1,y1):
    """Clip a straight XY move to the measured envelope + inward yield.

    When starting inside the 5-count yield zone (e.g. immediately after home),
    allow motion that does not reduce the current safety score so the arm can
    move back toward the interior.
    """
    # The precision-homed release pose is physically validated by the
    # endstops themselves, even though it lies inside the generic 5-count
    # plotting yield. Permit an exact return to that known safe pose.
    hx=state.get("home_x_counts")
    hy=state.get("home_y_counts")
    if hx is not None and hy is not None and math.hypot(x1-hx,y1-hy) <= 0.35:
        return x1,y1,False

    dx,dy=x1-x0,y1-y0
    length=(dx*dx+dy*dy)**0.5
    if length <= 1e-12:
        return x1,y1,False

    start_score=boundary_score(x0,y0)
    required = BOUNDARY_YIELD if start_score >= BOUNDARY_YIELD else start_score - 0.05

    # Sample no coarser than 0.5 joint-count along the complete path.
    n=max(2,int(length/0.5)+1)
    last=(x0,y0)
    clipped=False
    for i in range(1,n+1):
        t=i/n
        x=x0+dx*t
        y=y0+dy*t
        score=boundary_score(x,y)
        if score < 0 or score < required:
            clipped=True
            break
        last=(x,y)

    # If starting in the yield band, only permit moves that maintain or improve
    # clearance. Once safely inside, the normal 5-count requirement applies.
    if start_score < BOUNDARY_YIELD and not clipped:
        end_score=boundary_score(*last)
        if end_score + 0.05 < start_score:
            return x0,y0,True

    return last[0],last[1],clipped

def _joint_segment_is_safe(start, end):
    """Return whether a complete A/B segment stays inside the active envelope."""
    nx,ny,clipped=clip_xy_path(start[0],start[1],end[0],end[1])
    return not clipped and abs(nx-end[0]) <= 0.05 and abs(ny-end[1]) <= 0.05

def safe_joint_route(start, end, grid_step=2.0):
    """Find a short, fully validated A/B route inside the measured envelope.

    Cartesian targets are solved to joints first.  If the direct joint segment
    crosses a concave part of the measured envelope, an A* search uses only
    interior samples, then removes unnecessary corners.  The returned points
    are all checked again by ``clip_xy_path`` before they are queued.
    """
    start=(float(start[0]),float(start[1]))
    end=(float(end[0]),float(end[1]))
    if _joint_segment_is_safe(start,end):
        return [start,end]

    step=max(0.5,float(grid_step))
    amin=math.ceil(min(p[0] for p in BOUNDARY_POLY)/step)
    amax=math.floor(max(p[0] for p in BOUNDARY_POLY)/step)
    bmin=math.ceil(min(p[1] for p in BOUNDARY_POLY)/step)
    bmax=math.floor(max(p[1] for p in BOUNDARY_POLY)/step)
    nodes={}
    for ai in range(amin,amax+1):
        for bi in range(bmin,bmax+1):
            point=(ai*step,bi*step)
            if boundary_score(*point) >= BOUNDARY_YIELD:
                nodes[(ai,bi)]=point
    if not nodes:
        raise RuntimeError("Measured boundary has no interior route samples")

    start_id=(None,"start")
    end_id=(None,"end")
    positions={**nodes,start_id:start,end_id:end}

    def visible_anchors(point):
        ranked=sorted(nodes, key=lambda key: math.dist(point,nodes[key]))
        anchors=[]
        for key in ranked:
            if _joint_segment_is_safe(point,nodes[key]):
                anchors.append(key)
                if len(anchors) == 12:
                    break
        return anchors

    start_anchors=visible_anchors(start)
    end_anchors=visible_anchors(end)
    if not start_anchors or not end_anchors:
        raise RuntimeError("No safe corridor connects the current pose to this target")

    neighbor_offsets=((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))
    def neighbors(key):
        if key == start_id:
            return start_anchors
        if key == end_id:
            return end_anchors
        result=[]
        if key in start_anchors:
            result.append(start_id)
        if key in end_anchors:
            result.append(end_id)
        for da,db in neighbor_offsets:
            candidate=(key[0]+da,key[1]+db)
            if candidate in nodes and _joint_segment_is_safe(positions[key],nodes[candidate]):
                result.append(candidate)
        return result

    sequence=0
    queue=[(math.dist(start,end),0.0,sequence,start_id)]
    costs={start_id:0.0}
    previous={}
    while queue:
        _,cost,_,key=heapq.heappop(queue)
        if cost != costs.get(key):
            continue
        if key == end_id:
            break
        for candidate in neighbors(key):
            next_cost=cost+math.dist(positions[key],positions[candidate])
            if next_cost+1e-9 >= costs.get(candidate,float("inf")):
                continue
            costs[candidate]=next_cost
            previous[candidate]=key
            sequence+=1
            heapq.heappush(
                queue,
                (next_cost+math.dist(positions[candidate],end),next_cost,sequence,candidate),
            )
    if end_id not in previous:
        raise RuntimeError("No safe corridor connects the current pose to this target")

    route=[]
    key=end_id
    while True:
        route.append(positions[key])
        if key == start_id:
            break
        key=previous[key]
    route.reverse()

    # Keep only corners that are actually needed.  This gives Klipper a
    # small, smooth set of blended joint moves instead of a visible grid path.
    smoothed=[route[0]]
    cursor=0
    while cursor < len(route)-1:
        target=len(route)-1
        while target > cursor+1 and not _joint_segment_is_safe(route[cursor],route[target]):
            target-=1
        smoothed.append(route[target])
        cursor=target
    return smoothed

def queue_safe_world_route(plan):
    """Queue a boundary-safe joint corridor for one resolved world target."""
    start=(float(state["x"]),float(state["y"]))
    end=(float(plan["A"]),float(plan["B"]))
    route=safe_joint_route(start,end)
    if len(route) == 2:
        _gcode(
            f"ARM_MOVE_JOINT A={plan['A']:.5f} B={plan['B']:.5f} "
            f"C={plan['C']:.5f} D={plan['D']:.5f}",timeout=5)
        return route

    start_c=float(state["z"])
    start_d=float(state.get("linear_y",0.0))
    total=sum(math.dist(a,b) for a,b in zip(route,route[1:]))
    travelled=0.0
    commands=[]
    for before,point in zip(route,route[1:]):
        travelled+=math.dist(before,point)
        ratio=travelled/total if total > 1e-9 else 1.0
        c=start_c+(float(plan["C"])-start_c)*ratio
        d=start_d+(float(plan["D"])-start_d)*ratio
        commands.append(
            f"ARM_MOVE_JOINT A={point[0]:.5f} B={point[1]:.5f} "
            f"C={c:.5f} D={d:.5f}"
        )
    _gcode("\n".join(commands),timeout=8)
    return route

def smooth_dance_waypoints(home_z, cycles=10, arm_steps=1, base_steps=2):
    """Generate a looped dance with the same arm sweep at each base angle."""
    neutral=(100.0, 80.0)
    # User-selected dance poses. P2/P3 were supplied in Tool Y/Z and
    # converted through Assembly-1 IK using the current home calibration.
    arm_keyframes=[
        neutral,
        (29.8, 5.1),
        (96.0, 29.7),
        (135.2, 96.5),
        neutral,
    ]
    sweep=[0.0,30.0,60.0,90.0,120.0,150.0,180.0,150.0,120.0,90.0,60.0,30.0,0.0]
    points=[]

    # Visible, gentle lead-in from the current pose before the wider dance.
    # This avoids the old first move jumping directly from home to (100,80).
    start_x=float(state.get("x",29.8))
    start_y=float(state.get("y",5.1))
    lead=(29.8,5.1)
    for j in range(1,5):
        u=j/4.0
        s0=0.5-0.5*math.cos(math.pi*u)
        points.append((
            start_x+(lead[0]-start_x)*s0,
            start_y+(lead[1]-start_y)*s0,
            home_z
        ))
    for j in range(1,5):
        u=j/4.0
        s0=0.5-0.5*math.cos(math.pi*u)
        points.append((
            lead[0]+(neutral[0]-lead[0])*s0,
            lead[1]+(neutral[1]-lead[1])*s0,
            home_z
        ))

    def eased(a,b,u):
        s=0.5-0.5*math.cos(math.pi*u)
        return a+(b-a)*s

    def append_arm_loop(base_deg):
        z=home_z+base_deg
        n=max(1,int(arm_steps))
        for (x0,y0),(x1,y1) in zip(arm_keyframes,arm_keyframes[1:]):
            for j in range(1,n+1):
                u=j/n
                points.append((eased(x0,x1,u),eased(y0,y1,u),z))

    for _cycle in range(int(cycles)):
        points.append((neutral[0],neutral[1],home_z))
        for i,base_deg in enumerate(sweep):
            append_arm_loop(base_deg)
            if i+1 < len(sweep):
                next_base=sweep[i+1]
                n=max(2,int(base_steps))
                for j in range(1,n+1):
                    u=j/n
                    z=home_z+eased(base_deg,next_base,u)
                    # Move Arm X/Y during the base rotation too.  A sin^2 pulse
                    # starts/ends at neutral with zero slope, so there is no
                    # snap when leaving or arriving at each 30-degree stop.
                    pulse=math.sin(math.pi*u)**2
                    x=neutral[0]-20.0*pulse
                    y=neutral[1]-15.0*pulse
                    points.append((x,y,z))
    return points



_last_runtime_tune = None

def _planner_ahead_seconds():
    """Seconds of motion currently queued ahead of real time."""
    try:
        q=_moon_get("/printer/objects/query?toolhead", timeout=2.0)
        th=q["result"]["status"]["toolhead"]
        return max(0.0, float(th["print_time"])-float(th["estimated_print_time"]))
    except Exception:
        return 0.0

class _PlannerPacer:
    """Maintain a stable planner horizon without repeated Moonraker round trips."""
    def __init__(self, max_ahead=0.72):
        self.max_ahead=float(max_ahead)
        # One query establishes the starting horizon; after that we track
        # queued duration locally using monotonic time.
        initial=min(max(0.0,_planner_ahead_seconds()),self.max_ahead)
        self.queued_until=time.monotonic()+initial

    def wait(self):
        while True:
            check_stop()
            ahead=self.queued_until-time.monotonic()
            if ahead <= self.max_ahead:
                return
            time.sleep(min(0.04,max(0.008,(ahead-self.max_ahead)*0.45)))

    def add(self, duration):
        now=time.monotonic()
        self.queued_until=max(self.queued_until,now)+max(0.0,float(duration))

    def ahead(self):
        return max(0.0,self.queued_until-time.monotonic())

def _apply_runtime_tuning(force=False):
    """Apply limits in-place. No M400 and no extra-axis re-registration."""
    global _last_runtime_tune
    tune=(
        max(1.0,min(220.0,float(state.get("arm_speed",40.0)))),
        max(10.0,min(1800.0,float(state.get("arm_accel",200.0)))),
        max(0.0,min(40.0,float(state.get("arm_jerk",6.0)))),
        max(1.0,min(100.0,float(state.get("linear_speed",30.0)))),
        max(10.0,min(500.0,float(state.get("linear_accel",150.0)))),
        max(0.0,min(20.0,float(state.get("linear_jerk",3.0)))),
    )
    if force or tune != _last_runtime_tune:
        av,aa,ac,sv,sa,sc=tune
        _gcode(
            f"ARM_TUNE ARM_VELOCITY={av:.4f} ARM_ACCEL={aa:.4f} "
            f"ARM_CORNER={ac:.4f} SLIDE_VELOCITY={sv:.4f} "
            f"SLIDE_ACCEL={sa:.4f} SLIDE_CORNER={sc:.4f}",
            timeout=4
        )
        _last_runtime_tune=tune
    return tune

def queue_count_waypoints(waypoints, speed=40.0, accel=200.0,
                          linear_span=0.0, linear_speed=30.0):
    """Queue one complete smooth Dance cycle with FIXED tuning.

    No tuning checks occur inside the cycle.  The worker snapshots settings at
    the cycle boundary and the entire cycle is planned consistently.
    """
    speed=max(1.0,min(220.0,float(speed)))
    accel=max(10.0,min(1800.0,float(accel)))
    linear_speed=max(1.0,min(100.0,float(linear_speed)))

    pts=list(waypoints)
    cx,cy,cz=state["x"],state["y"],state["z"]
    d0=cd=float(state.get("linear_y",0.0) or 0.0)
    span=max(0.0,min(100.0,float(linear_span)))
    direction=1.0 if d0<=135.0 else -1.0
    total=max(1,len(pts)-1)

    _gcode("G91",timeout=4)

    # Larger, stable lookahead now that live retuning is deliberately deferred
    # to cycle boundaries.
    chunk=[]
    chunk_time=0.0
    target_chunk_time=0.50
    pacer=_PlannerPacer(max_ahead=1.15)

    for index,(tx,ty,tz) in enumerate(pts,1):
        check_stop()
        tx=float(tx); ty=float(ty); tz=max(0.0,float(tz))

        if state.get("boundary_enabled",True):
            nx,ny,clipped=clip_xy_path(cx,cy,tx,ty)
            if clipped or abs(nx-tx)>0.05 or abs(ny-ty)>0.05:
                raise RuntimeError(
                    f"Dance waypoint {index} outside Motor X/Y boundary "
                    f"({tx:.1f},{ty:.1f})"
                )
        else:
            nx,ny=tx,ty

        u=(index-1)/total
        tri=1.0-abs(2.0*u-1.0)
        nd=max(0.0,min(235.0,d0+direction*span*tri))

        dx,dy,dz,dd=nx-cx,ny-cy,tz-cz,nd-cd
        if abs(dx)+abs(dy)+abs(dz)+abs(dd)>1e-9:
            arm_dist=(dx*dx+dy*dy+dz*dz)**0.5
            slide_dist=abs(dd)
            arm_dt=arm_dist/speed if arm_dist>1e-9 else 0.0
            slide_dt=slide_dist/linear_speed if slide_dist>1e-9 else 0.0
            dt=max(arm_dt,slide_dt,0.015)

            combined=(dx*dx+dy*dy+dz*dz+dd*dd)**0.5
            feed=max(1.0,combined/dt)*60.0

            parts=["G1"]
            if abs(dx)>1e-9: parts.append(f"A{dx:.5f}")
            if abs(dy)>1e-9: parts.append(f"B{dy:.5f}")
            if abs(dz)>1e-9: parts.append(f"C{dz:.5f}")
            if abs(dd)>1e-9: parts.append(f"D{dd:.5f}")
            parts.append(f"F{feed:.1f}")
            chunk.append(" ".join(parts))
            chunk_time+=dt
            cx,cy,cz,cd=nx,ny,tz,nd

        if chunk and (chunk_time>=target_chunk_time or index==len(pts)):
            pacer.wait()
            state["message"]=(
                f"Dance cycle stream {index}/{len(pts)} | "
                f"queue {pacer.ahead():.2f}s"
            )
            _gcode("\n".join(chunk),timeout=8)
            pacer.add(chunk_time)
            chunk.clear()
            chunk_time=0.0

    # IMPORTANT: no M400 here.  The next cycle can be queued before this one
    # physically finishes, so Klipper never has to stop at the cycle seam.
    state.update(x=cx,y=cy,z=cz,linear_y=cd)

def jog_counts(dx,dy,dz,speed=40.0,accel=200.0):
    """Stream a straight A/B/C joint-space move with live retuning support."""
    state["arm_speed"]=max(1.0,min(220.0,float(speed)))
    state["arm_accel"]=max(10.0,min(1800.0,float(accel)))
    requested_x=max(0.0,state["x"]+dx)
    requested_y=max(0.0,state["y"]+dy)
    nz=max(0.0,state["z"]+dz)

    if not state.get("boundary_enabled", True):
        nx,ny,clipped=requested_x,requested_y,False
    else:
        nx,ny,clipped=clip_xy_path(
            state["x"],state["y"],requested_x,requested_y
        )

    sx,sy,sz=state["x"],state["y"],state["z"]
    fx,fy,fz=nx,ny,nz
    total_dist=((fx-sx)**2+(fy-sy)**2+(fz-sz)**2)**0.5

    if total_dist<1e-9:
        if clipped:
            state["message"]=f"BOUNDARY LIMIT — {BOUNDARY_YIELD:.1f}-count safety yield"
        return

    # <=1 degree joint-space segments: changing speed mid-move takes effect on
    # upcoming segments instead of waiting for one long G1 to finish.
    n=max(1,int(math.ceil(total_dist/1.0)))
    _apply_runtime_tuning(force=True)
    _gcode("G91",timeout=4)

    qx,qy,qz=sx,sy,sz
    chunk=[]
    chunk_time=0.0
    pacer=_PlannerPacer(max_ahead=0.65)

    for i in range(1,n+1):
        check_stop()
        u=i/n
        tx=sx+(fx-sx)*u
        ty=sy+(fy-sy)*u
        tz=sz+(fz-sz)*u
        ddx,ddy,ddz=tx-qx,ty-qy,tz-qz
        seg_dist=(ddx*ddx+ddy*ddy+ddz*ddz)**0.5

        arm_speed_now=max(1.0,min(220.0,float(state.get("arm_speed",speed))))
        dt=max(seg_dist/arm_speed_now,0.01)
        parts=["G1"]
        if abs(ddx)>1e-9: parts.append(f"A{ddx:.5f}")
        if abs(ddy)>1e-9: parts.append(f"B{ddy:.5f}")
        if abs(ddz)>1e-9: parts.append(f"C{ddz:.5f}")
        parts.append(f"F{arm_speed_now*60.0:.1f}")
        chunk.append(" ".join(parts))
        chunk_time+=dt
        qx,qy,qz=tx,ty,tz

        if chunk_time>=0.28 or i==n:
            pacer.wait()
            _apply_runtime_tuning()
            _gcode("\n".join(chunk),timeout=8)
            pacer.add(chunk_time)
            chunk.clear()
            chunk_time=0.0

    _gcode("M400",timeout=120)
    state.update(x=fx,y=fy,z=fz)
    if clipped:
        state["message"]=(
            f"BOUNDARY LIMITED — requested ({requested_x:.1f}, {requested_y:.1f}), "
            f"stopped at ({fx:.1f}, {fy:.1f}) with {BOUNDARY_YIELD:.1f}-count yield"
        )

def stream_linear_target(target, speed=None, accel=None):
    """Stream D in <=1 mm segments so live speed/accel changes can take effect."""
    if speed is not None:
        state["linear_speed"]=max(1.0,min(100.0,float(speed)))
    if accel is not None:
        state["linear_accel"]=max(10.0,min(500.0,float(accel)))
    start=float(state.get("linear_y",0.0))
    target=max(0.0,min(235.0,float(target)))
    dist=target-start
    if abs(dist)<1e-9:
        return

    n=max(1,int(math.ceil(abs(dist)/1.0)))
    _apply_runtime_tuning(force=True)
    _gcode("G90",timeout=4)

    chunk=[]
    chunk_time=0.0
    pacer=_PlannerPacer(max_ahead=0.65)
    for i in range(1,n+1):
        check_stop()
        u=i/n
        pos=start+dist*u
        prev=start+dist*((i-1)/n)
        seg=abs(pos-prev)
        slide_speed_now=max(1.0,min(100.0,float(state.get("linear_speed",speed or 30.0))))
        dt=max(seg/slide_speed_now,0.01)
        chunk.append(f"G1 D{pos:.5f} F{slide_speed_now*60.0:.1f}")
        chunk_time+=dt

        if chunk_time>=0.28 or i==n:
            pacer.wait()
            _apply_runtime_tuning()
            _gcode("\n".join(chunk),timeout=8)
            pacer.add(chunk_time)
            chunk.clear()
            chunk_time=0.0

    _gcode("M400",timeout=120)
    state["linear_y"]=target

def _validate_program_path(points):
    """Validate every programmed segment before putting any of it in motion."""
    previous={
        "a": float(state["x"]), "b": float(state["y"]),
        "c": float(state["z"]), "d": float(state.get("linear_y", 0.0)),
    }
    home_c=float(state.get("home_z_counts") or 2.5)
    for index, point in enumerate(points, 1):
        p=_program_point(point)
        if not state.get("linear_hardware", False) and abs(p["d"]) > 0.05:
            raise RuntimeError(f"Program point {index} uses rail travel but the rail is disabled")
        if not (home_c-0.05 <= p["c"] <= home_c+269.55):
            raise RuntimeError(f"Program point {index} Motor Z angle is outside its safe range")
        if not (0.0 <= p["d"] <= 235.0):
            raise RuntimeError(f"Program point {index} rail position is outside 0–235 mm")
        if state.get("boundary_enabled", True):
            nx,ny,clipped=clip_xy_path(previous["a"], previous["b"], p["a"], p["b"])
            if clipped or abs(nx-p["a"])>0.05 or abs(ny-p["b"])>0.05:
                raise RuntimeError(
                    f"Program point {index} or its path leaves the measured Motor X/Y boundary"
                )
        previous=p

def _queue_program(points, arm_speed, arm_accel, linear_speed, linear_accel, arm_corner):
    """Queue a single user program with blended waypoint transitions."""
    _validate_program_path(points)
    arm_speed=max(1.0,min(220.0,float(arm_speed)))
    arm_accel=max(10.0,min(1800.0,float(arm_accel)))
    linear_speed=max(1.0,min(100.0,float(linear_speed)))
    linear_accel=max(10.0,min(500.0,float(linear_accel)))
    arm_corner=max(0.0,min(50.0,float(arm_corner)))
    # Tune with the fastest requested waypoint speed. TIME below still makes
    # every individual point honor its own speed or explicit duration.
    arm_speed=max([arm_speed] + [_program_point(p)["speed"] for p in points])
    _gcode(
        f"ARM_TUNE ARM_VELOCITY={arm_speed:.4f} ARM_ACCEL={arm_accel:.4f} "
        f"ARM_CORNER={arm_corner:.4f} "
        f"SLIDE_VELOCITY={linear_speed:.4f} SLIDE_ACCEL={linear_accel:.4f} "
        f"SLIDE_CORNER={state['linear_jerk']:.4f}", timeout=5
    )
    current={"a":float(state["x"]),"b":float(state["y"]),
             "c":float(state["z"]),"d":float(state.get("linear_y",0.0))}
    chunk=[]
    chunk_time=0.0
    pacer=_PlannerPacer(max_ahead=1.10)
    for index, target in enumerate(points, 1):
        check_stop()
        p=_program_point(target)
        da=p["a"]-current["a"]; db=p["b"]-current["b"]
        dc=p["c"]-current["c"]; dd=p["d"]-current["d"]
        arm_distance=math.sqrt(da*da+db*db+dc*dc)
        auto_duration=max(arm_distance/p["speed"],abs(dd)/linear_speed,0.015)
        duration=p["time"] if p["time"] > 0 else auto_duration
        # Manual steppers do not inherit G90/G91 mode.  Their G1 parameters
        # are absolute, so feeding deltas here sent every waypoint to the
        # wrong pose.  ARM_MOVE_JOINT owns the manual-stepper semantics and
        # queues absolute targets into the same Klipper toolhead planner.
        chunk.append(
            f"ARM_MOVE_JOINT A={p['a']:.5f} B={p['b']:.5f} "
            f"C={p['c']:.5f} D={p['d']:.5f} TIME={duration:.5f}"
        )
        chunk_time+=duration
        current=p
        # A hold is an arrival boundary. Flush this waypoint even when its
        # move is shorter than the normal streaming chunk, then wait only
        # after Klipper confirms that it has physically arrived.
        if chunk_time>=0.50 or p["wait"] > 0 or index==len(points):
            pacer.wait()
            state["message"]=f"Program running {index}/{len(points)} · queued {pacer.ahead():.2f}s"
            _gcode("\n".join(chunk),timeout=8)
            pacer.add(chunk_time)
            chunk.clear()
            chunk_time=0.0
        if p["wait"] > 0:
            # A wait is deliberately an arrival boundary: finish this point,
            # then pause the worker. Points without waits retain corner blend.
            _gcode("M400",timeout=180)
            state.update(x=p["a"],y=p["b"],z=p["c"],linear_y=p["d"])
            state["message"]=f"Program waiting {p['wait']:.2f}s after {index}/{len(points)}"
            deadline=time.monotonic()+p["wait"]
            while time.monotonic() < deadline:
                check_stop()
                time.sleep(min(0.05, deadline-time.monotonic()))
            pacer=_PlannerPacer(max_ahead=1.10)
    _gcode("M400",timeout=180)
    state.update(x=current["a"],y=current["b"],z=current["c"],linear_y=current["d"])

def _program_worker(points, settings):
    global program_thread
    try:
        with lock:
            ensure_homed_for_motion()
            state["busy"]=True
            state["message"]=f"Program starting — {len(points)} waypoint{'s' if len(points)!=1 else ''}"
            _queue_program(points, **settings)
            state["message"]="Program complete"
    except MotionCancelled:
        state["message"]="Program stopped"
        stop_event.clear()
    except Exception as exc:
        state["message"]=f"Program error: {exc}"
    finally:
        state["busy"]=False
        with program_thread_lock:
            program_thread=None

def _emergency_kill(reason="emergency stop"):
    """One KILL implementation shared by the UI and confirmed sensor faults."""
    stop_event.set()
    state["message"]=f"KILL — {reason}; re-home required"
    state["busy"]=False
    state["homed"]=False
    try:
        _moon_post("/printer/emergency_stop", timeout=3)
    except Exception:
        pass

def _sensor_can_run_program():
    if SENSOR is not None and (SENSOR.guard.fault or SENSOR.monitor.fault_latched):
        return False, 'sensor fault requires acknowledgement'
    _sync_state_from_arm_robot()
    if not state.get("homed"):
        return False, "robot is not homed"
    if state.get("busy") or _program_running():
        return False, "robot is not idle"
    return True, ""

def _sensor_run_program(name):
    """Run a named saved program without modifying the Program editor."""
    global program_thread
    ok, reason=_sensor_can_run_program()
    if not ok:
        raise RuntimeError(reason)
    points, _defaults=_load_named_program(name)
    if not points:
        raise RuntimeError(f"Saved program {name!r} is empty")
    _validate_program_path(points)
    settings={
        "arm_speed":max(1.0,min(220.0,float(state["arm_speed"]))),
        "arm_accel":max(10.0,min(1800.0,float(state["arm_accel"]))),
        "linear_speed":max(1.0,min(100.0,float(state["linear_speed"]))),
        "linear_accel":max(10.0,min(500.0,float(state["linear_accel"]))),
        "arm_corner":max(0.0,min(50.0,float(state["arm_jerk"]))),
    }
    stop_event.clear()
    state["busy"]=True
    state["message"]=f"Sensor selected saved program — {name}"
    with program_thread_lock:
        if program_thread and program_thread.is_alive():
            state["busy"]=False
            raise RuntimeError("another program started before sensor trigger")
        program_thread=threading.Thread(
            target=_program_worker,args=(list(points),settings),daemon=True
        )
        program_thread.start()

correction_thread = None
correction_thread_lock = threading.Lock()

def _queue_recovery_stage(plan, label):
    """Queue one recovery stage, wait for arrival, then refresh live joints."""
    check_stop()
    state["message"] = "Tilt correction — " + label
    queue_safe_world_route(plan)
    _gcode("M400", timeout=180)
    _sync_state_from_arm_robot()

def _recovery_plan_at_radius(unit_x, unit_y, radius, z, rail, seed):
    """Resolve one world point at the current base heading and fixed rail."""
    plan=plan_world_target(unit_x*radius, rail+unit_y*radius, z, rail, seed=seed)
    safe_joint_route((float(seed["a"]),float(seed["b"])), (float(plan["A"]),float(plan["B"])))
    return plan

def _find_inboard_recovery_plan(unit_x, unit_y, current_radius, z, rail, seed):
    """Find the smallest reachable radial distance at a fixed safe height."""
    samples=workspace_payload().get("side_samples", [])
    radii=sorted({round(float(sample[0]),2) for sample in samples
                  if float(sample[0]) < current_radius-1.0})
    for radius in radii:
        try:
            return _recovery_plan_at_radius(unit_x,unit_y,radius,z,rail,seed)
        except RuntimeError:
            continue
    raise RuntimeError("no inboard recovery point is reachable inside the measured boundary")

def _find_high_recovery_plan(unit_x, unit_y, radius, minimum_z, rail, seed):
    """Find the highest modeled point at the chosen inboard radius."""
    top=float(workspace_payload()["bounds"]["z"][1])
    for z in range(int(math.floor(top)), int(math.ceil(minimum_z))-1, -2):
        try:
            return _recovery_plan_at_radius(unit_x,unit_y,radius,float(z),rail,seed)
        except RuntimeError:
            continue
    raise RuntimeError("no higher recovery point is reachable inside the measured boundary")

def _queue_saved_linkage_recovery(name):
    """Run a recorded recovery shape using Motor X/Y only.

    The saved route supplies the linkage shape; the live base and rail are
    deliberately retained so a tilt recovery cannot rotate the base or move
    the optional rail.  Each saved point is an arrival boundary so the
    operator-recorded counter-clockwise path is preserved.
    """
    points, _defaults=_load_named_program(name)
    if not points:
        raise RuntimeError("selected lock-up recovery route is empty")

    fixed_c=float(state["z"])
    fixed_d=float(state.get("linear_y",0.0))
    current=(float(state["x"]),float(state["y"]))
    for index, raw in enumerate(points,1):
        check_stop()
        target=_program_point(raw)
        route=safe_joint_route(current,(float(target["a"]),float(target["b"])))
        commands=[
            f"ARM_MOVE_JOINT A={a:.5f} B={b:.5f} C={fixed_c:.5f} D={fixed_d:.5f}"
            for a,b in route[1:]
        ]
        if commands:
            state["message"]=f"Tilt correction — saved X/Y path {index}/{len(points)}"
            _gcode("\n".join(commands),timeout=8)
            _gcode("M400",timeout=180)
        current=(float(target["a"]),float(target["b"]))
        state.update(x=current[0],y=current[1],z=fixed_c,linear_y=fixed_d)

def _start_tilt_warning_flash():
    """Flash red irregularly while the automatic correction is in motion."""
    if SENSOR is None:
        return None, None
    stop=threading.Event()

    def flash():
        try:
            while not stop.is_set():
                SENSOR.set_led("red")
                if stop.wait(random.uniform(0.08,0.22)):
                    break
                SENSOR.set_led("off")
                stop.wait(random.uniform(0.10,0.32))
        except Exception:
            # Warning feedback must never interrupt a safety recovery.
            pass
        finally:
            try:
                SENSOR.set_led("off")
            except Exception:
                pass

    thread=threading.Thread(target=flash,daemon=True,name="tilt-warning-flash")
    thread.start()
    return stop, thread

def _tilt_correction_worker(reason):
    """Slow, boundary-on recovery from a sustained triplate tilt fault."""
    global correction_thread
    saved_tune=None
    flash_stop=None
    flash_thread=None
    recovery_started=False
    try:
        with lock:
            _sync_state_from_arm_robot()
            if not state.get("boundary_enabled", True):
                raise RuntimeError("Boundary protection must be ON for tilt correction")
            if not state.get("homed"):
                raise RuntimeError("robot must be homed for tilt correction")
            if state.get("busy") or _program_running():
                raise RuntimeError("robot is busy; tilt correction was not started")
            if SENSOR is None or SENSOR.monitor.kill_enabled:
                raise RuntimeError("tilt correction is unavailable while KILL is enabled")

            state["busy"]=True
            recovery_started=True
            state["message"]="Tilt correction starting — " + str(reason)
            flash_stop,flash_thread=_start_tilt_warning_flash()
            saved_tune=(state["arm_speed"],state["arm_accel"],state["arm_jerk"],
                        state["linear_speed"],state["linear_accel"],state["linear_jerk"])
            _gcode("ARM_TUNE ARM_VELOCITY=20 ARM_ACCEL=80 ARM_CORNER=3 "
                   "SLIDE_VELOCITY=15 SLIDE_ACCEL=60 SLIDE_CORNER=2", timeout=5)

            saved_route=str(SENSOR.recovery_program or "").strip()
            if saved_route:
                _queue_saved_linkage_recovery(saved_route)
                state.update(homed=True,home_required=False)
                state["message"]="Tilt correction complete — normal motion available"
                return

            initial=kinematic_state(state["x"],state["y"],state["z"])["tool"]
            rail=float(state.get("linear_y",0.0))
            x=float(initial["x"])
            y=float(initial["y"])+rail
            z=float(initial["z"])
            radius=math.hypot(x,y-rail)
            if radius <= 1.0:
                raise RuntimeError("current tool heading is too close to the base axis for correction")
            unit_x=x/radius
            unit_y=(y-rail)/radius

            seed={"a":state["x"],"b":state["y"],"c":state["z"],"d":rail}
            hop=_recovery_plan_at_radius(unit_x,unit_y,radius,z+5.0,rail,seed)
            _queue_recovery_stage(hop,"5 mm clearance hop")

            seed={"a":state["x"],"b":state["y"],"c":state["z"],"d":state.get("linear_y",0.0)}
            inboard=_find_inboard_recovery_plan(unit_x,unit_y,radius,z+5.0,rail,seed)
            _queue_recovery_stage(inboard,"retracting toward the base axis")

            seed={"a":state["x"],"b":state["y"],"c":state["z"],"d":state.get("linear_y",0.0)}
            raised=_find_high_recovery_plan(unit_x,unit_y,float(inboard["radius_mm"]),z+5.0,rail,seed)
            _queue_recovery_stage(raised,"raising to the highest modeled safe height")
            state.update(homed=True,home_required=False)
            state["message"]="Tilt correction complete — normal motion available"
    except MotionCancelled:
        if recovery_started:
            state.update(homed=False,home_required=True)
            state["message"]="Tilt correction stopped before completion — press Home before normal motion"
        else:
            state["message"]="Tilt correction stopped"
        stop_event.clear()
    except Exception as exc:
        if recovery_started:
            state.update(homed=False,home_required=True)
            state["message"]="Tilt correction incomplete — press Home before normal motion: " + str(exc)
        else:
            state["message"]="Tilt correction not run: " + str(exc)
    finally:
        if flash_stop is not None:
            flash_stop.set()
        if flash_thread is not None:
            flash_thread.join(timeout=1)
        if saved_tune is not None:
            av,aa,ac,sv,sa,sc=saved_tune
            try:
                _gcode(f"ARM_TUNE ARM_VELOCITY={av:.4f} ARM_ACCEL={aa:.4f} "
                       f"ARM_CORNER={ac:.4f} SLIDE_VELOCITY={sv:.4f} "
                       f"SLIDE_ACCEL={sa:.4f} SLIDE_CORNER={sc:.4f}", timeout=5)
            except Exception:
                pass
        state["busy"]=False
        with correction_thread_lock:
            correction_thread=None

def _request_tilt_correction(reason):
    """Start at most one optional correction worker after a confirmed fault."""
    global correction_thread
    with correction_thread_lock:
        if correction_thread and correction_thread.is_alive():
            return
        correction_thread=threading.Thread(
            target=_tilt_correction_worker,args=(reason,),daemon=True,name="tilt-correction"
        )
        correction_thread.start()

def _dance_worker(speed, accel, linear_speed, linear_accel):
    """Continuous Dance with tuning sampled only at cycle boundaries."""
    global dance_thread
    cycle=0
    try:
        state["busy"]=True
        state["message"]="Dance starting — coordinated Motor X/Y/Z and rail"
        while not stop_event.is_set():
            check_stop()
            cycle+=1

            # Freeze all motion settings for this whole cycle.
            cycle_arm_speed=max(1.0,min(220.0,float(state.get("arm_speed",speed))))
            cycle_arm_accel=max(10.0,min(1800.0,float(state.get("arm_accel",accel))))
            cycle_arm_corner=max(0.0,min(40.0,float(state.get("arm_jerk",6.0))))
            cycle_slide_speed=max(1.0,min(100.0,float(state.get("linear_speed",linear_speed))))
            cycle_slide_accel=max(10.0,min(500.0,float(state.get("linear_accel",linear_accel))))
            cycle_slide_corner=max(0.0,min(20.0,float(state.get("linear_jerk",3.0))))

            # Apply exactly once at the cycle boundary.
            _gcode(
                f"ARM_TUNE ARM_VELOCITY={cycle_arm_speed:.4f} "
                f"ARM_ACCEL={cycle_arm_accel:.4f} "
                f"ARM_CORNER={cycle_arm_corner:.4f} "
                f"SLIDE_VELOCITY={cycle_slide_speed:.4f} "
                f"SLIDE_ACCEL={cycle_slide_accel:.4f} "
                f"SLIDE_CORNER={cycle_slide_corner:.4f}",
                timeout=4
            )

            home_z=(state["home_z_counts"]
                    if state["home_z_counts"] is not None else state["z"])
            waypoints=smooth_dance_waypoints(
                home_z,cycles=1,arm_steps=5,base_steps=5
            )
            state["message"]=(
                f"Dance cycle {cycle} — fixed settings: "
                f"arm {cycle_arm_speed:.1f} deg/s, "
                f"slide {cycle_slide_speed:.1f} mm/s"
            )
            queue_count_waypoints(
                waypoints,
                speed=cycle_arm_speed,
                accel=cycle_arm_accel,
                linear_span=100.0,
                linear_speed=cycle_slide_speed,
            )
        state["message"]="Dance stopped"
    except MotionCancelled:
        state["message"]="Dance stopped"
    except Exception as exc:
        state["message"]=f"Dance error: {exc}"
    finally:
        state["busy"]=False
        with dance_thread_lock:
            dance_thread=None



HTML = r'''<!doctype html>
<meta charset="utf-8">
<title>Robot Arm Control</title>
<style>
:root{color-scheme:dark;--bg:#080b0f;--panel:#11161c;--panel2:#0c1116;--line:#28323c;--text:#eef2f5;--muted:#8d98a4;--accent:#52a9e8;--safe:#6fd49b;--warn:#e8bc59}
*{box-sizing:border-box}html,body{margin:0;width:100%;height:100%;overflow:hidden}body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:var(--bg);color:var(--text)}
main{height:100vh;width:100vw;padding:10px;display:grid;grid-template-rows:44px minmax(0,1fr);gap:8px;overflow:hidden}
.topbar{display:flex;align-items:center;gap:8px;padding:0 6px}.title{font-size:20px;font-weight:800}.viewTabs{display:flex;gap:5px;margin-right:auto}.viewTabs button{padding:5px 9px}.status{display:flex;align-items:center;gap:6px}.badge{padding:5px 9px;border:1px solid var(--line);border-radius:999px;font-size:11px;font-weight:700;background:#0c1116;white-space:nowrap}.good{color:var(--safe)}.warn{color:var(--warn)}.bad{color:#ff9696}
button,input{font:inherit}button{padding:7px 10px;border-radius:8px;border:1px solid #34404b;background:#202832;color:#fff;font-size:12px;cursor:pointer}button:hover{background:#2b3540}button.primary{background:#23608d;border-color:#347aad}button.danger{background:#6d2b2b;border-color:#934545}button.active{outline:2px solid var(--accent);background:#20384d}button:disabled{opacity:.55;cursor:not-allowed}
input{width:72px;padding:6px 7px;border:1px solid #34404b;border-radius:7px;background:#090d12;color:#fff;font-size:12px}.wide{width:88px}
.dashboard{min-height:0;display:grid;grid-template-columns:minmax(0,1.18fr) minmax(500px,.82fr);grid-template-rows:minmax(0,1fr);gap:8px}.card{min-width:0;min-height:0;background:var(--panel);border:1px solid var(--line);border-radius:12px;overflow:hidden}.cardTitle{display:flex;align-items:flex-start;justify-content:space-between;gap:10px;padding:10px 12px 7px}.cardTitle h2{margin:0;font-size:14px}.muted{color:var(--muted);font-size:10px;line-height:1.25}.toolbar{display:flex;align-items:center;gap:5px;flex-wrap:wrap}
.armCard{grid-column:1;grid-row:1;display:grid;grid-template-rows:auto auto minmax(0,1fr) auto;padding-bottom:6px}.endstopStrip{display:flex;align-items:center;gap:6px;padding:0 10px 7px}.endstopStripLabel{font-size:9px;color:var(--muted);font-weight:750;text-transform:uppercase;letter-spacing:.06em;margin-right:2px}.endstopState{min-width:71px;padding:4px 6px;border:1px solid #2b3945;border-radius:6px;background:#0b1015;color:#aeb9c4;font-size:9px;font-weight:800;text-align:center}.endstopState.triggered{border-color:#d8963f;background:#2a1b0a;color:#ffca75}.endstopState.stale{opacity:.55;color:#87929e}.endstopState.disabled{opacity:.5}.viewGrid{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:7px;padding:0 10px 3px;min-height:0}.viewPane{position:relative;min-width:0;min-height:0;border:1px solid var(--line);border-radius:8px;overflow:hidden;background:radial-gradient(circle at 50% 35%,#171e26,#07090c 72%)}.viewPane canvas{display:block;width:100%;height:100%;min-height:0}.viewPane.free canvas{cursor:grab}.viewLabel{position:absolute;top:5px;left:7px;z-index:2;font-size:9px;font-weight:800;letter-spacing:.08em;color:#aeb9c4;background:rgba(7,10,13,.72);border:1px solid #2c3742;border-radius:5px;padding:2px 5px;pointer-events:none}.readouts{display:grid;grid-template-columns:repeat(6,1fr);gap:4px;padding:5px 10px 0}.readout{padding:3px 4px;border:1px solid #25303a;border-radius:7px;background:#0b1015;text-align:center}.readout .k{font-size:8px;color:var(--muted);text-transform:uppercase}.readout .v{font-size:13px;font-weight:700;margin-top:1px}
.controls{grid-column:2;grid-row:1;display:flex;flex-direction:column;padding:0 10px 7px;overflow-y:auto;overflow-x:hidden;scrollbar-gutter:stable}.controls>.cardTitle{padding:8px 2px 6px;align-items:center}.motionSettings{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:5px 7px;padding:0 0 7px}.setting{display:grid;grid-template-columns:minmax(0,1fr) 66px auto;align-items:center;gap:5px;padding:5px 7px;border:1px solid #242e38;border-radius:7px;background:#0c1116}.setting span{font-size:9px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.setting input{width:66px;padding:5px 6px}.setting b{font-size:9px;color:#b7c0c9;font-weight:650;white-space:nowrap}.group{background:var(--panel2);border:1px solid #242e38;border-radius:8px;padding:6px;margin-bottom:5px}.groupTitle{font-size:8px;color:var(--muted);text-transform:uppercase;letter-spacing:.09em;margin-bottom:4px}.axisRow{display:grid;grid-template-columns:62px 34px 58px 34px;gap:5px;align-items:center;margin:3px 0}.axisName{font-weight:750;font-size:12px}.axisRow .val{text-align:center;font-weight:700;font-variant-numeric:tabular-nums}.moveGrid{display:grid;grid-template-columns:1fr 1fr;gap:4px}.fieldrow{display:flex;align-items:center;gap:6px;flex-wrap:wrap}.fieldrow label{font-size:10px;color:var(--muted)}details{font-size:9px;color:var(--muted);margin-top:0}summary{cursor:pointer}
.plotCard{display:grid;grid-template-rows:auto minmax(0,1fr);padding:0 10px 10px;min-height:0}.plotCard .cardTitle{padding:8px 4px 7px;align-items:center}.plotCard .cardTitle h2{font-size:13px}.plotCard svg{width:100%;height:100%;display:block;background:#090c10;border:1px solid var(--line);border-radius:8px;overflow:visible}.workspaceCard{grid-column:1;grid-row:2}.boundaryCard{grid-column:2;grid-row:2}.legend{font-size:9px;color:var(--muted)}#rowsWrap{display:none}
#msg{font-size:11px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:min(42vw,560px)}
.fullPanel{display:none;min-height:0;overflow:auto;padding:2px}.fullPanel.active{display:grid}
.debugGrid{grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;height:100%}.debugCard{padding:10px;display:flex;flex-direction:column;min-height:250px}.debugCard canvas{width:100%;height:220px;background:#090c10;border:1px solid var(--line);border-radius:8px}
.metricTable{width:100%;border-collapse:collapse;font-size:11px;font-variant-numeric:tabular-nums}.metricTable th,.metricTable td{padding:4px 6px;border-bottom:1px solid #25303a;text-align:right}.metricTable th:first-child,.metricTable td:first-child{text-align:left}
#debugEvents{font:11px ui-monospace,SFMono-Regular,Menlo,monospace;white-space:pre-wrap;overflow:auto;max-height:260px}
.workspaceFull{grid-template-columns:minmax(0,1fr) 320px;grid-template-rows:minmax(540px,1fr) 300px;gap:8px;height:auto;min-height:100%}
.workspaceViewer{display:grid;grid-template-rows:auto minmax(0,1fr);padding:10px;min-height:0}
.workspaceToolbar{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:0 0 8px;flex-wrap:wrap}
.workspaceToolbar .viewModes{display:flex;gap:5px;flex-wrap:wrap}
.workspaceToolbar .viewModes button{padding:6px 10px}
.workspaceHint{font-size:10px;color:var(--muted)}
.workspaceFull canvas{width:100%;height:100%;min-height:520px;background:#090c10;border:1px solid var(--line);border-radius:8px;touch-action:none}
.workspaceInfo{padding:12px;overflow:auto}
.workspaceInfo h2{margin:0 0 4px;font-size:20px}
.workspaceStep{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:.09em;margin-bottom:5px}
.workspaceTargetGrid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.workspaceTargetGrid label{font-size:10px;color:var(--muted)}
.workspaceTargetGrid input{width:100%;margin-top:3px}
.workspaceResult{padding:8px 9px;border:1px solid #28323c;border-radius:8px;background:#0b1015;font-size:11px;line-height:1.5}
.workspaceActions{display:grid;grid-template-columns:1fr 1.35fr;gap:6px;margin-top:8px}
.workspaceActions button{min-height:38px}
.axisGrid2x2{display:grid;grid-template-columns:1fr 1fr;gap:7px}
.axisTile{display:grid;grid-template-columns:auto 44px minmax(52px,1fr) 44px;align-items:center;gap:6px;border:1px solid #28323c;border-radius:8px;padding:6px 7px;background:#0b1015}
.axisTile .axisLabel{font-weight:750;font-size:11px;white-space:nowrap}
.axisTile .axisValue{text-align:center;font-weight:750;font-variant-numeric:tabular-nums}
.workspaceSlices{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:8px;min-height:0}
.sliceCard{display:grid;grid-template-rows:auto minmax(0,1fr);padding:8px;min-height:0}
.sliceHead{display:grid;grid-template-columns:auto 1fr auto;align-items:center;gap:8px;padding-bottom:6px}
.sliceHead b{font-size:12px}
.sliceHead input[type=range]{width:100%;padding:0}
.sliceVal{font-size:10px;color:var(--muted);min-width:58px;text-align:right}
.workspaceFull canvas{min-height:250px}
.workspaceMeasured{grid-column:1/-1;display:grid;grid-template-columns:minmax(0,1.18fr) minmax(420px,.82fr);gap:8px;min-height:300px}.workspaceMeasured .plotCard{height:300px}
.programFull{grid-template-columns:minmax(390px,.8fr) minmax(620px,1.2fr);grid-template-rows:auto minmax(0,1fr);gap:8px;height:100%;overflow:hidden;padding:4px}.programPageToolbar{grid-column:1/-1;display:flex;align-items:center;justify-content:space-between;gap:10px;padding:6px 10px;background:var(--panel);border:1px solid #293542;border-radius:10px}.programPageToolbar strong{font-size:12px}.programPageToolbar .muted{font-size:10px;margin-top:1px}.programPageToolbar .programMode{min-width:300px}.programCard{padding:10px;overflow:hidden;display:flex;flex-direction:column}.programCard h2{margin:0;font-size:16px}.programCard>.muted{font-size:10px}.programPose{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin:7px 0}.programPose div{padding:5px 6px;border:1px solid #28323c;border-radius:7px;background:#0b1015;font-size:10px}.programPose b{display:block;margin-top:1px;font-variant-numeric:tabular-nums}.programSection{padding:8px;margin:0 0 6px;background:var(--panel2);border:1px solid #242e38;border-radius:8px}.programSectionTitle{display:flex;align-items:baseline;gap:7px;margin-bottom:6px;font-size:11px;font-weight:750}.programStep{display:inline-grid;place-items:center;width:17px;height:17px;border-radius:50%;background:#194a6d;color:#d8efff;font-size:10px}.programMode{display:flex;gap:4px;padding:3px;background:#090d12;border:1px solid #293542;border-radius:8px}.programMode button{flex:1;padding:5px 8px}.programMode button.active{background:#1b5d89;border-color:#3795d3;color:#fff}.programCoordinateGrid{display:grid;gap:0}.programCoordinateGroup{padding:6px;border:1px solid #293542;border-radius:7px;background:#0b1015}.programFull[data-coordinate-mode="joint"] .programCoordinateGroup.cartesian,.programFull[data-coordinate-mode="cartesian"] .programCoordinateGroup.joint{display:none}.programCoordinateGroup h3{margin:0 0 4px;font-size:10px;color:#cdd8e2}.programTargets{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:6px}.programTargets label{font-size:10px;color:var(--muted)}.programTargets input{width:100%;margin-top:2px;padding:5px 6px}.programJog{display:none}.programJog.active{display:block}.programDetails{margin:6px 0 0;border:1px solid #242e38;border-radius:8px;background:#0b1015;padding:6px}.programDetails summary{font-size:10px;color:#c7d0d9;font-weight:650}.programDetails[open] summary{margin-bottom:7px}.programDetails .group{margin:0 0 6px}.programDetails .group:last-child{margin-bottom:0}.programActions{display:grid;gap:6px;padding:0}.programActions .toolbar{justify-content:flex-start;flex-wrap:wrap}.programActionHint{font-size:10px;color:var(--muted);line-height:1.25}.programLibrary{display:grid;gap:4px;justify-items:end}.programLibrary .libraryRow{display:flex;align-items:center;justify-content:flex-end;gap:5px}.programLibrary label{display:flex;align-items:center;gap:5px;font-size:10px;color:var(--muted);white-space:nowrap}.programLibrary select,.programLibrary input{width:145px;padding:4px 6px}.programLibrary button{padding:5px 8px}.programReviewContent{display:grid;grid-template-columns:minmax(225px,.82fr) minmax(380px,1.18fr);gap:8px;align-items:start;margin-top:6px}.programPathViews{margin:0}.programPath{width:100%;height:205px;background:#090d12;border:1px solid #293542;border-radius:9px;display:block}.programPath.drag{cursor:grab}.programPath.drag:active{cursor:grabbing}.programReviewSide{display:grid;gap:6px;min-width:0}.programTableWrap{flex:1;min-height:0;max-height:none;overflow:auto;border:1px solid #293542;border-radius:9px;background:#090d12}.programTableWrap .programTable{margin-top:0!important}.programTable{width:100%;border-collapse:collapse;font-size:12px;font-variant-numeric:tabular-nums}.programTable th,.programTable td{padding:6px;border-bottom:1px solid #25303a;text-align:right}.programTable th:first-child,.programTable td:first-child{text-align:left}.programTable button{padding:4px 7px}.programTable input{width:66px;min-width:0;padding:4px}@media(max-width:1100px){.programFull{height:auto;overflow:auto}.programReviewContent{grid-template-columns:1fr}.programPath{height:240px}.programTableWrap{max-height:340px}.programPageToolbar{align-items:flex-start;flex-direction:column}.programPageToolbar .programMode{min-width:0;width:100%}}

.programLibrary{gap:6px;min-width:430px}.programLibrary .libraryLabel{font-size:10px;color:var(--muted);font-weight:700;text-transform:uppercase;letter-spacing:.06em;white-space:nowrap}.programLibrary select,.programLibrary input{width:160px}.programLibrary .danger{font-size:10px}.programLibraryHeader{display:flex;align-items:center;justify-content:flex-end;gap:7px}.programLibraryActions{display:flex;justify-content:flex-end;gap:5px}.programLibraryHint{font-size:10px;color:var(--muted);text-align:right;line-height:1.2}.programLibrary .programLibraryActions button{white-space:nowrap;font-size:12px}.programLibrary .programLibraryActions .danger{font-size:12px}


.programOrder{white-space:nowrap}.programOrder b{display:inline-block;min-width:18px;text-align:center}.programDrag{display:inline-block;padding:3px 4px;color:#aeb9c4;cursor:grab;user-select:none;font-weight:800;letter-spacing:-2px}.programDrag:active{cursor:grabbing}.programRowActions{white-space:nowrap}.programRowActions button{min-width:27px;margin-left:3px}.programTable tr.programDropTarget td{background:#173147}.programTable tr.programDragging{opacity:.45}
@media(max-height:660px) and (min-width:1101px){
  .dashboard{grid-template-rows:330px minmax(0,1fr)}
  .readouts{display:none}
  .controls .cardTitle{padding-top:3px;padding-bottom:2px}
  .controls .group{padding:4px;margin-bottom:3px}
}
@media(max-width:1100px){html,body{overflow:auto}main{height:auto;min-height:100vh}.dashboard{grid-template-columns:1fr;grid-template-rows:420px auto 280px 280px}.armCard{grid-column:1;grid-row:1}.controls{grid-column:1;grid-row:2;overflow:visible}.workspaceCard{grid-column:1;grid-row:3}.boundaryCard{grid-column:1;grid-row:4}}
</style>
<style>
#cameraPanel[hidden]{display:none!important}
#cameraPanel{position:fixed;right:16px;bottom:16px;width:min(420px,calc(100vw - 32px));z-index:50;background:#10171e;border:1px solid #435464;border-radius:12px;box-shadow:0 8px 30px #0008;overflow:hidden;resize:both;min-width:260px;min-height:200px;max-width:90vw;max-height:80vh}
.cameraHeader{display:flex;align-items:center;justify-content:space-between;padding:8px 10px;gap:8px;cursor:move;touch-action:none}.cameraHeader button{font-size:12px;padding:5px 9px;cursor:pointer}
#cameraFeed{display:block;width:100%;max-height:65vh;object-fit:contain;background:#080c10}#cameraInfo{padding:5px 10px;color:#9caab8;font-size:11px}
</style>
<main>
  <div class="topbar">
    <div class="title">EnderArm Control</div>
    <div class="viewTabs"><button id="tabControl" class="active">Control</button><button id="tabWorkspace">Workspace</button><button id="tabProgram">Programmed movements</button><button id="tabSensors">Sensors</button><button id="tabDebug">Debug</button><button id="tabSettings">Settings</button></div>
    <div id="msg">Loading…</div>
    <div class="status">
      <button id="cameraToggle" aria-expanded="false" aria-controls="cameraPanel">Camera</button>
      <span id="homedBadge" class="badge bad">NOT HOMED</span>
      <span id="motionBadge" class="badge">IDLE</span>
      <button id="boundaryToggle" class="good">Boundary ON</button>
      <button id="home" class="primary">Home</button>
      <button id="stop">STOP</button><button id="kill" class="danger">KILL</button>
    </div>
  </div>

  <div class="dashboard">
    <section class="card armCard">
      <div class="cardTitle">
        <div><h2>Live arm</h2><div class="muted">Motor angles θ · tool position in world X/Y/Z</div></div>
        <span class="badge good">LIVE</span>
      </div>
      <div class="endstopStrip" id="endstopStrip" aria-label="Physical endstop state; click to refresh" title="Click to refresh endstop state">
        <span class="endstopStripLabel" id="endstopStripLabel">Endstops · click to refresh</span>
        <span class="endstopState" id="endstopXLive">Motor X · —</span>
        <span class="endstopState" id="endstopYLive">Motor Y · —</span>
        <span class="endstopState" id="endstopZLive">Motor Z · —</span>
        <span class="endstopState disabled" id="endstopDLive">Rail · disabled</span>
      </div>
      <div class="viewGrid">
        <div class="viewPane"><span class="viewLabel">XY · TOP</span><canvas id="armXY" width="500" height="260"></canvas></div>
        <div class="viewPane"><span class="viewLabel">YZ · SIDE</span><canvas id="armYZ" width="500" height="260"></canvas></div>
        <div class="viewPane"><span class="viewLabel">XZ · FRONT</span><canvas id="armXZ" width="500" height="260"></canvas></div>
        <div class="viewPane free"><span class="viewLabel">FREE · DRAG</span><canvas id="armFree" width="500" height="260"></canvas></div>
      </div>
      <div class="readouts">
        <div class="readout"><div class="k">Motor X θ</div><div class="v" id="angLower">—</div></div>
        <div class="readout"><div class="k">Motor Y θ</div><div class="v" id="angUpper">—</div></div>
        <div class="readout"><div class="k">Motor Z θ</div><div class="v" id="angBase">—</div></div>
        <div class="readout"><div class="k">World X</div><div class="v" id="cartX">—</div></div>
        <div class="readout"><div class="k">World Y</div><div class="v" id="cartY">—</div></div>
        <div class="readout"><div class="k">World Z</div><div class="v" id="cartZ">—</div></div>
      </div>
      <span id="angBF" style="display:none"></span><span id="angFCH" style="display:none"></span>
    </section>

    <section class="card controls">
      <div class="cardTitle">
        <div><h2>Move robot</h2><div class="muted">Use motor angles θ (degrees) or tool position (mm)</div></div>
        <button id="dance" class="primary">Dance ∞</button>
      </div>

      <details class="group" style="margin-bottom:8px">
        <summary>Motion settings</summary>
        <div class="toolbar" style="margin-top:7px;gap:10px;justify-content:flex-end">
          <label class="muted"><input id="accelEnabled" type="checkbox" checked> Acceleration limiting</label>
          <label class="muted"><input id="cornerEnabled" type="checkbox" checked> Corner blending</label>
        </div>
        <div class="motionSettings" style="margin-top:6px">
          <label class="setting"><span>Arm speed</span><input id="armSpeed" type="number" min="1" max="220" step="1" value="80"><b>°/s</b></label>
          <label class="setting"><span>Arm accel</span><input id="armAccel" type="number" min="10" max="1800" step="10" value="250"><b>°/s²</b></label>
          <label class="setting"><span>Arm corner</span><input id="armJerk" type="number" min="0" max="40" step="0.5" value="18"><b>°/s</b></label>
          <label class="setting"><span>Slide speed</span><input id="linearSpeed" type="number" min="1" max="100" step="1" value="25"><b>mm/s</b></label>
          <label class="setting"><span>Slide accel</span><input id="linearAccel" type="number" min="10" max="500" step="10" value="100"><b>mm/s²</b></label>
          <label class="setting"><span>Slide corner</span><input id="linearJerk" type="number" min="0" max="20" step="0.5" value="5"><b>mm/s</b></label>
        </div>
      </details>

      <div class="group">
        <div class="groupTitle">Jog motor angles · degrees</div>
        <div class="toolbar" style="margin-bottom:6px"><span class="muted">Step °</span><button class="stepPreset" data-step-value="0.25">0.25°</button><button class="stepPreset active" data-step-value="1">1°</button><button class="stepPreset" data-step-value="5">5°</button><button class="stepPreset" data-step-value="10">10°</button><label class="muted">Custom ° <input id="step" type="number" min="0.1" max="20" step="0.1" value="1"></label></div>
        <div class="axisGrid2x2">
          <div class="axisTile"><span class="axisLabel">Motor X θ</span><button data-axis="x" data-dir="-1">−</button><span class="axisValue"><span id="sx">0.0</span>°</span><button data-axis="x" data-dir="1">+</button></div>
          <div class="axisTile"><span class="axisLabel">Motor Y θ</span><button data-axis="y" data-dir="-1">−</button><span class="axisValue"><span id="sy">0.0</span>°</span><button data-axis="y" data-dir="1">+</button></div>
          <div class="axisTile"><span class="axisLabel">Motor Z θ</span><button data-axis="z" data-dir="-1">−</button><span class="axisValue"><span id="sz">0.0</span>°</span><button data-axis="z" data-dir="1">+</button></div>
          <div class="axisTile" id="linearAxisTile"><span class="axisLabel" id="linearAxisLabel">Rail position</span><button data-linear-dir="-1">−</button><span class="axisValue" id="slinear">0.0 mm</span><button data-linear-dir="1">+</button></div>
        </div>
        <div class="toolbar" style="margin-top:6px">
          <label class="muted">Rail target · mm <input id="linearTarget" type="number" min="0" max="235" step="1" value="0"></label>
          <button id="linearGo">Go slide</button>
        </div>
      </div>

      <div class="moveGrid">
        <div class="group">
          <div class="groupTitle">Move motor angles directly · degrees</div>
          <div class="muted" style="margin-bottom:6px">Use motor X, Y, and Z angles when you know them.</div>
          <div class="fieldrow"><label>Motor X θ ° <input id="targetX" type="number" step="0.5" value="0"></label><label>Motor Y θ ° <input id="targetY" type="number" step="0.5" value="0"></label><label>Motor Z θ ° <input id="targetZ" type="number" step="0.5" value="0"></label></div>
          <div class="toolbar" style="margin-top:6px"><button id="goTarget" class="primary">Move motor angles</button><button id="useCurrent">Use current</button></div>
        </div>

        <div class="group">
          <div class="groupTitle">Move tool · arm-local Y/Z (mm)</div>
          <div class="muted" style="margin-bottom:6px">Y = reach from base · Z = height. The controller solves the joints.</div>
          <div class="fieldrow"><label>Reach Y · mm <input id="cartTargetY" type="number" step="1"></label><label>Height Z · mm <input id="cartTargetZ" type="number" step="1"></label></div>
          <div class="toolbar" style="margin-top:6px"><button id="cartGo" class="primary">Move tool</button><button id="cartUseCurrent">Use current</button></div>
        </div>
      </div>

      <details>
        <summary>Fine / coordinated controls</summary>
        <div class="toolbar" style="margin-top:6px"><button data-fine-axis="x" data-dir="-1">X − fine</button><button data-fine-axis="x" data-dir="1">X + fine</button><button data-fine-axis="y" data-dir="-1">Y − fine</button><button data-fine-axis="y" data-dir="1">Y + fine</button><label>ΔX <input id="dx" value="5"></label><label>ΔY <input id="dy" value="5"></label><button id="xyMove">Move XY</button><button id="xyBack">Reverse</button></div>
      </details>
    </section>

  </div>

  <div id="workspacePanel" class="fullPanel workspaceFull">
    <section class="workspaceSlices">
      <section class="card sliceCard">
        <div class="sliceHead"><b>XY plane</b><input id="sliceZ" type="range" value="0" step="1"><span id="sliceZVal" class="sliceVal">Z = 0</span></div>
        <canvas id="wsXYCanvas" width="720" height="430"></canvas>
      </section>
      <section class="card sliceCard">
        <div class="sliceHead"><b>XZ plane</b><input id="sliceY" type="range" value="0" step="1"><span id="sliceYVal" class="sliceVal">Y = 0</span></div>
        <canvas id="wsXZCanvas" width="720" height="430"></canvas>
      </section>
      <section class="card sliceCard">
        <div class="sliceHead"><b>YZ plane</b><input id="sliceX" type="range" value="0" step="1"><span id="sliceXVal" class="sliceVal">X = 0</span></div>
        <canvas id="wsYZCanvas" width="720" height="430"></canvas>
      </section>
      <section class="card sliceCard">
        <div class="sliceHead"><b>3D workspace</b><span class="workspaceHint">drag rotate · wheel zoom</span><button id="wsFocusView">Focus current + target</button><button id="wsResetView">Full workspace</button></div>
        <canvas id="workspaceCanvas" width="720" height="430"></canvas>
      </section>
    </section>

    <section class="card workspaceInfo">
      <h2>World Cartesian target</h2>
      <div class="muted" id="workspaceRailHint">Click a reachable slice point to plan and move immediately. The rail may be enabled for automatic or manual positioning, or disabled at 0 mm.</div>
      <div class="group" style="margin-top:12px">
        <div class="workspaceStep">World X / Y / Z · mm</div>
        <div class="workspaceTargetGrid">
          <label>X mm<input id="worldX" type="number" step="1" value="0"></label>
          <label>Y mm<input id="worldY" type="number" step="1" value="0"></label>
          <label>Z mm<input id="worldZ" type="number" step="1" value="0"></label>
        </div>
        <div class="toolbar" style="margin-top:8px"><button id="worldCurrent">Use current pose</button><button id="worldPreview">Preview</button></div>
      </div>
      <div class="group">
        <div class="workspaceStep">Jog world position</div>
        <div class="fieldrow"><label>Step mm <select id="cartStep"><option>1</option><option>5</option><option selected>10</option><option>25</option><option>50</option></select></label></div>
        <div class="toolbar" style="margin-top:7px;flex-wrap:wrap">
          <button data-cjog="x,-1">−X</button><button data-cjog="x,1">+X</button>
          <button data-cjog="y,-1">−Y</button><button data-cjog="y,1">+Y</button>
          <button data-cjog="z,-1">−Z</button><button data-cjog="z,1">+Z</button>
        </div>
      </div>
      <div id="worldPlanSummary" class="workspaceResult">Click a slice to validate and move to a reachable target.</div>
      <div class="workspaceActions">
        <button id="worldClearTarget">Reset 0,0,0</button>
        <button id="worldMove" class="primary" disabled>Move to target</button>
      </div>
      <details style="margin-top:10px">
        <summary id="railSummary">Rail control · mm</summary>
        <div class="group" style="margin-top:7px">
          <div class="fieldrow"><select id="railMode"><option value="auto" selected>Auto rail · recommended</option><option value="manual">Manual rail</option></select><label id="manualRailLabel" style="display:none">Rail mm <input id="worldD" type="number" min="0" max="235" step="1"></label></div>
        </div>
      </details>
      <div id="workspaceMeta" class="muted" style="margin-top:12px;line-height:1.45"></div>
    </section>
    <section class="workspaceMeasured">
      <section class="card plotCard">
        <div class="cardTitle">
          <div><h2>Measured arm-local Y/Z workspace</h2><div class="muted">Raw boundary in the arm-local tool plane: reach Y and height Z</div></div>
          <div class="toolbar"><label class="muted">World preview ° <input id="basePreview" type="number" step="5" value="0"></label><span class="badge">ARM-LOCAL</span></div>
        </div>
        <svg id="cartPlot" viewBox="0 0 700 360"></svg>
      </section>
      <section class="card plotCard">
        <div class="cardTitle">
          <div><h2>Measured Motor X/Y angle boundary</h2><div class="muted">Raw motor-angle trials and the fitted safe envelope used by Boundary ON</div></div>
          <div class="toolbar"><span id="testSession" class="badge">Test 1 · 0 pts</span><button id="newBoundaryTest">New Test</button><button id="record">Record Point</button></div>
        </div>
        <svg id="plot" viewBox="0 0 700 520"></svg>
        <div id="rowsWrap"><table><tbody id="rows"></tbody></table></div>
      </section>
    </section>
  </div>
  <div id="programPanel" class="fullPanel programFull" data-coordinate-mode="joint">
    <div class="programPageToolbar"><div><strong>Program coordinate system</strong><div class="muted">This one switch controls waypoint entry, jogging, live values, and the route table.</div></div><div class="programMode" aria-label="Program coordinate system"><button id="programModeJoint" class="active">Motor angles · θ °</button><button id="programModeCartesian">World position · XYZ mm</button></div></div>
    <section class="card programCard programBuilder">
      <h2>Add a waypoint</h2>
      <div class="muted" style="margin-top:4px">Choose either motor angles (θ, degrees) or tool position (world X/Y/Z, mm). They stay linked.</div>
      <div class="programPose" id="programCurrent"><div>Motor X θ<b>—</b></div><div>Motor Y θ<b>—</b></div><div>Motor Z θ<b>—</b></div><div>Rail mm<b>—</b></div></div>
      <section class="programSection"><div class="programCoordinateGrid"><div class="programCoordinateGroup joint"><h3>Motor angles · degrees</h3><div class="programTargets"><label>Motor X θ °<input id="programA" type="number" step="0.1"></label><label>Motor Y θ °<input id="programB" type="number" step="0.1"></label><label>Motor Z θ °<input id="programC" type="number" step="0.1"></label><label>Rail mm<input id="programD" type="number" min="0" max="235" step="1" value="0"></label></div></div><div class="programCoordinateGroup cartesian"><h3>Tool position · whole world millimetres</h3><div class="programTargets"><label>World X mm<input id="programToolX" type="number" step="1"></label><label>World Y mm<input id="programToolY" type="number" step="1"></label><label>World Z mm<input id="programToolZ" type="number" step="1"></label><label>Rail mm<input id="programToolD" type="number" min="0" max="235" step="1" value="0"></label></div><div id="programSolveStatus" class="muted" style="margin-top:4px">World position converts to the matching motor angles only when you add or analyze the waypoint.</div></div></div></section>
      <section class="programSection"><div class="programSectionTitle"><span class="programStep">1</span> Jog, then record</div><div id="programJogJoint" class="programJog active"><div class="fieldrow"><label>Angle step <select id="programStep"><option value="0.25">0.25°</option><option value="1" selected>1°</option><option value="5">5°</option><option value="10">10°</option></select></label><button data-program-axis="a" data-program-dir="-1">−Motor X</button><button data-program-axis="a" data-program-dir="1">+Motor X</button><button data-program-axis="b" data-program-dir="-1">−Motor Y</button><button data-program-axis="b" data-program-dir="1">+Motor Y</button><button data-program-axis="c" data-program-dir="-1">−Motor Z</button><button data-program-axis="c" data-program-dir="1">+Motor Z</button></div></div><div id="programJogCartesian" class="programJog"><div class="fieldrow"><label>Distance step <select id="programCartesianStep"><option value="1">1 mm</option><option value="5" selected>5 mm</option><option value="10">10 mm</option><option value="25">25 mm</option></select></label><button data-program-cart-axis="x" data-program-cart-dir="-1">−World X</button><button data-program-cart-axis="x" data-program-cart-dir="1">+World X</button><button data-program-cart-axis="y" data-program-cart-dir="-1">−World Y</button><button data-program-cart-axis="y" data-program-cart-dir="1">+World Y</button><button data-program-cart-axis="z" data-program-cart-dir="-1">−World Z</button><button data-program-cart-axis="z" data-program-cart-dir="1">+World Z</button></div></div><div class="toolbar" style="margin-top:9px"><button id="programRecord" class="primary">Record current pose</button><button id="programUseCurrent">Use current pose</button></div></section>
      <div class="toolbar"><button id="programAdd" class="primary">Add waypoint</button></div>
      <details class="programDetails"><summary>New waypoint timing defaults</summary><div class="group"><div class="groupTitle">New pose timing</div><div class="programTargets"><label>Speed °/s <span class="muted">when Auto</span><input id="programDefaultSpeed" type="number" min="1" max="220" step="1" value="80"></label><label>Fixed travel s <span class="muted">blank = Auto</span><input id="programDefaultTime" type="number" min="0" max="120" step="0.1" placeholder="Auto"></label><label>Hold after s <span class="muted">pause before next</span><input id="programDefaultWait" type="number" min="0" max="120" step="0.1" value="0"></label></div><div class="toolbar" style="margin-top:8px"><button id="programSaveDefaults">Save defaults</button></div></div></details>
    </section>
    <section class="card programCard">
      <div class="cardTitle" style="padding:0 0 5px"><div><h2>Review and run</h2><div class="muted">Edit the route below. Run checks the current values before the arm moves.</div></div><div class="programLibrary" aria-label="Saved programs"><div class="programLibraryHeader"><span class="libraryLabel">Saved program</span><select id="programSaved"><option value="">Choose a program…</option></select></div><div id="programSavedHint" class="programLibraryHint">Choose a saved program to load, save over, or delete.</div><div class="programLibraryActions"><button id="programLoad" title="Replace this editor with the selected saved route">Load</button><button id="programUpdate" class="primary" title="Save the current editor into the selected program; does not load it">Save</button><button id="programDelete" class="danger">Delete</button><button id="programNew">New program</button></div><div id="programNewRow" class="libraryRow" hidden><span class="libraryLabel">New program</span><input id="programName" maxlength="48" placeholder="Program name"><button id="programSave" class="primary">Create</button><button id="programCancelNew">Cancel</button></div></div></div>
      <div class="programReviewContent"><div class="programPathViews" aria-label="Recorded world-coordinate tool path preview"><canvas id="programPath3d" class="programPath drag" width="520" height="320" aria-label="Interactive world-coordinate 3D tool path; drag to rotate"></canvas></div><div class="programReviewSide"><div id="programStatus" class="workspaceResult">Record one or more locations to build a program.</div><div class="programActions"><div class="programActionHint">Load and save do not move the arm. Run validates the values currently in the table, then moves.</div><div class="toolbar"><label class="muted">Corner flow °/s <input id="programCorner" type="number" min="0" max="50" step="1" value="18"></label><button id="programClear" class="danger">Clear editor</button><button id="programRun" class="primary">Run route</button></div></div></div></div><div class="programTableWrap" style="margin-top:10px"><table class="programTable"><thead id="programTableHead"></thead><tbody id="programRows"></tbody></table></div>
    </section>
  </div>
  <div id="sensorPanel" class="fullPanel debugGrid">
    <section class="card debugCard">
      <div class="cardTitle"><div><h2>Live accelerometer</h2><div class="muted">FRDM-KL02Z · 3 axes · 1 g ≈ gravity at rest</div></div><span id="sensorLinkBadge" class="badge bad">DISCONNECTED</span></div>
      <table class="metricTable"><tbody>
        <tr><td>Device</td><td id="sensorDevice">—</td></tr>
        <tr><td>Firmware</td><td id="sensorFw">—</td></tr>
        <tr><td>Sequence</td><td id="sensorSeq">—</td></tr>
        <tr><td>Sample age</td><td id="sensorAge">—</td></tr>
        <tr><td>Health</td><td id="sensorHealth">—</td></tr>
        <tr><td>Acceleration X</td><td id="sensorAx">—</td></tr>
        <tr><td>Acceleration Y</td><td id="sensorAy">—</td></tr>
        <tr><td>Acceleration Z</td><td id="sensorAz">—</td></tr>
        <tr><td>Magnitude</td><td id="sensorMagnitude">—</td></tr>
        <tr><td>Orientation / movement</td><td id="sensorEvents">—</td></tr>
      </tbody></table>
    </section>
    <section class="card debugCard">
      <div class="cardTitle"><div><h2>Triplate orientation</h2><div class="muted">Calibrate stationary in the normal mounted pose. Monitoring does not move the robot.</div></div><span id="sensorFaultBadge" class="badge">NOT CALIBRATED</span></div>
      <table class="metricTable"><tbody><tr><td>Filtered deviation from calibrated normal</td><td id="sensorTilt">—</td></tr></tbody></table>
      <div class="group" style="margin-top:10px">
        <div class="fieldrow"><label>Allowed tilt ° <input id="sensorTiltLimit" type="number" min="1" max="60" step="1" value="15"></label><label>Persistence s <input id="sensorPersistence" type="number" min="0.05" max="10" step="0.05" value="0.75"></label><label>Filter s <input id="sensorTiltFilter" type="number" min="0.02" max="2" step="0.01" value="0.25"></label></div>
        <div class="toolbar" style="margin-top:8px"><button id="sensorCalibrate" class="primary">Calibrate current pose</button><button id="sensorMonitorApply">Apply monitoring</button><label class="muted"><input id="sensorMonitorEnabled" type="checkbox"> Monitor tilt</label><label class="muted"><input id="sensorCorrectEnabled" type="checkbox"> Correct on confirmed tilt</label><label class="muted"><input id="sensorKillEnabled" type="checkbox"> KILL on confirmed fault</label><button id="sensorAck">Acknowledge latched fault</button></div>
        <div class="muted" style="margin-top:8px">Short motion spikes are filtered. Samples outside 0.75–1.25 g are ignored for tilt.</div>
      </div>
      <div id="sensorFaultReason" class="workspaceResult">No sensor fault.</div>
      <div class="group" style="margin-top:10px"><div class="cardTitle"><div><h3>Lock-up recovery</h3><div class="muted">Choose a recorded counter-clockwise X/Y path for this specific lock-up. During correction, Motor Z and the rail stay fixed. With no route selected, the generic inboard-and-up recovery is used.</div></div></div><div class="toolbar"><select id="sensorRecoveryProgram" aria-label="Lock-up recovery route"><option value="">Use generic recovery</option></select><button id="sensorRecoverySave">Use selected route</button></div><div id="sensorRecoveryStatus" class="muted" style="margin-top:8px">Tilt correction is OFF. Selecting a route does not move the arm.</div></div>
      <details><summary>Impact & Z protection · optional</summary>
        <div class="muted">Sampled software guard, not a safety interlock. KILL requires controller recovery and rehoming. Z uses absolute acceleration in g, not degrees.</div>
        <div class="fieldrow"><label><input id="guardImpactStop" type="checkbox"> Impact → KILL</label><label><input id="guardImpactFlash" type="checkbox"> Impact → flash red (5 Hz)</label><label>Impact Δg <input id="guardImpactG" type="number" min="0.1" max="8" step="0.1" value="1.1"></label></div>
        <div class="fieldrow"><label><input id="guardZStop" type="checkbox"> |Z| → KILL</label><label><input id="guardZColor" type="checkbox"> |Z| → yellow</label><label>|Z| limit g <input id="guardZG" type="number" min="0.05" max="2" step="0.05" value="0.3"></label><label>Hold s <input id="guardZHold" type="number" min="0.05" max="2" step="0.05" value="0.15"></label></div>
        <div class="fieldrow"><label><input id="guardSwipeColors" type="checkbox"> Gesture colors · use mapped colors while disabled</label><button id="guardApply">Apply protection</button></div>
        <div id="guardStatus" class="muted">All protection off.</div>
      </details>
    </section>
    <section class="card debugCard">
      <div class="cardTitle"><div><h2>Touch slider & LED</h2><div class="muted">Slide a finger along the touch strip to test its response.</div></div></div>
      <table class="metricTable"><tbody>
        <tr><td>Slider position</td><td id="sensorSliderPos">—</td></tr>
        <tr><td>Raw electrodes</td><td id="sensorSliderRaw">—</td></tr>
        <tr><td>Activity / swipe</td><td id="sensorSliderActivity">—</td></tr>
        <tr><td>LED reported</td><td id="sensorLedStatus">—</td></tr>
        <tr><td>Feedback mode</td><td id="sensorModeStatus">—</td></tr>
      </tbody></table>
      <div class="toolbar" style="margin-top:10px"><select id="sensorLedColor"><option>off</option><option>red</option><option>green</option><option>blue</option><option>yellow</option><option>cyan</option><option>magenta</option><option>white</option></select><button id="sensorLedSet">Set LED</button></div>
      <div class="fieldrow" style="margin-top:10px"><label>LED follows <select id="sensorMode"><option value="manual">Manual color</option><option value="acceleration">Dominant acceleration axis</option><option value="orientation">Board orientation</option><option value="slider">Touch position</option><option value="tap">Tap / impact</option><option value="motion">Movement</option></select></label><button id="sensorModeSet">Set mode</button></div>
    </section>
    <section class="card debugCard">
      <div class="cardTitle"><div><h2>Touch gestures</h2><div class="muted">Press left / center / right, or swipe. Lift to confirm; runs after 2 seconds. Touch again or Disable to cancel.</div></div><span id="sensorSwipeBadge" class="badge">DISABLED</span></div>
      <div id="sensorTapRows"></div>
      <div class="toolbar" style="margin-top:8px"><button id="sensorTapSave">Save mappings</button><button id="sensorSwipeEnable" class="primary">Enable gestures</button><button id="sensorSwipeDisable">Disable</button></div>
      <div id="sensorSwipeStatus" class="workspaceResult" style="margin-top:8px">Disabled during initial sensor testing.</div>
    </section>
  </div>

  <div id="debugPanel" class="fullPanel debugGrid">
    <section class="card debugCard"><div class="cardTitle"><div><h2>Command latency</h2><div id="debugWindowLabel" class="muted">Rolling 5 min · milliseconds</div></div><div class="toolbar"><select id="debugWindow"><option value="60">1 min</option><option value="180">3 min</option><option value="300" selected>5 min</option><option value="600">10 min</option><option value="1800">30 min</option><option value="3600">1 hour</option></select><select id="debugMode"><option>OFF</option><option selected>BASIC</option><option>TRACE</option></select><button id="debugReset">Reset</button></div></div><table class="metricTable"><thead><tr><th>Channel</th><th>median</th><th>p95</th><th>p99</th><th>max</th><th>n</th></tr></thead><tbody id="latencyRows"></tbody></table><canvas id="latencyGraph" width="800" height="220"></canvas></section>
    <section class="card debugCard"><div class="cardTitle"><div><h2>Motion / MCU</h2><div class="muted">Queue, USB link, controller timing</div></div></div><table class="metricTable"><tbody id="motionRows"></tbody></table><canvas id="queueGraph" width="800" height="220"></canvas></section>
    <section class="card debugCard"><div class="cardTitle"><div><h2>System timing</h2><div class="muted">Wyse load and diagnostics loop jitter</div></div></div><table class="metricTable"><tbody id="systemRows"></tbody></table><canvas id="systemGraph" width="800" height="220"></canvas></section>
    <section class="card debugCard"><div class="cardTitle"><div><h2>Trace / logs</h2><div class="muted">TRACE mode records command-level events</div></div></div><div id="debugEvents">No trace events yet.</div></section>
  </div>

  <div id="settingsPanel" class="fullPanel" style="grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;align-content:start">
    <section class="card">
      <div class="cardTitle"><div><h2>Motor power</h2><div class="muted">Current axes: Motor X θ, Motor Y θ, Motor Z θ</div></div><span id="motorStateBadge" class="badge">UNKNOWN</span></div>
      <div class="group">
        <div class="muted" style="margin-bottom:8px">Disable motors when you want to move the arm by hand. Disabling holding torque invalidates the current home position, so the robot must be homed again before commanded motion.</div>
        <div class="toolbar">
          <button id="motorsEnable" class="primary">Enable motors</button>
          <button id="motorsDisable">Disable motors</button>
        </div>
      </div>
    </section>

    <section class="card">
      <div class="cardTitle"><div><h2>Endstops</h2><div class="muted">Physical switches currently installed</div></div><button id="settingsRefreshEndstops">Refresh</button></div>
      <div class="group">
        <div class="axisGrid2x2">
          <div class="axisTile"><span class="axisLabel">Motor X</span><span></span><span class="axisValue" id="settingsEndstopX">—</span><span></span></div>
          <div class="axisTile"><span class="axisLabel">Motor Y</span><span></span><span class="axisValue" id="settingsEndstopY">—</span><span></span></div>
          <div class="axisTile"><span class="axisLabel">Motor Z</span><span></span><span class="axisValue" id="settingsEndstopZ">—</span><span></span></div>
          <div class="axisTile" id="settingsLinearEndstopTile"><span class="axisLabel">Rail</span><span></span><span class="axisValue" id="settingsEndstopD">DISABLED</span><span></span></div>
        </div>
      </div>
    </section>

    <section class="card">
      <div class="cardTitle"><div><h2>Controller</h2><div class="muted">Klipper / MCU controls</div></div></div>
      <div class="group">
        <div class="toolbar">
          <button id="settingsHome" class="primary">Home robot</button>
          <button id="settingsFirmwareRestart">Firmware restart</button>
        </div>
        <div class="muted" id="homeModeNote" style="margin-top:8px">Home uses one slow touch per active axis. Rail is included only when enabled.</div>
      </div>
    </section>

    <section class="card">
      <div class="cardTitle"><div><h2>Linear rail</h2><div class="muted">Optional fourth motion axis</div></div><span id="linearModeBadge" class="badge">DISABLED</span></div>
      <div class="group">
        <div class="muted" style="margin-bottom:8px">Enable only when the rail motor and rail endstop are physically connected. Changing mode requires homing again.</div>
        <div class="toolbar">
          <button id="linearModeEnable" class="primary">Enable linear rail</button>
          <button id="linearModeDisable">Disable linear rail</button>
        </div>
        <div id="linearModeDetail" class="workspaceResult" style="margin-top:8px">Rail fixed at 0 mm. Motor-angle, world-position, and Dance motion run without the rail.</div>
      </div>
    </section>
  </div>
</main>
<aside id="cameraPanel" hidden aria-label="Live USB camera"><div class="cameraHeader"><strong>Live camera</strong><div><button id="cameraFull">Full screen</button><button id="cameraClose" aria-label="Close camera">Close</button></div></div><img id="cameraFeed" alt="Live USB camera view"><div id="cameraInfo">640 × 480 · live video</div></aside>
<script>
const S={x:0,y:0,z:0,linear_y:0,busy:false,homed:false},DISPLAY={x:0,y:0,z:0,linear_y:0},pts=[],$=id=>document.getElementById(id);
let motionAnim=null;
let selectedTarget=null;
let selectedCartTarget=null;
let danceRunning=false;
let cameraStatusTimer=null;
function showCamera(visible){
  $('cameraPanel').hidden=!visible;
  $('cameraToggle').setAttribute('aria-expanded',String(visible));
  clearInterval(cameraStatusTimer);
  if(visible){
    $('cameraFeed').src='/camera/stream?t='+Date.now();
    const status=async()=>{try{const s=await (await fetch('/camera/status')).json();$('cameraInfo').textContent=s.error||'640 × 480 · live video';}catch(e){$('cameraInfo').textContent='Camera connection interrupted';}};
    status();cameraStatusTimer=setInterval(status,3000);
  }else $('cameraFeed').removeAttribute('src');
}
$('cameraToggle').addEventListener('click',()=>showCamera($('cameraPanel').hidden));
$('cameraClose').addEventListener('click',()=>showCamera(false));
$('cameraFull').addEventListener('click',()=>{if(document.fullscreenElement)document.exitFullscreen();else $('cameraPanel').requestFullscreen();});
{
  const header=$('cameraPanel').querySelector('.cameraHeader'),panel=$('cameraPanel');
  let drag=null;
  header.addEventListener('pointerdown',e=>{
    if(e.target.closest('button')||document.fullscreenElement)return;
    const r=panel.getBoundingClientRect();drag={x:e.clientX,y:e.clientY,left:r.left,top:r.top};
    header.setPointerCapture(e.pointerId);
  });
  header.addEventListener('pointermove',e=>{
    if(!drag)return;
    panel.style.left=Math.max(0,Math.min(innerWidth-panel.offsetWidth,drag.left+e.clientX-drag.x))+'px';
    panel.style.top=Math.max(60,Math.min(innerHeight-50,drag.top+e.clientY-drag.y))+'px';
    panel.style.right='auto';panel.style.bottom='auto';
  });
  header.addEventListener('pointerup',()=>{drag=null;});
  header.addEventListener('pointercancel',()=>{drag=null;});
}

// Browser preview uses the SAME fixed convention as kinematics.py.
// Backend /state is authoritative; these values only let the canvas animate
// smoothly between controller updates.
// Side plane: A=(0,0,0), +Y outward, +Z up, 0 deg = +Z, positive toward +Y.
const KIN={
  deg:Math.PI/180,
  homeReleaseXCounts:3.5,
  homeReleaseYCounts:3.5,
  homeQXDeg:-70.0,
  homeQYDeg:-104.0,
  xDirection:+1.0,
  yDirection:+1.0,
  homeAssemblyBranch:0,
  yawZeroDeg:0.0,
  // Programmed-movement World XYZ keeps its established screen convention.
  // Conversion to the controller convention happens only at the API edge.
  yawSign:-1.0
};

function jointAnglesFromCounts(s){
  const hx=Number.isFinite(S.home_x_counts)?S.home_x_counts:KIN.homeReleaseXCounts;
  const hy=Number.isFinite(S.home_y_counts)?S.home_y_counts:KIN.homeReleaseYCounts;
  return {
    // The actual counts reached by the latest homing cycle are the anchor for
    // the measured geometric home. Moving away from home changes qX/qY.
    qLowerDeg:KIN.homeQXDeg + KIN.xDirection*(s.x-hx),
    qCrankDeg:KIN.homeQYDeg + KIN.yDirection*(s.y-hy),
    yawDeg:KIN.yawZeroDeg + KIN.yawSign*
      (s.z-(Number.isFinite(S.home_z_counts)?S.home_z_counts:0))
  };
}

function circleIntersections2D(a,ra,b,rb){
  const dy=b.y-a.y, dz=b.z-a.z;
  let d=Math.hypot(dy,dz);
  const minDistance=Math.abs(ra-rb), maxDistance=ra+rb, tolerance=1e-9;
  // A home pose can be exactly tangent. Preserve it despite tiny trig
  // roundoff, rather than incorrectly reporting that the linkage cannot close.
  if(d<1e-9 || d>maxDistance+tolerance || d<minDistance-tolerance) return [];
  d=Math.min(Math.max(d,minDistance),maxDistance);

  const x=(ra*ra-rb*rb+d*d)/(2*d);
  const h=Math.sqrt(Math.max(0,ra*ra-x*x));
  const y=a.y+x*dy/d;
  const z=a.z+x*dz/d;

  return [
    {y:y-dz*h/d,z:z+dy*h/d},
    {y:y+dz*h/d,z:z-dy*h/d}
  ];
}

function robotArmFromJointDegrees(qLowerDeg,qCrankDeg,yawDeg=0){
  const qLower=qLowerDeg*KIN.deg;
  const qCrank=qCrankDeg*KIN.deg;

  const A={y:0.0,z:0.0};
  const G={y:-26.933333,z:17.279918};

  const B={y:40*Math.sin(qCrank),z:40*Math.cos(qCrank)};
  const C={y:120*Math.sin(qLower),z:120*Math.cos(qLower)};

  const fChoices=circleIntersections2D(B,120,C,40);
  if(!fChoices.length) return null;

  // Preserve the measured assembly branch. The circle intersection is a
  // genuine two-solution linkage solve; do not force a branch based on the
  // rendered Cartesian Y coordinate.
  const F=fChoices[KIN.homeAssemblyBranch];

  const H={
    y:C.y+3*(C.y-F.y),
    z:C.z+3*(C.z-F.z)
  };

  // A-G-E-C parallelogram.
  const E={y:C.y+G.y,z:C.z+G.z};

  // D-E horizontal, 45 mm.
  const D={y:E.y+45,z:E.z};

  // C-D-I-H parallelogram.
  const I={
    y:H.y+D.y-C.y,
    z:H.z+D.z-C.z
  };

  // Tool is always 27 mm outward from H, level in side plane.
  const T={y:H.y+27,z:H.z};

  const yaw=yawDeg*KIN.deg;
  function world(q){
    return {
      x:q.y*Math.sin(yaw),
      y:q.y*Math.cos(yaw),
      z:q.z
    };
  }

  const side={A,B,C,D,E,F,G,H,I,T};
  const worldPts={};
  for(const k of Object.keys(side)) worldPts[k]=world(side[k]);

  const angle=(p0,p1)=>Math.atan2(p1.y-p0.y,p1.z-p0.z)/KIN.deg;
  return {
    side,
    world:worldPts,
    qLowerDeg,
    qCrankDeg,
    yawDeg,
    memberAngles:{
      AC:angle(A,C),
      AB:angle(A,B),
      BF:angle(B,F),
      FCH:angle(F,H)
    },
    tool:worldPts.T,
    radius:Math.hypot(worldPts.T.x,worldPts.T.y),
    reach:Math.hypot(Math.hypot(worldPts.T.x,worldPts.T.y),worldPts.T.z)
  };
}

function solvedRobotPose(s){
  const a=jointAnglesFromCounts(s);
  return robotArmFromJointDegrees(a.qLowerDeg,a.qCrankDeg,a.yawDeg);
}

function backendRobotPose(k){
  if(!k)return null;
  return {
    world:k.world,
    qLowerDeg:k.qX_deg,
    qCrankDeg:k.qY_deg,
    yawDeg:k.yaw_deg,
    tool:k.tool,
    radius:k.radius_mm,
    reach:k.reach_mm,
    memberAngles:k.member_angles_deg
  };
}

// Default to an orthographic-like Y-Z side view: +Y right, +Z up.
let viewYaw=-90*KIN.deg, viewPitch=0*KIN.deg;
let drag3d=false,last3dX=0,last3dY=0;

function updateKinematics(){
  // Between commands, render the exact Python/kinematics.py solution returned
  // by /state. During a browser motion preview, use the same equations locally
  // only to interpolate smoothly until the controller reports its final state.
  const p=livePoseValid
    ? solvedRobotPose(DISPLAY)
    : (S.kinematics ? backendRobotPose(S.kinematics) : solvedRobotPose(DISPLAY));
  if(!p){
    $('cartX').textContent='—';
    $('cartY').textContent='—';
    $('cartZ').textContent='—';
    $('angUpper').textContent='—';
    $('angLower').textContent='—';
    $('angBF').textContent='—';
    $('angFCH').textContent='—';
    $('angBase').textContent='—';
    drawArm3D(null);
    return;
  }

  $('cartX').textContent=p.tool.x.toFixed(1);
  $('cartY').textContent=p.tool.y.toFixed(1);
  $('cartZ').textContent=p.tool.z.toFixed(1);
  $('angUpper').textContent=p.qCrankDeg.toFixed(1)+'°';
  $('angLower').textContent=p.qLowerDeg.toFixed(1)+'°';
  $('angBF').textContent=p.memberAngles.BF.toFixed(1)+'°';
  $('angFCH').textContent=p.memberAngles.FCH.toFixed(1)+'°';
  $('angBase').textContent=p.yawDeg.toFixed(1)+'°';
  drawArm3D(p);
}

function drawArmView(canvasId,p,yaw,pitch){
  const c=$(canvasId); if(!c)return;
  const slideY=Number(DISPLAY.linear_y)||0;
  const ctx=c.getContext('2d'),W=c.width,H=c.height;
  ctx.clearRect(0,0,W,H);

  // Build the moving robot points first so every view can fit the entire
  // mechanism instead of relying on a fixed scale/origin.
  const q={};
  if(p){
    for(const [name,v] of Object.entries(p.world)){
      q[name]={x:v.x,y:v.y+slideY,z:v.z};
    }
  }

  function rotate(v){
    const cy=Math.cos(yaw),sy=Math.sin(yaw);
    const cp=Math.cos(pitch),sp=Math.sin(pitch);
    const x1=cy*v.x-sy*v.y;
    const y1=sy*v.x+cy*v.y;
    return {x:x1,z:sp*y1+cp*v.z,d:cp*y1-sp*v.z};
  }

  // Fixed projected windows tuned per panel.  They NEVER depend on the
  // current pose, so there is no zooming during motion, but each plane uses
  // its available canvas much better than one oversized 3D bounding box.
  let minX,maxX,minZ,maxZ;
  const padX=28,padY=20;

  if(canvasId==='armXY'){
    // Top view is rotated in-plane so the long 235 mm slide uses canvas width.
    // projected X = world Y, projected Z = -world X
    minX=-180; maxX=500;
    minZ=-235; maxZ=235;
  }else if(canvasId==='armYZ'){
    // Side: full slide travel + full arm-local radial/reach envelope.
    minX=-180; maxX=500;
    minZ=-210; maxZ=180;
  }else if(canvasId==='armXZ'){
    // Front: full yaw sweep across X and complete vertical arm envelope.
    minX=-235; maxX=235;
    minZ=-210; maxZ=180;
  }else{
    // Free view stays fixed-scale while rotating. Center follows only camera
    // orientation, never robot pose.
    const centerWorld={x:0,y:155,z:-15};
    const rc=rotate(centerWorld);
    minX=rc.x-285; maxX=rc.x+285;
    minZ=rc.z-215; maxZ=rc.z+215;
  }

  const sc=Math.min(
    (W-padX*2)/(maxX-minX),
    (H-padY*2)/(maxZ-minZ)
  );
  const centerX=(minX+maxX)/2;
  const centerZ=(minZ+maxZ)/2;

  function proj(v){
    const r=rotate(v);
    return {
      x:W/2+(r.x-centerX)*sc,
      y:H/2-(r.z-centerZ)*sc,
      d:r.d
    };
  }
  function line3(a,b,width,stroke){
    const A=proj(a),B=proj(b);
    ctx.beginPath();ctx.moveTo(A.x,A.y);ctx.lineTo(B.x,B.y);
    ctx.lineWidth=width;ctx.lineCap='round';ctx.strokeStyle=stroke;ctx.stroke();
  }
  function joint(v,r,fill,stroke='#dfe7ef'){
    const z=proj(v);ctx.beginPath();ctx.arc(z.x,z.y,r,0,Math.PI*2);
    ctx.fillStyle=fill;ctx.fill();ctx.lineWidth=2;ctx.strokeStyle=stroke;ctx.stroke();
  }
  function jointLabel(name,v){
    const z=proj(v);
    ctx.font='bold 12px system-ui';
    ctx.textAlign='left';ctx.textBaseline='bottom';
    ctx.lineWidth=3;ctx.strokeStyle='rgba(8,10,13,.95)';
    ctx.strokeText(name,z.x+7,z.y-5);
    ctx.fillStyle='#ffffff';ctx.fillText(name,z.x+7,z.y-5);
  }

  // Lightweight reference grid.
  for(let i=-300;i<=300;i+=50){
    line3({x:i,y:-300,z:0},{x:i,y:300,z:0},1,'rgba(120,140,160,.11)');
    line3({x:-300,y:i,z:0},{x:300,y:i,z:0},1,'rgba(120,140,160,.11)');
  }
  line3({x:0,y:0,z:0},{x:90,y:0,z:0},2,'#e35d6a');
  line3({x:0,y:0,z:0},{x:0,y:90,z:0},2,'#65c98c');
  line3({x:0,y:0,z:0},{x:0,y:0,z:90},2,'#64b5f6');

  // Linear carriage rail and base.
  line3({x:0,y:0,z:-38},{x:0,y:235,z:-38},7,'#39424c');
  line3({x:-18,y:slideY,z:-34},{x:18,y:slideY,z:-34},13,'#66727e');
  const baseTop={x:0,y:slideY,z:0},baseBottom={x:0,y:slideY,z:-28};
  line3(baseBottom,baseTop,14,'#596572');
  joint(baseBottom,11,'#303840','#84909c');
  joint(baseTop,8,'#e7edf3','#dfe7ef');

  if(!p){
    ctx.fillStyle='#ff8f8f';
    ctx.font='bold 15px system-ui';
    ctx.textAlign='center';
    ctx.fillText('LINKAGE CANNOT CLOSE',W/2,H/2);
    return;
  }

  line3(q.A,q.B,7,'#d35f54');
  line3(q.A,q.C,13,'#f0b35a');
  line3(q.B,q.F,7,'#3e86d8');
  line3(q.C,q.F,7,'#d35f54');
  line3(q.C,q.H,13,'#69b7ff');
  line3(q.A,q.G,6,'#7bd36a');
  line3(q.G,q.E,7,'#7bd36a');
  line3(q.C,q.E,6,'#d3a43c');
  line3(q.E,q.D,6,'#d3a43c');
  line3(q.C,q.D,6,'#d3a43c');
  line3(q.D,q.I,7,'#3e86d8');
  line3(q.H,q.I,6,'#d3a43c');
  line3(q.H,q.T,8,'#9be37c');

  const labeledJoints=[
    ['A',q.A,'#e7edf3'],['B',q.B,'#d35f54'],['C',q.C,'#f0b35a'],
    ['D',q.D,'#d3a43c'],['E',q.E,'#d3a43c'],['F',q.F,'#3e86d8'],
    ['G',q.G,'#7bd36a'],['H',q.H,'#69b7ff'],['I',q.I,'#3e86d8'],
    ['T',q.T,'#9be37c']
  ];
  for(const [name,pt,fill] of labeledJoints)joint(pt,name==='T'?7:5,fill);
  for(const [name,pt] of labeledJoints)jointLabel(name,pt);

  line3(q.T,{x:q.T.x,y:q.T.y,z:0},1.2,'rgba(155,227,124,.38)');
  const t=proj(q.T);
  ctx.fillStyle='#e8eef5';ctx.font='11px system-ui';
  ctx.textAlign='left';
  const toolText='Tool '+q.T.x.toFixed(0)+', '+q.T.y.toFixed(0)+', '+q.T.z.toFixed(0)+' mm';
  const tw=ctx.measureText(toolText).width;
  const tx=Math.min(W-tw-7,Math.max(7,t.x+9));
  const ty=Math.max(16,Math.min(H-6,t.y-7));
  ctx.fillText(toolText,tx,ty);

  ctx.fillStyle='rgba(220,228,236,.68)';ctx.font='10px system-ui';
  for(const [label,v] of [
    ['X',{x:105,y:0,z:0}],
    ['Y',{x:0,y:105,z:0}],
    ['Z',{x:0,y:0,z:105}]
  ]){
    const a=proj(v);
    ctx.fillText(label,
      Math.max(5,Math.min(W-12,a.x)),
      Math.max(12,Math.min(H-5,a.y))
    );
  }
}


function drawArm3D(p){
  // Fixed orthographic views + one user-rotatable free view.
  drawArmView('armXY',p,-Math.PI/2,Math.PI/2);
  drawArmView('armYZ',p,-Math.PI/2,0);
  drawArmView('armXZ',p,0,0);
  drawArmView('armFree',p,viewYaw,viewPitch);
}


// First measured boundary set at Z=250. These remain fixed as reference data.
const REF_RAW=[
{x:165,y:65},{x:163,y:137},{x:125,y:137},{x:133,y:40},
{x:73,y:80},{x:53,y:60},{x:53,y:60},{x:29,y:40},
{x:13,y:26},{x:0,y:10},{x:104,y:10},{x:148,y:60},
{x:154,y:60},{x:158,y:62},{x:164,y:66},{x:88,y:94},
{x:110,y:118},{x:148,y:146},{x:166,y:142},{x:188,y:112},
{x:188,y:90},{x:178,y:78},{x:0,y:0}
];

// Additional measured boundary trials supplied after the original run.
// These are persisted so they contribute to the averaged/local fitted boundary
// even after the browser or mapper is restarted.
const REF_REPEAT=[
{x:100,y:0},{x:150,y:50},{x:182,y:80},{x:184,y:138},
{x:22,y:22},{x:66,y:64},{x:116,y:116},{x:146,y:146},
{x:160,y:146},{x:162,y:138},{x:168,y:138},{x:186,y:130},
{x:184,y:110},{x:186,y:98},{x:186,y:78},
{x:102,y:0},{x:164,y:64},{x:184,y:96},
{x:161.5,y:123.9},{x:129.5,y:126.3},{x:148.1,y:130.2},
{x:175.5,y:130.6},{x:169.2,y:78.1}
];

// First-pass ordered envelope inferred from the original measurements.
// This is a reference model, not a hard safety limit; the redo measurements
// are intended to validate/refine it.
const REF_BOUNDARY=[
{x:0,y:0},{x:104,y:10},{x:133,y:40},{x:148,y:60},
{x:154,y:60},{x:158,y:62},{x:164,y:66},{x:165,y:65},
{x:178,y:78},{x:188,y:90},{x:188,y:112},{x:166,y:142},
{x:148,y:146},{x:125,y:137},{x:110,y:118},{x:88,y:94},
{x:73,y:80},{x:53,y:60},{x:29,y:40},{x:13,y:26},
{x:0,y:10},{x:0,y:0}
];

function localEdgeEquation(a,b){
  const dx=b.x-a.x,dy=b.y-a.y;
  if(Math.abs(dx)>=Math.abs(dy) && Math.abs(dx)>1e-9){
    const m=dy/dx, c=a.y-m*a.x;
    return {kind:'yx',m:m,c:c,text:'y = '+m.toFixed(3)+'x '+(c>=0?'+ ':'- ')+Math.abs(c).toFixed(2)};
  }
  if(Math.abs(dy)>1e-9){
    const m=dx/dy, c=a.x-m*a.y;
    return {kind:'xy',m:m,c:c,text:'x = '+m.toFixed(3)+'y '+(c>=0?'+ ':'- ')+Math.abs(c).toFixed(2)};
  }
  return {kind:'point',m:0,c:0,text:'single point'};
}

function averagedReferenceBoundary(){
  // Each original boundary vertex remains the anchor/truth. New repeat points
  // that land near it are averaged with it instead of replacing it.
  const radius=18;
  return REF_BOUNDARY.slice(0,-1).map(r=>{
    const allRepeats=REF_REPEAT.concat(pts);
    const nearby=allRepeats.filter(q=>Math.hypot(q.x-r.x,q.y-r.y)<=radius);
    if(!nearby.length)return {x:r.x,y:r.y,n:1};
    const xs=[r.x,...nearby.map(q=>q.x)];
    const ys=[r.y,...nearby.map(q=>q.y)];
    return {
      x:xs.reduce((a,b)=>a+b,0)/xs.length,
      y:ys.reduce((a,b)=>a+b,0)/ys.length,
      n:xs.length
    };
  });
}

function localEdges(){
  const v=averagedReferenceBoundary();
  const edges=[];
  for(let i=0;i<v.length;i++){
    const a=v[i],b=v[(i+1)%v.length];
    if(Math.hypot(b.x-a.x,b.y-a.y)<1e-9)continue;
    edges.push({a:a,b:b,eq:localEdgeEquation(a,b),index:i+1});
  }
  return edges;
}

const PAGE_TOKEN="__PAGE_TOKEN__";
const UI_RTT=[];async function api(path,body){const t0=performance.now(),isUserCommand=Boolean(body)&&!['/debug_mode','/debug_window','/debug_reset','/plan_world'].includes(path);if(body){body={...body,_token:PAGE_TOKEN}}try{const r=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});const j=await r.json();if(!r.ok)throw Error(j.error||'Request failed');return j}finally{if(isUserCommand){UI_RTT.push([Date.now()/1000,performance.now()-t0,path]);if(UI_RTT.length>2000)UI_RTT.shift()}}}
function motionDurationMs(from,to,feed){
  // Match the exact machine-space vector sent in jog_counts():
  // X count -> 0.50 mm, Y count -> 0.50 mm, Z/base count -> 0.10 mm.
  // Marlin G1 F is path speed in mm/min, so all three displayed axes should
  // reach the commanded endpoint together after path / feed seconds.
  const dx=(to.x-from.x)*0.50;
  const dy=(to.y-from.y)*0.50;
  const dz=(to.z-from.z)*0.10;
  const pathMm=Math.hypot(dx,dy,dz);
  const feedMmPerMin=Math.max(0.1,Number(feed)||2500);
  return pathMm/feedMmPerMin*60000.0;
}
function setMotionBadge(active){
  const badge=$('motionBadge');
  badge.textContent=active?'MOVING':'IDLE';
  badge.className='badge '+(active?'warn':'good');
}
function renderDisplayState(updatePlots=true){
  $('sx').textContent=DISPLAY.x.toFixed(1);
  $('sy').textContent=DISPLAY.y.toFixed(1);
  $('sz').textContent=DISPLAY.z.toFixed(1);
  if($('slinear')) $('slinear').textContent=(Number(DISPLAY.linear_y)||0).toFixed(1)+' mm';
  if($('linearTarget') && document.activeElement!==$('linearTarget')) $('linearTarget').value=(Number(S.linear_y)||0).toFixed(1);
  const syncField=(id,key,digits=1)=>{const e=$(id);if(e&&document.activeElement!==e&&Number.isFinite(Number(S[key])))e.value=Number(S[key]).toFixed(digits)};
  syncField('armSpeed','arm_speed',1); syncField('armAccel','arm_accel',0); syncField('armJerk','arm_jerk',1);
  syncField('linearSpeed','linear_speed',1); syncField('linearAccel','linear_accel',0); syncField('linearJerk','linear_jerk',1);
  // Live canvas + readouts are cheap enough for every animation frame.
  updateKinematics();
  // SVG analysis plots are intentionally NOT rebuilt every frame. Safari
  // otherwise falls behind the physical robot during queued dance motion.
  if(updatePlots){
    draw();
    drawCartesianWorkspace();
  }
}
function stopMotionAnimation(){
  if(motionAnim && motionAnim.raf) cancelAnimationFrame(motionAnim.raf);
  motionAnim=null;
  setMotionBadge(false);
  draw();
  drawCartesianWorkspace();
}
function animateCommandedPose(target,feed){
  stopMotionAnimation();
  const start={x:DISPLAY.x,y:DISPLAY.y,z:DISPLAY.z};
  const end={
    x:Number.isFinite(target.x)?target.x:start.x,
    y:Number.isFinite(target.y)?target.y:start.y,
    z:Number.isFinite(target.z)?target.z:start.z
  };
  const duration=motionDurationMs(start,end,feed);
  const t0=performance.now();
  motionAnim={start,end,duration,t0,raf:0,settled:false};
  setMotionBadge(true);
  const frame=now=>{
    if(!motionAnim)return;
    const u=Math.max(0,Math.min(1,(now-t0)/duration));
    DISPLAY.x=start.x+(end.x-start.x)*u;
    DISPLAY.y=start.y+(end.y-start.y)*u;
    DISPLAY.z=start.z+(end.z-start.z)*u;
    renderDisplayState(false);
    if(u<1) motionAnim.raf=requestAnimationFrame(frame);
    // Keep the final preview in place until the controller response arrives.
    // Otherwise a refresh can snap the 3D pose back to its old count state.
    else { motionAnim.raf=0; motionAnim.settled=true; }
  };
  motionAnim.raf=requestAnimationFrame(frame);
}

function smoothDanceDisplayWaypoints(homeZ,cycles=10,armSteps=1,baseSteps=2){
  const neutral={x:100,y:80};
  const arm=[
    neutral,
    {x:29.8,y:5.1},
    {x:96.0,y:29.7},
    {x:135.2,y:96.5},
    neutral
  ];
  const sweep=[0,30,60,90,120,150,180,150,120,90,60,30,0];
  const out=[];
  const ease=(a,b,u)=>{
    const s=0.5-0.5*Math.cos(Math.PI*u);
    return a+(b-a)*s;
  };
  const addArmLoop=baseDeg=>{
    const n=Math.max(1,Math.floor(armSteps));
    for(let k=0;k<arm.length-1;k++){
      const a=arm[k],b=arm[k+1];
      for(let j=1;j<=n;j++){
        const u=j/n;
        out.push({x:ease(a.x,b.x,u),y:ease(a.y,b.y,u),z:homeZ+baseDeg});
      }
    }
  };
  for(let cycle=0;cycle<cycles;cycle++){
    out.push({x:neutral.x,y:neutral.y,z:homeZ});
    for(let i=0;i<sweep.length;i++){
      const baseDeg=sweep[i];
      addArmLoop(baseDeg);
      if(i+1<sweep.length){
        const next=sweep[i+1];
        const n=Math.max(2,Math.floor(baseSteps));
        for(let j=1;j<=n;j++){
          const u=j/n;
          const pulse=Math.sin(Math.PI*u)**2;
          out.push({
            x:neutral.x-20*pulse,
            y:neutral.y-15*pulse,
            z:homeZ+ease(baseDeg,next,u)
          });
        }
      }
    }
  }
  return out;
}

function animateWaypointSequence(waypoints,feed){
  stopMotionAnimation();
  const points=[{x:DISPLAY.x,y:DISPLAY.y,z:DISPLAY.z},...waypoints];
  const durations=[];
  const cumulative=[0];
  for(let i=1;i<points.length;i++){
    const d=Math.max(1,motionDurationMs(points[i-1],points[i],feed));
    durations.push(d);
    cumulative.push(cumulative[cumulative.length-1]+d);
  }
  const totalMs=cumulative[cumulative.length-1];
  const t0=performance.now();
  motionAnim={raf:0,settled:false,totalMs};
  setMotionBadge(true);
  const frame=now=>{
    if(!motionAnim)return;
    const elapsed=Math.min(totalMs,Math.max(0,now-t0));
    let lo=0,hi=durations.length-1,seg=0;
    while(lo<=hi){
      const mid=(lo+hi)>>1;
      if(cumulative[mid+1] < elapsed) lo=mid+1;
      else {seg=mid;hi=mid-1;}
    }
    const a=points[seg],b=points[seg+1]||points[points.length-1];
    const startMs=cumulative[seg],dur=durations[seg]||1;
    const u=Math.max(0,Math.min(1,(elapsed-startMs)/dur));
    DISPLAY.x=a.x+(b.x-a.x)*u;
    DISPLAY.y=a.y+(b.y-a.y)*u;
    DISPLAY.z=a.z+(b.z-a.z)*u;
    renderDisplayState(false);
    if(elapsed<totalMs) motionAnim.raf=requestAnimationFrame(frame);
    else {motionAnim.raf=0;motionAnim.settled=true;}
  };
  motionAnim.raf=requestAnimationFrame(frame);
}

function updateLinearHardwareUI(){
  const enabled=Boolean(S.linear_hardware);
  const tile=$('linearAxisTile');
  if(tile)tile.style.opacity=enabled?'1':'.45';
  if($('linearAxisLabel'))$('linearAxisLabel').textContent=enabled?'Rail position':'Rail · disabled';
  document.querySelectorAll('[data-linear-dir]').forEach(b=>b.disabled=!enabled);
  if($('linearTarget'))$('linearTarget').disabled=!enabled;
  for(const id of ['programD','programToolD'])if($(id)){$(id).disabled=!enabled;if(!enabled)$(id).value='0';}
  if($('linearGo')){$('linearGo').disabled=!enabled;$('linearGo').textContent=enabled?'Go slide':'Rail disabled'}
  for(const id of ['linearSpeed','linearAccel','linearJerk'])if($(id))$(id).disabled=!enabled;
  if($('railMode'))$('railMode').disabled=!enabled;
  if($('railSummary'))$('railSummary').textContent=enabled?'Rail control · mm':'Rail · disabled (0 mm)';
  if($('workspaceRailHint'))$('workspaceRailHint').textContent=enabled
    ?'Rail is enabled. Use automatic rail selection or choose a manual rail position.'
    :'Rail is disabled. World Cartesian planning and motion use rail = 0 mm.';
  if($('linearModeBadge')){
    $('linearModeBadge').textContent=enabled?'ENABLED':'DISABLED';
    $('linearModeBadge').className='badge '+(enabled?'good':'bad');
  }
  if($('linearModeDetail'))$('linearModeDetail').textContent=enabled
    ?'Rail participates in homing, direct rail moves, world Cartesian auto-planning, and Dance.'
    :'Rail fixed at 0 mm. Motor-angle, world-position, and Dance motion run without the rail.';
  if($('settingsLinearEndstopTile'))$('settingsLinearEndstopTile').style.opacity=enabled?'1':'.45';
  if($('homeModeNote'))$('homeModeNote').textContent=enabled
    ?'Home: Motor X/Y touch → release → Motor Z touch → release → rail touch → release. No coarse/fine second pass.'
    :'Home: Motor X/Y touch → release → Motor Z touch → release. Rail is skipped. No coarse/fine second pass.';
  if(!enabled && $('manualRailLabel'))$('manualRailLabel').style.display='none';
}

let programAnalyzeInFlight=false,programHasDrafts=false,programCoordinateMode='joint',programTableMode='joint',programSolveTimer=null;
function programJointValues(){return {a:Number($('programA').value),b:Number($('programB').value),c:Number($('programC').value),d:Number($('programD').value)||0};}
function programToolFromJoint(){
  const q=programJointValues();
  if(![q.a,q.b,q.c].every(Number.isFinite))return null;
  const angles=jointAnglesFromCounts({x:q.a,y:q.b,z:q.c});
  const pose=robotArmFromJointDegrees(angles.qLowerDeg,angles.qCrankDeg,angles.yawDeg);
  if(!pose)return null;
  return {x:pose.tool.x,y:pose.tool.y+q.d,z:pose.tool.z,d:q.d};
}
function programToolFromPoint(q){
  const angles=jointAnglesFromCounts({x:Number(q.a),y:Number(q.b),z:Number(q.c)}),pose=robotArmFromJointDegrees(angles.qLowerDeg,angles.qCrankDeg,angles.yawDeg);
  return pose?{x:pose.tool.x,y:pose.tool.y+(Number(q.d)||0),z:pose.tool.z,d:Number(q.d)||0}:null;
}
function setProgramToolFromJoint(){
  const p=programToolFromJoint();if(!p)return;
  $('programToolX').value=p.x.toFixed(0);$('programToolY').value=p.y.toFixed(0);$('programToolZ').value=p.z.toFixed(0);$('programToolD').value=p.d.toFixed(0);
  $('programSolveStatus').textContent='Tool position calculated from the motor-angle solution.';
}
function setProgramCoordinateMode(mode){
  programCoordinateMode=mode;
  programTableMode=mode;
  $('programPanel').dataset.coordinateMode=mode;
  $('programModeJoint').classList.toggle('active',mode==='joint');$('programModeCartesian').classList.toggle('active',mode==='cartesian');
  $('programJogJoint').classList.toggle('active',mode==='joint');$('programJogCartesian').classList.toggle('active',mode==='cartesian');
  programHasDrafts=false;renderProgram(S.program);
}
async function solveProgramToolToJoint(){
  const x=Number($('programToolX').value),y=Number($('programToolY').value),z=Number($('programToolZ').value),d=Number($('programToolD').value)||0;
  if(![x,y,z].every(Number.isFinite)){ $('programSolveStatus').textContent='Enter World X, Y, and Z to calculate motor angles.';return false; }
  $('programSolveStatus').textContent='Calculating reachable motor-angle solution…';
  try{
    const plan=await api('/plan_world',{x:-x,y,z,linear_y:d});
    $('programA').value=Number(plan.A).toFixed(2);$('programB').value=Number(plan.B).toFixed(2);$('programC').value=Number(plan.C).toFixed(2);$('programD').value=Number(plan.D||0).toFixed(2);$('programToolD').value=Number(plan.D||0).toFixed(2);
    $('programSolveStatus').textContent='Reachable motor-angle solution calculated.';
    return true;
  }catch(e){$('programSolveStatus').textContent='No reachable motor-angle solution: '+e.message;return false;}
}
function scheduleProgramToolSolve(){clearTimeout(programSolveTimer);programSolveTimer=setTimeout(solveProgramToolToJoint,180);}
function programWorldFromState(){
  // Keep the Program tab's established screen-facing World XYZ convention.
  const p=solvedRobotPose(S);
  return p?{x:p.tool.x,y:p.tool.y+(Number(S.linear_y)||0),z:p.tool.z,d:Number(S.linear_y)||0}:null;
}
let programPathYaw=-0.78,programPathPitch=0.38,programPathZoom=1,programPathDragging=false,programPathLastX=0,programPathLastY=0;
function renderProgramPath(points){
  const path=[];
  for(const q of points||[]){
    const angles=jointAnglesFromCounts({x:Number(q.a),y:Number(q.b),z:Number(q.c)}),pose=robotArmFromJointDegrees(angles.qLowerDeg,angles.qCrankDeg,angles.yawDeg);
    if(pose)path.push({x:pose.tool.x,y:pose.tool.y+(Number(q.d)||0),z:pose.tool.z});
  }
  const cur=programWorldFromState();if(cur)path.unshift(cur);
  const canvas=$('programPath3d');if(!canvas)return;const ctx=clearC(canvas);
  ctx.fillStyle='#c6d1db';ctx.font='11px system-ui';ctx.fillText('3D world path · drag rotate · wheel zoom · double-click fit',10,17);
  if(!path.length){ctx.fillStyle='#7f8b98';ctx.font='10px system-ui';ctx.fillText('Add waypoints to preview the route.',10,37);return;}
  const ca=Math.cos(programPathYaw),sa=Math.sin(programPathYaw),cp=Math.cos(programPathPitch),sp=Math.sin(programPathPitch);
  const project=p=>{const xr=p.x*ca-p.y*sa,yr=p.x*sa+p.y*ca;return {u:xr,v:p.z*cp-yr*sp};};
  const view=path.map(project),us=view.map(p=>p.u),vs=view.map(p=>p.v),minU=Math.min(...us),maxU=Math.max(...us),minV=Math.min(...vs),maxV=Math.max(...vs),spanU=Math.max(maxU-minU,10),spanV=Math.max(maxV-minV,10),scale=Math.min((canvas.width-48)/spanU,(canvas.height-54)/spanV)*.94*programPathZoom,centerU=(minU+maxU)/2,centerV=(minV+maxV)/2;
  const P=p=>{const q=project(p);return [canvas.width*.5+(q.u-centerU)*scale,canvas.height*.54-(q.v-centerV)*scale];};
  const origin=[42,canvas.height-34],basis=(x,y,z)=>{const q=project({x,y,z}),n=Math.max(Math.hypot(q.u,q.v),.001);return [origin[0]+q.u/n*24,origin[1]-q.v/n*24];},ex=basis(1,0,0),ey=basis(0,1,0),ez=basis(0,0,1);
  line2(ctx,origin[0],origin[1],ex[0],ex[1],'#9a5555',1.4);line2(ctx,origin[0],origin[1],ey[0],ey[1],'#54976a',1.4);line2(ctx,origin[0],origin[1],ez[0],ez[1],'#538dbc',1.4);ctx.fillStyle='#9aa6b2';ctx.font='10px system-ui';ctx.fillText('X',ex[0]+4,ex[1]);ctx.fillText('Y',ey[0]+4,ey[1]);ctx.fillText('Z',ez[0]+4,ez[1]);
  ctx.beginPath();path.forEach((p,i)=>{const q=P(p);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);});ctx.strokeStyle='#4db1ea';ctx.lineWidth=2.5;ctx.stroke();
  path.forEach((p,i)=>{const q=P(p);ctx.beginPath();ctx.arc(q[0],q[1],i===0&&cur?5:4,0,Math.PI*2);ctx.fillStyle=i===0&&cur?'#eef3f7':'#f0ba55';ctx.fill();if(i>0){ctx.fillStyle='#e8edf2';ctx.font='10px system-ui';ctx.fillText(String(i),q[0]+6,q[1]-5);}});
}
function markProgramDraft(){
  // Keep the DOM inputs untouched while the user types.  The route becomes
  // authoritative only when it is safely validated immediately before Run.
  programHasDrafts=true;
  const status=$('programStatus');
  if(status)status.textContent=programTableMode==='cartesian'
    ?'Edits ready — Run will validate this whole-mm World XYZ route before moving.'
    :'Edits ready — Run will validate these motor-angle values before moving.';
}
function renderProgramSavedTarget(name,running=false){
  const update=$('programUpdate'),remove=$('programDelete'),hint=$('programSavedHint');
  if(update){
    update.disabled=!name||Boolean(running);
    update.textContent='Save';
    update.title=name
      ?'Save the current editor into “'+name+'”; does not load it'
      :'Choose a saved program first';
  }
  if(remove){
    remove.disabled=!name||Boolean(running);
    remove.textContent='Delete';
  }
  if(hint)hint.textContent=name
    ?'Load replaces editor · Save replaces '+name+'.'
    :'Choose a saved program to load, save over, or delete.';
}
function renderProgram(program){
  const current=$('programCurrent');
  if(current){
    const d=Number(S.linear_y)||0;
    const world=programWorldFromState();
    current.innerHTML=programCoordinateMode==='cartesian'&&world
      ?'<div>World X<b>'+fmt(world.x,1)+' mm</b></div><div>World Y<b>'+fmt(world.y,1)+' mm</b></div><div>World Z<b>'+fmt(world.z,1)+' mm</b></div><div>Rail<b>'+fmt(d,1)+' mm</b></div>'
      :'<div>Motor X θ<b>'+fmt(Number(S.x)||0,2)+'°</b></div><div>Motor Y θ<b>'+fmt(Number(S.y)||0,2)+'°</b></div><div>Motor Z θ<b>'+fmt(Number(S.z)||0,2)+'°</b></div><div>Rail<b>'+fmt(d,2)+' mm</b></div>';
  }
  // Start a new waypoint from the live pose, but never overwrite a value the
  // user is already entering.
  if($('programA') && !document.activeElement.matches('#programA,#programB,#programC,#programD,#programToolX,#programToolY,#programToolZ,#programToolD') && $('programA').value===''){
    $('programA').value=(Number(S.x)||0).toFixed(2);$('programB').value=(Number(S.y)||0).toFixed(2);$('programC').value=(Number(S.z)||0).toFixed(2);$('programD').value=(Number(S.linear_y)||0).toFixed(2);setProgramToolFromJoint();
  }
  const p=program||{points:[],defaults:{},saved:[],running:false},points=Array.isArray(p.points)?p.points:[],defaults=p.defaults||{},saved=Array.isArray(p.saved)?p.saved:[];
  const name=$('programName'),savedSelect=$('programSaved');
  // The save-as name is deliberately never filled from a loaded route. This
  // avoids an accidental overwrite when someone only meant to load and edit.
  if(savedSelect){
    const selected=savedSelect.value||p.active_name||'';
    savedSelect.innerHTML='<option value="">Choose a program…</option>'+saved.map(n=>'<option value="'+n+'"'+(n===selected?' selected':'')+'>'+n+'</option>').join('');
    renderProgramSavedTarget(selected,p.running);
  }
  [['programDefaultSpeed','speed',80],['programDefaultTime','time',0],['programDefaultWait','wait',0]].forEach(([id,key,fallback])=>{
    const input=$(id);if(input&&document.activeElement!==input)input.value=(key==='time'&&Number(defaults[key]??fallback)===0)?'':Number(defaults[key]??fallback).toFixed(key==='speed'?0:1);
  });
  const corner=$('programCorner');if(corner&&document.activeElement!==corner)corner.value=Number(S.arm_jerk??18).toFixed(0);
  const head=$('programTableHead');
  if(head)head.innerHTML=programTableMode==='joint'
    ?'<tr><th>Order</th><th>Motor X θ °</th><th>Motor Y θ °</th><th>Motor Z θ °</th><th>Rail mm</th><th>Speed °/s<br><span class="muted">when Auto</span></th><th>Fixed travel s<br><span class="muted">overrides speed</span></th><th>Hold after s</th><th>Actions</th></tr>'
    :'<tr><th>Order</th><th>World X mm</th><th>World Y mm</th><th>World Z mm</th><th>Rail mm</th><th>Speed °/s<br><span class="muted">when Auto</span></th><th>Fixed travel s<br><span class="muted">overrides speed</span></th><th>Hold after s</th><th>Actions</th></tr>';
  const rows=$('programRows');
  // State refreshes continue while the robot is idle. Do not replace the
  // table (and the user's partially typed value) during an edit.
  const editingWaypoint=programAnalyzeInFlight||programHasDrafts||(rows&&rows.contains(document.activeElement)&&document.activeElement.matches('input'));
  if(rows&&!editingWaypoint){
    rows.innerHTML=points.length?points.map((q,index)=>{const tool=programToolFromPoint(q)||{};const cartHint=programTableMode==='cartesian'&&Number.isFinite(Number(tool.x))
      ?' data-program-hint-a="'+Number(q.a)+'" data-program-hint-b="'+Number(q.b)+'" data-program-hint-c="'+Number(q.c)+'" data-program-hint-d="'+Number(q.d)+'" data-program-hint-x="'+Number(tool.x)+'" data-program-hint-y="'+Number(tool.y)+'" data-program-hint-z="'+Number(tool.z)+'"'
      :'';const coords=programTableMode==='joint'
      ?'<td><input data-program-a="'+index+'" type="number" step="0.1" value="'+fmt(Number(q.a),2)+'"></td><td><input data-program-b="'+index+'" type="number" step="0.1" value="'+fmt(Number(q.b),2)+'"></td><td><input data-program-c="'+index+'" type="number" step="0.1" value="'+fmt(Number(q.c),2)+'"></td><td><input data-program-d="'+index+'" type="number" min="0" max="235" step="0.1" value="'+fmt(Number(q.d),2)+'"></td>'
      :'<td><input data-program-x="'+index+'" type="number" step="1" value="'+fmt(Number(tool.x),0)+'"></td><td><input data-program-y="'+index+'" type="number" step="1" value="'+fmt(Number(tool.y),0)+'"></td><td><input data-program-z="'+index+'" type="number" step="1" value="'+fmt(Number(tool.z),0)+'"></td><td><input data-program-d="'+index+'" type="number" min="0" max="235" step="1" value="'+fmt(Number(q.d),0)+'"></td>';
      return '<tr data-program-row="'+index+'"'+cartHint+'><td class="programOrder"><span class="programDrag" draggable="true" title="Drag to reorder" aria-label="Drag waypoint '+String(index+1)+' to reorder">⠿</span><b>'+String(index+1)+'</b></td>'+coords+'<td><input data-program-speed="'+index+'" type="number" min="1" max="220" step="1" value="'+fmt(Number(q.speed),1)+'"></td><td><input data-program-time="'+index+'" type="number" min="0" max="120" step="0.1" placeholder="Auto" value="'+(Number(q.time)>0?fmt(Number(q.time),1):'')+'"></td><td><input data-program-wait="'+index+'" type="number" min="0" max="120" step="0.1" value="'+fmt(Number(q.wait),1)+'"></td><td class="programRowActions"><button data-program-up="'+index+'" title="Move waypoint up"'+(index===0?' disabled':'')+'>↑</button><button data-program-down="'+index+'" title="Move waypoint down"'+(index===points.length-1?' disabled':'')+'>↓</button><button data-program-remove="'+index+'">Remove</button></td></tr>';
    }).join(''):'<tr><td colspan="9" class="muted">No recorded locations yet.</td></tr>';
    rows.querySelectorAll('input').forEach(input=>{
      input.addEventListener('keydown',event=>{if(event.key==='Enter'){event.preventDefault();input.blur();}});
      input.addEventListener('input',markProgramDraft);
    });
    rows.querySelectorAll('[data-program-remove]').forEach(button=>button.addEventListener('click',async()=>{
      if(programHasDrafts){alert('Analyze the edited path first, or reload it before removing a point.');return;}
      try{rs(await api('/program_remove',{index:Number(button.dataset.programRemove)}))}catch(e){alert(e.message)}
    }));
    rows.querySelectorAll('[data-program-up],[data-program-down]').forEach(button=>button.addEventListener('click',async()=>{
      if(programHasDrafts){alert('Analyze the edited path first, or reload it before reordering points.');return;}
      const index=Number(button.dataset.programUp??button.dataset.programDown),destination=button.dataset.programUp!==undefined?index-1:index+1;
      try{rs(await api('/program_move',{index,destination}))}catch(e){alert(e.message)}
    }));
    let dragIndex=null;
    rows.querySelectorAll('.programDrag').forEach(handle=>{
      handle.addEventListener('dragstart',event=>{dragIndex=Number(handle.closest('tr').dataset.programRow);event.dataTransfer.effectAllowed='move';event.dataTransfer.setData('text/plain',String(dragIndex));handle.closest('tr').classList.add('programDragging');});
      handle.addEventListener('dragend',()=>{dragIndex=null;rows.querySelectorAll('.programDropTarget,.programDragging').forEach(row=>row.classList.remove('programDropTarget','programDragging'));});
    });
    rows.querySelectorAll('tr[data-program-row]').forEach(row=>{
      row.addEventListener('dragover',event=>{if(dragIndex===null)return;event.preventDefault();event.dataTransfer.dropEffect='move';row.classList.add('programDropTarget');});
      row.addEventListener('dragleave',()=>row.classList.remove('programDropTarget'));
      row.addEventListener('drop',async event=>{event.preventDefault();row.classList.remove('programDropTarget');if(dragIndex===null||programHasDrafts)return;const target=Number(row.dataset.programRow);if(target===dragIndex)return;const destination=dragIndex<target?target-1:target;try{rs(await api('/program_move',{index:dragIndex,destination}))}catch(e){alert(e.message)}finally{dragIndex=null;}});
    });
  }
  const status=$('programStatus');
  if(status)status.textContent=p.running?'Program is running — STOP finishes buffered motion.':(programHasDrafts?'Edits ready — Run will validate these current values before moving.':(points.length?points.length+' waypoint'+(points.length===1?'':'s')+' ready to run.':'Record one or more locations to build a program.'));
  if($('programRun')){ $('programRun').disabled=!S.homed||Boolean(S.home_required)||!points.length||Boolean(S.busy); $('programRun').textContent='Run route'; }
  if($('programAnalyze'))$('programAnalyze').disabled=!points.length||Boolean(p.running)||programAnalyzeInFlight;
  if($('programClear'))$('programClear').disabled=!points.length||Boolean(p.running);
  renderProgramPath(programHasDrafts&&programTableMode==='joint'?programDraftPoints():points);
}

function renderSensor(sensor){
  if(!sensor||!sensor.enabled)return;
  const fresh=Boolean(sensor.fresh),connected=Boolean(sensor.connected),frame=fresh?sensor.frame:null;
  const guard=sensor.guard||{};
  $('guardStatus').textContent=(guard.fault?'LATCHED: '+guard.fault:'Impact Δg '+(guard.delta_g==null?'—':Number(guard.delta_g).toFixed(2)))+' · impact KILL '+(guard.impact_stop?'ON':'OFF')+' · Z KILL '+(guard.z_stop?'ON':'OFF');
  if(!$('guardApply').dataset.dirty){
    for(const [id,key] of [['guardImpactStop','impact_stop'],['guardImpactFlash','impact_flash'],['guardZStop','z_stop'],['guardZColor','z_color'],['guardSwipeColors','swipe_colors']])$(id).checked=Boolean(guard[key]);
    for(const [id,key] of [['guardImpactG','impact_g'],['guardZG','z_g'],['guardZHold','z_persistence_s']])if(document.activeElement!==$(id))$(id).value=guard[key]??$(id).value;
  }
  const badge=$('sensorLinkBadge');
  if(badge){
    badge.textContent=fresh?'LIVE':(connected?'STALE':'DISCONNECTED');
    badge.className='badge '+(fresh?'good':'bad');
  }
  $('sensorAge').textContent=sensor.sample_age_s==null?'—':(Number(sensor.sample_age_s)*1000).toFixed(0)+' ms';
  $('sensorDevice').textContent=frame?.device||'—';
  $('sensorFw').textContent=frame?.fw||'—';
  $('sensorSeq').textContent=frame?.seq??'—';
  const health=frame?.health||{};
  $('sensorHealth').textContent=fresh?('accel '+(health.accel?'OK':'BAD')+' · slider '+(health.slider?'OK':'BAD')):'NO LIVE SAMPLE';
  const a=fresh&&Array.isArray(sensor.accel_g)?sensor.accel_g:null;
  $('sensorAx').textContent=a?Number(a[0]).toFixed(4)+' g':'—';
  $('sensorAy').textContent=a?Number(a[1]).toFixed(4)+' g':'—';
  $('sensorAz').textContent=a?Number(a[2]).toFixed(4)+' g':'—';
  $('sensorMagnitude').textContent=a?Math.hypot(...a).toFixed(3)+' g':'—';
  $('sensorEvents').textContent=frame?.events?frame.events.orientation+' · '+frame.events.motion+' · tap '+frame.events.tap:'—';
  $('sensorModeStatus').textContent=frame?.mode||'—';

  const mon=sensor.monitor||{},tilt=fresh&&mon.tilt_deg!=null?Number(mon.tilt_deg):null;
  $('sensorTilt').textContent=tilt==null?'—':tilt.toFixed(2)+'°';
  if(document.activeElement!==$('sensorTiltLimit'))$('sensorTiltLimit').value=Number(mon.allowed_deg??15).toFixed(1);
  if(document.activeElement!==$('sensorPersistence'))$('sensorPersistence').value=Number(mon.persistence_s??.75).toFixed(2);
  if(document.activeElement!==$('sensorTiltFilter'))$('sensorTiltFilter').value=Number(mon.filter_s??.25).toFixed(2);
  $('sensorMonitorEnabled').checked=Boolean(mon.enabled);
  $('sensorKillEnabled').checked=Boolean(mon.kill_enabled);
  $('sensorCorrectEnabled').checked=Boolean(sensor.correct_enabled);
  const fault=$('sensorFaultBadge');
  if(mon.fault_latched){fault.textContent='FAULT LATCHED';fault.className='badge bad'}
  else if(!mon.calibrated){fault.textContent='NOT CALIBRATED';fault.className='badge'}
  else if(mon.enabled){fault.textContent='MONITORING';fault.className='badge good'}
  else{fault.textContent='CALIBRATED';fault.className='badge good'}
  $('sensorFaultReason').textContent=mon.fault_reason||'No sensor fault.';
  const recoverySelect=$('sensorRecoveryProgram'), recoveryName=sensor.recovery_program||'';
  if(recoverySelect && document.activeElement!==recoverySelect){
    const saved=Array.isArray(S.program?.saved)?S.program.saved:[];
    recoverySelect.innerHTML='<option value="">Use generic recovery</option>'+saved.map(name=>'<option value="'+name+'"'+(name===recoveryName?' selected':'')+'>'+name+'</option>').join('');
  }
  if($('sensorRecoveryStatus'))$('sensorRecoveryStatus').textContent=Boolean(sensor.correct_enabled)
    ?(recoveryName?'Correction ON: '+recoveryName+' runs X/Y only at the current Motor Z and rail. Home is required only if recovery does not finish.'
      :'Correction ON: confirmed tilt runs the generic Boundary ON recovery and flashes red. Home is required only if recovery does not finish.')
    :(recoveryName?'Selected route: '+recoveryName+'. It will run only when correction is enabled and tilt is confirmed.'
      :'Tilt correction is OFF. A saved route selection does not move the arm.');

  const slider=frame?.slider;
  $('sensorSliderPos').textContent=slider?((Number(slider.position)/10).toFixed(1)+'%'):'—';
  $('sensorSliderRaw').textContent=slider?(String(slider.raw_left)+' / '+String(slider.raw_right)):'—';
  $('sensorSliderActivity').textContent=slider?((slider.active?'TOUCH':'idle')+' · '+(sensor.gestures?.[sensor.last_gesture]?.label||'no gesture yet')):'—';
  $('sensorLedStatus').textContent=frame?.led||'—';

  const swipe=sensor.swipe||{},swipeBadge=$('sensorSwipeBadge');
  swipeBadge.textContent=swipe.enabled?'ENABLED':'DISABLED';
  swipeBadge.className='badge '+(swipe.enabled?'warn':'');
  const gestures=sensor.gestures||{},selected=sensor.last_gesture;
  const countdown=swipe.countdown_s==null?'':(' · starts in '+Number(swipe.countdown_s).toFixed(1)+' s');
  $('sensorSwipeStatus').textContent=(swipe.error?swipe.error:(selected?((gestures[selected]?.label||selected)+' · '+(gestures[selected]?.program||'No program')+countdown):'Test a press or swipe; automatic starts are '+(swipe.enabled?'ON':'OFF')))+
    (swipe.feature_enabled?'':' · feature flag is OFF');
  $('sensorSwipeEnable').disabled=!swipe.feature_enabled||!fresh;
  const rows=$('sensorTapRows');
  const saved=Array.isArray(S.program?.saved)?S.program.saved:[];
  const mappingKey=JSON.stringify([gestures,saved]);
  if(rows&&rows.dataset.mappingKey!==mappingKey&&!rows.contains(document.activeElement)){
    rows.dataset.mappingKey=mappingKey;
    const safe=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
    rows.innerHTML=Object.entries(gestures).map(([i,tap])=>{
      const opts='<option value="">No program</option>'+saved.map(name=>'<option value="'+safe(name)+'"'+(name===tap.program?' selected':'')+'>'+safe(name)+'</option>').join('');
      const colors=['red','green','blue','yellow','cyan','magenta','white','off'].map(c=>'<option'+(c===tap.color?' selected':'')+'>'+c+'</option>').join('');
      return '<div class="fieldrow" style="margin:6px 0"><b>'+safe(tap.label)+'</b><select aria-label="'+safe(tap.label)+' program" data-sensor-program="'+i+'">'+opts+'</select><select aria-label="'+safe(tap.label)+' color" data-sensor-color="'+i+'">'+colors+'</select><span data-sensor-selected="'+i+'" class="badge good"'+(i===selected?'':' hidden')+'>LAST</span></div>';
    }).join('');
  }
  rows?.querySelectorAll('[data-sensor-selected]').forEach(el=>{el.hidden=el.dataset.sensorSelected!==selected});
}
async function refreshSensor(){
  if(activeView!=='sensors')return;
  try{renderSensor(await api('/sensor'))}catch(e){const b=$('sensorLinkBadge');if(b){b.textContent='DISCONNECTED';b.className='badge bad'}}
}
function rs(s){
  Object.assign(S,s);
  invalidateEndstopsAfterMotion();
  updateLinearHardwareUI();
  renderProgram(S.program);
  renderSensor(S.sensor);
  if(S.boundary_test_session){
    pts.length=0;
    for(const p of (S.boundary_test_session.points||[])) pts.push(p);
    const ts=$('testSession');
    if(ts) ts.textContent='Test '+S.boundary_test_session.id+' · '+pts.length+' pts';
  }
  $('msg').textContent=(S.busy?'BUSY — ':'')+(S.message||'');
  $('msg').className=S.busy?'busy':'ok';
  const hb=$('homedBadge');
  hb.textContent=S.home_required?'HOME REQUIRED':(S.homed?'HOMED':'NOT HOMED');
  hb.className='badge '+(S.homed&&!S.home_required?'good':'bad');
  const bb=$('boundaryToggle');
  if(bb){
    const on=S.boundary_enabled!==false;
    bb.textContent=on?'Boundary ON':'BOUNDARY OFF';
    bb.className=on?'good':'danger';
  }
  setMotionBadge(Boolean(S.busy||motionAnim));
  if($('targetZ') && !$('targetZ').dataset.userSet) $('targetZ').value=S.z.toFixed(1);

  // Establish one authoritative startup pose from backend state BEFORE
  // accepting planner telemetry. This prevents stale A/B/C/D frames from
  // flashing on page load.
  if(firstStateSync){
    DISPLAY.x=Number(S.x)||0;
    DISPLAY.y=Number(S.y)||0;
    DISPLAY.z=Number(S.z)||0;
    DISPLAY.linear_y=Number(S.linear_y)||0;
    LIVE_TARGET.x=DISPLAY.x;LIVE_TARGET.y=DISPLAY.y;LIVE_TARGET.z=DISPLAY.z;LIVE_TARGET.linear_y=DISPLAY.linear_y;
    livePoseValid=false;
    stateReady=true;
    firstStateSync=false;
    renderDisplayState();
  }else if(!S.homed){
    livePoseValid=false;
    DISPLAY.x=Number(S.x)||0;
    DISPLAY.y=Number(S.y)||0;
    DISPLAY.z=Number(S.z)||0;
    DISPLAY.linear_y=Number(S.linear_y)||0;
    LIVE_TARGET.x=DISPLAY.x;LIVE_TARGET.y=DISPLAY.y;LIVE_TARGET.z=DISPLAY.z;LIVE_TARGET.linear_y=DISPLAY.linear_y;
    renderDisplayState();
  }
}
async function refresh(){
  try{
    const latest=await api('/state');
    // Programmed movements is a client-side editor.  Its rows must never be
    // reconstructed from the last saved route by the background state poll.
    // Explicit editor actions (Analyze, add, record, reorder, remove, load)
    // call rs() themselves and are the only things that replace the table.
    if(activeView==='program'){Object.assign(S,latest);return;}
    rs(latest);
  }catch(e){$('msg').textContent=e.message}
}
let liveMotionBusy=false;
let livePoseValid=false;
let stateReady=false;
let firstStateSync=true;
let LIVE_TARGET={x:0,y:0,z:0,linear_y:0};
let liveRenderRaf=0;
let lastRenderTs=0;
let lastReadoutTs=0;
let moonWs=null, moonWsReady=false, moonWsRetry=null;

function applyLiveMotion(m){
  if(!stateReady || !m)return;
  const pos=m.live_position||m.position;
  if(!Array.isArray(pos)||pos.length<8)return;
  const x=Number(pos[4]),y=Number(pos[5]),z=Number(pos[6]),linear=Number(pos[7]);
  if(!Number.isFinite(x)||!Number.isFinite(y)||!Number.isFinite(z)||!Number.isFinite(linear))return;

  if(S.homed && Math.abs(x)+Math.abs(y)+Math.abs(z)+Math.abs(linear)<0.01)return;

  if(!livePoseValid && !S.busy){
    const sx=Number(S.x)||0, sy=Number(S.y)||0, sz=Number(S.z)||0;
    const sd=Number(S.linear_y)||0;
    const armErr=Math.hypot(x-sx,y-sy,z-sz);
    const slideErr=Math.abs(linear-sd);
    if(armErr>1.0 || slideErr>1.5)return;
  }

  // Telemetry only updates the latest target pose. Rendering is decoupled and
  // runs at a steady browser cadence below.
  LIVE_TARGET.x=x;
  LIVE_TARGET.y=y;
  LIVE_TARGET.z=z;
  LIVE_TARGET.linear_y=linear;
  livePoseValid=true;
  setMotionBadge(Math.abs(Number(m.live_velocity)||0)>0.001 || Boolean(S.busy));
}


function applyArmRobotStatus(a){
  if(!a)return;
  const u={};
  if(a.homed!==undefined)u.homed=Boolean(a.homed);
  if(a.linear_enabled!==undefined)u.linear_hardware=Boolean(a.linear_enabled);
  if(a.dance_active!==undefined && !(S.program&&S.program.running))u.busy=Boolean(a.dance_active);
  if(Number.isFinite(Number(a.A)))u.x=Number(a.A);
  if(Number.isFinite(Number(a.B)))u.y=Number(a.B);
  if(Number.isFinite(Number(a.C)))u.z=Number(a.C);
  if(Number.isFinite(Number(a.D)))u.linear_y=Number(a.D);
  if(Number.isFinite(Number(a.arm_velocity)))u.arm_speed=Number(a.arm_velocity);
  if(Number.isFinite(Number(a.arm_accel)))u.arm_accel=Number(a.arm_accel);
  if(Number.isFinite(Number(a.arm_corner)))u.arm_jerk=Number(a.arm_corner);
  if(Number.isFinite(Number(a.slide_velocity)))u.linear_speed=Number(a.slide_velocity);
  if(Number.isFinite(Number(a.slide_accel)))u.linear_accel=Number(a.slide_accel);
  if(Number.isFinite(Number(a.slide_corner)))u.linear_jerk=Number(a.slide_corner);
  rs(u);
}

function connectMoonrakerMotion(){
  try{
    const proto=location.protocol==='https:'?'wss':'ws';
    moonWs=new WebSocket(proto+'://'+location.host+'/controller-stream');
    moonWs.onopen=()=>{
      moonWsReady=true;
      moonWs.send(JSON.stringify({
        jsonrpc:'2.0',
        method:'printer.objects.subscribe',
        params:{objects:{motion_report:['live_position','live_velocity'],arm_robot:['homed','linear_enabled','dance_active','A','B','C','D','arm_velocity','arm_accel','arm_corner','slide_velocity','slide_accel','slide_corner']}},
        id:991
      }));
    };
    moonWs.onmessage=(ev)=>{
      try{
        const msg=JSON.parse(ev.data);
        if(msg.id===991 && msg.result && msg.result.status){
          if(msg.result.status.motion_report)applyLiveMotion(msg.result.status.motion_report);
          if(msg.result.status.arm_robot)applyArmRobotStatus(msg.result.status.arm_robot);
          return;
        }
        if(msg.method==='notify_status_update' && Array.isArray(msg.params)){
          const status=msg.params[0]||{};
          if(status.motion_report)applyLiveMotion(status.motion_report);
          if(status.arm_robot)applyArmRobotStatus(status.arm_robot);
        }
      }catch(e){}
    };
    moonWs.onclose=()=>{
      moonWsReady=false;
      clearTimeout(moonWsRetry);
      moonWsRetry=setTimeout(connectMoonrakerMotion,1000);
    };
    moonWs.onerror=()=>{try{moonWs.close()}catch(e){}};
  }catch(e){
    moonWsReady=false;
    clearTimeout(moonWsRetry);
    moonWsRetry=setTimeout(connectMoonrakerMotion,1000);
  }
}

async function refreshLiveMotion(){
  // HTTP fallback only. Normally the websocket above provides live planner
  // telemetry without request/response latency.
  if(moonWsReady||liveMotionBusy)return;
  liveMotionBusy=true;
  try{
    const r=await fetch('/live_motion',{cache:'no-store'});
    const m=await r.json();
    if(!m.error && Number.isFinite(m.x) && Number.isFinite(m.y) && Number.isFinite(m.z)){
      applyLiveMotion({
        live_position:[0,0,0,0,Number(m.x),Number(m.y),Number(m.z),Number(m.linear_y)||0],
        live_velocity:Number(m.velocity)||0
      });
    }
  }catch(e){}
  finally{liveMotionBusy=false;}
}
function liveRenderLoop(ts){
  liveRenderRaf=requestAnimationFrame(liveRenderLoop);

  // Four canvases are moderately expensive; 30 FPS is smooth and keeps CPU low.
  if(ts-lastRenderTs<33)return;
  lastRenderTs=ts;
  if(!livePoseValid)return;

  // Very short smoothing window to bridge irregular telemetry packet timing.
  // This is intentionally aggressive to keep added latency small.
  const alpha=0.78;
  DISPLAY.x += (LIVE_TARGET.x-DISPLAY.x)*alpha;
  DISPLAY.y += (LIVE_TARGET.y-DISPLAY.y)*alpha;
  DISPLAY.z += (LIVE_TARGET.z-DISPLAY.z)*alpha;
  DISPLAY.linear_y += (LIVE_TARGET.linear_y-DISPLAY.linear_y)*alpha;

  if(Math.abs(LIVE_TARGET.x-DISPLAY.x)<0.01)DISPLAY.x=LIVE_TARGET.x;
  if(Math.abs(LIVE_TARGET.y-DISPLAY.y)<0.01)DISPLAY.y=LIVE_TARGET.y;
  if(Math.abs(LIVE_TARGET.z-DISPLAY.z)<0.01)DISPLAY.z=LIVE_TARGET.z;
  if(Math.abs(LIVE_TARGET.linear_y-DISPLAY.linear_y)<0.02)DISPLAY.linear_y=LIVE_TARGET.linear_y;

  // Only the four live canvases/readout kinematics are redrawn here.
  // Heavy SVG workspace/boundary plots are not touched.
  updateKinematics();

  // DOM text changes are throttled; they are much slower than canvas drawing.
  if(ts-lastReadoutTs>100){
    $('sx').textContent=DISPLAY.x.toFixed(1);
    $('sy').textContent=DISPLAY.y.toFixed(1);
    $('sz').textContent=DISPLAY.z.toFixed(1);
    if($('slinear'))$('slinear').textContent=DISPLAY.linear_y.toFixed(1)+' mm';
    lastReadoutTs=ts;
  }
}


function motionSettings(){
  const accelEnabled=!$('accelEnabled')||$('accelEnabled').checked;
  const cornerEnabled=!$('cornerEnabled')||$('cornerEnabled').checked;
  return {
    arm_speed:Math.max(1,Math.min(220,parseFloat($('armSpeed').value)||80)),
    arm_accel:accelEnabled?Math.max(10,Math.min(1800,parseFloat($('armAccel').value)||250)):1800,
    arm_jerk:cornerEnabled?Math.max(0,Math.min(40,parseFloat($('armJerk').value)||18)):0,
    linear_speed:Math.max(1,Math.min(100,parseFloat($('linearSpeed').value)||25)),
    linear_accel:accelEnabled?Math.max(10,Math.min(500,parseFloat($('linearAccel').value)||100)):500,
    linear_jerk:cornerEnabled?Math.max(0,Math.min(20,parseFloat($('linearJerk').value)||5)):0
  };
}
async function applyMotionTuning(){
  try{rs(await api('/motion_tuning',motionSettings()))}
  catch(e){alert(e.message)}
}


async function linearJog(delta){
  const step=Math.abs(Number(delta)||0);
  try{const m=motionSettings();const finalState=await api('/linear_jog',{delta:Number(delta)||0,speed:m.linear_speed,accel:m.linear_accel});rs(finalState)}
  catch(e){alert(e.message)}
}
async function linearGoto(){
  const target=Math.max(0,Math.min(235,parseFloat($('linearTarget').value)||0));
  try{const m=motionSettings();const finalState=await api('/linear_goto',{target,speed:m.linear_speed,accel:m.linear_accel});rs(finalState)}
  catch(e){alert(e.message)}
}

async function jog(x,y,z){
  const m=motionSettings();
  const target={x:Math.max(0,S.x+(x||0)),y:Math.max(0,S.y+(y||0)),z:Math.max(0,S.z+(z||0))};
  try{
    const finalState=await api('/jog',{x:x||0,y:y||0,z:z||0,speed:m.arm_speed,accel:m.arm_accel});
    stopMotionAnimation(); rs(finalState);
  }catch(e){
    stopMotionAnimation();DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;DISPLAY.linear_y=Number(S.linear_y)||0;renderDisplayState();alert(e.message)
  }
}
async function fineJog(x,y){
  const target={x:Math.max(0,S.x+(x||0)),y:Math.max(0,S.y+(y||0)),z:S.z};
  try{const m=motionSettings();const finalState=await api('/fine_jog',{x:x||0,y:y||0,speed:m.arm_speed,accel:m.arm_accel});stopMotionAnimation();rs(finalState)}
  catch(e){stopMotionAnimation();DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;DISPLAY.linear_y=Number(S.linear_y)||0;renderDisplayState();alert(e.message)}
}
async function goCartesianTarget(y,z){
  y=Number(y); z=Number(z);
  if(!Number.isFinite(y)||!Number.isFinite(z)){alert('Enter valid Tool Y and Z');return;}
  const m=motionSettings();
  selectedCartTarget={y,z};
  drawCartesianWorkspace();
  try{
    const finalState=await api('/goto_cart',{y,z,speed:m.arm_speed,accel:m.arm_accel});
    stopMotionAnimation();
    rs(finalState);
  }catch(e){alert(e.message);refresh();}
}

async function goJointTarget(x,y,z,feedOverride=null){
  x=Number(x);y=Number(y);z=Number(z);
  if(!Number.isFinite(x)||!Number.isFinite(y)||!Number.isFinite(z)) throw Error('Invalid Motor X / Motor Y / Motor Z angle target');
  const m=motionSettings();
  selectedTarget={x,y};
  draw();
  try{
    const finalState=await api('/goto',{x,y,z,speed:m.arm_speed,accel:m.arm_accel});
    stopMotionAnimation();
    rs(finalState);
    return finalState;
  }catch(e){
    stopMotionAnimation();
    DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;DISPLAY.linear_y=Number(S.linear_y)||0;
    renderDisplayState();
    throw e;
  }
}
async function goToTarget(){
  const x=parseFloat($('targetX').value),y=parseFloat($('targetY').value),z=parseFloat($('targetZ').value);
  try{await goJointTarget(x,y,z)}
  catch(e){alert(e.message)}
}
async function runDance(){
  if(danceRunning)return;
  if(!S.homed){alert('Home robot first');return;}
  danceRunning=true;
  const btn=$('dance');
  btn.disabled=true;
  btn.textContent='Dance running — STOP to end';
  try{
    if(S.boundary_enabled===false) rs(await api('/boundary',{enabled:true}));
    const m=motionSettings();
    const started=await api('/dance_smooth',{
      speed:m.arm_speed,
      accel:m.arm_accel,
      linear_speed:m.linear_speed,
      linear_accel:m.linear_accel
    });
    rs(started);
    // The backend dance now runs independently.  Keep this button state tied
    // to /state, not to an endless HTTP request.
    const watch=setInterval(async()=>{
      try{
        const st=await api('/state');
        rs(st);
        if(!st.busy){
          clearInterval(watch);
          danceRunning=false;
          btn.disabled=false;
          btn.textContent='Dance ∞';
        }
      }catch(e){}
    },250);
  }catch(e){
    danceRunning=false;
    btn.disabled=false;
    btn.textContent='Dance ∞';
    alert(e.message);
  }
}

let homeRequestInFlight=false;
async function homeRobot(){
  if(homeRequestInFlight)return;
  homeRequestInFlight=true;
  const buttons=[$('home'),$('settingsHome')].filter(Boolean);
  buttons.forEach(button=>button.disabled=true);
  try{
    rs(await api('/home',{}));
    await refreshSettingsEndstops();
  }catch(e){alert(e.message)}
  finally{
    homeRequestInFlight=false;
    buttons.forEach(button=>button.disabled=false);
  }
}

function programUseCurrent(){
  $('programA').value=(Number(S.x)||0).toFixed(2);
  $('programB').value=(Number(S.y)||0).toFixed(2);
  $('programC').value=(Number(S.z)||0).toFixed(2);
  $('programD').value=(Number(S.linear_y)||0).toFixed(2);
  setProgramToolFromJoint();
}
async function programRecord(){
  try{rs(await api('/program_record',programWaypointSettings()))}catch(e){alert(e.message)}
}
function programWaypointSettings(){return {speed:Number($('programDefaultSpeed').value),time:Number($('programDefaultTime').value),wait:Number($('programDefaultWait').value)}}
async function programAdd(){
  if(programCoordinateMode==='cartesian' && !await solveProgramToolToJoint())return;
  const body={a:Number($('programA').value),b:Number($('programB').value),c:Number($('programC').value),d:Number($('programD').value),...programWaypointSettings()};
  if(!Number.isFinite(body.a)||!Number.isFinite(body.b)||!Number.isFinite(body.c)||!Number.isFinite(body.d)){alert('Enter Motor X, Motor Y, Motor Z, and rail values for the waypoint.');return;}
  try{rs(await api('/program_add',body))}catch(e){alert(e.message)}
}
function programDraftPoints(){
  const rows=$('programRows');
  if(!rows)return [];
  return [...rows.querySelectorAll('tr[data-program-row]')].map(row=>{
    const index=row.dataset.programRow;
    const read=key=>{
      const input=row.querySelector('[data-program-'+key+'="'+index+'"]');
      return key==='time'&&input.value.trim()===''?0:Number(input.value);
    };
    const timing={d:read('d'),speed:read('speed'),time:read('time'),wait:read('wait')};
    if(programTableMode==='joint')return {a:read('a'),b:read('b'),c:read('c'),...timing};
    const point={x:read('x'),y:read('y'),z:read('z'),...timing};
    // The table is whole millimetres, but a displayed joint route may have
    // an ambiguous inverse-kinematics branch. Keep its original branch only
    // while the displayed position still matches that exact stored pose.
    const hint=Object.fromEntries(['a','b','c','d','x','y','z'].map(key=>[key,Number(row.dataset['programHint'+key.toUpperCase()]) ]));
    if(Object.values(hint).every(Number.isFinite) &&
       Math.abs(point.x-hint.x)<=0.51 && Math.abs(point.y-hint.y)<=0.51 &&
       Math.abs(point.z-hint.z)<=0.51 && Math.abs(point.d-hint.d)<=0.51){
      point.joint_hint={a:hint.a,b:hint.b,c:hint.c,d:hint.d};
    }
    return point;
  });
}
async function validateProgramDraft(){
  const points=programDraftPoints();
  if(!points.length)throw new Error('Record or add at least one waypoint first.');
  if(points.some(point=>Object.entries(point).some(([key,value])=>key!=='joint_hint'&&!Number.isFinite(value))))throw new Error('Finish each waypoint with valid numbers before analyzing.');
  // The Program tab has historically used the screen-facing X convention.
  // Convert once at the controller boundary instead of changing what users
  // see or type in the table.
  const controllerPoints=programTableMode==='cartesian'
    ?points.map(point=>({...point,x:-point.x}))
    :points;
  const result=await api(programTableMode==='cartesian'?'/program_replace_cartesian':'/program_replace',{points:controllerPoints});
  programHasDrafts=false;
  rs(result);
}
async function programAnalyze(){
  programAnalyzeInFlight=true;
  try{await validateProgramDraft()}catch(e){programHasDrafts=true;alert(e.message)}finally{programAnalyzeInFlight=false;}
}
async function programRun(){
  let routeValidated=false;
  programAnalyzeInFlight=true;
  try{
    // Run is always preceded by the same safe validation as Analyze. This
    // makes the values visible in the editor the values that actually run.
    await validateProgramDraft();
    routeValidated=true;
    const m=motionSettings(),arm_corner=Number($('programCorner').value);
    if(!Number.isFinite(arm_corner)||arm_corner<0||arm_corner>50)throw new Error('Corner flow must be between 0 and 50.');
    rs(await api('/program_run',{arm_speed:m.arm_speed,arm_accel:m.arm_accel,linear_speed:m.linear_speed,linear_accel:m.linear_accel,arm_corner}));
  }catch(e){
    if(!routeValidated)programHasDrafts=true;
    alert(e.message);
  }finally{programAnalyzeInFlight=false;}
}
async function programSave(){
  const name=$('programName').value.trim();if(!name){alert('Enter a name to save this route as a new program.');return;}
  programAnalyzeInFlight=true;
  try{
    // Saving follows the same non-moving validation as Run, so a saved route
    // always contains the values currently visible in the editor.
    await validateProgramDraft();
    rs(await api('/program_save',{name}));
    $('programName').value='';
    $('programNewRow').hidden=true;
    $('programNew').hidden=false;
  }catch(e){programHasDrafts=true;alert(e.message)}finally{programAnalyzeInFlight=false;}
}
async function programUpdate(){
  const name=$('programSaved').value;if(!name){alert('Choose a saved program to update.');return;}
  if(!confirm('Overwrite saved program “'+name+'” with the route currently in the editor? This cannot be undone.'))return;
  programAnalyzeInFlight=true;
  try{
    // This deliberately writes only after the current editor values have
    // been validated. Loading, saving, and updating remain non-moving.
    await validateProgramDraft();
    rs(await api('/program_save',{name}));
  }catch(e){programHasDrafts=true;alert(e.message)}finally{programAnalyzeInFlight=false;}
}
async function programLoad(){
  const name=$('programSaved').value;if(!name){alert('Choose a saved program to load.');return;}
  try{rs(await api('/program_load',{name}));$('programName').value=''}catch(e){alert(e.message)}
}
async function programDelete(){
  const name=$('programSaved').value;if(!name){alert('Choose a saved program to delete.');return;}
  const confirmation=prompt('This permanently deletes saved program “'+name+'”. Type DELETE to continue.');
  if(confirmation!=='DELETE')return;
  try{rs(await api('/program_delete',{name,confirmation}))}catch(e){alert(e.message)}
}
async function programJog(axis,direction){
  const step=Number($('programStep').value)||1;
  await jog(axis==='a'?step*direction:0,axis==='b'?step*direction:0,axis==='c'?step*direction:0);
}
async function programCartesianJog(axis,direction){
  const step=Number($('programCartesianStep').value)||5, current=programWorldFromState();
  if(!current){alert('Current tool position is unavailable.');return;}
  current[axis]+=step*direction;
  try{
    const m=motionSettings();
    const body={x:-current.x,y:current.y,z:current.z,auto_d:!S.linear_hardware,linear_y:current.d,speed:m.arm_speed,accel:m.arm_accel};
    rs(await api('/goto_world',body));
    programUseCurrent();
  }catch(e){alert(e.message)}
}

$('programUseCurrent').addEventListener('click',programUseCurrent);
$('programRecord').addEventListener('click',programRecord);
$('programAdd').addEventListener('click',programAdd);
$('programSaveDefaults').addEventListener('click',async()=>{try{rs(await api('/program_defaults',programWaypointSettings()))}catch(e){alert(e.message)}});
$('programSave').addEventListener('click',programSave);
$('programUpdate').addEventListener('click',programUpdate);
$('programNew').addEventListener('click',()=>{
  $('programNewRow').hidden=false;
  $('programNew').hidden=true;
  $('programName').focus();
});
$('programCancelNew').addEventListener('click',()=>{
  $('programName').value='';
  $('programNewRow').hidden=true;
  $('programNew').hidden=false;
});
$('programSaved').addEventListener('change',()=>{
  const name=$('programSaved').value;
  renderProgramSavedTarget(name,S.program&&S.program.running);
});
$('programLoad').addEventListener('click',programLoad);
$('programDelete').addEventListener('click',programDelete);
if($('programAnalyze'))$('programAnalyze').addEventListener('click',programAnalyze);
$('programRun').addEventListener('click',programRun);
$('programClear').addEventListener('click',async()=>{if(!confirm('Clear every recorded program point?'))return;try{rs(await api('/program_clear',{}))}catch(e){alert(e.message)}});
document.querySelectorAll('[data-program-axis]').forEach(button=>button.addEventListener('click',()=>programJog(button.dataset.programAxis,Number(button.dataset.programDir))));
document.querySelectorAll('[data-program-cart-axis]').forEach(button=>button.addEventListener('click',()=>programCartesianJog(button.dataset.programCartAxis,Number(button.dataset.programCartDir))));
$('programModeJoint').addEventListener('click',()=>setProgramCoordinateMode('joint'));
$('programModeCartesian').addEventListener('click',()=>setProgramCoordinateMode('cartesian'));
['programA','programB','programC','programD'].forEach(id=>$(id).addEventListener('input',setProgramToolFromJoint));
['programToolX','programToolY','programToolZ','programToolD'].forEach(id=>$(id).addEventListener('input',scheduleProgramToolSolve));
const programPathCanvas=$('programPath3d');
programPathCanvas.addEventListener('pointerdown',event=>{programPathDragging=true;programPathLastX=event.clientX;programPathLastY=event.clientY;programPathCanvas.setPointerCapture(event.pointerId);});
programPathCanvas.addEventListener('pointermove',event=>{if(!programPathDragging)return;programPathYaw+=(event.clientX-programPathLastX)*0.009;programPathPitch=Math.max(-1.25,Math.min(1.25,programPathPitch+(event.clientY-programPathLastY)*0.009));programPathLastX=event.clientX;programPathLastY=event.clientY;renderProgramPath(programHasDrafts?programDraftPoints():(S.program?.points||[]));});
programPathCanvas.addEventListener('pointerup',()=>{programPathDragging=false});
programPathCanvas.addEventListener('pointercancel',()=>{programPathDragging=false});
programPathCanvas.addEventListener('wheel',event=>{event.preventDefault();programPathZoom=Math.max(.55,Math.min(3,programPathZoom*(event.deltaY<0?1.12:.89)));renderProgramPath(programHasDrafts?programDraftPoints():(S.program?.points||[]));},{passive:false});
programPathCanvas.addEventListener('dblclick',()=>{programPathZoom=1;renderProgramPath(programHasDrafts?programDraftPoints():(S.program?.points||[]));});

$('goTarget').addEventListener('click',goToTarget);
$('dance').addEventListener('click',runDance);
$('cartGo').addEventListener('click',()=>goCartesianTarget(parseFloat($('cartTargetY').value),parseFloat($('cartTargetZ').value)));
$('cartUseCurrent').addEventListener('click',()=>{
  const p=(!motionAnim && S.kinematics)?backendRobotPose(S.kinematics):solvedRobotPose(DISPLAY);
  if(!p)return;
  $('cartTargetY').value=p.tool.y.toFixed(1);
  $('cartTargetZ').value=p.tool.z.toFixed(1);
  selectedCartTarget={y:p.tool.y,z:p.tool.z};
  drawCartesianWorkspace();
});
$('basePreview').addEventListener('input',drawCartesianWorkspace);
$('targetZ').addEventListener('input',()=>{$('targetZ').dataset.userSet='1'});
$('useCurrent').addEventListener('click',()=>{
  selectedTarget={x:S.x,y:S.y};
  $('targetX').value=S.x.toFixed(1);
  $('targetY').value=S.y.toFixed(1);
  $('targetZ').value=S.z.toFixed(1);
  $('targetZ').dataset.userSet='1';
  draw();
});
document.querySelectorAll('[data-step-value]').forEach(b=>b.addEventListener('click',()=>{
  $('step').value=b.dataset.stepValue;
  document.querySelectorAll('[data-step-value]').forEach(q=>q.classList.remove('active'));
  b.classList.add('active');
}));
document.querySelectorAll('[data-axis]').forEach(b=>b.addEventListener('click',()=>{const v=parseFloat($('step').value)||1,d=parseFloat(b.dataset.dir),a=b.dataset.axis;jog(a==='x'?v*d:0,a==='y'?v*d:0,a==='z'?v*d:0)}));
document.querySelectorAll('[data-linear-dir]').forEach(b=>b.addEventListener('click',()=>{const v=parseFloat($('step').value)||1,d=parseFloat(b.dataset.linearDir);linearJog(v*d)}));
$('linearGo').addEventListener('click',linearGoto);
['armSpeed','armAccel','armJerk','linearSpeed','linearAccel','linearJerk'].forEach(id=>{
  $(id).addEventListener('change',applyMotionTuning);
  $(id).addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();applyMotionTuning();}});
});
['accelEnabled','cornerEnabled'].forEach(id=>$(id).addEventListener('change',applyMotionTuning));
document.querySelectorAll('[data-fine-axis]').forEach(b=>b.addEventListener('click',()=>{const d=parseFloat(b.dataset.dir),a=b.dataset.fineAxis;fineJog(a==='x'?0.25*d:0,a==='y'?0.25*d:0)}));
$('xyMove').addEventListener('click',()=>jog(parseFloat($('dx').value)||0,parseFloat($('dy').value)||0,0));
$('xyBack').addEventListener('click',()=>jog(-(parseFloat($('dx').value)||0),-(parseFloat($('dy').value)||0),0));
$('boundaryToggle').addEventListener('click',async()=>{
  const currentlyOn=S.boundary_enabled!==false;
  if(currentlyOn && !confirm('Turn OFF measured X/Y boundary protection? Motion can leave the validated workspace.'))return;
  try{rs(await api('/boundary',{enabled:!currentlyOn}))}catch(e){alert(e.message)}
});
$('home').addEventListener('click',homeRobot);
$('stop').addEventListener('click',async()=>{try{rs(await api('/stop',{}))}catch(e){alert(e.message)}});
$('kill').addEventListener('click',async()=>{if(!confirm('KILL immediately? Position will be lost and Home will be required.'))return;try{rs(await api('/kill',{}))}catch(e){alert(e.message)}});
$('record').addEventListener('click',async()=>{
  try{rs(await api('/boundary_test_record',{}))}catch(e){alert(e.message)}
});
$('newBoundaryTest').addEventListener('click',async()=>{
  if(!confirm('Start a completely new boundary test session? The old measured envelope stays as reference.'))return;
  try{rs(await api('/boundary_test_new',{}))}catch(e){alert(e.message)}
});

const armCanvas=$('armFree');
armCanvas.addEventListener('pointerdown',e=>{
  drag3d=true;last3dX=e.clientX;last3dY=e.clientY;armCanvas.setPointerCapture(e.pointerId);
});
armCanvas.addEventListener('pointermove',e=>{
  if(!drag3d)return;
  viewYaw+=(e.clientX-last3dX)*0.008;
  viewPitch=Math.max(-1.25,Math.min(1.25,viewPitch+(e.clientY-last3dY)*0.008));
  last3dX=e.clientX;last3dY=e.clientY;
  updateKinematics();
});
armCanvas.addEventListener('pointerup',()=>{drag3d=false});
armCanvas.addEventListener('pointercancel',()=>{drag3d=false});

function cartBoundaryPoints(){
  return REF_BOUNDARY.slice(0,-1).map(q=>{
    const a=jointAnglesFromCounts({x:q.x,y:q.y,z:DISPLAY.z});
    const p=robotArmFromJointDegrees(a.qLowerDeg,a.qCrankDeg,0);
    return p?{y:p.tool.y,z:p.tool.z}:null;
  }).filter(Boolean);
}
function drawCartesianWorkspace(){
  const svg=$('cartPlot'); if(!svg)return;
  const W=700,H=360,pad=40;
  const boundary=cartBoundaryPoints();
  const pose=(!motionAnim && S.kinematics)?backendRobotPose(S.kinematics):solvedRobotPose(DISPLAY);
  const pts=boundary.slice();
  if(pose)pts.push({y:pose.tool.y,z:pose.tool.z});
  if(selectedCartTarget)pts.push(selectedCartTarget);
  if(!pts.length)return;
  let minY=Math.min(...pts.map(q=>q.y)),maxY=Math.max(...pts.map(q=>q.y));
  let minZ=Math.min(...pts.map(q=>q.z)),maxZ=Math.max(...pts.map(q=>q.z));
  const dy=Math.max(40,maxY-minY),dz=Math.max(40,maxZ-minZ);
  minY-=dy*.12;maxY+=dy*.12;minZ-=dz*.12;maxZ+=dz*.12;
  const X=y=>pad+(W-2*pad)*(y-minY)/(maxY-minY);
  const Y=z=>H-pad-(H-2*pad)*(z-minZ)/(maxZ-minZ);
  let h='<rect width="700" height="360" fill="#0b0d10"/>';
  for(let i=0;i<=4;i++){
    const yy=minY+(maxY-minY)*i/4,zz=minZ+(maxZ-minZ)*i/4;
    h+='<line x1="'+X(yy)+'" y1="'+pad+'" x2="'+X(yy)+'" y2="'+(H-pad)+'" stroke="#202831"/><text x="'+X(yy)+'" y="345" fill="#76818d" text-anchor="middle" font-size="10">'+yy.toFixed(0)+'</text>';
    h+='<line x1="'+pad+'" y1="'+Y(zz)+'" x2="'+(W-pad)+'" y2="'+Y(zz)+'" stroke="#202831"/><text x="34" y="'+(Y(zz)+3)+'" fill="#76818d" text-anchor="end" font-size="10">'+zz.toFixed(0)+'</text>';
  }
  if(boundary.length){
    const closed=boundary.concat([boundary[0]]);
    h+='<polygon points="'+closed.map(q=>X(q.y)+','+Y(q.z)).join(' ')+'" fill="rgba(100,181,246,.08)" stroke="#64b5f6" stroke-width="2"/>';
  }
  if(pose)h+='<circle cx="'+X(pose.tool.y)+'" cy="'+Y(pose.tool.z)+'" r="7" fill="#64b5f6" stroke="#d9ecff" stroke-width="2"/><text x="'+(X(pose.tool.y)+10)+'" y="'+(Y(pose.tool.z)-8)+'" fill="#d9ecff" font-size="10">Current</text>';
  if(selectedCartTarget)h+='<circle cx="'+X(selectedCartTarget.y)+'" cy="'+Y(selectedCartTarget.z)+'" r="7" fill="none" stroke="#ffcc66" stroke-width="2"/><text x="'+(X(selectedCartTarget.y)+10)+'" y="'+(Y(selectedCartTarget.z)-8)+'" fill="#ffcc66" font-size="10">Target</text>';
  const previewDeg=Number($('basePreview')?.value||0);
  const pr=previewDeg*Math.PI/180;
  if(pose){
    const wx=pose.tool.y*Math.sin(pr), wy=pose.tool.y*Math.cos(pr);
    h+='<text x="690" y="18" fill="#98a2ad" text-anchor="end" font-size="10">Base '+previewDeg.toFixed(0)+'° → world X '+wx.toFixed(0)+', Y '+wy.toFixed(0)+'</text>';
  }
  h+='<text x="350" y="356" fill="#98a2ad" text-anchor="middle" font-size="10">Arm radial Y (mm)</text><text x="12" y="18" fill="#98a2ad" font-size="10">Arm Z (mm)</text>';
  svg.innerHTML=h;
  svg.dataset.minY=minY;svg.dataset.maxY=maxY;svg.dataset.minZ=minZ;svg.dataset.maxZ=maxZ;
}
function clientToSvg(svg,clientX,clientY){
  const rect=svg.getBoundingClientRect();
  const vb=svg.viewBox.baseVal;
  if(!rect.width||!rect.height||!vb.width||!vb.height)return null;
  // Default preserveAspectRatio is xMidYMid meet. Work in rendered pixels,
  // then remove the letterbox offset before converting to viewBox units.
  const scale=Math.min(rect.width/vb.width,rect.height/vb.height);
  const drawnW=vb.width*scale,drawnH=vb.height*scale;
  const offX=(rect.width-drawnW)/2,offY=(rect.height-drawnH)/2;
  const localX=clientX-rect.left-offX,localY=clientY-rect.top-offY;
  if(localX<0||localY<0||localX>drawnW||localY>drawnH)return null;
  return {x:vb.x+localX/scale,y:vb.y+localY/scale};
}

$('cartPlot').addEventListener('click',async e=>{
  const svg=$('cartPlot');
  const sp=clientToSvg(svg,e.clientX,e.clientY); if(!sp)return;
  const W=700,H=360,pad=40;
  if(sp.x<pad||sp.x>W-pad||sp.y<pad||sp.y>H-pad)return;
  const minY=Number(svg.dataset.minY),maxY=Number(svg.dataset.maxY),minZ=Number(svg.dataset.minZ),maxZ=Number(svg.dataset.maxZ);
  const y=minY+(sp.x-pad)*(maxY-minY)/(W-2*pad);
  const z=minZ+(H-pad-sp.y)*(maxZ-minZ)/(H-2*pad);
  $('cartTargetY').value=y.toFixed(1);$('cartTargetZ').value=z.toFixed(1);
  selectedCartTarget={y,z};drawCartesianWorkspace();
  await goCartesianTarget(y,z);
});

$('plot').addEventListener('click',async e=>{
  const svg=$('plot');
  // Use post-transform browser bounds + viewBox math. This remains correct
  // even when the entire 1600x900 UI is CSS-scaled by fitDesignSurface().
  const sp=clientToSvg(svg,e.clientX,e.clientY);
  if(!sp)return;
  const px=sp.x,py=sp.y;

  // Use the EXACT same scale inputs as draw(). Any difference here
  // moves the cursor target away from the visible click position.
  const W=700,H=520,p=48;
  const all=REF_RAW.concat(REF_REPEAT,REF_BOUNDARY,pts,[{x:DISPLAY.x,y:DISPLAY.y}]);
  const mx=Math.max(200,...all.map(q=>q.x))*1.06;
  const my=Math.max(155,...all.map(q=>q.y))*1.08;

  // Ignore clicks in the axis-label margins rather than creating bogus
  // negative/out-of-range targets.
  if(px<p || px>W-p || py<p || py>H-p)return;

  const x=(px-p)*mx/(W-2*p);
  const y=(H-p-py)*my/(H-2*p);

  selectedTarget={x,y};
  $('targetX').value=x.toFixed(1);
  $('targetY').value=y.toFixed(1);
  draw();

  // Plot click means "go here". The backend still applies the measured
  // boundary + 5-count safety yield and clips the path if necessary.
  try{
    const z=parseFloat($('targetZ').value);
    const tz=Number.isFinite(z)?z:S.z;
    await goJointTarget(x,y,tz);
  }catch(err){
    alert(err.message);
  }
});

function draw(){
const svg=$('plot'),W=700,H=520,p=48;
const all=REF_RAW.concat(REF_REPEAT,REF_BOUNDARY,pts,[{x:DISPLAY.x,y:DISPLAY.y}]);
const mx=Math.max(200,...all.map(q=>q.x))*1.06;
const my=Math.max(155,...all.map(q=>q.y))*1.08;
const X=v=>p+(W-2*p)*v/mx,Y=v=>H-p-(H-2*p)*v/my;
let h='<rect width="700" height="520" fill="#0c0c0c"/>';

// grid
for(let i=0;i<=5;i++){
  const vx=mx*i/5,vy=my*i/5;
  h+='<line x1="'+X(vx)+'" y1="'+p+'" x2="'+X(vx)+'" y2="'+(H-p)+'" stroke="#252525"/><text x="'+X(vx)+'" y="'+(H-14)+'" fill="#888" text-anchor="middle" font-size="12">'+vx.toFixed(0)+'</text>';
  h+='<line x1="'+p+'" y1="'+Y(vy)+'" x2="'+(W-p)+'" y2="'+Y(vy)+'" stroke="#252525"/><text x="38" y="'+(Y(vy)+4)+'" fill="#888" text-anchor="end" font-size="12">'+vy.toFixed(0)+'</text>';
}

// Draw the exact envelope used by the host and Klipper, including calibration.
const cleanFit=(S.boundary_poly||[]).map(q=>({x:q[0],y:q[1]}));
h+='<polyline points="'+cleanFit.map(q=>X(q.x)+','+Y(q.y)).join(' ')+'" fill="none" stroke="#f0b35a" stroke-width="3.5" stroke-linecap="round" stroke-linejoin="round"/>';

// original raw measurements
REF_RAW.forEach((q,i)=>{
  h+='<circle cx="'+X(q.x)+'" cy="'+Y(q.y)+'" r="3.5" fill="#667085" stroke="#a0a7b2" stroke-width="0.7"/>';
});

// persisted repeat measurements supplied after the original trial
REF_REPEAT.forEach((q,i)=>{
  h+='<circle cx="'+X(q.x)+'" cy="'+Y(q.y)+'" r="4" fill="none" stroke="#d7d7d7" stroke-width="1.4"/>';
});

// newly recorded repeat points + path
if(pts.length>1) h+='<polyline points="'+pts.map(q=>X(q.x)+','+Y(q.y)).join(' ')+'" fill="none" stroke="#fff" stroke-width="2" stroke-dasharray="5 4"/>';
pts.forEach((q,i)=>{
  h+='<circle cx="'+X(q.x)+'" cy="'+Y(q.y)+'" r="5" fill="#fff"/><text x="'+(X(q.x)+7)+'" y="'+(Y(q.y)-7)+'" fill="#fff" font-size="11">R'+(i+1)+'</text>';
});

// selected click/manual target
if(selectedTarget){
  const tx=X(selectedTarget.x),ty=Y(selectedTarget.y);
  h+='<line x1="'+(tx-9)+'" y1="'+ty+'" x2="'+(tx+9)+'" y2="'+ty+'" stroke="#ffcc66" stroke-width="2"/>';
  h+='<line x1="'+tx+'" y1="'+(ty-9)+'" x2="'+tx+'" y2="'+(ty+9)+'" stroke="#ffcc66" stroke-width="2"/>';
  h+='<circle cx="'+tx+'" cy="'+ty+'" r="7" fill="none" stroke="#ffcc66" stroke-width="2"/>';
  h+='<text x="'+(tx+10)+'" y="'+(ty-10)+'" fill="#ffcc66" font-size="11">Target '+selectedTarget.x.toFixed(1)+', '+selectedTarget.y.toFixed(1)+'</text>';
}

// live current position
h+='<circle cx="'+X(DISPLAY.x)+'" cy="'+Y(DISPLAY.y)+'" r="7" fill="#64b5f6" stroke="#d9ecff" stroke-width="2"/>';
svg.innerHTML=h;

$('rows').innerHTML=pts.map((q,i)=>'<tr><td>'+(i+1)+'</td><td>'+q.x.toFixed(1)+'</td><td>'+q.y.toFixed(1)+'</td><td>'+q.z.toFixed(1)+'</td><td>'+((q.base_deg??0).toFixed? q.base_deg.toFixed(1):q.base_deg)+'</td></tr>').join('');
}
setInterval(refresh,2000);
setInterval(refreshLiveMotion,250);
let endstopRefreshInFlight=false,endstopReadPose=null;
function renderLiveEndstop(id,label,triggered){
  const el=$(id); if(!el)return;
  el.textContent=label+' · '+(triggered?'TRIGGERED':'OPEN');
  el.className='endstopState'+(triggered?' triggered':'');
}
function markEndstopsStale(){
  endstopReadPose=null;
  const label=$('endstopStripLabel');if(label)label.textContent='Endstops · stale after motion · click to refresh';
  [['endstopXLive','Motor X'],['endstopYLive','Motor Y'],['endstopZLive','Motor Z']].forEach(([id,name])=>{
    const el=$(id);if(el){el.textContent=name+' · STALE';el.className='endstopState stale';}
  });
}
function invalidateEndstopsAfterMotion(){
  if(!endstopReadPose)return;
  const moved=Math.max(Math.abs((Number(S.x)||0)-endstopReadPose.a),Math.abs((Number(S.y)||0)-endstopReadPose.b),Math.abs((Number(S.z)||0)-endstopReadPose.c),Math.abs((Number(S.linear_y)||0)-endstopReadPose.d));
  if(moved>0.1)markEndstopsStale();
}
async function refreshLiveEndstops(){
  if(endstopRefreshInFlight)return;
  endstopRefreshInFlight=true;
  try{
    const r=await api('/settings_endstops',{});
    renderLiveEndstop('endstopXLive','Motor X',!!r.endstop_x);
    renderLiveEndstop('endstopYLive','Motor Y',!!r.endstop_y);
    renderLiveEndstop('endstopZLive','Motor Z',!!r.endstop_z);
    endstopReadPose={a:Number(S.x)||0,b:Number(S.y)||0,c:Number(S.z)||0,d:Number(S.linear_y)||0};
    const label=$('endstopStripLabel');if(label)label.textContent='Endstops · checked just now · click to refresh';
  }catch(e){
    ['endstopXLive','endstopYLive','endstopZLive'].forEach(id=>{
      const el=$(id); if(el){el.textContent='Endstop read error';el.className='endstopState';}
    });
  }finally{endstopRefreshInFlight=false;}
}
// QUERY_ENDSTOPS is a synchronous controller request. Keep it user-initiated
// so it cannot contend with program editing or motion planning.
$('endstopStrip').addEventListener('click',refreshLiveEndstops);
async function bootLiveUI(){
  await refresh();
  updateKinematics();
  draw();
  drawCartesianWorkspace();
  connectMoonrakerMotion();
  refreshLiveMotion();
  if(!liveRenderRaf)liveRenderRaf=requestAnimationFrame(liveRenderLoop);
}
bootLiveUI();

let activeView='control',workspaceLoaded=false,workspaceTransform=null,workspaceData=null,selectedWorld=null,worldPlan=null,worldMoveInFlight=false,debugWindowSeconds=300;
function setView(v){
  activeView=v;
  document.querySelector('.dashboard').style.display=(v==='control')?'grid':'none';
  $('workspacePanel').classList.toggle('active',v==='workspace');
  $('programPanel').classList.toggle('active',v==='program');
  $('sensorPanel').classList.toggle('active',v==='sensors');
  $('debugPanel').classList.toggle('active',v==='debug');
  $('settingsPanel').classList.toggle('active',v==='settings');
  ['tabControl','tabWorkspace','tabProgram','tabSensors','tabDebug','tabSettings'].forEach(id=>$(id).classList.remove('active'));
  $({control:'tabControl',workspace:'tabWorkspace',program:'tabProgram',sensors:'tabSensors',debug:'tabDebug',settings:'tabSettings'}[v]).classList.add('active');
  if(v==='debug')refreshDebug();
  if(v==='sensors')refreshSensor();
  if(v==='workspace'){if(!workspaceLoaded)loadWorkspace();else drawWorkspace()}
  if(v==='settings')refreshSettingsEndstops();
}
$('tabControl').addEventListener('click',()=>setView('control'));
$('tabWorkspace').addEventListener('click',()=>setView('workspace'));
$('tabProgram').addEventListener('click',()=>setView('program'));
$('tabSensors').addEventListener('click',()=>setView('sensors'));
$('tabDebug').addEventListener('click',()=>setView('debug'));
$('tabSettings').addEventListener('click',()=>setView('settings'));

async function sensorCommand(path,body){
  const result=await api(path,body||{});
  const sensor=result.sensor||result;
  renderSensor(sensor);
  return result;
}
for(const id of ['guardImpactStop','guardImpactFlash','guardZStop','guardZColor','guardSwipeColors','guardImpactG','guardZG','guardZHold'])$(id).addEventListener('input',()=>{$('guardApply').dataset.dirty='1'});
$('guardApply').addEventListener('click',async()=>{
  const options={impact_stop:$('guardImpactStop').checked,impact_flash:$('guardImpactFlash').checked,z_stop:$('guardZStop').checked,z_color:$('guardZColor').checked,swipe_colors:$('guardSwipeColors').checked,impact_g:Number($('guardImpactG').value),z_g:Number($('guardZG').value),z_persistence_s:Number($('guardZHold').value)};
  if((options.impact_stop||options.z_stop)&&!confirm('Enable sampled sensor KILL protection? A trigger stops Klipper and requires recovery/rehoming. Test LED feedback first; this is not a hardware safety interlock.'))return;
  try{await sensorCommand('/sensor_guard',options);delete $('guardApply').dataset.dirty}catch(e){alert(e.message)}
});
$('sensorCalibrate').addEventListener('click',async()=>{
  try{await sensorCommand('/sensor_calibrate',{})}catch(e){alert(e.message)}
});
$('sensorMonitorApply').addEventListener('click',async()=>{
  try{
    if($('sensorCorrectEnabled').checked && $('sensorKillEnabled').checked){
      throw new Error('Choose either tilt correction or KILL, not both.');
    }
    const recovery=$('sensorRecoveryProgram').value;
    const recoveryPlan=recovery
      ?('it will run the saved '+recovery+' Motor X/Y path while holding Motor Z and rail fixed')
      :('it will move 5 mm up, retract toward the base axis inside Boundary ON, then raise to the highest modeled safe height');
    if($('sensorCorrectEnabled').checked && !confirm('Enable automatic tilt correction? After a sustained tilt fault, the arm flashes red and '+recoveryPlan+'. Normal motion remains available only after every recovery stage completes; an interrupted or failed recovery requires Home. Keep the area clear; this is not a hardware safety interlock.'))return;
    await sensorCommand('/sensor_monitor',{
      allowed_deg:Number($('sensorTiltLimit').value),
      persistence_s:Number($('sensorPersistence').value),
      filter_s:Number($('sensorTiltFilter').value),
      enabled:$('sensorMonitorEnabled').checked,
      kill_enabled:$('sensorKillEnabled').checked,
      correct_enabled:$('sensorCorrectEnabled').checked
    });
  }catch(e){alert(e.message)}
});
$('sensorRecoverySave').addEventListener('click',async()=>{
  try{await sensorCommand('/sensor_recovery',{program:$('sensorRecoveryProgram').value})}catch(e){alert(e.message)}
});
$('sensorAck').addEventListener('click',async()=>{try{await sensorCommand('/sensor_ack',{})}catch(e){alert(e.message)}});
$('sensorLedSet').addEventListener('click',async()=>{try{await sensorCommand('/sensor_led',{color:$('sensorLedColor').value})}catch(e){alert(e.message)}});
$('sensorModeSet').addEventListener('click',async()=>{try{await sensorCommand('/sensor_mode',{mode:$('sensorMode').value})}catch(e){alert(e.message)}});
$('sensorTapSave').addEventListener('click',async()=>{
  try{
    const gestures=Object.fromEntries([...$('sensorTapRows').querySelectorAll('[data-sensor-program]')].map(el=>{
      const i=el.dataset.sensorProgram,color=$('sensorTapRows').querySelector('[data-sensor-color="'+i+'"]').value;
      return [i,{program:el.value,color}];
    }));
    await sensorCommand('/sensor_gestures',{gestures});
  }catch(e){alert(e.message)}
});
$('sensorSwipeEnable').addEventListener('click',async()=>{
  if(!confirm('Enable touch-triggered saved programs? A press or swipe can start robot motion 2 seconds after release when the robot is homed and idle.'))return;
  try{await sensorCommand('/sensor_swipe_enable',{enabled:true})}catch(e){alert(e.message)}
});
$('sensorSwipeDisable').addEventListener('click',async()=>{try{await sensorCommand('/sensor_swipe_enable',{enabled:false})}catch(e){alert(e.message)}});
setInterval(refreshSensor,250);

async function refreshSettingsEndstops(){
  try{
    const r=await api('/settings_endstops',{});
    const fmt=v=>v?'TRIGGERED':'open';
    $('settingsEndstopX').textContent=fmt(!!r.endstop_x);
    $('settingsEndstopY').textContent=fmt(!!r.endstop_y);
    $('settingsEndstopZ').textContent=fmt(!!r.endstop_z);
    if($('settingsEndstopD'))$('settingsEndstopD').textContent=S.linear_hardware
      ?(r.d===null||r.d===undefined?'—':fmt(!!r.d)):'DISABLED';
  }catch(e){
    $('settingsEndstopX').textContent='error';
    $('settingsEndstopY').textContent='error';
    $('settingsEndstopZ').textContent='error';
  }
}
function updateMotorSettings(r){
  if(!r || r.motors_enabled===null || r.motors_enabled===undefined){
    $('motorStateBadge').textContent='UNKNOWN';
    $('motorStateBadge').className='badge';
  }else if(r.motors_enabled){
    $('motorStateBadge').textContent='ENABLED';
    $('motorStateBadge').className='badge good';
  }else{
    $('motorStateBadge').textContent='DISABLED';
    $('motorStateBadge').className='badge bad';
  }
}
$('settingsRefreshEndstops').addEventListener('click',refreshSettingsEndstops);
async function setLinearMode(enabled){
  try{
    const r=await api('/linear_mode',{enabled:Boolean(enabled)});
    rs(r);
    workspaceLoaded=false;workspaceData=null;worldPlan=null;
    if(activeView==='workspace')await loadWorkspace();
    await refreshSettingsEndstops();
  }catch(e){alert(e.message)}
}
$('linearModeEnable').addEventListener('click',()=>setLinearMode(true));
$('linearModeDisable').addEventListener('click',()=>setLinearMode(false));
$('motorsEnable').addEventListener('click',async()=>{try{const r=await api('/motors_enable',{});updateMotorSettings(r);rs(r)}catch(e){alert(e.message)}});
$('motorsDisable').addEventListener('click',async()=>{try{const r=await api('/motors_disable',{});updateMotorSettings(r);rs(r)}catch(e){alert(e.message)}});
$('settingsHome').addEventListener('click',homeRobot);
$('settingsFirmwareRestart').addEventListener('click',async()=>{if(!confirm('Restart Klipper firmware connection? The robot will require homing again.'))return;try{const r=await api('/firmware_restart',{});rs(r)}catch(e){alert(e.message)}});

$('debugMode').addEventListener('change',async()=>{try{await api('/debug_mode',{mode:$('debugMode').value});await refreshDebug()}catch(e){alert(e.message)}});
$('debugWindow').addEventListener('change',async()=>{try{debugWindowSeconds=Number($('debugWindow').value);await api('/debug_window',{seconds:debugWindowSeconds});UI_RTT.splice(0,UI_RTT.length);await refreshDebug()}catch(e){alert(e.message)}});
$('debugReset').addEventListener('click',async()=>{try{UI_RTT.splice(0,UI_RTT.length);await api('/debug_reset',{});await refreshDebug()}catch(e){alert(e.message)}});

function uiRttStats(){
  const cutoff=Date.now()/1000-debugWindowSeconds;
  const v=UI_RTT.filter(x=>x[0]>=cutoff).map(x=>x[1]).sort((a,b)=>a-b);
  if(!v.length)return{};
  const q=p=>v[Math.min(v.length-1,Math.floor((v.length-1)*p))];
  return{median:q(.5),p95:q(.95),p99:q(.99),max:v[v.length-1],count:v.length};
}
function fmt(v,d=2){return Number.isFinite(Number(v))?Number(v).toFixed(d):'—'}
function drawSeries(canvas,seriesList){
  const ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height;ctx.clearRect(0,0,w,h);ctx.fillStyle='#090c10';ctx.fillRect(0,0,w,h);
  let all=[];seriesList.forEach(s=>(s.data||[]).forEach(p=>all.push(p)));
  if(!all.length){ctx.fillStyle='#8d98a4';ctx.fillText('No samples in selected window',12,20);return}
  let t0=Math.min(...all.map(p=>p[0])),t1=Math.max(...all.map(p=>p[0]));if(t1<=t0)t1=t0+1;
  let vals=all.map(p=>p[1]).filter(Number.isFinite),v0=Math.min(0,...vals),v1=Math.max(...vals);if(v1<=v0)v1=v0+1;
  ctx.strokeStyle='#28323c';for(let i=0;i<5;i++){let y=10+(h-30)*i/4;ctx.beginPath();ctx.moveTo(42,y);ctx.lineTo(w-8,y);ctx.stroke()}
  const pal=['#52a9e8','#6fd49b','#e8bc59','#e78aa7','#b7a0ff'];
  seriesList.forEach((s,si)=>{ctx.strokeStyle=pal[si%pal.length];ctx.beginPath();let first=true;for(const [t,v] of s.data||[]){let x=42+(w-52)*(t-t0)/(t1-t0),y=10+(h-30)*(1-(v-v0)/(v1-v0));if(first){ctx.moveTo(x,y);first=false}else ctx.lineTo(x,y)}ctx.stroke();ctx.fillStyle=pal[si%pal.length];ctx.fillText(s.label,48+si*145,h-5)});
  ctx.fillStyle='#8d98a4';ctx.fillText(fmt(v1),4,16);ctx.fillText(fmt(v0),4,h-22);
}
async function refreshDebug(){
  if(activeView!=='debug')return;
  try{
    const d=await api('/debug');debugWindowSeconds=Number(d.window_s||300);$('debugMode').value=d.mode||'BASIC';$('debugWindow').value=String(debugWindowSeconds);
    $('debugWindowLabel').textContent='Rolling '+({60:'1 min',180:'3 min',300:'5 min',600:'10 min',1800:'30 min',3600:'1 hour'}[debugWindowSeconds]||debugWindowSeconds+' s')+' · milliseconds';
    const st=d.stats||{},sr=d.series||{},names=['latency.http_handler','latency.moonraker_get','latency.moonraker_post','latency.gcode_submit'];
    const ur=uiRttStats();
    $('latencyRows').innerHTML='<tr><td>browser command RTT</td><td>'+fmt(ur.median)+'</td><td>'+fmt(ur.p95)+'</td><td>'+fmt(ur.p99)+'</td><td>'+fmt(ur.max)+'</td><td>'+fmt(ur.count,0)+'</td></tr>'+names.map(n=>{const x=st[n]||{};return '<tr><td>'+n.replace('latency.','')+'</td><td>'+fmt(x.median)+'</td><td>'+fmt(x.p95)+'</td><td>'+fmt(x.p99)+'</td><td>'+fmt(x.max)+'</td><td>'+fmt(x.count,0)+'</td></tr>'}).join('');
    drawSeries($('latencyGraph'),names.map(n=>({label:n.replace('latency.',''),data:sr[n]||[]})));
    const q=st['klipper.buffer_time_s']||st['arm.queue_ahead_s']||{},rtt=st['mcu.srtt_ms']||{},rtv=st['mcu.rttvar_ms']||{},ret=st['mcu.retransmit_bytes']||{},stalls=st['klipper.print_stalls']||{},dt=st['arm.dance_sample_dt_s']||{},as=st['motion.A_speed']||{},bs=st['motion.B_speed']||{},cs=st['motion.C_speed']||{},ds=st['motion.D_speed']||{};
    $('motionRows').innerHTML='<tr><td>actual Klipper buffer</td><td>'+fmt(q.current,3)+' s</td></tr><tr><td>Dance sample Δt</td><td>'+fmt((dt.current||0)*1000,2)+' ms</td></tr><tr><td>MCU SRTT</td><td>'+fmt(rtt.current,3)+' ms</td></tr><tr><td>RTT variation</td><td>'+fmt(rtv.current,3)+' ms</td></tr><tr><td>retransmit bytes</td><td>'+fmt(ret.current,0)+'</td></tr><tr><td>print stalls</td><td>'+fmt(stalls.current,0)+'</td></tr><tr><td>Motor X/Y/Z · rail speed</td><td>'+fmt(as.current,1)+' / '+fmt(bs.current,1)+' / '+fmt(cs.current,1)+' / '+fmt(ds.current,1)+'</td></tr>';
    drawSeries($('queueGraph'),[{label:'buffer s',data:sr['klipper.buffer_time_s']||sr['arm.queue_ahead_s']||[]},{label:'MCU RTT ms',data:sr['mcu.srtt_ms']||[]}]);
    const load=st['system.load']||{},mem=st['system.mem_available_kb']||{},jit=st['diagnostics.loop_jitter_ms']||{};
    $('systemRows').innerHTML='<tr><td>system load</td><td>'+fmt(load.current,2)+'</td></tr><tr><td>available RAM</td><td>'+fmt((mem.current||0)/1024,1)+' MB</td></tr><tr><td>diagnostic loop jitter</td><td>'+fmt(jit.p99,3)+' ms p99</td></tr>';
    drawSeries($('systemGraph'),[{label:'load',data:sr['system.load']||[]},{label:'loop jitter ms',data:sr['diagnostics.loop_jitter_ms']||[]}]);
    $('debugEvents').textContent=(d.events||[]).slice().reverse().map(e=>new Date(e.t*1000).toLocaleTimeString()+'  '+e.name+'  '+JSON.stringify(e.fields||{})).join('\n')||'No trace events in this run.';
  }catch(e){$('debugEvents').textContent='Debug error: '+e.message}
}
setInterval(()=>{if(activeView==='debug')refreshDebug()},2000);

function currentWorldTool(){
  const k=S.kinematics&&S.kinematics.tool?S.kinematics.tool:null;
  if(!k)return null;
  return{x:Number(k.x),y:Number(k.y)+Number(S.linear_y||0),z:Number(k.z)};
}

let workspaceDragging=false,workspaceMoved=false,workspaceLastX=0,workspaceLastY=0;
let workspaceCam={yaw:-0.72,pitch:0.48,zoom:1.0,focus:null,span:null};
let sliceViews={xy:{zoom:1,pu:0,pv:0},xz:{zoom:1,pu:0,pv:0},yz:{zoom:1,pu:0,pv:0}};
let sliceTransforms={},workspaceSliceCache=new Map();

function currentWorldTool(){
  const k=S.kinematics&&S.kinematics.tool?S.kinematics.tool:null;
  if(!k)return null;
  return{x:Number(k.x),y:Number(k.y)+Number(S.linear_y||0),z:Number(k.z)};
}
function setWorldXYZ(x,y,z,preview=false){
  x=Number(x);y=Number(y);z=Number(z);
  $('worldX').value=x.toFixed(1);$('worldY').value=y.toFixed(1);$('worldZ').value=z.toFixed(1);
  $('sliceX').value=x;$('sliceY').value=y;$('sliceZ').value=z;
  $('sliceXVal').textContent='X = '+fmt(x,1);$('sliceYVal').textContent='Y = '+fmt(y,1);$('sliceZVal').textContent='Z = '+fmt(z,1);
  selectedWorld={x,y};worldPlan=null;$('worldMove').disabled=true;drawWorkspace();
  if(preview)previewWorld();
}
function setWorldCurrent(){
  const k=currentWorldTool();if(!k)return;
  $('worldD').value=Number(S.linear_y||0).toFixed(1);
  setWorldXYZ(k.x,k.y,k.z,false);
  $('worldPlanSummary').textContent='Current pose loaded. Preview to verify.';
}
$('worldCurrent').addEventListener('click',setWorldCurrent);
$('worldClearTarget').addEventListener('click',()=>{setWorldXYZ(0,0,0,false);$('worldPlanSummary').textContent='Target reset to 0, 0, 0.'});
$('railMode').addEventListener('change',()=>{$('manualRailLabel').style.display=$('railMode').value==='manual'?'inline-flex':'none';worldPlan=null;$('worldMove').disabled=true;drawWorkspace()});
for(const id of ['worldX','worldY','worldZ'])$(id).addEventListener('change',()=>setWorldXYZ($('worldX').value,$('worldY').value,$('worldZ').value,false));

function wb(){const b=workspaceData.bounds;return{x0:+b.x[0],x1:+b.x[1],y0:+b.y[0],y1:+b.y[1],z0:+b.z[0],z1:+b.z[1]}}
function clearC(c){const x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);x.fillStyle='#090c10';x.fillRect(0,0,c.width,c.height);return x}
function marker(ctx,x,y,label,color){
  ctx.fillStyle='rgba(9,12,16,.92)';ctx.beginPath();ctx.arc(x,y,10,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,7,0,Math.PI*2);ctx.stroke();
  ctx.fillStyle=color;ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();
  ctx.font='600 12px system-ui';const w=ctx.measureText(label).width;
  ctx.fillStyle='rgba(9,12,16,.9)';ctx.fillRect(x+10,y-19,w+8,17);
  ctx.fillStyle=color;ctx.fillText(label,x+14,y-7);
}
function line2(ctx,x0,y0,x1,y1,color,w=1){ctx.strokeStyle=color;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(x0,y0);ctx.lineTo(x1,y1);ctx.stroke()}
function spec(mode){
  const b=wb();
  if(mode==='xy')return{fixed:'z',u:'x',v:'y',u0:b.x0,u1:b.x1,v0:b.y0,v1:b.y1,flip:true,f:+$('sliceZ').value};
  if(mode==='xz')return{fixed:'y',u:'x',v:'z',u0:b.x0,u1:b.x1,v0:b.z0,v1:b.z1,flip:true,f:+$('sliceY').value};
  return{fixed:'x',u:'y',v:'z',u0:b.y0,u1:b.y1,v0:b.z0,v1:b.z1,flip:false,f:+$('sliceX').value};
}
function pc(p,n){return n==='x'?+p[0]:n==='y'?+p[1]:+p[2]}
function transform(c,s,v){
  const pad=28,base=Math.min((c.width-2*pad)/(s.u1-s.u0||1),(c.height-2*pad)/(s.v1-s.v0||1)),scale=base*v.zoom;
  const um=(s.u0+s.u1)/2,vm=(s.v0+s.v1)/2,su=s.flip?-1:1,ox=c.width/2+v.pu,oy=c.height/2+v.pv;
  return{U:u=>ox+su*(u-um)*scale,V:q=>oy-(q-vm)*scale,iU:x=>um+(x-ox)/(su*scale),iV:y=>vm-(y-oy)/scale,scale};
}
function fullWorkspaceSlicePoints(s){
  const arm=workspaceData.side_samples||[];
  const meta=workspaceData.meta||{};
  const cr=meta.c_range||[2.5,272.0],dr=meta.d_range||[0,235];
  const cacheKey=s.fixed+':'+s.f.toFixed(2)+':'+Boolean(meta.linear_enabled);
  if(workspaceSliceCache.has(cacheKey))return workspaceSliceCache.get(cacheKey);
  // Rail-off slices are a continuous base sweep.  The old 15-degree sampling
  // made that sweep look like disconnected radial spokes.
  const cStep=meta.linear_enabled?15:1.5;
  const cs=[];for(let c=cr[0];c<=cr[1]+1e-9;c+=cStep)cs.push(c);if(!cs.length||cs[cs.length-1]<cr[1])cs.push(cr[1]);
  const ds=[];for(let d=dr[0];d<=dr[1]+1e-9;d+=25)ds.push(d);if(!ds.length||ds[ds.length-1]<dr[1])ds.push(dr[1]);

  // Recreate the full modeled workspace from every safe side-plane arm pose,
  // every measured base sample and every rail sample.  This avoids the
  // aliasing caused by the 12k-point 3D display downsample.
  const tol=6.0,out=[];
  for(const c of cs){
    const th=(c-2.5)*Math.PI/180,sn=Math.sin(th),co=Math.cos(th);
    for(const d of ds){
      for(const q of arm){
        const r=+q[0],z=+q[1],p={x:r*sn,y:d+r*co,z};
        if(Math.abs(p[s.fixed]-s.f)<=tol)out.push([p.x,p.y,p.z]);
      }
    }
  }
  workspaceSliceCache.set(cacheKey,out);return out;
}
function fillSlice(ctx,pts,s,t){
  // Build a hidden occupancy mask, then trace its exposed edges into closed
  // contour loops.  The grid is computational only; no cells are drawn.
  const cell=6,occ=new Set();
  for(const p of pts)occ.add(Math.floor(pc(p,s.u)/cell)+','+Math.floor(pc(p,s.v)/cell));

  // Close tiny sampling pinholes created by discrete C/D sampling.
  for(let pass=0;pass<2;pass++){
    const add=[];
    for(const k of occ){
      const [i,j]=k.split(',').map(Number);
      for(const [ni,nj] of [[i-1,j],[i+1,j],[i,j-1],[i,j+1]]){
        const nk=ni+','+nj;if(occ.has(nk))continue;
        const horiz=occ.has((ni-1)+','+nj)&&occ.has((ni+1)+','+nj);
        const vert=occ.has(ni+','+(nj-1))&&occ.has(ni+','+(nj+1));
        if(horiz||vert)add.push(nk);
      }
    }
    add.forEach(k=>occ.add(k));
  }

  const edges=[];
  const addEdge=(x0,y0,x1,y1)=>edges.push({a:[x0,y0],b:[x1,y1],used:false});
  for(const k of occ){
    const [i,j]=k.split(',').map(Number);
    // Directed CCW around occupied cells.
    if(!occ.has(i+','+(j-1)))addEdge(i,j,i+1,j);
    if(!occ.has((i+1)+','+j))addEdge(i+1,j,i+1,j+1);
    if(!occ.has(i+','+(j+1)))addEdge(i+1,j+1,i,j+1);
    if(!occ.has((i-1)+','+j))addEdge(i,j+1,i,j);
  }

  const key=p=>p[0]+','+p[1],from=new Map();
  edges.forEach((e,idx)=>{
    const k=key(e.a);
    if(!from.has(k))from.set(k,[]);
    from.get(k).push(idx);
  });

  const loops=[];
  for(let seed=0;seed<edges.length;seed++){
    if(edges[seed].used)continue;
    const loop=[];
    let idx=seed,start=key(edges[idx].a),guard=0;
    while(idx!=null && guard++<edges.length+10){
      const e=edges[idx];
      if(e.used)break;
      e.used=true;
      if(!loop.length)loop.push(e.a);
      loop.push(e.b);
      const bk=key(e.b);
      if(bk===start)break;
      const cand=(from.get(bk)||[]).find(j=>!edges[j].used);
      idx=(cand===undefined)?null:cand;
    }
    if(loop.length>=4 && key(loop[0])===key(loop[loop.length-1]))loops.push(loop.slice(0,-1));
  }

  function smoothClosed(poly,passes=2){
    let out=poly.map(p=>[p[0]*cell,p[1]*cell]);
    for(let n=0;n<passes;n++){
      const q=[];
      for(let i=0;i<out.length;i++){
        const a=out[i],b=out[(i+1)%out.length];
        q.push([.75*a[0]+.25*b[0],.75*a[1]+.25*b[1]]);
        q.push([.25*a[0]+.75*b[0],.25*a[1]+.75*b[1]]);
      }
      out=q;
    }
    return out;
  }

  const contours=loops.map(l=>smoothClosed(l,2)).filter(l=>l.length>=6);

  // Fill all components and holes as one even-odd compound path.
  ctx.beginPath();
  for(const poly of contours){
    ctx.moveTo(t.U(poly[0][0]),t.V(poly[0][1]));
    for(let i=1;i<poly.length;i++)ctx.lineTo(t.U(poly[i][0]),t.V(poly[i][1]));
    ctx.closePath();
  }
  ctx.fillStyle='rgba(45,145,205,.26)';
  if(contours.length)ctx.fill('evenodd');

  // Clean outer/inner contour lines only — no voxel borders.
  ctx.strokeStyle='rgba(82,169,232,.96)';
  ctx.lineWidth=1.8;
  ctx.lineJoin='round';
  ctx.lineCap='round';
  for(const poly of contours){
    ctx.beginPath();
    ctx.moveTo(t.U(poly[0][0]),t.V(poly[0][1]));
    for(let i=1;i<poly.length;i++)ctx.lineTo(t.U(poly[i][0]),t.V(poly[i][1]));
    ctx.closePath();
    ctx.stroke();
  }
}
function drawSlice(mode){
  if(!workspaceData)return;
  const c=$(mode==='xy'?'wsXYCanvas':mode==='xz'?'wsXZCanvas':'wsYZCanvas'),ctx=clearC(c),s=spec(mode),v=sliceViews[mode],t=transform(c,s,v);sliceTransforms[mode]=t;
  fillSlice(ctx,fullWorkspaceSlicePoints(s),s,t);
  line2(ctx,0,t.V(0),c.width,t.V(0),'#26313b');line2(ctx,t.U(0),0,t.U(0),c.height,'#26313b');
  const cur=currentWorldTool();if(cur)marker(ctx,t.U(cur[s.u]),t.V(cur[s.v]),'current','#eef2f5');
  const q={x:+$('worldX').value,y:+$('worldY').value,z:+$('worldZ').value};marker(ctx,t.U(q[s.u]),t.V(q[s.v]),'target',worldPlan?'#6fd49b':'#e8bc59');
  ctx.fillStyle='#eef2f5';ctx.font='12px system-ui';ctx.fillText(mode.toUpperCase()+' @ '+s.fixed.toUpperCase()+' = '+fmt(s.f,1),10,17);
  ctx.fillStyle='#8d98a4';ctx.font='9px system-ui';ctx.fillText('blue = modeled reachable slice',10,31);
}
function draw3D(){
  if(!workspaceData)return;
  const c=$('workspaceCanvas'),ctx=clearC(c),b=wb(),focus=workspaceCam.focus,
    cx=focus?focus.x:(b.x0+b.x1)/2,cy=focus?focus.y:(b.y0+b.y1)/2,cz=focus?focus.z:(b.z0+b.z1)/2,
    span=workspaceCam.span||Math.max(b.x1-b.x0,b.y1-b.y0,b.z1-b.z0,1);
  const sc=Math.min(c.width,c.height)*.90/span*workspaceCam.zoom,ca=Math.cos(workspaceCam.yaw),sa=Math.sin(workspaceCam.yaw),cp=Math.cos(workspaceCam.pitch),sp=Math.sin(workspaceCam.pitch);
  const P=(x,y,z)=>{const dx=-(x-cx),dy=y-cy,dz=z-cz,xr=dx*ca-dy*sa,yr=dx*sa+dy*ca,depth=dz*sp+yr*cp;return[c.width/2+xr*sc,c.height/2-(dz*cp-yr*sp)*sc,depth]};

  // The workspace data is a point cloud, but drawing its individual samples
  // made the actual volume nearly invisible.  Merge nearby screen samples
  // into soft, depth-shaded cells so the reachable shape reads as a volume.
  const cell=3,bins=new Map(),pts=workspaceData.points||[];
  for(const point of pts){
    const q=P(+point[0],+point[1],+point[2]);
    if(q[0]<-2||q[0]>c.width+2||q[1]<-2||q[1]>c.height+2)continue;
    const ix=Math.floor(q[0]/cell),iy=Math.floor(q[1]/cell),key=ix+','+iy;
    const bin=bins.get(key)||{ix,iy,count:0,depth:0};
    bin.count++;bin.depth+=q[2];bins.set(key,bin);
  }
  for(const bin of bins.values()){
    const density=Math.min(1,Math.log2(bin.count+1)/2.5);
    const near=Math.max(0,Math.min(1,.5+(bin.depth/bin.count)/(span*.75)));
    const alpha=.20+.66*density;
    const blue=Math.round(178+42*near),green=Math.round(120+54*near);
    ctx.fillStyle=`rgba(37,${green},${blue},${alpha.toFixed(3)})`;
    ctx.fillRect(bin.ix*cell,bin.iy*cell,cell+0.6,cell+0.6);
  }
  if(workspaceData.meta&&workspaceData.meta.linear_enabled){const a=P(0,0,0),b1=P(0,235,0);line2(ctx,a[0],a[1],b1[0],b1[1],'#e8bc59',2)}
  const cur=currentWorldTool();if(cur){const q=P(cur.x,cur.y,cur.z);marker(ctx,q[0],q[1],'current','#eef2f5')}
  const q=P(+$('worldX').value,+$('worldY').value,+$('worldZ').value);marker(ctx,q[0],q[1],'target',worldPlan?'#6fd49b':'#e8bc59');
  ctx.fillStyle='rgba(9,12,16,.90)';ctx.fillRect(9,9,focus?238:208,36);
  ctx.fillStyle='#eef2f5';ctx.font='600 11px system-ui';ctx.fillText(focus?'FOCUSED MOVE VOLUME':'FULL REACHABLE VOLUME',16,24);
  ctx.fillStyle='#aeb9c4';ctx.font='10px system-ui';ctx.fillText('bright blue = denser reachable space',16,38);
}
function drawWorkspace(){drawSlice('xy');drawSlice('xz');drawSlice('yz');draw3D()}

for(const [id,axis] of [['sliceX','x'],['sliceY','y'],['sliceZ','z']])$(id).addEventListener('input',()=>{
  const v=+$(id).value;$(axis==='x'?'worldX':axis==='y'?'worldY':'worldZ').value=v.toFixed(1);$(id+'Val').textContent=axis.toUpperCase()+' = '+fmt(v,1);
  selectedWorld={x:+$('worldX').value,y:+$('worldY').value};worldPlan=null;$('worldMove').disabled=true;drawWorkspace();
});
function bindSlice(mode,id){
  const c=$(id),v=sliceViews[mode];let drag=false,moved=false,lx=0,ly=0;
  c.addEventListener('pointerdown',e=>{drag=true;moved=false;lx=e.clientX;ly=e.clientY;c.setPointerCapture(e.pointerId)});
  c.addEventListener('pointermove',e=>{if(!drag)return;const dx=e.clientX-lx,dy=e.clientY-ly;if(Math.abs(dx)+Math.abs(dy)>3)moved=true;if(moved){v.pu+=dx*c.width/c.clientWidth;v.pv+=dy*c.height/c.clientHeight;drawSlice(mode)}lx=e.clientX;ly=e.clientY});
  c.addEventListener('pointerup',async e=>{if(!drag)return;drag=false;try{c.releasePointerCapture(e.pointerId)}catch(_){}
    if(moved)return;const r=c.getBoundingClientRect(),px=(e.clientX-r.left)*c.width/r.width,py=(e.clientY-r.top)*c.height/r.height,t=sliceTransforms[mode],u=t.iU(px),w=t.iV(py);
    let x=+$('worldX').value,y=+$('worldY').value,z=+$('worldZ').value;if(mode==='xy'){x=u;y=w}else if(mode==='xz'){x=u;z=w}else{y=u;z=w}setWorldXYZ(x,y,z,false);await previewWorld();await moveWorldTarget();
  });
  c.addEventListener('wheel',e=>{e.preventDefault();v.zoom=Math.max(.5,Math.min(4,v.zoom*Math.exp(-e.deltaY*.0012)));drawSlice(mode)},{passive:false});
  c.addEventListener('dblclick',()=>{v.zoom=1;v.pu=0;v.pv=0;drawSlice(mode)});
}
bindSlice('xy','wsXYCanvas');bindSlice('xz','wsXZCanvas');bindSlice('yz','wsYZCanvas');

const wsC=$('workspaceCanvas');
wsC.addEventListener('pointerdown',e=>{workspaceDragging=true;workspaceMoved=false;workspaceLastX=e.clientX;workspaceLastY=e.clientY;wsC.setPointerCapture(e.pointerId)});
wsC.addEventListener('pointermove',e=>{if(!workspaceDragging)return;const dx=e.clientX-workspaceLastX,dy=e.clientY-workspaceLastY;if(Math.abs(dx)+Math.abs(dy)>2)workspaceMoved=true;workspaceCam.yaw+=dx*.008;workspaceCam.pitch=Math.max(-1.35,Math.min(1.35,workspaceCam.pitch+dy*.008));workspaceLastX=e.clientX;workspaceLastY=e.clientY;draw3D()});
wsC.addEventListener('pointerup',e=>{workspaceDragging=false;try{wsC.releasePointerCapture(e.pointerId)}catch(_){}});
wsC.addEventListener('wheel',e=>{e.preventDefault();workspaceCam.zoom=Math.max(.45,Math.min(3.5,workspaceCam.zoom*Math.exp(-e.deltaY*.0012)));draw3D()},{passive:false});
$('wsFocusView').addEventListener('click',()=>{
  const target={x:+$('worldX').value,y:+$('worldY').value,z:+$('worldZ').value},cur=currentWorldTool();
  const points=cur?[cur,target]:[target];
  const focus={x:points.reduce((n,p)=>n+p.x,0)/points.length,y:points.reduce((n,p)=>n+p.y,0)/points.length,z:points.reduce((n,p)=>n+p.z,0)/points.length};
  const spread=Math.max(80,...points.map(p=>Math.hypot(p.x-focus.x,p.y-focus.y,p.z-focus.z)*2));
  workspaceCam={...workspaceCam,focus,span:Math.min(260,spread*1.8),zoom:1};draw3D();
});
$('wsResetView').addEventListener('click',()=>{workspaceCam={yaw:-.72,pitch:.48,zoom:1,focus:null,span:null};draw3D()});

async function previewWorld(){
  if(!S.homed){$('worldPlanSummary').innerHTML='<span class="bad">Home the robot first.</span>';return null}
  const x=+$('worldX').value,y=+$('worldY').value,z=+$('worldZ').value;selectedWorld={x,y};worldPlan=null;$('worldMove').disabled=true;$('worldPlanSummary').textContent='Solving…';drawWorkspace();
  const body={x,y,z};
  if(!S.linear_hardware)body.linear_y=0;
  else if($('railMode').value==='manual')body.linear_y=+$('worldD').value;
  try{
    worldPlan=await api('/plan_world',body);
    $('worldD').value=Number(worldPlan.D).toFixed(1);
    const railText=S.linear_hardware?('Rail '+fmt(worldPlan.D,1)+' mm'):'Rail disabled · 0 mm';
    $('worldPlanSummary').innerHTML=worldPlan.direct
      ?'<span class="good"><b>Boundary OFF — direct move</b></span><br>World X/Y/Z '+fmt(x,1)+', '+fmt(y,1)+', '+fmt(z,1)+' mm<br>'+railText+' · Klipper resolves the real arm geometry when you move.'
      :'<span class="good"><b>'+((worldPlan.route_points||2)>2?'Safe corridor ready':'Direct safe path')+'</b></span><br>World X/Y/Z '+fmt(x,1)+', '+fmt(y,1)+', '+fmt(z,1)+' mm<br>'+railText+' · Motor Z θ '+fmt(worldPlan.C,1)+'°<br>Motor X/Y θ '+fmt(worldPlan.A,1)+'° / '+fmt(worldPlan.B,1)+'°'+((worldPlan.route_points||2)>2?'<br>'+((worldPlan.route_points||2)-2)+' blended boundary corner'+((worldPlan.route_points||2)===3?'':'s'):'');
    $('worldMove').disabled=false;drawWorkspace();return worldPlan
  }catch(e){worldPlan=null;$('worldPlanSummary').innerHTML='<span class="bad"><b>Not reachable:</b> '+e.message+'</span>';drawWorkspace();return null}
}
$('worldPreview').addEventListener('click',previewWorld);
async function moveWorldTarget(){
  if(!worldPlan||worldMoveInFlight)return false;
  worldMoveInFlight=true;
  const body={x:+$('worldX').value,y:+$('worldY').value,z:+$('worldZ').value,auto_d:Boolean(S.linear_hardware&&$('railMode').value==='auto')};
  if(!S.linear_hardware)body.linear_y=0;
  else if($('railMode').value==='manual')body.linear_y=+$('worldD').value;
  try{
    rs(await api('/goto_world',body));
    worldPlan=null;$('worldMove').disabled=true;
    $('worldPlanSummary').innerHTML='<span class="good"><b>Move queued.</b></span>';
    drawWorkspace();
    return true;
  }catch(e){
    $('worldPlanSummary').innerHTML='<span class="bad">'+e.message+'</span>';
    return false;
  }finally{worldMoveInFlight=false}
}
$('worldMove').addEventListener('click',moveWorldTarget);
document.querySelectorAll('[data-cjog]').forEach(btn=>btn.addEventListener('click',async()=>{const [a,s]=btn.dataset.cjog.split(','),step=+$('cartStep').value*+s,id=a==='x'?'worldX':a==='y'?'worldY':'worldZ';$(id).value=(+$(id).value+step).toFixed(1);setWorldXYZ($('worldX').value,$('worldY').value,$('worldZ').value,false);await previewWorld()}));

async function loadWorkspace(){
  workspaceLoaded=true;workspaceSliceCache.clear();
  try{workspaceData=await api('/workspace_map');const b=wb(),m=workspaceData.meta||{};$('sliceX').min=Math.floor(b.x0);$('sliceX').max=Math.ceil(b.x1);$('sliceY').min=Math.floor(b.y0);$('sliceY').max=Math.ceil(b.y1);$('sliceZ').min=Math.floor(b.z0);$('sliceZ').max=Math.ceil(b.z1);setWorldXYZ(0,0,0,false);$('workspaceMeta').textContent='Model: '+m.ab_samples+' safe arm poses · '+m.total_samples.toLocaleString()+' world samples · '+(m.linear_enabled?'linear rail 0–235 mm':'rail disabled · D=0');drawWorkspace()}catch(e){$('workspaceMeta').textContent='Workspace error: '+e.message;workspaceLoaded=false}
}
setInterval(()=>{if(activeView==='workspace'&&workspaceLoaded)drawWorkspace()},700);
</script>'''

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def reply(self,obj,code=200):
        data=json.dumps(obj).encode(); self.send_response(code); self.send_header("Content-Type","application/json"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
        if (hasattr(self,"_request_started") and self.command=="POST"
                and self.path not in ("/debug_mode","/debug_window","/debug_reset","/plan_world")):
            DIAG.latency("http_handler",time.perf_counter()-self._request_started,{"path":self.path,"code":code})
    def do_GET(self):
        self._request_started=time.perf_counter()
        if self.path=='/controller-stream':
            # Keep telemetry on the same private origin as UI and camera.
            # Reject browser connections initiated from other websites.
            origin=urllib.parse.urlparse(self.headers.get('Origin',''))
            if origin.netloc != self.headers.get('Host') or self.headers.get('Upgrade','').lower()!='websocket':
                self.send_error(403)
                return
            upstream=None
            try:
                upstream=socket.create_connection(('127.0.0.1',7125),timeout=5)
                headers=['GET /websocket HTTP/1.1','Host: 127.0.0.1:7125',
                         'Upgrade: websocket','Connection: Upgrade']
                for key in ('Sec-WebSocket-Key','Sec-WebSocket-Version','Sec-WebSocket-Protocol'):
                    value=self.headers.get(key)
                    if value:
                        headers.append(key+': '+value)
                upstream.sendall(('\r\n'.join(headers)+'\r\n\r\n').encode())
                upstream.settimeout(None)
                self.connection.settimeout(None)
                while True:
                    readable,_,_=select.select([self.connection,upstream],[],[],60)
                    if not readable:
                        continue
                    for source in readable:
                        data=source.recv(65536)
                        if not data:
                            return
                        (upstream if source is self.connection else self.connection).sendall(data)
            except OSError:
                pass
            finally:
                if upstream:
                    upstream.close()
                self.close_connection=True
            return
        if self.path.startswith('/camera/') and not FEATURES['camera']:
            self.send_error(404, 'Camera feature disabled')
            return
        if self.path.startswith('/camera/stream'):
            CAMERA.stream(self)
            return
        if self.path=='/camera/status':
            self.reply({'error':CAMERA.error,'viewers':CAMERA.viewers})
            return
        if self.path=="/":
            page=HTML.replace("__PAGE_TOKEN__",PAGE_TOKEN)
            if not FEATURES['camera']:
                page=page.replace('<button id="cameraToggle"', '<button hidden id="cameraToggle"')
            if not FEATURES['rail']:
                page=page.replace('<button id="linearModeEnable"', '<button disabled id="linearModeEnable"')
            if not FEATURES['sensors']:
                page=page.replace('<button id="tabSensors"', '<button hidden id="tabSensors"')
            data=page.encode(); self.send_response(200); self.send_header("Content-Type","text/html; charset=utf-8"); self.send_header("Cache-Control","no-store, no-cache, must-revalidate, max-age=0"); self.send_header("Pragma","no-cache"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
        elif self.path=="/state": self.reply(state_payload())
        elif self.path=="/sensor":
            if not FEATURES["sensors"] or SENSOR is None:
                self.reply({"enabled":False},404)
            else:
                self.reply(SENSOR.snapshot())
        elif self.path=="/live_motion": self.reply(live_motion_payload())
        elif self.path=="/debug": self.reply(DIAG.snapshot())
        elif self.path=="/workspace_map": self.reply(workspace_payload_for_mode())
        elif self.path=="/endstops":
            # Retire the unauthenticated legacy endpoint. In particular, an
            # old browser page must not be able to poll QUERY_ENDSTOPS while a
            # new page is homing the robot.
            self.send_error(410, "Use the Control endstop strip to refresh")
        elif self.path=="/config":
            with lock:
                lines=read_config()
            self.reply({"lines":lines})
        else: self.send_error(404)
    def do_POST(self):
        self._request_started=time.perf_counter()
        global dance_thread, home_in_progress, program_thread, program_defaults, program_active_name
        n=int(self.headers.get("Content-Length","0")); body=json.loads(self.rfile.read(n) or b"{}")

        # Reject commands from stale/cached browser instances. STOP remains
        # intentionally tokenless so any tab can always stop motion.
        if self.path not in ("/stop","/kill") and body.get("_token") != PAGE_TOKEN:
            self.reply({"error":"Stale control page — refresh before commanding motion"},409)
            return

        if self.path.startswith("/sensor_"):
            if not FEATURES["sensors"] or SENSOR is None:
                self.reply({"error":"Sensor feature disabled or unavailable"},404)
                return
            try:
                if self.path=="/sensor_calibrate":
                    result=SENSOR.calibrate()
                    self.reply({"calibration":result,"sensor":SENSOR.snapshot()})
                elif self.path=="/sensor_monitor":
                    self.reply(SENSOR.configure_monitor(
                        allowed_deg=body.get("allowed_deg"),
                        persistence_s=body.get("persistence_s"),
                        filter_s=body.get("filter_s"),
                        enabled=body.get("enabled"),
                        kill_enabled=body.get("kill_enabled"),
                        correct_enabled=body.get("correct_enabled"),
                    ))
                elif self.path=="/sensor_recovery":
                    self.reply(SENSOR.set_recovery_program(body.get("program", "")))
                elif self.path=="/sensor_ack":
                    self.reply(SENSOR.acknowledge_fault())
                elif self.path=="/sensor_guard":
                    self.reply(SENSOR.configure_guard({k:v for k,v in body.items() if k!='_token'}))
                elif self.path=="/sensor_led":
                    self.reply(SENSOR.set_led(body.get("color","off")))
                elif self.path=="/sensor_mode":
                    self.reply(SENSOR.set_mode(body.get("mode","manual")))
                elif self.path=="/sensor_swipe_enable":
                    self.reply(SENSOR.set_swipe_enabled(bool(body.get("enabled",False))))
                elif self.path=="/sensor_taps":
                    self.reply(SENSOR.set_taps(body.get("taps")))
                elif self.path=="/sensor_gestures":
                    self.reply(SENSOR.set_gestures(body.get("gestures")))
                else:
                    self.send_error(404)
            except Exception as exc:
                self.reply({"error":str(exc),"sensor":SENSOR.snapshot()},400)
            return

        if self.path=="/debug_mode":
            try:
                mode=DIAG.set_mode(body.get("mode","BASIC"))
                self.reply({"mode":mode})
            except Exception as e:
                self.reply({"error":str(e)},400)
            return
        if self.path=="/debug_window":
            try:
                seconds=DIAG.set_window(body.get("seconds",300))
                self.reply({"window_s":seconds})
            except Exception as e:
                self.reply({"error":str(e)},400)
            return
        if self.path=="/debug_reset":
            DIAG.reset()
            self.reply({"ok":True,"window_s":DIAG.window_s,"mode":DIAG.mode})
            return
        if self.path=="/plan_world":
            try:
                _sync_state_from_arm_robot()
                pref=body.get("linear_y",None) if state.get("linear_hardware",False) else 0.0
                # Boundary OFF means the Workspace screen is informational;
                # do not use its measured-only planner as a gate.  Direct
                # moves are resolved by Klipper against real geometry.
                if not state.get("boundary_enabled",True):
                    d=float(pref if pref is not None else state.get("linear_y",0.0))
                    plan={"reachable":True,"direct":True,"x":float(body["x"]),
                          "y":float(body["y"]),"z":float(body["z"]),"D":d}
                else:
                    plan=plan_world_target(body["x"],body["y"],body["z"],pref)
                    route=safe_joint_route(
                        (float(state["x"]),float(state["y"])),
                        (float(plan["A"]),float(plan["B"])),
                    )
                    plan["route_points"]=len(route)
                    plan["direct"]=len(route)==2
                self.reply(plan)
            except Exception as e:
                self.reply({"error":str(e)},400)
            return

        if self.path.startswith("/program_"):
            try:
                with lock:
                    if self.path=="/program_record":
                        ensure_homed_for_motion()
                        # A record click made during a jog records the settled
                        # destination, never an arbitrary in-flight position.
                        _gcode("M400",timeout=120)
                        _sync_state_from_arm_robot()
                        raw={"a":state["x"],"b":state["y"],"c":state["z"],"d":state.get("linear_y",0.0)}
                        raw.update(body)
                        point=_program_point(raw, program_defaults)
                        program_points.append(point)
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Recorded program point {len(program_points)}"
                    elif self.path=="/program_add":
                        program_points.append(_program_point(body, program_defaults))
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Added program point {len(program_points)}"
                    elif self.path=="/program_replace":
                        if _program_running():
                            raise RuntimeError("Stop the running program before analyzing changes")
                        raw_points=body.get("points")
                        if not isinstance(raw_points,list) or not raw_points:
                            raise RuntimeError("Provide at least one program waypoint")
                        if len(raw_points)>1000:
                            raise RuntimeError("A program may contain at most 1,000 waypoints")
                        candidate=[_program_point(point, program_defaults) for point in raw_points]
                        _validate_program_path(candidate)
                        program_points[:]=candidate
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Path analyzed — {len(candidate)} waypoint{'s' if len(candidate)!=1 else ''} ready"
                    elif self.path=="/program_replace_cartesian":
                        if _program_running():
                            raise RuntimeError("Stop the running program before analyzing changes")
                        raw_points=body.get("points")
                        if not isinstance(raw_points,list) or not raw_points:
                            raise RuntimeError("Provide at least one Cartesian waypoint")
                        if len(raw_points)>1000:
                            raise RuntimeError("A program may contain at most 1,000 waypoints")
                        candidate=[]
                        # Preserve the continuous linkage branch selected by
                        # each preceding waypoint.  Solving all points from
                        # the initial live pose can falsely reject a valid
                        # Cartesian route after it has turned through several
                        # positions.
                        working={
                            "a": float(state["x"]), "b": float(state["y"]),
                            "c": float(state["z"]),
                            "d": float(state.get("linear_y", 0.0)),
                        }
                        for index, raw in enumerate(raw_points,1):
                            try:
                                tx=float(raw["x"]); ty=float(raw["y"]); tz=float(raw["z"])
                            except (KeyError, TypeError, ValueError):
                                raise RuntimeError(f"Cartesian waypoint {index} needs X, Y, and Z in mm")
                            if not all(math.isfinite(value) for value in (tx,ty,tz)):
                                raise RuntimeError(f"Cartesian waypoint {index} needs finite X, Y, and Z values")
                            timing=_program_settings(raw, program_defaults)
                            requested_d=float(raw.get("d",0.0))
                            if not math.isfinite(requested_d):
                                raise RuntimeError(f"Cartesian waypoint {index} has an invalid D value")
                            # A Cartesian point can have more than one valid
                            # linkage branch.  When the UI is only displaying
                            # an existing joint route in World XYZ, preserve
                            # its exact joint solution.  Once the user edits
                            # the displayed position, this verified hint is
                            # ignored and normal IK chooses a new solution.
                            plan=None
                            hint=raw.get("joint_hint")
                            if isinstance(hint,dict):
                                try:
                                    hinted=_program_point(hint, program_defaults)
                                    hkin=kinematic_state(hinted["a"],hinted["b"],hinted["c"])
                                    htool=hkin["tool"]
                                    hy=float(htool["y"])+hinted["d"]
                                    if (abs(hinted["d"]-requested_d)<=0.51 and
                                            math.dist((float(htool["x"]),hy,float(htool["z"])),
                                                      (tx,ty,tz))<=0.75):
                                        plan={"A":hinted["a"],"B":hinted["b"],
                                              "C":hinted["c"],"D":hinted["d"]}
                                except (TypeError, ValueError, KeyError):
                                    pass
                            if plan is None:
                                try:
                                    plan=plan_world_target(
                                        tx, ty, tz,
                                        requested_d if state.get("linear_hardware",False) else 0.0,
                                        seed=working,
                                    )
                                except RuntimeError as exc:
                                    raise RuntimeError(f"Cartesian waypoint {index}: {exc}") from exc
                            candidate.append(_program_point({
                                "a":plan["A"],"b":plan["B"],"c":plan["C"],"d":plan["D"],**timing
                            }, program_defaults))
                            working={"a":plan["A"],"b":plan["B"],"c":plan["C"],"d":plan["D"]}
                        _validate_program_path(candidate)
                        program_points[:]=candidate
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Cartesian route analyzed — {len(candidate)} joint waypoint{'s' if len(candidate)!=1 else ''} ready"
                    elif self.path=="/program_update":
                        index=int(body.get("index",-1))
                        if not 0 <= index < len(program_points):
                            raise RuntimeError("Program point no longer exists")
                        updated=dict(program_points[index])
                        updated.update({key:body[key] for key in ("a","b","c","d","speed","time","wait") if key in body})
                        candidate=list(program_points)
                        candidate[index]=_program_point(updated, program_defaults)
                        _validate_program_path(candidate)
                        program_points[:]=candidate
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Updated program point {index+1}"
                    elif self.path=="/program_defaults":
                        program_defaults=_program_settings(body, program_defaults)
                        _save_program(program_points, program_defaults)
                        state["message"]="Program defaults saved"
                    elif self.path=="/program_save":
                        if _program_running():
                            raise RuntimeError("Stop the running program before saving")
                        name=_program_name(body.get("name"))
                        _save_named_program(name, program_points, program_defaults)
                        program_active_name=name
                        state["message"]=f"Saved program ‘{name}’"
                    elif self.path=="/program_load":
                        if _program_running():
                            raise RuntimeError("Stop the running program before loading")
                        name=_program_name(body.get("name"))
                        loaded_points, loaded_defaults=_load_named_program(name)
                        _validate_program_path(loaded_points)
                        program_points[:]=loaded_points
                        program_defaults=loaded_defaults
                        program_active_name=name
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Loaded program ‘{name}’ — no motion sent"
                    elif self.path=="/program_delete":
                        if _program_running():
                            raise RuntimeError("Stop the running program before deleting")
                        name=_program_name(body.get("name"))
                        if body.get("confirmation")!="DELETE":
                            raise RuntimeError("Type DELETE to remove a saved program")
                        os.remove(_program_library_path(name))
                        if program_active_name==name:
                            program_active_name=None
                        state["message"]=f"Deleted saved program ‘{name}’"
                    elif self.path=="/program_remove":
                        index=int(body.get("index",-1))
                        if not 0 <= index < len(program_points):
                            raise RuntimeError("Program point no longer exists")
                        program_points.pop(index)
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Removed program point {index+1}"
                    elif self.path=="/program_move":
                        if state.get("busy") or _program_running():
                            raise RuntimeError("Stop the running program before reordering it")
                        index=int(body.get("index",-1))
                        destination=int(body.get("destination",-1))
                        if not 0 <= index < len(program_points):
                            raise RuntimeError("Program point no longer exists")
                        if not 0 <= destination < len(program_points):
                            raise RuntimeError("New waypoint position is invalid")
                        candidate=list(program_points)
                        point=candidate.pop(index)
                        candidate.insert(destination,point)
                        _validate_program_path(candidate)
                        program_points[:]=candidate
                        _save_program(program_points, program_defaults)
                        state["message"]=f"Moved waypoint {index+1} to position {destination+1}"
                    elif self.path=="/program_clear":
                        if _program_running():
                            raise RuntimeError("Stop the running program before clearing it")
                        program_points.clear()
                        _save_program(program_points, program_defaults)
                        state["message"]="Program cleared"
                    elif self.path=="/program_run":
                        ensure_homed_for_motion()
                        if state.get("busy") or _program_running():
                            raise RuntimeError("Another motion is already running")
                        if not program_points:
                            raise RuntimeError("Record or add at least one program point")
                        points=list(program_points)
                        settings={
                            "arm_speed":max(1.0,min(220.0,float(body.get("arm_speed",state["arm_speed"])))),
                            "arm_accel":max(10.0,min(1800.0,float(body.get("arm_accel",state["arm_accel"])))),
                            "linear_speed":max(1.0,min(100.0,float(body.get("linear_speed",state["linear_speed"])))),
                            "linear_accel":max(10.0,min(500.0,float(body.get("linear_accel",state["linear_accel"])))),
                            "arm_corner":max(0.0,min(50.0,float(body.get("arm_corner",state["arm_jerk"])))),
                        }
                        _validate_program_path(points)
                        stop_event.clear()
                        state["busy"]=True
                        state["message"]=f"Program queued — {len(points)} waypoint{'s' if len(points)!=1 else ''}"
                        with program_thread_lock:
                            program_thread=threading.Thread(
                                target=_program_worker,args=(points,settings),daemon=True
                            )
                            program_thread.start()
                    else:
                        self.send_error(404)
                        return
                self.reply(state_payload())
            except Exception as e:
                state["message"]=str(e)
                self.reply({"error":str(e),**state_payload()},500)
            return

        # STOP: non-destructive motion stop. Preserve homing and controller.
        if self.path=="/stop":
            stop_event.set()
            state["message"]="STOP requested — finishing buffered motion"
            try:
                _gcode("ARM_DANCE_STOP",timeout=2)
            except Exception:
                pass
            state["busy"]=False
            self.reply(state_payload())
            return

        # KILL: destructive emergency stop. Re-home required afterward.
        if self.path=="/kill":
            _emergency_kill("emergency stop")
            self.reply(state_payload())
            return

        # Live motion tuning must bypass the long-running motion lock.
        # It updates state immediately and changes limits for future planner
        # segments without M400 or axis re-registration.
        if self.path=="/motion_tuning":
            try:
                arm_speed=max(1.0,min(220.0,float(body.get("arm_speed",state.get("arm_speed",40.0)))))
                arm_accel=max(10.0,min(1800.0,float(body.get("arm_accel",state.get("arm_accel",200.0)))))
                arm_jerk=max(0.0,min(40.0,float(body.get("arm_jerk",state.get("arm_jerk",6.0)))))
                linear_speed=max(1.0,min(100.0,float(body.get("linear_speed",state.get("linear_speed",30.0)))))
                linear_accel=max(10.0,min(500.0,float(body.get("linear_accel",state.get("linear_accel",150.0)))))
                linear_jerk=max(0.0,min(20.0,float(body.get("linear_jerk",state.get("linear_jerk",3.0)))))
                _gcode(
                    f"ARM_TUNE ARM_VELOCITY={arm_speed:.4f} "
                    f"ARM_ACCEL={arm_accel:.4f} ARM_CORNER={arm_jerk:.4f} "
                    f"SLIDE_VELOCITY={linear_speed:.4f} "
                    f"SLIDE_ACCEL={linear_accel:.4f} SLIDE_CORNER={linear_jerk:.4f}",
                    timeout=5
                )
                state.update(
                    arm_speed=arm_speed,arm_accel=arm_accel,arm_jerk=arm_jerk,
                    linear_speed=linear_speed,linear_accel=linear_accel,linear_jerk=linear_jerk
                )
                arm_st=_sync_state_from_arm_robot()
                if arm_st.get("tune_pending"):
                    state["message"]="Motion tuning queued for next native Dance cycle"
                else:
                    state["message"]="Motion tuning applied in Klipper"
                self.reply(state_payload())
            except Exception as e:
                self.reply({"error":str(e),**state},500)
            return

        if self.path=="/home":
            with home_request_lock:
                if home_in_progress:
                    self.reply({"error":"Homing is already in progress",**state},409)
                    return
                home_in_progress=True

        try:
            with lock:
                state["busy"]=True
                if self.path=="/settings_endstops":
                    x,y,z=endstops()
                    d=None
                    if state.get("linear_hardware",False):
                        _gcode("QUERY_ENDSTOPS",timeout=8)
                        time.sleep(0.03)
                        store=_moon_get("/server/gcode_store?count=16",timeout=5)["result"]["gcode_store"]
                        for item in reversed(store):
                            msg=item.get("message","")
                            m=re.search(r"manual_stepper joint_4:(open|TRIGGERED|triggered)",msg)
                            if m:
                                d=m.group(1).lower()=="triggered"
                                break
                    state["message"]="Endstop status refreshed"
                    state["busy"]=False
                    self.reply({**state,"endstop_x":x,"endstop_y":y,"endstop_z":z,"endstop_d":d})
                    return
                elif self.path=="/linear_mode":
                    enabled=bool(body.get("enabled",False))
                    if enabled and not FEATURES['rail']:
                        raise RuntimeError('Rail feature disabled in features.json')
                    _gcode(f"ARM_LINEAR_MODE ENABLE={1 if enabled else 0}",timeout=8)
                    _save_linear_mode(enabled)
                    state["linear_hardware"]=enabled
                    state["linear_y"]=0.0
                    state["homed"]=False
                    state["message"]=(
                        "Rail enabled — connect motor/endstop and Home robot"
                        if enabled else
                        "Rail disabled — fixed at 0 mm; Home robot"
                    )
                elif self.path=="/motors_enable":
                    send("M17")
                    state["motors_enabled"]=True
                    state["message"]="Motor X / Motor Y / Motor Z enabled"
                elif self.path=="/motors_disable":
                    try:
                        _gcode("ARM_DANCE_STOP",timeout=5)
                    except Exception:
                        pass
                    send("M18")
                    state["motors_enabled"]=False
                    state["homed"]=False
                    state["message"]="Motors disabled — re-home before commanded motion"
                elif self.path=="/firmware_restart":
                    _gcode("FIRMWARE_RESTART",timeout=6)
                    wait_for_controller(12)
                    state["motors_enabled"]=None
                    state["homed"]=False
                    state["message"]="Firmware restarted — home required"
                elif self.path=="/boundary_test_new":
                    boundary_test_session["id"] += 1
                    boundary_test_session["started_at"] = time.time()
                    boundary_test_session["points"].clear()
                    state["message"] = f"Boundary test session {boundary_test_session['id']} started"
                elif self.path=="/boundary_test_record":
                    if not state["homed"]: raise RuntimeError("Home first")
                    boundary_test_session["points"].append(_boundary_test_point())
                    state["message"] = f"Recorded boundary point {len(boundary_test_session['points'])}"
                elif self.path=="/boundary":
                    enabled=bool(body.get("enabled", True))
                    _gcode(f"ARM_BOUNDARY ENABLE={1 if enabled else 0}",timeout=5)
                    _save_boundary_mode(enabled)
                    state["boundary_enabled"]=enabled
                    state["message"]=("Boundary protection ON" if state["boundary_enabled"] else "WARNING: Boundary protection OFF")
                elif self.path=="/home":
                    state["message"]="Homing..."; precise_home(); state["message"]="Home complete"
                elif self.path=="/linear_jog":
                    if not state.get("linear_hardware",False):
                        raise RuntimeError("Rail is disabled in Settings")
                    ensure_homed_for_motion()
                    delta=float(body.get("delta",0.0))
                    target=max(0.0,min(235.0,state["linear_y"]+delta))
                    _gcode(f"ARM_MOVE_JOINT D={target:.5f}",timeout=5)
                    state["message"]=f"Native slide target D={target:.1f} mm"
                elif self.path=="/linear_goto":
                    if not state.get("linear_hardware",False):
                        raise RuntimeError("Rail is disabled in Settings")
                    ensure_homed_for_motion()
                    target=max(0.0,min(235.0,float(body.get("target",0.0))))
                    _gcode(f"ARM_MOVE_JOINT D={target:.5f}",timeout=5)
                    state["message"]=f"Native slide target D={target:.1f} mm"
                elif self.path=="/jog":
                    ensure_homed_for_motion()
                    ta=state["x"]+float(body.get("x",0.0))
                    tb=state["y"]+float(body.get("y",0.0))
                    tc=state["z"]+float(body.get("z",0.0))
                    _gcode(f"ARM_MOVE_JOINT A={ta:.5f} B={tb:.5f} C={tc:.5f}",timeout=5)
                    state["message"]="Native joint jog queued"
                elif self.path=="/fine_jog":
                    ensure_homed_for_motion()
                    ta=state["x"]+float(body.get("x",0.0))
                    tb=state["y"]+float(body.get("y",0.0))
                    _gcode(f"ARM_MOVE_JOINT A={ta:.5f} B={tb:.5f}",timeout=5)
                    state["message"]="Native fine joint jog queued"
                elif self.path=="/dance_smooth":
                    ensure_homed_for_motion()
                    if state.get("busy"): raise RuntimeError("Dance is already running")
                    _gcode(
                        f"ARM_TUNE ARM_VELOCITY={state['arm_speed']:.4f} "
                        f"ARM_ACCEL={state['arm_accel']:.4f} ARM_CORNER={state['arm_jerk']:.4f} "
                        f"SLIDE_VELOCITY={state['linear_speed']:.4f} "
                        f"SLIDE_ACCEL={state['linear_accel']:.4f} "
                        f"SLIDE_CORNER={state['linear_jerk']:.4f}",
                        timeout=5
                    )
                    _gcode("ARM_DANCE CYCLES=INF SAMPLES=720", timeout=8)
                    state["busy"]=True
                    state["message"]="Native Klipper Dance running"
                    self.reply(state_payload())
                    return
                elif self.path=="/goto":
                    ensure_homed_for_motion()
                    ta=float(body.get("x",state["x"]))
                    tb=float(body.get("y",state["y"]))
                    tc=float(body.get("z",state["z"]))
                    _gcode(f"ARM_MOVE_JOINT A={ta:.5f} B={tb:.5f} C={tc:.5f}",timeout=5)
                    state["message"]=f"Native joint target A={ta:.1f}, B={tb:.1f}, C={tc:.1f}"
                elif self.path=="/goto_cart":
                    ensure_homed_for_motion()
                    target_y=float(body["y"])
                    target_z=float(body["z"])
                    # Native Klipper IK performs the exact linkage and
                    # boundary checks.  Avoid the old mapper-side grid search:
                    # it delayed a click for seconds before any motion queued.
                    _gcode(
                        f"ARM_MOVE_TOOL FRAME=ARM Y={target_y:.5f} Z={target_z:.5f}",
                        timeout=5
                    )
                    state["message"]=(
                        f"Arm-local move queued Y={target_y:.1f}, Z={target_z:.1f}")
                elif self.path=="/goto_world":
                    ensure_homed_for_motion()
                    tx=float(body["x"]); ty=float(body["y"]); tz=float(body["z"])
                    rail_enabled=bool(state.get("linear_hardware",False))
                    auto_d=rail_enabled and bool(body.get("auto_d",True))
                    requested_d=(None if auto_d else body.get("linear_y",0.0)) if rail_enabled else 0.0
                    if state.get("boundary_enabled",True):
                        plan=plan_world_target(tx,ty,tz,requested_d)
                        d=float(plan["D"]) if rail_enabled else 0.0
                        route=queue_safe_world_route(plan)
                        route_note=(
                            " · direct safe path" if len(route)==2 else
                            f" · safe corridor via {len(route)-2} corner"
                            f"{'s' if len(route) != 3 else ''}"
                        )
                    else:
                        # Do not ask the Boundary ON workspace planner to
                        # approve a direct Cartesian move when protection is
                        # off.  Klipper receives the requested pose directly.
                        d=(float(state.get("linear_y",0.0)) if auto_d else
                           float(requested_d or 0.0)) if rail_enabled else 0.0
                        cmd=(f"ARM_MOVE_TOOL FRAME=WORLD X={tx:.5f} Y={ty:.5f} "
                             f"Z={tz:.5f} D={d:.5f}")
                        _gcode(cmd,timeout=5)
                        route_note=""
                    state["message"]=(f"World move queued X={tx:.1f}, Y={ty:.1f}, "
                                      f"Z={tz:.1f}; D={d:.1f} mm"
                                      + route_note
                                      if rail_enabled else
                                      f"World move queued X={tx:.1f}, Y={ty:.1f}, Z={tz:.1f}; rail disabled"
                                      + route_note)
                else:
                    self.send_error(404); return
                state["busy"]=False; self.reply(state_payload())
        except MotionCancelled as e:
            try:
                send("M18")
            except Exception:
                pass
            state["busy"]=False
            state["homed"]=False
            state["message"]="STOPPED — motors disabled; re-home before jogging"
            stop_event.clear()
            self.reply({"error":str(e),**state},409)
        except Exception as e:
            state["busy"]=False; state["message"]=str(e); self.reply({"error":str(e),**state},500)
        finally:
            if self.path=="/home":
                with home_request_lock:
                    home_in_progress=False

if __name__=="__main__":
    if FEATURES['remote_access']:
        start_remote_access()
    if FEATURES['diagnostics']:
        DIAG.start()
    else:
        DIAG.set_mode('OFF')
    wait_for_controller()
    if FEATURES["sensors"]:
        from sensor_service import SensorService
        SENSOR=SensorService(
            SENSOR_CONFIG_FILE,
            SENSOR_PORT,
            swipe_feature=FEATURES["sensor_swipe_programs"],
            on_kill=lambda reason:_emergency_kill("confirmed tilt fault: "+str(reason)),
            on_correct=_request_tilt_correction,
            can_run_program=_sensor_can_run_program,
            run_program=_sensor_run_program,
        )
        SENSOR.start()
    try:
        _gcode(f"ARM_LINEAR_MODE ENABLE={1 if state.get('linear_hardware',False) else 0}",timeout=8)
        _gcode(f"ARM_BOUNDARY ENABLE={1 if state.get('boundary_enabled',True) else 0}",timeout=8)
    except Exception as exc:
        state["message"]=f"Linear mode restore warning: {exc}"
    _sync_state_from_arm_robot()
    state["message"]="Connected — Klipper owns robot motion; UI is client only"
    url=f"http://{HOST}:{HTTP_PORT}"
    print("Robot mapper:",url)
    print("Keep this terminal open. Ctrl+C stops server.")
    # Do not auto-open/reload a browser tab on restart; stale UI requests can be confusing.
    try: ThreadingHTTPServer((HOST,HTTP_PORT),Handler).serve_forever()
    except KeyboardInterrupt: pass
    finally:
        if SENSOR is not None:
            SENSOR.stop()
        DIAG.stop()
