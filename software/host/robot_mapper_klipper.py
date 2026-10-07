import fcntl
import json
import math
import re
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import urllib.request
import urllib.parse

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
BOUNDARY_YIELD = 5.0
NORMAL_MOVE_FEED_DEFAULT = 2500
NORMAL_ACCEL_MM_S2 = 500.0
DANCE_ACCEL_DEFAULT_MM_S2 = 80.0
MAPPING_MODE = False

# Same model shown by the gold HTML trace.
FIT_UPPER_M = 0.9702
FIT_UPPER_B = 7.9157
FIT_UPPER_X0 = 0.0
FIT_UPPER_X1 = 146.0

FIT_LOWER_M = 0.8810
FIT_LOWER_B = -78.4290
FIT_LOWER_X0 = 89.02
FIT_LOWER_X1 = 182.0

FIT_CAP_P0 = (146.0, 149.56)
FIT_CAP_P1 = (194.3, 154.5)
FIT_CAP_P2 = (191.0, 99.0)
FIT_CAP_P3 = (182.0, 81.91)

def _bezier_xy(t, p0, p1, p2, p3):
    u=1.0-t
    x=(u**3)*p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + (t**3)*p3[0]
    y=(u**3)*p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + (t**3)*p3[1]
    return x,y

def _fitted_cap_xy(t):
    return _bezier_xy(t,FIT_CAP_P0,FIT_CAP_P1,FIT_CAP_P2,FIT_CAP_P3)

def _build_boundary_poly():
    pts=[(0.0,0.0),(0.0,FIT_UPPER_B)]

    # upper line, from x=0 to the cap join
    for i in range(1,81):
        x=FIT_UPPER_X0+(FIT_UPPER_X1-FIT_UPPER_X0)*i/80.0
        pts.append((x,FIT_UPPER_M*x+FIT_UPPER_B))

    # single smooth rounded cap fit through measured upper/right boundary
    for i in range(1,161):
        pts.append(_fitted_cap_xy(i/160.0))

    # lower line, returning toward its y=0 intercept
    for i in range(1,81):
        x=FIT_LOWER_X1+(FIT_LOWER_X0-FIT_LOWER_X1)*i/80.0
        pts.append((x,FIT_LOWER_M*x+FIT_LOWER_B))

    # close along y=0 back to origin
    pts.append((0.0,0.0))
    return pts

BOUNDARY_POLY = _build_boundary_poly()

# Prevent two mapper instances from fighting over the same controller.
_process_lock_file = open("/tmp/robot_mapper.lock", "w")
try:
    fcntl.flock(_process_lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError as exc:
    raise RuntimeError("Another Robot Joint-Space Mapper instance is already running") from exc

# Klipper/Moonraker owns the MCU serial link.  This process only talks to
# Moonraker locally on the Linux host, so browser-network latency never enters motion timing.
lock = threading.Lock()
stop_event = threading.Event()
_axes_registered = False
_current_accel = 80.0

class MotionCancelled(RuntimeError):
    pass

state = {"x":0.0,"y":0.0,"z":0.0,"homed":False,"busy":False,"message":"Connecting to controller...","home_x_counts":None,"home_y_counts":None,"home_z_counts":None,"boundary_enabled":True}
boundary_test_session = {
    "id": 1,
    "started_at": time.time(),
    "points": [],
}

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
    yaw_degrees = -(float(z_counts) - float(home_z))

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


def state_payload():
    """Copy live mapper state and attach one canonical kinematic solution."""
    payload = dict(state)
    payload["boundary_test_session"] = boundary_test_session
    try:
        payload["kinematics"] = kinematic_state(
            state["x"], state["y"], state["z"]
        )
    except ValueError as exc:
        payload["kinematics"] = None
        payload["kinematics_error"] = str(exc)
    return payload

def check_stop():
    if stop_event.is_set():
        raise MotionCancelled("Motion stopped by user")

def _moon_get(path, timeout=5.0):
    with urllib.request.urlopen(MOONRAKER + path, timeout=timeout) as r:
        return json.loads(r.read().decode())

def _moon_post(path, params=None, timeout=30.0):
    data = urllib.parse.urlencode(params or {}).encode()
    req = urllib.request.Request(MOONRAKER + path, data=data, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw=r.read().decode()
        return json.loads(raw) if raw else {}

def _gcode(script, timeout=120.0):
    check_stop()
    return _moon_post("/printer/gcode/script", {"script": script}, timeout=timeout)

def _printer_info():
    try:
        return _moon_get("/printer/info", timeout=3.0).get("result", {})
    except Exception:
        return {}

def wait_for_controller(timeout=20.0):
    deadline=time.time()+timeout
    last=None
    while time.time()<deadline:
        info=_printer_info()
        last=info
        if info.get("state")=="ready":
            return info
        time.sleep(0.5)
    raise RuntimeError(f"Klipper never became ready: {last}")

def _axis_cmd(joint, axis):
    return f"MANUAL_STEPPER STEPPER={joint} GCODE_AXIS={axis} LIMIT_VELOCITY=40 LIMIT_ACCEL=120 INSTANTANEOUS_CORNER_VELOCITY=1"

def unregister_gcode_axes():
    global _axes_registered
    for joint in ("joint_x","joint_y","joint_z"):
        try:
            _gcode(f"MANUAL_STEPPER STEPPER={joint} GCODE_AXIS=", timeout=10)
        except Exception:
            pass
    _axes_registered=False

def register_gcode_axes():
    global _axes_registered
    unregister_gcode_axes()
    _gcode(_axis_cmd("joint_x","A"), timeout=10)
    _gcode(_axis_cmd("joint_y","B"), timeout=10)
    _gcode(_axis_cmd("joint_z","C"), timeout=10)
    _axes_registered=True

def _transform_g1(cmd):
    # Existing page expresses motor commands in old Marlin machine units.
    # Convert those deltas back to the mapper's output-joint degrees:
    # X: 0.50 Marlin unit / degree; Y: -0.50; Z: -0.10.
    vals={}
    for axis,val in re.findall(r"([XYZF])\\s*(-?\\d+(?:\\.\\d+)?)", cmd, re.I):
        vals[axis.upper()]=float(val)
    parts=["G1"]
    if "X" in vals: parts.append(f"A{vals['X']/X_COUNT_MOVE:.5f}")
    if "Y" in vals: parts.append(f"B{vals['Y']/Y_COUNT_MOVE:.5f}")
    if "Z" in vals: parts.append(f"C{vals['Z']/Z_COUNT_MOVE:.5f}")
    if "F" in vals:
        # Old feed was Marlin mm/min. X/Y conversion is 2 joint-deg per unit.
        parts.append(f"F{max(120.0,min(4800.0,vals['F']*2.0)):.0f}")
    return " ".join(parts)

def send(cmd, timeout=15):
    global _current_accel
    c=cmd.strip()
    u=c.upper()
    if u.startswith("M115"):
        return [json.dumps(wait_for_controller(timeout=max(2.0,float(timeout))))]
    if u.startswith("M503"):
        return read_config()
    if u.startswith("M211"):
        return ["ok"]
    if u.startswith("M17"):
        for j in ("joint_x","joint_y","joint_z"):
            _gcode(f"MANUAL_STEPPER STEPPER={j} ENABLE=1", timeout=10)
        return ["ok"]
    if u.startswith("M18") or u.startswith("M84"):
        unregister_gcode_axes()
        for j in ("joint_x","joint_y","joint_z"):
            _gcode(f"MANUAL_STEPPER STEPPER={j} ENABLE=0", timeout=10)
        return ["ok"]
    if u.startswith("M204"):
        m=re.search(r"(?:P|T)(-?\\d+(?:\\.\\d+)?)", c, re.I)
        if m:
            _current_accel=max(5.0,min(120.0,float(m.group(1))))
        return ["ok"]
    if u.startswith("G91") or u.startswith("G90") or u.startswith("M400"):
        _gcode(u.split()[0], timeout=max(15.0,float(timeout)))
        return ["ok"]
    if u.startswith("G92"):
        # G92 is only used by the legacy homing functions; native Klipper
        # homing below sets manual-stepper positions directly.
        return ["ok"]
    if u.startswith("G1"):
        if not _axes_registered:
            register_gcode_axes()
        _gcode(_transform_g1(c), timeout=max(30.0,float(timeout)))
        return ["ok"]
    _gcode(c, timeout=max(15.0,float(timeout)))
    return ["ok"]

def read_config():
    return [
        "Klipper/Moonraker motion backend",
        "joint_x: 20T->90T, rotation_distance=80 deg/rev",
        "joint_y: 20T->90T, rotation_distance=80 deg/rev",
        "joint_z: 20T->90T, rotation_distance=80 deg/rev",
    ]

def move(cmd):
    send(cmd, timeout=30)
    send("M400", timeout=120)

def endstops():
    _gcode("QUERY_ENDSTOPS", timeout=10)
    time.sleep(0.20)
    store=_moon_get("/server/gcode_store?count=20", timeout=5)["result"]["gcode_store"]
    for item in reversed(store):
        msg=item.get("message","")
        if item.get("type")!="response" or "manual_stepper joint_x:" not in msg:
            continue
        vals={}
        for name,status in re.findall(r"manual_stepper (joint_[xyz]):(open|TRIGGERED|triggered)", msg):
            vals[name]=status.lower()=="triggered"
        if len(vals)==3:
            return vals["joint_x"],vals["joint_y"],vals["joint_z"]
    raise RuntimeError("Could not read Klipper endstops")

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
    """Klipper-native two-stage arm homing.

    X/Y approach together and stop independently on their own switches, then Z.
    After coarse release the sequence repeats slowly for precision contact.
    """
    if clear_stop:
        stop_event.clear()

    info=_printer_info()
    if info.get("state")=="shutdown":
        # A UI STOP intentionally emergency-stops Klipper. Recover before homing.
        try:
            _moon_post("/printer/gcode/script", {"script":"FIRMWARE_RESTART"}, timeout=10)
        except Exception:
            pass
        wait_for_controller(15)

    unregister_gcode_axes()
    state["homed"]=False

    def mstep(joint, pos=None, speed=5.0, accel=15.0, stop=None, sync=True,
              set_position=None, enable=None):
        bits=[f"MANUAL_STEPPER STEPPER={joint}"]
        if enable is not None: bits.append(f"ENABLE={1 if enable else 0}")
        if set_position is not None: bits.append(f"SET_POSITION={set_position:.5f}")
        if pos is not None:
            bits += [f"MOVE={pos:.5f}",f"SPEED={speed:.3f}",f"ACCEL={accel:.3f}"]
            if stop: bits.append(f"STOP_ON_ENDSTOP={stop}")
            if not sync: bits.append("SYNC=0")
        _gcode(" ".join(bits), timeout=30)

    for j in ("joint_x","joint_y","joint_z"):
        mstep(j, enable=True, set_position=0.0)

    # Coarse X/Y: tested equivalent of the old 2.5-Marlin-unit (~5 deg) steps.
    state["message"]="Homing — coarse X/Y together"
    step=5.0
    for i in range(40):
        check_stop()
        xh,yh,_=endstops()
        if xh and yh:
            break
        target=-(i+1)*step
        if not xh: mstep("joint_x",target,7.0,20.0,"try_probe",False)
        if not yh: mstep("joint_y",target,7.0,20.0,"try_probe",False)
        time.sleep(1.0)
    else:
        xh,yh,_=endstops()
        raise RuntimeError(f"XY coarse home safety limit: X={xh}, Y={yh}")

    # Coarse Z.
    state["message"]="Homing — coarse Z"
    mstep("joint_z", set_position=0.0)
    for i in range(60):
        check_stop()
        _,_,zh=endstops()
        if zh:
            break
        mstep("joint_z",-(i+1)*5.0,7.0,20.0,"try_probe",True)
    else:
        raise RuntimeError("Z coarse home safety limit")

    # Coarse contacts become temporary zero.
    for j in ("joint_x","joint_y","joint_z"):
        mstep(j,set_position=0.0)

    # Release each switch, then add 1.5 deg extra clearance.
    state["message"]="Homing — switch release"
    def release(joint,index):
        pos=0.0
        for _ in range(30):
            check_stop()
            if not endstops()[index]:
                break
            pos += 1.0
            mstep(joint,pos,5.0,15.0)
        else:
            raise RuntimeError(f"Could not release {joint}")
        pos += 1.5
        mstep(joint,pos,5.0,15.0)
        return pos

    release("joint_x",0)
    release("joint_y",1)
    release("joint_z",2)

    for j in ("joint_x","joint_y","joint_z"):
        mstep(j,set_position=0.0)

    # Precision X/Y together; 0.5 degree increments.
    state["message"]="Homing — precision X/Y together"
    for i in range(40):
        check_stop()
        xh,yh,_=endstops()
        if xh and yh:
            break
        target=-(i+1)*0.5
        if not xh: mstep("joint_x",target,2.8,8.0,"try_probe",False)
        if not yh: mstep("joint_y",target,2.8,8.0,"try_probe",False)
        time.sleep(0.55)
    else:
        raise RuntimeError("XY fine home safety limit")

    # Precision Z.
    state["message"]="Homing — precision Z"
    mstep("joint_z",set_position=0.0)
    for i in range(40):
        check_stop()
        _,_,zh=endstops()
        if zh:
            break
        mstep("joint_z",-(i+1)*0.5,2.0,6.0,"try_probe",True)
    else:
        raise RuntimeError("Z fine home safety limit")

    # Exact switch contact is coordinate zero.
    for j in ("joint_x","joint_y","joint_z"):
        mstep(j,set_position=0.0)

    state["message"]="Homing — final release"
    ox=release("joint_x",0)
    oy=release("joint_y",1)
    oz=release("joint_z",2)

    xh,yh,zh=endstops()
    if xh or yh or zh:
        raise RuntimeError(f"Home release incomplete: X={xh}, Y={yh}, Z={zh}")

    state.update(
        x=ox,y=oy,z=oz,homed=True,
        home_x_counts=ox,home_y_counts=oy,home_z_counts=oz
    )
    register_gcode_axes()
    _gcode("G90", timeout=10)
    state["message"]=f"Precision home complete — release X={ox:.1f}°, Y={oy:.1f}°, Z={oz:.1f}°"


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

def clip_xy_path(x0,y0,x1,y1):
    """Clip a straight XY move to the measured envelope + inward yield.

    When starting inside the 5-count yield zone (e.g. immediately after home),
    allow motion that does not reduce the current safety score so the arm can
    move back toward the interior.
    """
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

def smooth_dance_waypoints(home_z, cycles=10, arm_steps=1, base_steps=2):
    """Generate a looped dance with the same arm sweep at each base angle."""
    neutral=(100.0, 80.0)
    # User-selected dance poses. P2/P3 were supplied in Tool Y/Z and
    # converted through Assembly-1 IK using the current home calibration.
    arm_keyframes=[
        neutral,
        (29.8, 5.1),
        (96.0, 29.7),
        (135.2, 96.5),        neutral,
    ]
    sweep=[0.0,30.0,60.0,90.0,120.0,150.0,180.0,150.0,120.0,90.0,60.0,30.0,0.0]
    points=[]

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


def queue_count_waypoints(waypoints, feed=1200):
    """Queue absolute logical waypoints without stopping at each segment.

    Every segment is boundary-checked, converted to Marlin relative units, and
    queued with G1. Only the final M400 waits for physical completion, allowing
    Marlin's planner to blend adjacent segments into a smooth path.
    """
    feed=max(60.0,min(5000.0,float(feed)))
    send("G91", timeout=15)
    cx,cy,cz=state["x"],state["y"],state["z"]
    for index,(tx,ty,tz) in enumerate(waypoints,1):
        check_stop()
        tx=float(tx); ty=float(ty); tz=max(0.0,float(tz))
        if state.get("boundary_enabled", True):
            nx,ny,clipped=clip_xy_path(cx,cy,tx,ty)
            if clipped or abs(nx-tx)>0.05 or abs(ny-ty)>0.05:
                raise RuntimeError(f"Dance waypoint {index} reached Arm X/Y boundary")
        else:
            nx,ny=tx,ty
        dx,dy,dz=nx-cx,ny-cy,tz-cz
        if abs(dx)+abs(dy)+abs(dz)>1e-9:
            parts=[]
            if abs(dx)>1e-9: parts.append(f"X{dx*X_COUNT_MOVE:.4f}")
            if abs(dy)>1e-9: parts.append(f"Y{dy*Y_COUNT_MOVE:.4f}")
            if abs(dz)>1e-9: parts.append(f"Z{dz*Z_COUNT_MOVE:.4f}")
            send("G1 "+" ".join(parts)+f" F{feed:.0f}", timeout=15)
            cx,cy,cz=nx,ny,tz
            state.update(x=cx,y=cy,z=cz)
    send("M400", timeout=300)


def jog_counts(dx,dy,dz,feed=240):
    requested_x=max(0.0,state["x"]+dx)
    requested_y=max(0.0,state["y"]+dy)
    nz=max(0.0,state["z"]+dz)

    if not state.get("boundary_enabled", True):
        nx,ny,clipped=requested_x,requested_y,False
    else:
        nx,ny,clipped=clip_xy_path(
            state["x"],state["y"],requested_x,requested_y
        )

    dx,dy,dz=nx-state["x"],ny-state["y"],nz-state["z"]
    if abs(dx)+abs(dy)+abs(dz)<1e-9:
        if clipped:
            state["message"]=f"BOUNDARY LIMIT — {BOUNDARY_YIELD:.1f}-count safety yield"
        return

    p=[]
    if dx: p.append(f"X{dx*X_COUNT_MOVE:.4f}")
    if dy: p.append(f"Y{dy*Y_COUNT_MOVE:.4f}")
    if dz: p.append(f"Z{dz*Z_COUNT_MOVE:.4f}")
    send("G91")
    move("G1 "+" ".join(p)+f" F{feed}")
    state.update(x=nx,y=ny,z=nz)
    if clipped:
        state["message"]=(
            f"BOUNDARY LIMITED — requested ({requested_x:.1f}, {requested_y:.1f}), "
            f"stopped at ({nx:.1f}, {ny:.1f}) with {BOUNDARY_YIELD:.1f}-count yield"
        )


HTML = r'''<!doctype html>
<meta charset="utf-8">
<title>Robot Arm Control</title>
<style>
:root{color-scheme:dark;--bg:#080b0f;--panel:#11161c;--panel2:#0c1116;--line:#28323c;--text:#eef2f5;--muted:#8d98a4;--accent:#52a9e8;--safe:#6fd49b;--warn:#e8bc59}
*{box-sizing:border-box}html,body{margin:0;width:100%;height:100%;overflow:hidden}body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:var(--bg);color:var(--text)}
main{height:100vh;width:100vw;padding:10px;display:grid;grid-template-rows:44px minmax(0,1fr);gap:8px;overflow:hidden}
.topbar{display:flex;align-items:center;gap:8px;padding:0 6px}.title{font-size:20px;font-weight:800;margin-right:auto}.status{display:flex;align-items:center;gap:6px}.badge{padding:5px 9px;border:1px solid var(--line);border-radius:999px;font-size:11px;font-weight:700;background:#0c1116;white-space:nowrap}.good{color:var(--safe)}.warn{color:var(--warn)}.bad{color:#ff9696}
button,input{font:inherit}button{padding:7px 10px;border-radius:8px;border:1px solid #34404b;background:#202832;color:#fff;font-size:12px;cursor:pointer}button:hover{background:#2b3540}button.primary{background:#23608d;border-color:#347aad}button.danger{background:#6d2b2b;border-color:#934545}button.active{outline:2px solid var(--accent);background:#20384d}button:disabled{opacity:.55;cursor:not-allowed}
input{width:72px;padding:6px 7px;border:1px solid #34404b;border-radius:7px;background:#090d12;color:#fff;font-size:12px}.wide{width:88px}
.dashboard{min-height:0;display:grid;grid-template-columns:minmax(0,1.15fr) minmax(420px,.85fr);grid-template-rows:360px minmax(0,1fr);gap:8px}.card{min-width:0;min-height:0;background:var(--panel);border:1px solid var(--line);border-radius:12px;overflow:hidden}.cardTitle{display:flex;align-items:flex-start;justify-content:space-between;gap:10px;padding:10px 12px 7px}.cardTitle h2{margin:0;font-size:14px}.muted{color:var(--muted);font-size:10px;line-height:1.25}.toolbar{display:flex;align-items:center;gap:5px;flex-wrap:wrap}
.armCard{grid-column:1;grid-row:1;display:grid;grid-template-rows:auto minmax(0,1fr) auto;padding-bottom:6px}#arm3d{display:block;width:calc(100% - 16px);height:100%;min-height:0;margin:0 8px;border:1px solid var(--line);border-radius:9px;background:radial-gradient(circle at 50% 35%,#171e26,#07090c 72%);cursor:grab}.readouts{display:grid;grid-template-columns:repeat(6,1fr);gap:4px;padding:4px 8px 0}.readout{padding:3px 4px;border:1px solid #25303a;border-radius:7px;background:#0b1015;text-align:center}.readout .k{font-size:8px;color:var(--muted);text-transform:uppercase}.readout .v{font-size:13px;font-weight:700;margin-top:1px}
.controls{grid-column:2;grid-row:1;display:flex;flex-direction:column;padding:0 8px 5px;overflow:hidden}.controls>.cardTitle{padding:5px 2px 3px}.group{background:var(--panel2);border:1px solid #242e38;border-radius:8px;padding:5px;margin-bottom:4px}.groupTitle{font-size:8px;color:var(--muted);text-transform:uppercase;letter-spacing:.09em;margin-bottom:4px}.axisRow{display:grid;grid-template-columns:50px 30px 50px 30px;gap:4px;align-items:center;margin:2px 0}.axisName{font-weight:750;font-size:12px}.axisRow .val{text-align:center;font-weight:700;font-variant-numeric:tabular-nums}.moveGrid{display:grid;grid-template-columns:1fr 1fr;gap:4px}.fieldrow{display:flex;align-items:center;gap:6px;flex-wrap:wrap}.fieldrow label{font-size:10px;color:var(--muted)}details{font-size:9px;color:var(--muted);margin-top:0}summary{cursor:pointer}
.plotCard{display:grid;grid-template-rows:auto minmax(0,1fr);padding:0 8px 8px;min-height:0}.plotCard .cardTitle{padding:8px 4px 6px;align-items:center}.plotCard .cardTitle h2{font-size:13px}.plotCard svg{width:100%;height:100%;display:block;background:#090c10;border:1px solid var(--line);border-radius:8px}.workspaceCard{grid-column:1;grid-row:2}.boundaryCard{grid-column:2;grid-row:2}.legend{font-size:9px;color:var(--muted)}#rowsWrap{display:none}
#msg{font-size:11px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:min(42vw,560px)}
@media(max-height:660px) and (min-width:1101px){
  .dashboard{grid-template-rows:330px minmax(0,1fr)}
  .readouts{display:none}
  .controls .cardTitle{padding-top:3px;padding-bottom:2px}
  .controls .group{padding:4px;margin-bottom:3px}
}
@media(max-width:1100px){html,body{overflow:auto}main{height:auto;min-height:100vh}.dashboard{grid-template-columns:1fr;grid-template-rows:420px auto 280px 280px}.armCard{grid-column:1;grid-row:1}.controls{grid-column:1;grid-row:2;overflow:visible}.workspaceCard{grid-column:1;grid-row:3}.boundaryCard{grid-column:1;grid-row:4}}
</style>
<main>
  <div class="topbar">
    <div class="title">Alex Arm Control</div>
    <div id="msg">Loading…</div>
    <div class="status">
      <span id="homedBadge" class="badge bad">NOT HOMED</span>
      <span id="motionBadge" class="badge">IDLE</span>
      <button id="boundaryToggle" class="good">Boundary ON</button>
      <button id="home" class="primary">Home</button>
      <button id="stop" class="danger">STOP</button>
    </div>
  </div>

  <div class="dashboard">
    <section class="card armCard">
      <div class="cardTitle">
        <div><h2>Live arm</h2><div class="muted">Assembly 1 · arm-local Y/Z · labels A–T</div></div>
        <span class="badge good">LIVE</span>
      </div>
      <canvas id="arm3d" width="1000" height="560"></canvas>
      <div class="readouts">
        <div class="readout"><div class="k">Arm X</div><div class="v" id="angLower">—</div></div>
        <div class="readout"><div class="k">Arm Y</div><div class="v" id="angUpper">—</div></div>
        <div class="readout"><div class="k">Base Z</div><div class="v" id="angBase">—</div></div>
        <div class="readout"><div class="k">Tool X</div><div class="v" id="cartX">—</div></div>
        <div class="readout"><div class="k">Tool Y</div><div class="v" id="cartY">—</div></div>
        <div class="readout"><div class="k">Tool Z</div><div class="v" id="cartZ">—</div></div>
      </div>
      <span id="angBF" style="display:none"></span><span id="angFCH" style="display:none"></span>
    </section>

    <section class="card controls">
      <div class="cardTitle">
        <div><h2>Move robot</h2><div class="muted">Direct joints, tool target, or Dance</div></div>
        <div class="toolbar">
          <button id="dance" class="primary">Dance ∞</button>
          <label class="muted">F <input id="moveFeed" class="wide" type="number" min="60" max="5000" step="50" value="2500"></label>
          <label class="muted">a <input id="danceAccel" class="wide" type="number" min="20" max="200" step="10" value="80"></label>
        </div>
      </div>

      <div class="group">
        <div class="groupTitle">Step arm joints</div>
        <div class="toolbar" style="margin-bottom:6px"><span class="muted">Step</span><button class="stepPreset" data-step-value="0.25">.25</button><button class="stepPreset active" data-step-value="1">1</button><button class="stepPreset" data-step-value="5">5</button><button class="stepPreset" data-step-value="10">10</button><input id="step" type="number" min="0.1" max="20" step="0.1" value="1"></div>
        <div class="axisRow"><span class="axisName">Arm X</span><button data-axis="x" data-dir="-1">−</button><span class="val" id="sx">0.0</span><button data-axis="x" data-dir="1">+</button></div>
        <div class="axisRow"><span class="axisName">Arm Y</span><button data-axis="y" data-dir="-1">−</button><span class="val" id="sy">0.0</span><button data-axis="y" data-dir="1">+</button></div>
        <div class="axisRow"><span class="axisName">Base Z</span><button data-axis="z" data-dir="-1">−</button><span class="val" id="sz">0.0</span><button data-axis="z" data-dir="1">+</button></div>
      </div>

      <div class="moveGrid">
        <div class="group">
          <div class="groupTitle">Absolute arm target</div>
          <div class="fieldrow"><label>Arm X <input id="targetX" type="number" step="0.5" value="0"></label><label>Arm Y <input id="targetY" type="number" step="0.5" value="0"></label><label>Base Z <input id="targetZ" type="number" step="0.5" value="0"></label></div>
          <div class="toolbar" style="margin-top:6px"><button id="goTarget" class="primary">Go joint</button><button id="useCurrent">Use current</button></div>
        </div>

        <div class="group">
          <div class="groupTitle">Arm-local tool target</div>
          <div class="fieldrow"><label>Y mm <input id="cartTargetY" type="number" step="1"></label><label>Z mm <input id="cartTargetZ" type="number" step="1"></label></div>
          <div class="toolbar" style="margin-top:6px"><button id="cartGo" class="primary">Go tool</button><button id="cartUseCurrent">Use current</button></div>
        </div>
      </div>

      <details>
        <summary>Fine / coordinated controls</summary>
        <div class="toolbar" style="margin-top:6px"><button data-fine-axis="x" data-dir="-1">X − fine</button><button data-fine-axis="x" data-dir="1">X + fine</button><button data-fine-axis="y" data-dir="-1">Y − fine</button><button data-fine-axis="y" data-dir="1">Y + fine</button><label>ΔX <input id="dx" value="5"></label><label>ΔY <input id="dy" value="5"></label><button id="xyMove">Move XY</button><button id="xyBack">Reverse</button></div>
      </details>
    </section>

    <section class="card plotCard workspaceCard">
      <div class="cardTitle">
        <div><h2>Arm Y/Z workspace</h2><div class="muted">Arm-local radial Y / Z</div></div>
        <div class="toolbar"><label class="muted">World preview ° <input id="basePreview" type="number" step="5" value="0"></label><span class="badge">ARM-LOCAL</span></div>
      </div>
      <svg id="cartPlot" viewBox="0 0 700 360"></svg>
    </section>

    <section class="card plotCard boundaryCard">
      <div class="cardTitle">
        <div><h2>Arm X/Y boundary</h2><div class="muted">Measured safe joint-space region</div></div>
        <div class="toolbar"><span id="testSession" class="badge">Test 1 · 0 pts</span><button id="newBoundaryTest">New Test</button><button id="record">Record Point</button></div>
      </div>
      <svg id="plot" viewBox="0 0 700 520"></svg>
      <div id="rowsWrap"><table><tbody id="rows"></tbody></table></div>
    </section>
  </div>
</main>
<script>
const S={x:0,y:0,z:0,busy:false,homed:false},DISPLAY={x:0,y:0,z:0},pts=[],$=id=>document.getElementById(id);
let motionAnim=null;
let selectedTarget=null;
let selectedCartTarget=null;
let danceRunning=false;

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
  const p=(!motionAnim && S.kinematics)
    ? backendRobotPose(S.kinematics)
    : solvedRobotPose(DISPLAY);
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

function drawArm3D(p){
  const c=$('arm3d'); if(!c)return;
  const ctx=c.getContext('2d'),W=c.width,H=c.height;
  ctx.clearRect(0,0,W,H);

  function proj(v){
    const cy=Math.cos(viewYaw),sy=Math.sin(viewYaw);
    const cp=Math.cos(viewPitch),sp=Math.sin(viewPitch);
    const x1=cy*v.x-sy*v.y;
    const y1=sy*v.x+cy*v.y;
    const y2=cp*y1-sp*v.z;
    const z2=sp*y1+cp*v.z;
    const sc=1.18;
    return {x:W*0.50+x1*sc,y:H*0.72-z2*sc,d:y2};
  }
  function line3(a,b,width,stroke){
    const A=proj(a),B=proj(b);
    ctx.beginPath();ctx.moveTo(A.x,A.y);ctx.lineTo(B.x,B.y);
    ctx.lineWidth=width;ctx.lineCap='round';ctx.strokeStyle=stroke;ctx.stroke();
  }
  function joint(v,r,fill,stroke='#dfe7ef'){
    const q=proj(v);ctx.beginPath();ctx.arc(q.x,q.y,r,0,Math.PI*2);
    ctx.fillStyle=fill;ctx.fill();ctx.lineWidth=2;ctx.strokeStyle=stroke;ctx.stroke();
  }
  function jointLabel(name,v){
    const q=proj(v);
    ctx.font='bold 15px system-ui';
    ctx.textAlign='left';
    ctx.textBaseline='bottom';
    ctx.lineWidth=4;
    ctx.strokeStyle='rgba(8,10,13,.95)';
    ctx.strokeText(name,q.x+9,q.y-7);
    ctx.fillStyle='#ffffff';
    ctx.fillText(name,q.x+9,q.y-7);
  }

  for(let i=-300;i<=300;i+=50){
    line3({x:i,y:-300,z:0},{x:i,y:300,z:0},1,'rgba(120,140,160,.16)');
    line3({x:-300,y:i,z:0},{x:300,y:i,z:0},1,'rgba(120,140,160,.16)');
  }
  line3({x:0,y:0,z:0},{x:90,y:0,z:0},2,'#e35d6a');
  line3({x:0,y:0,z:0},{x:0,y:90,z:0},2,'#65c98c');
  line3({x:0,y:0,z:0},{x:0,y:0,z:90},2,'#64b5f6');

  const baseTop={x:0,y:0,z:0},baseBottom={x:0,y:0,z:-28};
  line3(baseBottom,baseTop,18,'#596572');
  joint(baseBottom,15,'#303840','#84909c');
  joint(baseTop,11,'#e7edf3','#dfe7ef');

  if(!p){
    ctx.fillStyle='#ff8f8f';
    ctx.font='bold 18px system-ui';
    ctx.fillText('LINKAGE CANNOT CLOSE',W*0.5-110,H*0.5);
    return;
  }

  const q=p.world;

  // Exact final supplied linkage.
  line3(q.A,q.B,9,'#d35f54');     // 40 mm motor crank
  line3(q.A,q.C,18,'#f0b35a');    // red lower arm 120
  line3(q.B,q.F,9,'#3e86d8');     // lower blue 120
  line3(q.C,q.F,9,'#d35f54');     // red tail 40
  line3(q.C,q.H,18,'#69b7ff');    // red/upper arm 120

  line3(q.A,q.G,8,'#7bd36a');     // fixed ground spacing 32
  line3(q.G,q.E,9,'#7bd36a');     // green support 120
  line3(q.C,q.E,8,'#d3a43c');     // 32
  line3(q.E,q.D,8,'#d3a43c');     // 45 horizontal
  line3(q.C,q.D,8,'#d3a43c');     // 25

  line3(q.D,q.I,9,'#3e86d8');     // upper blue 120
  line3(q.H,q.I,8,'#d3a43c');     // wrist plate 25
  line3(q.H,q.T,11,'#9be37c');    // tool 27 horizontal

  const labeledJoints=[
    ['A',q.A,'#e7edf3'],['B',q.B,'#d35f54'],['C',q.C,'#f0b35a'],
    ['D',q.D,'#d3a43c'],['E',q.E,'#d3a43c'],['F',q.F,'#3e86d8'],
    ['G',q.G,'#7bd36a'],['H',q.H,'#69b7ff'],['I',q.I,'#3e86d8'],
    ['T',q.T,'#9be37c']
  ];
  for(const [name,pt,fill] of labeledJoints) joint(pt,name==='T'?9:6,fill);
  for(const [name,pt] of labeledJoints) jointLabel(name,pt);

  line3(q.T,{x:q.T.x,y:q.T.y,z:0},1.5,'rgba(155,227,124,.42)');
  const t=proj(q.T);
  ctx.fillStyle='#e8eef5';ctx.font='14px system-ui';
  ctx.fillText('Tool  '+q.T.x.toFixed(0)+', '+q.T.y.toFixed(0)+', '+q.T.z.toFixed(0)+' mm',t.x+12,t.y-10);

  ctx.fillStyle='rgba(220,228,236,.72)';ctx.font='12px system-ui';
  ctx.fillText('X',proj({x:105,y:0,z:0}).x,proj({x:105,y:0,z:0}).y);
  ctx.fillText('Y',proj({x:0,y:105,z:0}).x,proj({x:0,y:105,z:0}).y);
  ctx.fillText('Z',proj({x:0,y:0,z:105}).x,proj({x:0,y:0,z:105}).y);
}

// Clean fitted boundary trace for visual tuning.
// Keep raw data visible; this is just the simplified overlay.
const FIT_UPPER={m:0.9702,b:7.9157,x0:0,x1:146.0};
const FIT_LOWER={m:0.8810,b:-78.4290,x0:89.02,x1:182.0};
// Single smooth rounded cap fit through the measured upper/right boundary.
// No artificial notch: the whole tip remains one continuous curve.
const FIT_CAP={
  p0:{x:146.0,y:149.56},
  p1:{x:194.3,y:154.5},
  p2:{x:191.0,y:99.0},
  p3:{x:182.0,y:81.91}
};

function bezierPoint(t,p0,p1,p2,p3){
  const u=1-t;
  return {
    x:u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x,
    y:u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y
  };
}
function fittedCapPoint(t){
  return bezierPoint(t,FIT_CAP.p0,FIT_CAP.p1,FIT_CAP.p2,FIT_CAP.p3);
}
function smoothCapPoints(){
  const out=[];
  for(let i=0;i<=160;i++) out.push(fittedCapPoint(i/160));
  return out;
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

async function api(path,body){const r=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});const j=await r.json();if(!r.ok)throw Error(j.error||'Request failed');return j}
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

function rs(s){
  Object.assign(S,s);
  if(S.boundary_test_session){
    pts.length=0;
    for(const p of (S.boundary_test_session.points||[])) pts.push(p);
    const ts=$('testSession');
    if(ts) ts.textContent='Test '+S.boundary_test_session.id+' · '+pts.length+' pts';
  }
  $('msg').textContent=(S.busy?'BUSY — ':'')+(S.message||'');
  $('msg').className=S.busy?'busy':'ok';
  const hb=$('homedBadge');
  hb.textContent=S.homed?'HOMED':'NOT HOMED';
  hb.className='badge '+(S.homed?'good':'bad');
  const bb=$('boundaryToggle');
  if(bb){
    const on=S.boundary_enabled!==false;
    bb.textContent=on?'Boundary ON':'BOUNDARY OFF';
    bb.className=on?'good':'danger';
  }
  setMotionBadge(Boolean(S.busy||motionAnim));
  if($('targetZ') && !$('targetZ').dataset.userSet) $('targetZ').value=S.z.toFixed(1);
  if(!motionAnim){
    DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;
    renderDisplayState();
  }
}
async function refresh(){try{rs(await api('/state'))}catch(e){$('msg').textContent=e.message}}
async function jog(x,y,z){
  const feed=Math.max(60,Math.min(5000,parseFloat($('moveFeed').value)||2500));
  const target={x:Math.max(0,S.x+(x||0)),y:Math.max(0,S.y+(y||0)),z:Math.max(0,S.z+(z||0))};
  animateCommandedPose(target,feed);
  try{
    const finalState=await api('/jog',{x:x||0,y:y||0,z:z||0,feed});
    stopMotionAnimation(); rs(finalState);
  }catch(e){
    stopMotionAnimation();DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;renderDisplayState();alert(e.message)
  }
}
async function fineJog(x,y){
  const target={x:Math.max(0,S.x+(x||0)),y:Math.max(0,S.y+(y||0)),z:S.z};
  animateCommandedPose(target,120);
  try{const finalState=await api('/fine_jog',{x:x||0,y:y||0});stopMotionAnimation();rs(finalState)}
  catch(e){stopMotionAnimation();DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;renderDisplayState();alert(e.message)}
}
async function goCartesianTarget(y,z){
  y=Number(y); z=Number(z);
  if(!Number.isFinite(y)||!Number.isFinite(z)){alert('Enter valid Tool Y and Z');return;}
  const feed=Math.max(60,Math.min(5000,parseFloat($('moveFeed').value)||2500));
  selectedCartTarget={y,z};
  drawCartesianWorkspace();
  try{
    const finalState=await api('/goto_cart',{y,z,feed});
    stopMotionAnimation();
    rs(finalState);
  }catch(e){alert(e.message);refresh();}
}

async function goJointTarget(x,y,z,feedOverride=null){
  x=Number(x);y=Number(y);z=Number(z);  if(!Number.isFinite(x)||!Number.isFinite(y)||!Number.isFinite(z)) throw Error('Invalid Arm X / Arm Y / Base Z target');
  const feed=feedOverride===null
    ?Math.max(60,Math.min(5000,parseFloat($('moveFeed').value)||2500))
    :Math.max(60,Math.min(5000,Number(feedOverride)||2500));
  selectedTarget={x,y};
  draw();
  animateCommandedPose({x,y,z},feed);
  try{
    const finalState=await api('/goto',{x,y,z,feed});
    stopMotionAnimation();
    rs(finalState);
    return finalState;
  }catch(e){
    stopMotionAnimation();
    DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;
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
  btn.textContent='Dance ∞ — STOP to end';
  try{
    if(S.boundary_enabled===false) rs(await api('/boundary',{enabled:true}));
    const requested=Math.max(300,Math.min(3000,parseFloat($('moveFeed').value)||1200));
    const feed=Math.min(1200,requested);
    const accel=Math.max(20,Math.min(200,parseFloat($('danceAccel').value)||80));
    const homeZ=Number.isFinite(S.home_z_counts)?S.home_z_counts:S.z;
    const waypoints=smoothDanceDisplayWaypoints(homeZ,10,1,2);
    animateWaypointSequence(waypoints,feed);
    const finalState=await api('/dance_smooth',{feed,accel});
    stopMotionAnimation();
    rs(finalState);
  }catch(e){
    stopMotionAnimation();
    DISPLAY.x=S.x;DISPLAY.y=S.y;DISPLAY.z=S.z;renderDisplayState();
    // STOP intentionally ends the endless dance with HTTP 409.
    if(!String(e.message||'').toLowerCase().includes('cancel')) alert(e.message);
  }finally{
    danceRunning=false;
    btn.disabled=false;
    btn.textContent='Dance ∞';
  }
}
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
document.querySelectorAll('[data-fine-axis]').forEach(b=>b.addEventListener('click',()=>{const d=parseFloat(b.dataset.dir),a=b.dataset.fineAxis;fineJog(a==='x'?0.25*d:0,a==='y'?0.25*d:0)}));
$('xyMove').addEventListener('click',()=>jog(parseFloat($('dx').value)||0,parseFloat($('dy').value)||0,0));
$('xyBack').addEventListener('click',()=>jog(-(parseFloat($('dx').value)||0),-(parseFloat($('dy').value)||0),0));
$('boundaryToggle').addEventListener('click',async()=>{
  const currentlyOn=S.boundary_enabled!==false;
  if(currentlyOn && !confirm('Turn OFF measured X/Y boundary protection? Motion can leave the validated workspace.'))return;
  try{rs(await api('/boundary',{enabled:!currentlyOn}))}catch(e){alert(e.message)}
});
$('home').addEventListener('click',async()=>{try{rs(await api('/home',{}))}catch(e){alert(e.message)}});
$('stop').addEventListener('click',async()=>{try{rs(await api('/stop',{}))}catch(e){alert(e.message)}});
$('record').addEventListener('click',async()=>{
  try{rs(await api('/boundary_test_record',{}))}catch(e){alert(e.message)}
});
$('newBoundaryTest').addEventListener('click',async()=>{
  if(!confirm('Start a completely new boundary test session? The old measured envelope stays as reference.'))return;
  try{rs(await api('/boundary_test_new',{}))}catch(e){alert(e.message)}
});

const armCanvas=$('arm3d');
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
    const feed=Math.max(60,Math.min(5000,parseFloat($('moveFeed').value)||2500));
    const z=parseFloat($('targetZ').value);
    const tz=Number.isFinite(z)?z:S.z;
    animateCommandedPose({x,y,z:tz},feed);
    const finalState=await api('/goto',{x,y,z:tz,feed});
    stopMotionAnimation(); rs(finalState);
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

// clean fitted boundary trace: upper line -> smooth cap -> lower line
const cleanFit=[];
for(let i=0;i<=80;i++){
  const x=FIT_UPPER.x0+(FIT_UPPER.x1-FIT_UPPER.x0)*i/80;
  cleanFit.push({x:x,y:FIT_UPPER.m*x+FIT_UPPER.b});
}
smoothCapPoints().slice(1).forEach(q=>cleanFit.push(q));
for(let i=1;i<=80;i++){
  const x=FIT_LOWER.x1+(FIT_LOWER.x0-FIT_LOWER.x1)*i/80;
  cleanFit.push({x:x,y:FIT_LOWER.m*x+FIT_LOWER.b});
}
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
setInterval(refresh,800);refresh();updateKinematics();draw();drawCartesianWorkspace();
</script>'''

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def reply(self,obj,code=200):
        data=json.dumps(obj).encode(); self.send_response(code); self.send_header("Content-Type","application/json"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_GET(self):
        if self.path=="/":
            data=HTML.encode(); self.send_response(200); self.send_header("Content-Type","text/html; charset=utf-8"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
        elif self.path=="/state": self.reply(state_payload())
        elif self.path=="/endstops":
            with lock:
                x,y,z=endstops()
            self.reply({"x":x,"y":y,"z":z})
        elif self.path=="/config":
            with lock:
                lines=read_config()
            self.reply({"lines":lines})
        else: self.send_error(404)
    def do_POST(self):
        n=int(self.headers.get("Content-Length","0")); body=json.loads(self.rfile.read(n) or b"{}")

        # STOP must not wait for the motion lock held by a homing request.
        if self.path=="/stop":
            stop_event.set()
            state["message"]="STOP requested — Klipper emergency stop"
            state["homed"]=False
            try:
                _moon_post("/printer/emergency_stop", timeout=3)
            except Exception:
                pass
            self.reply(state_payload())
            return

        try:
            with lock:
                state["busy"]=True
                if self.path=="/boundary_test_new":
                    boundary_test_session["id"] += 1
                    boundary_test_session["started_at"] = time.time()
                    boundary_test_session["points"].clear()
                    state["message"] = f"Boundary test session {boundary_test_session['id']} started"
                elif self.path=="/boundary_test_record":
                    if not state["homed"]: raise RuntimeError("Home first")
                    boundary_test_session["points"].append(_boundary_test_point())
                    state["message"] = f"Recorded boundary point {len(boundary_test_session['points'])}"
                elif self.path=="/boundary":
                    state["boundary_enabled"]=bool(body.get("enabled", True))
                    state["message"]=("Boundary protection ON" if state["boundary_enabled"] else "WARNING: Boundary protection OFF")
                elif self.path=="/home":
                    state["message"]="Homing..."; precise_home(); state["message"]="Precision home complete"
                elif self.path=="/jog":
                    if not state["homed"]: raise RuntimeError("Home first")
                    state["message"]="Jog complete"
                    feed=max(60.0,min(5000.0,float(body.get("feed",NORMAL_MOVE_FEED_DEFAULT))))
                    jog_counts(float(body.get("x",0)),float(body.get("y",0)),float(body.get("z",0)),feed=feed)
                elif self.path=="/fine_jog":
                    if not state["homed"]: raise RuntimeError("Home first")
                    state["message"]="Fine jog complete"
                    jog_counts(float(body.get("x",0)),float(body.get("y",0)),0.0,feed=120)
                    if not state["message"].startswith("BOUNDARY"):
                        state["message"]="Fine jog complete"
                elif self.path=="/dance_smooth":
                    if not state["homed"]: raise RuntimeError("Home first")
                    state["boundary_enabled"]=True
                    stop_event.clear()
                    feed=max(300.0,min(3000.0,float(body.get("feed",1200.0))))
                    accel=max(20.0,min(200.0,float(body.get("accel",DANCE_ACCEL_DEFAULT_MM_S2))))
                    batch=0
                    send(f"M204 P{accel:.0f} T{accel:.0f}", timeout=15)
                    try:
                        while True:
                            check_stop()
                            batch += 1
                            home_z=state["home_z_counts"] if state["home_z_counts"] is not None else state["z"]
                            state["message"]=(f"Dance continuous — batch {batch}, F{feed:.0f}, "
                                              f"a={accel:.0f} mm/s^2")
                            waypoints=smooth_dance_waypoints(home_z,cycles=10,arm_steps=1,base_steps=2)
                            queue_count_waypoints(waypoints,feed=feed)
                            check_stop()
                            state["message"]=f"Dance batch {batch} complete — precision homing"
                            precise_home(clear_stop=False)
                            check_stop()
                            # Reapply the dance acceleration after homing because homing may
                            # use/restore other motion settings.
                            send(f"M204 P{accel:.0f} T{accel:.0f}", timeout=15)
                    finally:
                        # Never leave the controller at the temporary dance acceleration.
                        try:
                            send(f"M204 P{NORMAL_ACCEL_MM_S2:.0f} T{NORMAL_ACCEL_MM_S2:.0f}", timeout=15)
                        except Exception:
                            pass
                elif self.path=="/goto":
                    if not state["homed"]: raise RuntimeError("Home first")
                    tx=float(body.get("x",state["x"]))
                    ty=float(body.get("y",state["y"]))
                    tz=float(body.get("z",state["z"]))
                    if tx < 0 or ty < 0 or tz < 0:
                        raise RuntimeError("Target X/Y/Z must be non-negative")
                    feed=max(60.0,min(5000.0,float(body.get("feed",NORMAL_MOVE_FEED_DEFAULT))))
                    state["message"]=f"Moving to X={tx:.1f}, Y={ty:.1f}, Z={tz:.1f} at F{feed:.0f}"
                    jog_counts(tx-state["x"],ty-state["y"],tz-state["z"],feed=feed)
                    if not state["message"].startswith("BOUNDARY"):
                        state["message"]=f"Reached X={state['x']:.1f}, Y={state['y']:.1f}, Z={state['z']:.1f}"
                elif self.path=="/goto_cart":
                    if not state["homed"]: raise RuntimeError("Home first")
                    target_y=float(body["y"])
                    target_z=float(body["z"])
                    feed=max(60.0,min(5000.0,float(body.get("feed",NORMAL_MOVE_FEED_DEFAULT))))
                    seed_lower,seed_crank=joint_angles_from_mapper_counts(state["x"],state["y"])
                    q_lower,q_crank=inverse_side(target_y,target_z,seed_lower,seed_crank,assembly=1)
                    hx=state["home_x_counts"]
                    hy=state["home_y_counts"]
                    tx=hx+(q_lower-HOME_AC_DEGREES)/X_JOINT_DIRECTION
                    ty=hy+(q_crank-HOME_QY_DEGREES)/Y_JOINT_DIRECTION
                    if tx < 0 or ty < 0:
                        raise RuntimeError("Cartesian target maps outside non-negative joint coordinates")
                    state["message"]=f"Cartesian move to Tool Y={target_y:.1f}, Z={target_z:.1f}"
                    jog_counts(tx-state["x"],ty-state["y"],0.0,feed=feed)
                    if not state["message"].startswith("BOUNDARY"):
                        state["message"]=f"Reached Tool Y={target_y:.1f}, Z={target_z:.1f}"
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

if __name__=="__main__":
    wait_for_controller()
    register_gcode_axes()
    send("M17")
    state["message"]="Connected — Klipper/Moonraker ready"
    url=f"http://{HOST}:{HTTP_PORT}"
    print("Robot mapper:",url)
    print("Klipper backend via local Moonraker.")
    try: ThreadingHTTPServer((HOST,HTTP_PORT),Handler).serve_forever()
    except KeyboardInterrupt: pass
    finally:
        try: send("M18")
        except Exception: pass