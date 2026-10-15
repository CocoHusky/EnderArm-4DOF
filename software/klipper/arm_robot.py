import math
import importlib.util
import sys
import os

# Keep one authoritative physical geometry source shared with the UI.
# Klipper itself already owns a package named "kinematics", so load the robot
# geometry by absolute filename under a unique module name.
_KIN_PATH = os.environ.get('ENDERARM_KINEMATICS',
                          os.path.expanduser('~/arm-control/kinematics.py'))
_spec = importlib.util.spec_from_file_location("enderarm_kinematics", _KIN_PATH)
_arm_kin = importlib.util.module_from_spec(_spec)
sys.modules["enderarm_kinematics"] = _arm_kin
_spec.loader.exec_module(_arm_kin)
_boundary_spec = importlib.util.spec_from_file_location(
    'enderarm_boundary', os.path.join(os.path.dirname(_KIN_PATH), 'boundary_model.py'))
_arm_boundary = importlib.util.module_from_spec(_boundary_spec)
_boundary_spec.loader.exec_module(_arm_boundary)
HOME_AC_DEGREES = _arm_kin.HOME_AC_DEGREES
HOME_QY_DEGREES = _arm_kin.HOME_QY_DEGREES
full_linkage_kinematics = _arm_kin.full_linkage_kinematics
inverse_side = _arm_kin.inverse_side
tool_is_outwardmost = _arm_kin.tool_is_outwardmost

# Native robot-arm motion commands for Klipper.
# Supports:
# - linear slide on joint_4 / D axis (Ender-3 Y carriage geometry)
# - simultaneous independent-endstop X/Y homing
# - Z/base homing
# - planner-coordinated A/B/C/D motion after home

class _XYHomingToolhead:
    def __init__(self, printer, xjoint, yjoint):
        self.printer = printer
        self.x = xjoint
        self.y = yjoint
        self.homing_accel = 40.0

    def _real_toolhead(self):
        return self.printer.lookup_object("toolhead")

    def _align_time(self):
        th = self._real_toolhead()
        base = th.get_last_move_time()
        t = max(base, self.x.next_cmd_time, self.y.next_cmd_time)
        if t > base:
            th.dwell(t - base)
        self.x.next_cmd_time = t
        self.y.next_cmd_time = t
        return t

    def flush_step_generation(self):
        self._real_toolhead().flush_step_generation()

    def get_position(self):
        return [self.x.commanded_pos, self.y.commanded_pos, 0., 0.]

    def set_position(self, newpos, homing_axes=""):
        self.x.do_set_position(newpos[0])
        self.y.do_set_position(newpos[1])

    def get_last_move_time(self):
        return self._align_time()

    def dwell(self, delay):
        t = self._align_time() + max(0., delay)
        self.x.next_cmd_time = t
        self.y.next_cmd_time = t

    def drip_move(self, newpos, speed, drip_completion):
        start = self._align_time()
        xend = self.x._submit_move(start, newpos[0], speed, self.homing_accel)
        yend = self.y._submit_move(start, newpos[1], speed, self.homing_accel)
        end = max(xend, yend)
        self.x.motion_queuing.drip_update_time(start, end, drip_completion)
        self.x.motion_queuing.wipe_trapq(self.x.trapq)
        self.y.motion_queuing.wipe_trapq(self.y.trapq)

    def get_kinematics(self):
        return self

    def get_steppers(self):
        return self.x.steppers + self.y.steppers

    def calc_position(self, stepper_positions):
        return [
            stepper_positions[self.x.rail.get_name()],
            stepper_positions[self.y.rail.get_name()],
            0.
        ]


class _ABCDHomingToolhead:
    """Concurrent A/B/C homing with independent endstop triggers."""
    def __init__(self, printer, ajoint, bjoint, cjoint, djoint):
        self.printer=printer
        self.a=ajoint; self.b=bjoint; self.c=cjoint; self.d=djoint
        self.homing_accel=40.0
        self.axis_speeds=(6.0,6.0,6.0,8.0)

    def _real_toolhead(self):
        return self.printer.lookup_object("toolhead")

    def _align_time(self):
        th=self._real_toolhead()
        base=th.get_last_move_time()
        t=max(base,self.a.next_cmd_time,self.b.next_cmd_time,self.c.next_cmd_time,self.d.next_cmd_time)
        if t>base:
            th.dwell(t-base)
        self.a.next_cmd_time=t; self.b.next_cmd_time=t; self.c.next_cmd_time=t; self.d.next_cmd_time=t
        return t

    def flush_step_generation(self):
        self._real_toolhead().flush_step_generation()

    def get_position(self):
        return [self.a.commanded_pos,self.b.commanded_pos,self.c.commanded_pos,self.d.commanded_pos]

    def set_position(self,newpos,homing_axes=""):
        self.a.do_set_position(newpos[0])
        self.b.do_set_position(newpos[1])
        self.c.do_set_position(newpos[2])
        self.d.do_set_position(newpos[3])

    def get_last_move_time(self):
        return self._align_time()

    def dwell(self,delay):
        t=self._align_time()+max(0.,delay)
        self.a.next_cmd_time=t; self.b.next_cmd_time=t; self.c.next_cmd_time=t; self.d.next_cmd_time=t

    def drip_move(self,newpos,speed,drip_completion):
        start=self._align_time()
        sa,sb,sc,sd=self.axis_speeds
        aend=self.a._submit_move(start,newpos[0],sa,self.homing_accel)
        bend=self.b._submit_move(start,newpos[1],sb,self.homing_accel)
        cend=self.c._submit_move(start,newpos[2],sc,self.homing_accel)
        dend=self.d._submit_move(start,newpos[3],sd,self.homing_accel)
        end=max(aend,bend,cend,dend)
        self.a.motion_queuing.drip_update_time(start,end,drip_completion)
        self.a.motion_queuing.wipe_trapq(self.a.trapq)
        self.b.motion_queuing.wipe_trapq(self.b.trapq)
        self.c.motion_queuing.wipe_trapq(self.c.trapq)
        self.d.motion_queuing.wipe_trapq(self.d.trapq)

    def get_kinematics(self):
        return self

    def get_steppers(self):
        return self.a.steppers+self.b.steppers+self.c.steppers+self.d.steppers

    def calc_position(self,stepper_positions):
        return [
            stepper_positions[self.a.rail.get_name()],
            stepper_positions[self.b.rail.get_name()],
            stepper_positions[self.c.rail.get_name()]
        ]


class ArmRobot:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command("ARM_HOME", self.cmd_ARM_HOME,
                                    desc="Home linear slide, then arm joints")
        self.gcode.register_command("ARM_HOME_LINEAR", self.cmd_ARM_HOME_LINEAR,
                                    desc="Home only linear slide")
        self.gcode.register_command("ARM_LINEAR_MODE", self.cmd_ARM_LINEAR_MODE,
                                    desc="Enable or disable the optional linear rail")
        self.gcode.register_command("ARM_BOUNDARY", self.cmd_ARM_BOUNDARY,
                                    desc="Enable or disable measured A/B workspace enforcement")
        self.gcode.register_command("ARM_HOME_TEST", self.cmd_ARM_HOME_TEST,
                                    desc="Test native concurrent X/Y homing")
        self.gcode.register_command("ARM_TUNE", self.cmd_ARM_TUNE,
                                    desc="Update live robot motion limits without re-registering axes")
        self.gcode.register_command("ARM_MOVE_JOINT", self.cmd_ARM_MOVE_JOINT,
                                    desc="Native planned A/B/C/D robot move")
        self.gcode.register_command("ARM_MOVE_TOOL", self.cmd_ARM_MOVE_TOOL,
                                    desc="Native Cartesian tool move")
        self.gcode.register_command("ARM_DANCE", self.cmd_ARM_DANCE,
                                    desc="Start native continuous robot dance")
        self.gcode.register_command("ARM_DANCE_STOP", self.cmd_ARM_DANCE_STOP,
                                    desc="Stop adding native dance motion")
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        # Native planner state.  The reactor timer runs inside Klipper, so no
        # Moonraker/browser latency participates in trajectory generation.
        self.homed = False
        # This controls only the measured software A/B envelope.  Homing,
        # linkage closure, base-angle, rail, and configured hardware limits
        # remain enforced in both modes.
        self.boundary_enabled = True
        self.home_a = 3.5
        self.home_b = 3.5
        self.home_c = 2.5
        self.arm_velocity = 80.0
        self.arm_accel = 250.0
        self.arm_corner = 18.0
        self.slide_velocity = 25.0
        self.slide_accel = 100.0
        self.slide_corner = 5.0
        # The D rail remains a registered software coordinate so the planner/UI
        # keep a stable A/B/C/D shape. Hardware use is runtime-selectable.
        self.linear_enabled = False

        self.dance_active = False
        self.dance_cycles_remaining = 0
        self.dance_cycle = 0
        self.dance_sample = 0
        self.dance_samples = 720
        self.dance_cycle_time = 8.0
        self.dance_phase_time = 0.0
        self.dance_queued_until = 0.0
        self.dance_settings = None
        self.pending_tune = None
        self.dance_direct_time = 0.0
        self.dance_elapsed = 0.0
        self.dance_dt = 0.010
        self.dance_ramp_time = 2.0
        self.dance_timer = self.reactor.register_timer(
            self._dance_timer_cb, self.reactor.NEVER)

    def _handle_connect(self):
        self.x = self.printer.lookup_object("manual_stepper joint_x")
        self.y = self.printer.lookup_object("manual_stepper joint_y")
        self.z = self.printer.lookup_object("manual_stepper joint_z")
        self.slide = self.printer.lookup_object("manual_stepper joint_4")
        self.toolhead = self.printer.lookup_object("toolhead")
        self.xytool = _XYHomingToolhead(self.printer, self.x, self.y)
        self.abcdtool = _ABCDHomingToolhead(self.printer, self.x, self.y, self.z, self.slide)

    def _unregister_axis(self, joint):
        if joint.axis_gcode_id is None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.remove_extra_axis(joint)
        joint.axis_gcode_id = None

    def _unregister_all(self):
        for joint in (self.x, self.y, self.z, self.slide):
            self._unregister_axis(joint)

    def _register_axis(self, joint, axis, velocity, accel, corner):
        self._unregister_axis(joint)
        toolhead = self.printer.lookup_object("toolhead")
        joint.axis_gcode_id = axis
        joint.instant_corner_v = corner
        joint.gaxis_limit_velocity = velocity
        joint.gaxis_limit_accel = accel
        toolhead.add_extra_axis(joint, joint.commanded_pos)

    def _register_all(self):
        self._register_axis(self.x, "A", 220., 1800., 12.)
        self._register_axis(self.y, "B", 220., 1800., 12.)
        self._register_axis(self.z, "C", 220., 1800., 12.)
        # D is linear carriage position in millimeters.
        self._register_axis(self.slide, "D", 100., 500., 5.)

    def _apply_tune_values(self, vals):
        arm_v, arm_a, arm_c, slide_v, slide_a, slide_c = vals
        for joint in (self.x, self.y, self.z):
            joint.gaxis_limit_velocity = arm_v
            joint.gaxis_limit_accel = arm_a
            joint.instant_corner_v = arm_c
        self.slide.gaxis_limit_velocity = slide_v
        self.slide.gaxis_limit_accel = slide_a
        self.slide.instant_corner_v = slide_c
        self.arm_velocity = arm_v
        self.arm_accel = arm_a
        self.arm_corner = arm_c
        self.slide_velocity = slide_v
        self.slide_accel = slide_a
        self.slide_corner = slide_c

    def cmd_ARM_TUNE(self, gcmd):
        """Tune immediately when idle, or atomically at next Dance cycle."""
        vals = (
            gcmd.get_float("ARM_VELOCITY", self.arm_velocity, above=0.),
            gcmd.get_float("ARM_ACCEL", self.arm_accel, above=0.),
            gcmd.get_float("ARM_CORNER", self.arm_corner, minval=0.),
            gcmd.get_float("SLIDE_VELOCITY", self.slide_velocity, above=0.),
            gcmd.get_float("SLIDE_ACCEL", self.slide_accel, above=0.),
            gcmd.get_float("SLIDE_CORNER", self.slide_corner, minval=0.),
        )
        if self.dance_active:
            self.pending_tune = vals
            gcmd.respond_info(
                "ARM_TUNE queued for next Dance cycle: "
                "arm=%.2fdeg/s %.2fdeg/s^2 corner=%.2fdeg/s | "
                "slide=%.2fmm/s %.2fmm/s^2 corner=%.2fmm/s" % vals)
            return
        self._apply_tune_values(vals)
        gcmd.respond_info(
            "ARM_TUNE applied: arm=%.2fdeg/s %.2fdeg/s^2 corner=%.2fdeg/s | "
            "slide=%.2fmm/s %.2fmm/s^2 corner=%.2fmm/s" % vals)


    # ------------------------------------------------------------------
    # Native robot trajectory planner
    # ------------------------------------------------------------------
    def _axis_indices(self):
        result = {}
        for idx, axis in enumerate(self.toolhead.get_extra_axes()):
            if axis is None or not hasattr(axis, "get_axis_gcode_id"):
                continue
            aid = axis.get_axis_gcode_id()
            if aid in ("A", "B", "C", "D"):
                result[aid] = idx
        if len(result) != 4:
            raise self.gcode.error(
                "Robot A/B/C/D axes are not registered: %s" % (result,))
        return result

    def _joint_position(self):
        idx = self._axis_indices()
        pos = self.toolhead.get_position()
        return (pos[idx["A"]], pos[idx["B"]],
                pos[idx["C"]], pos[idx["D"]])

    def _build_boundary_poly(self):
        return _arm_boundary.build_boundary_poly()

    def _point_in_poly(self, x, y, poly):
        inside = False
        j = len(poly)-1
        for i in range(len(poly)):
            xi,yi = poly[i]; xj,yj = poly[j]
            if ((yi > y) != (yj > y)):
                xc = (xj-xi)*(y-yi)/(yj-yi)+xi
                if x < xc:
                    inside = not inside
            j = i
        return inside

    def _seg_dist(self, px,py,ax,ay,bx,by):
        vx,vy=bx-ax,by-ay; wx,wy=px-ax,py-ay
        vv=vx*vx+vy*vy
        if vv <= 1e-12:
            return math.hypot(px-ax,py-ay)
        t=max(0.0,min(1.0,(wx*vx+wy*vy)/vv))
        qx,qy=ax+t*vx,ay+t*vy
        return math.hypot(px-qx,py-qy)

    def _boundary_score(self, x, y):
        if not hasattr(self, "_boundary_poly"):
            self._boundary_poly = self._build_boundary_poly()
        p=self._boundary_poly
        d=min(self._seg_dist(x,y,*p[i],*p[(i+1)%len(p)])
              for i in range(len(p)))
        return d if self._point_in_poly(x,y,p) else -d

    def _check_joint_path(self, a0,b0,a1,b1):
        # Same 5-degree inward yield used by the UI.  The physical home release
        # pose is allowed to start inside the generic yield as long as movement
        # improves clearance.
        if math.hypot(a1-self.home_a,b1-self.home_b) <= 0.35:
            return
        length=math.hypot(a1-a0,b1-b0)
        if length <= 1e-9:
            return
        start_score=self._boundary_score(a0,b0)
        required=5.0 if start_score >= 5.0 else start_score-0.05
        n=max(2,int(length/0.5)+1)
        last_score=start_score
        for i in range(1,n+1):
            u=float(i)/n
            a=a0+(a1-a0)*u
            b=b0+(b1-b0)*u
            sc=self._boundary_score(a,b)
            if sc < 0.0 or sc < required:
                raise self.gcode.error(
                    "Robot A/B path leaves measured safe boundary at "
                    "A=%.2f B=%.2f" % (a,b))
            last_score=sc

    def _queue_joint_target(self, a, b, c, d,
                            arm_velocity=None, slide_velocity=None,
                            check_boundary=True, duration_override=None):
        if not self.homed:
            raise self.gcode.error("Robot must be homed first")
        idx=self._axis_indices()
        old=self.toolhead.get_position()
        cur_a,cur_b,cur_c,cur_d = (
            old[idx["A"]], old[idx["B"]], old[idx["C"]], old[idx["D"]])
        a=float(a); b=float(b); c=float(c)
        d=float(d) if self.linear_enabled else 0.0
        if not 0.0 <= d <= 235.0:
            raise self.gcode.error("Linear D target %.3f outside 0..235mm" % d)
        base_min=self.home_c
        base_max=self.home_c + 269.5
        if not base_min <= c <= base_max:
            raise self.gcode.error(
                "Base C target %.3f outside measured %.1f..%.1fdeg range"
                % (c,base_min,base_max))
        if check_boundary:
            self._check_joint_path(cur_a,cur_b,a,b)

        da=a-cur_a; db=b-cur_b; dc=c-cur_c; dd=d-cur_d
        arm_dist=math.sqrt(da*da+db*db+dc*dc)
        slide_dist=abs(dd)
        av=max(0.1,float(arm_velocity or self.arm_velocity))
        sv=max(0.1,float(slide_velocity or self.slide_velocity))
        minimum_duration=max(arm_dist/av if arm_dist else 0.0,
                             slide_dist/sv if slide_dist else 0.0, 0.001)
        if duration_override is None:
            duration=minimum_duration
        else:
            # Time-parameterized trajectories may request slower timing,
            # but may never force a segment faster than configured axis limits.
            duration=max(float(duration_override), minimum_duration)
        combined=math.sqrt(arm_dist*arm_dist+dd*dd)
        if combined <= 1e-9:
            return 0.0

        newpos=list(old)
        newpos[idx["A"]]=a
        newpos[idx["B"]]=b
        newpos[idx["C"]]=c
        newpos[idx["D"]]=d
        self.toolhead.move(newpos, combined/duration)
        return duration

    def cmd_ARM_BOUNDARY(self, gcmd):
        self.boundary_enabled = bool(gcmd.get_int("ENABLE", 1, minval=0,
                                                   maxval=1))
        gcmd.respond_info("Measured A/B boundary %s" % (
            "enabled" if self.boundary_enabled else "disabled"))

    def _current_q(self):
        a,b,c,d=self._joint_position()
        qx=HOME_AC_DEGREES + (a-self.home_a)
        qy=HOME_QY_DEGREES + (b-self.home_b)
        return qx,qy

    def _normalize_base_target(self, target_c):
        """Choose the equivalent yaw turn that lies in measured C travel."""
        base_min=self.home_c
        base_max=self.home_c + 269.5
        c=float(target_c)
        while c < base_min:
            c += 360.0
        while c > base_max and c-360.0 >= base_min:
            c -= 360.0
        return c

    def _tool_to_joint_target(self, x, y, z, d=None, frame="WORLD"):
        cur_a,cur_b,cur_c,cur_d=self._joint_position()
        d=(cur_d if d is None else float(d)) if self.linear_enabled else 0.0
        qx0,qy0=self._current_q()

        if frame == "ARM":
            radius=float(y)
            target_z=float(z)
            target_c=cur_c
        else:
            rel_y=float(y)-d
            tx=float(x)
            radius=math.hypot(tx,rel_y)
            if radius <= 1e-6:
                raise self.gcode.error("Tool target lies on base yaw axis")
            yaw=math.degrees(math.atan2(tx,rel_y))
            # atan2 returns -180..180°. C has nearly 270° of usable travel,
            # so negative headings such as -143° may have a valid equivalent
            # (216.8° here) inside the physical base range.
            target_c=self._normalize_base_target(self.home_c+yaw)
            target_z=float(z)

        try:
            qx,qy=inverse_side(radius,target_z,qx0,qy0)
        except ValueError as exc:
            raise self.gcode.error("IK failed: %s" % (exc,))

        # Choose equivalent angles closest to current branch.
        qx=qx0 + ((qx-qx0+180.0) % 360.0) - 180.0
        qy=qy0 + ((qy-qy0+180.0) % 360.0) - 180.0

        try:
            piv=full_linkage_kinematics(qx,qy)
            # The measured outward-tool branch is part of the software
            # workspace policy.  It must not silently remain active after the
            # operator explicitly turns Boundary OFF.
            if self.boundary_enabled and not tool_is_outwardmost(piv):
                raise self.gcode.error(
                    "IK solution violates outward-tool assembly invariant")
        except ValueError as exc:
            raise self.gcode.error("IK linkage closure failed: %s" % (exc,))

        target_a=self.home_a + (qx-HOME_AC_DEGREES)
        target_b=self.home_b + (qy-HOME_QY_DEGREES)
        return target_a,target_b,target_c,d

    def cmd_ARM_MOVE_JOINT(self, gcmd):
        if self.dance_active:
            raise gcmd.error("Stop ARM_DANCE before point-to-point moves")
        a0,b0,c0,d0=self._joint_position()
        a=gcmd.get_float("A",a0)
        b=gcmd.get_float("B",b0)
        c=gcmd.get_float("C",c0)
        d=gcmd.get_float("D",d0) if self.linear_enabled else 0.0
        duration=gcmd.get_float("TIME",None,above=0.)
        actual_duration=self._queue_joint_target(
            a,b,c,d,check_boundary=self.boundary_enabled,
            duration_override=duration)
        gcmd.respond_info(
            "ARM_MOVE_JOINT queued A=%.3f B=%.3f C=%.3f D=%.3f time=%.4fs"
            % (a,b,c,d,actual_duration))

    def cmd_ARM_MOVE_TOOL(self, gcmd):
        if self.dance_active:
            raise gcmd.error("Stop ARM_DANCE before Cartesian moves")
        frame=gcmd.get("FRAME","WORLD").upper()
        if frame not in ("WORLD","ARM"):
            raise gcmd.error("FRAME must be WORLD or ARM")
        d=gcmd.get_float("D",None) if self.linear_enabled else 0.0
        if frame == "ARM":
            y=gcmd.get_float("Y")
            z=gcmd.get_float("Z")
            a,b,c,d=self._tool_to_joint_target(0.0,y,z,d,frame)
        else:
            x=gcmd.get_float("X")
            y=gcmd.get_float("Y")
            z=gcmd.get_float("Z")
            a,b,c,d=self._tool_to_joint_target(x,y,z,d,frame)
        self._queue_joint_target(a,b,c,d,check_boundary=self.boundary_enabled)
        gcmd.respond_info(
            "ARM_MOVE_TOOL queued -> A=%.3f B=%.3f C=%.3f D=%.3f"
            % (a,b,c,d))

    def _dance_phase(self, elapsed):
        # Shared base clock with a short smooth startup ramp.  Individual axes
        # multiply this phase by different non-matching ratios.
        omega=2.0*math.pi/max(self.dance_cycle_time, 1e-6)
        ramp=max(self.dance_ramp_time, 1e-6)
        if elapsed < ramp:
            u=max(0.0,min(1.0,elapsed/ramp))
            speed_scale=3.0*u*u - 2.0*u*u*u
            theta_dot=omega*speed_scale
            theta_ddot=omega*(6.0*u - 6.0*u*u)/ramp
            theta=omega*ramp*(u*u*u - 0.5*u*u*u*u)
            return theta,theta_dot,theta_ddot
        theta=0.5*omega*ramp + omega*(elapsed-ramp)
        return theta,omega,0.0

    def _dance_state(self, elapsed):
        wps=self.dance_ab_waypoints
        durs=self.dance_ab_durations
        total=sum(durs)
        tau=elapsed % total
        seg=0
        while seg < len(durs)-1 and tau >= durs[seg]:
            tau -= durs[seg]
            seg += 1
        T=durs[seg]
        u=max(0.0,min(1.0,tau/T))
        s5=10.0*u**3 - 15.0*u**4 + 6.0*u**5
        ds=(30.0*u*u - 60.0*u**3 + 30.0*u**4)/T
        dds=(60.0*u - 180.0*u*u + 120.0*u**3)/(T*T)
        p0=wps[seg]; p1=wps[(seg+1)%len(wps)]
        da=p1[0]-p0[0]; db=p1[1]-p0[1]
        a=p0[0]+da*s5; b=p0[1]+db*s5
        va=da*ds; vb=db*ds; aa=da*dds; ab=db*dds
        qc=self.dance_c_omega*elapsed
        qd=self.dance_d_omega*elapsed
        cc=math.cos(qc); sc=math.sin(qc)
        cd=math.cos(qd); sd=math.sin(qd)
        c=self.home_c + 125.0*(1.0-cc)
        d=50.0*(1.0-cd) if self.linear_enabled else 0.0
        vc=125.0*self.dance_c_omega*sc
        vd=50.0*self.dance_d_omega*sd if self.linear_enabled else 0.0
        ac=125.0*self.dance_c_omega*self.dance_c_omega*cc
        ad=50.0*self.dance_d_omega*self.dance_d_omega*cd if self.linear_enabled else 0.0
        return (a,b,c,d),(va,vb,vc,vd),(aa,ab,ac,ad)

    def _raw_dance_point(self, t):
        return (116.3,109.9,self.home_c,0.0)

    def _snapshot_dance_settings(self):
        if self.pending_tune is not None:
            self._apply_tune_values(self.pending_tune)
            self.pending_tune = None
        self.dance_settings = (
            self.arm_velocity, self.arm_accel, self.arm_corner,
            self.slide_velocity, self.slide_accel, self.slide_corner)
        av,aa,ac,sv,sa,sc=self.dance_settings
        self.dance_ab_waypoints=(
            (116.3,109.9),
            (99.2023229039,25.4163657817),
            (129.3277682872,49.9015150689),
            (3.5,3.5))
        self.dance_ab_durations=[]
        for i,p0 in enumerate(self.dance_ab_waypoints):
            p1=self.dance_ab_waypoints[(i+1)%len(self.dance_ab_waypoints)]
            delta=max(abs(p1[0]-p0[0]),abs(p1[1]-p0[1]))
            tv=1.875*delta/max(0.90*av,0.1)
            ta=math.sqrt(5.773503*delta/max(0.90*aa,0.1))
            self.dance_ab_durations.append(max(tv,ta,0.35))
        self.dance_c_omega=0.90*min(
            av/125.0, math.sqrt(aa/125.0))
        self.dance_d_omega=0.90*min(
            sv/50.0, math.sqrt(sa/50.0))
        self.dance_cycle_time=sum(self.dance_ab_durations)

    def _dance_point(self, sample):
        t=2.0*math.pi*(float(sample)/max(1,self.dance_samples))
        return self._raw_dance_point(t)

    def _validate_dance_path(self):
        wps=self.dance_ab_waypoints
        if self.boundary_enabled:
            for i,p0 in enumerate(wps):
                p1=wps[(i+1)%len(wps)]
                self._check_joint_path(p0[0],p0[1],p1[0],p1[1])
        if self.home_c+250.0 > self.home_c+269.5:
            raise self.gcode.error("Dance base range invalid")

    def _append_axis_hermite(self, joint, print_time, p0, v0, p1, v1, dt):
        # Two constant-acceleration halves exactly match endpoint position and
        # velocity.  Velocity is continuous across every interval.
        h=0.5*dt
        dv=v1-v0
        dx=p1-p0
        a1=4.0*(dx-dt*v0)/(dt*dt) - dv/dt
        a2=3.0*dv/dt - 4.0*(dx-dt*v0)/(dt*dt)
        pmid=p0 + v0*h + 0.5*a1*h*h
        vmid=v0 + a1*h
        joint.trapq_append(joint.trapq, print_time,
                           h,0.0,0.0,
                           p0,0.0,0.0, 1.0,0.0,0.0,
                           v0,0.0,a1)
        joint.trapq_append(joint.trapq, print_time+h,
                           h,0.0,0.0,
                           pmid,0.0,0.0, 1.0,0.0,0.0,
                           vmid,0.0,a2)

    def _queue_direct_dance_interval(self):
        dt=self.dance_dt
        p0,v0,a0=self._dance_state(self.dance_elapsed)
        p1,v1,a1=self._dance_state(self.dance_elapsed+dt)

        # A/B waypoint legs are validated once when Dance starts.
        # Do not apply the generic 5-degree yield rule to every tiny interval,
        # because the intentional near-home pose is inside that yield band.
        if not self.home_c <= p1[2] <= self.home_c+269.5:
            raise self.gcode.error("Dance base path outside measured range")
        if not 0.0 <= p1[3] <= 235.0:
            raise self.gcode.error("Dance rail path outside measured range")

        joints=(self.x,self.y,self.z,self.slide) if self.linear_enabled else (self.x,self.y,self.z)
        for i,joint in enumerate(joints):
            self._append_axis_hermite(
                joint,self.dance_direct_time,p0[i],v0[i],p1[i],v1[i],dt)

        self.dance_direct_time += dt
        self.dance_elapsed += dt

        # Match normal Klipper semantics: commanded position is the end of the
        # queued buffer, while motion_report remains the actual live position.
        idx=self._axis_indices()
        pos=self.toolhead.commanded_pos
        pos[idx["A"]]=p1[0]; pos[idx["B"]]=p1[1]
        pos[idx["C"]]=p1[2]; pos[idx["D"]]=p1[3]
        self.x.commanded_pos=p1[0]
        self.y.commanded_pos=p1[1]
        self.z.commanded_pos=p1[2]
        self.slide.commanded_pos=p1[3] if self.linear_enabled else 0.0
        for joint in joints:
            joint.next_cmd_time=self.dance_direct_time

        phase_cycles=max(0.0,(self.dance_elapsed-self.dance_ramp_time)
                         / max(self.dance_cycle_time,1e-6))
        self.dance_cycle=int(phase_cycles)
        self.dance_sample=int((phase_cycles%1.0)*self.dance_samples)

    def _dance_timer_cb(self, eventtime):
        if not self.dance_active:
            return self.reactor.NEVER

        now=self.reactor.monotonic()
        est=self.toolhead.mcu.estimated_print_time(now)
        target_ahead=2.00
        max_intervals=260
        queued=0
        while (self.dance_active
               and self.dance_direct_time-est < target_ahead
               and queued < max_intervals):
            self._queue_direct_dance_interval()
            queued += 1

        # Direct trapq Dance must NEVER call toolhead.dwell() from this timer.
        # dwell() is a blocking/high-level print-time operation and can prevent
        # this refill callback from running again before the buffer drains.
        # motion_queuing is the owner of low-level trapq flushing: announce the
        # future horizon and let its background flush timer generate steps.
        self.x.motion_queuing.note_mcu_movequeue_activity(
            self.dance_direct_time, is_step_gen=True)
        self.dance_queued_until=self.dance_direct_time

        if self.dance_cycles_remaining > 0:
            completed=max(0.0,(self.dance_elapsed-self.dance_ramp_time)
                          / max(self.dance_cycle_time,1e-6))
            if completed >= self.dance_cycles_remaining:
                self.dance_active=False
                return self.reactor.NEVER
        return now+0.010

    def cmd_ARM_DANCE(self, gcmd):
        if not self.homed:
            raise gcmd.error("Robot must be homed first")
        if self.dance_active:
            raise gcmd.error("ARM_DANCE is already running")
        cycles_raw=gcmd.get("CYCLES","INF").upper()
        if cycles_raw in ("INF","INFINITE","FOREVER"):
            cycles=-1
        else:
            try:
                cycles=int(cycles_raw)
            except ValueError:
                raise gcmd.error("CYCLES must be a positive integer or INF")
            if cycles < 1:
                raise gcmd.error("CYCLES must be >=1")
        samples=gcmd.get_int("SAMPLES",720,minval=180,maxval=1200)
        self.dance_samples=samples
        self.dance_cycles_remaining=cycles
        self.dance_cycle=0
        self.dance_sample=0
        self.dance_active=True
        self._snapshot_dance_settings()
        self._validate_dance_path()
        sa,sb,sc,sd=self._dance_point(0)
        self._queue_joint_target(
            sa,sb,sc,sd,
            arm_velocity=self.arm_velocity,
            slide_velocity=self.slide_velocity,
            check_boundary=self.boundary_enabled)
        # Flush the one smooth centering move. Direct sine motion begins after
        # that exact print-time, from zero phase velocity.
        self.dance_direct_time=self.toolhead.get_last_move_time()
        self.dance_elapsed=0.0
        self.dance_queued_until=self.dance_direct_time
        now=self.reactor.monotonic()
        self.reactor.update_timer(self.dance_timer, now+0.010)
        gcmd.respond_info(
            "ARM_DANCE started CYCLES=%s SAMPLES=%d native Klipper planner"
            % (cycles_raw,samples))

    def cmd_ARM_DANCE_STOP(self, gcmd):
        self.dance_active=False
        self.reactor.update_timer(self.dance_timer,self.reactor.NEVER)
        # Restore ordinary configured junction limits after the buffered Dance
        # motion drains.
        for joint in (self.x,self.y,self.z):
            joint.instant_corner_v=self.arm_corner
        self.slide.instant_corner_v=self.slide_corner
        gcmd.respond_info(
            "ARM_DANCE stopped adding motion; already queued motion will finish")

    def get_status(self, eventtime):
        try:
            a,b,c,d=self._joint_position()
        except Exception:
            a=b=c=d=0.0
        # Lightweight arm-local queue estimate for status only.
        # The Debug subsystem reads Klipper toolhead buffer_time directly.
        now=self.reactor.monotonic()
        if self.dance_active and hasattr(self, "toolhead"):
            try:
                et=self.toolhead.mcu.estimated_print_time(now)
                queue_ahead=max(0.0,self.dance_direct_time-et)
            except Exception:
                queue_ahead=max(0.0,self.dance_queued_until-now)
        else:
            queue_ahead=0.0
        return {
            "homed": self.homed,
            "linear_enabled": self.linear_enabled,
            "dance_active": self.dance_active,
            "dance_cycle": self.dance_cycle,
            "dance_sample": self.dance_sample,
            "dance_samples": self.dance_samples,
            "dance_cycle_time": self.dance_cycle_time,
            "dance_sample_dt_s": self.dance_cycle_time/max(1,self.dance_samples),
            "queue_ahead_s": queue_ahead,
            "queue_target_s": 1.35,
            "base_min_deg": self.home_c,
            "base_max_deg": self.home_c + 269.5,
            "slide_min_mm": 0.0,
            "slide_max_mm": 235.0,
            "A": a, "B": b, "C": c, "D": d,
            "arm_velocity": self.arm_velocity,
            "arm_accel": self.arm_accel,
            "arm_corner": self.arm_corner,
            "slide_velocity": self.slide_velocity,
            "slide_accel": self.slide_accel,
            "slide_corner": self.slide_corner,
            "tune_pending": self.pending_tune is not None,
            "home_timing": getattr(self, "home_timing", {}),
        }

    def _prepare_manual(self):
        self._unregister_all()
        for joint in (self.x, self.y, self.z):
            joint.do_enable(1)
        if self.linear_enabled:
            self.slide.do_enable(1)
        else:
            self.slide.do_enable(0)

    def _home_xy(self, target, speed, accel):
        self.xytool.homing_accel = accel
        endstops = self.x.rail.get_endstops() + self.y.rail.get_endstops()
        homing = self.printer.lookup_object("homing")
        return homing.manual_home(
            self.xytool, endstops, [target, target, 0., 0.],
            speed, True, True, True)

    def _move_xy(self, xpos, ypos, speed, accel):
        self.x.do_move(xpos, speed, accel, False)
        self.y.do_move(ypos, speed, accel, False)
        self.x.sync_print_time()
        self.y.sync_print_time()

    def _home_abcd(self, atarget, btarget, ctarget, dtarget,
                   arm_speed, base_speed, slide_speed, accel):
        self.abcdtool.homing_accel=accel
        self.abcdtool.axis_speeds=(arm_speed,arm_speed,base_speed,slide_speed)
        endstops=(self.x.rail.get_endstops()
                  + self.y.rail.get_endstops()
                  + self.z.rail.get_endstops()
                  + self.slide.rail.get_endstops())
        homing=self.printer.lookup_object("homing")
        ta=abs(atarget)/max(arm_speed,0.1)
        tb=abs(btarget)/max(arm_speed,0.1)
        tc=abs(ctarget)/max(base_speed,0.1)
        td=abs(dtarget)/max(slide_speed,0.1)
        move_t=max(ta,tb,tc,td,0.001)
        xyz_d=math.sqrt(atarget*atarget+btarget*btarget+ctarget*ctarget)
        sample_speed=xyz_d/move_t
        return homing.manual_home(
            self.abcdtool,endstops,[atarget,btarget,ctarget,dtarget],
            sample_speed,True,True,True)

    def _release_abcd(self, a_amount, b_amount, c_amount, d_amount,
                      arm_speed=5.0, base_speed=5.0,
                      slide_speed=6.0, accel=30.0):
        for joint in (self.x,self.y,self.z,self.slide):
            joint.do_set_position(0.)
        start=max(self.toolhead.get_last_move_time(),
                  self.x.next_cmd_time,self.y.next_cmd_time,
                  self.z.next_cmd_time,self.slide.next_cmd_time)
        aend=self.x._submit_move(start,a_amount,arm_speed,accel)
        bend=self.y._submit_move(start,b_amount,arm_speed,accel)
        cend=self.z._submit_move(start,c_amount,base_speed,accel)
        dend=self.slide._submit_move(start,d_amount,slide_speed,accel)
        end=max(aend,bend,cend,dend)
        self.x.motion_queuing.note_mcu_movequeue_activity(end)
        self.x.sync_print_time(); self.y.sync_print_time()
        self.z.sync_print_time(); self.slide.sync_print_time()

    def _home_z(self, target, speed, accel):
        self.z.do_homing_move(target, speed, accel, True, True, True)

    def _release_xy(self, amount, speed=20.0, accel=100.0):
        self.x.do_set_position(0.)
        self.y.do_set_position(0.)
        self._move_xy(amount, amount, speed, accel)

    def _release_z(self, amount):
        self.z.do_set_position(0.)
        self.z.do_move(amount, 5.0, 30.0, True)

    def _home_slide_native(self, speed=5.0, accel=30.0):
        if not self.linear_enabled:
            raise self.gcode.error("Linear D is disabled in Settings")
        # One slow touch only; no coarse/fine second pass.
        self.slide.do_set_position(0.)
        self.slide.do_homing_move(-250.0, speed, accel, True, True, True)
        self.slide.do_set_position(0.)
        self.slide.do_move(3.0, speed, accel, True)
        # Released switch position is logical D=0.
        self.slide.do_set_position(0.)

    def cmd_ARM_HOME_LINEAR(self, gcmd):
        if not self.linear_enabled:
            raise gcmd.error("Linear D is disabled in Settings")
        speed=min(gcmd.get_float("SPEED",5.0,above=0.),8.0)
        accel=min(gcmd.get_float("ACCEL",30.0,above=0.),50.0)
        start=self.reactor.monotonic()
        self._unregister_all()
        self.slide.do_enable(1)
        self._home_slide_native(speed,accel)
        self._register_all()
        self.homed=False
        gcmd.respond_info(
            "ARM_HOME_LINEAR complete in %.3fs; D=0 at released home; full ARM_HOME required"
            % (self.reactor.monotonic()-start))

    def cmd_ARM_LINEAR_MODE(self, gcmd):
        if self.dance_active:
            raise gcmd.error("Stop ARM_DANCE before changing linear rail mode")
        enabled=bool(gcmd.get_int("ENABLE",1 if self.linear_enabled else 0,minval=0,maxval=1))
        if enabled == self.linear_enabled:
            gcmd.respond_info("Linear rail already %s" % ("ENABLED" if enabled else "DISABLED"))
            return

        self._unregister_all()
        self.linear_enabled=enabled
        # Mode changes invalidate world coordinates. Require a new home.
        self.homed=False
        self.slide.do_set_position(0.)
        if enabled:
            # Do not move the rail here. Homing will establish physical D=0.
            self.slide.do_enable(0)
        else:
            self.slide.do_enable(0)
        self._register_all()
        gcmd.respond_info(
            "Linear rail %s; D=%s; ARM_HOME required"
            % ("ENABLED" if enabled else "DISABLED",
               "active after homing" if enabled else "fixed at 0"))

    def cmd_ARM_HOME_TEST(self, gcmd):
        dist = gcmd.get_float("DIST", 15.0, above=0.)
        speed = min(gcmd.get_float("SPEED", 5.0, above=0.), 8.0)
        accel = min(gcmd.get_float("ACCEL", 30.0, above=0.), 50.0)
        self._prepare_manual()
        self.x.do_set_position(0.)
        self.y.do_set_position(0.)
        start = self.reactor.monotonic()
        self._home_xy(-dist, speed, accel)
        elapsed = self.reactor.monotonic() - start
        self._release_xy(3.5)
        self._register_all()
        gcmd.respond_info("ARM_HOME_TEST X/Y complete in %.3fs" % elapsed)

    def cmd_ARM_HOME(self, gcmd):
        start = self.reactor.monotonic()
        home_speed = min(gcmd.get_float('SPEED', 30.0, above=0.), 30.0)
        accel = min(gcmd.get_float('ACCEL', 100.0, above=0.), 120.0)
        timings = {"speed_deg_s": home_speed, "accel_deg_s2": accel}
        self.home_timing = {"stage": "prepare", **timings}

        self._prepare_manual()

        # A/B side joints: one touch, then release 50% farther from the
        # switches. Reset the logical coordinates to the calibrated home pose
        # afterward so the workspace/kinematics reference does not shift.
        self.x.do_set_position(0.)
        self.y.do_set_position(0.)
        self.home_timing = {"stage": "xy_seek", **timings}
        stage_start = self.reactor.monotonic()
        self._home_xy(-220.0, home_speed, accel)
        timings["xy_seek_s"] = self.reactor.monotonic() - stage_start
        self.home_timing = {"stage": "xy_release", **timings}
        stage_start = self.reactor.monotonic()
        self._release_xy(5.25, home_speed, accel)
        timings["xy_release_s"] = self.reactor.monotonic() - stage_start
        self.x.do_set_position(3.5)
        self.y.do_set_position(3.5)

        # C / base-Z rotation: release 7.5° from the switch
        # extra clearance) while retaining the calibrated logical coordinate.
        self.z.do_set_position(0.)
        self.home_timing = {"stage": "z_seek", **timings}
        stage_start = self.reactor.monotonic()
        self.z.do_homing_move(-320.0, home_speed, accel, True, True, True)
        timings["z_seek_s"] = self.reactor.monotonic() - stage_start
        self.home_timing = {"stage": "z_release", **timings}
        stage_start = self.reactor.monotonic()
        self.z.do_set_position(0.)
        self.z.do_move(7.5, home_speed, accel, True)
        timings["z_release_s"] = self.reactor.monotonic() - stage_start
        self.z.do_set_position(2.5)

        # Optional D rail: same one-touch policy when enabled.
        if self.linear_enabled:
            self._home_slide_native(home_speed, accel)
        else:
            self.slide.do_set_position(0.)
            self.slide.do_enable(0)

        self._register_all()
        self.homed = True
        self.home_a = 3.5
        self.home_b = 3.5
        self.home_c = 2.5
        self.dance_active = False
        self.reactor.update_timer(self.dance_timer, self.reactor.NEVER)

        elapsed = self.reactor.monotonic() - start
        self.home_timing = {"stage": "complete", "total_s": elapsed, **timings}
        gcmd.respond_info(
            "ARM_HOME complete %.3fs @ %.1fdeg/s: XY seek %.3fs, XY release %.3fs, "
            "Z seek %.3fs, Z release %.3fs; linear D %s"
            % (elapsed, home_speed, timings["xy_seek_s"], timings["xy_release_s"],
               timings["z_seek_s"], timings["z_release_s"],
               "homed" if self.linear_enabled else "disabled/fixed at 0"))


def load_config(config):
    return ArmRobot(config)
