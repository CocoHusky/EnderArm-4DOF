# Native robot-arm motion commands for Klipper.
# Fast multi-endstop X/Y homing and local robot motion.

class _XYHomingToolhead:
    def __init__(self, printer, xjoint, yjoint):
        self.printer = printer
        self.x = xjoint
        self.y = yjoint
        self.homing_accel = 1000.0

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
        self.x.next_cmd_time = end
        self.y.next_cmd_time = end

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


class ArmRobot:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command("ARM_HOME", self.cmd_ARM_HOME,
                                    desc="Fast native robot-arm homing")
        self.gcode.register_command("ARM_HOME_TEST", self.cmd_ARM_HOME_TEST,
                                    desc="Test native concurrent X/Y homing")
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.x = self.printer.lookup_object("manual_stepper joint_x")
        self.y = self.printer.lookup_object("manual_stepper joint_y")
        self.z = self.printer.lookup_object("manual_stepper joint_z")
        self.xytool = _XYHomingToolhead(self.printer, self.x, self.y)

    def _unregister_axis(self, joint):
        if joint.axis_gcode_id is None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.remove_extra_axis(joint)
        joint.axis_gcode_id = None

    def _unregister_all(self):
        for joint in (self.x, self.y, self.z):
            self._unregister_axis(joint)

    def _register_axis(self, joint, axis, velocity=220., accel=1800.,
                       corner=45.):
        self._unregister_axis(joint)
        toolhead = self.printer.lookup_object("toolhead")
        joint.axis_gcode_id = axis
        joint.instant_corner_v = corner
        joint.gaxis_limit_velocity = velocity
        joint.gaxis_limit_accel = accel
        toolhead.add_extra_axis(joint, joint.commanded_pos)

    def _register_all(self):
        self._register_axis(self.x, "A", 220., 1800., 45.)
        self._register_axis(self.y, "B", 220., 1800., 45.)
        self._register_axis(self.z, "C", 260., 2200., 55.)

    def _prepare_manual(self):
        self._unregister_all()
        for joint in (self.x, self.y, self.z):
            joint.do_enable(1)

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

    def _home_z(self, target, speed, accel):
        self.z.do_homing_move(target, speed, accel, True, True, True)

    def _release_xy(self, amount=4.0):
        self.x.do_set_position(0.)
        self.y.do_set_position(0.)
        self._move_xy(amount, amount, 120.0, 1600.0)

    def _release_z(self, amount=4.0):
        self.z.do_set_position(0.)
        self.z.do_move(amount, 140.0, 1800.0, True)

    def cmd_ARM_HOME_TEST(self, gcmd):
        dist = gcmd.get_float("DIST", 15.0, above=0.)
        speed = gcmd.get_float("SPEED", 70.0, above=0.)
        accel = gcmd.get_float("ACCEL", 1000.0, above=0.)
        self._prepare_manual()
        self.x.do_set_position(0.)
        self.y.do_set_position(0.)
        start = self.reactor.monotonic()
        self._home_xy(-dist, speed, accel)
        elapsed = self.reactor.monotonic() - start
        self._release_xy(4.0)
        gcmd.respond_info("ARM_HOME_TEST X/Y complete in %.3fs" % elapsed)

    def cmd_ARM_HOME(self, gcmd):
        start = self.reactor.monotonic()
        coarse_speed = gcmd.get_float("SPEED", 110.0, above=0.)
        fine_speed = gcmd.get_float("FINE_SPEED", 28.0, above=0.)
        accel = gcmd.get_float("ACCEL", 1400.0, above=0.)

        self._prepare_manual()

        # X/Y coarse: one multi-endstop HomingMove.
        # Each MCU endstop halts only its associated motor; the other
        # joint continues until its own endstop fires.
        self.x.do_set_position(0.)
        self.y.do_set_position(0.)
        self._home_xy(-180.0, coarse_speed, accel)

        # Short release and precision re-home.
        self._release_xy(4.0)
        self.x.do_set_position(0.)
        self.y.do_set_position(0.)
        self._home_xy(-10.0, fine_speed, 450.0)
        self._release_xy(4.0)

        # Z coarse + precision.
        self.z.do_set_position(0.)
        self._home_z(-180.0, 120.0, 1600.0)
        self._release_z(4.0)
        self.z.do_set_position(0.)
        self._home_z(-10.0, 30.0, 500.0)
        self._release_z(4.0)

        # Normal robot control uses planner-coordinated A/B/C axes.
        self._register_all()

        elapsed = self.reactor.monotonic() - start
        gcmd.respond_info(
            "ARM_HOME complete in %.3fs; release A/B/C=4.0deg" % elapsed)


def load_config(config):
    return ArmRobot(config)