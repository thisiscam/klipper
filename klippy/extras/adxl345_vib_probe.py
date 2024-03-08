"""An accelerometer-based vibration based probe."""

from . import probe, adxl345, force_move, resonance_tester

REG_THRESH_TAP = 0x1D
REG_DUR = 0x21
REG_INT_MAP = 0x2F
REG_TAP_AXES = 0x2A
REG_INT_ENABLE = 0x2E
REG_INT_SOURCE = 0x30

DUR_SCALE = 0.000625  # 0.625 msec / LSB
TAP_SCALE = 0.0625 * adxl345.FREEFALL_ACCEL  # 62.5mg/LSB * Earth gravity in mm/s**2

ADXL345_REST_TIME = .1


class ADXL345VibProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(config, 'deactivate_gcode', '')
        probe_pin = config.get('probe_pin')
        self.freq = config.getint('freq', 50, minval=10, maxval=1000.)
        self.accel = config.getfloat('accel', 500, above=10, maxval=10000.)
        self.position_endstop = config.getfloat('z_offset')
        try:
          self.axis = resonance_tester.parse_axis(config.get("axis", "X").lower())
        except resonance_tester.AxisParseError as e:
          raise config.error(str(e))

        self._old_max_accel = self._old_max_accel_to_decel = None

        self.adxl345 = self.printer.lookup_object('adxl345')
        self.next_cmd_time = self.action_end_time = 0.
        # # Create an "endstop" object to handle the sensor pin
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(probe_pin, can_invert=True,
                                      can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        # Add wrapper methods for endstops
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # Register commands and callbacks
        self.gcode = self.printer.lookup_object('gcode')
        # self.gcode.register_mux_command("SET_ACCEL_PROBE", "CHIP", None, self.cmd_SET_ACCEL_PROBE, desc=self.cmd_SET_ACCEL_PROBE_help)
        self.printer.register_event_handler('klippy:connect', self.init_adxl)
        self.printer.register_event_handler('klippy:mcu_identify', self.handle_mcu_identify)
        self.printer.add_object('probe', probe.PrinterProbe(config, self))

    def init_adxl(self):
        chip = self.adxl345
        # chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        # chip.set_reg(adxl345.REG_DATA_FORMAT, 0x0B)

    def handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def get_position_endstop(self):
        return self.position_endstop

    def probe_prepare(self, hmove, movepos, speed):
        self.activate_gcode.run_gcode_from_command()
        chip = self.adxl345
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        toolhead.dwell(ADXL345_REST_TIME)
        print_time = toolhead.get_last_move_time()

        X, Y, Z, E = toolhead.get_position()
        sign = 1.
        freq = self.freq
        # Override maximum acceleration and acceleration to
        # deceleration based on the maximum test frequency
        systime = self.printer.get_reactor().monotonic()
        toolhead_info = toolhead.get_status(systime)
        self._old_max_accel = toolhead_info['max_accel']
        self._old_max_accel_to_decel = toolhead_info['max_accel_to_decel']
        max_accel = self.accel
        self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    max_accel, max_accel))
        input_shaper = self.printer.lookup_object('input_shaper', None)
        if input_shaper is not None:
            input_shaper.disable_shaping()

        toolhead.cmd_M204(self.gcode.create_gcode_command("M204", "M204", {"S": self.accel}))

        axis_r, accel_t, cruise_t, speed = force_move.calc_move_time(movepos[2] - Z, speed, self.accel)
        move_t = accel_t * 2 + cruise_t
        moves = []

        t = 0
        t_seg = .25 / freq
        max_v = self.accel * t_seg
        L = .5 * self.accel * t_seg**2
        dX, dY, dZ = self.axis.get_point(L)
        vx, vy, vz = self.axis.get_point(max_v)

        def z_speed_at_time(t):
            return axis_r * (self.accel * min(t, accel_t) + speed * max(0, t - accel_t) - self.accel * max(0, t - accel_t - cruise_t))

        while t < move_t:
            nX = X + sign * dX
            nY = Y + sign * dY
            z_move_velocity = z_speed_at_time(t)
            max_v = vx **2 + vy **2 + (vz + z_move_velocity)**2
            Z += z_move_velocity * t_seg
            nZ = Z + sign * dZ
            moves.append(([nX, nY, nZ, E], max_v))
            t += t_seg / 2
            z_move_velocity = z_speed_at_time(t)
            max_v = vx **2 + vy **2 + (vz + z_move_velocity)**2
            Z += z_move_velocity * t_seg
            moves.append(([X, Y, Z, E], max_v))
            t += t_seg
            sign = -sign

        hmove.set_homing_moves(moves)

    def probe_finish(self, hmove):
        chip = self.adxl345
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(ADXL345_REST_TIME)
        print_time = toolhead.get_last_move_time()
        clock = chip.mcu.print_time_to_clock(print_time)

        # Restore the original acceleration values
        self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    self._old_max_accel, self._old_max_accel_to_decel))
        # Restore input shaper if it was disabled for resonance testing
        input_shaper = self.printer.lookup_object('input_shaper', None)
        if input_shaper is not None:
            input_shaper.enable_shaping()
        self.deactivate_gcode.run_gcode_from_command()

    cmd_SET_ACCEL_PROBE_help = "Configure ADXL345 parameters related to probing"

    def cmd_SET_ACCEL_PROBE(self, gcmd):
        chip = self.adxl345
        self.tap_thresh = gcmd.get_float('TAP_THRESH', self.tap_thresh,
                                         minval=TAP_SCALE, maxval=100000.)
        self.tap_dur = gcmd.get_float('TAP_DUR', self.tap_dur,
                                      above=DUR_SCALE, maxval=0.1)
        chip.set_reg(REG_THRESH_TAP, int(self.tap_thresh / TAP_SCALE))
        chip.set_reg(REG_DUR, int(self.tap_dur / DUR_SCALE))


def load_config(config):
    return ADXL345VibProbe(config)