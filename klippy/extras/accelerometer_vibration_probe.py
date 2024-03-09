"""An accelerometer-vibration based probe."""

from . import probe, adxl345, force_move, resonance_tester, load_cell, load_cell_probe

REST_TIME = .1

class AccelerometerVibrationProbe(load_cell_probe.LoadCellEndstop):
    def __init__(self, config, load_cell_inst):
        super().__init__(config, load_cell_inst)
        self.freq = config.getint('freq', 50, minval=10, maxval=1000.)
        self.accel = config.getfloat('accel', 500, above=10, maxval=10000.)
        self.position_endstop = config.getfloat('z_offset')
        try:
          self.axis = resonance_tester.parse_axis(config.get("axis", "X").lower())
        except resonance_tester.AxisParseError as e:
          raise config.error(str(e))

        self._old_max_accel = self._old_max_accel_to_decel = None

    def handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def deactivate_probe(self):
        toolhead = self._printer.lookup_object('toolhead')
        self.deactivate_gcode.run_gcode_from_command()
    def activate_probe(self):
        toolhead = self._printer.lookup_object('toolhead')
        self.activate_gcode.run_gcode_from_command()
    def multi_probe_begin(self):
        if self.deactivate_on_each_sample:
            return
        self.multi = 'FIRST'
    def multi_probe_end(self):
        if self.deactivate_on_each_sample:
            return
        self.deactivate_probe()
        self.multi = 'OFF'
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.activate_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self.deactivate_probe()

    def probe_prepare(self, hmove, movepos, speed):
        self.activate_gcode.run_gcode_from_command()
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        toolhead.dwell(REST_TIME)
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

        t_seg = .25 / freq
        max_v = self.accel * t_seg
        L = .5 * self.accel * t_seg**2
        dX, dY, dZ = self.axis.get_point(L)
        vx, vy, vz = self.axis.get_point(max_v)

        def z_speed_at_time(t):
          if t < accel_t:
            return axis_r * self.accel * t
          elif t < accel_t + cruise_t:
            return axis_r * speed
          else:
            return axis_r * (speed - self.accel * (t - accel_t - cruise_t))

        t = 0
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
        toolhead.dwell(REST_TIME)
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
    # Sensor types supported by load_cell_probe
    sensors = {}
    sensors['adx345_vibration'] = adxl345.ADXL345
    sensor_class = config.getchoice('sensor_type', sensors)
    sensor = sensor_class(config, allocate_endstop_oid=True)
    lc = load_cell.LoadCell(config, sensor)
    printer = config.get_printer()
    name = config.get_name().split()[-1]
    lc_name = 'accelerometer_vibration' if name == "accelerometer_vibration_probe" else 'accelerometer_vibration ' + name
    printer.add_object(lc_name, lc)
    lce = AccelerometerVibrationProbe(config, lc)
    lc_probe = load_cell.LoadCellPrinterProbe(config, lc, lce)
    #TODO: for multiple probes this cant be static value 'probe'
    printer.add_object('probe', lc_probe)
    return lc_probe
