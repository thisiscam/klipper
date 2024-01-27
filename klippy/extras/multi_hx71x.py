# HX711/HX717 Support
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import multiplex_adc

class MultiHX71x(multiplex_adc.MultiplexAdcSensor):
    def __init__(self, config,
                 sample_rate_options, default_sample_rate,
                 gain_options, default_gain):
        self.printer = printer = config.get_printer()
        self.name = config.get_name()
        self.dout_pins = ['', '', '', '']
        self.sclk_pins = ['', '', '', '']
        self.chip_count = 0
        self.mcu = None
        self.oid = None
        ppins = printer.lookup_object('pins')
        for i in range(0, 4):
            dount_pin = config.get('dout%i_pin' % (i), default=None)
            sclk_pin = config.get('sclk%i_pin' % (i), default=None)
            if dount_pin is None or sclk_pin is None:
                break
            self.chip_count += 1
            dout_pin_params = ppins.lookup_pin(dount_pin)
            sclk_pin_params = ppins.lookup_pin(sclk_pin)
            mcu = dout_pin_params['chip']
            if self.oid is None:
                self.mcu = mcu
                self.oid = mcu.create_oid()
            if mcu is not self.mcu:
                raise self.printer.config_error("Multi-HX71x config error: \
                All HX71x chips must be connected to the same MCU")
            self.dout_pins[i] = dout_pin_params['pin']
            self.sclk_pins[i] = sclk_pin_params['pin']
        if self.chip_count < 1:
            raise self.printer.config_error("Multi-HX71x config error: \
                The minimum number of sensor chips is 1")
        if self.chip_count < 4:
            for i in range(self.chip_count, 4):
                # REVIEW: the pin names have to be valid
                # copying names feels hacky, is there a 'NONE' value?
                self.dout_pins[i] = self.dout_pins[self.chip_count - 1]
                self.sclk_pins[i] = self.sclk_pins[self.chip_count - 1]
        # Sampels per second choices
        self.sps = config.getchoice('sample_rate', sample_rate_options,
                                    default=default_sample_rate)
        # gain/channel choices
        self.gain_channel = int(config.getchoice('gain', gain_options,
                                                 default=default_gain))
        ppins.register_chip(self.name, self)
        mcu.register_config_callback(self._build_config)
    def _build_config(self):
        self.mcu.add_config_cmd("config_multi_hx71x oid=%d chip_count=%d" \
        " gain_channel=%d dout0_pin=%s sclk0_pin=%s dout1_pin=%s sclk1_pin=%s" \
        " dout2_pin=%s sclk2_pin=%s dout3_pin=%s sclk3_pin=%s"
            % (self.oid, self.chip_count, self.gain_channel,
                self.dout_pins[0], self.sclk_pins[0],
                self.dout_pins[1], self.sclk_pins[1],
                self.dout_pins[2], self.sclk_pins[2],
                self.dout_pins[3], self.sclk_pins[3]))
    def get_oid(self):
        return self.oid
    def get_mux_adc_sensor_type(self):
        return 'multi_hx71x'
    def get_mcu(self):
        return self.mcu
    def start_capture(self):
        pass # capture cant be stopped/started on these chips
    def stop_capture(self):
        pass # capture cant be stopped/started on these chips
    def get_samples_per_second(self):
        return self.sps
    def get_bits(self):
        return 24

class MultiHX711(MultiHX71x):
    def __init__(self, config):
        super(MultiHX711, self).__init__(config,
            # HX711 sps options
            {80: 80, 10: 10}, 10,
            # HX711 gain/channel options
            {'A-128': 1, 'B-32': 2, 'A-64': 3}, 'A-128')

class MultiHX717(MultiHX71x):
    def __init__(self, config):
        super(MultiHX717, self).__init__(config,
            # HX717 sps options
            {320: 320, 80: 80, 20: 20, 10: 10}, 320,
            # HX717 gain/channel options
            {'A-128':1, 'B-64': 2, 'A-64': 3, 'B-8': 4}, 'A-128')
