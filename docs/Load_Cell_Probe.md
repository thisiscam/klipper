# Load Cell Probes

## Related Documentation

* [load_cell_probe](Config_Reference.md#load_cell_probe) Config Reference
* [load_cell](Config_Reference.md#load_cell) Config Reference
* [load_cell](G-Codes.md#load_cell) G-Code Commands

## Verify the Load Cell First
A [load_cell_probe] is also a [load_cell] and G-code commands related to
[load_cell] work with [load_cell_probe]. Before attempting to use a load cell
probe, follow the directions for calibrating the load cell with
`CALIBRATE_LOAD_CELL` and checking its operation with `LOAD_CELL_DIAGNOSTIC`. 

## Verify Probe Operation Before Homing

Once you get the load cell part of [load_cell_probe] working you should verify
that the probe works before probing or homing the machine:

1. With nothing touching the load cell probe, run `TARE_LOAD_CELL`.
1. Run `QUERY_PROBE`, it should return:

   `// probe: open`
1. Apply a small force to the load cell and run `QUERY_PROBE`. It should report:

   `// probe: TRIGGERED`
   
If not you should not attempt to use the probe, it may crash your printer. Check
your configuration and `LOAD_CELL_DIAGNOSTIC` carefully to look for issues.

## Viewing Live Load Cell Graphs

It is strongly suggested that you set up the Klipper Load Cell Debugging Tool.
Follow the instruction in the
[Load Cell](Load_Cell.md#viewing-live-load-cell-graphs) documentation.

This tool plots extra graphs and details about probe taps which can be very
helpful to see. Hopefully in the future something similar is built into the
klipper front ends.

## Probing Temperature

Currently we suggest keeping the nozzle temperature below the level that causes
the filament to ooze while homing and probing. For most filaments this is a
limit of 150C but you may need t go lower for PLA.

Klipper does not yet have a generic way to detect "bad taps" due to filament
ooze. They may or may not be rejected by the existing code.

## Temperature Protection

The Voron project has a great macro for protecting your print surface from the
hot nozzle. See [Voron Tap's `activate_gcode`](https://github.com/VoronDesign/Voron-Tap/blob/main/config/tap_klipper_instructions.md)

It is highly suggested to add something like this to your config.

## Temperature Compensation

The nozzle will expand after heating to printing temperatures. This will cause
the nozzle to get closer to the print surface. 

If we calculate how much the nozzle expands its possible to compensate for this
with a macro:

```
[gcode_macro EXTRUDER_SET_THERMAL_COMP]
gcode:
    {% set expansion_coefficient = 0.00059 %} # measured empirically
    # pass in PRINT_TEMP= the final extruder temp before heating up
    {% set print_temp = params.PRINT_TEMP | float %}
    {% set temp_delta = print_temp - printer.extruder.target %}
    {% set thermal_comp_z = expansion_coefficient * temp_delta %}
    { action_respond_info('Extruder thermal compensation: %.5fmm for temperature
                           change %.1fC' % (thermal_comp_z, temp_delta)) }
    SET_GCODE_OFFSET Z_ADJUST={thermal_comp_z} MOVE=1
```

Call the macro while the nozzle is still set to the probing temperature. Pass in
the temperature that you will be printing at:

```
EXTRUDER_SET_THERMAL_COMP PRINT_TEMP={print_temp}
```

### Calculating the `expansion_coefficient`

1. Make sure the nozzle is clean and no filament is loaded. It must no ooze.
1. Run `PROBE_ACCURACY` with the nozzle at probing temperature (e.g. 140C)
2. Heat the nozzle up to the highest expected printing temp (1.g. 280C)
3. Run `PROBE_ACCURACY` again

Take the average value from each of the `PROBE_ACCURACY` runs and subtract them,
then divide by the temperature change in C.

```
expansion_coefficient = (cold_avg - hot_avg) / (285 - 140)
```

(we are looking to build this into the `[z_thermal_adjust]` module soon with
multiple named compensations)

## Continuous Tear Filters

Klipper implements a butterworth filter on the MCU to provide continuous tearing
of the load cell while probing. Continuous tearing means the 0 value moves with
drift caused by external factors like bowden tubes and thermal changes. This is
aimed at toolhead sensors that experience lots of external forces that change
while probing. 

The filter parameters should be selected based on drift seen on the printer
during normal operation. A Jupyter notebook is provided in scripts,
`filter_workbench.ipynb`, to perform a detailed investigation with real captured
data and FFTs.

For those just trying to get a filter working follow these suggestions:
* The only essential option is `continuous_tear_highpass`. A conservative
starting value is `0.5`. Prusa shipped the MK4 with a setting on `0.8` and the
XL with `11.2`. This is probably a safe range to experiment with. Setting this
value too high will result in excess force going through the tool head.
* Keep `continuous_tear_trigger_force_grams` low. The default is `40`g. The
filter keeps the internal grams value very close to 0 so a large trigger force
is not needed.
* Set `safety_limit_grams` to a conservative value. The default value is 1Kg
and this will keep your toolhead safe while experimenting. If you hit this limit
the `continuous_tear_highpass` value may be too high, or your
`reference_tare_counts` may need adjusting to be closet to the sensors 0 at
startup.  

## Suggestions for Load Cell Tool Boards

### Sensor Selection

Ideally a sensor would meet these criteria:
* Uses SPI communications
* Has a pin can be used to indicate sample ready without SPI communications.
This is often called the "data ready" or "DRDY" pin.
* Has a programmable gain amplifier level of 128
* Indicates via SPI if the sensor has been reset. Detecting resets avoids
timing errors in homing and using noisy data at startup. It can also help users
track down wiring and grounding issues.
* A selectable sample rate between 500Hz and 2Khz. Higher sample rates dont turn
out to be beneficial in our 3D printers because they produce so much noise when
moving fast. Sample rates below 250Hz will require slower probing speeds. They
also increase the force on the toolhead due to longer delays between
measurements. E.g. a 500Hz sensor moving at 5mm/s has the same safety factor as
a 100Hz sensor moving at only 1mm/s.
* If designing for under-bed applications and you want to sense multiple load
cells, use a chip that can sample all of its inputs simultaneously. Multiplex
ADCs that require switching channels have a settling of several samples. This
should be avoided for probing applications.

Implementing support for a new sensor is not particularly difficult with
Klipper's `bulk_sensor` infrastructure.

### 5V Power Filtering

We strongly suggest using larger capacitors than specified by the ADC chip
manufacturer. ADC chips are usually targeted at low noise environments, like
battery powered devices. Sensor manufacturers suggested application notes
generally assume a quiet power supply. Treat their suggested capacitor values as
minimums. 

3D printers put huge amounts of noise onto the 5V bus and this can ruin the
sensors accuracy. Test the sensor on the board with a typical 3D printer power
supply and active stepper drivers before deciding on smoothing capacitor sizes.  

### HX711 and HX717 Notes

We know this sensor is popular because of its low cost and availability in the
supply chain. However this is a sensor with several drawbacks. 

The HX71x sensors use bit bang communication which has a high overhead on the
MCU. Using a sensor that communicates via SPI would save resources on the tool
board's CPU.

The HX71x lack a way to communicate reset events to the MCU. Klipper detects
resets with heuristics but this is not ideal.

For probing applications we greatly prefer the HX717 version for probing because
of its higher sample rate (320 vs 80).

If designing a board for an under-bed sensor with multiple chips, the clock
lines should be tied to an external clock source. Klipper can compensate for a
single chip's clock drift. But for multiple chips with independent clock drift
the estimated measurement time will be less accurate. 