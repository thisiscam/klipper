// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "load_cell_endstop.h" // load_cell_endstop_report_sample
#include <stdint.h>

struct hx71x_adc {
    struct timer timer;
    uint32_t rest_ticks;
    struct gpio_in dout[4]; // pins used to receive data from the hx71x 0
    struct gpio_out sclk[4]; // pins used to generate clock for the hx71x 0
    uint8_t chip_count;      // the numbers of sensor chips, 3 or 4
    uint8_t gain_channel;   // the gain+channel selection (1-4)
    uint8_t flags, data_count, overflow;
    struct sensor_bulk sb;
    struct load_cell_endstop *lce;
};

// Flag types
enum {FLAG_PENDING = 1 << 0
    , FLAG_RESET_REQUIRED = 1 << 1  // set by chip error event
};

#define BYTES_PER_SAMPLE 4

static struct task_wake wake_hx71x;

/****************************************************************
 * Timing
 ****************************************************************/

typedef unsigned int hx71x_time_t;

static hx71x_time_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline int
hx71x_check_elapsed(hx71x_time_t t1, hx71x_time_t t2
                       , hx71x_time_t ticks)
{
    return t2 - t1 >= ticks;
}

// The AVR micro-controllers require specialized timing
#if CONFIG_MACH_AVR

#include <avr/interrupt.h> // TCNT1

static hx71x_time_t
hx71x_get_time(void)
{
    return TCNT1;
}

#define hx71x_delay_no_irq(start, ticks) (void)(ticks)
#define hx71x_delay(start, ticks) (void)(ticks)

#else

static hx71x_time_t
hx71x_get_time(void)
{
    return timer_read_time();
}

static inline void
hx71x_delay_no_irq(hx71x_time_t start, hx71x_time_t ticks)
{
    while (!hx71x_check_elapsed(start, hx71x_get_time(), ticks))
        ;
}

static inline void
hx71x_delay(hx71x_time_t start, hx71x_time_t ticks)
{
    while (!hx71x_check_elapsed(start, hx71x_get_time(), ticks))
        irq_poll();
}

#endif

/****************************************************************
 * HX711 and HX717 Sensor Support
 ****************************************************************/
// both HX717 and HX711 have 200ns min pulse time for clock pin on/off
#define MIN_PULSE_TIME nsecs_to_ticks(200)

static inline uint8_t
is_flag_set(const uint8_t mask, struct hx71x_adc *hx71x)
{
    return !!(mask & hx71x->flags);
}

static inline void
set_flag(uint8_t mask, struct hx71x_adc *hx71x)
{
    hx71x->flags |= mask;
}

static inline void
clear_flag(uint8_t mask, struct hx71x_adc *hx71x)
{
    hx71x->flags &= ~mask;
}

// Event handler that wakes wake_hx71x() periodically
static uint_fast8_t
hx71x_event(struct timer *timer)
{
    struct hx71x_adc *hx71x = container_of(timer, struct hx71x_adc, timer);
    set_flag(FLAG_PENDING, hx71x);
    sched_wake_task(&wake_hx71x);
    return SF_DONE;
}

// Helper code to reschedule the hx71x_event() timer
static void
hx71x_reschedule_timer(struct hx71x_adc *hx71x)
{
    irq_disable();
    set_flag(FLAG_PENDING, hx71x);
    hx71x->timer.waketime = timer_read_time() + hx71x->rest_ticks;
    sched_add_timer(&hx71x->timer);
    irq_enable();
}

void
hx71x_reset(struct hx71x_adc *hx71x, uint8_t oid) {
    // stop running the read task if one exists:
    sched_del_timer(&hx71x->timer);
    hx71x->flags = 0;
    set_flag(FLAG_RESET_REQUIRED, hx71x);
    // The chips are reset by setting PD_SCK pin high and waiting
    // 60us for HX711 or 100us for HX717. Host/mcu delay is larger than this.
    for (uint_fast8_t i = 0; i < hx71x->chip_count; i++) {
        gpio_out_write(hx71x->sclk[i], 1);
    }
    // Notify host of the reset
    sendf("reset_hx71x oid=%c", oid);
}

int8_t
hx71x_is_data_ready(struct hx71x_adc *hx71x) {
    // if any pin is high the samples are not all ready
    for (uint_fast8_t i = 0; i < hx71x->chip_count; i++) {
        if (gpio_in_read(hx71x->dout[i])) {
            return 0;
        }
    }
    return 1;
}

// Add a measurement to the buffer
static void
add_sample(struct hx71x_adc *hx71x, uint_fast32_t counts)
{
    hx71x->sb.data[hx71x->sb.data_count] = counts;
    hx71x->sb.data[hx71x->sb.data_count + 1] = counts >> 8;
    hx71x->sb.data[hx71x->sb.data_count + 2] = counts >> 16;
    hx71x->sb.data[hx71x->sb.data_count + 3] = counts >> 24;
    hx71x->sb.data_count += BYTES_PER_SAMPLE;
}

static void
flush_samples(struct hx71x_adc *hx71x, uint8_t oid)
{
    const uint8_t block_size = BYTES_PER_SAMPLE * hx71x->chip_count;
    if (hx71x->sb.data_count + block_size > ARRAY_SIZE(hx71x->sb.data))
        sensor_bulk_report(&hx71x->sb, oid);
}

// Pulse all clock pins to move to the next bit
inline static void
hx71x_pulse_clocks(struct hx71x_adc *hx71x) {
    irq_disable();
    uint_fast8_t i;
    hx71x_time_t start_time = hx71x_get_time();
    for (i = 0; i < hx71x->chip_count; i++) {
        gpio_out_write(hx71x->sclk[i], 1);
    }
    hx71x_delay_no_irq(start_time, MIN_PULSE_TIME);
    for (i = 0; i < hx71x->chip_count; i++) {
        gpio_out_write(hx71x->sclk[i], 0);
    }
    irq_enable();
}

// hx71x ADC query
void
hx71x_read_adc(struct hx71x_adc *hx71x, uint8_t oid)
{
    if (!hx71x_is_data_ready(hx71x)) {
        hx71x_reschedule_timer(hx71x);
        return;
    }

    // data is ready
    int32_t counts[4] = {0, 0, 0, 0};
    hx71x_time_t start_time = timer_read_time();
    uint_fast8_t i;
    for (uint_fast8_t sample_idx = 0; sample_idx < 24; sample_idx++) {
        hx71x_pulse_clocks(hx71x);
        hx71x_delay(hx71x_get_time(), MIN_PULSE_TIME);
        // read 2's compliment int bits
        for (i = 0; i < hx71x->chip_count; i++) {
            counts[i] = (counts[i] << 1) | gpio_in_read(hx71x->dout[i]);
        }
    }

    // bit bang 1 to 4 more bits to configure gain & channel for the next sample
    for (uint8_t gain_idx = 0; gain_idx < hx71x->gain_channel; gain_idx++) {
        hx71x_pulse_clocks(hx71x);
        // test if this delay is causing bad reads?
        //if (gain_idx < hx71x->gain_channel - 1) {
        hx71x_delay(hx71x_get_time(), MIN_PULSE_TIME);
        //}
    }

    hx71x_time_t time_diff = timer_read_time() - start_time;
    if (time_diff >= hx71x->rest_ticks) {
        // some IRQ delayed this read so much that the chips must be reset
        hx71x_reset(hx71x, oid);
        return;
    }

    int32_t total_counts = 0;
    for (i = 0; i < hx71x->chip_count; i++) {
        // extend 2's complement 24 bits to 32bits
        if (counts[i] >= 0x800000) {
            counts[i] |= 0xFF000000;
        }
        if (!gpio_in_read(hx71x->dout[i]) || counts[i] < -0x7FFFFF
                || counts[i] > 0x7FFFFF) {
            // something went wrong with the read, reset chips
            hx71x_reset(hx71x, oid);
            return;
        }
        total_counts += counts[i];
        add_sample(hx71x, counts[i]);
    }

    // endstop is optional, report if enabled
    if (hx71x->lce) {
        load_cell_endstop_report_sample(hx71x->lce, total_counts, start_time);
    }

    flush_samples(hx71x, oid);
    hx71x_reschedule_timer(hx71x);
}

// Create a hx71x sensor
void
command_config_hx71x(uint32_t *args)
{
    struct hx71x_adc *hx71x = oid_alloc(args[0]
                , command_config_hx71x, sizeof(*hx71x));
    hx71x->timer.func = hx71x_event;
    hx71x->flags = 0;
    uint8_t chip_count = args[1];
    if (chip_count < 1 || chip_count > 4) {
        shutdown("HX71x only supports 1 to 4 sensors");
    }
    hx71x->chip_count = chip_count;
    uint8_t gain_channel = args[2];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("HX71x gain/channel out of range 1-4");
    }
    hx71x->gain_channel = gain_channel;
    // optional endstop
    if (args[3] != 0) {
        hx71x->lce = load_cell_endstop_oid_lookup(args[3]);
    }
    // Setup pins for 1-4 sensor chips
    uint8_t arg_idx = 4;
    for (uint8_t chip_idx = 0; chip_idx < chip_count; chip_idx++) {
        hx71x->dout[chip_idx] = gpio_in_setup(args[arg_idx], -1);
        hx71x->sclk[chip_idx] = gpio_out_setup(args[arg_idx + 1], 0);
        arg_idx += 2;
    }
}
DECL_COMMAND(command_config_hx71x, "config_hx71x oid=%c"
    " chip_count=%c gain_channel=%c load_cell_endstop_oid=%c"
    " dout1_pin=%u sclk1_pin=%u dout2_pin=%u sclk2_pin=%u"
    " dout3_pin=%u sclk3_pin=%u dout4_pin=%u sclk4_pin=%u");

// start/stop capturing ADC data
void
command_query_hx71x(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    sched_del_timer(&hx71x->timer);
    hx71x->flags = 0;
    hx71x->rest_ticks = args[1];
    if (!hx71x->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    sensor_bulk_reset(&hx71x->sb);
    // Put all chips in run mode, in case they were reset
    for (uint_fast8_t i = 0; i < hx71x->chip_count; i++) {
        gpio_out_write(hx71x->sclk[i], 0);
    }
    hx71x_reschedule_timer(hx71x);
}
DECL_COMMAND(command_query_hx71x,
             "query_hx71x oid=%c rest_ticks=%u");

void
command_query_hx71x_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    const uint32_t start_t = timer_read_time();
    const uint8_t reset_required = is_flag_set(FLAG_RESET_REQUIRED, hx71x);
    uint8_t pending_bytes = 0;
    if (!reset_required) {
        pending_bytes = hx71x_is_data_ready(hx71x);
        pending_bytes *= (BYTES_PER_SAMPLE * hx71x->chip_count);
    }
    const uint32_t end_t = timer_read_time();
    sensor_bulk_status(&hx71x->sb, oid, start_t, (end_t - start_t)
                      , pending_bytes);
}
DECL_COMMAND(command_query_hx71x_status, "query_hx71x_status oid=%c");

// Background task that performs measurements
void
hx71x_capture_task(void)
{
    if (!sched_check_wake(&wake_hx71x))
        return;
    uint8_t oid;
    struct hx71x_adc *hx71x;
    foreach_oid(oid, hx71x, command_config_hx71x) {
        uint_fast8_t flags = hx71x->flags;
        if (flags & FLAG_PENDING) {
            hx71x_read_adc(hx71x, oid);
        }
    }
}
DECL_TASK(hx71x_capture_task);
