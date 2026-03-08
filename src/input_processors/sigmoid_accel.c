/*
 * Copyright (c) 2024 Alice Zhang
 * SPDX-License-Identifier: MIT
 *
 * Apple Magic Trackpad-style acceleration for ZMK
 *
 * Based on:
 * - 1 Euro Filter (Casiez et al., CHI 2012)
 * - libinput pointer acceleration
 */

#define DT_DRV_COMPAT zmk_input_processor_sigmoid_accel

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define ONE_EURO_MINCUTOFF 1000  // 1.0 Hz
#define ONE_EURO_BETA 100       // 0.1
#define ONE_EURO_DCUTOFF 1000   // 1.0 Hz
#define TAU_FACTOR 159155       // 1e6 / (2 * PI), for microsecond calculations

struct sigmoid_accel_config {
    int32_t min_scale;
    int32_t max_scale;
    int32_t threshold;
    int32_t steepness;
};

struct sigmoid_accel_state {
    int32_t pending_x;
    int32_t pending_y;
    bool has_x;
    bool has_y;
    int64_t last_time_us;
    int32_t avg_velocity;
    int32_t filtered_x;
    int32_t filtered_y;
    int32_t dx_filtered;
    int32_t dy_filtered;
    bool initialized;
    int32_t remainder_x;
    int32_t remainder_y;
};

// Pre-computed sigmoid(x) * 100 for x = -3.0 to 3.0, step 0.5
static const int8_t SIGMOID_LUT[13] = {
    5, 7, 12, 18, 27, 38, 50, 62, 73, 82, 88, 93, 95
};

static int32_t sigmoid_lookup(int32_t x_scaled) {
    if (x_scaled <= -300) return 5;
    if (x_scaled >= 300) return 95;

    int32_t shifted = x_scaled + 300;
    int32_t index = shifted / 50;
    int32_t frac = shifted % 50;

    if (index >= 12) return SIGMOID_LUT[12];

    int32_t v0 = SIGMOID_LUT[index];
    int32_t v1 = SIGMOID_LUT[index + 1];
    return v0 + (v1 - v0) * frac / 50;
}

static int32_t isqrt(int32_t n) {
    if (n <= 0) return 0;
    if (n < 2) return n;

    int32_t x = n;
    int32_t y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

static int32_t calc_alpha(int32_t cutoff_hz_x1000, int32_t dt_us) {
    if (dt_us <= 0) dt_us = 1000;
    if (cutoff_hz_x1000 <= 0) cutoff_hz_x1000 = 1000;

    int32_t tau_us = TAU_FACTOR * 1000 / cutoff_hz_x1000;
    int32_t alpha = dt_us * 1000 / (dt_us + tau_us);

    if (alpha < 10) return 10;
    if (alpha > 990) return 990;
    return alpha;
}

static int32_t one_euro_filter(int32_t x, int32_t *x_filt, int32_t *dx_filt,
                               int32_t dt_us) {
    int32_t dx = (x * 1000 - *x_filt);
    if (dt_us > 0) {
        dx = dx * 1000 / dt_us;
    }

    int32_t alpha_d = calc_alpha(ONE_EURO_DCUTOFF, dt_us);
    *dx_filt = (*dx_filt * (1000 - alpha_d) + dx * alpha_d) / 1000;

    int32_t cutoff = ONE_EURO_MINCUTOFF + ONE_EURO_BETA * abs(*dx_filt) / 1000;
    if (cutoff > 50000) cutoff = 50000;

    int32_t alpha = calc_alpha(cutoff, dt_us);
    *x_filt = (*x_filt * (1000 - alpha) + x * 1000 * alpha) / 1000;
    return *x_filt;
}

// Exponential moving average for velocity smoothing
static int32_t smooth_velocity(int32_t current, int32_t new_val) {
    // alpha = 0.4 (400/1000) - responsive but smooth
    return (current * 600 + new_val * 400) / 1000;
}

static int32_t calculate_scale(const struct sigmoid_accel_config *config,
                               int32_t velocity) {
    int32_t delta = velocity - config->threshold;
    int32_t sigmoid_input = delta * config->steepness / 100;
    int32_t sigmoid_val = sigmoid_lookup(sigmoid_input);
    return config->min_scale +
           (config->max_scale - config->min_scale) * sigmoid_val / 100;
}

static void reset_state(struct sigmoid_accel_state *s) {
    s->avg_velocity = 0;
    s->filtered_x = 0;
    s->filtered_y = 0;
    s->dx_filtered = 0;
    s->dy_filtered = 0;
    s->remainder_x = 0;
    s->remainder_y = 0;
    s->has_x = false;
    s->has_y = false;
    s->initialized = false;
}

static int sigmoid_accel_handle_event(const struct device *dev,
                                      struct input_event *event,
                                      uint32_t param1,
                                      uint32_t param2,
                                      struct zmk_input_processor_state *state) {
    const struct sigmoid_accel_config *config = dev->config;
    struct sigmoid_accel_state *s = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t dt_us = now_us - s->last_time_us;

    // Reset state after long gap (finger lifted)
    if (dt_us <= 0 || dt_us > 100000) {
        reset_state(s);
        dt_us = 8000;
    }

    if (event->code == INPUT_REL_X) {
        s->pending_x = event->value;
        s->has_x = true;
    } else {
        s->pending_y = event->value;
        s->has_y = true;
    }

    int32_t dx = s->has_x ? s->pending_x : 0;
    int32_t dy = s->has_y ? s->pending_y : 0;
    int32_t magnitude = isqrt(dx * dx + dy * dy);
    int32_t instant_velocity = magnitude * 10000 / (int32_t)dt_us;

    if (!s->initialized) {
        s->avg_velocity = instant_velocity;
        s->filtered_x = event->value * 1000;
        s->filtered_y = event->value * 1000;
        s->initialized = true;
    }

    s->avg_velocity = smooth_velocity(s->avg_velocity, instant_velocity);
    int32_t scale = calculate_scale(config, s->avg_velocity);

    int32_t filtered_value;
    int32_t *remainder;
    bool is_x = (event->code == INPUT_REL_X);

    // Apply jitter filter only at slow speeds
    if (s->avg_velocity < config->threshold / 2) {
        if (is_x) {
            filtered_value = one_euro_filter(event->value, &s->filtered_x,
                                             &s->dx_filtered, dt_us) / 1000;
        } else {
            filtered_value = one_euro_filter(event->value, &s->filtered_y,
                                             &s->dy_filtered, dt_us) / 1000;
        }
    } else {
        filtered_value = event->value;
        if (is_x) {
            s->filtered_x = event->value * 1000;
        } else {
            s->filtered_y = event->value * 1000;
        }
    }

    remainder = is_x ? &s->remainder_x : &s->remainder_y;
    int32_t scaled = filtered_value * scale + *remainder;
    event->value = scaled / 100;
    *remainder = scaled % 100;

    s->last_time_us = now_us;

    if (is_x) {
        s->has_x = false;
    } else {
        s->has_y = false;
    }

    LOG_DBG("v=%d scale=%d.%02d in=%d out=%d",
            s->avg_velocity, scale / 100, scale % 100,
            filtered_value, event->value);

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api sigmoid_accel_driver_api = {
    .handle_event = sigmoid_accel_handle_event,
};

#define SIGMOID_ACCEL_INST(n)                                              \
    static struct sigmoid_accel_state sigmoid_accel_state_##n = {};        \
    static const struct sigmoid_accel_config sigmoid_accel_config_##n = {  \
        .min_scale = DT_INST_PROP(n, min_scale),                           \
        .max_scale = DT_INST_PROP(n, max_scale),                           \
        .threshold = DT_INST_PROP(n, threshold),                           \
        .steepness = DT_INST_PROP(n, steepness),                           \
    };                                                                     \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &sigmoid_accel_state_##n,         \
                          &sigmoid_accel_config_##n, POST_KERNEL,          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,             \
                          &sigmoid_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SIGMOID_ACCEL_INST)
