/*
 * Copyright (c) 2024 Alice Zhang
 * SPDX-License-Identifier: MIT
 *
 * Apple Magic Trackpad-style acceleration for ZMK
 * Handles both pointer (X/Y) and scroll (WHEEL/HWHEEL) events
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

#define ONE_EURO_MINCUTOFF 1000
#define ONE_EURO_BETA 100
#define ONE_EURO_DCUTOFF 1000
#define TAU_FACTOR 159155

struct sigmoid_accel_config {
    int32_t min_scale;
    int32_t max_scale;
    int32_t threshold;
    int32_t steepness;
};

struct axis_state {
    int32_t pending;
    bool has_pending;
    int32_t filtered;
    int32_t dx_filtered;
    int32_t remainder;
};

struct sigmoid_accel_state {
    struct axis_state x;
    struct axis_state y;
    struct axis_state wheel;
    struct axis_state hwheel;
    int64_t last_time_us;
    int32_t avg_velocity;
    int32_t scroll_velocity;
    bool initialized;
    bool scroll_initialized;
};

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

static int32_t smooth_velocity(int32_t current, int32_t new_val) {
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

static void reset_axis(struct axis_state *a) {
    a->pending = 0;
    a->has_pending = false;
    a->filtered = 0;
    a->dx_filtered = 0;
    a->remainder = 0;
}

static void reset_pointer_state(struct sigmoid_accel_state *s) {
    reset_axis(&s->x);
    reset_axis(&s->y);
    s->avg_velocity = 0;
    s->initialized = false;
}

static void reset_scroll_state(struct sigmoid_accel_state *s) {
    reset_axis(&s->wheel);
    reset_axis(&s->hwheel);
    s->scroll_velocity = 0;
    s->scroll_initialized = false;
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

    bool is_pointer = (event->code == INPUT_REL_X || event->code == INPUT_REL_Y);
    bool is_scroll = (event->code == INPUT_REL_WHEEL || event->code == INPUT_REL_HWHEEL);

    if (!is_pointer && !is_scroll) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t dt_us = now_us - s->last_time_us;

    if (dt_us <= 0 || dt_us > 100000) {
        reset_pointer_state(s);
        reset_scroll_state(s);
        dt_us = 8000;
    }

    struct axis_state *axis;
    struct axis_state *axis2;
    int32_t *velocity;
    bool *is_init;

    if (is_pointer) {
        if (event->code == INPUT_REL_X) {
            axis = &s->x;
            axis2 = &s->y;
        } else {
            axis = &s->y;
            axis2 = &s->x;
        }
        velocity = &s->avg_velocity;
        is_init = &s->initialized;
    } else {
        if (event->code == INPUT_REL_WHEEL) {
            axis = &s->wheel;
            axis2 = &s->hwheel;
        } else {
            axis = &s->hwheel;
            axis2 = &s->wheel;
        }
        velocity = &s->scroll_velocity;
        is_init = &s->scroll_initialized;
    }

    axis->pending = event->value;
    axis->has_pending = true;

    int32_t d1 = axis->has_pending ? axis->pending : 0;
    int32_t d2 = axis2->has_pending ? axis2->pending : 0;
    int32_t magnitude = isqrt(d1 * d1 + d2 * d2);
    int32_t instant_velocity = magnitude * 10000 / (int32_t)dt_us;

    if (!*is_init) {
        *velocity = instant_velocity;
        axis->filtered = event->value * 1000;
        *is_init = true;
    }

    *velocity = smooth_velocity(*velocity, instant_velocity);

    // Scroll uses adjusted parameters for smoother feel
    int32_t scale;
    if (is_scroll) {
        // Lower threshold, higher max for scroll (MX Master-like)
        int32_t scroll_threshold = config->threshold / 2;
        int32_t delta = *velocity - scroll_threshold;
        int32_t sigmoid_input = delta * config->steepness / 100;
        int32_t sigmoid_val = sigmoid_lookup(sigmoid_input);
        // Scroll range: 0.5x to 6x for fast flicks
        scale = 50 + (600 - 50) * sigmoid_val / 100;
    } else {
        scale = calculate_scale(config, *velocity);
    }

    int32_t filtered_value;
    int32_t filter_threshold = is_scroll ? config->threshold / 4 : config->threshold / 2;

    if (*velocity < filter_threshold) {
        filtered_value = one_euro_filter(event->value, &axis->filtered,
                                         &axis->dx_filtered, dt_us) / 1000;
    } else {
        filtered_value = event->value;
        axis->filtered = event->value * 1000;
    }

    int32_t scaled = filtered_value * scale + axis->remainder;
    event->value = scaled / 100;
    axis->remainder = scaled % 100;

    s->last_time_us = now_us;
    axis->has_pending = false;

    LOG_DBG("%s v=%d scale=%d.%02d in=%d out=%d",
            is_scroll ? "scroll" : "ptr",
            *velocity, scale / 100, scale % 100,
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
