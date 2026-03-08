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
#define EMA_ALPHA 400
#define SCROLL_MIN_SCALE 50
#define SCROLL_MAX_SCALE 600

struct sigmoid_accel_config {
    int32_t min_scale;
    int32_t max_scale;
    int32_t threshold;
    int32_t steepness;
};

struct axis_state {
    int32_t pending;
    int32_t filtered;
    int32_t dx_filtered;
    int32_t remainder;
    bool has_pending;
};

struct input_group_state {
    struct axis_state primary;
    struct axis_state secondary;
    int64_t last_time_us;
    int32_t velocity;
    bool initialized;
};

struct sigmoid_accel_state {
    struct input_group_state pointer;
    struct input_group_state scroll;
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

    return SIGMOID_LUT[index] + (SIGMOID_LUT[index + 1] - SIGMOID_LUT[index]) * frac / 50;
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

static int32_t one_euro_filter(int32_t x, struct axis_state *axis, int32_t dt_us) {
    int32_t dx = (x * 1000 - axis->filtered);
    if (dt_us > 0) {
        dx = dx * 1000 / dt_us;
    }

    int32_t alpha_d = calc_alpha(ONE_EURO_DCUTOFF, dt_us);
    axis->dx_filtered = (axis->dx_filtered * (1000 - alpha_d) + dx * alpha_d) / 1000;

    int32_t cutoff = ONE_EURO_MINCUTOFF + ONE_EURO_BETA * abs(axis->dx_filtered) / 1000;
    if (cutoff > 50000) cutoff = 50000;

    int32_t alpha = calc_alpha(cutoff, dt_us);
    axis->filtered = (axis->filtered * (1000 - alpha) + x * 1000 * alpha) / 1000;
    return axis->filtered;
}

static int32_t calculate_scale(int32_t velocity, int32_t threshold, int32_t steepness,
                               int32_t min_scale, int32_t max_scale) {
    int32_t delta = velocity - threshold;
    int32_t sigmoid_input = delta * steepness / 100;
    int32_t sigmoid_val = sigmoid_lookup(sigmoid_input);
    return min_scale + (max_scale - min_scale) * sigmoid_val / 100;
}

static void reset_axis(struct axis_state *a) {
    a->pending = 0;
    a->filtered = 0;
    a->dx_filtered = 0;
    a->remainder = 0;
    a->has_pending = false;
}

static void reset_group(struct input_group_state *g) {
    reset_axis(&g->primary);
    reset_axis(&g->secondary);
    g->velocity = 0;
    g->initialized = false;
}

static int32_t process_axis(const struct sigmoid_accel_config *config,
                            struct input_group_state *group,
                            struct axis_state *axis,
                            int32_t value,
                            int64_t dt_us,
                            bool is_scroll) {
    axis->pending = value;
    axis->has_pending = true;

    int32_t d1 = axis->pending;
    int32_t d2 = group->secondary.has_pending ? group->secondary.pending : 0;
    int32_t magnitude = isqrt(d1 * d1 + d2 * d2);
    int32_t instant_velocity = magnitude * 10000 / (int32_t)dt_us;

    if (!group->initialized) {
        group->velocity = instant_velocity;
        axis->filtered = value * 1000;
        group->initialized = true;
    }

    group->velocity = (group->velocity * (1000 - EMA_ALPHA) + instant_velocity * EMA_ALPHA) / 1000;

    int32_t scale;
    int32_t filter_threshold;

    if (is_scroll) {
        scale = calculate_scale(group->velocity, config->threshold / 2, config->steepness,
                                SCROLL_MIN_SCALE, SCROLL_MAX_SCALE);
        filter_threshold = config->threshold / 4;
    } else {
        scale = calculate_scale(group->velocity, config->threshold, config->steepness,
                                config->min_scale, config->max_scale);
        filter_threshold = config->threshold / 2;
    }

    int32_t filtered_value;
    if (group->velocity < filter_threshold) {
        filtered_value = one_euro_filter(value, axis, dt_us) / 1000;
    } else {
        filtered_value = value;
        axis->filtered = value * 1000;
    }

    int32_t scaled = filtered_value * scale + axis->remainder;
    int32_t output = scaled / 100;
    axis->remainder = scaled % 100;

    axis->has_pending = false;

    LOG_DBG("%s v=%d scale=%d.%02d in=%d out=%d",
            is_scroll ? "scroll" : "ptr",
            group->velocity, scale / 100, scale % 100,
            filtered_value, output);

    return output;
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

    struct input_group_state *group;
    struct axis_state *axis;
    bool is_scroll;

    switch (event->code) {
    case INPUT_REL_X:
        group = &s->pointer;
        axis = &group->primary;
        is_scroll = false;
        break;
    case INPUT_REL_Y:
        group = &s->pointer;
        axis = &group->secondary;
        is_scroll = false;
        break;
    case INPUT_REL_WHEEL:
        group = &s->scroll;
        axis = &group->primary;
        is_scroll = true;
        break;
    case INPUT_REL_HWHEEL:
        group = &s->scroll;
        axis = &group->secondary;
        is_scroll = true;
        break;
    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t dt_us = now_us - group->last_time_us;

    if (dt_us <= 0 || dt_us > 100000) {
        reset_group(group);
        dt_us = 8000;
    }

    event->value = process_axis(config, group, axis, event->value, dt_us, is_scroll);
    group->last_time_us = now_us;

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
