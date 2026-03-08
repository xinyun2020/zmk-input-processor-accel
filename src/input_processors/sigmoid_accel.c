/*
 * Copyright (c) 2024 Alice Zhang
 * SPDX-License-Identifier: MIT
 *
 * Sigmoid acceleration input processor for ZMK
 * Provides Steam Deck-style trackpad acceleration:
 * - Slow movements remain precise (low scale)
 * - Fast movements are amplified (high scale)
 * - Smooth transition via sigmoid curve
 */

#define DT_DRV_COMPAT zmk_input_processor_sigmoid_accel

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct sigmoid_accel_config {
    int32_t min_scale;   // Minimum scale (at slow speeds), /100
    int32_t max_scale;   // Maximum scale (at fast speeds), /100
    int32_t threshold;   // Velocity threshold for curve center
    int32_t steepness;   // Sigmoid steepness, /100
};

struct sigmoid_accel_state {
    int32_t last_x;
    int32_t last_y;
    int64_t last_time_us;
    int32_t remainder_x;
    int32_t remainder_y;
};

/*
 * Fast integer approximation of sigmoid function
 * Returns value between 0 and 100 (representing 0.0 to 1.0)
 *
 * sigmoid(x) = 1 / (1 + e^(-x))
 *
 * We use a piecewise linear approximation for efficiency:
 * - x < -3: returns ~0
 * - x > 3: returns ~100
 * - linear interpolation in between
 */
static int32_t sigmoid_approx(int32_t x_scaled) {
    // x_scaled is the input multiplied by steepness
    // We want to map the range [-300, 300] to [0, 100]

    if (x_scaled <= -300) {
        return 0;
    }
    if (x_scaled >= 300) {
        return 100;
    }

    // Linear approximation in the middle range
    // Map [-300, 300] to [0, 100]
    return (x_scaled + 300) * 100 / 600;
}

/*
 * Calculate velocity-based scale factor using sigmoid curve
 * Returns scale factor multiplied by 100
 */
static int32_t calculate_scale(const struct sigmoid_accel_config *config,
                                int32_t velocity) {
    // Calculate how far above/below threshold
    int32_t delta = velocity - config->threshold;

    // Apply steepness (steepness is /100, so multiply then divide)
    int32_t sigmoid_input = delta * config->steepness / 100;

    // Get sigmoid value (0-100)
    int32_t sigmoid_val = sigmoid_approx(sigmoid_input);

    // Interpolate between min and max scale
    int32_t scale = config->min_scale +
                    (config->max_scale - config->min_scale) * sigmoid_val / 100;

    return scale;
}

/*
 * Integer square root approximation
 */
static int32_t isqrt(int32_t n) {
    if (n < 0) return 0;
    if (n < 2) return n;

    int32_t x = n;
    int32_t y = (x + 1) / 2;

    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

static int sigmoid_accel_handle_event(const struct device *dev,
                                       struct input_event *event,
                                       uint32_t param1,
                                       uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    const struct sigmoid_accel_config *config = dev->config;
    struct sigmoid_accel_state *proc_state = dev->data;

    // Only process relative X/Y events
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Get current time
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

    // Calculate time delta
    int64_t dt_us = now_us - proc_state->last_time_us;
    if (dt_us <= 0) {
        dt_us = 1000; // Default to 1ms if no previous time
    }

    // Update tracking
    int32_t dx = 0, dy = 0;
    if (event->code == INPUT_REL_X) {
        dx = event->value;
        proc_state->last_x = event->value;
    } else {
        dy = event->value;
        proc_state->last_y = event->value;
    }

    // Calculate velocity (pixels per 10ms for reasonable numbers)
    // Use the magnitude of the current delta as instantaneous velocity
    int32_t velocity = abs(event->value) * 10000 / (int32_t)dt_us;

    // Calculate scale factor based on velocity
    int32_t scale = calculate_scale(config, velocity);

    // Apply scale with remainder tracking for precision
    int32_t *remainder = (event->code == INPUT_REL_X) ?
                         &proc_state->remainder_x : &proc_state->remainder_y;

    int32_t scaled_with_remainder = event->value * scale + *remainder;
    event->value = scaled_with_remainder / 100;
    *remainder = scaled_with_remainder % 100;

    // Update time
    proc_state->last_time_us = now_us;

    LOG_DBG("Sigmoid accel: vel=%d scale=%d.%02d in=%d out=%d",
            velocity, scale / 100, scale % 100,
            (event->code == INPUT_REL_X) ? dx : dy, event->value);

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api sigmoid_accel_driver_api = {
    .handle_event = sigmoid_accel_handle_event,
};

#define SIGMOID_ACCEL_INST(n)                                                   \
    static struct sigmoid_accel_state sigmoid_accel_state_##n = {};             \
    static const struct sigmoid_accel_config sigmoid_accel_config_##n = {       \
        .min_scale = DT_INST_PROP(n, min_scale),                                \
        .max_scale = DT_INST_PROP(n, max_scale),                                \
        .threshold = DT_INST_PROP(n, threshold),                                \
        .steepness = DT_INST_PROP(n, steepness),                                \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &sigmoid_accel_state_##n,              \
                          &sigmoid_accel_config_##n, POST_KERNEL,               \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                  \
                          &sigmoid_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SIGMOID_ACCEL_INST)
