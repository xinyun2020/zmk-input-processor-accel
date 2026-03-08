/*
 * Copyright (c) 2024 Alice Zhang
 * SPDX-License-Identifier: MIT
 *
 * Apple Magic Trackpad-style acceleration input processor for ZMK
 *
 * Features:
 * - True sigmoid curve (lookup table, not linear approximation)
 * - Combined XY velocity for consistent diagonal movement
 * - Velocity averaging across multiple samples (smooths acceleration changes)
 * - 1 Euro filter for jitter reduction at slow speeds
 */

#define DT_DRV_COMPAT zmk_input_processor_sigmoid_accel

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Number of velocity samples to average */
#define VELOCITY_HISTORY_SIZE 4

/* 1 Euro filter parameters (scaled by 1000) */
#define ONE_EURO_MINCUTOFF 1000   /* 1.0 Hz - jitter reduction */
#define ONE_EURO_BETA 100        /* 0.1 - speed coefficient */
#define ONE_EURO_DCUTOFF 1000    /* 1.0 Hz - derivative cutoff */

struct sigmoid_accel_config {
    int32_t min_scale;   /* Minimum scale (at slow speeds), /100 */
    int32_t max_scale;   /* Maximum scale (at fast speeds), /100 */
    int32_t threshold;   /* Velocity threshold for curve center */
    int32_t steepness;   /* Sigmoid steepness, /100 */
};

struct sigmoid_accel_state {
    /* Current event accumulator (X and Y arrive separately) */
    int32_t pending_x;
    int32_t pending_y;
    bool has_x;
    bool has_y;

    /* Timing */
    int64_t last_time_us;

    /* Velocity history for averaging */
    int32_t velocity_history[VELOCITY_HISTORY_SIZE];
    int velocity_index;
    int velocity_count;

    /* 1 Euro filter state (scaled by 1000) */
    int32_t filtered_x;
    int32_t filtered_y;
    int32_t dx_filtered;
    int32_t dy_filtered;
    bool filter_initialized;

    /* Sub-pixel remainders */
    int32_t remainder_x;
    int32_t remainder_y;
};

/*
 * True sigmoid lookup table
 * Maps input range [-300, 300] to output [0, 100]
 * Pre-computed: sigmoid(x/100) * 100 for x = -300 to 300, step 50
 *
 * sigmoid(x) = 1 / (1 + e^(-x))
 */
static const int8_t SIGMOID_LUT[13] = {
    5,    /* x = -3.0: sigmoid = 0.047 */
    7,    /* x = -2.5: sigmoid = 0.076 */
    12,   /* x = -2.0: sigmoid = 0.119 */
    18,   /* x = -1.5: sigmoid = 0.182 */
    27,   /* x = -1.0: sigmoid = 0.269 */
    38,   /* x = -0.5: sigmoid = 0.378 */
    50,   /* x =  0.0: sigmoid = 0.500 */
    62,   /* x =  0.5: sigmoid = 0.622 */
    73,   /* x =  1.0: sigmoid = 0.731 */
    82,   /* x =  1.5: sigmoid = 0.818 */
    88,   /* x =  2.0: sigmoid = 0.881 */
    93,   /* x =  2.5: sigmoid = 0.924 */
    95,   /* x =  3.0: sigmoid = 0.953 */
};

/*
 * True sigmoid function using lookup table with interpolation
 * Input: x_scaled in range [-300, 300] (representing -3.0 to 3.0)
 * Output: 0-100 (representing 0.0 to 1.0)
 */
static int32_t sigmoid_lookup(int32_t x_scaled) {
    /* Clamp to valid range */
    if (x_scaled <= -300) return 5;
    if (x_scaled >= 300) return 95;

    /* Map to LUT index: [-300,300] -> [0,12] */
    int32_t shifted = x_scaled + 300;  /* 0 to 600 */
    int32_t index = shifted / 50;       /* 0 to 12 */
    int32_t frac = shifted % 50;        /* 0 to 49 */

    /* Linear interpolation between LUT entries */
    if (index >= 12) return SIGMOID_LUT[12];

    int32_t v0 = SIGMOID_LUT[index];
    int32_t v1 = SIGMOID_LUT[index + 1];

    return v0 + (v1 - v0) * frac / 50;
}

/*
 * Integer square root (Newton's method)
 */
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

/*
 * Calculate smoothing alpha for 1 Euro filter
 * alpha = 1 / (1 + tau/dt) where tau = 1/(2*pi*fc)
 * Returns alpha scaled by 1000
 */
static int32_t calc_alpha(int32_t cutoff_hz_x1000, int32_t dt_us) {
    if (dt_us <= 0) dt_us = 1000;

    /* tau = 1000000 / (2 * pi * cutoff_hz) in microseconds */
    /* Simplified: tau_us = 159155 * 1000 / cutoff_hz_x1000 */
    int32_t tau_us = 159155000 / cutoff_hz_x1000;

    /* alpha = dt / (dt + tau) scaled by 1000 */
    int32_t alpha = (int32_t)dt_us * 1000 / ((int32_t)dt_us + tau_us);

    /* Clamp to valid range */
    if (alpha < 10) alpha = 10;
    if (alpha > 990) alpha = 990;

    return alpha;
}

/*
 * Apply 1 Euro filter to reduce jitter at slow speeds
 * Returns filtered value, updates filter state
 */
static int32_t one_euro_filter(int32_t x, int32_t *x_filtered, int32_t *dx_filtered,
                                int32_t dt_us, int32_t velocity) {
    /* Calculate derivative */
    int32_t dx = (x - *x_filtered) * 1000;
    if (dt_us > 0) {
        dx = dx * 1000 / dt_us;  /* dx per millisecond, scaled */
    }

    /* Filter the derivative */
    int32_t alpha_d = calc_alpha(ONE_EURO_DCUTOFF, dt_us);
    *dx_filtered = (*dx_filtered * (1000 - alpha_d) + dx * alpha_d) / 1000;

    /* Adaptive cutoff: higher velocity = higher cutoff = less smoothing */
    int32_t cutoff = ONE_EURO_MINCUTOFF + ONE_EURO_BETA * abs(*dx_filtered) / 1000;
    if (cutoff > 50000) cutoff = 50000;  /* Cap at 50 Hz */

    /* Filter the signal */
    int32_t alpha = calc_alpha(cutoff, dt_us);
    *x_filtered = (*x_filtered * (1000 - alpha) + x * 1000 * alpha / 1000) / 1000;

    return *x_filtered;
}

/*
 * Add velocity to history and return averaged velocity
 */
static int32_t add_velocity(struct sigmoid_accel_state *state, int32_t velocity) {
    state->velocity_history[state->velocity_index] = velocity;
    state->velocity_index = (state->velocity_index + 1) % VELOCITY_HISTORY_SIZE;
    if (state->velocity_count < VELOCITY_HISTORY_SIZE) {
        state->velocity_count++;
    }

    /* Calculate weighted average (recent samples weighted more) */
    int32_t sum = 0;
    int32_t weight_sum = 0;
    for (int i = 0; i < state->velocity_count; i++) {
        int weight = i + 1;  /* 1, 2, 3, 4 */
        sum += state->velocity_history[i] * weight;
        weight_sum += weight;
    }

    return weight_sum > 0 ? sum / weight_sum : velocity;
}

/*
 * Calculate scale factor using true sigmoid curve
 */
static int32_t calculate_scale(const struct sigmoid_accel_config *config,
                                int32_t velocity) {
    /* Calculate how far above/below threshold */
    int32_t delta = velocity - config->threshold;

    /* Apply steepness (steepness is /100) */
    int32_t sigmoid_input = delta * config->steepness / 100;

    /* Get sigmoid value using true curve (0-100) */
    int32_t sigmoid_val = sigmoid_lookup(sigmoid_input);

    /* Interpolate between min and max scale */
    int32_t scale = config->min_scale +
                    (config->max_scale - config->min_scale) * sigmoid_val / 100;

    return scale;
}

static int sigmoid_accel_handle_event(const struct device *dev,
                                       struct input_event *event,
                                       uint32_t param1,
                                       uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    const struct sigmoid_accel_config *config = dev->config;
    struct sigmoid_accel_state *proc_state = dev->data;

    /* Only process relative X/Y events */
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Get current time */
    int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    int64_t dt_us = now_us - proc_state->last_time_us;
    if (dt_us <= 0 || dt_us > 100000) {
        dt_us = 8000;  /* Default to ~8ms (125Hz) */
    }

    /* Accumulate X and Y (they arrive as separate events) */
    if (event->code == INPUT_REL_X) {
        proc_state->pending_x = event->value;
        proc_state->has_x = true;
    } else {
        proc_state->pending_y = event->value;
        proc_state->has_y = true;
    }

    /* Calculate combined velocity using both axes */
    int32_t dx = proc_state->has_x ? proc_state->pending_x : 0;
    int32_t dy = proc_state->has_y ? proc_state->pending_y : 0;

    /* Magnitude of movement vector (scaled for velocity calc) */
    int32_t magnitude = isqrt(dx * dx + dy * dy);

    /* Velocity in units per 10ms */
    int32_t instant_velocity = magnitude * 10000 / (int32_t)dt_us;

    /* Average velocity across history for smoother acceleration */
    int32_t avg_velocity = add_velocity(proc_state, instant_velocity);

    /* Calculate scale factor based on averaged velocity */
    int32_t scale = calculate_scale(config, avg_velocity);

    /* Initialize 1 Euro filter if needed */
    if (!proc_state->filter_initialized) {
        proc_state->filtered_x = event->value * 1000;
        proc_state->filtered_y = event->value * 1000;
        proc_state->dx_filtered = 0;
        proc_state->dy_filtered = 0;
        proc_state->filter_initialized = true;
    }

    /* Apply 1 Euro filter for jitter reduction at slow speeds */
    int32_t filtered_value;
    int32_t *remainder;

    if (event->code == INPUT_REL_X) {
        /* Only apply heavy filtering at very slow speeds */
        if (avg_velocity < config->threshold / 2) {
            filtered_value = one_euro_filter(event->value,
                                              &proc_state->filtered_x,
                                              &proc_state->dx_filtered,
                                              dt_us, avg_velocity) / 1000;
        } else {
            filtered_value = event->value;
            proc_state->filtered_x = event->value * 1000;
        }
        remainder = &proc_state->remainder_x;
    } else {
        if (avg_velocity < config->threshold / 2) {
            filtered_value = one_euro_filter(event->value,
                                              &proc_state->filtered_y,
                                              &proc_state->dy_filtered,
                                              dt_us, avg_velocity) / 1000;
        } else {
            filtered_value = event->value;
            proc_state->filtered_y = event->value * 1000;
        }
        remainder = &proc_state->remainder_y;
    }

    /* Apply scale with remainder tracking for sub-pixel precision */
    int32_t scaled_with_remainder = filtered_value * scale + *remainder;
    event->value = scaled_with_remainder / 100;
    *remainder = scaled_with_remainder % 100;

    /* Update timing */
    proc_state->last_time_us = now_us;

    /* Clear pending after processing (ready for next pair) */
    if (event->code == INPUT_REL_X) {
        proc_state->has_x = false;
    } else {
        proc_state->has_y = false;
    }

    LOG_DBG("Accel: v_inst=%d v_avg=%d scale=%d.%02d in=%d out=%d",
            instant_velocity, avg_velocity,
            scale / 100, scale % 100,
            filtered_value, event->value);

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
