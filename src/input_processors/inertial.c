/*
 * Copyright (c) 2024 Alice Zhang
 * SPDX-License-Identifier: MIT
 *
 * Inertial scrolling input processor for ZMK
 * Provides momentum/coasting after finger lift:
 * - Tracks velocity while finger is on trackpad
 * - Continues sending decaying movement events after lift
 */

#define DT_DRV_COMPAT zmk_input_processor_inertial

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct inertial_config {
    int32_t decay_rate;      // Velocity decay per tick (percent, /100)
    int32_t min_velocity;    // Stop when velocity drops below this
    int32_t tick_ms;         // Time between inertial updates
    int32_t lift_timeout_ms; // Time without input to detect finger lift
};

struct inertial_state {
    int32_t velocity_x;
    int32_t velocity_y;
    int64_t last_input_time;
    int64_t last_inject_time;  // Track when we last injected events
    int32_t remainder_x;
    int32_t remainder_y;
    bool coasting;
    struct k_work_delayable coast_work;
    const struct device *dev;           // Our processor device
    const struct device *input_dev;     // Original input device (trackpad)
};

static void coast_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct inertial_state *state = CONTAINER_OF(dwork, struct inertial_state, coast_work);
    const struct inertial_config *config = state->dev->config;

    // First call after lift timeout - start coasting
    if (!state->coasting) {
        // Check if we have enough velocity to coast
        if (abs(state->velocity_x) < config->min_velocity &&
            abs(state->velocity_y) < config->min_velocity) {
            LOG_DBG("Inertial: velocity too low to coast");
            return;
        }
        state->coasting = true;
        state->remainder_x = 0;
        state->remainder_y = 0;
        LOG_DBG("Inertial: starting coast vel_x=%d vel_y=%d",
                state->velocity_x, state->velocity_y);
    }

    // Apply decay
    state->velocity_x = state->velocity_x * config->decay_rate / 100;
    state->velocity_y = state->velocity_y * config->decay_rate / 100;

    // Check if we should stop
    if (abs(state->velocity_x) < config->min_velocity &&
        abs(state->velocity_y) < config->min_velocity) {
        state->coasting = false;
        state->velocity_x = 0;
        state->velocity_y = 0;
        LOG_DBG("Inertial: coast stopped");
        return;
    }

    // Calculate movement with remainder tracking
    int32_t move_x = state->velocity_x + state->remainder_x;
    int32_t move_y = state->velocity_y + state->remainder_y;

    int16_t dx = move_x / 100;
    int16_t dy = move_y / 100;

    state->remainder_x = move_x % 100;
    state->remainder_y = move_y % 100;

    if ((dx != 0 || dy != 0) && state->input_dev != NULL) {
        // Record injection time to ignore these events in handler
        state->last_inject_time = k_uptime_get();

        // Inject synthetic events back through the input pipeline
        // Use sync=true on last event to trigger report
        if (dx != 0 && dy != 0) {
            input_report_rel(state->input_dev, INPUT_REL_X, dx, false, K_NO_WAIT);
            input_report_rel(state->input_dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
        } else if (dx != 0) {
            input_report_rel(state->input_dev, INPUT_REL_X, dx, true, K_NO_WAIT);
        } else {
            input_report_rel(state->input_dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
        }

        LOG_DBG("Inertial: coast dx=%d dy=%d vel_x=%d vel_y=%d",
                dx, dy, state->velocity_x, state->velocity_y);
    }

    // Schedule next tick
    k_work_schedule(dwork, K_MSEC(config->tick_ms));
}

static int inertial_handle_event(const struct device *dev,
                                  struct input_event *event,
                                  uint32_t param1,
                                  uint32_t param2,
                                  struct zmk_input_processor_state *state) {
    const struct inertial_config *config = dev->config;
    struct inertial_state *proc_state = dev->data;

    // Only process relative X/Y events
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now = k_uptime_get();

    // Ignore events that arrive shortly after we injected (likely our own)
    // This prevents the feedback loop where injected events restart coasting
    if (proc_state->coasting && (now - proc_state->last_inject_time) < 50) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Capture the input device for later synthetic event injection
    if (event->dev != NULL) {
        proc_state->input_dev = event->dev;
    }

    // If we were coasting, stop - finger is back
    if (proc_state->coasting) {
        proc_state->coasting = false;
        k_work_cancel_delayable(&proc_state->coast_work);
        LOG_DBG("Inertial: coast interrupted by new input");
    }

    // Calculate time since last input
    int64_t dt_ms = now - proc_state->last_input_time;
    if (dt_ms <= 0) dt_ms = 1;

    // Update velocity (exponential moving average)
    // velocity = pixels per 100ms, scaled by 100 for precision
    int32_t instant_vel = event->value * 10000 / (int32_t)dt_ms;

    if (event->code == INPUT_REL_X) {
        // Blend: 70% new, 30% old for smoothing
        proc_state->velocity_x = (instant_vel * 70 + proc_state->velocity_x * 30) / 100;
    } else {
        proc_state->velocity_y = (instant_vel * 70 + proc_state->velocity_y * 30) / 100;
    }

    proc_state->last_input_time = now;

    // Schedule lift detection
    k_work_schedule(&proc_state->coast_work, K_MSEC(config->lift_timeout_ms));

    return ZMK_INPUT_PROC_CONTINUE;
}

static int inertial_init(const struct device *dev) {
    struct inertial_state *state = dev->data;
    state->dev = dev;
    state->input_dev = NULL;  // Will be captured from first event
    state->coasting = false;
    state->velocity_x = 0;
    state->velocity_y = 0;
    state->remainder_x = 0;
    state->remainder_y = 0;
    state->last_input_time = 0;
    state->last_inject_time = 0;
    k_work_init_delayable(&state->coast_work, coast_work_handler);
    return 0;
}

static struct zmk_input_processor_driver_api inertial_driver_api = {
    .handle_event = inertial_handle_event,
};

#define INERTIAL_INST(n)                                                        \
    static struct inertial_state inertial_state_##n = {};                       \
    static const struct inertial_config inertial_config_##n = {                 \
        .decay_rate = DT_INST_PROP(n, decay_rate),                              \
        .min_velocity = DT_INST_PROP(n, min_velocity),                          \
        .tick_ms = DT_INST_PROP(n, tick_ms),                                    \
        .lift_timeout_ms = DT_INST_PROP(n, lift_timeout_ms),                    \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, inertial_init, NULL, &inertial_state_##n,          \
                          &inertial_config_##n, POST_KERNEL,                    \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                  \
                          &inertial_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INERTIAL_INST)
