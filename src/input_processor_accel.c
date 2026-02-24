/*
 * Copyright (c) 2024 The ZMK Contributors
 * Modifications (c) 2025 yamaryu211
 *
 * SPDX-License-Identifier: MIT
 *
 * ZMK Input Processor Acceleration
 * Based on oleksandrmaslov's implementation with nuovotaka's features integrated.
 *
 * Supports two modes:
 *   Level 1 (Simple): Input magnitude-based acceleration curves
 *   Level 2 (Standard): Speed-based acceleration (original oleksandrmaslov logic)
 *
 * Both levels support: DPI adjustment, Y-axis boost, presets, track-remainders.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#define DT_DRV_COMPAT zmk_input_processor_acceleration

#ifndef CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY
#define CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY 50
#endif

#define ACCEL_MAX_CODES 4
#define SCALE 1000
#define DPI_REFERENCE 800

/* =============================================================================
 * Preset definitions — applied at compile time via Kconfig
 * Values from nuovotaka's implementation
 * ============================================================================= */

/* Default values (Custom or no preset) — use device tree properties */
#define PRESET_SENSITIVITY    1000
#define PRESET_MAX_FACTOR     3500
#define PRESET_MIN_FACTOR     1000
#define PRESET_SPEED_THRESH   1000
#define PRESET_SPEED_MAX      6000
#define PRESET_CURVE_TYPE     1
#define PRESET_Y_BOOST        1000
#define PRESET_SENSOR_DPI     800
#define PRESET_ACCEL_EXP      1

#if defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_OFFICE_OPTICAL)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1000
#  define PRESET_MAX_FACTOR   2200
#  define PRESET_MIN_FACTOR   980
#  define PRESET_SPEED_THRESH 700
#  define PRESET_SPEED_MAX    2600
#  define PRESET_CURVE_TYPE   1
#  define PRESET_Y_BOOST      1080
#  define PRESET_SENSOR_DPI   800

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_OFFICE_LASER)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1000
#  define PRESET_MAX_FACTOR   1500
#  define PRESET_MIN_FACTOR   1000
#  define PRESET_SPEED_THRESH 1000
#  define PRESET_SPEED_MAX    2000
#  define PRESET_CURVE_TYPE   0
#  define PRESET_Y_BOOST      1000
#  define PRESET_SENSOR_DPI   1600

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_OFFICE_TRACKBALL)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1500
#  define PRESET_MAX_FACTOR   2000
#  define PRESET_MIN_FACTOR   950
#  define PRESET_SPEED_THRESH 800
#  define PRESET_SPEED_MAX    2400
#  define PRESET_CURVE_TYPE   1
#  define PRESET_Y_BOOST      1100
#  define PRESET_SENSOR_DPI   400

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_OFFICE_TRACKPAD)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1200
#  define PRESET_MAX_FACTOR   1800
#  define PRESET_MIN_FACTOR   900
#  define PRESET_SPEED_THRESH 600
#  define PRESET_SPEED_MAX    2200
#  define PRESET_CURVE_TYPE   0
#  define PRESET_Y_BOOST      1000
#  define PRESET_SENSOR_DPI   1000

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_GAMING_OPTICAL)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1000
#  define PRESET_MAX_FACTOR   2500
#  define PRESET_MIN_FACTOR   950
#  define PRESET_SPEED_THRESH 550
#  define PRESET_SPEED_MAX    2800
#  define PRESET_CURVE_TYPE   2
#  define PRESET_Y_BOOST      1120
#  define PRESET_SENSOR_DPI   1200

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_GAMING_LASER)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  600
#  define PRESET_MAX_FACTOR   2500
#  define PRESET_MIN_FACTOR   950
#  define PRESET_SPEED_THRESH 550
#  define PRESET_SPEED_MAX    2800
#  define PRESET_CURVE_TYPE   2
#  define PRESET_Y_BOOST      1120
#  define PRESET_SENSOR_DPI   3200

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_GAMING_TRACKBALL)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1200
#  define PRESET_MAX_FACTOR   2300
#  define PRESET_MIN_FACTOR   940
#  define PRESET_SPEED_THRESH 600
#  define PRESET_SPEED_MAX    2700
#  define PRESET_CURVE_TYPE   2
#  define PRESET_Y_BOOST      1150
#  define PRESET_SENSOR_DPI   800

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_GAMING_TRACKPAD)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1100
#  define PRESET_MAX_FACTOR   2200
#  define PRESET_MIN_FACTOR   920
#  define PRESET_SPEED_THRESH 500
#  define PRESET_SPEED_MAX    2400
#  define PRESET_CURVE_TYPE   1
#  define PRESET_Y_BOOST      1050
#  define PRESET_SENSOR_DPI   1200

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_HIGH_SENS_OPTICAL)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1100
#  define PRESET_MAX_FACTOR   2800
#  define PRESET_MIN_FACTOR   900
#  define PRESET_SPEED_THRESH 450
#  define PRESET_SPEED_MAX    2400
#  define PRESET_CURVE_TYPE   1
#  define PRESET_Y_BOOST      1150
#  define PRESET_SENSOR_DPI   1600

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_HIGH_SENS_LASER)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  500
#  define PRESET_MAX_FACTOR   2800
#  define PRESET_MIN_FACTOR   900
#  define PRESET_SPEED_THRESH 450
#  define PRESET_SPEED_MAX    2400
#  define PRESET_CURVE_TYPE   1
#  define PRESET_Y_BOOST      1150
#  define PRESET_SENSOR_DPI   6400

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_HIGH_SENS_TRACKBALL)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1400
#  define PRESET_MAX_FACTOR   2600
#  define PRESET_MIN_FACTOR   880
#  define PRESET_SPEED_THRESH 500
#  define PRESET_SPEED_MAX    2500
#  define PRESET_CURVE_TYPE   1
#  define PRESET_Y_BOOST      1200
#  define PRESET_SENSOR_DPI   800

#elif defined(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_HIGH_SENS_TRACKPAD)
#  undef PRESET_SENSITIVITY
#  undef PRESET_MAX_FACTOR
#  undef PRESET_MIN_FACTOR
#  undef PRESET_SPEED_THRESH
#  undef PRESET_SPEED_MAX
#  undef PRESET_CURVE_TYPE
#  undef PRESET_Y_BOOST
#  undef PRESET_SENSOR_DPI
#  define PRESET_SENSITIVITY  1300
#  define PRESET_MAX_FACTOR   2400
#  define PRESET_MIN_FACTOR   850
#  define PRESET_SPEED_THRESH 400
#  define PRESET_SPEED_MAX    2300
#  define PRESET_CURVE_TYPE   1
#  define PRESET_Y_BOOST      1100
#  define PRESET_SENSOR_DPI   1200

#endif /* presets */

/* =============================================================================
 * Data structures
 * ============================================================================= */

static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state);

static const struct zmk_input_processor_driver_api accel_api = {
    .handle_event = accel_handle_event,
};

struct accel_config {
    uint8_t  input_type;
    const uint16_t *codes;
    uint32_t codes_count;
    bool     track_remainders;
    /* Core acceleration parameters */
    uint16_t sensitivity;           /* x1000 (1000 = 1.0x) */
    uint16_t min_factor;            /* x1000 */
    uint16_t max_factor;            /* x1000 */
    uint32_t speed_threshold;       /* counts/sec */
    uint32_t speed_max;             /* counts/sec */
    uint8_t  acceleration_exponent; /* 1=linear, 2=quadratic, ... */
    /* Extended parameters (from nuovotaka) */
    uint8_t  curve_type;            /* Level 1: 0=Linear, 1=Mild, 2=Strong */
    uint16_t sensor_dpi;            /* Sensor DPI */
    uint16_t y_boost;               /* Y-axis boost x1000 */
    uint8_t  level;                 /* 1=Simple, 2=Standard */
};

struct accel_data {
    int64_t last_time_ms[ACCEL_MAX_CODES];  /* per-axis timestamp */
    int32_t last_phys[ACCEL_MAX_CODES];     /* last raw delta per axis */
    int16_t remainders[ACCEL_MAX_CODES];    /* thousandths remainder per axis */
};

/* =============================================================================
 * Utility functions
 * ============================================================================= */

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline bool code_to_index(const struct accel_config *cfg, uint16_t code, uint32_t *out_idx) {
    for (uint32_t i = 0; i < cfg->codes_count; ++i) {
        if (cfg->codes[i] == code) {
            if (out_idx) *out_idx = i;
            return true;
        }
    }
    return false;
}

/* DPI-adjusted sensitivity: sensitivity * DPI_REFERENCE / sensor_dpi */
static inline uint32_t compute_dpi_sensitivity(const struct accel_config *cfg) {
    uint32_t dpi = clamp_u32(cfg->sensor_dpi, 200, 8000);
    if (dpi == DPI_REFERENCE) {
        return cfg->sensitivity;
    }
    uint32_t adjusted = (uint32_t)((uint64_t)cfg->sensitivity * DPI_REFERENCE / dpi);
    return clamp_u32(adjusted, 100, 5000);
}

/* =============================================================================
 * Level 2 (Standard) — Speed-based acceleration (oleksandrmaslov's core logic)
 * ============================================================================= */

static uint32_t pow_scaled(uint32_t t, uint8_t exp) {
    t = clamp_u32(t, 0, SCALE);
    if (exp <= 1) return t;
    uint64_t acc = t;
    for (uint8_t i = 1; i < exp; ++i) {
        acc = (acc * t) / SCALE;
    }
    if (acc > UINT32_MAX) acc = UINT32_MAX;
    return (uint32_t)acc;
}

static uint32_t compute_factor_scaled(const struct accel_config *cfg, uint32_t cps) {
    const uint32_t f_min = clamp_u32(cfg->min_factor, 100, 20000);
    const uint32_t f_max = clamp_u32(cfg->max_factor, f_min, 20000);
    const uint32_t v1 = cfg->speed_threshold;
    const uint32_t v2 = (cfg->speed_max > v1) ? cfg->speed_max : (v1 + 1);
    const uint8_t  e  = cfg->acceleration_exponent ? cfg->acceleration_exponent : 1;

    const uint32_t base = (f_min > SCALE) ? f_min : SCALE;

    if (cps <= v1) {
        uint32_t t = (v1 == 0) ? SCALE : (uint32_t)((uint64_t)cps * SCALE / v1);
        uint32_t shaped = pow_scaled(t, e);
        int32_t span = (int32_t)base - (int32_t)f_min;
        int32_t f = (int32_t)f_min + (int32_t)((int64_t)span * shaped / SCALE);
        return clamp_u32((uint32_t)f, f_min, base);
    } else if (cps >= v2) {
        return f_max;
    } else {
        uint32_t t = (uint32_t)((uint64_t)(cps - v1) * SCALE / (v2 - v1));
        uint32_t shaped = pow_scaled(t, e);
        int32_t span = (int32_t)f_max - (int32_t)base;
        int32_t f = (int32_t)base + (int32_t)((int64_t)span * shaped / SCALE);
        return clamp_u32((uint32_t)f, base, f_max);
    }
}

/* =============================================================================
 * Level 1 (Simple) — Input magnitude-based acceleration (nuovotaka's logic)
 * ============================================================================= */

static uint32_t compute_level1_curve_factor(const struct accel_config *cfg, int32_t abs_input) {
    uint32_t curve_factor = SCALE;
    uint32_t max_add = (cfg->max_factor > SCALE) ? (cfg->max_factor - SCALE) : 0;

    switch (cfg->curve_type) {
    case 0: /* Linear */
        {
            uint32_t add = (uint32_t)abs_input * 10;
            if (add > max_add) add = max_add;
            curve_factor = SCALE + add;
        }
        break;
    case 1: /* Mild — quadratic */
        {
            uint64_t add = (uint64_t)abs_input * abs_input * 25 / 100;
            if (add > max_add) add = max_add;
            curve_factor = SCALE + (uint32_t)add;
        }
        break;
    case 2: /* Strong — quadratic */
        {
            uint64_t add = (uint64_t)abs_input * abs_input * 50 / 100;
            if (add > max_add) add = max_add;
            curve_factor = SCALE + (uint32_t)add;
        }
        break;
    default:
        /* Fallback to Mild */
        {
            uint64_t add = (uint64_t)abs_input * abs_input * 25 / 100;
            if (add > max_add) add = max_add;
            curve_factor = SCALE + (uint32_t)add;
        }
        break;
    }

    return clamp_u32(curve_factor, SCALE, cfg->max_factor);
}

/* =============================================================================
 * Event handler — dispatches to Level 1 or Level 2
 *
 * When track_remainders is enabled, we compute at full precision (keeping an
 * extra factor of SCALE) and only divide once at the end so that fractional
 * thousandths are accumulated across events.
 * ============================================================================= */

static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    if (event->type != cfg->input_type) {
        return 0;
    }

    uint32_t idx = 0;
    if (!code_to_index(cfg, event->code, &idx)) {
        return 0;
    }

    const int32_t raw = event->value;
    const int64_t now = k_uptime_get();

    if (raw == 0) {
        data->last_time_ms[idx] = now;
        return 0;
    }

    /* DPI-adjusted sensitivity */
    uint32_t dpi_sens = compute_dpi_sensitivity(cfg);

    /*
     * result_scaled holds the value multiplied by SCALE so that we can
     * accumulate fractional remainders when track_remainders is on.
     * For the non-remainder path we just divide by SCALE at the end.
     */
    int64_t result_scaled;

    if (cfg->level == 1) {
        /* ===== Level 1 (Simple): Input magnitude-based ===== */
        /* raw * dpi_sens gives us a value scaled by SCALE */
        result_scaled = (int64_t)raw * (int64_t)dpi_sens;

        /* Apply curve based on input magnitude */
        int32_t abs_input = abs(raw);
        if (abs_input > 1) {
            uint32_t curve_factor = compute_level1_curve_factor(cfg, abs_input);
            if (curve_factor > SCALE) {
                result_scaled = result_scaled * (int64_t)curve_factor / SCALE;
            }
        }
    } else {
        /* ===== Level 2 (Standard): Speed-based ===== */
        uint32_t dt_ms = 1;
        if (data->last_time_ms[idx] > 0 && now > data->last_time_ms[idx]) {
            int64_t diff = now - data->last_time_ms[idx];
            if (diff > 100) diff = 100;
            dt_ms = (uint32_t)diff;
        }

        uint32_t cps = (uint32_t)(((uint64_t)abs(raw) * 1000ULL) / dt_ms);
        uint32_t factor = compute_factor_scaled(cfg, cps);

        /* Direction reversal: reset to 1.0x */
        if ((int64_t)data->last_phys[idx] * (int64_t)raw < 0 && factor > SCALE) {
            factor = SCALE;
        }

        /* raw * dpi_sens * factor / SCALE keeps one factor of SCALE */
        result_scaled = (int64_t)raw * (int64_t)dpi_sens;
        result_scaled = result_scaled * (int64_t)factor / SCALE;
    }

    /* Y-axis boost (shared between levels) */
    if (cfg->y_boost > SCALE && event->code == INPUT_REL_Y) {
        result_scaled = result_scaled * (int64_t)cfg->y_boost / SCALE;
    }

    /* Now result_scaled = final_value * SCALE.  Divide by SCALE to get output. */
    if (cfg->track_remainders) {
        int64_t total = result_scaled + (int64_t)data->remainders[idx];
        int32_t out = (int32_t)(total / SCALE);
        int32_t rem = (int32_t)(total - (int64_t)out * SCALE);

        /* Minimum movement guarantee */
        if (raw != 0 && out == 0) {
            out = (raw > 0) ? 1 : -1;
        }

        event->value = clamp_i32(out, -32768, 32767);
        data->remainders[idx] = (int16_t)rem;
    } else {
        int64_t result = result_scaled / SCALE;

        /* Minimum movement guarantee */
        if (raw != 0 && result == 0) {
            result = (raw > 0) ? 1 : -1;
        }

        result = (result > 32767) ? 32767 : ((result < -32768) ? -32768 : result);
        event->value = (int32_t)result;
    }

    data->last_phys[idx] = raw;
    data->last_time_ms[idx] = now;
    return 0;
}

/* =============================================================================
 * Device instantiation macro
 *
 * When a preset is active, PRESET_* macros override device tree defaults.
 * When CUSTOM preset is selected, device tree properties are used directly.
 * ============================================================================= */

/* Helper: use preset value if a preset is active, otherwise use DT property */
#ifdef CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_CUSTOM
#  define ACCEL_PROP_OR_PRESET(inst, prop, preset_val) \
       DT_INST_PROP_OR(inst, prop, preset_val)
#else
#  define ACCEL_PROP_OR_PRESET(inst, prop, preset_val) (preset_val)
#endif

/*
 * Codes array: use DT 'codes' property if present, else default REL_X/REL_Y.
 * DT_INST_FOREACH_PROP_ELEM is used to safely expand array properties.
 */
#define ACCEL_CODE_ELEM(node_id, prop, idx) DT_PROP_BY_IDX(node_id, prop, idx),

#define ACCEL_CODES_DEFINE(inst)                                                    \
    static const uint16_t accel_codes_##inst[] = {                                  \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, codes),                             \
            (DT_INST_FOREACH_PROP_ELEM(inst, codes, ACCEL_CODE_ELEM)),              \
            (INPUT_REL_X, INPUT_REL_Y,))                                            \
    };

#define ACCEL_CODES_COUNT(inst)                                                     \
    (sizeof(accel_codes_##inst) / sizeof(accel_codes_##inst[0]))

/* Determine level from Kconfig */
#ifdef CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL
#  define ACCEL_LEVEL CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL
#else
#  define ACCEL_LEVEL 2  /* Default to Standard (oleksandrmaslov behavior) */
#endif

#define ACCEL_INST_INIT(inst)                                                       \
    ACCEL_CODES_DEFINE(inst)                                                        \
    static const struct accel_config accel_config_##inst = {                        \
        .input_type = DT_INST_PROP_OR(inst, input_type, INPUT_EV_REL),              \
        .codes = accel_codes_##inst,                                                \
        .codes_count = ACCEL_CODES_COUNT(inst),                                     \
        .track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),          \
        .sensitivity = ACCEL_PROP_OR_PRESET(inst, sensitivity, PRESET_SENSITIVITY), \
        .min_factor = ACCEL_PROP_OR_PRESET(inst, min_factor, PRESET_MIN_FACTOR),    \
        .max_factor = ACCEL_PROP_OR_PRESET(inst, max_factor, PRESET_MAX_FACTOR),    \
        .speed_threshold =                                                          \
            ACCEL_PROP_OR_PRESET(inst, speed_threshold, PRESET_SPEED_THRESH),       \
        .speed_max = ACCEL_PROP_OR_PRESET(inst, speed_max, PRESET_SPEED_MAX),       \
        .acceleration_exponent =                                                    \
            ACCEL_PROP_OR_PRESET(inst, acceleration_exponent, PRESET_ACCEL_EXP),    \
        .curve_type = ACCEL_PROP_OR_PRESET(inst, curve_type, PRESET_CURVE_TYPE),    \
        .sensor_dpi = ACCEL_PROP_OR_PRESET(inst, sensor_dpi, PRESET_SENSOR_DPI),    \
        .y_boost = ACCEL_PROP_OR_PRESET(inst, y_boost, PRESET_Y_BOOST),             \
        .level = ACCEL_LEVEL,                                                       \
    };                                                                              \
    static struct accel_data accel_data_##inst = {0};                               \
    DEVICE_DT_INST_DEFINE(inst,                                                     \
                          NULL,                                                     \
                          NULL,                                                     \
                          &accel_data_##inst,                                       \
                          &accel_config_##inst,                                     \
                          POST_KERNEL,                                              \
                          CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY,                 \
                          &accel_api);

DT_INST_FOREACH_STATUS_OKAY(ACCEL_INST_INIT)
