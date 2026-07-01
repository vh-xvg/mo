/*
 * MIT License
 *
 * Copyright (c) 2026 Adrian Port
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef VIBSENSE_FAKE_WINDOW_H
#define VIBSENSE_FAKE_WINDOW_H

#include <stdint.h>
#include "vibsense.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Deterministic fake-window generator for EVE/live-display testing.
 *
 * The generator owns internal static sample buffers and points window->samples
 * at those buffers.  The pointer remains valid until the same slot is reused.
 * It is not thread-safe.
 *
 * Important for dual-sensor display testing:
 * - vibsense_fake_fill_both_windows() fills windows[0] with IIS3DWBG1 data and
 *   windows[1] with ADXL355 data.
 * - The two windows are based on the same underlying vibration case, but have
 *   slightly different phase/noise/scale so the two polar vectors are visibly
 *   distinct while still plausibly tracking the same engine vibration.
 */

#define VIBSENSE_FAKE_SCENARIO_COUNT 12u
#define VIBSENSE_FAKE_MAX_SAMPLES    1600u
#define VIBSENSE_FAKE_NUM_SLOTS      2u

#define VIBSENSE_FAKE_SLOT_PRIMARY   0u
#define VIBSENSE_FAKE_SLOT_SECONDARY 1u

typedef struct
{
    uint32_t scenario_index;       /* 0..11; UINT32_MAX means auto-advance */
    uint32_t revolutions;          /* 0 means use canned scenario value */
    uint32_t samples_per_rev;      /* 0 means use canned scenario value */
    uint32_t tick_rate_hz;         /* 0 means 1000000 */
    uint8_t  accel_source;         /* 0 means canned/default source */
    uint8_t  accel_range_g;        /* 0 means source-appropriate default */
    uint16_t raw_sample_bits;      /* 0 means source-appropriate default */

    /*
     * Optional test perturbations.
     * Use these when you want a display vector that is not exactly the canned
     * case.  A zero field means "no extra adjustment".
     */
    double ips_scale;              /* 0 means 1.0 */
    double phase_offset_deg;       /* added to canned phase */
    double noise_scale;            /* 0 means 1.0 */
} vibsense_fake_config_t;

void vibsense_fake_reset(void);
void vibsense_fake_default_config(vibsense_fake_config_t *cfg);

/* Backward-compatible API: auto-advances, fills primary slot, IIS by default. */
int vibsense_fake_fill_window(vibsense_window_t *window);

/* Backward-compatible explicit config API. */
int vibsense_fake_fill_window_ex(vibsense_window_t *window,
                                 const vibsense_fake_config_t *cfg);

/*
 * Fill a specified static slot.  This is useful when two windows must be valid
 * at the same time.
 */
int vibsense_fake_fill_window_slot(vibsense_window_t *window,
                                   uint32_t slot,
                                   const vibsense_fake_config_t *cfg);

/* Convenience: fill one source into primary slot. */
int vibsense_fake_fill_window_source(vibsense_window_t *window,
                                     uint8_t accel_source);

/*
 * Convenience: fill two simultaneous windows:
 *   windows[0] = IIS3DWBG1
 *   windows[1] = ADXL355
 *
 * The scenario auto-advances once per call, not once per sensor.
 */
int vibsense_fake_fill_both_windows(vibsense_window_t windows[2]);

/* Same as above, but with explicit base config. */
int vibsense_fake_fill_both_windows_ex(vibsense_window_t windows[2],
                                       const vibsense_fake_config_t *base_cfg);

const char *vibsense_fake_scenario_name(uint32_t scenario_index);
const char *vibsense_fake_source_name(uint8_t accel_source);

#ifdef __cplusplus
}
#endif

#endif /* VIBSENSE_FAKE_WINDOW_H */
