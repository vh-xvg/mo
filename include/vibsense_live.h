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

#ifndef VIBSENSE_LIVE_H
#define VIBSENSE_LIVE_H

#include <stdint.h>
#include "vibsense.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VIBSENSE_LIVE_OK              0
#define VIBSENSE_LIVE_ERR_INVAL      -1
#define VIBSENSE_LIVE_ERR_EMPTY      -2
#define VIBSENSE_LIVE_QUALITY_LOW_RPM       (1u << 0)
#define VIBSENSE_LIVE_QUALITY_SHORT_WINDOW  (1u << 1)
#define VIBSENSE_LIVE_QUALITY_BAD_SCALE     (1u << 2)
#define VIBSENSE_LIVE_QUALITY_CLAMPED_DOT   (1u << 3)

/*
 * Compact display product derived from a completed vibsense_window_t.
 * Values are intended for live trend/display use, not final balancing analysis.
 */
typedef struct
{
    uint8_t  valid;
    uint8_t  accel_source;
    uint16_t sample_bits;

    uint32_t sample_count;
    uint32_t revolutions;

    double rpm;

    double order1_x_g;          /* 1/rev acceleration peak amplitude, X axis, g */
    double order1_y_g;          /* 1/rev acceleration peak amplitude, Y axis, g */
    double order1_xy_g;         /* sqrt(X^2 + Y^2), g */

    double ips_x;               /* approximate velocity peak, inches/sec */
    double ips_y;
    double ips_xy;

    double phase_deg;           /* 0..360, relative to optical index/TDC */
    double rms_x_g;
    double rms_y_g;
    double rms_xy_g;

    double dot_x_norm;          /* -1..+1 for EVE polar display */
    double dot_y_norm;          /* -1..+1 for EVE polar display */
    double display_full_scale_ips;

    uint32_t quality_flags;
} vibsense_live_metrics_t;

typedef struct
{
    double display_full_scale_ips;     /* e.g. 0.50 IPS at outer ring */
    double min_reasonable_rpm;         /* e.g. 500 */
    uint32_t min_revolutions;          /* e.g. 2 */
} vibsense_live_config_t;

void vibsense_live_default_config(vibsense_live_config_t *cfg);
int  vibsense_live_analyze_window(const vibsense_window_t *window,
                                  const vibsense_live_config_t *cfg,
                                  vibsense_live_metrics_t *out);

#ifdef __cplusplus
}
#endif

#endif /* VIBSENSE_LIVE_H */
