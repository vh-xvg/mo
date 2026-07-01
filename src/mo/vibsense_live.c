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

#include "vibsense_live.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define TWO_PI      (2.0 * M_PI)
#define G_IN_PER_S2 386.08858267716535

static double sample_counts_to_g(const vibsense_window_sample_t *s, uint32_t *quality)
{
    if (!s || s->accel_range_g == 0u || s->raw_sample_bits < 8u || s->raw_sample_bits > 30u)
    {
        if (quality)
            *quality |= VIBSENSE_LIVE_QUALITY_BAD_SCALE;
        return 1.0;
    }

    /*
     * Display approximation:
     * raw samples are sign-extended. Treat +/- full scale as +/- range_g.
     * This is adequate for the live page; detailed calibration belongs offline.
     */
    double denom = (double)((1u << (s->raw_sample_bits - 1u)) - 1u);
    return (double)s->accel_range_g / denom;
}

void vibsense_live_default_config(vibsense_live_config_t *cfg)
{
    if (!cfg)
        return;
    cfg->display_full_scale_ips = 0.50;
    cfg->min_reasonable_rpm = 500.0;
    cfg->min_revolutions = 2u;
}

int vibsense_live_analyze_window(const vibsense_window_t *window,
                                 const vibsense_live_config_t *cfg_in,
                                 vibsense_live_metrics_t *out)
{
    if (!window || !out)
        return VIBSENSE_LIVE_ERR_INVAL;
    if (!window->samples || window->sample_count == 0u || window->completed_revolutions == 0u)
        return VIBSENSE_LIVE_ERR_EMPTY;

    vibsense_live_config_t cfg;
    if (cfg_in)
        cfg = *cfg_in;
    else
        vibsense_live_default_config(&cfg);

    if (cfg.display_full_scale_ips <= 0.0)
        cfg.display_full_scale_ips = 0.50;

    memset(out, 0, sizeof(*out));
    out->valid = 1u;
    out->sample_count = window->sample_count;
    out->revolutions = window->completed_revolutions;
    out->display_full_scale_ips = cfg.display_full_scale_ips;

    if (window->end_tick > window->start_tick && window->tick_rate_hz != 0u)
    {
        double seconds = (double)(window->end_tick - window->start_tick) / (double)window->tick_rate_hz;
        if (seconds > 0.0)
            out->rpm = 60.0 * (double)window->completed_revolutions / seconds;
    }

    if (out->rpm < cfg.min_reasonable_rpm)
        out->quality_flags |= VIBSENSE_LIVE_QUALITY_LOW_RPM;
    if (window->completed_revolutions < cfg.min_revolutions)
        out->quality_flags |= VIBSENSE_LIVE_QUALITY_SHORT_WINDOW;

    double re_x = 0.0, im_x = 0.0;
    double re_y = 0.0, im_y = 0.0;
    double sumsq_x = 0.0, sumsq_y = 0.0;

    for (uint32_t i = 0; i < window->sample_count; i++)
    {
        const vibsense_window_sample_t *s = &window->samples[i];
        uint32_t q = 0u;
        double scale = sample_counts_to_g(s, &q);
        out->quality_flags |= q;

        double x = (double)s->ax * scale;
        double y = (double)s->ay * scale;
        double c = cos(s->phase_rad);
        double sn = sin(s->phase_rad);

        re_x += x * c;
        im_x -= x * sn;
        re_y += y * c;
        im_y -= y * sn;

        sumsq_x += x * x;
        sumsq_y += y * y;

        out->accel_source = s->accel_source;
        out->sample_bits = s->raw_sample_bits;
    }

    double n = (double)window->sample_count;

    double Xr = (2.0 / n) * re_x;
    double Xi = (2.0 / n) * im_x;
    double Yr = (2.0 / n) * re_y;
    double Yi = (2.0 / n) * im_y;

    out->order1_x_g = hypot(Xr, Xi);
    out->order1_y_g = hypot(Yr, Yi);
    out->order1_xy_g = hypot(out->order1_x_g, out->order1_y_g);

    out->rms_x_g = sqrt(sumsq_x / n);
    out->rms_y_g = sqrt(sumsq_y / n);
    out->rms_xy_g = hypot(out->rms_x_g, out->rms_y_g);

    if (out->rpm > 0.0)
    {
        double omega = TWO_PI * out->rpm / 60.0;
        out->ips_x = out->order1_x_g * G_IN_PER_S2 / omega;
        out->ips_y = out->order1_y_g * G_IN_PER_S2 / omega;
        out->ips_xy = out->order1_xy_g * G_IN_PER_S2 / omega;
    }

    /*
     * Combined phase for the central polar display.  Treat X and Y order
     * coefficients as a complex spatial vector; this is deliberately light
     * weight and intended for trend/awareness, not final balance solving.
     */
    double Cr = Xr - Yi;
    double Ci = Xi + Yr;
    double phase = atan2(Ci, Cr);
    if (phase < 0.0)
        phase += TWO_PI;
    out->phase_deg = phase * 180.0 / M_PI;

    double r = out->ips_xy / cfg.display_full_scale_ips;
    if (r > 1.0)
    {
        r = 1.0;
        out->quality_flags |= VIBSENSE_LIVE_QUALITY_CLAMPED_DOT;
    }
    if (r < 0.0)
        r = 0.0;

    out->dot_x_norm = r * cos(phase);
    out->dot_y_norm = -r * sin(phase);  /* screen Y is down */

    return VIBSENSE_LIVE_OK;
}
