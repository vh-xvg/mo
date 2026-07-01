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

#include "vibsense_fake_window.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define TWO_PI (2.0 * M_PI)
#define G_IN_PER_S2 386.08858267716535

typedef struct
{
    const char *name;
    double ips;
    double phase_deg;
    double rpm;
    double noise_g;
    double second_order_fraction;
    double y_fraction;
    uint32_t revolutions;
    uint32_t samples_per_rev;
} fake_case_t;

static const fake_case_t fake_cases[VIBSENSE_FAKE_SCENARIO_COUNT] = {
    { "smooth 0.10 ips 000 deg",     0.100,   0.0, 2350.0, 0.0025, 0.02, 0.85, 3, 96 },
    { "smooth 0.11 ips 006 deg",     0.110,   6.0, 2380.0, 0.0030, 0.03, 0.90, 3, 96 },
    { "smooth 0.12 ips 011 deg",     0.120,  11.0, 2410.0, 0.0035, 0.04, 0.95, 3, 96 },
    { "slight noise 0.13 ips 017",   0.130,  17.0, 2440.0, 0.0060, 0.05, 1.00, 3, 96 },
    { "moderate 0.14 ips 023 deg",   0.140,  23.0, 2475.0, 0.0040, 0.08, 0.80, 4, 80 },
    { "moderate 0.15 ips 030 deg",   0.150,  30.0, 2500.0, 0.0045, 0.10, 0.75, 4, 80 },
    { "moderate 0.16 ips 036 deg",   0.160,  36.0, 2530.0, 0.0050, 0.12, 0.90, 4, 80 },
    { "higher 0.17 ips 042 deg",     0.170,  42.0, 2550.0, 0.0065, 0.15, 1.05, 4, 80 },
    { "higher 0.18 ips 049 deg",     0.180,  49.0, 2450.0, 0.0070, 0.18, 1.10, 5, 64 },
    { "higher 0.19 ips 055 deg",     0.190,  55.0, 2400.0, 0.0080, 0.20, 0.95, 5, 64 },
    { "noisy 0.20 ips 061 deg",      0.200,  61.0, 2300.0, 0.0140, 0.25, 1.00, 5, 64 },
    { "low-rpm short caution",       0.120,  67.0,  420.0, 0.0060, 0.10, 1.00, 1, 128 }
};

static vibsense_window_sample_t fake_samples[VIBSENSE_FAKE_NUM_SLOTS][VIBSENSE_FAKE_MAX_SAMPLES];
static uint32_t fake_call_index = 0;
static uint32_t fake_lfsr = 0x13579bdfu;

static uint32_t fake_rand_u32(void)
{
    uint32_t x = fake_lfsr;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    fake_lfsr = x ? x : 0x2468ace1u;
    return fake_lfsr;
}

static double fake_rand_unit(void)
{
    uint32_t r = fake_rand_u32();
    return ((double)(r & 0x00ffffffu) / 8388607.5) - 1.0;
}

static void source_defaults(uint8_t source,
                            uint8_t *range_g,
                            uint16_t *raw_bits)
{
    switch (source)
    {
    case VIB_ACCEL_SOURCE_ADXL355:
        *range_g = 8u;
        *raw_bits = 20u;
        break;

    case VIB_ACCEL_SOURCE_IIS3DWBG1:
    default:
        *range_g = 16u;
        *raw_bits = 16u;
        break;
    }
}

static int32_t g_to_counts(double g, uint8_t range_g, uint16_t raw_bits)
{
    if (range_g == 0u || raw_bits < 8u || raw_bits > 30u)
        return 0;

    double full = (double)((1u << (raw_bits - 1u)) - 1u);
    double counts = g * full / (double)range_g;

    if (counts > full)
        counts = full;
    if (counts < -full)
        counts = -full;

    return (int32_t)((counts >= 0.0) ? (counts + 0.5) : (counts - 0.5));
}

void vibsense_fake_reset(void)
{
    fake_call_index = 0;
    fake_lfsr = 0x13579bdfu;
    memset(fake_samples, 0, sizeof(fake_samples));
}

void vibsense_fake_default_config(vibsense_fake_config_t *cfg)
{
    if (!cfg)
        return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->scenario_index = UINT32_MAX;
    cfg->tick_rate_hz = 1000000u;
    cfg->accel_source = VIB_ACCEL_SOURCE_IIS3DWBG1;
    cfg->ips_scale = 1.0;
    cfg->noise_scale = 1.0;
}

const char *vibsense_fake_scenario_name(uint32_t scenario_index)
{
    return fake_cases[scenario_index % VIBSENSE_FAKE_SCENARIO_COUNT].name;
}

const char *vibsense_fake_source_name(uint8_t accel_source)
{
    switch (accel_source)
    {
    case VIB_ACCEL_SOURCE_IIS3DWBG1:
        return "IIS";
    case VIB_ACCEL_SOURCE_ADXL355:
        return "ADXL";
    case VIB_ACCEL_SOURCE_BOTH:
        return "BOTH";
    default:
        return "--";
    }
}

int vibsense_fake_fill_window(vibsense_window_t *window)
{
    vibsense_fake_config_t cfg;
    vibsense_fake_default_config(&cfg);
    return vibsense_fake_fill_window_slot(window, VIBSENSE_FAKE_SLOT_PRIMARY, &cfg);
}

int vibsense_fake_fill_window_ex(vibsense_window_t *window,
                                 const vibsense_fake_config_t *cfg)
{
    return vibsense_fake_fill_window_slot(window, VIBSENSE_FAKE_SLOT_PRIMARY, cfg);
}

int vibsense_fake_fill_window_source(vibsense_window_t *window,
                                     uint8_t accel_source)
{
    vibsense_fake_config_t cfg;
    vibsense_fake_default_config(&cfg);
    cfg.accel_source = accel_source;
    return vibsense_fake_fill_window_slot(window, VIBSENSE_FAKE_SLOT_PRIMARY, &cfg);
}

int vibsense_fake_fill_both_windows(vibsense_window_t windows[2])
{
    vibsense_fake_config_t cfg;
    vibsense_fake_default_config(&cfg);
    return vibsense_fake_fill_both_windows_ex(windows, &cfg);
}

int vibsense_fake_fill_both_windows_ex(vibsense_window_t windows[2],
                                       const vibsense_fake_config_t *base_cfg)
{
    if (!windows)
        return -1;

    vibsense_fake_config_t cfg;
    if (base_cfg)
        cfg = *base_cfg;
    else
        vibsense_fake_default_config(&cfg);

    uint32_t scenario = cfg.scenario_index;
    if (scenario == UINT32_MAX)
        scenario = fake_call_index++;
    scenario %= VIBSENSE_FAKE_SCENARIO_COUNT;

    /*
     * IIS first, ADXL second. Keep the same scenario for both so the vectors
     * should broadly agree, but add small per-sensor differences to exercise
     * the dual-vector display path.
     */
    vibsense_fake_config_t iis = cfg;
    iis.scenario_index = scenario;
    iis.accel_source = VIB_ACCEL_SOURCE_IIS3DWBG1;
    iis.accel_range_g = 16u;
    iis.raw_sample_bits = 16u;
    if (iis.ips_scale == 0.0)
        iis.ips_scale = 1.0;
    if (iis.noise_scale == 0.0)
        iis.noise_scale = 1.0;

    vibsense_fake_config_t adxl = cfg;
    adxl.scenario_index = scenario;
    adxl.accel_source = VIB_ACCEL_SOURCE_ADXL355;
    adxl.accel_range_g = 8u;
    adxl.raw_sample_bits = 20u;
    /*
     * Deliberately make the fake ADXL vector visibly different from the IIS
     * vector when BOTH sensors are displayed.  This is for display testing only:
     *   - 2x radial magnitude
     *   - 30 degrees anti-clockwise from the IIS vector
     *
     * The sign convention here follows the live polar display convention used
     * by the analyzer.  If the visual direction appears reversed on your page,
     * change -30.0 to +30.0.
     */
    adxl.ips_scale = (cfg.ips_scale == 0.0 ? 1.0 : cfg.ips_scale) * 0.5;
    adxl.phase_offset_deg = cfg.phase_offset_deg - 60.0;
    adxl.noise_scale = (cfg.noise_scale == 0.0 ? 1.0 : cfg.noise_scale) * 0.75;

    int rc = vibsense_fake_fill_window_slot(&windows[0], VIBSENSE_FAKE_SLOT_PRIMARY, &iis);
    if (rc != 0)
        return rc;

    rc = vibsense_fake_fill_window_slot(&windows[1], VIBSENSE_FAKE_SLOT_SECONDARY, &adxl);
    return rc;
}

int vibsense_fake_fill_window_slot(vibsense_window_t *window,
                                   uint32_t slot,
                                   const vibsense_fake_config_t *cfg_in)
{
    if (!window || slot >= VIBSENSE_FAKE_NUM_SLOTS)
        return -1;

    vibsense_fake_config_t cfg;
    if (cfg_in)
        cfg = *cfg_in;
    else
        vibsense_fake_default_config(&cfg);

    uint32_t scenario = cfg.scenario_index;
    if (scenario == UINT32_MAX)
        scenario = fake_call_index++;
    scenario %= VIBSENSE_FAKE_SCENARIO_COUNT;

    const fake_case_t *fc = &fake_cases[scenario];

    uint8_t accel_source = cfg.accel_source ? cfg.accel_source : VIB_ACCEL_SOURCE_IIS3DWBG1;
    if (accel_source == VIB_ACCEL_SOURCE_BOTH)
        accel_source = VIB_ACCEL_SOURCE_IIS3DWBG1;

    uint8_t default_range_g = 0u;
    uint16_t default_bits = 0u;
    source_defaults(accel_source, &default_range_g, &default_bits);

    uint32_t revs = cfg.revolutions ? cfg.revolutions : fc->revolutions;
    uint32_t samples_per_rev = cfg.samples_per_rev ? cfg.samples_per_rev : fc->samples_per_rev;
    uint32_t tick_rate_hz = cfg.tick_rate_hz ? cfg.tick_rate_hz : 1000000u;
    uint8_t accel_range_g = cfg.accel_range_g ? cfg.accel_range_g : default_range_g;
    uint16_t raw_bits = cfg.raw_sample_bits ? cfg.raw_sample_bits : default_bits;
    double ips_scale = (cfg.ips_scale == 0.0) ? 1.0 : cfg.ips_scale;
    double noise_scale = (cfg.noise_scale == 0.0) ? 1.0 : cfg.noise_scale;

    if (revs == 0u || samples_per_rev == 0u)
        return -2;

    uint32_t sample_count = revs * samples_per_rev;
    if (sample_count > VIBSENSE_FAKE_MAX_SAMPLES)
        sample_count = VIBSENSE_FAKE_MAX_SAMPLES;

    double omega = TWO_PI * fc->rpm / 60.0;
    double accel_g_xy = (fc->ips * ips_scale) * omega / G_IN_PER_S2;
    double phase = (fc->phase_deg + cfg.phase_offset_deg) * M_PI / 180.0;
    double ticks_per_rev_d = (60.0 * (double)tick_rate_hz) / fc->rpm;
    uint64_t start_tick = 1000000ull + (uint64_t)scenario * 100000ull;

    for (uint32_t i = 0; i < sample_count; i++)
    {
        uint32_t rev = i / samples_per_rev;
        uint32_t k = i % samples_per_rev;
        double theta = TWO_PI * (double)k / (double)samples_per_rev;
        double sample_progress = (double)i / (double)samples_per_rev;

        /*
         * Slowly wobble the displayed vector by a few degrees over repeated
         * calls.  This is intentionally small so the page looks alive without
         * looking broken.
         */
        double drift = (2.0 * M_PI / 180.0) * sin(0.35 * (double)fake_call_index +
                                                  0.11 * (double)i +
                                                  0.37 * (double)slot);
        double ph = phase + drift;

        double xg = accel_g_xy * cos(theta + ph);
        double yg = fc->y_fraction * accel_g_xy * sin(theta + ph);

        xg += fc->second_order_fraction * accel_g_xy * cos(2.0 * theta - 0.4 * ph);
        yg += 0.7 * fc->second_order_fraction * accel_g_xy * sin(2.0 * theta + 0.2 * ph);

        xg += noise_scale * fc->noise_g * fake_rand_unit();
        yg += noise_scale * fc->noise_g * fake_rand_unit();

        double zg = 0.15 * accel_g_xy * sin(theta - ph) +
                    0.6 * noise_scale * fc->noise_g * fake_rand_unit();

        fake_samples[slot][i].tick = start_tick + (uint64_t)(ticks_per_rev_d * sample_progress + 0.5);
        fake_samples[slot][i].phase_rad = theta;
        fake_samples[slot][i].revolution_offset = rev;
        fake_samples[slot][i].ax = g_to_counts(xg, accel_range_g, raw_bits);
        fake_samples[slot][i].ay = g_to_counts(yg, accel_range_g, raw_bits);
        fake_samples[slot][i].az = g_to_counts(zg, accel_range_g, raw_bits);
        fake_samples[slot][i].accel_source = accel_source;
        fake_samples[slot][i].accel_range_g = accel_range_g;
        fake_samples[slot][i].raw_sample_bits = raw_bits;
    }

    memset(window, 0, sizeof(*window));
    window->requested_revolutions = revs;
    window->completed_revolutions = revs;
    window->start_tick = start_tick;
    window->end_tick = start_tick + (uint64_t)(ticks_per_rev_d * (double)revs + 0.5);
    window->tick_rate_hz = tick_rate_hz;
    window->sample_count = sample_count;
    window->samples = fake_samples[slot];

    return 0;
}
