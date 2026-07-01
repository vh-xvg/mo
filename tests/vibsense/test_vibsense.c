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

#include "vibsense.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define TICK_HZ 1000000u
#define RPM 2400u
#define REV_TICKS (TICK_HZ * 60u / RPM)
#define SDS_PPR 3u
#define SAMPLE_RATE 4000u
#define SAMPLE_DT (TICK_HZ / SAMPLE_RATE)

static int feed_status(vibsense_context_t *ctx)
{
    vib_status_v1_t s;
    memset(&s, 0, sizeof(s));
    s.type = VIB_PAYLOAD_STATUS_V1;
    s.version = VIB_PAYLOAD_VERSION_1;
    s.length = sizeof(s);
    s.timer_tick_hz = TICK_HZ;
    s.sample_rate_hz = SAMPLE_RATE;
    return vibsense_ingest_payload(ctx, &s, sizeof(s));
}

static int feed_tach(vibsense_context_t *ctx, uint32_t tick, uint8_t src, uint32_t pulse, uint32_t rev)
{
    vib_tach_event_v1_t e;
    memset(&e, 0, sizeof(e));
    e.type = VIB_PAYLOAD_TACH_EVENT_V1;
    e.version = VIB_PAYLOAD_VERSION_1;
    e.length = sizeof(e);
    e.event_ticks = tick;
    e.source = src;
    e.event_flags = VIB_TACH_EVENT_FLAG_ACCEPTED_PULSE | VIB_TACH_EVENT_FLAG_PERIOD_VALID;
    e.pulse_index = pulse;
    e.revolution_index = rev;
    if (src == VIB_TACH_SOURCE_SDS)
    {
        e.pulses_per_rev = SDS_PPR;
        e.period_ticks = REV_TICKS / SDS_PPR;
        e.event_flags |= VIB_TACH_EVENT_FLAG_SDS_PULSE;
    }
    else
    {
        e.pulses_per_rev = 1;
        e.period_ticks = REV_TICKS;
        e.event_flags |= VIB_TACH_EVENT_FLAG_OPTICAL_INDEX;
    }
    return vibsense_ingest_payload(ctx, &e, sizeof(e));
}

static int feed_accel_block(vibsense_context_t *ctx, uint32_t first_tick, uint16_t samples, uint8_t source)
{
    size_t len = sizeof(vib_accel_block_v1_hdr_t) + (size_t)samples * sizeof(vib_accel_sample_raw_t);
    unsigned char *buf = (unsigned char *)calloc(1, len);
    if (!buf)
        return -99;

    vib_accel_block_v1_hdr_t *h = (vib_accel_block_v1_hdr_t *)buf;
    vib_accel_sample_raw_t *s = (vib_accel_sample_raw_t *)(buf + sizeof(*h));
    h->type = VIB_PAYLOAD_ACCEL_BLOCK_V1;
    h->version = VIB_PAYLOAD_VERSION_1;
    h->length = (uint16_t)len;
    h->first_sample_ticks = first_tick;
    h->sample_dt_ticks = SAMPLE_DT;
    h->sample_count = samples;
    h->last_sample_ticks = first_tick + (samples - 1u) * SAMPLE_DT;
    h->accel_source = source;
    h->accel_range_g = 16;
    h->raw_sample_bits = source == VIB_ACCEL_SOURCE_ADXL355 ? 20 : 16;

    double scale = source == VIB_ACCEL_SOURCE_ADXL355 ? 2.0 : 1.0;
    double phase_offset = source == VIB_ACCEL_SOURCE_ADXL355 ? -M_PI / 6.0 : 0.0;

    for (uint16_t i = 0; i < samples; i++)
    {
        uint32_t t = first_tick + (uint32_t)i * SAMPLE_DT;
        double phase = 2.0 * M_PI * ((double)(t % REV_TICKS) / (double)REV_TICKS) + phase_offset;
        s[i].ax = (int32_t)lrint(scale * 10000.0 * cos(phase));
        s[i].ay = (int32_t)lrint(scale * 5000.0 * sin(phase));
        s[i].az = (int32_t)lrint(scale * 1000.0);
    }

    int rc = vibsense_ingest_payload(ctx, buf, len);
    free(buf);
    return rc;
}

int main(void)
{
    vibsense_context_t *ctx = vibsense_create();
    if (!ctx)
        return 1;

    const uint32_t target_revs = 10;
    if (vibsense_arm_sources(ctx, target_revs, VIBSENSE_ACCEL_MASK_BOTH) != VIBSENSE_OK)
        return 2;
    if (feed_status(ctx) != VIBSENSE_OK)
        return 3;

    int rc = VIBSENSE_OK;
    uint32_t last_accel_tick = 0;
    int saw_closing_without_done = 0;

    for (uint32_t rev = 0; rev <= target_revs + 1u && rc != VIBSENSE_DONE; rev++)
    {
        uint32_t rev_tick = rev * REV_TICKS;
        rc = feed_tach(ctx, rev_tick, VIB_TACH_SOURCE_OPTICAL, rev, rev);
        if (rc < 0) return 4;

        if (rev == target_revs && rc != VIBSENSE_DONE)
            saw_closing_without_done = 1; /* end index arrived, but accel queues not drained yet */

        for (uint32_t p = 0; p < SDS_PPR; p++)
        {
            uint32_t tick = rev_tick + p * (REV_TICKS / SDS_PPR);
            int trc = feed_tach(ctx, tick, VIB_TACH_SOURCE_SDS, rev * SDS_PPR + p, rev);
            if (trc < 0) return 5;
        }

        while (last_accel_tick < rev_tick + REV_TICKS && rc != VIBSENSE_DONE)
        {
            rc = feed_accel_block(ctx, last_accel_tick, 32, VIB_ACCEL_SOURCE_IIS3DWBG1);
            if (rc < 0) return 6;
            if (rc == VIBSENSE_DONE) break;

            rc = feed_accel_block(ctx, last_accel_tick, 32, VIB_ACCEL_SOURCE_ADXL355);
            if (rc < 0) return 7;
            last_accel_tick += 32u * SAMPLE_DT;
        }
    }

    if (rc != VIBSENSE_DONE)
    {
        fprintf(stderr, "did not complete: rc=%d\n", rc);
        return 8;
    }
    if (!saw_closing_without_done)
    {
        fprintf(stderr, "closing state was not exercised\n");
        return 9;
    }

    vibsense_window_t all, iis, adxl;
    if (vibsense_get_window(ctx, &all) != VIBSENSE_OK)
        return 10;
    if (vibsense_get_window_by_source(ctx, VIB_ACCEL_SOURCE_IIS3DWBG1, &iis) != VIBSENSE_OK)
        return 11;
    if (vibsense_get_window_by_source(ctx, VIB_ACCEL_SOURCE_ADXL355, &adxl) != VIBSENSE_OK)
        return 12;

    if (iis.sample_count == 0u || adxl.sample_count == 0u)
        return 13;
    if (all.sample_count != iis.sample_count + adxl.sample_count)
        return 14;

    vibsense_order_snapshot_t snap_iis, snap_adxl;
    if (vibsense_compute_order_snapshot_by_source(ctx, VIB_ACCEL_SOURCE_IIS3DWBG1, &snap_iis) != VIBSENSE_OK)
        return 15;
    if (vibsense_compute_order_snapshot_by_source(ctx, VIB_ACCEL_SOURCE_ADXL355, &snap_adxl) != VIBSENSE_OK)
        return 16;

    printf("window all=%u iis=%u adxl=%u start=%llu end=%llu rpm=%.3f\n",
           all.sample_count, iis.sample_count, adxl.sample_count,
           (unsigned long long)all.start_tick,
           (unsigned long long)all.end_tick,
           snap_iis.rpm_mean);
    printf("IIS  ax_amp=%.2f phase=%.3f\n", snap_iis.order1_amp_counts[0], snap_iis.order1_phase_rad[0]);
    printf("ADXL ax_amp=%.2f phase=%.3f\n", snap_adxl.order1_amp_counts[0], snap_adxl.order1_phase_rad[0]);

    if (adxl.sample_count != iis.sample_count)
        return 17;
    if (snap_adxl.order1_amp_counts[0] < 1.5 * snap_iis.order1_amp_counts[0])
        return 18;

    vibsense_csv_metadata_t meta = {0};
    meta.aircraft = "VH-XVG";
    meta.engine = "Lycoming IO-540";
    meta.sample_name = "synthetic_dual_test";
    meta.utc_time = "2026-05-20T10:15:00Z";
    meta.axis_orientation = "x=vertical,y=lateral,z=fore-aft";
    meta.optical_reference = "0_deg_top_dead_center_viewed_from_spinner_side";
    meta.rotation_direction = "CW_viewed_from_spinner_side";

    if (vibsense_write_window_csv_with_metadata(ctx, "test_window_all.csv", &meta) != VIBSENSE_OK)
        return 19;
    meta.sample_name = "synthetic_iis_test";
    if (vibsense_write_window_csv_by_source_with_metadata(ctx, VIB_ACCEL_SOURCE_IIS3DWBG1, "test_window_iis.csv", &meta) != VIBSENSE_OK)
        return 20;
    meta.sample_name = "synthetic_adxl_test";
    if (vibsense_write_window_csv_by_source_with_metadata(ctx, VIB_ACCEL_SOURCE_ADXL355, "test_window_adxl.csv", &meta) != VIBSENSE_OK)
        return 21;

    vibsense_destroy(ctx);
    return 0;
}
