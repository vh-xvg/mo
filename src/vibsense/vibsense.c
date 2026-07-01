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

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define TWO_PI (2.0 * M_PI)
#define INITIAL_TACH_CAP 256u
#define INITIAL_ACCEL_CAP 8192u
#define INITIAL_WINDOW_CAP 8192u
#define DEFAULT_TICK_RATE_HZ 1000000u

typedef struct
{
    uint64_t tick;
    uint32_t period_ticks;
    uint32_t pulse_index;
    uint32_t revolution_index;
    uint16_t flags;
    uint8_t source;
    uint8_t pulses_per_rev;
} tach_rec_t;

typedef struct
{
    uint64_t tick;
    int32_t ax;
    int32_t ay;
    int32_t az;
    uint8_t accel_source;
    uint8_t accel_range_g;
    uint16_t raw_sample_bits;
} accel_rec_t;

typedef struct
{
    vibsense_window_sample_t *samples;
    size_t len;
    size_t cap;
} source_window_t;

struct vibsense_context
{
    bool armed;
    bool closing;
    bool done;
    uint32_t target_revs;
    uint32_t tick_rate_hz;
    uint32_t active_source_mask;       /* 0 means ANY/backward-compatible */
    uint32_t observed_source_mask;
    uint32_t drained_source_mask;      /* sources with at least one sample tick > end_tick */

    bool have_start;
    bool have_end;
    uint64_t start_tick;
    uint64_t end_tick;
    uint32_t optical_revs_seen_after_start;

    uint32_t last_tick32;
    uint64_t tick_hi;
    bool have_tick_unwrap;

    tach_rec_t *tach;
    size_t tach_len;
    size_t tach_cap;

    accel_rec_t *accel;
    size_t accel_len;
    size_t accel_cap;

    vibsense_window_sample_t *window;
    size_t window_len;
    size_t window_cap;

    source_window_t adxl_window;
    source_window_t iis_window;
};

static uint32_t source_to_mask(uint8_t source)
{
    switch (source)
    {
    case VIB_ACCEL_SOURCE_ADXL355:
        return VIBSENSE_ACCEL_MASK_ADXL355;
    case VIB_ACCEL_SOURCE_IIS3DWBG1:
        return VIBSENSE_ACCEL_MASK_IIS3DWBG1;
    case VIB_ACCEL_SOURCE_BOTH:
        return VIBSENSE_ACCEL_MASK_BOTH;
    default:
        return 0u;
    }
}

static source_window_t *source_window_for(vibsense_context_t *ctx, uint8_t source)
{
    switch (source)
    {
    case VIB_ACCEL_SOURCE_ADXL355:
        return &ctx->adxl_window;
    case VIB_ACCEL_SOURCE_IIS3DWBG1:
        return &ctx->iis_window;
    default:
        return NULL;
    }
}

static const source_window_t *const_source_window_for(const vibsense_context_t *ctx, uint8_t source)
{
    switch (source)
    {
    case VIB_ACCEL_SOURCE_ADXL355:
        return &ctx->adxl_window;
    case VIB_ACCEL_SOURCE_IIS3DWBG1:
        return &ctx->iis_window;
    default:
        return NULL;
    }
}

static uint64_t unwrap_tick(vibsense_context_t *ctx, uint32_t tick32)
{
    if (!ctx->have_tick_unwrap)
    {
        ctx->have_tick_unwrap = true;
        ctx->last_tick32 = tick32;
        return tick32;
    }

    if (tick32 < ctx->last_tick32 && (ctx->last_tick32 - tick32) > 0x80000000u)
        ctx->tick_hi += (1ull << 32);

    ctx->last_tick32 = tick32;
    return ctx->tick_hi | (uint64_t)tick32;
}

static int reserve_tach(vibsense_context_t *ctx, size_t extra)
{
    if (ctx->tach_len + extra <= ctx->tach_cap)
        return VIBSENSE_OK;

    size_t new_cap = ctx->tach_cap ? ctx->tach_cap : INITIAL_TACH_CAP;
    while (new_cap < ctx->tach_len + extra)
        new_cap *= 2u;

    tach_rec_t *p = (tach_rec_t *)realloc(ctx->tach, new_cap * sizeof(*p));
    if (!p)
        return VIBSENSE_ERR_NOMEM;

    ctx->tach = p;
    ctx->tach_cap = new_cap;
    return VIBSENSE_OK;
}

static int reserve_accel(vibsense_context_t *ctx, size_t extra)
{
    if (ctx->accel_len + extra <= ctx->accel_cap)
        return VIBSENSE_OK;

    size_t new_cap = ctx->accel_cap ? ctx->accel_cap : INITIAL_ACCEL_CAP;
    while (new_cap < ctx->accel_len + extra)
        new_cap *= 2u;

    accel_rec_t *p = (accel_rec_t *)realloc(ctx->accel, new_cap * sizeof(*p));
    if (!p)
        return VIBSENSE_ERR_NOMEM;

    ctx->accel = p;
    ctx->accel_cap = new_cap;
    return VIBSENSE_OK;
}

static int reserve_window(vibsense_context_t *ctx, size_t extra)
{
    if (ctx->window_len + extra <= ctx->window_cap)
        return VIBSENSE_OK;

    size_t new_cap = ctx->window_cap ? ctx->window_cap : INITIAL_WINDOW_CAP;
    while (new_cap < ctx->window_len + extra)
        new_cap *= 2u;

    vibsense_window_sample_t *p = (vibsense_window_sample_t *)realloc(ctx->window, new_cap * sizeof(*p));
    if (!p)
        return VIBSENSE_ERR_NOMEM;

    ctx->window = p;
    ctx->window_cap = new_cap;
    return VIBSENSE_OK;
}

static int reserve_source_window(source_window_t *sw, size_t extra)
{
    if (sw->len + extra <= sw->cap)
        return VIBSENSE_OK;

    size_t new_cap = sw->cap ? sw->cap : INITIAL_WINDOW_CAP;
    while (new_cap < sw->len + extra)
        new_cap *= 2u;

    vibsense_window_sample_t *p = (vibsense_window_sample_t *)realloc(sw->samples, new_cap * sizeof(*p));
    if (!p)
        return VIBSENSE_ERR_NOMEM;

    sw->samples = p;
    sw->cap = new_cap;
    return VIBSENSE_OK;
}

static bool is_optical_tdc(const tach_rec_t *t)
{
    return t->source == VIB_TACH_SOURCE_OPTICAL;
}

static int find_sds_segment(const vibsense_context_t *ctx, uint64_t tick, size_t *idx_out)
{
    size_t best = (size_t)-1;

    for (size_t i = 0; i < ctx->tach_len; i++)
    {
        const tach_rec_t *t = &ctx->tach[i];
        if (t->source != VIB_TACH_SOURCE_SDS)
            continue;
        if (t->tick <= tick)
            best = i;
        else
            break;
    }

    if (best == (size_t)-1)
        return VIBSENSE_ERR_NOLOCK;

    *idx_out = best;
    return VIBSENSE_OK;
}

static int find_start_revolution_index(const vibsense_context_t *ctx, uint32_t *rev_idx)
{
    for (size_t i = 0; i < ctx->tach_len; i++)
    {
        const tach_rec_t *t = &ctx->tach[i];
        if (is_optical_tdc(t) && t->tick == ctx->start_tick)
        {
            *rev_idx = t->revolution_index;
            return VIBSENSE_OK;
        }
    }
    return VIBSENSE_ERR_NOLOCK;
}

static int phase_at_tick(const vibsense_context_t *ctx,
                         uint64_t tick,
                         uint32_t *rev_offset,
                         double *phase_rad)
{
    if (!ctx->have_start || tick <= ctx->start_tick || tick >= ctx->end_tick)
        return VIBSENSE_ERR_INVAL;

    size_t sds_idx = 0;
    if (find_sds_segment(ctx, tick, &sds_idx) == VIBSENSE_OK)
    {
        const tach_rec_t *s = &ctx->tach[sds_idx];
        uint8_t ppr = s->pulses_per_rev ? s->pulses_per_rev : 3u;
        uint32_t period = s->period_ticks;
        if (period == 0u)
            period = (uint32_t)((ctx->tick_rate_hz ? ctx->tick_rate_hz : DEFAULT_TICK_RATE_HZ) / 135u);

        double pulse_phase = fmod((double)(s->pulse_index % ppr) / (double)ppr, 1.0);
        double interp = (double)(tick - s->tick) / (double)period / (double)ppr;
        double cycles_abs = (double)s->revolution_index + pulse_phase + interp;

        uint32_t start_rev = 0u;
        if (find_start_revolution_index(ctx, &start_rev) != VIBSENSE_OK)
            return VIBSENSE_ERR_NOLOCK;

        double rel = cycles_abs - (double)start_rev;
        if (rel < 0.0)
            return VIBSENSE_ERR_NOLOCK;

        uint32_t ro = (uint32_t)floor(rel);
        double frac = rel - (double)ro;
        if (ro >= ctx->target_revs)
            return VIBSENSE_ERR_INVAL;

        *rev_offset = ro;
        *phase_rad = frac * TWO_PI;
        return VIBSENSE_OK;
    }

    /* Fallback: interpolate uniformly between optical boundaries. */
    uint64_t prev = ctx->start_tick;
    uint64_t next = ctx->end_tick;
    uint32_t ro = 0u;
    bool have_prev = false;

    for (size_t i = 0; i < ctx->tach_len; i++)
    {
        const tach_rec_t *t = &ctx->tach[i];
        if (!is_optical_tdc(t))
            continue;
        if (t->tick <= tick)
        {
            prev = t->tick;
            ro = t->revolution_index;
            have_prev = true;
        }
        else
        {
            next = t->tick;
            break;
        }
    }

    if (!have_prev || next <= prev)
        return VIBSENSE_ERR_NOLOCK;

    uint32_t start_rev = 0u;
    if (find_start_revolution_index(ctx, &start_rev) != VIBSENSE_OK)
        return VIBSENSE_ERR_NOLOCK;

    uint32_t off = ro - start_rev;
    if (off >= ctx->target_revs)
        return VIBSENSE_ERR_INVAL;

    double frac = (double)(tick - prev) / (double)(next - prev);
    if (frac < 0.0 || frac >= 1.0)
        return VIBSENSE_ERR_INVAL;

    *rev_offset = off;
    *phase_rad = frac * TWO_PI;
    return VIBSENSE_OK;
}

static int build_window(vibsense_context_t *ctx)
{
    ctx->window_len = 0;
    ctx->adxl_window.len = 0;
    ctx->iis_window.len = 0;

    if (!ctx->have_start || !ctx->have_end || ctx->end_tick <= ctx->start_tick)
        return VIBSENSE_ERR_STATE;

    for (size_t i = 0; i < ctx->accel_len; i++)
    {
        const accel_rec_t *a = &ctx->accel[i];
        if (a->tick <= ctx->start_tick || a->tick >= ctx->end_tick)
            continue;

        if (ctx->active_source_mask != VIBSENSE_ACCEL_MASK_ANY &&
            (source_to_mask(a->accel_source) & ctx->active_source_mask) == 0u)
            continue;

        uint32_t rev_offset = 0u;
        double phase = 0.0;
        if (phase_at_tick(ctx, a->tick, &rev_offset, &phase) != VIBSENSE_OK)
            continue;

        int rc = reserve_window(ctx, 1u);
        if (rc != VIBSENSE_OK)
            return rc;

        vibsense_window_sample_t ws;
        ws.tick = a->tick;
        ws.phase_rad = phase;
        ws.revolution_offset = rev_offset;
        ws.ax = a->ax;
        ws.ay = a->ay;
        ws.az = a->az;
        ws.accel_source = a->accel_source;
        ws.accel_range_g = a->accel_range_g;
        ws.raw_sample_bits = a->raw_sample_bits;

        ctx->window[ctx->window_len++] = ws;

        source_window_t *sw = source_window_for(ctx, a->accel_source);
        if (sw)
        {
            rc = reserve_source_window(sw, 1u);
            if (rc != VIBSENSE_OK)
                return rc;
            sw->samples[sw->len++] = ws;
        }
    }

    if (ctx->active_source_mask == VIBSENSE_ACCEL_MASK_ANY)
        return ctx->window_len ? VIBSENSE_OK : VIBSENSE_ERR_NOLOCK;

    if ((ctx->active_source_mask & VIBSENSE_ACCEL_MASK_ADXL355) && ctx->adxl_window.len == 0u)
        return VIBSENSE_ERR_NOLOCK;
    if ((ctx->active_source_mask & VIBSENSE_ACCEL_MASK_IIS3DWBG1) && ctx->iis_window.len == 0u)
        return VIBSENSE_ERR_NOLOCK;

    return VIBSENSE_OK;
}

static bool closing_drain_complete(const vibsense_context_t *ctx)
{
    if (!ctx->closing || !ctx->have_end)
        return false;

    if (ctx->active_source_mask == VIBSENSE_ACCEL_MASK_ANY)
        return ctx->drained_source_mask != 0u;

    return (ctx->drained_source_mask & ctx->active_source_mask) == ctx->active_source_mask;
}

static int try_finish(vibsense_context_t *ctx)
{
    if (ctx->done)
        return VIBSENSE_DONE;

    if (!closing_drain_complete(ctx))
        return VIBSENSE_OK;

    int rc = build_window(ctx);
    if (rc != VIBSENSE_OK)
        return rc;

    ctx->done = true;
    ctx->closing = false;
    return VIBSENSE_DONE;
}

static int ingest_tach(vibsense_context_t *ctx, const vib_tach_event_v1_t *e)
{
    if (e->length != sizeof(*e) || e->version != VIB_PAYLOAD_VERSION_1)
        return VIBSENSE_ERR_PROTOCOL;

    int rc = reserve_tach(ctx, 1u);
    if (rc != VIBSENSE_OK)
        return rc;

    uint64_t tick = unwrap_tick(ctx, e->event_ticks);
    tach_rec_t *t = &ctx->tach[ctx->tach_len++];
    t->tick = tick;
    t->period_ticks = e->period_ticks;
    t->pulse_index = e->pulse_index;
    t->revolution_index = e->revolution_index;
    t->flags = e->event_flags;
    t->source = e->source;
    t->pulses_per_rev = e->pulses_per_rev;

    if (!ctx->armed || ctx->done || !is_optical_tdc(t))
        return VIBSENSE_OK;

    if (!ctx->have_start)
    {
        ctx->have_start = true;
        ctx->start_tick = tick;
        ctx->optical_revs_seen_after_start = 0u;
        return VIBSENSE_OK;
    }

    if (!ctx->have_end)
    {
        ctx->optical_revs_seen_after_start++;
        if (ctx->optical_revs_seen_after_start >= ctx->target_revs)
        {
            ctx->have_end = true;
            ctx->end_tick = tick;
            ctx->closing = true;
            return try_finish(ctx);
        }
    }

    return VIBSENSE_OK;
}

static int ingest_accel(vibsense_context_t *ctx, const uint8_t *payload, size_t payload_len)
{
    if (payload_len < sizeof(vib_accel_block_v1_hdr_t))
        return VIBSENSE_ERR_PROTOCOL;

    const vib_accel_block_v1_hdr_t *h = (const vib_accel_block_v1_hdr_t *)payload;
    if (h->length != payload_len || h->version != VIB_PAYLOAD_VERSION_1)
        return VIBSENSE_ERR_PROTOCOL;

    size_t expected = sizeof(*h) + (size_t)h->sample_count * sizeof(vib_accel_sample_raw_t);
    if (expected != payload_len)
        return VIBSENSE_ERR_PROTOCOL;

    if (!ctx->armed || ctx->done)
        return VIBSENSE_OK;

    uint32_t src_mask = source_to_mask(h->accel_source);
    if (src_mask == 0u)
        return VIBSENSE_ERR_PROTOCOL;

    ctx->observed_source_mask |= src_mask;

    int rc = reserve_accel(ctx, h->sample_count);
    if (rc != VIBSENSE_OK)
        return rc;

    const vib_accel_sample_raw_t *s = (const vib_accel_sample_raw_t *)(payload + sizeof(*h));
    uint64_t first = unwrap_tick(ctx, h->first_sample_ticks);
    uint64_t last = first;

    for (uint16_t i = 0; i < h->sample_count; i++)
    {
        accel_rec_t *a = &ctx->accel[ctx->accel_len++];
        a->tick = first + (uint64_t)i * (uint64_t)h->sample_dt_ticks;
        last = a->tick;
        a->ax = s[i].ax;
        a->ay = s[i].ay;
        a->az = s[i].az;
        a->accel_source = h->accel_source;
        a->accel_range_g = h->accel_range_g;
        a->raw_sample_bits = h->raw_sample_bits;
    }

    if (ctx->closing && ctx->have_end && last > ctx->end_tick)
    {
        if (ctx->active_source_mask == VIBSENSE_ACCEL_MASK_ANY ||
            (src_mask & ctx->active_source_mask) != 0u)
            ctx->drained_source_mask |= src_mask;
    }

    return try_finish(ctx);
}

vibsense_context_t *vibsense_create(void)
{
    vibsense_context_t *ctx = (vibsense_context_t *)calloc(1u, sizeof(*ctx));
    if (ctx)
        ctx->tick_rate_hz = DEFAULT_TICK_RATE_HZ;
    return ctx;
}

void vibsense_destroy(vibsense_context_t *ctx)
{
    if (!ctx)
        return;
    free(ctx->tach);
    free(ctx->accel);
    free(ctx->window);
    free(ctx->adxl_window.samples);
    free(ctx->iis_window.samples);
    free(ctx);
}

int vibsense_reset(vibsense_context_t *ctx)
{
    if (!ctx)
        return VIBSENSE_ERR_INVAL;

    free(ctx->tach);
    free(ctx->accel);
    free(ctx->window);
    free(ctx->adxl_window.samples);
    free(ctx->iis_window.samples);
    memset(ctx, 0, sizeof(*ctx));
    ctx->tick_rate_hz = DEFAULT_TICK_RATE_HZ;
    return VIBSENSE_OK;
}

int vibsense_arm_sources(vibsense_context_t *ctx,
                         uint32_t target_revolutions,
                         uint32_t accel_source_mask)
{
    if (!ctx || target_revolutions == 0u || target_revolutions > 100000u)
        return VIBSENSE_ERR_INVAL;

    if (accel_source_mask & ~VIBSENSE_ACCEL_MASK_BOTH)
        return VIBSENSE_ERR_INVAL;

    int rc = vibsense_reset(ctx);
    if (rc != VIBSENSE_OK)
        return rc;

    ctx->armed = true;
    ctx->target_revs = target_revolutions;
    ctx->active_source_mask = accel_source_mask;
    return VIBSENSE_OK;
}

int vibsense_arm_accel_source(vibsense_context_t *ctx,
                              uint32_t target_revolutions,
                              uint8_t accel_source)
{
    return vibsense_arm_sources(ctx, target_revolutions, source_to_mask(accel_source));
}

int vibsense_arm(vibsense_context_t *ctx, uint32_t target_revolutions)
{
    return vibsense_arm_sources(ctx, target_revolutions, VIBSENSE_ACCEL_MASK_ANY);
}

int vibsense_ingest_payload(vibsense_context_t *ctx, const void *payload, size_t payload_len)
{
    if (!ctx || !payload || payload_len < sizeof(vib_payload_header_t))
        return VIBSENSE_ERR_INVAL;

    const vib_payload_header_t *ph = (const vib_payload_header_t *)payload;
    if (ph->length != payload_len)
        return VIBSENSE_ERR_PROTOCOL;

    switch (ph->type)
    {
    case VIB_PAYLOAD_TACH_EVENT_V1:
        if (payload_len != sizeof(vib_tach_event_v1_t))
            return VIBSENSE_ERR_PROTOCOL;
        return ingest_tach(ctx, (const vib_tach_event_v1_t *)payload);

    case VIB_PAYLOAD_ACCEL_BLOCK_V1:
        return ingest_accel(ctx, (const uint8_t *)payload, payload_len);

    case VIB_PAYLOAD_STATUS_V1:
        if (payload_len == sizeof(vib_status_v1_t))
        {
            const vib_status_v1_t *s = (const vib_status_v1_t *)payload;
            if (s->timer_tick_hz)
                ctx->tick_rate_hz = s->timer_tick_hz;
            return VIBSENSE_OK;
        }
        return VIBSENSE_ERR_PROTOCOL;

    case VIB_PAYLOAD_CONFIG_SNAPSHOT_V1:
        if (payload_len == sizeof(vib_config_snapshot_v1_t))
        {
            const vib_config_snapshot_v1_t *c = (const vib_config_snapshot_v1_t *)payload;
            if (c->tick_rate_hz)
                ctx->tick_rate_hz = c->tick_rate_hz;
            return VIBSENSE_OK;
        }
        return VIBSENSE_ERR_PROTOCOL;

    default:
        return VIBSENSE_ERR_PROTOCOL;
    }
}

static int fill_window_out(const vibsense_context_t *ctx,
                           const vibsense_window_sample_t *samples,
                           size_t len,
                           vibsense_window_t *out)
{
    if (!ctx || !out)
        return VIBSENSE_ERR_INVAL;
    if (!ctx->done)
        return VIBSENSE_ERR_STATE;

    out->requested_revolutions = ctx->target_revs;
    out->completed_revolutions = ctx->target_revs;
    out->start_tick = ctx->start_tick;
    out->end_tick = ctx->end_tick;
    out->tick_rate_hz = ctx->tick_rate_hz;
    out->sample_count = (uint32_t)len;
    out->samples = samples;
    return VIBSENSE_OK;
}

int vibsense_get_window(const vibsense_context_t *ctx, vibsense_window_t *out)
{
    return fill_window_out(ctx, ctx ? ctx->window : NULL, ctx ? ctx->window_len : 0u, out);
}

int vibsense_get_window_by_source(const vibsense_context_t *ctx,
                                  uint8_t accel_source,
                                  vibsense_window_t *out)
{
    if (!ctx || !out)
        return VIBSENSE_ERR_INVAL;
    const source_window_t *sw = const_source_window_for(ctx, accel_source);
    if (!sw)
        return VIBSENSE_ERR_INVAL;
    return fill_window_out(ctx, sw->samples, sw->len, out);
}

static int compute_order_from_window(const vibsense_window_t *win, vibsense_order_snapshot_t *out)
{
    if (!win || !out)
        return VIBSENSE_ERR_INVAL;
    if (!win->samples || win->sample_count == 0u)
        return VIBSENSE_ERR_STATE;

    memset(out, 0, sizeof(*out));
    out->revolutions = win->completed_revolutions;
    out->sample_count = win->sample_count;

    double re[3] = {0.0, 0.0, 0.0};
    double im[3] = {0.0, 0.0, 0.0};
    double sumsq[3] = {0.0, 0.0, 0.0};

    for (uint32_t i = 0; i < win->sample_count; i++)
    {
        const vibsense_window_sample_t *s = &win->samples[i];
        double c = cos(s->phase_rad);
        double sn = sin(s->phase_rad);
        double v[3] = {(double)s->ax, (double)s->ay, (double)s->az};
        for (int a = 0; a < 3; a++)
        {
            re[a] += v[a] * c;
            im[a] -= v[a] * sn;
            sumsq[a] += v[a] * v[a];
        }
    }

    double n = (double)win->sample_count;
    for (int a = 0; a < 3; a++)
    {
        double R = (2.0 / n) * re[a];
        double I = (2.0 / n) * im[a];
        out->order1_amp_counts[a] = hypot(R, I);
        out->order1_phase_rad[a] = atan2(I, R);
        out->rms_counts[a] = sqrt(sumsq[a] / n);
    }

    if (win->end_tick > win->start_tick && win->tick_rate_hz)
    {
        double seconds = (double)(win->end_tick - win->start_tick) / (double)win->tick_rate_hz;
        out->rpm_mean = seconds > 0.0 ? (60.0 * (double)win->completed_revolutions / seconds) : 0.0;
    }

    return VIBSENSE_OK;
}

int vibsense_compute_order_snapshot(const vibsense_context_t *ctx, vibsense_order_snapshot_t *out)
{
    vibsense_window_t win;
    int rc = vibsense_get_window(ctx, &win);
    if (rc != VIBSENSE_OK)
        return rc;
    return compute_order_from_window(&win, out);
}

int vibsense_compute_order_snapshot_by_source(const vibsense_context_t *ctx,
                                              uint8_t accel_source,
                                              vibsense_order_snapshot_t *out)
{
    vibsense_window_t win;
    int rc = vibsense_get_window_by_source(ctx, accel_source, &win);
    if (rc != VIBSENSE_OK)
        return rc;
    return compute_order_from_window(&win, out);
}


static const char *accel_source_name(uint8_t source)
{
    switch (source)
    {
    case VIB_ACCEL_SOURCE_ADXL355:
        return "ADXL355";
    case VIB_ACCEL_SOURCE_IIS3DWBG1:
        return "IIS3DWBG1";
    case VIB_ACCEL_SOURCE_BOTH:
        return "BOTH";
    case VIB_ACCEL_SOURCE_NONE:
        return "NONE";
    default:
        return "MIXED";
    }
}

static void make_utc_now(char *buf, size_t len)
{
    if (!buf || len == 0u)
        return;

    time_t now = time(NULL);
    struct tm *tmp = gmtime(&now);
    struct tm tm_utc;
    if (tmp)
        tm_utc = *tmp;
    else
        memset(&tm_utc, 0, sizeof(tm_utc));
    strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
}

static uint8_t derive_window_source(const vibsense_window_t *win)
{
    if (!win || win->sample_count == 0u)
        return VIB_ACCEL_SOURCE_NONE;

    uint8_t src = win->samples[0].accel_source;
    for (uint32_t i = 1; i < win->sample_count; i++)
    {
        if (win->samples[i].accel_source != src)
            return VIB_ACCEL_SOURCE_BOTH;
    }
    return src;
}

static uint16_t derive_window_range_g(const vibsense_window_t *win)
{
    if (!win || win->sample_count == 0u)
        return 0u;
    return win->samples[0].accel_range_g;
}

static uint16_t derive_window_bits(const vibsense_window_t *win)
{
    if (!win || win->sample_count == 0u)
        return 0u;
    return win->samples[0].raw_sample_bits;
}

static double derive_window_rpm(const vibsense_window_t *win)
{
    if (!win || win->tick_rate_hz == 0u || win->end_tick <= win->start_tick)
        return 0.0;

    double seconds = (double)(win->end_tick - win->start_tick) / (double)win->tick_rate_hz;
    return seconds > 0.0 ? 60.0 * (double)win->completed_revolutions / seconds : 0.0;
}

static int write_window_csv_fp_meta(const vibsense_window_t *win,
                                    FILE *fp,
                                    const vibsense_csv_metadata_t *meta)
{
    if (!win || !fp)
        return VIBSENSE_ERR_INVAL;

    char utc_buf[32];
    const char *utc_time = (meta && meta->utc_time) ? meta->utc_time : NULL;
    if (!utc_time)
    {
        make_utc_now(utc_buf, sizeof(utc_buf));
        utc_time = utc_buf;
    }

    const char *aircraft = (meta && meta->aircraft) ? meta->aircraft : "VH-XVG";
    const char *engine = (meta && meta->engine) ? meta->engine : "Lycoming IO-540";
    const char *sample_name = (meta && meta->sample_name) ? meta->sample_name : "unspecified";
    const char *axis_orientation = (meta && meta->axis_orientation) ?
        meta->axis_orientation : "x=vertical,y=lateral,z=fore-aft";
    const char *optical_reference = (meta && meta->optical_reference) ?
        meta->optical_reference : "0_deg_top_dead_center_viewed_from_spinner_side";
    const char *rotation_direction = (meta && meta->rotation_direction) ?
        meta->rotation_direction : "CW_viewed_from_spinner_side";

    uint32_t tick_rate_hz = (meta && meta->tick_rate_hz) ? meta->tick_rate_hz : win->tick_rate_hz;
    uint32_t target_revolutions = (meta && meta->target_revolutions) ?
        meta->target_revolutions : win->requested_revolutions;
    double rpm_mean = (meta && meta->rpm_mean > 0.0) ? meta->rpm_mean : derive_window_rpm(win);
    uint8_t accel_source = (meta && meta->accel_source) ? meta->accel_source : derive_window_source(win);
    uint16_t accel_range_g = (meta && meta->accel_range_g) ? meta->accel_range_g : derive_window_range_g(win);
    uint16_t accel_bits = (meta && meta->accel_bits) ? meta->accel_bits : derive_window_bits(win);

    if (fprintf(fp, "# vibsense_csv_version: 2\n") < 0 ||
        fprintf(fp, "# aircraft: %s\n", aircraft) < 0 ||
        fprintf(fp, "# engine: %s\n", engine) < 0 ||
        fprintf(fp, "# sample_name: %s\n", sample_name) < 0 ||
        fprintf(fp, "# utc_time: %s\n", utc_time) < 0 ||
        fprintf(fp, "# tick_rate_hz: %u\n", tick_rate_hz) < 0 ||
        fprintf(fp, "# target_revolutions: %u\n", target_revolutions) < 0 ||
        fprintf(fp, "# completed_revolutions: %u\n", win->completed_revolutions) < 0 ||
        fprintf(fp, "# start_tick: %llu\n", (unsigned long long)win->start_tick) < 0 ||
        fprintf(fp, "# end_tick: %llu\n", (unsigned long long)win->end_tick) < 0 ||
        fprintf(fp, "# sample_count: %u\n", win->sample_count) < 0 ||
        fprintf(fp, "# rpm_mean: %.6f\n", rpm_mean) < 0 ||
        fprintf(fp, "# accel_source: %s\n", accel_source_name(accel_source)) < 0 ||
        fprintf(fp, "# accel_range_g: %u\n", accel_range_g) < 0 ||
        fprintf(fp, "# accel_bits: %u\n", accel_bits) < 0 ||
        fprintf(fp, "# axis_orientation: %s\n", axis_orientation) < 0 ||
        fprintf(fp, "# optical_reference: %s\n", optical_reference) < 0 ||
        fprintf(fp, "# rotation_direction: %s\n", rotation_direction) < 0 ||
        fprintf(fp, "tick,rev_offset,phase_deg,ax,ay,az,source,range_g,bits\n") < 0)
        return VIBSENSE_ERR_IO;

    for (uint32_t i = 0; i < win->sample_count; i++)
    {
        const vibsense_window_sample_t *s = &win->samples[i];
        double deg = s->phase_rad * 180.0 / M_PI;
        if (fprintf(fp, "%llu,%u,%.9f,%ld,%ld,%ld,%u,%u,%u\n",
                    (unsigned long long)s->tick,
                    s->revolution_offset,
                    deg,
                    (long)s->ax,
                    (long)s->ay,
                    (long)s->az,
                    s->accel_source,
                    s->accel_range_g,
                    s->raw_sample_bits) < 0)
            return VIBSENSE_ERR_IO;
    }

    return VIBSENSE_OK;
}



int vibsense_write_window_csv_fp(const vibsense_context_t *ctx, FILE *fp)
{
    return vibsense_write_window_csv_fp_with_metadata(ctx, fp, NULL);
}

int vibsense_write_window_csv(const vibsense_context_t *ctx, const char *path)
{
    return vibsense_write_window_csv_with_metadata(ctx, path, NULL);
}

int vibsense_write_window_csv_by_source_fp(const vibsense_context_t *ctx, uint8_t accel_source, FILE *fp)
{
    return vibsense_write_window_csv_by_source_fp_with_metadata(ctx, accel_source, fp, NULL);
}

int vibsense_write_window_csv_by_source(const vibsense_context_t *ctx, uint8_t accel_source, const char *path)
{
    return vibsense_write_window_csv_by_source_with_metadata(ctx, accel_source, path, NULL);
}

int vibsense_write_window_csv_fp_with_metadata(const vibsense_context_t *ctx,
                                               FILE *fp,
                                               const vibsense_csv_metadata_t *meta)
{
    vibsense_window_t win;
    int rc = vibsense_get_window(ctx, &win);
    if (rc != VIBSENSE_OK)
        return rc;
    return write_window_csv_fp_meta(&win, fp, meta);
}

int vibsense_write_window_csv_with_metadata(const vibsense_context_t *ctx,
                                            const char *path,
                                            const vibsense_csv_metadata_t *meta)
{
    if (!path)
        return VIBSENSE_ERR_INVAL;

    FILE *fp = fopen(path, "w");
    if (!fp)
        return VIBSENSE_ERR_IO;

    int rc = vibsense_write_window_csv_fp_with_metadata(ctx, fp, meta);
    if (fclose(fp) != 0 && rc == VIBSENSE_OK)
        rc = VIBSENSE_ERR_IO;
    return rc;
}

int vibsense_write_window_csv_by_source_fp_with_metadata(const vibsense_context_t *ctx,
                                                         uint8_t accel_source,
                                                         FILE *fp,
                                                         const vibsense_csv_metadata_t *meta)
{
    vibsense_window_t win;
    int rc = vibsense_get_window_by_source(ctx, accel_source, &win);
    if (rc != VIBSENSE_OK)
        return rc;
    return write_window_csv_fp_meta(&win, fp, meta);
}

int vibsense_write_window_csv_by_source_with_metadata(const vibsense_context_t *ctx,
                                                      uint8_t accel_source,
                                                      const char *path,
                                                      const vibsense_csv_metadata_t *meta)
{
    if (!path)
        return VIBSENSE_ERR_INVAL;

    FILE *fp = fopen(path, "w");
    if (!fp)
        return VIBSENSE_ERR_IO;

    int rc = vibsense_write_window_csv_by_source_fp_with_metadata(ctx, accel_source, fp, meta);
    if (fclose(fp) != 0 && rc == VIBSENSE_OK)
        rc = VIBSENSE_ERR_IO;
    return rc;
}
