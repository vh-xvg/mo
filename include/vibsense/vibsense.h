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

#ifndef VIBSENSE_H
#define VIBSENSE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "vib_sense_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VIBSENSE_OK            0
#define VIBSENSE_DONE          1
#define VIBSENSE_ERR_INVAL    -1
#define VIBSENSE_ERR_NOMEM    -2
#define VIBSENSE_ERR_STATE    -3
#define VIBSENSE_ERR_PROTOCOL -4
#define VIBSENSE_ERR_IO       -5
#define VIBSENSE_ERR_NOLOCK   -6

/* Source-mask bits used by vibsense_arm_sources(). */
#define VIBSENSE_ACCEL_MASK_ADXL355    (1u << 0)
#define VIBSENSE_ACCEL_MASK_IIS3DWBG1  (1u << 1)
#define VIBSENSE_ACCEL_MASK_BOTH       (VIBSENSE_ACCEL_MASK_ADXL355 | VIBSENSE_ACCEL_MASK_IIS3DWBG1)
#define VIBSENSE_ACCEL_MASK_ANY        0u

typedef struct vibsense_context vibsense_context_t;

typedef struct
{
    uint64_t tick;
    double   phase_rad;          /* [0, 2*pi), relative to optical TDC/index */
    uint32_t revolution_offset;  /* 0..target_revolutions-1 */
    int32_t  ax;
    int32_t  ay;
    int32_t  az;
    uint8_t  accel_source;
    uint8_t  accel_range_g;
    uint16_t raw_sample_bits;
} vibsense_window_sample_t;

typedef struct
{
    uint32_t requested_revolutions;
    uint32_t completed_revolutions;
    uint64_t start_tick;         /* optical TDC/index starting boundary */
    uint64_t end_tick;           /* optical TDC/index ending boundary */
    uint32_t tick_rate_hz;
    uint32_t sample_count;
    const vibsense_window_sample_t *samples;
} vibsense_window_t;


/*
 * Optional metadata written as commented CSV header lines.
 * Pass NULL strings to use library defaults where available.
 * utc_time should be ISO-8601 UTC, e.g. 2026-05-20T10:15:00Z.
 * If utc_time is NULL, the writer generates the current UTC time.
 * rpm_mean <= 0.0 means derive RPM from window ticks/revolutions.
 * accel_source/range/bits set to 0 mean derive from the window samples.
 */
typedef struct
{
    const char *aircraft;
    const char *engine;
    const char *sample_name;
    const char *utc_time;

    uint32_t tick_rate_hz;
    uint32_t target_revolutions;
    double   rpm_mean;

    uint8_t  accel_source;
    uint16_t accel_range_g;
    uint16_t accel_bits;

    const char *axis_orientation;
    const char *optical_reference;
    const char *rotation_direction;
} vibsense_csv_metadata_t;

typedef struct
{
    uint32_t revolutions;
    uint32_t sample_count;
    double rpm_mean;
    double rpm_stddev;
    double order1_amp_counts[3];
    double order1_phase_rad[3];
    double rms_counts[3];
    uint32_t quality_flags;
} vibsense_order_snapshot_t;

vibsense_context_t *vibsense_create(void);
void vibsense_destroy(vibsense_context_t *ctx);

/* Reset all state and free all old dynamic window/tach/accel memory. */
int vibsense_reset(vibsense_context_t *ctx);

/*
 * Arm a new acquisition window for any active accelerometer source.
 * This is backward-compatible with earlier single-source code.
 */
int vibsense_arm(vibsense_context_t *ctx, uint32_t target_revolutions);

/*
 * Arm a new acquisition window and specify the accelerometer source mask.
 * Use VIBSENSE_ACCEL_MASK_IIS3DWBG1, VIBSENSE_ACCEL_MASK_ADXL355, or
 * VIBSENSE_ACCEL_MASK_BOTH.  VIBSENSE_ACCEL_MASK_ANY accepts whichever accel
 * source appears and completes after any source drains past the ending index.
 */
int vibsense_arm_sources(vibsense_context_t *ctx,
                         uint32_t target_revolutions,
                         uint32_t accel_source_mask);

/*
 * Convenience wrapper accepting vib protocol source values:
 *   VIB_ACCEL_SOURCE_IIS3DWBG1, VIB_ACCEL_SOURCE_ADXL355, VIB_ACCEL_SOURCE_BOTH.
 */
int vibsense_arm_accel_source(vibsense_context_t *ctx,
                              uint32_t target_revolutions,
                              uint8_t accel_source);

/*
 * Ingest one vib_sense_protocol.h payload.
 * Returns:
 *   VIBSENSE_OK    still accumulating or idle
 *   VIBSENSE_DONE  the requested full-rotation window is complete and drained
 *   <0             error
 */
int vibsense_ingest_payload(vibsense_context_t *ctx, const void *payload, size_t payload_len);

/* Get the completed, trimmed, phase-annotated mixed-source window. Pointer remains owned by ctx. */
int vibsense_get_window(const vibsense_context_t *ctx, vibsense_window_t *out);

/*
 * Get the completed, trimmed, phase-annotated window for one accelerometer source.
 * The returned samples pointer is owned by ctx and remains valid until reset/arm/destroy.
 */
int vibsense_get_window_by_source(const vibsense_context_t *ctx,
                                  uint8_t accel_source,
                                  vibsense_window_t *out);

/* Compute 1/rev metrics from the completed mixed-source window. */
int vibsense_compute_order_snapshot(const vibsense_context_t *ctx, vibsense_order_snapshot_t *out);

/* Compute 1/rev metrics from one accelerometer source. */
int vibsense_compute_order_snapshot_by_source(const vibsense_context_t *ctx,
                                              uint8_t accel_source,
                                              vibsense_order_snapshot_t *out);

/* Convenience exports. */
int vibsense_write_window_csv(const vibsense_context_t *ctx, const char *path);
int vibsense_write_window_csv_fp(const vibsense_context_t *ctx, FILE *fp);
int vibsense_write_window_csv_by_source(const vibsense_context_t *ctx, uint8_t accel_source, const char *path);
int vibsense_write_window_csv_by_source_fp(const vibsense_context_t *ctx, uint8_t accel_source, FILE *fp);

int vibsense_write_window_csv_with_metadata(const vibsense_context_t *ctx,
                                            const char *path,
                                            const vibsense_csv_metadata_t *meta);
int vibsense_write_window_csv_fp_with_metadata(const vibsense_context_t *ctx,
                                               FILE *fp,
                                               const vibsense_csv_metadata_t *meta);
int vibsense_write_window_csv_by_source_with_metadata(const vibsense_context_t *ctx,
                                                      uint8_t accel_source,
                                                      const char *path,
                                                      const vibsense_csv_metadata_t *meta);
int vibsense_write_window_csv_by_source_fp_with_metadata(const vibsense_context_t *ctx,
                                                         uint8_t accel_source,
                                                         FILE *fp,
                                                         const vibsense_csv_metadata_t *meta);

#ifdef __cplusplus
}
#endif

#endif
