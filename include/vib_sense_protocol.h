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

#ifndef VIB_SENSE_PROTOCOL_H
#define VIB_SENSE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Prop balance sensor protocol payload definitions
 *
 * Target hardware architecture:
 * - STM32C071 head unit.
 * - Shared SPI1 bus.
 * - ADXL355 optional accelerometer, chip select PA4, DRDY on PB0.
 * - IIS3DWBG1/IIS3DWB optional accelerometer, chip select PA3,
 *   INT1 on PA10, INT2 on PB5.
 * - OPB732 optical once-per-revolution index on PA0 / TIM2_CH1.
 * - SDS EM-5 tach input, nominally 3 pulses per revolution on an IO-540,
 *   on PA2 / TIM2_CH3 after input shaping.
 *
 * Notes:
 * - These structures define only the payload contents returned by the sense
 *   system. Transport framing, command/response encapsulation, and CRC-32 are
 *   handled externally.
 * - All multi-byte fields are little-endian on the wire.
 * - Raw accelerometer samples are transmitted as sign-extended 32-bit integers.
 *   Scale, ODR, and sensor source are reported separately so the Raspberry Pi
 *   can convert counts to physical units.
 * - The timestamp timebase is expected to be a single free-running MCU tick
 *   counter shared by accelerometer sampling, optical index capture, and SDS
 *   tach capture.
 */

#pragma pack(push, 4)

/* --------------------------------------------------------------------------
 * Protocol constants
 * -------------------------------------------------------------------------- */

enum
{
    VIB_PAYLOAD_STATUS_V1          = 0x01,
    VIB_PAYLOAD_ACCEL_BLOCK_V1     = 0x02,
    VIB_PAYLOAD_TACH_EVENT_V1      = 0x03,
    VIB_PAYLOAD_TACH_DEBUG_V1      = 0x04,
    VIB_PAYLOAD_CONFIG_SNAPSHOT_V1 = 0x05
};

enum
{
    VIB_PAYLOAD_VERSION_1 = 1
};

/* --------------------------------------------------------------------------
 * Accelerometer source values
 * -------------------------------------------------------------------------- */

enum
{
    VIB_ACCEL_SOURCE_NONE      = 0,
    VIB_ACCEL_SOURCE_ADXL355   = 1,
    VIB_ACCEL_SOURCE_IIS3DWBG1 = 2,
    VIB_ACCEL_SOURCE_BOTH      = 3
};

/* --------------------------------------------------------------------------
 * Tach source values
 * -------------------------------------------------------------------------- */

enum
{
    VIB_TACH_SOURCE_OPTICAL = 0,   /* OPB732 optical index, nominally 1 pulse/rev */
    VIB_TACH_SOURCE_SDS     = 1    /* SDS EM-5 tach output, nominally 3 pulses/rev on IO-540 */
};

/* --------------------------------------------------------------------------
 * Status flags
 * -------------------------------------------------------------------------- */

enum
{
    VIB_STATUS_FLAG_USB_OK                         = (1u << 0),
    VIB_STATUS_FLAG_USING_EXTERNAL_CLOCK           = (1u << 1),

    VIB_STATUS_FLAG_ADXL355_PRESENT                = (1u << 2),
    VIB_STATUS_FLAG_ADXL355_OK                     = (1u << 3),
    VIB_STATUS_FLAG_ADXL355_DRDY_PRESENT           = (1u << 4),

    VIB_STATUS_FLAG_IIS3DWBG1_PRESENT              = (1u << 5),
    VIB_STATUS_FLAG_IIS3DWBG1_OK                   = (1u << 6),
    VIB_STATUS_FLAG_IIS3DWBG1_INT1_PRESENT         = (1u << 7),
    VIB_STATUS_FLAG_IIS3DWBG1_INT2_PRESENT         = (1u << 8),

    VIB_STATUS_FLAG_OPTICAL_TACH_OK                = (1u << 9),
    VIB_STATUS_FLAG_OPTICAL_SIGNAL_PRESENT         = (1u << 10),
    VIB_STATUS_FLAG_OPTICAL_PERIOD_VALID           = (1u << 11),

    VIB_STATUS_FLAG_SDS_TACH_OK                    = (1u << 12),
    VIB_STATUS_FLAG_SDS_SIGNAL_PRESENT             = (1u << 13),
    VIB_STATUS_FLAG_SDS_PERIOD_VALID               = (1u << 14),

    VIB_STATUS_FLAG_ROTATION_MODEL_LOCKED          = (1u << 15),
    VIB_STATUS_FLAG_SAMPLE_OVERRUN_LATCHED         = (1u << 16),
    VIB_STATUS_FLAG_SPI_ERROR_LATCHED              = (1u << 17),
    VIB_STATUS_FLAG_USB_BACKPRESSURE_LATCHED       = (1u << 18),
    VIB_STATUS_FLAG_CONFIG_DIRTY                   = (1u << 19)
};

/* --------------------------------------------------------------------------
 * Temperature flags
 * -------------------------------------------------------------------------- */

enum
{
    VIB_TEMP_FLAG_ADXL355_VALID   = (1u << 0),
    VIB_TEMP_FLAG_IIS3DWBG1_VALID = (1u << 1),
    VIB_TEMP_FLAG_MCU_VALID       = (1u << 2)
};

/* --------------------------------------------------------------------------
 * Accel block flags
 * -------------------------------------------------------------------------- */

enum
{
    VIB_ACCEL_BLOCK_FLAG_TIMING_NOMINAL       = (1u << 0),
    VIB_ACCEL_BLOCK_FLAG_CONTAINS_GAP_BEFORE  = (1u << 1),
    VIB_ACCEL_BLOCK_FLAG_SAMPLES_LOST         = (1u << 2),
    VIB_ACCEL_BLOCK_FLAG_SAMPLE_CLOCK_RESYNC  = (1u << 3),
    VIB_ACCEL_BLOCK_FLAG_DRDY_MISSED          = (1u << 4),
    VIB_ACCEL_BLOCK_FLAG_FIFO_WATERMARK       = (1u << 5),
    VIB_ACCEL_BLOCK_FLAG_FIFO_OVERRUN         = (1u << 6)
};

/* --------------------------------------------------------------------------
 * Tach event flags
 * -------------------------------------------------------------------------- */

enum
{
    VIB_TACH_EVENT_FLAG_ACCEPTED_PULSE                = (1u << 0),
    VIB_TACH_EVENT_FLAG_PERIOD_VALID                  = (1u << 1),
    VIB_TACH_EVENT_FLAG_PERIOD_PREDICTED_FROM_PRIOR   = (1u << 2),
    VIB_TACH_EVENT_FLAG_EDGE_RISING                   = (1u << 3),
    VIB_TACH_EVENT_FLAG_PHASE_OFFSET_APPLIED_IN_MCU   = (1u << 4),
    VIB_TACH_EVENT_FLAG_DETECTOR_MARGIN_LOW           = (1u << 5),
    VIB_TACH_EVENT_FLAG_PULSE_AFTER_BLANK_WINDOW      = (1u << 6),
    VIB_TACH_EVENT_FLAG_RECOVERED_AFTER_DROPOUT       = (1u << 7),
    VIB_TACH_EVENT_FLAG_OPTICAL_INDEX                 = (1u << 8),
    VIB_TACH_EVENT_FLAG_SDS_PULSE                     = (1u << 9)
};

/* --------------------------------------------------------------------------
 * Threshold mode values
 * -------------------------------------------------------------------------- */

enum
{
    VIB_THRESHOLD_MODE_FIXED_DIVIDER  = 0,
    VIB_THRESHOLD_MODE_TRIM_POT       = 1,
    VIB_THRESHOLD_MODE_DAC_CONTROLLED = 2,
    VIB_THRESHOLD_MODE_COMPARATOR     = 3
};

/* --------------------------------------------------------------------------
 * Common payload header
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t  type;
    uint8_t  version;
    uint16_t length;
} vib_payload_header_t;

/* --------------------------------------------------------------------------
 * Status payload v1
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t  type;                 /* VIB_PAYLOAD_STATUS_V1 */
    uint8_t  version;              /* VIB_PAYLOAD_VERSION_1 */
    uint16_t length;               /* sizeof(vib_status_v1_t) */

    uint32_t seq;
    uint32_t timestamp_ticks;

    uint32_t uptime_ms;
    uint32_t sample_rate_hz;
    uint32_t timer_tick_hz;

    uint32_t accel_samples_sent;
    uint32_t tach_events_sent;

    uint32_t accel_overruns;
    uint32_t usb_backpressure_events;
    uint32_t spi_errors;
    uint32_t tach_reject_count;
    uint32_t accel_samples_dropped_total;

    uint32_t system_flags;
    uint16_t temp_flags;
    uint8_t  active_accel_source;      /* VIB_ACCEL_SOURCE_* */
    uint8_t  reserved0;

    int32_t  last_valid_optical_period_ticks;
    int32_t  last_valid_sds_period_ticks;
    int32_t  last_valid_rpm_millirpm;  /* Derived, preferably from SDS if valid */

    uint16_t optical_threshold_adc;
    uint16_t optical_detector_level_adc;

    int16_t  adxl355_temp_cC;          /* centi-deg C, valid if flag set */
    int16_t  iis3dwbg1_temp_cC;        /* centi-deg C, valid if flag set */
    int16_t  mcu_temp_cC;              /* centi-deg C, valid if flag set */
    int16_t  reserved1;
} vib_status_v1_t;

/* --------------------------------------------------------------------------
 * Raw accel sample
 * -------------------------------------------------------------------------- */

typedef struct
{
    int32_t ax;
    int32_t ay;
    int32_t az;
} vib_accel_sample_raw_t;

/* --------------------------------------------------------------------------
 * Accel block payload header v1
 *
 * Followed immediately by sample_count instances of vib_accel_sample_raw_t.
 * Total payload length is:
 *   sizeof(vib_accel_block_v1_hdr_t) +
 *   sample_count * sizeof(vib_accel_sample_raw_t)
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t  type;                 /* VIB_PAYLOAD_ACCEL_BLOCK_V1 */
    uint8_t  version;              /* VIB_PAYLOAD_VERSION_1 */
    uint16_t length;               /* total payload bytes including samples */

    uint32_t seq;

    uint32_t first_sample_ticks;
    uint16_t sample_dt_ticks;
    uint16_t sample_count;
    uint32_t last_sample_ticks;

    uint32_t block_flags;
    uint32_t tach_event_count_seen;

    uint8_t  accel_source;         /* VIB_ACCEL_SOURCE_ADXL355 or VIB_ACCEL_SOURCE_IIS3DWBG1 */
    uint8_t  accel_range_g;        /* nominal full scale, e.g. 2/4/8/16 */
    uint16_t raw_sample_bits;      /* 20 for ADXL355, 16 for IIS3DWBG1 when sign-extended */
} vib_accel_block_v1_hdr_t;

/* --------------------------------------------------------------------------
 * Tach event payload v1
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t  type;                 /* VIB_PAYLOAD_TACH_EVENT_V1 */
    uint8_t  version;              /* VIB_PAYLOAD_VERSION_1 */
    uint16_t length;               /* sizeof(vib_tach_event_v1_t) */

    uint32_t seq;

    uint32_t event_ticks;
    uint32_t period_ticks;
    uint32_t pulse_index;
    uint32_t revolution_index;

    uint16_t event_flags;
    uint8_t  source;               /* VIB_TACH_SOURCE_OPTICAL or VIB_TACH_SOURCE_SDS */
    uint8_t  pulses_per_rev;       /* 1 optical, 3 SDS for six-cylinder EM-5 */

    uint16_t pulse_width_ticks_lo16;
    int16_t  detector_peak_adc;
    int16_t  detector_baseline_adc;
    uint16_t reserved0;
} vib_tach_event_v1_t;

/* --------------------------------------------------------------------------
 * Tach debug / analog payload v1
 *
 * Primarily for the optical analog/comparator front end. SDS diagnostics should
 * normally be inferred from tach event period/jitter statistics.
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t  type;                 /* VIB_PAYLOAD_TACH_DEBUG_V1 */
    uint8_t  version;              /* VIB_PAYLOAD_VERSION_1 */
    uint16_t length;               /* sizeof(vib_tach_debug_v1_t) */

    uint32_t seq;
    uint32_t timestamp_ticks;

    uint16_t optical_threshold_adc;
    uint16_t optical_filtered_adc;
    uint16_t optical_raw_adc;
    uint16_t reserved0;
} vib_tach_debug_v1_t;

/* --------------------------------------------------------------------------
 * Configuration snapshot payload v1
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t  type;                 /* VIB_PAYLOAD_CONFIG_SNAPSHOT_V1 */
    uint8_t  version;              /* VIB_PAYLOAD_VERSION_1 */
    uint16_t length;               /* sizeof(vib_config_snapshot_v1_t) */

    uint32_t seq;
    uint32_t timestamp_ticks;

    uint32_t sample_rate_hz;
    uint32_t tick_rate_hz;

    uint16_t led_pulse_width_us;
    uint16_t led_rep_rate_hz;

    uint16_t optical_blank_percent_x100;
    uint16_t tach_period_jump_limit_percent_x100;

    uint16_t sample_block_size;
    uint16_t threshold_mode;

    int16_t  optical_phase_offset_mdeg;
    int16_t  sds_phase_offset_mdeg;

    uint8_t  optical_pulses_per_rev;  /* normally 1 */
    uint8_t  sds_pulses_per_rev;      /* normally 3 */
    uint8_t  active_accel_source;     /* VIB_ACCEL_SOURCE_* */
    uint8_t  configured_accel_sources;/* bitmask or VIB_ACCEL_SOURCE_BOTH */

    uint16_t adxl355_range_g;
    uint16_t iis3dwbg1_range_g;

    uint16_t adxl355_sample_bits;     /* normally 20 */
    uint16_t iis3dwbg1_sample_bits;   /* normally 16 */
} vib_config_snapshot_v1_t;

#pragma pack(pop)

_Static_assert(sizeof(vib_payload_header_t) == 4, "Size mismatch");
_Static_assert(sizeof(vib_status_v1_t) == 84, "Size mismatch");
_Static_assert(sizeof(vib_accel_sample_raw_t) == 12, "Size mismatch");
_Static_assert(sizeof(vib_accel_block_v1_hdr_t) == 32, "Size mismatch");
_Static_assert(sizeof(vib_tach_event_v1_t) == 36, "Size mismatch");
_Static_assert(sizeof(vib_tach_debug_v1_t) == 20, "Size mismatch");
_Static_assert(sizeof(vib_config_snapshot_v1_t) == 48, "Size mismatch");

#ifdef __cplusplus
}
#endif

#endif /* VIB_SENSE_PROTOCOL_H */
