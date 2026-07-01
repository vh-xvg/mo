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

//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 3 of the License, or (at your option)
// any later version. See COPYING for more details.
//
// Useful data structures and values for interfacing with XVG's EFI Power module
//
//

#ifndef _EFI_PROTOCOL_H_
#define _EFI_PROTOCOL_H_

#include "vib_sense_protocol.h"

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

#define EFI_PROTOCOL_VERSION    ((0<<8)|1)

#define EFI_PREAMBLE            (uint8_t) 0xaa

// Timeout used for serial operations
#define EFI_TIMEOUT_USEC        500000              // 500 msec timeout
#define HF_TIMEOUT_USEC         500000              // 500 msec timeout

// Error codes for header status byte
#define S_OK                    0
#define S_BUSY                  1
#define S_NODEV                 2
#define S_BADMODE               3
#define S_INVALID               4

// Local error codes
#define EBADHEADERCRC           -100
#define EBADHEADERSTS           -101
#define EBADDATACRC             -102
#define EDATATIMEOUT            -103

// Length of a command frame
#define XVG_EFI_COMMANDLEN      16

// Serial protocol operation codes (Second header byte)
#define OP_NULL                 0
#define OP_RESET                1
#define OP_ECHO                 2
#define OP_TEST                 3
#define OP_CONFIG               4
#define OP_BIAS                 5
#define OP_ENABLE_DISABLE       6
#define OP_SCALE_FACTORS        7
#define OP_TPS_LATCH            8           /* Set TPS latch conditions */
#define OP_TPS_RESET            9           /* Reset a TPS device       */
#define OP_PWM                  31          /* Used on load board only */
#define OP_CURRENTS             32
#define OP_STATS                33
#define OP_TEMPS                34
#define OP_LOG                  35          /* Get LOG records */
#define OP_FPS_RECORD           36          /* Get fuel pump start record */

// Opcodes processed by the flap and avionics redundancy controller
#define OP_FLAP                 37          /* Manipulate flaps and parameters */
#define OP_BPOWER               38          /* Operations on backup power board */
#define OP_FLAP_SETTINGS        39          /* Get/Set flap position settings */

// Opcodes processed by the vibration head sensor unit
#define OP_VIB                  40          /* Control vibration sensor */

#define OP_MAX                  40          /* Make this the same as the last one */

// The sequence distance between a sent and received sequence number.
#define EFI_SEQUENCE_DISTANCE(tx,rx)        ((tx)>=(rx)?((tx)-(rx)):(info->num_sequence+(tx)-(rx)))

#define U32SIZE(x)                      (sizeof(x)/sizeof(uint32_t))

// Baud rate vs. code for gpi[7:5] coming out of reset
#define EFI_BAUD_RATE                   115200

// opdata1 fields for OP_TEST
#define TEST_INJECTORS                  0x1
#define TEST_COILPACK_LEFT              0x2
#define TEST_COILPACK_RIGHT             0x4
#define TEST_FUELPUMPS                  0x8

// opdata1 fields for OP_TEMPS
#define TURN_ON_TEMP_MODE               0x1
#define TURN_OFF_TEMP_MODE              0x2
#define GET_TEMPS                       0x4

// opdata1 fields for OP_FLAP
#define FLAP_UP_BUTTON                  0x1     /* Momentary UP command */
#define FLAP_DOWN_BUTTON                0x2     /* Momentary DOWN command */
#define FLAP_MODE_CONTINUOUS            0x4     /* Force flap mode to be continuous */
                                                /* In this case soft buttons are not active */
#define FLAP_MODE_AIRSPEED_ON           0x8     /* Flap controller airspeed checks on */
#define FLAP_MODE_AIRSPEED_OFF          0x10    /* Flap controller airspeed checks off */

// sequence fields for OP_FLAP_SETTINGS
#define FLAP_SETTINGS_PUT               0x80     /* Put new flap position settings */
#define FLAP_SENSOR_REVERSED            0x40     /* Flap sensor is reverse polarity */

// pwm, opdata2 = {pulse,period} counts
// Units are currently 100 usec increments, see tim15 parameters
#define TEST_LOAD_PWM                   0x10
#define PWM_PERIOD(x)                   ((uint16_t)((x)&0xffff))
#define PWM_PULSE(x)                    ((uint16_t)(((x)>>16)&0xffff))
#define PWM_SPEC(pulse,period)          ((((uint32_t)((pulse)))<<16)|(period))

//#define TEST_ALL (TEST_INJECTORS|TEST_COILPACK_LEFT|TEST_COILPACK_RIGHT|TEST_FUELPUMPS|TEST_JUNCTION_TEMPS)
#define TEST_ALL (TEST_INJECTORS|TEST_COILPACK_LEFT|TEST_COILPACK_RIGHT|TEST_FUELPUMPS)

// Types for calls to get_fake_data() etc.
#define DT_INJECTOR                     1
#define DT_COILPACK                     2
#define DT_FUELPUMP                     3

// How many of each type of thing we have
#define CYLINDERS                       6
#define COILPACKS                       2
#define FUELPUMPS                       2

#define NUM_CURRENTS                  (CYLINDERS+COILPACKS+FUELPUMPS)

// How the outside world sees things
#define IS_INJECTOR(x)                  ((x)<CYLINDERS)
#if 0
#define IS_COILPACK(x)                  (((x)==6)||((x)==8))
#define IS_FUELPUMP(x)                  (((x)==7)||((x)==9))
#define IS_LEFT_COILPACK(x)             ((x)==6)
#define IS_RIGHT_COILPACK(x)            ((x)==8)
#define IS_LEFT_FUELPUMP(x)             ((x)==7)
#define IS_RIGHT_FUELPUMP(x)            ((x)==9)
#endif
#define IS_COILPACK(x)                  (((x)==6)||((x)==7))
#define IS_FUELPUMP(x)                  (((x)==8)||((x)==9))
#define IS_LEFT_COILPACK(x)             ((x)==6)
#define IS_RIGHT_COILPACK(x)            ((x)==7)
#define IS_LEFT_FUELPUMP(x)             ((x)==8)
#define IS_RIGHT_FUELPUMP(x)            ((x)==9)

// How many points of data are transferred, max, for each type, in an OP_CURRENTS frame
// These numbers are determined by the display dimensions and how the displays are
// set up. If less than these are requested, data will be scaled.
#define MAX_INJECTOR_POINTS                 288
#define SUMMARY_INJECTOR_POINTS             80
#define MAX_COILPUMP_POINTS                 448

// Bitmasks for what is being asked for in opdata1 of an OP_CURRENTS
#define OP_CURRENT_INJECTORS_MASK       0x3f
#define OP_CURRENT_INJECTORS_SHIFT      0
#define OP_CURRENT_COILPACKS_MASK       (0x3<<CYLINDERS)
#define OP_CURRENT_COILPACKS_SHIFT      CYLINDERS
#define OP_CURRENT_FUELPUMPS_MASK       (0x3<<(CYLINDERS+COILPACKS))
#define OP_CURRENT_FUELPUMPS_SHIFT      (CYLINDERS+COILPACKS)
#define OP_CURRENT_ALL_MASK             (OP_CURRENT_INJECTORS_MASK | OP_CURRENT_COILPACKS_MASK | OP_CURRENT_FUELPUMPS_MASK)

// Definitions for recording fuel pump starting surge
// Sample every 48 point, that's nominally 1.99 msec
// Capture 400 of these, and that's 1 second. 400 fits
// nicely on the display.
#define FPS_SAMPLE_RATE      48
#define FPS_RECORDS         400


// Structure definitions, LE platforms
//#if __BYTE_ORDER == __BIG_ENDIAN && !defined(WIN32)
//  #include "efi_protocol_be.h"
//#else

// Statistic counters
typedef struct efi_stats {
  uint32_t rx_length_errors;
  uint32_t oversampled;
  uint32_t tracking_single_reversals;
  uint32_t tracking_double_reversals;
  uint32_t preamble_errors;
  uint32_t header_crc_errors;
  uint32_t sequence_errors;
  uint32_t tx_busy;
  uint32_t i2c1_write_requests;
  uint32_t i2c1_write_interrupts;
  uint32_t i2c1_read_requests;
  uint32_t i2c1_read_interrupts;
  uint32_t i2c1_start_failures;
  uint32_t i2c1_busy_failures;
  uint32_t i2c1_start_error_failures;
  uint32_t i2c1_last_hal_error;
  uint32_t i2c1_recover_request;
  uint32_t i2c1_error_count;
  uint32_t i2c1_fatal_error_count;
  uint32_t fatal_error_count;
  uint32_t single_glitches[NUM_CURRENTS];
  uint32_t double_glitches[NUM_CURRENTS];
  uint32_t pulses[NUM_CURRENTS];
  uint32_t overrun[NUM_CURRENTS];
  uint32_t faults[NUM_CURRENTS];
  uint32_t log_records_discarded[NUM_CURRENTS];
  } efi_stats_t;

typedef struct flaps_stats {
  uint32_t rx_length_errors;
  uint32_t preamble_errors;
  uint32_t header_crc_errors;
  uint32_t sequence_errors;
  } flaps_stats_t;

// Options for the mo application
typedef struct efi_options {
  uint8_t door_switches;
  uint8_t engine_start_interlock;
  uint8_t flap_controller;
  uint8_t old_battery_backup_board;
  uint8_t battery_display;
  uint8_t o2_sensor;
  uint8_t vibration_sensor;                     // Vib sensor present
  uint8_t vib_source;                           // Which source (or both), see vib_sense_protocol.h
  uint8_t vib_live_rotations;                   // Revs of data for live display
  uint8_t vib_snapshot_rotations;               // Revs of data for snapshot (csv) file
  uint8_t ac_display;
  uint8_t hf_display;
  } efi_options_t;

// Generic header
struct efi_header {
        uint8_t  preamble;                      // Magic #, always 0xaa
        uint8_t  opcode;                        // The operation being requested/performed
        int8_t   status;                        // Return status in responses
        uint8_t  sequence;                      // Modulo 256 sequence #, incremented for each request
        uint32_t opdata1;
        uint32_t opdata2;                       // Length of subsequent frame in some cases (always multiple of 4)
        uint32_t crc32;                         // Computed across previous 12 bytes inclusive
        } __attribute__((packed,aligned(8)));   // 16 bytes total

struct injector_detail {
        uint16_t duration_usec;
        uint16_t current_ma;                    // Normally peak, b15 set -> average
        } __attribute__((packed,aligned(4)));

struct coilpack_detail {
        uint16_t dwell_time_usec;
        uint16_t current_ma;                   // Normally peak, b15 set -> average
        uint16_t derived_rpm;
        uint16_t spare;
        } __attribute__((packed,aligned(4)));

struct fuelpump_detail {
        uint16_t run_time_seconds;
        uint16_t current_ma;                    // Normally average (ignore b15)
        uint16_t surge_current_ma;              // From last startup
        uint16_t ripple_ma;                     // Commutator ripple
        } __attribute__((packed,aligned(4)));

struct efi_currents_header {
        uint32_t latch_power_fault_status;      // b0 up = latch faults, b16 up = fault state
        uint32_t update_status;
          union {
            struct injector_detail injector;
            struct coilpack_detail coil;
            struct fuelpump_detail pump;
          } u[NUM_CURRENTS];
        } __attribute__((packed,aligned(4)));

// Maximum size OP_CURRENTS response frame
struct efi_currents {
        struct efi_currents_header status;
        uint8_t injector_current[CYLINDERS][MAX_INJECTOR_POINTS];
        uint8_t coilpack_current[COILPACKS][MAX_COILPUMP_POINTS];
        uint8_t fuelpump_current[FUELPUMPS][MAX_COILPUMP_POINTS];
        } __attribute__((packed,aligned(4)));

struct efi_temps {
        uint16_t junction_temperature[NUM_CURRENTS];
        } __attribute__((packed,aligned(4)));

struct fp_temps {
        float junction_temperature[NUM_CURRENTS];
        } __attribute__((packed,aligned(4)));

struct gui_temps {
        char junction_temperature[NUM_CURRENTS][16];
        } __attribute__((packed,aligned(4)));

struct log_offset {
        uint16_t offset;
        uint16_t length;
        } __attribute__((packed,aligned(4)));

#define MAX_EFI_FRAME_DATA_SIZE (8192 - sizeof(struct efi_header) - 4)      /* -4 is for CRC32 */
#define MAX_LOG_DATA_BUFSIZE    MAX_EFI_FRAME_DATA_SIZE - sizeof(struct log_offset) * NUM_CURRENTS

struct log_data {
        struct log_offset offset[NUM_CURRENTS];
        uint8_t data[MAX_LOG_DATA_BUFSIZE];
        } __attribute__((packed,aligned(4)));

struct pump_start_record {
        uint32_t fuelpump_start_tick;
        uint16_t data[FPS_RECORDS];
        } __attribute__((packed,aligned(4)));

#define CONFIG_MAGIC 0xAA42

struct config_data {                        // Keep this a multiple of 8 bytes
        uint16_t magic;
        uint8_t board_revision_major;
        uint8_t board_revision_minor;
        uint8_t cylinders;
        uint8_t coilpacks;
        uint8_t fuelpumps;
        uint8_t spare1;
        uint32_t firmware_revision;
        uint32_t board_serial_number;
        uint32_t spare2;
        uint32_t spare3;
        } __attribute__((packed,aligned(4)));

struct flap_settings {
        uint16_t reflex_position;                   // ADC counts 
        uint16_t zero_position;
        uint16_t half_position;
        uint16_t full_position;
        uint16_t deadband;
        } __attribute__((packed,aligned(4)));

// Default flap settings, changed at run time
#define DF_REFLEX           300
#define DF_ZERO             800
#define DF_HALF             2000
#define DF_FULL             3700
#define DF_DEADBAND         30

struct bpower_data {
        uint16_t primary_voltage;                   // ADC counts
        uint16_t secondary_voltage;
        uint16_t battery_voltage;
        uint16_t current_flap_position;
        uint16_t flap_motor_current;
        uint16_t efis_airspeed;
        uint16_t spare;
        uint16_t flags;
        } __attribute__((packed,aligned(4)));

// flags in the above
#define BF_GPS_PRIMARY              0x1
#define BF_GPS_SECONDARY            0x2
#define BF_COM_PRIMARY              0x4
#define BF_COM_SECONDARY            0x8
#define BF_XPNDR_STATUS             0x10
#define BF_INTCM_STATUS             0x20
#define BF_OTHER_STATUS             0x40
#define BF_FLAPS_SOURCE             0x80

// b8:15 are a copy of the flap motor controller fault codes

struct efi_frame {
    struct efi_header header;
    union {
      struct efi_currents currents;
      struct efi_stats stats;
      struct flaps_stats fstats;
      struct efi_temps temps;
      struct log_data log;
      struct pump_start_record fps;
      struct config_data config;
      struct flap_settings flap_pot;
      struct bpower_data bpower;
      vib_payload_header_t vib_payload_header;
      vib_status_v1_t vib_status;
      vib_config_snapshot_v1_t vib_config;
      vib_accel_block_v1_hdr_t vib_accel;
      vib_tach_event_v1_t vib_tach_event;
      vib_tach_debug_v1_t vib_tach_debug;
      uint8_t buffer[MAX_EFI_FRAME_DATA_SIZE];
    };
    uint32_t crc32;             // Not really correct - it's at the end of the content, not at the end of the frame structure
  } __attribute__((packed,aligned(8)));

// Non-precise attempts at deciding fault type.
#define FAULT_DEVICE            1
#define FAULT_WIRING            2

// Fault value for display data
#define MO_FAULT                255

// How data for plots is handed around
typedef struct plotdata {
  uint16_t points;              // How many points of data there are
  uint8_t fault;                // Non-zero for a fault code
  uint8_t no_update;            // No update since last time, do not redraw plot
  uint16_t duration;            // Pulse duration in usec, or run time in seconds
  uint16_t current_ma;          // Peak or average in b13:0, b15=1 means this is an average
  union {
    uint16_t derived_rpm;       // For coilpacks, derived RPM from pulse frequency
    uint16_t surge_ma;          // For fuelpumps, peak current from last startup
    };
  uint16_t ripple_ma;           // For fuelpumps, peak-peak ripple current
  uint8_t *plot_data;
  } __attribute__((packed,aligned(4))) plot_data_t;

#define PD_CURRENT_MASK         0x3fff
#define PD_CURRENT_AVERAGE      0x8000

#define MAX_PLOT_POINTS 448

plot_data_t *efi_get_plot_data(uint8_t, uint32_t, uint32_t, uint32_t);
void efi_get_temps_update(void);
void do_efi_write_latch_settings(uint16_t);
void efi_tps_reset(int);
int mo_usleep(unsigned int);

// Battery backup response structure and MACRO's
typedef struct bat_response {
  uint16_t battery_voltage_adc_status;
  uint16_t right_voltage_adc;
  uint16_t battery_vadc1;
  uint8_t crc32[4];
  } __attribute__((packed,aligned(4))) bat_response_t;

#define BAT_RELAY_ON_RIGHT(r)   (((r)->battery_voltage_adc_status & 0x2000) ? 1 : 0)
#define BAT_BATTERY_UP(r)       (((r)->battery_voltage_adc_status & 0x4000) ? 1 : 0)
#define BAT_RIGHT_UP(r)         (((r)->battery_voltage_adc_status & 0x8000) ? 1 : 0)
#define BAT_VOLTAGE(adc)        ((double)((adc) & 0xfff) * (((10000.0 + 90900.0) / 10000.0) / 4095.0 * 3.3))
#define FLAPS_VOLTAGE(adc)      ((double)((adc) & 0xfff) * (((10000.0 + 90900.0) / 10000.0) / 4095.0 * 3.3))
#define FLAPS_CURRENT(adc)      ((double)((adc) & 0xfff) / 4095.0 * 3.3 * 5.0 / 1.5)

// A guess at the threshold average coilpack current, in mA, below which the alternator might be off.
// Used to do the "ALT" warnings in the summary display
// Right is smaller because right alternator generates nothing at low RPM, so we give it a more
// forgiving threshold to avoid false indications.
#define LEFT_COILPACK_ALTERNATOR_THRESHOLD 6700
#define RIGHT_COILPACK_ALTERNATOR_THRESHOLD 6400

// HF Radio interfaces
typedef struct hf_data {
  uint32_t vfoA;                // VFO A frequency, Hz
  uint32_t vfoB;                // VFO B frequency, Hz
  uint32_t flags;
  float pa_current;
  float pa_power_dissipation;
  float input_power;
  float forward_power;
  float reflected_power;
  float swr;
  uint16_t s_meter;
  } __attribute__((packed,aligned(4))) hf_data_t;

// Bit fields for "flags"
#define HF_TRANSMITTING         0x1
#define HF_USB                  0x2
#define HF_PRESENT              0x80000000

// Log record header
// An un-deflated log entry
struct log_header {
  uint32_t start_sample;
  uint16_t nbytes;
  } __attribute__((packed,aligned(1)));

#define LOG_TYPE_NEWVALUE   -128
#define LOG_TYPE_REPEAT     -127
#define LOG_TYPE_FAULT      -126
#define LOG_TYPE_RESERVED   LOG_TYPE_FAULT
#define LOG_MAX_DIFF        127

#define LOG_ITEMS           (CYLINDERS+COILPACKS)

// Vibration sensor OP_VIB wrapper opcodes (in opdata1[7:0])
#define VIB_OP_ECHO         0
#define VIB_OP_RESET        1                       /* Initialize everything */
#define VIB_OP_ACCEL        2                       /* Select/Deselect accelerometers */
#define VIB_OP_GET_EVENT    3                       /* Get next event */
#define VIB_OP_GET_STATUS   4                       /* Get status */
#define VIB_OP_GET_CONFIG   5                       /* Get config details */
#define VIB_OP_DEBUG        6                       /* TBD */

// Structure for keeping track of remote module response times
typedef struct {
  uint8_t opcode;           // What the opcode is
  const char *name;         // The name of that operation
  uint32_t num;             // How many of these operations have occurred
  double min, max, tot;     // min, max and total response time, in msec
} response_counts_t;        // (average = total / num)


uint8_t hf_operation(hf_data_t *);
uint8_t hf_get_data(hf_data_t *);
void hf_tune(int on);
void dbprintf(const char *, ...);




#endif
