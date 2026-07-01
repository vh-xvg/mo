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
//
// mo - "Matrix Orbital", named after the display manufacturer.
//      A dumb name, but it stuck
//
// Things needed under arm:
//      sudo apt-get install libconfig-dev espeak 
//
// Things needed under x86_64:
//      sudo dnf -y install libconfig-devel espeak
//
#ifdef ARM
#include <lgpio.h>
#ifdef USE_HAT2
  #include <iio.h>
#endif
#endif
#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <stdatomic.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/select.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <sched.h>
#include <sys/mman.h>
#include <math.h>
#include <getopt.h>
#include <ctype.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <syslog.h>
#include <libconfig.h>
#include <spawn.h>
#include <sys/wait.h>
//#include <fftw3.h>

#define __USE_GNU
#include <pthread.h>

#include <efi_protocol.h>
#include <eve_xvg.h>
#include "tts.h"

#include "lidar.h"
#ifdef USE_HAT2
  #include "ad5592.h"
#endif

#include "vibsense.h"
#include "vibsense_fake_window.h"
#include "vibsense_live.h"
#include "eve_vibration_display.h"

#ifdef USE_LGPIO
#define GPIO16    16
#define GPIO17    17
#define GPIO25    25
#define GPIO27    27

#define LEFT_BATTERY_ALARM_PIN      GPIO17
#define RIGHT_BATTERY_ALARM_PIN     GPIO27

#ifdef USE_HAT2
  #define LEFT_DOOR_SWITCH            21      /* GPIO21, DIN0 */
  #define BAGGAGE_DOOR_SWITCH         19      /* GPIO90, DIN1 */
  #define RIGHT_DOOR_SWITCH           20      /* GPIO20, DIN4 */
  #define PRIMARY_TACHO_GPIO           7      /*  GPIO7, DIN2 */
  #define SECONDARY_TACHO_GPIO        25      /* GPIO25, DIN3 */

  #define ONEWIRE_GPIO                 6      /* GPIO6 */
  #define ENABLE_START_GPIO           18      /* GPIO18, DOUT0 */
  #define ENABLE_O2_SENSOR_GPIO       15      /* GPIO15, DOUT1 */
  // Unused at present:
  // DOUT2 is GPIO14, DOUT3 is GPIO24
  // DIN5 is GPIO26

  #define AIR_FUEL_RATIO               0      /* AIN0 */
#else
  #define BAGGAGE_DOOR_SWITCH         GPIO25
#endif
#endif

#ifdef ARM
#define INCLUDE_BAT
#define INCLUDE_FLAPS
#define INCLUDE_HF
#define REAL_DOORS 1
#else
//#define REAL_DOORS 0
#define INCLUDE_FLAPS
#define INCLUDE_BAT
#define INCLUDE_HF
#endif

//#define INCLUDE_LIDAR
#define INCLUDE_EFI
#define INCLUDE_LOGGING
#define INCLUDE_ETX

#define POLL_INTERVAL_MS    40

#ifdef USE_LGPIO
  int gpio_handle = -1;
#endif

extern volatile sig_atomic_t  eve_do_screenshot;
extern char *eve_do_screenshot_filename;
static void mo_load_config(void);
static void mo_setup_display_loop(void);
#ifdef INCLUDE_FLAPS
static void mo_write_config(char *);
static void mo_update_flap_config_settings(void);
#endif

static void check_for_crash_recovery(void);
const char *restart_state_filename = "restart_state";

char initial_splash[256] = { "assets/images/default_splash.jpg" };
char config_writeback[256] = { "config/new_mo.config" };

struct pilot_data *pilots = NULL;
int sec_num_pilots = 0;
int pilot_in_command = -1;

#ifdef INCLUDE_FLAPS
pthread_mutex_t flaps_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t flaps_req_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t flaps_done_cond = PTHREAD_COND_INITIALIZER;

#define ACTIVATE_FLAPS_THREAD(x) \
  do { \
        pthread_cond_signal(&flaps_req_cond); \
        while ( (x) && !flaps_thread_abort) { \
          int rc = pthread_cond_wait(&flaps_done_cond, &flaps_lock); \
          if (rc != 0) { \
            break; \
          } \
        } \
        pthread_mutex_unlock(&flaps_lock); \
     } while(0)

static struct efi_frame flaps_rx_frame;
struct flap_settings flaps = { DF_REFLEX, DF_ZERO, DF_HALF, DF_FULL, DF_DEADBAND};

struct bpower_data *flaps_data = (struct bpower_data *) &flaps_rx_frame.bpower;
int flaps_data_valid = 0;

uint8_t flaps_config_good = 0;          // Set if all parameters present in config file
uint8_t flaps_calibrated = 0;           // Set if flaps position calibrations have been written to flaps module
uint8_t flaps_get_data = 0;             // Get updated flap data
uint32_t flaps_put_action = 0;           // Performs an OP_FLAP
uint8_t flaps_put_settings = 0;         // Set flap positions
uint8_t flaps_mode_continuous = 0;      // Flaps are in continuous mode, soft buttons disabled
uint8_t flaps_mode_airspeed = 1;        // Airspeed checks active
uint8_t flaps_sensor_reverse = 0;       // Flaps sensor is reversed
#endif

pthread_mutex_t vib_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t vib_wait = PTHREAD_COND_INITIALIZER;

static struct efi_frame vib_rx_frame;
static struct efi_frame vib_tx_frame;

static int vib_fd = -1;                 // -ve means no vib device present
uint8_t vib_noise = 0;
uint8_t vib_do_reset = 1;               // Init to doing a reset first up
uint8_t vib_running = 0;
uint8_t vib_data_valid = 0;
uint8_t vib_write_snapshot = 0;         // Write a 200 revolution snapshot log

uint8_t vib_live_rotations = VIB_LIVE_ROTATIONS;
uint8_t vib_snapshot_rotations = VIB_SNAPSHOT_ROTATIONS;

static uint8_t rev_adjust_mode = 0;

vibsense_context_t *ctx;
vibsense_window_t vib_window1;
vibsense_window_t vib_window2;
uint32_t vib_window_updated = 0;
uint32_t last_vib_window_updated = 0;

int vib_rotations = VIB_LIVE_ROTATIONS;
uint8_t vib_get_next_event = 0;

pthread_mutex_t efi_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t efi_req_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t efi_done_cond = PTHREAD_COND_INITIALIZER;

static struct efi_frame rx_frame;
static struct efi_frame tx_frame;

struct efi_options options = {0};

uint32_t efi_get_currents = 0;
uint32_t efi_get_current_points = 0;
uint8_t  efi_get_stats = 0;
uint8_t  efi_get_temps = 0;
uint8_t  efi_get_logs = 0;
uint8_t  efi_get_psdata = 0;

int local_fake_data_ok = 0;
int fake_pump_is_on[FUELPUMPS] = {1,0};

int efi_fake_data_ok = 0;
int efi_diag_data_ok = 0;
int efi_real_with_fake_engine = 0;
int fake_engine_rpm = 0;
int simulate_engine_rpm = 2300;
int fake_door_countdown = 0;
static int door_override = 0;
static int door_override_count = 3;
int allow_open_doors = 0;
int do_lcdoff = 0;
int run_lidar = 0;
int poll_junction_temperatures = 0;
int injector_divisor = 0;                       // These scale factors were determined emperically
int coilpack_divisor = 0;                       // ""
int do_logging = 0;
int enable_periodic_stats = 1;                  // Issue an O_STATS every half second
static char efi_write_config_arg[256] = {0};
static int efi_write_config = 0;
static int efi_read_config = 0;
struct config_data efi_board_config = {0};      // Config data from the board we're talking to
static int getout_taps = 0;
int tft_backlight = 100;
_Atomic uint8_t tft_present = 0;

char *mo_software_version = "1.0(a)";

_Atomic uint32_t primary_rpm = 0;                   // As measured from SDSEFI pulses
_Atomic uint32_t secondary_rpm = 0;

_Atomic uint32_t engine_rpm = 0;
_Atomic int engine_is_running = 0;

uint16_t saved_average_current[NUM_CURRENTS] = {0};
uint16_t average_current_threshold[2] = { LEFT_COILPACK_ALTERNATOR_THRESHOLD,
                                          RIGHT_COILPACK_ALTERNATOR_THRESHOLD };

uint8_t implied_ecu_failure = 0;            // 0x1 == primary ECU, 0x2 == secondary ECU

uint32_t msec_tick = 0;                     // msec ticks since invocation.
static uint32_t last_msec_tick = 0;
                                            // NOT guaranteed to always increment by 1
uint32_t msec_when_engine_started = 0;

_Atomic float air_fuel_ratio = 14.7;
float air_fuel_ratio_correction = 0.0;      // Corrects for sensor error, determined by peak EGT testing
_Atomic uint8_t o2_sensor_power_is_on = 0;  // 1 if O2 sensor is powered up
_Atomic uint8_t o2_sensor_not_ready = 1;    // So it starts out that way
_Atomic uint8_t o2_sensor_error = 0;

uint8_t bat_noise = 0;
uint8_t efi_noise = 0;
uint8_t flaps_noise = 0;
uint8_t log_noise = 0;
uint8_t lidar_noise = 0;
uint8_t lidar_testmode = 0;                 // Use test mode settings for Lidar (to test in a room)
int     lidar_enabled = 0;
int     lidar_landing_runon_seconds = 5;
int     log_time = 0;                       // Log for a duration, for command line testing
int     log_snapshot_time = 0;
int     log_above_rpm = 0;
int     log_above_rpm_post_seconds = 0;
uint8_t log_state = 0;
uint8_t log_while_running = 0;              // Logs as long as engine is running
_Atomic uint8_t log_start = 0;
_Atomic uint8_t log_finish = 0;
uint32_t log_finish_sample_count = 0;
_Atomic static uint8_t logging_active = 0;          // 1 if logging is active
uint8_t hf_noise = 0;
#ifdef USE_HAT2
uint8_t hat2_noise = 0;
#else
uint8_t monarco_noise = 0;
#endif
uint8_t ac_noise = 0;
static int do_lcd_blank = 0;

static uint8_t efi_left_coil_disabled;
static uint8_t efi_right_coil_disabled;
static uint8_t efi_injectors_disabled;

static dstate_t initial_screen = DS_INITIAL_SCREEN;

// Variables we get from the A/C controller

aircon_data_t aircon_data = {0};

static int efi_fd = -1;                                    // -ve means no efi device present
#ifdef INCLUDE_FLAPS
_Atomic static int flaps_fd = -1;                          // -ve means no efi device present
_Atomic static int flaps_active = 0;
#endif

pthread_mutex_t efi_stats_lock = PTHREAD_MUTEX_INITIALIZER;
efi_stats_t efi_stats = {0};
struct efi_temps efi_temps = {0};
struct fp_temps fp_temps = {0};
struct gui_temps gui_temps = {0};

_Atomic uint8_t fuelpump_on[FUELPUMPS] = {0};
_Atomic static uint8_t fuelpump_turned_on[FUELPUMPS] = {0};
static struct timespec ts_pumpstart[FUELPUMPS] = {0};
static struct timespec ts_pumpstart_realtime[FUELPUMPS] = {0};
static uint8_t fuelpump_data_sequence[FUELPUMPS];

pthread_mutex_t bat_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t bat_wait = PTHREAD_COND_INITIALIZER;

pthread_mutex_t etx_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t etx_wait = PTHREAD_COND_INITIALIZER;
uint32_t etx_get_status = 0;

pthread_mutex_t hf_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t hf_wait = PTHREAD_COND_INITIALIZER;
uint32_t hf_get_status = 0;

pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t log_wait = PTHREAD_COND_INITIALIZER;

// The status header recovered from OP_CURRENTS frames
struct efi_currents_header efi_currents_sts = {0};

uint8_t real_injector_data[CYLINDERS][MAX_INJECTOR_POINTS];
uint8_t real_coilpack_data[COILPACKS][MAX_COILPUMP_POINTS];
uint8_t real_fuelpump_data[FUELPUMPS][MAX_COILPUMP_POINTS];

uint8_t *all_real_data[] = {
  &real_injector_data[0][0],
  &real_injector_data[1][0],
  &real_injector_data[2][0],
  &real_injector_data[3][0],
  &real_injector_data[4][0],
  &real_injector_data[5][0],
  &real_coilpack_data[0][0],
  &real_coilpack_data[1][0],
  &real_fuelpump_data[0][0],
  &real_fuelpump_data[1][0] };

static struct pump_start_record fuelpump_start_current_plot[FUELPUMPS];
static uint8_t fscp_valid[FUELPUMPS] = {0};

int verbose = 0;
int debug_level = 0;
int baudrate = 0;           // Default is 115200
#ifdef OLD_HEARTBEAT_STUFF
int enable_heartbeat = 0;
#endif
int something_unlocked = 1;
int current_left, current_right, current_baggage;
uint8_t baggage_enabled = 0;                            // Enable to allow baggage door switch
uint16_t latch_config = 0;

void do_efi_coilpack_test(uint8_t, uint8_t);
void efi_coilpack_test(uint8_t, uint8_t);
void do_efi_injector_test(uint8_t, uint8_t);
void efi_injector_test(uint8_t, uint8_t);
// Door gpio pin status
#define UNLOCKED 0
#define LOCKED 1
#define BLANK 2

void *tick_thread(void*);
void *lidar_thread(void*);
void *efi_thread(void*);
void *bat_thread(void*);
void *etx_thread(void*);
void *hf_thread(void*);


#define ACTIVATE_EFI_THREAD(x) \
  do { \
        pthread_cond_signal(&efi_req_cond); \
        while ( (x) && !efi_thread_abort) { \
          int rc = pthread_cond_wait(&efi_done_cond, &efi_lock); \
          if (rc != 0) { \
            break; \
          } \
        } \
        pthread_mutex_unlock(&efi_lock); \
     } while(0)



volatile sig_atomic_t lidar_thread_abort = 0;
volatile sig_atomic_t efi_thread_abort = 0;
volatile sig_atomic_t vib_thread_abort = 0;
volatile sig_atomic_t flaps_thread_abort = 0;
volatile sig_atomic_t etx_thread_abort = 0;
volatile sig_atomic_t hf_thread_abort = 0;
volatile sig_atomic_t tick_thread_abort = 0;

volatile sig_atomic_t mo_abort = 0;

void *ac_thread(void*);
int ac_thread_abort = 0;

#ifdef INCLUDE_LOGGING
void *log_thread(void*);
void log_pumpstart(int);
int log_thread_abort = 0;
int log_post_finish = 0;
static uint8_t delayed_log_pumpstart = 0;

// fd's for redirect
int logging_stdout = 0;
int logging_stderr = 0;
int saved_stdout = 0;
int saved_stderr = 0;
int stdout_redirected = 0;

static void stdout_redirect(char *dir)
{
  char buf[256];

  snprintf(buf, sizeof(buf), "%s/stdout.log", dir);
  logging_stdout = open(buf, O_RDWR|O_CREAT|O_APPEND, 0600);
  if (logging_stdout == -1)
    return;
  snprintf(buf, sizeof(buf), "%s/stderr.log", dir);
  logging_stderr = open(buf, O_RDWR|O_CREAT|O_APPEND, 0600);
  if (logging_stderr == -1) {
    close(logging_stdout);
    return;
    }

  saved_stdout = dup(fileno(stdout));
  saved_stderr = dup(fileno(stderr));

  if (dup2(logging_stdout, fileno(stdout)) == -1) {
    close(saved_stdout);
    close(saved_stderr);
    return;
    }
  if (dup2(logging_stderr, fileno(stderr)) == -1) {
    dup2(saved_stdout, fileno(stdout));
    close(saved_stdout);
    close(saved_stderr);
    close(logging_stdout);
    return;
    }
  
  stdout_redirected = 1;
}


static void stdout_revert()
{
  if (!stdout_redirected)
    return;

  fflush(stdout);
  close(logging_stdout);
  fflush(stderr);
  close(logging_stderr);

  dup2(saved_stdout, fileno(stdout));
  dup2(saved_stderr, fileno(stderr));

  close(saved_stdout);
  close(saved_stderr);

  stdout_redirected = 0;

  dbprintf("stdout_revert: back to normal\n");
}

#endif

// Write the full stated amount, but return if there are fatal errors

static int write_full(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    size_t done = 0;

    while (done < len) {
        ssize_t n = write(fd, p + done, len - done);

        if (n > 0) {
            done += (size_t)n;
            continue;
        }

        if (n < 0 && errno == EINTR)
            continue;

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            continue;   /* harmless for blocking fds; useful if flags change */

        return -1;
    }

    return 0;
}

//
// Kludge up a thread safe usleep() that will work on both OS's
//
int mo_usleep(unsigned int usec)
{
  struct timespec req;
  struct timespec rem;
  int ret;

  req.tv_sec  = usec / 1000000U;
  req.tv_nsec = (long)(usec % 1000000U) * 1000L;

  for (;;) {
    ret = clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &rem);
    if (ret == EINTR) {
      req = rem;
      continue;
      }
    return(ret);
  }
}

static void do_efi_transaction(int, struct efi_header *, struct efi_frame *, int);
static void do_flaps_transaction(int, struct efi_header *, struct efi_frame *, int);
static void do_vib_transaction(int, struct efi_header *, struct efi_frame *, int);

void dbprintf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  struct timeval tv_now;
  struct tm t, *now = &t;
  int nb;

  gettimeofday(&tv_now, NULL);
  localtime_r(&tv_now.tv_sec, &t);

  nb = snprintf(buf, sizeof(buf), "%2d:%02d:%02d.%03ld  ", 
    now->tm_hour, now->tm_min, now->tm_sec, tv_now.tv_usec / 1000);

  va_start(ap, fmt);
  vsnprintf(&buf[nb], sizeof(buf) - nb, fmt, ap);
  va_end(ap);

  printf("%s", buf);
  return;
}


#if 0
// This has to track the associated enum array...
const static char *dsnames[] = {
  "DS_DISCONNECTED",
  "DS_IDLE",
  "DS_INITIAL_SCREEN",
  "DS_INITIAL_BATTERY_BACKUP",
  "DS_SET_BACKLIGHT",
  "DS_SECURITY",
  "DS_DOORS",
  "DS_START",
  "DS_CHECKLISTS",
  "DS_SUMMARY",
  "DS_COILPUMPS",
  "DS_INJECTORS",
  "DS_BATTERY_BACKUP",
  "DS_VIBRATION",
  "DS_AIRCON",
  "DS_O2_SENSOR",
  "DS_COILPACK_DETAIL",
  "DS_FUELPUMP_DETAIL",
  "DS_INJECTOR_DETAIL",
  "DS_HFRADIO",
  "DS_DIAGNOSTIC",
  "DS_STAT_COMMS",
  "DS_STAT_EFI_RESPONSE",
  "DS_STAT_INJECTORS",
  "DS_STAT_COILPUMPS",
  "DS_STAT_JUNCTION_TEMPS",
  "DS_FUELPUMP_LASTSTART",
  "DS_ETX_FAULT",
  "DS_CONFIGURE",
  "DS_CONFIGURE_FLAPS",
  "DS_NO_CONFIGURATION"
  };
#endif

//#define dbprintf(...) debugPrintf(__VA_ARGS__)

// Set an absolute timeout suitable for pthread_cond_timedwait().
//
// These condition variables are created with PTHREAD_COND_INITIALIZER, so
// pthread_cond_timedwait() interprets abstime using CLOCK_REALTIME.  Use the
// same clock here.  Using CLOCK_MONOTONIC with default condition variables can
// make timed waits expire immediately or at the wrong time.
static inline void set_timeout(struct timespec *spec, long useconds) {
  int64_t nanos;
  if (clock_gettime(CLOCK_REALTIME, spec) < 0) {
    perror("set_timeout: ");
    return;
  }
  nanos = (int64_t) spec->tv_nsec + (int64_t)useconds * 1000LL;
  spec->tv_sec += (nanos / 1000000000LL);
  spec->tv_nsec = (nanos % 1000000000LL);
}

#ifdef USE_HAT2

// Note: All digital inputs are inverted in hardware.
#define GRAB_DOORS \
        current_left = (lgGpioRead(gpio_handle, LEFT_DOOR_SWITCH)) ? UNLOCKED : LOCKED; \
        current_right = (lgGpioRead(gpio_handle, RIGHT_DOOR_SWITCH)) ? UNLOCKED : LOCKED; \
        current_baggage = (baggage_enabled && (lgGpioRead(gpio_handle, BAGGAGE_DOOR_SWITCH))) ? UNLOCKED : LOCKED;
        // XXX fix above for baggage door once sensors in place ..........^^

#define DISABLE_START               lgGpioWrite(gpio_handle, ENABLE_START_GPIO, 0)
#define ENABLE_START                lgGpioWrite(gpio_handle, ENABLE_START_GPIO, 1)
#define DISABLE_O2_SENSOR_POWER     lgGpioWrite(gpio_handle, ENABLE_O2_SENSOR_GPIO, 0)
#define ENABLE_O2_SENSOR_POWER      lgGpioWrite(gpio_handle, ENABLE_O2_SENSOR_GPIO, 1)
#define O2_SENSOR_POWER_ON          lgGpioRead(gpio_handle, ENABLE_O2_SENSOR_GPIO)

void *hat2_thread(void *);
int hat2_thread_abort = 0;

#else
  // Old MONARCO stuff is gone now 
  // Make the defines exist for compilation under Linux
  #define DISABLE_START
  #define ENABLE_START
  #define DISABLE_O2_SENSOR_POWER
  #define ENABLE_O2_SENSOR_POWER
  #define O2_SENSOR_POWER_ON        0
#endif

#define DIMENSION(a) (sizeof(a)/sizeof(a[0]))

#define BAUDRATE B115200
#define HF_BAUDRATE B38400

#define EFI0_DEFAULT "/dev/ttyUSB-efi"              /* USB virtual com port, symlink in udev rules */
#define VIB0_DEFAULT "/dev/ttyUSB-vib"              /* USB virtual com port, symlink in udev rules */

#ifdef ARM
  #if (PIMODEL == 5)
    #define BAT0_DEFAULT "/dev/ttyAMA2"               /* This is uart2 */
    #define FLAPS0_DEFAULT "/dev/ttyAMA2"             /* Flap and redundant power controller (Replaces BAT) */
    #define HF0_DEFAULT "/dev/ttyAMA4"                /* This us uart4 */
  #else
    #define BAT0_DEFAULT "/dev/ttyAMA1"               /* This is uart3 */
    #define FLAPS0_DEFAULT "/dev/ttyAMA1"             /* Flap and redundant power controller (Replaces BAT) */
    #define HF0_DEFAULT "/dev/ttyAMA2"                /* This is uart5 */
  #endif
#else
  #define FLAPS0_DEFAULT "/dev/ttyUSB0"               /* Flap and redundant power controller */
  #define HF0_DEFAULT "/dev/ttyUSB1"                  /* Just so it exists */
  #define BAT0_DEFAULT "/dev/ttyUSB2"
#endif

#define DEVSIZE 256

char efi_device[DEVSIZE] = EFI0_DEFAULT;
#ifdef INCLUDE_FLAPS
char flaps_device[DEVSIZE] = FLAPS0_DEFAULT;
#endif
#ifdef INCLUDE_BAT
char bat_device[DEVSIZE] = BAT0_DEFAULT;
#endif
char hf_device[DEVSIZE] = HF0_DEFAULT;
char vib_device[DEVSIZE] = VIB0_DEFAULT;

#ifdef USE_HAT2

static _Atomic uint32_t tacho_count_primary, tacho_count_secondary;     // Incremented by gpio_events

// We settle on a measurement every 50 msec
const uint64_t interval_ns = (uint64_t) 50 * 1e6; 		// 50 msec period

// Use these externally to do a DAC operation
extern uint8_t write_dac[2];
extern uint16_t dac_data[2];

void hat2_loop(uint32_t tick)
{
  static uint32_t tacho_cnt_primary, tacho_cnt_secondary;
  static uint32_t last_tacho_cnt_primary, last_tacho_cnt_secondary;
  static uint32_t tacho_delta_primary, tacho_delta_secondary;
  static uint32_t last_tick;

  GRAB_DOORS;
  something_unlocked = (current_left == UNLOCKED) || (current_right == UNLOCKED) || (current_baggage == UNLOCKED);

  #define DAC_LIMIT 1024
  #define DAC_INCREMENT 64

  static uint8_t do_dac_diagnostic = 0;
  static uint8_t dac_diagnostic_init = 1;
  if (do_dac_diagnostic) {
    if (dac_diagnostic_init == 1) {
      dac_data[1] = DAC_LIMIT/2;
      dac_diagnostic_init = 0;
    }
    if (write_dac[0] == 0) {
      dac_data[0] += DAC_INCREMENT;
      if (dac_data[0] > DAC_LIMIT)
        dac_data[0] = 0;
      write_dac[0] = 1;
      }
    if (write_dac[1] == 0) {
      dac_data[1] += DAC_INCREMENT;
      if (dac_data[1] > DAC_LIMIT)
        dac_data[1] = 0;
      write_dac[1] = 1;
      }
  }

  ad5592_main();

  // Determine if the engine is running. Tacho pulses are 3 per revolution, so for minimum RPM of (say) 500, that's
  // 1500 counts per minute. If we see more than 180 rpm (3 blades per second) on EITHER tacho input, we
  // disable startEnable

  if (tick > last_tick && (tick % (1000 / (int)(interval_ns/1e6))) == 0) {
    last_tick = tick;
    tacho_cnt_primary = (fake_engine_rpm) ? fake_engine_rpm * 3 : tacho_count_primary;
    tacho_cnt_secondary = (fake_engine_rpm)? fake_engine_rpm * 3 : tacho_count_secondary;

    if (fake_engine_rpm) {
      primary_rpm = fake_engine_rpm;
      secondary_rpm = fake_engine_rpm;
      engine_rpm = fake_engine_rpm;
    } else {
      if (tacho_cnt_primary < last_tacho_cnt_primary)
        tacho_delta_primary = tacho_cnt_primary + ((uint32_t)(0xffffffff) - last_tacho_cnt_primary) + 1;     // Wrapped
      else
        tacho_delta_primary = tacho_cnt_primary - last_tacho_cnt_primary;

      if (tacho_cnt_secondary < last_tacho_cnt_secondary)
        tacho_delta_secondary = tacho_cnt_secondary + ((uint32_t)(0xffffffff) - last_tacho_cnt_secondary) + 1;     // Wrapped
      else
        tacho_delta_secondary = tacho_cnt_secondary - last_tacho_cnt_secondary;

      primary_rpm = tacho_delta_primary * 60 / 3;
      secondary_rpm = tacho_delta_secondary * 60 / 3;

      //dbprintf("pri rpm %d, sec rpm %d\n", primary_rpm, secondary_rpm);

      if (tick > 225) {
        // Prevent false rpm indications from system startup
        // 225 ticks is 5 seconds
        if (primary_rpm >= 180 && secondary_rpm >= 180)
          engine_rpm = (primary_rpm + secondary_rpm) / 2;
        else if (primary_rpm >= 180)
          engine_rpm = primary_rpm;
        else
          engine_rpm = secondary_rpm;
      }
    }

    if (engine_rpm >= 180) {
      if (!engine_is_running) {
        dbprintf("Engine started @ %d msec, rpm %d from tacho counts %d/%d\n", msec_tick, engine_rpm, tacho_cnt_primary, tacho_cnt_secondary);
        msec_when_engine_started = msec_tick;
        }
      engine_is_running = 1;
      }
    else {
      if (engine_is_running) {
        dbprintf("Engine stopped, rpm %d from tacho counts %d/%d\n", engine_rpm, tacho_cnt_primary, tacho_cnt_secondary);
        ENABLE_START;                                               // Allow in-air restarts
        }
      engine_is_running = 0;
      }

    last_tacho_cnt_primary = tacho_cnt_primary;
    last_tacho_cnt_secondary = tacho_cnt_secondary;

    if (hat2_noise && ((tick % 300) == 0)) {
      dbprintf("hat2: Tacho's are %d and %d rpm from delta's %d and %d\n", primary_rpm, secondary_rpm, tacho_delta_primary, tacho_delta_secondary);
    }
  }
}



void *hat2_thread(void *marg)
{
  struct timespec ts;
  static struct timespec ts_start;

  uint32_t tick;
  uint8_t sts;
  uint64_t t;
  int loop = 0;

  dbprintf("hat2_thread: started\n");

  // Initialize

  // Entry time
  if ((sts = clock_gettime(CLOCK_MONOTONIC, &ts)) < 0) {
    fprintf(stderr, "Problem with clock_gettime(), returned %d (%s)\n", sts, strerror(sts));
    return 0;
  }

  loop = 0;
  tick = 0;
  for (hat2_thread_abort = 0; !hat2_thread_abort; ) {
    tick++;
    // Figure out next time to wait 'till, based on sample interval
    t = interval_ns;
    while (t >= (uint64_t) 1e9) {
      ts.tv_sec++;
      t -= 1e9;
    }
    ts.tv_nsec += (long)t;
    if (ts.tv_nsec >= 1e9) {
      ts.tv_sec++;
      ts.tv_nsec -= 1e9;
    }

    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    // XXX Do scan here

    hat2_loop(tick);

#if 0
        struct timespec ts_finish;
        clock_gettime(CLOCK_MONOTONIC, &ts_finish);                  // Derive measurement time
        long seconds = ts_finish.tv_sec - ts_start.tv_sec;
        long nanoseconds = ts_finish.tv_nsec - ts_start.tv_nsec;
        double hat2_elapsed_msec = ((double)seconds)*1e3 + ((double)nanoseconds)*1e-6;

        if (hat2_noise)
          dbprintf("hat2_elapsed_msec is %.2f\n", hat2_elapsed_msec);
#endif

    //printf("hat2: Calling clock_nanosleep with tv_sec %ld tv_nsec %ld\n", ts.tv_sec, ts.tv_nsec);
    sts = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    if (sts) {
      fprintf(stderr, "hat2: Problem with clock_nanosleep(), returned %d (%s)\n", sts, strerror(sts));
      fprintf(stderr, "hat2: ts.tv_sec %ld ts.tv_nsec %ld\n", ts.tv_sec, ts.tv_nsec);
      return 0;
    }
    loop++;
  }

  dbprintf("hat2_thread: Abort: Setting DAC outputs to zero\n");
  write_dac[0] = write_dac[1] = 1;
  dac_data[0] = dac_data[1] = 0;

  while (write_dac[0] || write_dac[1])
    ad5592_main();

  dbprintf("hat2_thread: Done\n");

  return((void *) NULL);

}

#endif


int lcd_width = 480;
int lcd_height = 128;

int do_std_screenshots = 0;
int do_demand_screenshot = 0;
int print_time = 0;

uint8_t debug_comms = 0;

////////////////////////////////////////////////////////////////////////////////
// Local fake data generator
////////////////////////////////////////////////////////////////////////////////

// Fake injector baseline data, AMPS * 100
// 72 points across 12 msec. OR 144 points, OR 288 points
static uint8_t fake_injector_data[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  5, 15, 26, 37, 45, 42, 40, 42, 43, 42, 45, 48, 55, 59, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62,
 61, 60, 62, 61, 60, 60, 61, 62, 63, 60, 59, 59, 60, 59, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62,
 61, 60, 62, 61, 60, 60, 61, 62, 63, 60, 59, 59, 60, 59, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62,
 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62,
 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62,
 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62,
 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 53, 45, 37, 30, 25,
 15,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};
static int fake_injector_duration = 18500;

static uint8_t noisy_injector_data[sizeof(fake_injector_data)];

static uint8_t fake_coilpack_data_dc_offset[MAX_PLOT_POINTS];
static uint8_t fake_coilpack_data[MAX_PLOT_POINTS];
static uint8_t noisy_coilpack_data[MAX_PLOT_POINTS];

static uint8_t fake_fuelpump_data[MAX_PLOT_POINTS];
static uint8_t noisy_fuelpump_data[MAX_PLOT_POINTS];

int simulate_injector_fault = 0;
int simulate_left_coilpack_fault = 0;
int simulate_left_fuelpump_fault = 0;
int simulate_excessive_dwell_time = 0;
int simulate_dc_leakage_coilpack_left = 0;
int simulate_zero_injector_current = 0;
int simulate_primary_ecu_failure = 0;
int simulate_secondary_ecu_failure = 0;

uint8_t *mo_make_fake_data(int type, int points, int device_index)
{
  int i, j, skip, limit;
  uint8_t *p, *q, *r;
  uint8_t rmask;
  static int initialized = 0;
  static int last_points = -1;
  int coil_peak = 80;
  int coil_max;
  int low = points/4;
  int high = 3 * low;

  if (!initialized || (points != last_points)) {
    if (simulate_excessive_dwell_time)
      coil_max = coil_peak * 0.75;
    else
      coil_max = coil_peak;

    //printf("points %d, last_points %d ... %ld\n", points, last_points, sizeof(fake_injector_data));
    for (i = 0; i < high; i++)
      fake_coilpack_data[i] = 0;
    for (i = low; i <= high; i++) {
      fake_coilpack_data[i] = (coil_peak * (i - low))/(high - low);
      if (fake_coilpack_data[i] > coil_max)
        fake_coilpack_data[i] = coil_max;
      }
    fake_coilpack_data[i++] = coil_max / 2;
    while (i < points)
      fake_coilpack_data[i++] = 0;

    for (i = 0; i < points; i++) {
      fake_fuelpump_data[i] = 52 + 4 * sin(((double)i * 2 * 3.14)/(double)14);
      }

    initialized = 1;
    last_points = points;
    }

  // Make the data noisy, so we can see the updates
  if (type == DT_INJECTOR) {
    p = &fake_injector_data[0];
    q = &noisy_injector_data[0];
    limit = sizeof(fake_injector_data);
    skip = (points == 36) ? 8 : (points == 72) ? 4 : (points == 144) ? 2 : 0;
    rmask = 0x1;
    memset(q, 0, sizeof(noisy_injector_data));
  } else if (type == DT_COILPACK) {
    if (device_index == 0 && simulate_dc_leakage_coilpack_left == 1) {
      int base = 30;
      int x;
      j = rand() % (MAX_PLOT_POINTS / 10);
      for (i = 0; i < MAX_PLOT_POINTS; i++) {
        x = fake_coilpack_data[j++];
        if (j >= MAX_PLOT_POINTS)
          j = 0;
        if (x > base)
          x *= 0.8;
        if (x < base)
          x = base;
        fake_coilpack_data_dc_offset[i] = x;
        }
      p = &fake_coilpack_data_dc_offset[0];
    } else {
      p = &fake_coilpack_data[0];
    }
    q = &noisy_coilpack_data[0];
    limit = points;
    skip = 0;
    rmask = 0x1;
  } else {
    p = &fake_fuelpump_data[0];
    q = &noisy_fuelpump_data[0];
    skip = 0;
    rmask = 0x3;
    if (fake_pump_is_on[device_index] == 1)
      limit = points;
    else {
      limit = 0;
      memset(q, 0, sizeof(noisy_fuelpump_data));
      }
  }

  r = q;
  if (type == DT_INJECTOR && simulate_zero_injector_current == 1)
    return(r);
  if (type == DT_INJECTOR && simulate_primary_ecu_failure == 1 && device_index < 3)
    return(r);
  if (type == DT_INJECTOR && simulate_secondary_ecu_failure == 1 && device_index > 2)
    return(r);
#ifdef MO_EXTRA_SIMULATION
  if (type == DT_COILPACK && simulate_primary_ecu_failure == 1 && device_index == 0) {
    memset(r, 0, sizeof(noisy_coilpack_data));
    return(r);
    }
#endif

  for (i = 0; i < limit; i++, p++) {
    if (skip && ((i%skip)!=0))
      continue;
    if (*p)
      *q++ = *p + (rand() & rmask);
    else
      *q++ = 0;
    }
  return(r);
}

//static uint8_t *mo_generate_local_fake_data(uint8_t, uint32_t, uint32_t);

static uint8_t zero_data[512];

static uint8_t *mo_generate_local_fake_data(uint8_t index, uint32_t mask, uint32_t points)
{
  int i, idx;

  for (i = 0, idx = 0; i < NUM_CURRENTS; i++)
    if (mask & (1 << i)) {
      if (idx == index)
        break;
      idx++;
      }
  if (i<CYLINDERS) {
    if (efi_injectors_disabled)
      return(zero_data);
    else
      return(mo_make_fake_data(DT_INJECTOR, points, i));
    }
  else if (i<(CYLINDERS+COILPACKS)) {
    if ((i == 6 && efi_left_coil_disabled) || (i == 7 && efi_right_coil_disabled))
      return(zero_data);
    else
      return(mo_make_fake_data(DT_COILPACK, points, i-CYLINDERS));
    }
  else {
    return(mo_make_fake_data(DT_FUELPUMP, points, i-CYLINDERS-COILPACKS));
    }
}

#define adcval(f)   (int)(((f)*1000.0*4095.0)/2900.0/5.0)

static void make_fake_pump_start_data()
{

  static int fps_settle = adcval(4.5);
  static int fps_inrush_current = adcval(11.2);

  struct pump_start_record *ps = &fuelpump_start_current_plot[0];
  int i, val;
  float r = 0.1;

  ps->data[0] = 0;
  ps->data[1] = fps_inrush_current / 2;
  ps->data[2] = fps_inrush_current;
  val = fps_inrush_current - fps_settle;
  for (i = 3; i < FPS_RECORDS; i++) {
    val = (int)(val * pow((1 - r),(i-3)));
    ps->data[i] = val;
    }
  fscp_valid[0] = 1;
}

uint8_t get_door_state(void)
{
  return(current_left | (current_right <<1) | (current_baggage <<2));
}


static void update_fuelpump_status(int left, int right)
{
  uint8_t last_left_fuelpump_on, last_right_fuelpump_on;

  last_left_fuelpump_on = fuelpump_on[0];
  last_right_fuelpump_on = fuelpump_on[1];
  if (!local_fake_data_ok) {
    fuelpump_on[0] = left;
    fuelpump_on[1] = right;
    }

  if (fuelpump_on[0] && !last_left_fuelpump_on) {
    fuelpump_turned_on[0] = 1;
    clock_gettime(CLOCK_MONOTONIC, &ts_pumpstart[0]);
    clock_gettime(CLOCK_REALTIME, &ts_pumpstart_realtime[0]);       // For logging
    //dbprintf("update_fuelpump_status: fuelpump 0 started\n");
    }
  if (fuelpump_on[1] && !last_right_fuelpump_on) {
    fuelpump_turned_on[1] = 1;
    clock_gettime(CLOCK_MONOTONIC, &ts_pumpstart[1]);
    clock_gettime(CLOCK_REALTIME, &ts_pumpstart_realtime[1]);       // For logging
    //dbprintf("update_fuelpump_status: fuelpump 1 started\n");
    }
}

plot_data_t pdata = {0};

static uint8_t no_update[NUM_CURRENTS] = {0};             // Count consecutive no-update events

// Beyond this count, zero the data
#define MAX_NO_UPDATE 3

static void do_update_status(int idx, plot_data_t *pd)
{
  int nup;

  nup = (efi_currents_sts.update_status & (0x1 << (idx))) ? 0 : 1;
  if (nup) {
    if (no_update[idx] >= MAX_NO_UPDATE) {
      pd->no_update = 1;
      // Zero out the data
      if (idx < CYLINDERS) {
        for (int k = 0; k < MAX_INJECTOR_POINTS; k++)
          real_injector_data[idx][k] = 0;
      } else if (idx < (CYLINDERS+COILPACKS)) {
        for (int k = 0; k < MAX_COILPUMP_POINTS; k++)
          real_coilpack_data[idx-CYLINDERS][k] = 0;
        } else {
        for (int k = 0; k < MAX_COILPUMP_POINTS; k++)
          real_fuelpump_data[idx-CYLINDERS-COILPACKS][k] = 0;
        }
      }
    else
      no_update[idx]++;
    }
  else {
    no_update[idx] = 0;
    pd->no_update = 0;
    }

  if (IS_INJECTOR(idx)) {
    pd->duration = efi_currents_sts.u[idx].injector.duration_usec;
    pd->current_ma = efi_currents_sts.u[idx].injector.current_ma;
  } else if (IS_COILPACK(idx)) {
    pd->duration = efi_currents_sts.u[idx].coil.dwell_time_usec;
    pd->current_ma = efi_currents_sts.u[idx].coil.current_ma;
  } else {
    pd->duration = efi_currents_sts.u[idx].pump.run_time_seconds;
    pd->current_ma = efi_currents_sts.u[idx].pump.current_ma;
    pd->surge_ma = efi_currents_sts.u[idx].pump.surge_current_ma;
  }
}

plot_data_t *efi_get_plot_data(uint8_t index, uint32_t mask, uint32_t injector_points, uint32_t coilpack_points)
{
  int i, idx;
  plot_data_t *pd = &pdata;
  static uint8_t fake_pump_started[2] = {0,0};
  static time_t fake_pump_start_time[2];
  struct timespec ts_now;

  int points = (index < 6) ? injector_points : coilpack_points;

  pd->points = points;

  if (local_fake_data_ok) {
    // No efi connection, generate fake data if that's OK.
    pd->plot_data = mo_generate_local_fake_data(index, mask, points);
#if 0
    // Fault injector #2, just to exercise the display
    if (simulate_injector_fault && ((((mask & 0x3f) == 0x3f) && index == 2) || ((mask & 0x3f)==0x4 && index == 0)))
      pd->fault = FAULT_WIRING;
    else if (simulate_left_coilpack_fault && ((((mask & 0x3c0) == 0x3c0) && index == 0) || ((mask & 0x3ff)==0x40 && index == 0)))
      pd->fault = FAULT_WIRING;
    else if (simulate_left_fuelpump_fault && ((((mask & 0x3c0) == 0x3c0) && index == 2) || ((mask & 0x3ff)==0x100 && index == 0)))
      pd->fault = FAULT_WIRING;
    else
      pd->fault = 0;
#endif

    // Fake up some other numbers
    for (i = 0, idx = 0; i < NUM_CURRENTS; i++) {
      if (mask & (1 << i)) {
        if (idx == index)
          break;
        idx++;
      }
    }
    if (pd->fault) {
      pd->duration = 0;
      pd->current_ma = 0;
    } else if (IS_INJECTOR(i)) {
      if (simulate_zero_injector_current == 1) {
        pd->duration = 0;
        pd->current_ma = 0;
      } else if (simulate_primary_ecu_failure == 1 && i < 3) {
        implied_ecu_failure = 0x1;
        pd->duration = 0;
        pd->current_ma = 0;
      } else if (simulate_secondary_ecu_failure == 1 && i > 2) {
        implied_ecu_failure = 0x2;
        pd->duration = 0;
        pd->current_ma = 0;
      } else {
        pd->duration = fake_injector_duration | (uint16_t)(rand()&0xff);
        pd->current_ma = 950 | (uint16_t)(rand()&0xff);
        }
    } else if (IS_COILPACK(i)) {
      if (IS_LEFT_COILPACK(i) && simulate_dc_leakage_coilpack_left == 1) {
        pd->duration = 0;
        pd->current_ma = PD_CURRENT_AVERAGE | 1800 | (uint16_t)(rand()&0x1ff);       // Average current
        pd->derived_rpm = 0;
      } else if (IS_LEFT_COILPACK(i) && simulate_primary_ecu_failure == 1) {
        pd->duration = 0;
        pd->current_ma = 0;
        pd->derived_rpm = 0;
      } else {
        pd->duration = ((simulate_excessive_dwell_time) ? 4500 : 3500) | (uint16_t)(rand()&0xff);
        pd->current_ma = 6500 | (uint16_t)(rand()&0x1ff);
        pd->derived_rpm = simulate_engine_rpm + (((rand() & 0x1) == 1) ? -1 : 1) * (rand() & 0x3);
      }
    } else {
      int pump_index = i&0x1;
      if (fake_pump_is_on[pump_index] == 0) {
        fake_pump_start_time[pump_index] = 0;
        fake_pump_started[pump_index] = 0;
        pd->duration = 0;
      } else if (!fake_pump_started[pump_index]) {
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        fake_pump_start_time[pump_index] = ts_now.tv_sec;
        fake_pump_started[pump_index] = 1;
      } else {
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        pd->duration = (uint16_t)(ts_now.tv_sec - fake_pump_start_time[pump_index]);
        }

      if (fake_pump_is_on[pump_index] == 1) {
        pd->current_ma = 4500 | (uint16_t)(rand()&0xff);
        pd->ripple_ma = 750 | (uint16_t)(rand()&0xff);
        pd->surge_ma = 11100 | (uint16_t)(rand()&0x3ff);
      } else {
        pd->current_ma = 0;
        pd->ripple_ma = 0;
        pd->surge_ma = 0;
      }
    }

    if (mask == 0x3ff)
      saved_average_current[i] = (saved_average_current[i] * 3 + (pd->current_ma & PD_CURRENT_MASK)) >> 2;

    return(pd);
  } else if (efi_fd < 0) {
      return(NULL);
  } else {
    // Get data from the EFI redundant power box
    if (index == 0) {
      // Get a new tranch of real data
      if (efi_noise)
        dbprintf("efi_get_plot_data: TOP\n");
      pthread_mutex_lock(&efi_lock);
      efi_get_current_points = injector_points | (coilpack_points << 16);
      efi_get_currents = mask;
      pthread_cond_signal(&efi_req_cond);
      while (efi_get_currents && !efi_thread_abort) {
        int rc = pthread_cond_wait(&efi_done_cond, &efi_lock);      // Usually takes around 3 msec
        if (rc != 0) {
          pthread_mutex_unlock(&efi_lock);
          return(NULL);
        }
      }
      pthread_mutex_unlock(&efi_lock);
      if (efi_noise)
        dbprintf("efi_get_plot_data: back\n");

      // The TFT display is the only system that issues OP_CURRENT requests,
      // so the data is safe now and cannot be over-written.
      for (i = 0; i < NUM_CURRENTS; i++)
        if (mask & (1 << i)) {
          pd->plot_data = all_real_data[i];     // First hit is index 0
          break;
        }
      //dbprintf("get_plot_data, pumps %d/%d\n", 
      //  (efi_currents_sts.latch_power_fault_status & 0x40000000) ? 1 : 0,
      //  (efi_currents_sts.latch_power_fault_status & 0x80000000) ? 1 : 0);
   
      // Update fuelpump status
      // Don't do this if logging active, to avoid inconsistent results during
      // ON/OFF transitions, we let the OP_LOG entries dominate
      if (!logging_active)
        update_fuelpump_status(
          (efi_currents_sts.latch_power_fault_status & 0x40000000) ? 1 : 0,
          (efi_currents_sts.latch_power_fault_status & 0x80000000) ? 1 : 0);

      // Now the remainder
      implied_ecu_failure = (uint8_t)(efi_currents_sts.latch_power_fault_status >> 28) & 0x3;
      pd->fault = (efi_currents_sts.latch_power_fault_status & (0x1 << (i+16))) ? FAULT_WIRING : 0;
      do_update_status(i, pd);

      if (IS_INJECTOR(i)) {
        pd->duration = efi_currents_sts.u[i].injector.duration_usec;
        pd->current_ma = efi_currents_sts.u[i].injector.current_ma;
      } else if (IS_COILPACK(i)) {
        pd->duration = efi_currents_sts.u[i].coil.dwell_time_usec;
        pd->current_ma = efi_currents_sts.u[i].coil.current_ma;
        pd->derived_rpm = efi_currents_sts.u[i].coil.derived_rpm;
      } else {
        pd->duration = efi_currents_sts.u[i].pump.run_time_seconds;
        pd->current_ma = efi_currents_sts.u[i].pump.current_ma;
        pd->surge_ma = efi_currents_sts.u[i].pump.surge_current_ma;
        pd->ripple_ma = efi_currents_sts.u[i].pump.ripple_ma;
      }
      saved_average_current[i] = (saved_average_current[i] * 3 + (pd->current_ma & PD_CURRENT_MASK)) >> 2;

      return(pd);
    } else {
      // Plots always step through indices sequentially, so
      // we've already got the data. So, no need to get it.

      for (i = 0, idx = 0; i < NUM_CURRENTS; i++) {
        if (mask & (1 << i)) {
          if (idx == index) {
            pd->plot_data = all_real_data[i];

            if (IS_INJECTOR(i)) {
              pd->duration = efi_currents_sts.u[i].injector.duration_usec;
              pd->current_ma = efi_currents_sts.u[i].injector.current_ma;
            } else if (IS_COILPACK(i)) {
              pd->duration = efi_currents_sts.u[i].coil.dwell_time_usec;
              pd->current_ma = efi_currents_sts.u[i].coil.current_ma;
              pd->derived_rpm = efi_currents_sts.u[i].coil.derived_rpm;
            } else {
              pd->duration = efi_currents_sts.u[i].pump.run_time_seconds;
              pd->current_ma = efi_currents_sts.u[i].pump.current_ma;
              pd->surge_ma = efi_currents_sts.u[i].pump.surge_current_ma;
              pd->ripple_ma = efi_currents_sts.u[i].pump.ripple_ma;
            }
            saved_average_current[i] = (saved_average_current[i] * 3 + (pd->current_ma & PD_CURRENT_MASK)) >> 2;

            break;
          }
          idx++;
        }
      }
      pd->fault = (efi_currents_sts.latch_power_fault_status & (0x1 << (i+16))) ? FAULT_WIRING : 0;
      do_update_status(i, pd);

      return(pd);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Interfaces with EFI redundant power module
////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_BAT

static struct termios saved_bat_attributes;
static int bat_fd = -1;

static void restore_bat_attributes(void) 
{
  if (bat_fd >= 0)
    tcsetattr(bat_fd,TCSANOW,&saved_bat_attributes);
}
#endif

static struct termios saved_hf_attributes;
static int hf = -1;

static void restore_hf_attributes(void) 
{
  if (hf >= 0)
    tcsetattr(hf,TCSANOW,&saved_hf_attributes);
}

#ifdef INCLUDE_FLAPS
static struct termios saved_flaps_attributes;
static void restore_flaps_attributes(void) 
{
  if (flaps_fd >= 0)
    tcsetattr(flaps_fd,TCSANOW,&saved_flaps_attributes);
}
#endif

static struct termios saved_vib_attributes;
static void restore_vib_attributes(void) 
{
  if (vib_fd >= 0)
    tcsetattr(vib_fd,TCSANOW,&saved_vib_attributes);
}

// CRC-32 calculator
static uint32_t crc32_table[256];
static int made_crc32_table = 0;

#define DI32 0x04c11db7L

void mo_init_crc32(void)
    {
    uint32_t i, j;
    uint32_t crc;

    for (i=0; i<256; i++)
        {
        crc = i << 24;
        for (j=0; j<8; j++)
            {
            if (crc & 0x80000000L)
                crc = (crc << 1) ^ DI32;
            else
                crc = (crc << 1);
            }
        crc32_table[i] = crc;
        }
    made_crc32_table = 1;
    }

static uint32_t mo_crc32(unsigned char *p, int len, int plug_in)
    {
    uint32_t crc = 0xffffffffU, crc_sav;
    uint32_t i;

    if (!made_crc32_table)
        mo_init_crc32();

    while (len--)
        {
        i = ((crc >> 24) ^ *p++) & 0xff;
        crc = (crc << 8) ^ crc32_table[i];
        }

    crc_sav = crc;

    //applog(LOG_DEBUG, "mo_crc32: crc is 0x%08x", crc);

    if (plug_in)
        {
        for (i = 0; i < 4; i++, crc >>= 8)
            *p++ = crc & 0xff;
        }

    return(crc_sav);
    }

//
// Request/Response interface with EFI box
//
static char *opcodes[] = {
  "OP_NULL", "OP_RESET", "OP_ECHO", "OP_TEST", "OP_CONFIG", "unknown", "OP_ENABLE_DISABLE", "OP_SCALE_FACTORS", "OP_TPS_LATCH", "OP_TPS_RESET",   // 0 - 9
  "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown",   // 10 - 19
  "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown",   // 20 - 29
  "unknown", "OP_PWM", "OP_CURRENTS", "OP_STATS", "OP_TEMPS", "OP_LOG", "OP_FPS_RECORD", "OP_FLAP", "OP_BPOWER", "OP_FLAP_SETTINGS",     // 30 - 39
  "OP_VIB"
  };

// The sequence distance between a sent and received sequence number.
#define EFI_SEQUENCE_DISTANCE(tx,rx)        ((tx)>=(rx)?((tx)-(rx)):(info->num_sequence+(tx)-(rx)))

static void efi_dump_frame(struct efi_frame *h, char *t, int data_size, int dump_data)
{
  char buf[20];
  uint8_t *p;
  int i;

  snprintf(buf, sizeof(buf), "(%d byte data frame)", data_size);

  dbprintf("%8s: %02x %02x %02x %02x %08x %08x %08x     (%s)\n",
    t,
    h->header.preamble & 0xff,
    h->header.opcode & 0xff,
    h->header.status & 0xff,
    h->header.sequence & 0xff,
    h->header.opdata1,
    h->header.opdata2,
    h->header.crc32,
    opcodes[h->header.opcode % (OP_MAX+1)]);

  if (dump_data && data_size > 0) {
    for (p = &h->buffer[0], i = 0; i < data_size;) {
        printf("%8d: ", i);
      for (int j = 0; j < 32 && (i+j < data_size); j++) {
        printf("%02x ", *p++);
      }
      i += 32;
      printf("\n");
    }
  }
}


void efi_dump_config(struct config_data *c)
{
  printf("Configuration data:\n");
  printf("     Magic:          0x%04x (%s)\n", c->magic, (c->magic == CONFIG_MAGIC) ? "correct" : "incorrect");
  printf("     Board Revision: %1d.%1d\n", c->board_revision_major, c->board_revision_minor);
  printf("     Board Serial #: %1d\n", c->board_serial_number);
  printf("     Cylinders:      %1d\n", c->cylinders);
  printf("     Coilpacks:      %1d\n", c->coilpacks);
  printf("     Fuelpumps:      %1d\n", c->fuelpumps);
  printf("     Firmware Rev:   %1d.%1d\n", (c->firmware_revision & 0xff), ((c->firmware_revision & 0xff00) >> 8));

}

////////////////////////////////////////////////////////////////////////////////
// Get a frame, with timeouts
////////////////////////////////////////////////////////////////////////////////

static int efi_get_frame(int fd, struct efi_frame *h)
{
  int read_amount = sizeof(h->header);
  int total_amount;
  uint8_t *result = (uint8_t *)h;
  struct timeval timeout;
  ssize_t ret = 0;
  uint32_t fcrc;
  fd_set rd;

  // Read 1 byte until we get a valid preamble
  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (unlikely(ret < 0))
      {
      dbprintf("efi_get_frame: Error %d on select in efi_get_header, for fd %d\n", errno, fd);
      perror("efi_get_frame: ooops");
      return(ret);
      }
    if (!ret)
      return(0);                    // Timeout
    ret = read(fd, result, 1);      // Read 1 byte
    if (unlikely(ret <= 0))
      return(ret);                  // Error or timeout
    } while (*result != EFI_PREAMBLE);

  // Found a preamble byte, now read the rest of the frame header
  result++;
  read_amount--;
  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (unlikely(ret <= 0))
      return(ret);                                  // Error or timeout

    ret = read(fd, result, read_amount);
    if (unlikely(ret < 0))
      {
      dbprintf("efi: Error %d on read in efi_get_header", errno);
      //applog(LOG_WARNING, "efi: Error %d on read in efi_get_header", errno);
      return(ret);
      }
    result += ret;
    read_amount -= ret;
  } while (read_amount > 0);

  // Check the header CRC and status
  fcrc = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);
  if (fcrc != h->header.crc32) {
    dbprintf("efi_get_frame: Bad header checksum fcrc 0x%08x vs 0x%08x\n", fcrc, h->header.crc32);
    efi_dump_frame(h, "FROM", sizeof(struct efi_header), 0);
    return(EBADHEADERCRC);
    }

  if (h->header.status != 0) {
    dbprintf("efi_get_frame: Bad header status %d (0x%x)\n", h->header.status, h->header.status);
    efi_dump_frame(h, "FROM", sizeof(struct efi_header), 0);
    return(EBADHEADERSTS);
    }

  // If there's a data frame, get it
  switch (h->header.opcode) {
    case OP_STATS:
      read_amount = sizeof(struct efi_stats) + 4;
      break;

    case OP_CURRENTS:
      if ((h->header.opdata2 & OP_CURRENT_ALL_MASK) == 0)
        read_amount = sizeof(struct efi_currents) + 4;
      else {
        int injector_points = ((uint16_t)(h->header.opdata1) & 0xffff);
        int coilpump_points = ((uint16_t)(h->header.opdata1 >> 16) & 0xffff);
        int i;

        read_amount = 4;                    // CRC
        read_amount += sizeof(struct efi_currents_header);

        for (i = 0; i < CYLINDERS; i++)
          if (h->header.opdata2 & (1 << (i+OP_CURRENT_INJECTORS_SHIFT)))
            read_amount += injector_points;
        for (i = 0; i < COILPACKS; i++)
          if (h->header.opdata2 & (1 << (i+OP_CURRENT_COILPACKS_SHIFT)))
            read_amount += coilpump_points;
        for (i = 0; i < FUELPUMPS; i++)
          if (h->header.opdata2 & (1 << (i+OP_CURRENT_FUELPUMPS_SHIFT)))
            read_amount += coilpump_points;
        }
      break;

    case OP_TEMPS:
      read_amount = sizeof(struct efi_temps) + 4;
      break;

    case OP_LOG:
      read_amount = h->header.opdata2 + 4;
      if (read_amount == 4)
        return(sizeof(h->header));              // Header only, no data
      break;

    case OP_FPS_RECORD:
      read_amount = sizeof(struct pump_start_record) + 4;
      break;

    case OP_CONFIG:
      read_amount = sizeof(struct config_data) + 4;
      break;

    default:
      return(sizeof(h->header));            // No data, return
      break;
  }

  if (read_amount > MAX_EFI_FRAME_DATA_SIZE)    // Should never happen, but if it does...
    read_amount = MAX_EFI_FRAME_DATA_SIZE;      // will lead to a CRC error

  total_amount = sizeof(h->header);

  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (ret <= 0) {
      dbprintf("efi: ret %ld from select \n", (long int)ret);
      return(ret);                                  // Error or timeout
      }

    ret = read(fd, result, read_amount);
    if (unlikely(ret < 0))
      {
      dbprintf("efi: ret %ld from read attempt of %d\n", (long int)ret, read_amount);
      perror("read() in efi_get_frame");
      return(ret);
      }
    result += ret;
    total_amount += ret;
    read_amount -= ret;
  } while (read_amount > 0);

  // Compute the data CRC, and check it out
  fcrc = mo_crc32((unsigned char *)h->buffer, total_amount - sizeof(h->header) - 4, 0);
  result -= sizeof(uint32_t);
  if (fcrc != *(uint32_t *)result) {
    dbprintf("ERROR: Bad data CRC 0x%08x, expected 0x%08x from total_amount %d\n", *(uint32_t *)result, fcrc, total_amount);
    efi_dump_frame(h, "FROM", total_amount - sizeof(h->header), 1);
    return(EBADDATACRC);
    }

  return(total_amount - 4);
}


void efi_echo()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_ECHO;
  h->opdata1 = 0;
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

void efi_config(struct config_data *c)
{
  struct efi_header *h = &tx_frame.header;
  struct efi_frame *f = &rx_frame;
  int len = sizeof(struct efi_header);

  h->opcode = OP_CONFIG;
  h->opdata1 = (c) ? 1 : 0;         // 1 == Write
  h->opdata2 = 0;

  if (c) {
    memcpy(&tx_frame.config, c, sizeof(*c));
    len += sizeof(struct config_data);
    }

  do_efi_transaction(efi_fd, h, f, len);
}

// 100 Hz
#define LOADON_1MSEC_100HZ ((999<<16)|9999)
#define LOADON_2MSEC_100HZ ((1999<<16)|9999)
#define LOADON_3MSEC_100HZ ((2999<<16)|9999)
#define LOADON_4MSEC_100HZ ((3999<<16)|59999)
#define LOADON_5MSEC_100HZ ((4999<<16)|59999)
#define LOADON_6MSEC_100HZ ((5999<<16)|59999)
#define LOADON_7MSEC_100HZ ((6999<<16)|59999)
#define LOADON_8MSEC_100HZ ((7999<<16)|59999)

// 25 Hz
#define LOADON_5MSEC_25HZ ((4999<<16)|39999)

static uint8_t eit_mask = 0;
static uint8_t eit_enable = 0;
static uint8_t eit_request = 0;

void do_efi_injector_test(uint8_t mask, uint8_t enable)
{

  if (efi_fd < 0)
    return;

  pthread_mutex_lock(&efi_lock);
  eit_mask = mask;
  eit_enable = enable;
  eit_request = 1;
  ACTIVATE_EFI_THREAD(eit_request);
}

void efi_injector_test(uint8_t mask, uint8_t enable)
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  //dbprintf("efi_injector_test: entered: %s\n", (enable) ? "ENABLE" : "DISABLE");
  h->opcode = OP_TEST;
  h->opdata1 = ((efi_left_coil_disabled) ? TEST_COILPACK_LEFT : 0) | ((efi_right_coil_disabled) ? TEST_COILPACK_RIGHT : 0);
  if (enable)
    ;
  else
    h->opdata1 |= TEST_INJECTORS;      // Disable all injectors

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

static uint8_t ect_which = 0;
static uint8_t ect_enable = 0;
static uint8_t ect_request = 0;

void do_efi_coilpack_test(uint8_t which, uint8_t enable)
{
  if (efi_fd < 0)
    return;

  pthread_mutex_lock(&efi_lock);
  ect_which = which;
  ect_enable = enable;
  ect_request = 1;
  ACTIVATE_EFI_THREAD(ect_request);

  //dbprintf("do_efi_coilpack_test: done\n");
}

void efi_coilpack_test(uint8_t which, uint8_t enable)
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  //dbprintf("efi_coilpack_test: entered: %s %s\n", (which) ? "RIGHT: " : "LEFT : ", (enable) ? "ENABLE" : "DISABLE");
  h->opcode = OP_TEST;
  // Remember opposite state
  h->opdata1 = (which) ? ((efi_left_coil_disabled) ? TEST_COILPACK_LEFT : 0) : ((efi_right_coil_disabled) ? TEST_COILPACK_RIGHT : 0);
  h->opdata1 |= (efi_injectors_disabled) ? TEST_INJECTORS : 0;
  if (enable)
    ;
  else
    h->opdata1 |= (which) ? TEST_COILPACK_RIGHT : TEST_COILPACK_LEFT;  // Bitmask saying which channel
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

uint8_t rls_request = 0;
uint16_t rls_latch_settings = 0;

void do_efi_write_latch_settings(uint16_t latch_settings)
{
  if (efi_fd < 0)
    return;

  pthread_mutex_lock(&efi_lock);
  rls_latch_settings = latch_settings;
  rls_request = 1;
  ACTIVATE_EFI_THREAD(rls_request);
}

void efi_write_latch_settings(uint16_t latch_settings)
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_TPS_LATCH;
  h->opdata1 = latch_settings;
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

static uint8_t tps_request = 0;
static int tps_idx = 0;

void do_efi_tps_reset(int idx)
{
  pthread_mutex_lock(&efi_lock);
  tps_idx = idx;
  tps_request = 1;
  ACTIVATE_EFI_THREAD(tps_request);
}

void efi_tps_reset(int idx)
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_TPS_RESET;
  h->opdata1 = (uint32_t)idx;
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}


void efi_currents(uint32_t opdata2)
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  //dbprintf("efi_currents: fake %d\n", efi_fake_data_ok);

  h->opcode = OP_CURRENTS;
  h->opdata1 = efi_get_current_points;
  h->opdata2 = opdata2 | ((efi_fake_data_ok) ? 0x80000000 : 0);         // Bitmask of currents to get (b9:0), fake (b31)

  if (efi_noise && verbose)
    dbprintf("efi_currents(): OP_CURRENTS: opdata1 0x%08x opdata2 0x%08x\n", h->opdata1, h->opdata2);

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

void efi_get_stats_update()
{
  if (efi_fd < 0)
    return;

  // Update stats, wait for update
  pthread_mutex_lock(&efi_lock);
  efi_get_stats = 1;
  pthread_cond_signal(&efi_req_cond);
  while (efi_get_stats && !efi_thread_abort) {
    int rc = pthread_cond_wait(&efi_done_cond, &efi_lock);
    if (rc != 0) {
      pthread_mutex_unlock(&efi_lock);
      return;
    }
  }
  pthread_mutex_unlock(&efi_lock);

}

static void efi_get_fuelpump_laststart_data(int index)
{
  if (efi_fd < 0)
    return;

  pthread_mutex_lock(&efi_lock);
  efi_get_psdata = index+1;             // 1 = left pump, 2 = right pump
  pthread_cond_signal(&efi_req_cond);
  while (efi_get_psdata && !efi_thread_abort) {
    int rc = pthread_cond_wait(&efi_done_cond, &efi_lock);
    if (rc != 0) {
      pthread_mutex_unlock(&efi_lock);
      return;
    }
  }
  pthread_mutex_unlock(&efi_lock);
}

void efi_get_temps_update()
{
  // Update temps, wait for update
  pthread_mutex_lock(&efi_lock);
  efi_get_temps = 1;
  pthread_cond_signal(&efi_req_cond);
  while (efi_get_temps && !efi_thread_abort) {
    int rc = pthread_cond_wait(&efi_done_cond, &efi_lock);
    if (rc != 0) {
      pthread_mutex_unlock(&efi_lock);
      return;
    }
  }
  pthread_mutex_unlock(&efi_lock);
}

static uint8_t enter_temp_request = 0;

static void do_efi_enter_temp_mode()
{
  if (efi_fd < 0)
    return;

  pthread_mutex_lock(&efi_lock);
  enter_temp_request = 1;
  ACTIVATE_EFI_THREAD(enter_temp_request);
}

static void efi_enter_temp_mode()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_TEMPS;
  h->opdata1 = TURN_ON_TEMP_MODE;
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

static uint8_t leave_temp_request = 0;

static void do_efi_leave_temp_mode()
{
  if (efi_fd < 0)
    return;

  pthread_mutex_lock(&efi_lock);
  leave_temp_request = 1;
  ACTIVATE_EFI_THREAD(leave_temp_request);
}

static void efi_leave_temp_mode()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_TEMPS;
  h->opdata1 = TURN_OFF_TEMP_MODE;
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

void efi_update_stats()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_STATS;
  h->opdata1 = 0;
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

void efi_update_temps()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_TEMPS;
  h->opdata1 = GET_TEMPS;
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

void efi_get_pump_start_data()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_FPS_RECORD;
  h->opdata1 = efi_get_psdata-1;    // 0 = left pump, 1 = right pump
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

void efi_gather_logrecords()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_LOG;
  if (log_finish)
    h->opdata1 = 0x1;
  else if (log_start)
    h->opdata1 = 0x2;
  else
    h->opdata1 = 0;
  h->opdata2 = 0;                       // Data length returned in this field

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));

  if (log_finish) {
    logging_active = 0;
    log_finish = 0;
    log_finish_sample_count = f->header.opdata1;
    if (log_noise)
      dbprintf("Logging finished, final count %d\n", f->header.opdata1);
    }
  else if (log_start) {
    if (log_noise)
      dbprintf("Logging started\n");
    log_start = 0;
    }
}


void efi_stats_update(struct efi_stats *s)
{
  int i;

  pthread_mutex_lock(&efi_stats_lock);
  efi_stats.rx_length_errors += s->rx_length_errors;
  efi_stats.oversampled += s->oversampled;
  efi_stats.tracking_single_reversals += s->tracking_single_reversals;
  efi_stats.tracking_double_reversals += s->tracking_double_reversals;
  efi_stats.preamble_errors += s->preamble_errors;
  efi_stats.header_crc_errors += s->header_crc_errors;
  //efi_stats.sequence_errors += s->sequence_errors;                    // XXX Unnecessary
  for (i = 0; i < NUM_CURRENTS; i++) {
    efi_stats.single_glitches[i] += s->single_glitches[i];
    efi_stats.double_glitches[i] += s->double_glitches[i];
    efi_stats.pulses[i] += s->pulses[i];
    efi_stats.overrun[i] += s->overrun[i];
    efi_stats.faults[i] += s->faults[i];
    efi_stats.log_records_discarded[i] += s->log_records_discarded[i];
    }
  pthread_mutex_unlock(&efi_stats_lock);
}

static double adc_to_junction_temp(int idx, uint16_t adc_count) {
  const double vref = (double) 2.9;
  const double dIsnst_dT = 0.0112;                          // mA per degree C

  double voltage = (double)adc_count / 4095.0 * vref;
  double Rsense = (idx < 6) ? 2000 : 1000;
  double current = voltage / Rsense * 1000.0;               // Isense in mA
  double jt = (current - 0.85) / dIsnst_dT + 25;

  return(jt);
}


void efi_temps_update(struct efi_frame *t)
{
  int i;

  //dbprintf("Junction temps: opdata2 is %d\n", t->header.opdata2);
  for (i = 0; i < NUM_CURRENTS; i++) {
    efi_temps.junction_temperature[i] = t->temps.junction_temperature[i];

    double x;
    if (t->temps.junction_temperature[i] > 0)  {

      x = adc_to_junction_temp(i, t->temps.junction_temperature[i]);
      fp_temps.junction_temperature[i] = (float)x;
      if (x > (double) 5.0) {
        snprintf((char *) &gui_temps.junction_temperature[i], 16, "%5.1fC", x);
        //dbprintf("  junction temp [%d] = %.1f\n", i, x);
        }
      else {
        //dbprintf("  junction temp [%d] = (silly value)\n", i);
        snprintf((char *) &gui_temps.junction_temperature[i], 16, "%5.1fX", x);
        }
    }
    else {
      //dbprintf("  junction temp [%d] = N/A\n", i);
      snprintf((char *) &gui_temps.junction_temperature[i], 16, "N/A");
      }
  }
}

static pthread_mutex_t fuelpump_start_data_lock = PTHREAD_MUTEX_INITIALIZER;

void efi_fuelpump_start_response(struct efi_frame *f)
{
  int idx = f->header.opdata1 & 0x1;

  pthread_mutex_lock(&fuelpump_start_data_lock);
  memcpy(&fuelpump_start_current_plot[idx], &f->fps, sizeof(struct pump_start_record));
  fuelpump_data_sequence[idx]++;
  fscp_valid[idx] = 1;
  pthread_mutex_unlock(&fuelpump_start_data_lock);

#ifdef INCLUDE_LOGGING
  if (logging_active)
    log_pumpstart(idx);
  else
    delayed_log_pumpstart |= (1 << idx);
#endif

}


////////////////////////////////////////////////////////////////////////////////
// Save returned log messages
////////////////////////////////////////////////////////////////////////////////

static int log_fd[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
#ifdef ARM
static char log_dev[128] = { "/media/port/XVG_Logs" };
#else
static char log_dev[128] = { "/run/media/port/XVG_Logs" };        // Fedora, other
#endif
static char log_dir[256] = { "runtime/logs/test" };
static char const *log_name[] = {
        "injector_0", "injector_1", "injector_2", "injector_3", "injector_4", "injector_5",
        "coil_left", "coil_right", "fuelpump_left", "fuelpump_right"};

// Update log records, from returned data
void efi_log_update(struct efi_frame *t)
{
  struct log_data *ld = (struct log_data *) &t->log;
  uint8_t *data = (uint8_t *) &t->log;                      // Offsets are wrt this point
  uint8_t *p;
  int i, nb;
  char buf[256];

  for (i = 0; i < DIMENSION(ld->offset); i++) {
    nb = ld->offset[i].length;
    p = data + ld->offset[i].offset;
#if 0
    if (nb)
      dbprintf("idx %d offset %d length %d (0x%02x %02x %02x %02x %02x %02x)\n",
        i, ld->offset[i].offset, nb, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5));
#endif
    if (log_fd[i] < 0) {
      snprintf(buf, sizeof(buf), "%s/log_%s.bin", log_dir, log_name[i]);
      log_fd[i] = open(buf, O_CREAT|O_WRONLY|O_TRUNC, S_IRWXU|S_IRWXG);
      if (log_fd[i] < 0) {
        fprintf(stderr, "Can't create file %s\n", buf);
        perror("open() in efi_log_update");
        return;
      }
    }
    if (nb) {
      if (write_full(log_fd[i], p, nb))
        dbprintf("Write error for file %d (fd %d), nb %d\n", i, log_fd[i], nb);
      }
  }
}

void log_timefile(struct timespec *start, struct timespec *end, uint32_t num_samples)
{
  FILE *f;
  char buf[256];

  snprintf(buf, sizeof(buf), "%s/log_times.txt", log_dir);
  f = fopen(buf, "w+");
  if (f) {
    fprintf(f, "%lu %lu %lu %lu %u\n", start->tv_sec, start->tv_nsec, end->tv_sec, end->tv_nsec, num_samples);
    if (pilots && pilot_in_command >= 0)
      fprintf(f, "Pilot: %s\n", (pilots+pilot_in_command)->fullname);
    else
      fprintf(f, "Pilot: unknown\n");
    fclose(f);
    }
}

static int pslog_fd[] = {-1, -1};
static char const *pslog_name[] = {"left_pumpstart_detail", "right_pumpstart_detail"};

void log_pumpstart(int idx)
{
  static struct timespec lps[FUELPUMPS];        // Saved time when we last logged a pump start

  char buf[256];
  struct timespec now;
  uint32_t msec_since_last_log;

  clock_gettime(CLOCK_MONOTONIC, &now);
  time_t secs = now.tv_sec - lps[idx].tv_sec;
  long   nsecs = now.tv_nsec - lps[idx].tv_nsec;
  msec_since_last_log = (uint32_t)((secs * 1000) + (nsecs / 1000000));

  // Ignore pump start events that occur within a 2 second interval,
  // The log is 1 second long, if a fuel pump is toggled more often
  // then that the intermediate log data will be mangled anyway.
  if (msec_since_last_log < 2000)
    return;

  if (pslog_fd[idx] == -1) {
    snprintf(buf, sizeof(buf), "%s/log_%s.bin", log_dir, pslog_name[idx]);
    pslog_fd[idx] = open(buf, O_CREAT|O_WRONLY|O_TRUNC, S_IRWXU|S_IRWXG);
    if (pslog_fd[idx] < 0) {
      fprintf(stderr, "Can't create file %s\n", buf);
      perror("open() in log_pumpstart");
      return;
    }
  }

  // No error checking, if it fails, too bad.
  pthread_mutex_lock(&fuelpump_start_data_lock);
  write_full(pslog_fd[idx], (uint8_t *)&ts_pumpstart_realtime[idx], sizeof(struct timespec));
  write_full(pslog_fd[idx], (uint8_t *)&fuelpump_start_current_plot[idx], sizeof(struct pump_start_record));
  pthread_mutex_unlock(&fuelpump_start_data_lock);

  lps[idx].tv_sec = now.tv_sec;
  lps[idx].tv_nsec = now.tv_nsec;

  dbprintf("log_pumpstart: %s fuelpump\n", (idx) ? "RIGHT" : "LEFT");
}

void efi_log_close()
{
  int i;

  if (log_post_finish > 0)
    dbprintf("efi_log_close: Turning OFF logging fuelpumps %s/%s\n", (fuelpump_on[0]) ? "ON" : "OFF", (fuelpump_on[1]) ? "ON" : "OFF");

  for (i = 0; i < DIMENSION(log_fd); i++)
    if (log_fd[i] >= 0) {
      close(log_fd[i]);
      log_fd[i] = -1;
      }

  for (i = 0; i < DIMENSION(pslog_fd); i++)
    if (pslog_fd[i] >= 0) {
      close(pslog_fd[i]);
      pslog_fd[i] = -1;
      }
}


////////////////////////////////////////////////////////////////////////////////
// Dump EFI Statistics
////////////////////////////////////////////////////////////////////////////////
void dump_efi_stats() {
  const static char *name[] = {"i0", "i1", "i2", "i3", "i4", "i5", "c0", "c1", "f0", "f1"};
  int i;

  dbprintf("EFI Stats:\n");
  pthread_mutex_lock(&efi_stats_lock);
  printf("      %20s:  %d\n", "rx_length errors", efi_stats.rx_length_errors);
  printf("      %20s:  %d\n", "preamble errors", efi_stats.preamble_errors);
  printf("      %20s:  %d\n", "header crc errors", efi_stats.header_crc_errors);
  printf("      %20s:  %d\n", "sequence errors", efi_stats.sequence_errors);
  printf("      %20s:  %d\n", "oversampled", efi_stats.oversampled);
  printf("      %20s:  %d\n", "single reversals", efi_stats.tracking_single_reversals);
  printf("      %20s:  %d\n", "double reversals", efi_stats.tracking_double_reversals);
  for (i = 0; i < NUM_CURRENTS; i++) {
    printf("     %s:   %s %8d single glitches %8d double glitches %8d overruns %8d faults %8d log_records_discarded %8d\n",
      name[i], (i < 8) ? "pulses " : "samples",
      efi_stats.pulses[i], efi_stats.single_glitches[i],
      efi_stats.double_glitches[i], efi_stats.overrun[i], efi_stats.faults[i], efi_stats.log_records_discarded[i]);
    }
  pthread_mutex_unlock(&efi_stats_lock);
}


static void efi_set_divisors(uint32_t injector_divisor, uint32_t coilpack_divisor)
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &rx_frame;

  h->opcode = OP_SCALE_FACTORS;
  h->opdata1 = ((coilpack_divisor & 0xff) << 8) | (injector_divisor & 0xff);
  h->opdata2 = 0;

  do_efi_transaction(efi_fd, h, f, sizeof(struct efi_header));
}

//
// Debug dump of OP_CURRENTS results back from EFI power box
//
static void efi_dump_currents(struct efi_frame *f)
{
  int i, j, all;
  uint8_t *p = f->buffer;
  struct efi_currents_header efcs;
  int injector_points = ((uint16_t)(f->header.opdata1) & 0xffff);
  int coilpump_points = ((uint16_t)(f->header.opdata1 >> 16) & 0xffff);
  dbprintf("efi_dump_currents: injector_points %d, coilpump_points %d\n", injector_points, coilpump_points);

  all = ((f->header.opdata2 & OP_CURRENT_ALL_MASK) == 0) ? 1 : 0;

  memcpy(&efcs, p, sizeof(efcs));
  p += sizeof(efcs);
  dbprintf("efi_current_header: power_fault_status 0x%08x update_status 0x%08x\n", efcs.latch_power_fault_status, efcs.update_status);
  for (j = 0; j < FUELPUMPS; ++j)
    dbprintf("  pump[%d] run_time_seconds %d average_ma %d surge_ma %d\n", j, efcs.u[CYLINDERS+COILPACKS+j].pump.run_time_seconds,
        efcs.u[CYLINDERS+COILPACKS+j].pump.current_ma, efcs.u[CYLINDERS+COILPACKS+j].pump.surge_current_ma);

  for (j = 0; j < CYLINDERS; ++j) {
    if (all || (f->header.opdata2 & (1 << (j+OP_CURRENT_INJECTORS_SHIFT)))) {
      printf("i%d: ", j);
      for (i = 0; i < injector_points; ++i)
        if (*p == MO_FAULT)
          printf("\033[31;40m%02x \033[0m", *p++);
        else
          printf("%02x ", *p++);
      printf("\n");
    }
  }
  for (j = 0; j < COILPACKS; ++j) {
    if (all || (f->header.opdata2 & (1 << (j+OP_CURRENT_COILPACKS_SHIFT)))) {
      printf("c%d: ", j);
      for (i = 0; i < coilpump_points; ++i)
        if (*p == MO_FAULT)
          printf("\033[31;40m%02x \033[0m", *p++);
        else
          printf("%02x ", *p++);
      printf("\n");
      }
  }
  for (j = 0; j < FUELPUMPS; ++j) {
    if (all || (f->header.opdata2 & (1 << (j+OP_CURRENT_FUELPUMPS_SHIFT)))) {
      printf("f%d: ", j);
      for (i = 0; i < coilpump_points; ++i)
        if (*p == MO_FAULT)
          printf("\033[31;40m%02x \033[0m", *p++);
        else
          printf("%02x ", *p++);
      printf("\n");
      }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// Do a transaction with the EFI power box.
// opcode, opdata* fields filled in.
//
////////////////////////////////////////////////////////////////////////////////

static response_counts_t efi_counts[] = {
  { OP_CURRENTS,    "OP_CURRENTS",   0, 99.9, 0.0, 0.0 },
  { OP_LOG,         "OP_LOG",        0, 99.9, 0.0, 0.0 },
  { OP_STATS,       "OP_STATS",      0, 99.9, 0.0, 0.0 },
  { OP_TEMPS,       "OP_TEMPS",      0, 99.9, 0.0, 0.0 },
  { OP_FPS_RECORD,  "OP_FPS_RECORD", 0, 99.9, 0.0, 0.0 },
  { OP_CONFIG,      "OP_CONFIG",     0, 99.9, 0.0, 0.0 },
};
const static int num_efi_count = DIMENSION(efi_counts);

static void reset_efi_counts(void)
{
  for (int i = 0; i < DIMENSION(efi_counts); i++) {
    efi_counts[i].num = 0;
    efi_counts[i].min = 99.9;
    efi_counts[i].max = 0.0;
    efi_counts[i].tot = 0.0;
    }
}

void count_efi_stats(struct efi_frame *f, double elapsed_msec)
{
  response_counts_t *e;
  int i;

  for (e = &efi_counts[0], i = 0; i < num_efi_count; i++, e++) {
    if (e->opcode == f->header.opcode) {
      e->num++;
      if (e->min > elapsed_msec)
        e->min = elapsed_msec;
      if (e->max < elapsed_msec)
        e->max = elapsed_msec;
      e->tot += elapsed_msec;
      return;
    }
  }
}


static void do_efi_transaction(int fd, struct efi_header *h, struct efi_frame *f, int len)
{
  struct timespec begin, end;
  static uint8_t sequence = 1;
  int sts;
  int i, j;
  int tx_len = (len) ? len : sizeof(struct efi_header);

  h->preamble = EFI_PREAMBLE;
  h->sequence = sequence++;
  h->status = 0;
  h->crc32 = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);

  if (efi_noise && verbose) {
    if (tx_len > sizeof(struct efi_header))
      efi_dump_frame((struct efi_frame *)h, "TO", tx_len - sizeof(struct efi_header), 1);
    else
      efi_dump_frame((struct efi_frame *)h, "TO", 0, 0);
    }

  if (tx_len > sizeof(struct efi_header)) {
    // Add data CRC
    mo_crc32((unsigned char *)(h+1), tx_len - sizeof(struct efi_header), 1);
    tx_len += 4;
  }

  clock_gettime(CLOCK_MONOTONIC, &begin);
  if (write_full(fd, (char *)h, tx_len)) {
    dbprintf("do_efi_transaction: ERROR: write failed for length %d\n", tx_len);
    efi_thread_abort = 1;                   // Bad connection, restart thread
    return;
  }

  sts = efi_get_frame(fd, f);
  if (sts > 0) {
    clock_gettime(CLOCK_MONOTONIC, &end);
    long seconds = end.tv_sec - begin.tv_sec;
    long nanoseconds = end.tv_nsec - begin.tv_nsec;
    double elapsed_msec = seconds*1e3 + nanoseconds*1e-6;

    count_efi_stats(f, elapsed_msec);

    if (efi_noise && verbose) {
      efi_dump_frame(f, "FROM", sts - sizeof(struct efi_header), 1);
      printf("\n");
      }
    if (sts >= sizeof(*h)) {
      if (f->header.opcode == OP_CURRENTS) {
        if (efi_noise) {
          dbprintf("OP_CURRENTS took %.3f msec, opdata1 0x%08x, opdata2 0x%08x sts 0x%08x\n", elapsed_msec,
            f->header.opdata1, f->header.opdata2, f->currents.status.latch_power_fault_status);
          if (verbose)
            efi_dump_currents(f);
          }

        //int all = ((f->header.opdata2 & OP_CURRENT_ALL_MASK) ? 0 : 1);
        int all = (((f->header.opdata2 & OP_CURRENT_ALL_MASK) == OP_CURRENT_ALL_MASK) ? 1 : 0);
        uint8_t *p = f->buffer;
        int injector_points = ((uint16_t)(f->header.opdata1) & 0xffff);
        int coilpump_points = ((uint16_t)(f->header.opdata1 >> 16) & 0xffff);

        // Get the latch/power/fault status
        memcpy(&efi_currents_sts, p, sizeof(efi_currents_sts));
        p += sizeof(efi_currents_sts);
        if (verbose) {
          dbprintf("efi_currents_sts.latch_power_fault_status 0x%08x, update_sts 0x%08x\n",
                efi_currents_sts.latch_power_fault_status, efi_currents_sts.update_status);
        }

        // Fill in the data. For channels that are in fault, the data
        // is meaningless, but is sent anyway to keep the frame size
        // predictable / fixed.
        for (j = 0; j < CYLINDERS; ++j)
          if (all || (f->header.opdata2 & (1<<(j+OP_CURRENT_INJECTORS_SHIFT)))) {
            if (!(efi_currents_sts.update_status & (0x1 << j))) {
              p += injector_points;
              continue;                                     // No update available, keep prior data
              }
            for (i = 0; i < injector_points; ++i)
              real_injector_data[j][i] = *p++;
            }
        for (j = 0; j < COILPACKS; ++j)
          if (all || (f->header.opdata2 & (1<<(j+OP_CURRENT_COILPACKS_SHIFT)))) {
            if (!(efi_currents_sts.update_status & (0x1 << (j+OP_CURRENT_COILPACKS_SHIFT)))) {
              p += coilpump_points;
              continue;                                     // No update available, keep prior data
            }
            for (i = 0; i < coilpump_points; ++i)
              real_coilpack_data[j][i] = *p++;
            }
        for (j = 0; j < FUELPUMPS; ++j)
          if (all || (f->header.opdata2 & (1<<(j+OP_CURRENT_FUELPUMPS_SHIFT)))) {
            if (!(efi_currents_sts.update_status & (0x1 << (j+OP_CURRENT_FUELPUMPS_SHIFT)))) {
              p += coilpump_points;
              continue;                                     // No update available, keep prior data
            }
            for (i = 0; i < coilpump_points; ++i)
              real_fuelpump_data[j][i] = *p++;
          }
        }
      else if (f->header.opcode == OP_STATS) {
        //dbprintf("OP_STATS took %.3f msec, pumps %d/%d\n", elapsed_msec,
        //        f->header.opdata1 & 0x1, (f->header.opdata1 >> 1) & 0x1);
        // Update fuelpump status
        // Don't do this if logging active, to avoid inconsistent results during
        // ON/OFF transitions, we let the OP_LOG entries dominate
        if (!logging_active)
          update_fuelpump_status(f->header.opdata1 & 0x1, (f->header.opdata1 >> 1) & 0x1);
        efi_stats_update(&f->stats);
        }
      else if (f->header.opcode == OP_TEMPS) {
        if (efi_noise)
          dbprintf("OP_TEMPS took %.3f msec, frame length %d\n", elapsed_msec, sts);
        efi_temps_update(f);
        }
      else if (f->header.opcode == OP_LOG) {
        if (log_noise) {
          dbprintf("OP_LOG took %.3f msec, frame length %d, pumps %s/%s\n", elapsed_msec, sts,
                          (f->header.opdata1 & 0x10000) ? "ON" : "OFF",
                          (f->header.opdata1 & 0x20000) ? "ON" : "OFF");
          //dbprintf("sizeof(struct log_header) is %d\n", sizeof(struct log_header));
          //if (log_post_finish)
          //  efi_dump_frame(f, "log_post_finish", sts - sizeof(f->header),  1);
          }
        // Update fuelpump status
        update_fuelpump_status(
            (f->header.opdata1 & 0x10000) ? 1 : 0,
            (f->header.opdata1 & 0x20000) ? 1 : 0);

        // Deal with the data, if there is any
        if (sts > sizeof(f->header))
          efi_log_update(f);
        }
      else if (f->header.opcode == OP_FPS_RECORD) {
        //dbprintf("OP_FPS_RECORD took %.3f msec, frame length %d\n", elapsed_msec, sts);
        if (sts > sizeof(f->header))
          efi_fuelpump_start_response(f);
        }
      else if (f->header.opcode == OP_CONFIG) {
        if (sts > sizeof(f->header)) {
          memcpy((char *)&efi_board_config, (char *)&f->config, sizeof(struct config_data));
          if (efi_read_config || efi_write_config)
            efi_dump_config(&f->config);
          }
        }
      }
    } else {
      efi_thread_abort = 1;
      if (sts == 0) {
        dbprintf("efi: TIMEOUT\n");
        }
      else {
        dbprintf("efi: read returned bad status %d\n", sts);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// The thread that talks to the EFI Power Board
////////////////////////////////////////////////////////////////////////////////

#define EFI_REQUEST_ACTIVE (efi_get_currents || efi_get_psdata || efi_get_logs || eit_request || efi_get_temps || \
                            efi_get_temps || enter_temp_request || leave_temp_request || efi_get_stats || \
                            ect_request || eit_request || rls_request || tps_request)

static pthread_mutex_t termios_lock = PTHREAD_MUTEX_INITIALIZER;

static struct termios saved_efi_attributes;
static int saved_efi_valid = 0;

static void restore_efi_attributes(void)
{
  pthread_mutex_lock(&termios_lock);
  if (efi_fd >= 0 && saved_efi_valid)
    tcsetattr(efi_fd, TCSANOW, &saved_efi_attributes);
  pthread_mutex_unlock(&termios_lock);
}

void *efi_thread(void *marg)
{
  struct termios term;
  int sts;
  struct timespec ts;
  static uint8_t premsg = 1;

  while (!mo_abort) {
    if (efi_noise)
      dbprintf("efi_thread: started/restarted\n");

    // Open the CDC port
    while (!mo_abort) {
      efi_fd = open(efi_device, O_RDWR | O_NOCTTY);
      if (efi_fd < 0) {
        if (premsg) {
          if (efi_noise) {
            dbprintf("efi: ERROR: Opening CDC port %s (%s)\n", efi_device, strerror(errno));
            }
          premsg = 0;
          }
        mo_usleep(100000);
        }
      else {
        if (efi_noise)
          dbprintf("efi: Opened serial port %s OK\n", efi_device);
        break;
      }
    }
    if (mo_abort)
      break;

    sts = tcgetattr(efi_fd,&term);
    if (sts < 0) {
      dbprintf("efi: Error %d (%s) from tcgetattr\n", errno, strerror(errno));
      close(efi_fd);
      mo_usleep(50000);
      continue;
      }
    pthread_mutex_lock(&termios_lock);
    if (!saved_efi_valid) {
      memcpy(&saved_efi_attributes, &term, sizeof(saved_efi_attributes));
      saved_efi_valid = 1;
      }
    pthread_mutex_unlock(&termios_lock);

    cfmakeraw(&term);
    cfsetispeed(&term, BAUDRATE);
    cfsetospeed(&term, BAUDRATE);

    sts = tcsetattr(efi_fd,TCSANOW,&term);
    if (sts < 0) {
      dbprintf("efi: Error sts %d, errno %d (%s) from tcsetattr\n", sts, errno, strerror(errno));
      close(efi_fd);
      mo_usleep(50000);
      continue;
    }
    tcflush(efi_fd, TCIOFLUSH);
    clock_gettime(CLOCK_MONOTONIC, &ts);

    if (injector_divisor > 0 || coilpack_divisor > 0)
      efi_set_divisors(injector_divisor, coilpack_divisor);

    if (!(efi_write_config || efi_read_config))
      efi_config(NULL);               // Read the configuration data

    efi_thread_abort = 0;

    // Loop forever servicing requests
    while (!mo_abort && !efi_thread_abort) {
      set_timeout(&ts, 500000);           // Half a second
      pthread_mutex_lock(&efi_lock);
      if (!EFI_REQUEST_ACTIVE)
        sts = pthread_cond_timedwait(&efi_req_cond, &efi_lock, &ts);
      else
        sts = 0;
      if (sts == ETIMEDOUT) {
        pthread_mutex_unlock(&efi_lock);
        // There used to be a heartbeat here, but it doesn't seem necessary any more
        continue;
        }
      else if (!sts) {
        while (EFI_REQUEST_ACTIVE) {
          if (efi_noise)
            dbprintf("efi_thread: TOP: efi_get_currents %d efi_get_psdata %d efi_get_logs %d efi_get_stats %d efi_get_temps %d eit %d ect %d\n",
                          efi_get_currents, efi_get_psdata, efi_get_logs, efi_get_stats, efi_get_temps, eit_request, ect_request);
          // Someone has requested something
          if (efi_get_currents) {
            pthread_mutex_unlock(&efi_lock);
            efi_currents(efi_get_currents);
            pthread_mutex_lock(&efi_lock);
            efi_get_currents = 0;
            }
          else if (efi_get_psdata) {
            pthread_mutex_unlock(&efi_lock);
            efi_get_pump_start_data();
            pthread_mutex_lock(&efi_lock);
            efi_get_psdata = 0;
            }
          else if (efi_get_logs) {
            pthread_mutex_unlock(&efi_lock);
            efi_gather_logrecords();
            pthread_mutex_lock(&efi_lock);
            efi_get_logs = 0;
            }
          else if (eit_request) {
            pthread_mutex_unlock(&efi_lock);
            efi_injector_test(eit_mask, eit_enable);
            pthread_mutex_lock(&efi_lock);
            eit_request = 0;
            }
          else if (ect_request) {
            pthread_mutex_unlock(&efi_lock);
            efi_coilpack_test(ect_which, ect_enable);
            pthread_mutex_lock(&efi_lock);
            ect_request = 0;
            }
          else if (efi_get_temps) {
            pthread_mutex_unlock(&efi_lock);
            efi_update_stats();
            efi_update_temps();
            pthread_mutex_lock(&efi_lock);
            efi_get_temps = 0;
            }
          else if (enter_temp_request) {
            pthread_mutex_unlock(&efi_lock);
            efi_enter_temp_mode();
            pthread_mutex_lock(&efi_lock);
            enter_temp_request = 0;
            }
          else if (leave_temp_request) {
            pthread_mutex_unlock(&efi_lock);
            efi_leave_temp_mode();
            pthread_mutex_lock(&efi_lock);
            leave_temp_request = 0;
            }
          else if (efi_get_stats) {
            pthread_mutex_unlock(&efi_lock);
            efi_update_stats();
            pthread_mutex_lock(&efi_lock);
            efi_get_stats = 0;
            }
          else if (rls_request) {
            pthread_mutex_unlock(&efi_lock);
            efi_write_latch_settings(rls_latch_settings);
            pthread_mutex_lock(&efi_lock);
            rls_request = 0;
            }
          else if (tps_request) {
            pthread_mutex_unlock(&efi_lock);
            efi_tps_reset(tps_idx);
            pthread_mutex_lock(&efi_lock);
            tps_request = 0;
            }
          pthread_cond_broadcast(&efi_done_cond);
          }
        if (efi_noise)
          dbprintf("efi_thread: BOT: efi_get_currents %d efi_get_psdata %d efi_get_logs %d efi_get_stats %d efi_get_temps %d eit %d ect %d\n",
                        efi_get_currents, efi_get_psdata, efi_get_logs, efi_get_stats, efi_get_temps, eit_request, ect_request);

        pthread_mutex_unlock(&efi_lock);
      } else {
        dbprintf("efi_thread: unexpected result from pthread-cond-timedwait: %d (%s)\n", sts, strerror(sts));
        pthread_mutex_unlock(&efi_lock);
        break;
      }
    }

  // Ckear all request flags, to make sure none are lying around set
  // the next time the loop starts up.
  pthread_mutex_lock(&efi_lock);
  efi_get_currents = 0;
  efi_get_psdata = 0;
  efi_get_logs = 0;
  eit_request = 0;
  efi_get_temps = 0;
  efi_get_temps = 0;
  enter_temp_request = 0;
  leave_temp_request = 0;
  efi_get_stats = 0;
  rls_request = 0;
  tps_request = 0;
  pthread_cond_broadcast(&efi_done_cond);
  pthread_mutex_unlock(&efi_lock);

  efi_log_close();
  close(efi_fd);
  efi_fd = -1;
  premsg = 1;
  mo_usleep(50000);
  }

  dbprintf("efi_thread: Done\n");
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// The thread that talks to the Backup Battery switch box
////////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_BAT

#define BAT_ADC(v)          ((uint16_t) ((v) / (((1000.0 + 9100.0) / 1000.0) / 4095.0 * 3.3)))

bat_response_t bat_status = {0};
int bat_status_valid = 0;
int debug_fake_battery_backup = 0;
int debug_bat_comm = 0;

static int bat_tx_rx(int fd, char *tx, int ntx, unsigned char *rx, int nrx)
{
  struct timeval timeout;
  fd_set rd;
  uint8_t *result = (uint8_t *)rx;
  ssize_t ret = 0;
  int nb;

  if (!ntx)
    return(0);

  if (debug_bat_comm)
    printf("bat: PUT:        0x%x\n", (unsigned int)(*tx) & 0xff);

  write(fd, tx, ntx);

  if (!nrx)                                 // PUT operation
    return(0);

  nb = 0;
  timeout.tv_sec = 0;
  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = HF_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (ret <= 0) {
      if (bat_noise)
        fprintf(stderr, "bat_tx_rx: Error ret %zd from select\n", ret);
      return(ret);                                  // Error or timeout
      }

    ret = read(fd, result, 1);
    if (unlikely(ret <= 0))
      {
      fprintf(stderr, "bat_tx_rx: Error ret %zd from read\n", ret);
      return(ret);
      }
    if (debug_bat_comm) {
      printf("bat: GET:        0x%x\n", (unsigned int)*result & 0xff);
    }
    nb++;
    result++;
  } while (nb < nrx);

  return(nb);
}

static void bat_scan(int fd, bat_response_t *br)
{
  int nb;
  unsigned char poop[10];

  memset(poop, 0, sizeof(poop));
  nb = bat_tx_rx(fd, "a", 1, poop, sizeof(poop));
  if (nb != sizeof(poop)) {
    bat_status_valid = 0;
    return;
  } else {

    uint32_t crc;
    uint32_t received_crc;

    crc = mo_crc32(poop, 6, 0);

    received_crc = poop[6] | ((uint32_t) poop[7] << 8) | ((uint32_t) poop[8] << 16) | ((uint32_t) poop[9] << 24);
    if (received_crc != crc) {
      dbprintf("bat_scan: ERROR: calculated crc 0x%08x, received crc 0x%08x\n", crc, received_crc);
      bat_status_valid = 0;
      return;
      }

    br->battery_voltage_adc_status = ((uint16_t)poop[1] << 8) | (uint16_t)poop[0];
    br->right_voltage_adc = ((uint16_t)poop[3] << 8) | (uint16_t)poop[2];
    br->battery_vadc1 = ((uint16_t)poop[5] << 8) | (uint16_t)poop[4];

    if (bat_noise)
      dbprintf("bat_scan: %s: bup %d rup %d bat_voltage %.2f r_voltage %.2f int_voltage %.2f\n", 
        (BAT_RELAY_ON_RIGHT(br)) ? "  RIGHT" : "BATTERY",
        BAT_BATTERY_UP(br), BAT_RIGHT_UP(br), (float)(BAT_VOLTAGE(br->battery_voltage_adc_status)),
        (float)(BAT_VOLTAGE(br->right_voltage_adc)), (float)(BAT_VOLTAGE(br->battery_vadc1)));

    bat_status_valid = 1;
  }
}

void *bat_thread(void *marg)
{
  struct termios term;
  int sts;
  struct timespec ts;
  static uint8_t first = 1;

  bat_response_t *br = &bat_status;

  dbprintf("bat_thread: started\n");
  // Open the serial port
  bat_fd = open(bat_device, O_RDWR | O_NOCTTY);
  if (bat_fd < 0) {
    dbprintf("bat: ERROR opening serial port %s\n", bat_device);
    return NULL ;
    }
  else {
    if (bat_noise)
      printf("bat: opened serial port %s OK\n", bat_device);
  }

  sts = tcgetattr(bat_fd,&term);
  if (sts < 0) {
    fprintf(stderr, "bat: Error %d (%s) from tcgetattr\n", errno, strerror(errno));
    return NULL;
  }
  sts = tcgetattr(bat_fd,&saved_bat_attributes);
  if (first) {
    first = 0;
    atexit(restore_bat_attributes);
    }
  cfmakeraw(&term);

  cfsetispeed(&term, BAUDRATE);
  cfsetospeed(&term, BAUDRATE);

  if (tcsetattr(bat_fd,TCSANOW,&term) != 0) {
    fprintf(stderr, "bat: Error %d (%s) from tcsetattr\n", errno, strerror(errno));
    return NULL;
  }
  tcflush(bat_fd, TCIOFLUSH);

  if ((sts = clock_gettime(CLOCK_MONOTONIC, &ts)) < 0) {
    fprintf(stderr, "bat: Problem with clock_gettime(), returned %d (%s)\n", sts, strerror(sts));
    return 0;
  }

  // Loop forever servicing requests
  while (!mo_abort) {
    set_timeout(&ts, 1000000);
    pthread_mutex_lock(&bat_lock);
    sts = pthread_cond_timedwait(&bat_wait, &bat_lock, &ts);
    if (sts == ETIMEDOUT) {
      pthread_mutex_unlock(&bat_lock);
      // Do an echo to keep an eye on things
      bat_scan(bat_fd, br);
      continue;
      }
    else if (sts) {
      pthread_mutex_unlock(&bat_lock);
      dbprintf("bat_thread: unexpected result from pthread-cond-timedwait: %d\n", sts);
      perror("cond-wait in bat_thread()");
      pthread_mutex_unlock(&bat_lock);
      break;
      }
    else
      pthread_mutex_unlock(&bat_lock);
    }

  dbprintf("bat_thread: Done\n");
  close(bat_fd);
  return NULL;
}
#endif

////////////////////////////////////////////////////////////////////////////////
// The thread that talks to the ETX1200 fault pins
////////////////////////////////////////////////////////////////////////////////

#define ETX_HISTORY_SIZE    4
#define TWO_MINS_45_SECS    ((3 * 60 * 10) - (15 * 10))
#define ETX_NOISE_WAIT_TIME 10*60*1000       /* 10 minutes in msec */

static int check_etx_blink(uint32_t his[ETX_HISTORY_SIZE])
{
  uint32_t his_count[ETX_HISTORY_SIZE];

  for (int i = 0; i < ETX_HISTORY_SIZE; i++)
    his_count[i] = his[i] & 0xffff;

  if (10 <= his_count[0] && his_count[0] <= 30)
    if (10 <= his_count[1] && his_count[1] <= 30) {
      return(ETX_BLINK_2SEC);
    }

  if (31 <= his_count[0] && his_count[0] <= 70)
    if (31 <= his_count[1] && his_count[1] <= 70) {
      return(ETX_BLINK_5SEC);
    }

  return(ETX_NO_BLINKING);
}

_Atomic uint8_t left_battery_fault = ETX_NO_BLINKING;
_Atomic uint8_t right_battery_fault = ETX_NO_BLINKING;

void *etx_thread(void *marg)
{
  static uint8_t etx_first_start_done = 0;
  static uint8_t last_left_battery_alarm, last_right_battery_alarm;
  static uint32_t left_noise_limit, right_noise_limit;
  const char *fault_text[] = {"No fault", "High temperature", "Charge fault", "BMS hardware fault"};
  const char *fmt1 = "%s battery back online after over current protection fault";

  uint8_t left_battery_alarm, right_battery_alarm;
  uint32_t left_history[ETX_HISTORY_SIZE] = {0};
  uint32_t right_history[ETX_HISTORY_SIZE] = {0};
  int sts;
  struct timespec ts;

  int left_history_size = 0;
  int right_history_size = 0;
  uint32_t left_consec = 0, right_consec = 0;
  char buf[256];

  dbprintf("etx_thread: started\n");

  while (!mo_abort && !etx_thread_abort) {
    if (etx_first_start_done == 0) {
      // Delay for a while on startup
      for (int i = 0; i < 300 && !mo_abort && !etx_thread_abort; i++)
        mo_usleep(100000);
      etx_first_start_done = 1;
    }
    set_timeout(&ts, 100000);           // 100 msec
    pthread_mutex_lock(&etx_lock);
    sts = pthread_cond_timedwait(&etx_wait, &etx_lock, &ts);
    if (sts == ETIMEDOUT) {

#ifdef ARM
  #ifdef USE_LGPIO
      // Note: digital inputs are inverted in hardware
      left_battery_alarm = (lgGpioRead(gpio_handle, LEFT_BATTERY_ALARM_PIN)) ? 0 : 1;
      right_battery_alarm = (lgGpioRead(gpio_handle, RIGHT_BATTERY_ALARM_PIN)) ? 0 : 1;
  #else
      left_battery_alarm =  digitalRead(LEFT_BATTERY_ALARM_PIN);
      right_battery_alarm =  digitalRead(RIGHT_BATTERY_ALARM_PIN);
  #endif
#else
      left_battery_alarm = 0;
      right_battery_alarm = 0;
#endif

      if (left_battery_alarm != last_left_battery_alarm) {
        // Change of state. Prepare a possible history entry
        left_consec &= 0xffff;          // Truncate

        if (last_left_battery_alarm && left_consec > 71) {
          // Fault wire went low, after being on solid for > 7.1 seconds
          if (left_consec > TWO_MINS_45_SECS) {
            // > 2 minutes 45 seconds. Manual says 3 minutes
            snprintf(buf, sizeof(buf), fmt1, "Left");
            dbprintf("%s\n", buf);
            tts_say(buf);
          }
          left_battery_fault = ETX_NO_BLINKING;
          left_consec = 0;                          // Start again
          left_history_size = 0;
          goto ldone;
        }

        // Log the last pulse in history
        left_consec |= last_left_battery_alarm << 31;
        for (int i = ETX_HISTORY_SIZE-1; i > 0; i--)
          left_history[i] = left_history[i-1];
        left_history[0] = left_consec;
        left_history_size++;
        left_consec = 0;
        if (left_history_size > 2)
          left_battery_fault = check_etx_blink(left_history);
        } else {
          left_consec++;        // Consecutive 100 msec samples in same state
          if (left_consec > 71)
            left_battery_fault = (left_battery_alarm) ? ETX_SOLID: ETX_NO_BLINKING;
        }
ldone:
      if (right_battery_alarm != last_right_battery_alarm) {
        // Change of state. Prepare a possible history entry
        right_consec &= 0xffff;          // Truncate

        if (last_right_battery_alarm && right_consec > 71) {
          // Fault wire went low, after being on solid for > 7.1 seconds
          if (right_consec > TWO_MINS_45_SECS) {
            // > 2 minutes 45 seconds. Manual says 3 minutes
            snprintf(buf, sizeof(buf), fmt1, "Right");
            dbprintf("%s\n", buf);
            tts_say(buf);
          }
          right_battery_fault = ETX_NO_BLINKING;
          right_consec = 0;                         // Start again
          right_history_size = 0;
          goto rdone;
        }

        // Log the last pulse in history
        right_consec |= last_right_battery_alarm << 31;
        for (int i = ETX_HISTORY_SIZE-1; i > 0; i--)
          right_history[i] = right_history[i-1];
        right_history[0] = right_consec;
        right_history_size++;
        right_consec = 0;
        if (right_history_size > 2)
          right_battery_fault = check_etx_blink(right_history);
        } else {
          right_consec++;        // Consecutive 100 msec samples in same state
          if (right_consec > 71)
            right_battery_fault = (right_battery_alarm) ? ETX_SOLID: ETX_NO_BLINKING;
        }
rdone:

      last_left_battery_alarm = left_battery_alarm;
      last_right_battery_alarm = right_battery_alarm;

      // Audio alarms
      if (left_battery_fault != ETX_NO_BLINKING) {
        if (!left_noise_limit || msec_tick >= left_noise_limit) {
          snprintf(buf, sizeof(buf), "Left  main battery %s", fault_text[left_battery_fault]);
          dbprintf("%s\n", buf);
          tts_say(buf);
          left_noise_limit = msec_tick + ETX_NOISE_WAIT_TIME;
          }
      }
      if (right_battery_fault != ETX_NO_BLINKING) {
        if (!right_noise_limit || msec_tick >= right_noise_limit) {
          snprintf(buf, sizeof(buf), "Right main battery %s", fault_text[right_battery_fault]);
          dbprintf("%s\n", buf);
          tts_say(buf);
          right_noise_limit = msec_tick + ETX_NOISE_WAIT_TIME;
          }
      }

      pthread_mutex_unlock(&etx_lock);
      } else {
        pthread_mutex_unlock(&etx_lock);
      }
    }

  dbprintf("etx_thread: Done\n");
  return NULL;
}


#ifdef INCLUDE_FLAPS
////////////////////////////////////////////////////////////////////////////////
// Get a frame, with timeouts
////////////////////////////////////////////////////////////////////////////////

static int flaps_get_frame(int fd, struct efi_frame *h)
{
  int read_amount = sizeof(h->header);
  int total_amount;
  uint8_t *result = (uint8_t *)h;
  struct timeval timeout;
  ssize_t ret = 0;
  uint32_t fcrc;
  fd_set rd;

  // Read 1 byte until we get a valid preamble
  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (unlikely(ret < 0))
      {
      dbprintf("flaps_get_frame: Error %d on select in flaps_get_header, for fd %d\n", errno, fd);
      perror("flaps_get_frame: ooops");
      return(ret);
      }
    if (!ret)
      return(0);                    // Timeout
    ret = read(fd, result, 1);      // Read 1 byte
    if (unlikely(ret <= 0))
      return(ret);                  // Error or timeout
    } while (*result != EFI_PREAMBLE);

  // Found a preamble byte, now read the rest of the frame header
  result++;
  read_amount--;
  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (unlikely(ret <= 0))
      return(ret);                                  // Error or timeout

    ret = read(fd, result, read_amount);
    if (unlikely(ret < 0))
      {
      dbprintf("flaps: Error %d on read in flaps_get_header", errno);
      //applog(LOG_WARNING, "flaps: Error %d on read in flaps_get_header", errno);
      return(ret);
      }
    result += ret;
    read_amount -= ret;
  } while (read_amount > 0);

  // Check the header CRC and status
  fcrc = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);
  if (fcrc != h->header.crc32) {
    dbprintf("flaps_get_frame: Bad header checksum fcrc 0x%08x vs 0x%08x\n", fcrc, h->header.crc32);
    efi_dump_frame(h, "FROM", sizeof(struct efi_header), 0);
    return(EBADHEADERCRC);
    }

  if (h->header.status != 0) {
    dbprintf("flaps_get_frame: Bad header status %d (0x%x)\n", h->header.status, h->header.status);
    efi_dump_frame(h, "FROM", sizeof(struct efi_header), 0);
    return(EBADHEADERSTS);
    }

  // If there's a data frame, get it
  switch (h->header.opcode) {
    case OP_ECHO:
      return(sizeof(h->header));            // No data, return
      break;

    case OP_FLAP:
      return(sizeof(h->header));            // No data, return
      break;

    case OP_BPOWER:
      //dbprintf("flaps: OP_BPOWER\n");
      read_amount = sizeof(struct bpower_data) + 4;
      break;

    default:
      return(sizeof(h->header));            // No data, return
      break;
  }

  total_amount = sizeof(h->header);

  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (ret <= 0) {
      //dbprintf("flaps: ret %ld from select \n", (long int)ret);
      return(ret);                                  // Error or timeout
      }

    ret = read(fd, result, read_amount);
    if (unlikely(ret < 0))
      {
      dbprintf("flaps: ret %ld from read attempt of %d\n", (long int)ret, read_amount);
      perror("read() in flaps_get_frame");
      return(ret);
      }
    result += ret;
    total_amount += ret;
    read_amount -= ret;
  } while (read_amount > 0);

  // Compute the data CRC, and check it out
  fcrc = mo_crc32((unsigned char *)h->buffer, total_amount - sizeof(h->header) - 4, 0);
  result -= sizeof(uint32_t);
  if (fcrc != *(uint32_t *)result) {
    dbprintf("ERROR: Bad data CRC 0x%08x, expected 0x%08x from total_amount %d\n", *(uint32_t *)result, fcrc, total_amount);
    efi_dump_frame(h, "FROM", total_amount - sizeof(h->header), 1);
    return(EBADDATACRC);
    }

  return(total_amount - 4);
}

////////////////////////////////////////////////////////////////////////////////
// Do a transaction with the Flaps power box.
// opcode, opdata* fields filled in.
////////////////////////////////////////////////////////////////////////////////

static void do_flaps_transaction(int fd, struct efi_header *h, struct efi_frame *f, int len)
{
  struct timespec begin;
  //struct timespec end;
  //static uint8_t sequence = 1;
  static uint8_t consecutive_timeouts = 0;
  int sts;
  int tx_len = (len) ? len : sizeof(struct efi_header);

  h->preamble = EFI_PREAMBLE;
  //h->sequence = sequence++;
  h->status = 0;
  h->crc32 = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);

  if (flaps_noise && verbose)
    dbprintf("do_flaps_transaction: entered\n");

  if (flaps_noise && verbose) {
    if (tx_len > sizeof(struct efi_header))
      efi_dump_frame((struct efi_frame *)h, "TO", tx_len - sizeof(struct efi_header), 1);
    else
      efi_dump_frame((struct efi_frame *)h, "TO", 0, 0);
    }

  if (tx_len > sizeof(struct efi_header)) {
    // Add data CRC
    mo_crc32((unsigned char *)(h+1), tx_len - sizeof(struct efi_header), 1);
    tx_len += 4;
  }

  clock_gettime(CLOCK_MONOTONIC, &begin);
  if (write_full(fd, (char *)h, tx_len)) {
    dbprintf("do_flaps_transaction: ERROR: write failed for %d bytes\n", tx_len);
    flaps_thread_abort = 1;                   // Bad connection, restart thread
    flaps_data_valid = 0;
    return;
    }

  sts = flaps_get_frame(fd, f);
  if (sts > 0) {
    consecutive_timeouts = 0;

    //clock_gettime(CLOCK_MONOTONIC, &end);
    //long seconds = end.tv_sec - begin.tv_sec;
    //long nanoseconds = end.tv_nsec - begin.tv_nsec;
    //double elapsed_msec = seconds*1e3 + nanoseconds*1e-6;

    if (flaps_noise && verbose) {
      efi_dump_frame(f, "FROM", sts - sizeof(struct efi_header), 1);
      printf("\n");
      }
    }

    if (sts >= sizeof(*h)) {
      if (f->header.opcode == OP_BPOWER) {
        flaps_data_valid = 1;
        }
      else if (f->header.opcode == OP_FLAP_SETTINGS) {
        flaps_calibrated = 1;
        }
    } else {
      if (++consecutive_timeouts == 5) {
        consecutive_timeouts = 0;
        flaps_thread_abort = 1;
        flaps_data_valid = 0;
        }
      if (sts == 0) {
        dbprintf("flaps: TIMEOUT\n");
        }
      else {
        dbprintf("flaps: read returned bad status %d\n", sts);
        }
    }
}

void flaps_echo()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &flaps_rx_frame;

  h->opcode = OP_ECHO;
  h->opdata1 = 0;
  h->opdata2 = 0;

  do_flaps_transaction(flaps_fd, h, f, sizeof(struct efi_header));
}

void flaps_op_flap(uint32_t op1)
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &flaps_rx_frame;

  h->opcode = OP_FLAP;
  h->opdata1 = op1;
  h->opdata2 = 0;

  do_flaps_transaction(flaps_fd, h, f, sizeof(struct efi_header));
}

void flaps_put_flap_settings(void)
{
  if (!flaps_active)
    return;

  if (flaps_noise)
    dbprintf("flaps_put_flap_settings: entered\n");
  pthread_mutex_lock(&flaps_lock);
  flaps_put_settings = 1;
  ACTIVATE_FLAPS_THREAD(flaps_put_settings);
  if (flaps_noise)
    dbprintf("flaps_put_flap_settings: returning\n");
}


void flaps_op_flap_settings()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &flaps_rx_frame;

  h->opcode = OP_FLAP_SETTINGS;
  h->sequence = FLAP_SETTINGS_PUT | (flaps.deadband >> 2) | ((flaps_sensor_reverse) ? FLAP_SENSOR_REVERSED : 0);
  h->opdata1 = flaps.reflex_position | ((uint32_t)flaps.zero_position << 16);
  h->opdata2 = flaps.half_position | ((uint32_t)flaps.full_position << 16);

  dbprintf("FLAPS: R=%1d: reflex %d zero %d half %d full %d deadband %d\n",
    flaps_sensor_reverse, flaps.reflex_position, flaps.zero_position,
        flaps.half_position, flaps.full_position, flaps.deadband);

  do_flaps_transaction(flaps_fd, h, f, sizeof(struct efi_header));
}


void flaps_request_data(void)
{
  if (!flaps_active)
    return;

  pthread_mutex_lock(&flaps_lock);
  flaps_get_data = 1;
  ACTIVATE_FLAPS_THREAD(flaps_get_data);
}

void flaps_bpower()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &flaps_rx_frame;

  h->opcode = OP_BPOWER;
  h->opdata1 = 0;
  h->opdata2 = 0;

  do_flaps_transaction(flaps_fd, h, f, sizeof(struct efi_header));
}


static void make_flaps_fake_data(void)
{
    struct bpower_data *f = flaps_data;

    f->primary_voltage = 1600 + (rand() & 0x1f);
    f->secondary_voltage = 1600 + (rand() & 0x1f);
    f->battery_voltage = 1600 + (rand() & 0x1f);
    f->current_flap_position = 1000;
    f->flap_motor_current = 1000;
    f->flags = 0x7f;

    flaps_data_valid = 1;
}


////////////////////////////////////////////////////////////////////////////////
// The thread that talks to the Flap controller and redundant avionics power board
////////////////////////////////////////////////////////////////////////////////

void *flaps_thread(void *marg)
{
  struct termios term;
  int sts;
  struct timespec ts;
  static uint8_t premsg = 1;
  static uint8_t first = 1;

  while (!mo_abort) {
    if (flaps_noise)
      dbprintf("flaps_thread: started/restarted\n");

    // Open the serial port
    while (!mo_abort) {
      flaps_fd = open(flaps_device, O_RDWR | O_NOCTTY);
      if (flaps_fd < 0) {
        if (premsg) {
          if (flaps_noise) {
            dbprintf("flaps: ERROR: Opening serial port %s (%s)\n", flaps_device, strerror(errno));
            }
          premsg = 0;
          }
        if (local_fake_data_ok > 0)
          make_flaps_fake_data();
        mo_usleep(100000);
        }
      else {
        if (flaps_noise)
          dbprintf("flaps: Opened serial port %s OK\n", flaps_device);
        break;
      }
    }
    if (mo_abort)
      break;

    sts = tcgetattr(flaps_fd,&term);
    if (sts < 0) {
      dbprintf("flaps: Error %d (%s) from tcgetattr\n", errno, strerror(errno));
      close(flaps_fd);
      mo_usleep(50000);
      continue;
    }
    sts = tcgetattr(flaps_fd, &saved_flaps_attributes);
    if (first) {
      first = 0;
      atexit(restore_flaps_attributes);
      }
    cfmakeraw(&term);

    cfsetispeed(&term, BAUDRATE);
    cfsetospeed(&term, BAUDRATE);

    if (tcsetattr(flaps_fd,TCSANOW,&term) != 0) {
      dbprintf("flaps: Error %d (%s) from tcsetattr\n", errno, strerror(errno));
      return NULL;
    }
    tcflush(flaps_fd, TCIOFLUSH);
    clock_gettime(CLOCK_MONOTONIC, &ts);

    flaps_active = 1;

    // Loop forever servicing requests
    while (!mo_abort && !flaps_thread_abort) {
      set_timeout(&ts, 200000);           // 200 msec
      pthread_mutex_lock(&flaps_lock);
      if (!(flaps_put_settings || flaps_put_action || flaps_get_data))
        sts = pthread_cond_timedwait(&flaps_req_cond, &flaps_lock, &ts);
      else
        sts = 0;
      if (sts == ETIMEDOUT) {
        pthread_mutex_unlock(&flaps_lock);
        //flaps_bpower();
        continue;
        }
      else if (!sts) {
        while (flaps_put_settings || flaps_put_action || flaps_get_data) {
          // Someone has requested something
          //dbprintf("flaps: TOP: put_settings %d put_action %d get_data %d\n", flaps_put_settings, flaps_put_action, flaps_get_data);
          if (flaps_put_settings) {
            pthread_mutex_unlock(&flaps_lock);
            flaps_op_flap_settings();
            pthread_mutex_lock(&flaps_lock);
            flaps_put_settings = 0;
            }
          else if (flaps_put_action) {
            pthread_mutex_unlock(&flaps_lock);
            flaps_op_flap(flaps_put_action & 0x7fffffff);
            pthread_mutex_lock(&flaps_lock);
            flaps_put_action = 0;
            }
          else if (flaps_get_data) {
            pthread_mutex_unlock(&flaps_lock);
            flaps_bpower();
            pthread_mutex_lock(&flaps_lock);
            flaps_get_data = 0;
            }
          pthread_cond_broadcast(&flaps_done_cond);
          }
        //dbprintf("flaps: BOT: put_settings %d put_action %d get_data %d\n", flaps_put_settings, flaps_put_action, flaps_get_data);
      } else {
        dbprintf("flaps_thread: unexpected result from pthread-cond-timedwait: %d (%s)\n", sts, strerror(sts));
        pthread_mutex_unlock(&flaps_lock);
        break;
      }
      pthread_mutex_unlock(&flaps_lock);
    }

  close(flaps_fd);
  flaps_active = 0;
  flaps_thread_abort = 0;
  flaps_fd = -1;
  premsg = 1;
  mo_usleep(50000);
  }
  dbprintf("flaps_thread: Done\n");
  return(0);
}
#endif


/*
 * Rules:
 * - options.vib_source == VIB_ACCEL_SOURCE_BOTH:
 *       metrics[0] = IIS data
 *       metrics[1] = ADXL data
 * - otherwise:
 *       metrics[0] = selected source
 */

static vibsense_live_config_t vib_cfg;

int vib_get_data_for_display(vibsense_live_metrics_t *metrics)
{
  vibsense_window_t win;
  vibsense_window_t wins[2];

  if (!metrics)
    return(0);

  memset(&metrics[0], 0, sizeof(metrics[0]));
  memset(&metrics[1], 0, sizeof(metrics[1]));

  if (vib_cfg.display_full_scale_ips == 0) {    // Do once
    vibsense_live_default_config(&vib_cfg);
    vib_cfg.display_full_scale_ips = 0.25;
    vibsense_fake_reset();
    }

  if (local_fake_data_ok) {
    if (options.vib_source == VIB_ACCEL_SOURCE_BOTH) {
      if (vibsense_fake_fill_both_windows(wins) != 0)
        return(0);

      if (vibsense_live_analyze_window(&wins[0], &vib_cfg, &metrics[0]) != VIBSENSE_LIVE_OK)
        return(0);

      if (vibsense_live_analyze_window(&wins[1], &vib_cfg, &metrics[1]) != VIBSENSE_LIVE_OK)
        memset(&metrics[1], 0, sizeof(metrics[1]));

      return(1);
      }

    uint8_t src = options.vib_source;
    if (src != VIB_ACCEL_SOURCE_ADXL355 && src != VIB_ACCEL_SOURCE_IIS3DWBG1)
      src = VIB_ACCEL_SOURCE_IIS3DWBG1;

    if (vibsense_fake_fill_window_source(&win, src) != 0)
      return(0);

    if (vibsense_live_analyze_window(&win, &vib_cfg, &metrics[0]) != VIBSENSE_LIVE_OK)
      return(0);

    return(1);
  } else {
  /*
   * Real-data path
   * - If a single source is active, fill/analyze metrics[0].
   * - If both are active, fill/analyze metrics[0] from IIS and metrics[1] from ADXL.
   */
  pthread_mutex_lock(&vib_lock);
  if (vib_window_updated != last_vib_window_updated) {
    // There's new data available. 
    last_vib_window_updated = vib_window_updated;
    if (vibsense_live_analyze_window(&vib_window1, &vib_cfg, &metrics[0]) != VIBSENSE_LIVE_OK) {
      pthread_mutex_unlock(&vib_lock);
      return(0);
      }
    if (options.vib_source == VIB_ACCEL_SOURCE_BOTH) {
      if (vibsense_live_analyze_window(&vib_window2, &vib_cfg, &metrics[1]) != VIBSENSE_LIVE_OK) {
        pthread_mutex_unlock(&vib_lock);
        return(0);
        }
      }
    }
  pthread_mutex_unlock(&vib_lock);
  return(1);
  }
}


static void rotate_vib_options(int up)
{
  int new = options.vib_source;

  if (up) {
    if (rev_adjust_mode)
      vib_rotations++;
    else
      switch(options.vib_source) {
        case 1: new = 2; break;
        case 2: new = 3; break;
        case 3: new = 1; break;
        default: break;
        }
  } else {
    if (rev_adjust_mode) {
      if (vib_rotations > 1)
        --vib_rotations;
      }
    else
      switch(options.vib_source) {
        case 1: new = 3; break;
        case 2: new = 1; break;
        case 3: new = 2; break;
        default: break;
        }
  }

  options.vib_source = new;
}


void vib_reset()
{
  struct efi_header *h = &vib_tx_frame.header;
  struct efi_frame *f = &vib_rx_frame;
  int len = sizeof(struct efi_header);

  if (vib_noise)
    dbprintf("vib_reset(): entered\n");
  h->opcode = OP_VIB;
  h->opdata1 = VIB_OP_RESET;
  h->opdata2 = 0;

  do_vib_transaction(vib_fd, h, f, len);
}

void vib_get_event()
{
  struct efi_header *h = &vib_tx_frame.header;
  struct efi_frame *f = &vib_rx_frame;
  int len = sizeof(struct efi_header);

  if (vib_noise)
    dbprintf("vib_get_event(): entered\n");

  h->opcode = OP_VIB;
  h->opdata1 = VIB_OP_GET_EVENT;
  h->opdata2 = 0;

  do_vib_transaction(vib_fd, h, f, len);
}


////////////////////////////////////////////////////////////////////////////////
// Get a frame, with timeouts
////////////////////////////////////////////////////////////////////////////////

static int vib_get_frame(int fd, struct efi_frame *h)
{
  int read_amount = sizeof(h->header);
  int total_amount;
  uint8_t *result = (uint8_t *)h;
  struct timeval timeout;
  ssize_t ret = 0;
  uint32_t fcrc;
  fd_set rd;

  // Read 1 byte until we get a valid preamble
  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (unlikely(ret < 0))
      {
      dbprintf("vib_get_frame: Error %d on select in vib_get_header, for fd %d\n", errno, fd);
      perror("vib_get_frame: ooops");
      return(ret);
      }
    if (!ret)
      return(0);                    // Timeout
    ret = read(fd, result, 1);      // Read 1 byte
    if (unlikely(ret <= 0))
      return(ret);                  // Error or timeout
    } while (*result != EFI_PREAMBLE);

  // Found a preamble byte, now read the rest of the frame header
  result++;
  read_amount--;
  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (unlikely(ret <= 0))
      return(ret);                                  // Error or timeout

    ret = read(fd, result, read_amount);
    if (unlikely(ret < 0))
      {
      dbprintf("vib: Error %d on read in vib_get_header", errno);
      //applog(LOG_WARNING, "vib: Error %d on read in vib_get_header", errno);
      return(ret);
      }
    result += ret;
    read_amount -= ret;
  } while (read_amount > 0);

  // Check the header CRC and status
  fcrc = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);
  if (fcrc != h->header.crc32) {
    dbprintf("vib_get_frame: Bad header checksum fcrc 0x%08x vs 0x%08x\n", fcrc, h->header.crc32);
    efi_dump_frame(h, "FROM", sizeof(struct efi_header), 0);
    return(EBADHEADERCRC);
    }

  if (h->header.status != 0) {
    dbprintf("vib_get_frame: Bad header status %d (0x%x)\n", h->header.status, h->header.status);
    efi_dump_frame(h, "FROM", sizeof(struct efi_header), 0);
    return(EBADHEADERSTS);
    }

  // If there's a data frame, get it
  switch (h->header.opcode) {
    case OP_ECHO:
      return(sizeof(h->header));            // No data, return
      break;

    case OP_VIB:
      read_amount = h->header.opdata2 + 4;  // Header CRC is correct at this point, so length is good
      if (read_amount == 4)
        return(sizeof(h->header));            // No data, return
      if (read_amount < 4 || read_amount > MAX_EFI_FRAME_DATA_SIZE)
        return(sizeof(h->header));            // No data, return
      break;

    default:
      return(sizeof(h->header));            // No data, return
      break;
  }

  total_amount = sizeof(h->header);

  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (ret <= 0) {
      //dbprintf("vib: ret %ld from select \n", (long int)ret);
      return(ret);                                  // Error or timeout
      }

    ret = read(fd, result, read_amount);
    if (unlikely(ret < 0))
      {
      dbprintf("vib: ret %ld from read attempt of %d\n", (long int)ret, read_amount);
      perror("read() in vib_get_frame");
      return(ret);
      }
    result += ret;
    total_amount += ret;
    read_amount -= ret;
  } while (read_amount > 0);

  // Compute the data CRC, and check it out
  fcrc = mo_crc32((unsigned char *)h->buffer, total_amount - sizeof(h->header) - 4, 0);
  result -= sizeof(uint32_t);
  if (fcrc != *(uint32_t *)result) {
    dbprintf("ERROR: Bad data CRC 0x%08x, expected 0x%08x from total_amount %d\n", *(uint32_t *)result, fcrc, total_amount);
    efi_dump_frame(h, "FROM", total_amount - sizeof(h->header), 1);
    return(EBADDATACRC);
    }

  return(total_amount - 4);
}

////////////////////////////////////////////////////////////////////////////////
// Do a transaction with the Flaps power box.
// opcode, opdata* fields filled in.
////////////////////////////////////////////////////////////////////////////////

static void do_vib_transaction(int fd, struct efi_header *h, struct efi_frame *f, int len)
{
  struct timespec begin;
  //struct timespec end;
  //static uint8_t sequence = 1;
  static uint8_t consecutive_timeouts = 0;
  int sts;
  int rc;
  int tx_len = (len) ? len : sizeof(struct efi_header);

  h->preamble = EFI_PREAMBLE;
  //h->sequence = sequence++;
  h->status = 0;
  h->crc32 = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);

  if (vib_noise)
    dbprintf("do_vib_transaction: entered\n");

  if (vib_noise && verbose) {
    if (tx_len > sizeof(struct efi_header))
      efi_dump_frame((struct efi_frame *)h, "TO", tx_len - sizeof(struct efi_header), 1);
    else
      efi_dump_frame((struct efi_frame *)h, "TO", 0, 0);
    }

  if (tx_len > sizeof(struct efi_header)) {
    // Add data CRC
    mo_crc32((unsigned char *)(h+1), tx_len - sizeof(struct efi_header), 1);
    tx_len += 4;
  }

  clock_gettime(CLOCK_MONOTONIC, &begin);
  if (write_full(fd, (char *)h, tx_len)) {
    dbprintf("do_vib_transaction: ERROR: write failed for %d bytes\n", tx_len);
    vib_thread_abort = 1;                   // Bad connection, restart thread
    vib_data_valid = 0;
    return; }

  sts = vib_get_frame(fd, f);
  if (sts > 0) {
    consecutive_timeouts = 0;

    //clock_gettime(CLOCK_MONOTONIC, &end);
    //long seconds = end.tv_sec - begin.tv_sec;
    //long nanoseconds = end.tv_nsec - begin.tv_nsec;
    //double elapsed_msec = seconds*1e3 + nanoseconds*1e-6;

    if (vib_noise && verbose) {
      efi_dump_frame(f, "FROM", sts - sizeof(struct efi_header), 1);
      printf("\n");
      }
    }

    if (sts >= sizeof(*h)) {
      if (f->header.opcode == OP_VIB) {
        //vib_payload_header_t *vph = &f->vib_payload_header;

        switch (f->header.opdata1 & 0xff) {
          case VIB_OP_ECHO:
            break;

          case VIB_OP_RESET:
            vibsense_arm_sources(ctx, vib_rotations, options.vib_source);   // Arm libsense
            vib_get_next_event = 1;
            break;

          case VIB_OP_ACCEL:
            break;

          case VIB_OP_GET_EVENT:
            char *vibtype[] = {"status", "accel", "tach", "tach debug", "snapshot"};
            if (vib_noise)
              dbprintf("VIB_OP_GET_EVENT: event type %d (%s)\n", f->vib_payload_header.type, vibtype[f->vib_payload_header.type]);

            rc = vibsense_ingest_payload(ctx, &f->vib_payload_header, sts - sizeof(f->header));      // CRC already dropped above
            if (rc == VIBSENSE_DONE) {
              if (vib_noise) {
                vib_get_next_event = 0;
                dbprintf("VIBSENSE_DONE\n");
                }
              // The desired window(s) is/are available, update local data
              switch (options.vib_source) {
                case VIB_ACCEL_SOURCE_IIS3DWBG1:
                  vibsense_get_window_by_source(ctx, VIB_ACCEL_SOURCE_IIS3DWBG1, &vib_window1);
                  break;

                case VIB_ACCEL_SOURCE_ADXL355:
                  vibsense_get_window_by_source(ctx, VIB_ACCEL_SOURCE_ADXL355, &vib_window1);
                  break;

                case VIB_ACCEL_SOURCE_BOTH:
                  vibsense_get_window_by_source(ctx, VIB_ACCEL_SOURCE_IIS3DWBG1, &vib_window1);
                  vibsense_get_window_by_source(ctx, VIB_ACCEL_SOURCE_ADXL355, &vib_window2);
                  vib_window_updated++;
                  break;

                default:
                  break;
                }
              }
            else if (rc == VIBSENSE_OK) {
              vib_get_next_event = 1;               // Keep getting more events
              if (vib_noise) {
                dbprintf("VIBSENSE_OK\n");
                }
              if (vib_write_snapshot) {
                // XXX
                }

              }
            else if (rc < 0) {
              dbprintf("OP_VIB: VIB_OP_GET_EVENT: vibsense_ingest_payload() returned error code %d\n", rc);
              }
            break;

          case VIB_OP_GET_STATUS:
            break;

          case VIB_OP_GET_CONFIG:
            break;

          case VIB_OP_DEBUG:
            break;

          default:
            break;
          }
        }
    } else {
      if (++consecutive_timeouts == 5) {
        consecutive_timeouts = 0;
        vib_thread_abort = 1;
        vib_data_valid = 0;
        }
      if (sts == 0) {
        dbprintf("vib: TIMEOUT\n");
        }
      else {
        dbprintf("vib: read returned bad status %d\n", sts);
        }
    }
}

void vib_echo()
{
  struct efi_header header, *h = &header;
  struct efi_frame *f = &vib_rx_frame;

  h->opcode = OP_ECHO;
  h->opdata1 = 0;
  h->opdata2 = 0;

  do_vib_transaction(vib_fd, h, f, sizeof(struct efi_header));
}


////////////////////////////////////////////////////////////////////////////////
// The thread that talks to the Vibration sensor head board
////////////////////////////////////////////////////////////////////////////////

enum {
  VMO_POWERUP = 1,
  VMO_IDLE,
  VMO_RUNNING
  };

void *vib_thread(void *marg)
{
  struct termios term;
  int sts;
  struct timespec ts;
  static uint8_t premsg = 1;
  static uint8_t first = 1;
  static uint8_t vmo_state = VMO_POWERUP;

  while (!mo_abort) {
    if (vib_noise)
      dbprintf("vib_thread: started/restarted\n");

    // Open the serial port
    while (!mo_abort && !vib_thread_abort) {
      vib_fd = open(vib_device, O_RDWR | O_NOCTTY);
      if (vib_fd < 0) {
        if (premsg) {
          if (vib_noise) {
            dbprintf("vib: ERROR: Opening serial port %s (%s)\n", vib_device, strerror(errno));
            }
          premsg = 0;
          }
        mo_usleep(100000);
        }
      else {
        if (vib_noise)
          dbprintf("vib: Opened serial port %s OK\n", vib_device);
        break;
      }
    }
    if (mo_abort || vib_thread_abort)
      break;

    sts = tcgetattr(vib_fd,&term);
    if (sts < 0) {
      dbprintf("vib: Error %d (%s) from tcgetattr\n", errno, strerror(errno));
      close(vib_fd);
      mo_usleep(50000);
      continue;
    }
    sts = tcgetattr(vib_fd,&saved_vib_attributes);
    if (first) {
      first = 0;
      atexit(restore_vib_attributes);
      }
    cfmakeraw(&term);

    cfsetispeed(&term, BAUDRATE);
    cfsetospeed(&term, BAUDRATE);

    if (tcsetattr(vib_fd,TCSANOW,&term) != 0) {
      dbprintf("vib: Error %d (%s) from tcsetattr\n", errno, strerror(errno));
      return NULL;
    }
    tcflush(vib_fd, TCIOFLUSH);
    clock_gettime(CLOCK_MONOTONIC, &ts);

    //if (!(vib_write_config || vib_read_config))
    //  vib_config(NULL);               // Read the configuration data

    // Loop forever servicing requests
    while (!vib_thread_abort) {
      set_timeout(&ts, 500000);           // Half a second
      pthread_mutex_lock(&vib_lock);
      //if (!vib_get_next_event && (vmo_state == VMO_RUNNING) /* || more here */)
      if (0)
        sts = pthread_cond_timedwait(&vib_wait, &vib_lock, &ts);
      else
        sts = 0;
      if (sts == ETIMEDOUT) {
        pthread_mutex_unlock(&vib_lock);
        // Do a heartbeat thing here, monitor engine vibration even though
        // vibration display is not active.
        //if (ctx) {
        //  }
        continue;
        }
      else if (!sts) {
        switch(vmo_state) {
          case VMO_POWERUP:
            pthread_mutex_unlock(&vib_lock);
            ctx = vibsense_create();                    // Create vibsense context
            if (ctx) {
              if (vib_noise)
                dbprintf("VMO_POWERUP: vibsense context created\n");
              vmo_state = VMO_IDLE;
              }
            break;

          case VMO_IDLE:
            pthread_mutex_unlock(&vib_lock);
            if (engine_is_running) {
              pthread_mutex_unlock(&vib_lock);
              if (vib_noise)
                dbprintf("VMO_IDLE: engine running, calling vib_reset()\n");
              vib_reset();
              vmo_state = VMO_RUNNING;
              }
            else
              vmo_state = VMO_IDLE;
            break;

          case VMO_RUNNING:
            vib_running = 1;
            if (vib_get_next_event) {
              vib_get_next_event = 0;
              if (vib_noise)
                dbprintf("VMO_RUNNING: get next event\n");
              pthread_mutex_unlock(&vib_lock);
              vib_get_event();
            } else {
              pthread_mutex_unlock(&vib_lock);
            }
            // else if () more requests here...
            break;

          default:
            pthread_mutex_unlock(&vib_lock);
            break;
          }

      } else {
        pthread_mutex_unlock(&vib_lock);
        break;
      }
    mo_usleep(50000);            // XXX temp
    }

  //vib_log_close();
  vibsense_destroy(ctx);
  ctx = NULL;
  close(vib_fd);
  vib_do_reset = 1;         // For next time
  vib_fd = -1;
  premsg = 1;
  mo_usleep(50000);
  }

  dbprintf("vib_thread: Done\n");
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// The thread that talks to the HF Radio
////////////////////////////////////////////////////////////////////////////////

//hf_response_t hf_status = {0};
int hf_status_valid = 0;
int hf_put_stuff = 0;
int hf_do_tune = 0;
int hf_tune_on = 0;
int debug_hf_radio = 0;

hf_data_t hf_data = {0};
hf_data_t hf_put_data = {0};

uint8_t hf_get_data(hf_data_t *h)
{
  pthread_mutex_lock(&hf_lock);
  hf_get_status = 1;
  pthread_mutex_unlock(&hf_lock);
  pthread_cond_signal(&hf_wait);

  memcpy(h, &hf_data, sizeof(hf_data));
  return(0);
}

uint8_t hf_operation(hf_data_t *h)
{
  pthread_mutex_lock(&hf_lock);
  memcpy(&hf_put_data, h, sizeof(hf_data));
  hf_put_stuff = 1;
  pthread_mutex_unlock(&hf_lock);
  pthread_cond_signal(&hf_wait);

  // XXX Fix this
  return(0);
}

void hf_tune(int on)
{
  pthread_mutex_lock(&hf_lock);
  hf_do_tune = 1;
  hf_tune_on = on;
  pthread_mutex_unlock(&hf_lock);
  pthread_cond_signal(&hf_wait);
}

static int hf_tx_rx(char *tx, int ntx, char *rx, int nrx)
{
  struct timeval timeout;
  fd_set rd;
  uint8_t *result = (uint8_t *)rx;
  ssize_t ret = 0;
  int nb;

  if (!ntx)
    return(0);

  if (debug_hf_radio)
    printf("HF: PUT:        %s\n", tx);

  write(hf, tx, ntx);

  if (!nrx)                                 // PUT operation
    return(0);

  nb = 0;
  timeout.tv_sec = 0;
  do {
np_again:
    timeout.tv_sec = 0;
    timeout.tv_usec = HF_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(hf, &rd);
    ret = select(hf+1, &rd, NULL, NULL, &timeout);
    if (ret <= 0) {
      //fprintf(stderr, "hf_tx_rx: Error ret %zd from select\n", ret);
      return(ret);                                  // Error or timeout
      }

    ret = read(hf, result, 1);
    if (unlikely(ret < 0))
      {
      fprintf(stderr, "hf_tx_rx: Error ret %zd from read\n", ret);
      //applog(LOG_WARNING, "efi: Error %d on read in efi_get_header", errno);
      return(ret);
      }
    // Filter out non-printables. Some 0xff's can be returned on power-up
    if (!isprint(*result)) {
      dbprintf("hf_tx_rx discarding non-printable character 0x%x\n", *result);
      goto np_again;
    }

    nb++;
  } while (*result++ != ';' && nb < nrx-1);

  *result = '\0';

  if (debug_hf_radio) {
    printf("HF: GET:        %s\n", rx);
  }

  return(nb);
}


int am_test = 1;

int hf_put_operation()
{
  char tx_buf[64]; //, rx_buf[64];
  int nb;

  //printf("hf_put_operation, hf_data.vfoA %d, hf_data.vfoB %d\n", hf_data.vfoA, hf_data.vfoB);
  if (hf_put_data.vfoA != 0) {
    snprintf(tx_buf, sizeof(tx_buf), "FA%011d;MD%d;", hf_put_data.vfoA,
        (am_test) ? 5 : ((hf_put_data.flags & HF_USB) ? 2 : 1));         // Set VFO A Frequency command
    nb = hf_tx_rx(tx_buf, strlen(tx_buf), NULL, 0);
    if (nb < 0)
      return(nb);
    else {
      // Success
      return(nb);
    }
  }

  return(0);
}

void hf_set_tune(int on)
{
  if (on)
    hf_tx_rx("SWH16;", 6, NULL, 0);
  else
    hf_tx_rx("SWT16;", 6, NULL, 0);
}

// Do GETs of the type that return 4 digits, return the number (or an error)
static int hf_poll_resp4(char *cmd)
{
  char rx_buf[64], *r = rx_buf, *p;
  int nb, pa;

  if (*cmd == '^')
    pa = 1;
  else
    pa = 0;

  nb = hf_tx_rx(cmd, strlen(cmd), rx_buf, sizeof(rx_buf));
  //printf("nb = %d, str = %s\n", nb, rx_buf);
  if (nb <= 0)
    return(-1);
  else {
    if (pa) {
      cmd++;        // Skip the '^'
      r++;
      }
    if (*r == *cmd && *(r+1) == *(cmd+1)) {
      r += 2;
      if ((p = strchr(r, ';')))
        *p = '\0';
      else
        return(-1);

      return(atoi(r));
    } else {
      return(-1);
    }
  }
}

static float hf_poll_swr()
{
  char rx_buf[64], *r = rx_buf, *p;
  int nb; // , pa;

  nb = hf_tx_rx("^SW;", 4, rx_buf, sizeof(rx_buf));
  //printf("nb = %d, str = %s\n", nb, rx_buf);
  if (nb < 0)
    return(0.0);
  else {
    r++;        // Skip the '^'
    if (*r == 'S' && *(r+1) == 'W') {
      r += 2;
      if ((p = strchr(r, ';')))
        *p = '\0';
      else
        return(0.0);

      return((float)atof(r)/10.0);
    } else {
      return(0.0);
    }
  }
}


// Poll the HF system, to see what's happening
static int hf_poll_status()
{
  char rx_buf[64];
  int tmp;
  int nb;
  float val;

  if (hf_noise)
    dbprintf("Polling HF Radio\n");

  // Determine if radio is transmitting or not
  nb = hf_tx_rx("TQ;", 3, rx_buf, sizeof(rx_buf));
  if (nb < 0) {
    hf_data.flags &= ~HF_PRESENT;
    return(nb);
  } else {
    //printf("nb = %d, str = %s\n", nb, rx_buf);
    if (rx_buf[0] == 'T' && rx_buf[1] == 'Q') {
      hf_data.flags |= HF_PRESENT;
      if (rx_buf[2] == '1')
        hf_data.flags |= HF_TRANSMITTING;
      else
        hf_data.flags &= ~HF_TRANSMITTING;
      }
      else {
        hf_data.flags &= ~HF_PRESENT;
        return(-1);
      }
    }

  if (hf_data.flags & HF_TRANSMITTING) {

    nb = hf_tx_rx("^OP;", 4, rx_buf, sizeof(rx_buf));
    if (nb < 0) {
      hf_data.flags &= ~HF_PRESENT;
      return(nb);
    } else {
      if (hf_noise)
        dbprintf("nb = %d, str = %s\n", nb, rx_buf);
      if (rx_buf[1] == 'O' && rx_buf[2] == 'P') {
        if (rx_buf[3] == '1') {
          if (hf_noise)
            dbprintf("HF PA is in OPERATE mode\n");
        } else {
          if (hf_noise)
            dbprintf("HF PA is in STANDBY mode\n");
            //dbprintf("HF PA is in STANDBY mode, attempting switch...\n");
            //hf_tx_rx("^OP1;", 5, NULL, 0);
          }
        }
    }

    // Check for a Fault
    nb = hf_tx_rx("^FL;", 4, rx_buf, sizeof(rx_buf));
    if (nb < 0) {
      if (hf_noise)
        dbprintf("No response to ^FL; command\n");
      hf_data.flags &= ~HF_PRESENT;
      return(nb);
    } else {
      if (hf_noise)
        dbprintf("FL command returned \"%s\"\n", rx_buf);
      if (rx_buf[1] == 'F' && rx_buf[2] == 'L') {
        switch (rx_buf[3]) {
          case 'N':
            if (hf_noise)
              dbprintf("     No fault\n");
            break;
          case 'A':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Antenna Tuner mismatch, SWR = %.1f\n", val);
            break;
          case 'C':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Excessive PA drain current, %.1f amps\n", val);
            break;
          case 'D':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Excessive PA power dissipation, %.1f watts\n", val);
            break;
          case 'H':
            val = (float)atoi(&rx_buf[4]) / 1000;
            if (hf_noise)
              dbprintf("     High supply voltage, %.1f volts\n", val);
            break;
          case 'I':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Excessive input power, %.1f watts\n", val);
            break;
          case 'L':
            val = (float)atoi(&rx_buf[4]) / 1000;
            if (hf_noise)
              dbprintf("     Low supply voltage, %.1f volts\n", val);
            break;
          case 'P':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Excessive output power, %.1f watts\n", val);
            break;
          case 'R':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Excessive reflected power, %.1f watts\n", val);
            break;
          case 'S':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Excessive SWR, %.1f\n", val);
            break;
          case 'T':
            val = (float)atoi(&rx_buf[4]) / 10;
            if (hf_noise)
              dbprintf("     Excessive temperature, %.1f degrees C\n", val);
            break;

          default:
            break;
        }
      }
    }

    // Gather transmit information
    if ((tmp = hf_poll_resp4("^PC;")) >= 0) {
      hf_data.pa_current = (float)(tmp) / 10.0;
      if (hf_noise)
        dbprintf("HF PA current is %.1f Amps\n", hf_data.pa_current);
    }
    if ((tmp = hf_poll_resp4("^PD;")) >= 0) {
      hf_data.pa_power_dissipation = (float)(tmp) / 10.0;
      if (hf_noise)
        dbprintf("HF PA power dissipation is %.1f Watts\n", hf_data.pa_power_dissipation);
    }
    if ((tmp = hf_poll_resp4("^PI;")) >= 0) {
      hf_data.input_power = (float)(tmp) / 10.0;
      if (hf_noise)
        dbprintf("HF PA inputpower is %.1f Watts\n", hf_data.input_power);
    }
    if ((tmp = hf_poll_resp4("^PF;")) >= 0) {
      hf_data.forward_power = (float)(tmp) / 10.0;
      if (hf_noise)
        dbprintf("HF PA forward power is %.1f Watts\n", hf_data.forward_power);
    }
    if ((tmp = hf_poll_resp4("^PV;")) >= 0) {
      hf_data.reflected_power = (float)(tmp) / 10.0;
      if (hf_noise)
        dbprintf("HF PA reflected power is %.1f Watts\n", hf_data.reflected_power);
    }
    hf_data.swr = hf_poll_swr();
  } else {
    // Gather receive information
#if 1
    if ((tmp = hf_poll_resp4("SM;")) >= 0) {
      hf_data.s_meter = 10*tmp;
      //dbprintf("HF S-Meter reading is %d\n", hf_data.s_meter);
    }
#endif
#if 0
    if (hf_tx_rx("BG;", 3, rx_buf, sizeof(rx_buf)) >= 0) {
      dbprintf("BG returned %s\n", rx_buf);
      int x = atoi(&rx_buf[2]);
      hf_data.s_meter = 10*x;
    }
#endif
  }


  return(0);
}

void *hf_thread(void *marg)
{
  struct termios term;
  struct timespec ts;
  static uint8_t first = 1;
  int sts;

  //hf_response_t *br = &hf_status;

  if (hf_noise)
    dbprintf("hf_thread: started\n");

  do {
    // Open the serial port
    hf = open(hf_device, O_RDWR | O_NOCTTY);
    if (hf < 0) {
      if (hf_noise)
        dbprintf("hf : ERROR: Opening serial port %s\n", hf_device);
      for (int i = 0; i < 50 && !mo_abort && !hf_thread_abort; i++)
        mo_usleep(100000);
      }
    else {
      if (hf_noise)
        dbprintf("hf : opened serial port %s OK\n", hf_device);
      }
    } while (hf < 0 && !hf_thread_abort);

    if (hf >= 0) {
      sts = tcgetattr(hf,&term);
      if (sts < 0) {
        fprintf(stderr, "hf : Error %d (%s) from tcgetattr\n", errno, strerror(errno));
        return NULL;
      }
      sts = tcgetattr(hf,&saved_hf_attributes);
      if (first) {
        first = 0;
        atexit(restore_hf_attributes);
        }
      cfmakeraw(&term);

      cfsetispeed(&term, HF_BAUDRATE);
      cfsetospeed(&term, HF_BAUDRATE);
      if (tcsetattr(hf,TCSANOW,&term) != 0) {
        fprintf(stderr, "hf : Error %d (%s) from tcsetattr\n", errno, strerror(errno));
        return NULL;
        }
      tcflush(hf, TCIOFLUSH);
      }

    if ((sts = clock_gettime(CLOCK_MONOTONIC, &ts)) < 0) {
      fprintf(stderr, "hf : Problem with clock_gettime(), returned %d (%s)\n", sts, strerror(sts));
      return 0;
    }

  // Figure out if anything is there
  sts = hf_poll_status();
  hf_tx_rx("^OP1;", 5, NULL, 0);

  // Loop forever servicing requests
  while (!hf_thread_abort) {
    set_timeout(&ts, 1000000);           // One second
    pthread_mutex_lock(&hf_lock);
    sts = pthread_cond_timedwait(&hf_wait, &hf_lock, &ts);
    if (sts == ETIMEDOUT) {
      pthread_mutex_unlock(&hf_lock);
#ifdef OLD_HEARTBEAT_STUFF
      if (enable_heartbeat) {
        // Do an echo to keep an eye on things
        sts = hf_poll_status();
        }
#endif
      continue;
      }
    else if (!sts) {
      // Someone has requested something
      if (hf_put_stuff) {
        hf_put_operation();
        hf_put_stuff = 0;
      }
      if (hf_get_status) {
        sts = hf_poll_status();
        hf_get_status = 0;
      }
      if (hf_do_tune) {
        hf_set_tune(hf_tune_on);
        hf_do_tune = 0;
      }
      pthread_mutex_unlock(&hf_lock);
    } else {
      fprintf(stderr, "hf_thread: unexpected result from pthread-cond-timedwait: %d\n", sts);
      perror("cond-wait in hf_thread");
      pthread_mutex_unlock(&hf_lock);
      break;
    }
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// The data logging thread
////////////////////////////////////////////////////////////////////////////////


#ifdef INCLUDE_LOGGING

#define LOG_POST_FINISH_DELAY   10

void *log_thread(void *marg)
{
  struct timespec ts_logstart, ts_lognow;
  uint32_t msec_logging;
  int sts;
  int sleep_usec = 60000;
  int start_logging, really_start_logging;

  if (log_noise)
    dbprintf("log_thread: started\n");

  // Create a log file for this session

  // Loop forever servicing requests
  really_start_logging = 0;
  while (!log_thread_abort) {
    if (!logging_active && !log_post_finish) {
      start_logging = ( do_logging ||
                        (log_while_running && (fuelpump_on[0] || fuelpump_on[1])) ||
                        (log_above_rpm > 0 && engine_is_running && engine_rpm > log_above_rpm)
                      ) ? 1 : 0;
      if (start_logging)
        really_start_logging++;         // Prevent false starts
      if (really_start_logging > 3) {
        really_start_logging = 0;
        // Create a timestamp, and make a directory with that name
        struct timeval tv_now;
        struct tm t, *now = &t;

        gettimeofday(&tv_now, NULL);
        localtime_r(&tv_now.tv_sec, &t);
        snprintf(log_dir, sizeof(log_dir), "%s/logs/%04d%02d%02d_%02d%02d%02d", log_dev,
            now->tm_year + 1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
        sts = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (sts) {
          // No media present, make it locally
          dbprintf("start_logging: No USB flash drive found, using local file system\n");
          snprintf(log_dir, sizeof(log_dir), "runtime/logs/%04d%02d%02d_%02d%02d%02d", 
            now->tm_year + 1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
          sts = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
          if (sts) {
            dbprintf("start_logging: Failed to create local directory for logging, giving up\n");
            do_logging = 0;
            start_logging = 0;
            log_while_running = 0;
            continue;
            }
          else
            dbprintf("Logging: log_dir \"%s\" created successfully\n", log_dir);
          }
        else
          dbprintf("Logging: log_dir \"%s\" created successfully\n", log_dir);
        dbprintf("Turning on logging to %s, do_logging %d RPM %4d fuelpumps %s/%s\n", log_dir,
            do_logging, engine_rpm, (fuelpump_on[0]) ? "ON" : "OFF", (fuelpump_on[1]) ? "ON" : "OFF");
        log_start = 1;
        logging_active = 1;

        stdout_redirect(log_dir);
        dbprintf("Turning on logging to %s, do_logging %d RPM %4d fuelpumps %s/%s\n", log_dir,
            do_logging, engine_rpm, (fuelpump_on[0]) ? "ON" : "OFF", (fuelpump_on[1]) ? "ON" : "OFF");

        // The pump start that caused us to start logging - the log of that gets
        // delayed until the above logging directory nonsense is sorted out. Do
        // that log now.
        if (delayed_log_pumpstart)
          dbprintf("delayed_log_pumpstart is 0x%x\n", delayed_log_pumpstart);
        if (delayed_log_pumpstart & 0x1) {
          log_pumpstart(0);
          delayed_log_pumpstart &= ~(0x1);
        }
        if (delayed_log_pumpstart & 0x2) {
          log_pumpstart(1);
          delayed_log_pumpstart = 0;
        }

        sleep_usec = 30000;
        if ((sts = clock_gettime(CLOCK_MONOTONIC, &ts_logstart)) < 0) {
          fprintf(stderr, "log: Problem with clock_gettime(), returned %d (%s)\n", sts, strerror(sts));
          return 0;
          }
        }
      }

    if (logging_active) {
      pthread_mutex_lock(&efi_lock);
      efi_get_logs = 1;
      pthread_cond_signal(&efi_req_cond);
      while (efi_get_logs && !efi_thread_abort) {
        int rc = pthread_cond_wait(&efi_done_cond, &efi_lock);
        if (rc != 0) {
          pthread_mutex_unlock(&efi_lock);
          return(NULL);
        }
      }
      pthread_mutex_unlock(&efi_lock);

      if (log_time > 0) {
        clock_gettime(CLOCK_MONOTONIC, &ts_lognow);
        time_t secs = ts_lognow.tv_sec - ts_logstart.tv_sec;
        long   nsecs = ts_lognow.tv_nsec - ts_logstart.tv_nsec;
        msec_logging = (uint32_t)((secs * 1000) + (nsecs / 1000000));

        if (msec_logging >= (log_time * 1000)) {
          do_logging = 0;
          logging_active = 0;
          log_finish = 1;
          log_time = 0;
          log_post_finish = 5;
          if (log_noise)
            dbprintf("Setting log_finish\n");
          }
        }
      else if (log_while_running > 0) {
        if (fuelpump_on[0] == 0 && fuelpump_on[1] == 0) {
          do_logging = 0;
          logging_active = 0;
          log_finish = 1;
          log_post_finish = LOG_POST_FINISH_DELAY;
          dbprintf("Engine is off - stopping logging (%d,%d)\n", fuelpump_on[0], fuelpump_on[1]);
          }
        }
      else if (log_above_rpm > 0 && engine_is_running && engine_rpm < (log_above_rpm - 200)) {      // -200 for some hysteresis
          if (log_above_rpm_post_seconds > 0) {
            // Keep logging for this many seconds after RPM drops below threshold
            log_time = (msec_logging + (1000 * log_above_rpm_post_seconds)) / 1000;     // Just fake it up
            dbprintf("RPM (%d) dropped below threshold (d) - logging for another %d seconds\n",
                engine_rpm, log_above_rpm, log_above_rpm_post_seconds);
          } else {
            do_logging = 0;
            logging_active = 0;
            log_finish = 1;
            log_post_finish = LOG_POST_FINISH_DELAY;
            dbprintf("RPM (%d) dropped below threshold (d) - stopping logging\n", engine_rpm, log_above_rpm);
            }
        }
      }
    else if (log_post_finish > 0) {
      if (log_noise)
        dbprintf("log_post_finish = %d\n", log_post_finish);
      log_post_finish--;
      if (log_post_finish == 1) {
        efi_log_close();
        log_post_finish = 0;
        log_state = 0;
        stdout_revert();
        dbprintf("log_post_finish...\n");
      } else {
        pthread_mutex_lock(&efi_lock);
        efi_get_logs = 1;
        pthread_cond_signal(&efi_req_cond);
        while (efi_get_logs && !efi_thread_abort) {
          int rc = pthread_cond_wait(&efi_done_cond, &efi_lock);
          if (rc != 0) {
            pthread_mutex_unlock(&efi_lock);
            return(NULL);
          }
        }
        pthread_mutex_unlock(&efi_lock);

        if (log_post_finish == LOG_POST_FINISH_DELAY-1) {
          if (log_finish_sample_count > 0) {
            clock_gettime(CLOCK_MONOTONIC, &ts_lognow);
            time_t secs = ts_lognow.tv_sec - ts_logstart.tv_sec;
            long   nsecs = ts_lognow.tv_nsec - ts_logstart.tv_nsec;
            double sample_time = ((double)secs * 1e6 + (double)(nsecs / 1000)) / (double)log_finish_sample_count;
            dbprintf("Logging stopped, derived sample time %.4f usec\n", sample_time);
            log_timefile(&ts_logstart, &ts_lognow, log_finish_sample_count);
            log_finish_sample_count = 0;
            }
          }
        }
      }
    else {
      sleep_usec = 60000;
      }

    mo_usleep(sleep_usec);
  }

  dbprintf("log_thread: Done\n");
  return NULL;
}
#endif

//
// Play a sound file.
//
void play_sound(char *what)
{
  char buf[256];

  snprintf(buf, sizeof(buf), "aplay -q assets/sounds/%s.wav &\n", what);
  system(buf);
}

////////////////////////////////////////////////////////////////////////////////
// The Lidar thread
////////////////////////////////////////////////////////////////////////////////

static int lidar_logpoints = 0;
static double *lidar_trace = NULL;

static int did_takeoff = 0;
static int arm_landing_trace = 0;
int lidar_num_callouts = 0;
const static char *lidar_takeoff_filename = NULL;
const static char *lidar_landing_filename = NULL;

static void lidar_log(float altitude, int takeoff, int done)
{
  static int head = 0, nrec = 0, tlnum = 1;
  char buf[128];

  if (done) {
    int tail;
    FILE *f;

    if (takeoff)
      did_takeoff = 1;              // Declare a takeoff happened

    if (nrec == lidar_logpoints)
      tail = head;                  // Head will point to the oldest point.
    else
      tail = 0;

    snprintf(buf, sizeof(buf), "%s/log_%s%d.dat", (logging_active) ? log_dir : ".", (takeoff) ? lidar_takeoff_filename : lidar_landing_filename, tlnum);
    f = fopen(buf, "w+");
    if (f) {
      struct timeval tv_now;
      struct tm t, *now = &t;

      gettimeofday(&tv_now, NULL);
      localtime_r(&tv_now.tv_sec, &t);

      fprintf(f, "# %s # %d\n", (takeoff) ? "Takeoff" : "Landing", tlnum);
      fprintf(f, "# Date/Time: %04d-%02d-%02d %02d:%02d:%02d\n",
        now->tm_year + 1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
      if (pilots && pilot_in_command >= 0)
        fprintf(f, "# Pilot: %s\n", (pilots+pilot_in_command)->fullname);
      else
        fprintf(f, "# Pilot: unknown\n");

      for (int i = 0; i < nrec; i++) {
        fprintf(f, "%d %.2f\n", i, *(lidar_trace + tail++));
        if (tail >= lidar_logpoints)
          tail = 0;
        }
      fclose(f);
      dbprintf("lidar_log: Wrote out %s log to file %s, %d records\n", (takeoff) ? "takeoff" : "landing", buf, nrec);
      if (!takeoff)
        tlnum++;
      }

    head = 0;                   // Start again
    nrec = 0;
    return;
  }

  *(lidar_trace + head++) = altitude;
  if (head >= lidar_logpoints)
    head = 0;
  if (nrec < lidar_logpoints)
    nrec++;
}

static int lidar_checksts(uint8_t sts)
{
  static int first = 1;

  if (first) {
    first = 0;
    if (sts & 0x2)
      dbprintf("Lidar: Signal overflow flag set\n");
    if (sts & 0x4)
      dbprintf("Lidar: Reference overflow flag set\n");
    if (sts & 0x8)
      dbprintf("Lidar: Peak detection flag set\n");
    if (sts & 0x10)
      dbprintf("Lidar: Device is in DC regulation\n");
    if ((sts & 0x20) == 0)
      dbprintf("Lidar: Reference and receiver bias are operational\n");
    }

  return (~sts & 0x38);     // Non-zero if any of bits 3, 4 or 5 are zero
}

struct callouts {
  double limit;                 // Height crossing that causes a callout
  int say_feet;                 // Whether or not to say the word "feet" in the callout
  const char *text;             // Text version so tts_say() can say it
  };

struct callouts *lidar_callouts = NULL;
struct callouts *lidar_callouts_declare_takeoff = NULL;

// The height of the Lidar receiver above the wheel height, in flight
static double height_above_wheels = (float) (27/12);        // XXX Calibrate this with the wheels floating one day
                                                            // 27" is height above the ground, with wheels on the ground
static double takeoff_margin = (float) 20.0;                // How many feet above callout[0].limit before we declare a takeoff
static double callout_margin = (float) 0.3;                 // A little margin above the limit, for when we do the callout

static int callouts_armed = 0;                              // If set, we call out altitudes for landing
double lidar_elapsed_msec;                                  // How long an altitude measurement took

float avg_lidar_distance;                                   // Running averaged distance

#define NLHIS               8
#define RPM_CRUISE          2300

void lidar_compute(float distance, int sts) 
{
  static float history[NLHIS] = {0};
  static float highest_value = 0;
  static int num_his = 0;
  static int crossing = 0;
  static struct callouts *c;
  static int last_was_good = 0;

  float avg_distance;
  char buf[128];
  int i;

  // If the measurement is no good, clear out history and return
  if (sts) {
    if (last_was_good)
      dbprintf("Lidar: Probably beyond range, callouts_armed %d, highest value recorded is %.1f feet\n", callouts_armed, highest_value);
    highest_value = 0.0;
    num_his = 0;
    c = lidar_callouts;
    last_was_good = 0;
    return;
  }

  last_was_good = 1;
  if (distance > highest_value)
    highest_value = distance;

  // Accumulate history and keep the last NLHIS items
  if (num_his == NLHIS) {
    for (i = NLHIS-1; i > 0; i--)
      history[i] = history[i-1];
    history[0] = distance;
  } else {
    history[num_his++] = distance;
    c = lidar_callouts;
    return;
  }

  // Compute average distance across last NLHIS samples
  for (i = 0, avg_distance = 0.0; i < NLHIS; i++)
    avg_distance += history[i];
  avg_distance /= NLHIS;
  avg_lidar_distance = avg_distance;


  // If the rpm is above RPM_CRUISE, and we see our altitude
  // go past the first callout limit, we must be taking off
  if (engine_is_running && engine_rpm > RPM_CRUISE) {
    // Arm ourselves when we take off
    if (!callouts_armed && lidar_callouts_declare_takeoff != NULL
        && avg_distance > lidar_callouts_declare_takeoff->limit + height_above_wheels + takeoff_margin) {
      dbprintf("Lidar: Landing callouts armed, engine_rpm %d, height %.1f feet\n", engine_rpm, distance);
      c = lidar_callouts;
      crossing = 0;
      callouts_armed = 1;
      if (!did_takeoff && !arm_landing_trace) {
        did_takeoff = 1;
        c = lidar_callouts;
        dbprintf("Lidar: Declaring a takeoff, avg_height %.1f feet, callouts[0].limit %.1f feet\n",
            avg_distance, lidar_callouts->limit);
        lidar_log(0.0, 1, 1);
        }
      }

    if (!callouts_armed)
      return;
    }

  // We have a valid distance measurement. Track it.
  if (avg_distance > (lidar_callouts->limit + height_above_wheels + takeoff_margin)) {
    // Higher than max value in lidar_callouts[0].limit, account for dithering around this height,
    // or takeoff / go-around.
    if (crossing)
      dbprintf("Lidar: resetting to initial height, crossing %d\n", crossing);
    crossing = 0;
    c = lidar_callouts;
    return;
  }

  if (c->limit == 0.0) {                        // We've landed
    if (did_takeoff) {
      arm_landing_trace = lidar_landing_runon_seconds * 20;         // A few more seconds, then write log
      did_takeoff = 0;
      }
    }

  if (avg_distance < (c->limit + height_above_wheels + callout_margin)) {
    // We've crossed below the next limit. Say it and switch.
    if (callouts_armed) {
      dbprintf("lidar_compute: %.1f %4s < %.0f+%.1f+%.1f, lidar elapsed time %.3f msec, did_takeoff %d\n",
            avg_distance,
            (c->say_feet) ? "feet" : "",
            c->limit, height_above_wheels, callout_margin,
            lidar_elapsed_msec, did_takeoff);
      snprintf(buf, sizeof(buf), "%s %s", c->text, (c->say_feet) ? "feet" : "");
      dbprintf("LIDAR_SPEECH: %s\n", buf);
      crossing++;
      if (c->limit > 0.0) {
        c++;                                    // Proceed to next limit, if we're not at the last
        if (lidar_noise)
          dbprintf("lidar: switching to next limit (%.1f feet)\n", c->limit);
        }
      else
        callouts_armed = 0;
      }
  }
}


void *lidar_thread(void *marg)
{
  static int print_first_good_reading = 1;
  struct timespec ts;
  static struct timespec ts_start, ts_finish;

  // Lidar measurement times range from around 4 msec at short distances to around 22.5 msec
  // at the longest distances and beyond. We settle on a measurement every 50 msec, or 20
  // measurements per second.
  uint64_t interval_ns = (uint64_t) 50 * 1e6; 		// 50 msec period

  int lidar_ok = -1;
  int last_lidar_was_ok = 0;
  uint16_t distance;
  uint8_t sts;
  float feet;
  float last_good_height = 0.0;
  uint64_t t;
  int loop = 0;

  dbprintf("lidar_thread: started\n");
  while (!lidar_enabled) {
    mo_usleep(5000);
  }
  dbprintf("lidar_thread: enabled\n");

  // Initialize Lidar
  lidar_i2c_init();
  lidar_configure(3, LIDARLITE_ADDR_DEFAULT);                   // Select maximum range
  //lidar_set_threshold_bypass(LIDARLITE_ADDR_DEFAULT, 0x60);	// Reduced sensitivity

  // Entry time
  if ((sts = clock_gettime(CLOCK_MONOTONIC, &ts)) < 0) {
    fprintf(stderr, "Problem with clock_gettime(), returned %d (%s)\n", sts, strerror(sts));
    return 0;
  }

  loop = 0;
  while (!lidar_thread_abort) {
    // Figure out next time to wait 'till, based on sample interval
    t = interval_ns;
    while (t >= (uint64_t) 1e9) {
      ts.tv_sec++;
      t -= 1e9;
    }
    ts.tv_nsec += (long)t;
    if (ts.tv_nsec >= 1e9) {
      ts.tv_sec++;
      ts.tv_nsec -= 1e9;
    }

    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    lidar_takeRange(LIDARLITE_ADDR_DEFAULT);                        // Start a measurement
    while (1) {
      int ret;

      ret = lidar_i2cRead(LLv3_STATUS, &sts, 1, LIDARLITE_ADDR_DEFAULT);
      if (ret != 1) {
        // Lidar may not be connected. Quietly keep trying.
        if (lidar_ok == -1) {
          dbprintf("Lidar: Not found\n");
          lidar_ok = 0;
          }
        if (lidar_ok == 1) {
          dbprintf("Lidar: Failed\n");
          lidar_ok = 0;
          }
        break;                                                      // Try again in 50 msec
      } else {
        if (lidar_ok <= 0) {
          dbprintf("Lidar: OK\n");
          lidar_ok = 1;
        }
      }
      if ((sts & 0x1) == 0) {                                       // Test for not busy
        // Lidar not busy - measurement must have completed
        clock_gettime(CLOCK_MONOTONIC, &ts_finish);                  // Derive measurement time
        long seconds = ts_finish.tv_sec - ts_start.tv_sec;
        long nanoseconds = ts_finish.tv_nsec - ts_start.tv_nsec;
        lidar_elapsed_msec = seconds*1e3 + nanoseconds*1e-6;

	if (lidar_checksts(sts)) {
	  // Height measurement is invalid
          distance = lidar_readDistance(LIDARLITE_ADDR_DEFAULT);
          feet = (float)distance / 100 * 39.37 / 12;
          lidar_compute(feet, 1);
          if (lidar_noise && last_lidar_was_ok)
	    dbprintf("Lidar: Invalid from sts 0x%x, last good height %.1f, measurement time %.3f msec\n",
                    sts, last_good_height, lidar_elapsed_msec);
          last_good_height = 0.0;
          last_lidar_was_ok = 0;
	} else {
          // Height measurement is valid
          distance = lidar_readDistance(LIDARLITE_ADDR_DEFAULT);
          feet = (float)distance / 100 * 39.37 / 12;
          if (feet > 200.0) {
            // Not possible, ignore this sample
            break;
          }
          last_good_height = feet;
          lidar_compute(feet, 0);
          lidar_log(feet, (did_takeoff > 0) ? 0 : 1, 0);
          if (arm_landing_trace > 0) {
            if (--arm_landing_trace == 0) {
              lidar_log(0.0, 0, 1);                     // Complete the landing trace log
            }
          }
          if (print_first_good_reading) {
	    dbprintf("Lidar: raw height of Lidar is %.2f feet, measurement time %.3f msec, did_takeoff %d\n", feet, lidar_elapsed_msec, did_takeoff);
            print_first_good_reading = 0;
            }
	  if (lidar_noise && (!last_lidar_was_ok || (loop % 1000) == 0))
	    dbprintf("Lidar: raw height is %.1f feet after %d loops, last measurement time %.3f msec\n", feet, loop, lidar_elapsed_msec);
          last_lidar_was_ok = 1;
	}
        break;
      } else {
        // Lidar is busy, wait for 1 msec and try again
        mo_usleep(1000);
        continue;
      }
    }

    //printf("Lidar: Calling clock_nanosleep with tv_sec %ld tv_nsec %ld\n", ts.tv_sec, ts.tv_nsec);
    sts = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    if (sts) {
      fprintf(stderr, "Lidar: Problem with clock_nanosleep(), returned %d (%s)\n", sts, strerror(sts));
      fprintf(stderr, "Lidar: ts.tv_sec %ld ts.tv_nsec %ld\n", ts.tv_sec, ts.tv_nsec);
      return 0;
    }
    loop++;
  }

  return(NULL);
}

// Only here if the read from the configuration file was successful.
static void lidar_init()
{
  lidar_trace = malloc(sizeof(double) * lidar_logpoints);

  if (!lidar_trace) {
    dbprintf("lidar: Allocation error, disabling lidar\n");
    lidar_enabled = 0;
  } else {
    lidar_enabled = 1;
  }
}


////////////////////////////////////////////////////////////////////////////////
//
// EVE3/4 Display Handler
//
////////////////////////////////////////////////////////////////////////////////

//#ifdef USE_libmpsse
  #include <libftdi1/ftdi.h>
//  #include "mpsse.h"
  typedef int FT_HANDLE;
//#else
//  #include "ftd2xx.h"
//  #include "libMPSSE_spi.h"
//#endif

int BT81x_Init(void);
FT_HANDLE eve_open(void);

//FT_HANDLE ftHandle = 0;

typedef enum {
  CL_LEFTCOIL,
  CL_RIGHTCOIL,
  CL_INJECTORS
  } cltype_t;

cltype_t checklist_state = CL_LEFTCOIL;

// These initialized during startup.
display_loop_t *dl_head, *dl_last;
display_loop_t *dl_current;
display_loop_t *dl_stat;                    // Separate chain for statistics
display_loop_t *dl;                         // Outside so it can be read/set by die_and_restart()


void *dl_alloc()
{
  display_loop_t *d;

  d = malloc(sizeof(display_loop_t));
  if (d)
    memset(d, 0, sizeof(display_loop_t));

  return(d);
}

// Add a new display_loop item onto the end of the list
static display_loop_t *dl_add(dstate_t state)
{
  display_loop_t  *new;

  new = dl_alloc();

  //dbprintf("dl_add state %d\n", state);
  if (new) {
    dl_last->next = new;
    new->prev = dl_last;
    new->ds = state;

    dl_last = new;
    }

  return(new);
}

static display_loop_t *dl_addto(display_loop_t *d, dstate_t state)
{
  display_loop_t  *new;

  new = dl_alloc();

  if (new) {
    if (d)
      d->next = new;
    new->prev = d;
    new->ds = state;
    }

  return(new);
}

static display_loop_t *dl_add_up(display_loop_t *d, dstate_t state)
{
  display_loop_t  *new;

  new = dl_alloc();

  if (new) {
    if (d)
      d->up = new;
    new->down = d;
    new->ds = state;
    }

  return(new);
}

static display_loop_t *dl_add_down(display_loop_t *d, dstate_t state)
{
  display_loop_t  *new;

  new = dl_alloc();

  if (new) {
    if (d)
      d->down = new;
    new->up = d;
    new->ds = state;
    }

  return(new);
}

static display_loop_t *dl_find(dstate_t state)
{
  display_loop_t  *d;

  for (d = dl_head; d && d->ds != state; d = d->next)
    ;

  return(d);
}

static display_loop_t *dl_extra(display_loop_t *ret, dstate_t state)
{
  static display_loop_t *extra;

  // Extra used for detailed displays, diagnostic etc.
  // State etc. set at run time
  if (!extra) {
    extra = dl_alloc();
    }

  if (extra) {
    extra->next = ret;
    extra->prev = ret;
    extra->detail = ret;
    extra->ds = state;
    }

  return(extra);
}

static display_loop_t *dl_extra2(display_loop_t *ret, dstate_t state)
{
  static display_loop_t *extra;

  // Extra used for detailed displays, diagnostic etc.
  // State etc. set at run time
  if (!extra) {
    extra = dl_alloc();
    }

  if (extra) {
    extra->next = ret;
    extra->prev = ret;
    extra->detail = ret;
    extra->ds = state;
    }

  return(extra);
}

static display_loop_t *dl_extra3(display_loop_t *ret, dstate_t state)
{
  static display_loop_t *extra;

  // Extra used for detailed displays, diagnostic etc.
  // State etc. set at run time
  if (!extra) {
    extra = dl_alloc();
    }

  if (extra) {
    extra->next = ret;
    extra->prev = ret;
    extra->detail = ret;
    extra->ds = state;
    }

  return(extra);
}

static void mo_setup_display_loop()
{
  display_loop_t *d;
  display_loop_t *d_prev = NULL;
  display_loop_t *d_loop = NULL;
  display_loop_t *dl_beginning = NULL;
  display_loop_t *du, *dd;


  // First display after security,
  // battery backup system, if present
  if (options.flap_controller == 1 || options.old_battery_backup_board == 1) {
    d = dl_add(DS_INITIAL_BATTERY_BACKUP);
    dl_beginning = d;
  }

  // Next comes the doors
  if (options.door_switches == 1) {
    d = dl_add(DS_DOORS);
    if (!dl_beginning)
      dl_beginning = d;
    else
      d->prev = dl_beginning;
    }

  // Next is "Ready To Start", which could be first if none of the above
  d = dl_add(DS_START);
  if (!dl_beginning)
    dl_beginning = d;
  else
    d->prev = dl_beginning;

  // Now on to checklists
  d = dl_add(DS_CHECKLISTS);
  if (!dl_beginning)
    dl_beginning = d;
  else
    d->prev = dl_beginning;

  //////////////////////////////////////////////////////////////////////////////// 
  // After checklists, we go to the engine summary display
  // This is where we start keeping track of "previous" and "loop", because it
  // can represent the point where the display order loops back on itself
  //////////////////////////////////////////////////////////////////////////////// 
  d = d_prev = d_loop = dl_add(DS_SUMMARY);
  if (!dl_beginning)
    dl_beginning = d;
  else
    d->prev = dl_beginning;

  // The more detailed coilpump and injector displays are now up/down from the main summary display
  du = dl_add_up(d, DS_COILPUMPS);
  if (du) {
    du->down = d;
    du->prev = dl_beginning;
    }

  dd = dl_add_down(d, DS_INJECTORS);
  if (dd) {
    dd->up = d;
    dd->prev = dl_beginning;
    }
  
  // Make a loop
  if (dd && du) {
    du->up = dd;
    dd->down = du;
    }


  if (options.vibration_sensor == 1 ) {
    d = dl_add(DS_VIBRATION);
    if (d) {
      d->prev = d_prev;
      d_prev = d;
      // Link prior detailed displays to this one
      if (du) {
        du->next = d;
        du = NULL;
        }
      if (dd) {
        dd->next = d;
        dd = NULL;
        }
      }
  }

  if (options.flap_controller == 1 || options.old_battery_backup_board == 1) {
    d = dl_add(DS_BATTERY_BACKUP);          // Really the flaps display
    if (d) {
      d->prev = d_prev;
      d_prev = d;
      // Link prior detailed displays to this one
      if (du) {
        du->next = d;
        du = NULL;
        }
      if (dd) {
        dd->next = d;
        dd = NULL;
        }
      }
  }

  if (options.o2_sensor == 1) {
    d = dl_add(DS_O2_SENSOR);
    if (d) {
      d->prev = d_prev;
      d_prev = d;
      // Link prior detailed displays to this one
      if (du) {
        du->next = d;
        du = NULL;
        }
      if (dd) {
        dd->next = d;
        dd = NULL;
        }
      }
  }

  if (options.ac_display == 1) {
    d = dl_add(DS_AIRCON);
    if (d) {
      d->prev = d_prev;
      d_prev = d;
      // Link prior detailed displays to this one
      if (du) {
        du->next = d;
        du = NULL;
        }
      if (dd) {
        dd->next = d;
        dd = NULL;
        }
      }
  }

  if (options.battery_display == 1) {
    d = dl_add(DS_ETX_FAULT);
    if (d) {
      d->prev = d_prev;
      d_prev = d;
      // Link prior detailed displays to this one
      if (du) {
        du->next = d;
        du = NULL;
        }
      if (dd) {
        dd->next = d;
        dd = NULL;
        }
      }
  }

  if (options.hf_display == 1) {
    d = dl_add(DS_HFRADIO);
    if (d) {
      d->prev = d_prev;
      d_prev = d;
      // Link prior detailed displays to this one
      if (du) {
        du->next = d;
        du = NULL;
        }
      if (dd) {
        dd->next = d;
        dd = NULL;
        }
      }
  }

  // Last thing is always the diagnostics entry point
  d = dl_add(DS_DIAGNOSTIC);
  if (d) {
    d->prev = d_prev;
    d_prev = d;
    // Link prior detailed displays to this one
    if (du) {
      du->next = d;
      du = NULL;
      }
    if (dd) {
      dd->next = d;
      dd = NULL;
      }
    }

  // Close the end of the chain, back to the start (DS_SUMMARY)
  if (d)
    d->next = d_loop;

  // And change the start's "prev" link to wrap back to the end
  d_loop->prev = d;

  // Now make up the "Statistics" chain
  dl_stat = dl_alloc();
  dl_stat->ds = DS_STAT_EFI_RESPONSE;
  dl_stat->prev = dl_find(DS_DIAGNOSTIC);

  d = dl_stat->next = dl_addto(dl_stat, DS_STAT_COMMS);
  d->prev = dl_stat;
  d_prev = d;

  d = dl_addto(d, DS_STAT_INJECTORS);
  d->prev = d_prev;
  d_prev = d;

  d = dl_addto(d, DS_STAT_COILPUMPS);
  d->prev = d_prev;
  d_prev = d;

  d = dl_addto(d, DS_STAT_JUNCTION_TEMPS);
  d->next = dl_find(DS_DIAGNOSTIC);
  d->prev = d_prev;

}

static void eve_handler() {
  static uint32_t msec_entered_ready_to_start;
  static int index;
  static int announce_ready_to_start;

  int action;
  static int last_action;

  static uint8_t ret_code = 0;
  struct pilot_data *p;
  char buf[64];
  int redraw;
  uint8_t last_sequence;

  clret_t ret;

  switch (dl->ds) {
    case DS_DISCONNECTED:
      last_action = -1;
      if (BT81x_Init()) {
        tft_present = 0;
        mo_usleep(1000000);
        break;
        }

      tft_present = 1;
      eve_backlight((uint8_t)tft_backlight);          // Set brightness
      //Calibrate();
      //load_calibration();
      //Cmd_SetRotate(1);                           // Inverted landscape
      // The above doesn't work :-(
      DISABLE_START;
      DISABLE_O2_SENSOR_POWER;
      o2_sensor_power_is_on = 0;
      dl = dl->next;
    break;

    case DS_IDLE:
      if (do_lcd_blank) {
        eve_blank();
        exit(0);
        }
      eve_initial_screen();
      dl = dl_find(initial_screen);
      break;

    case DS_NO_CONFIGURATION:
      break;

    case DS_INITIAL_SCREEN:
      announce_ready_to_start = 1;
      if (eve_debounce_tagread(NULL, 1) > 0) {
        dl = dl->next;
      }
      break;

    case DS_SECURITY:
      pilot_in_command = eve_security_page();
      if (pilot_in_command < 0 || pilot_in_command > sec_num_pilots) {
        dbprintf("DS_SECURITY: Error: eve_security_page() returned %d\n", pilot_in_command);
        break;
        }
      if (mo_abort)
        break;
      p = pilots + pilot_in_command;

      if (p->options & SEC_OPTION_LOCAL_FAKE_DATA) {
        //printf("Setting local_fake_data_ok\n");
        fuelpump_on[0] = fake_pump_is_on[0];
        fuelpump_on[1] = fake_pump_is_on[1];
        local_fake_data_ok = 1;
        allow_open_doors = 1;
        log_while_running = 0;              // No point in wasting disk space
        }
      if (p->options & SEC_OPTION_EFI_FAKE_DATA) {
        //printf("Setting efi_fake_data_ok\n");
        make_fake_pump_start_data();
        efi_fake_data_ok = 1;
        allow_open_doors = 1;
        }
      if (p->options & SEC_OPTION_EFI_DIAG_DATA) {
        efi_diag_data_ok = 1;
        allow_open_doors = 1;
        dbprintf("Using mode SEC_EFI_DIAG, open doors OK\n");
        }
      if (p->options & SEC_OPTION_REAL_WITH_FAKE_ENGINE) {
        efi_real_with_fake_engine = 1;
        allow_open_doors = 1;
        dbprintf("Using mode SEC_REAL_WITH_FAKE_ENGINE, open doors OK\n");
        }

      snprintf(buf, sizeof(buf), "Welcome, %s", p->name);
      tts_say(buf);

      if (engine_is_running)
        dl = dl_find(DS_SUMMARY);           // We've been rebooted while the engine is running. In this case,
      else                                  // the most useful thing is to go straight to the summary screen
        {
        // Otherwise, go through normal startup procedures
        dl = dl->next;
        }
      break;

    case DS_INITIAL_BATTERY_BACKUP:
      door_override = 0;
      if (options.flap_controller) {
        if (flaps_fd < 0) {
          dl = dl->next;
          break;
          }
        action = eve_flaps_display(0);
      } else {
        if (bat_fd < 0) {
          dl = dl->next;
          break;
          }
        action = eve_battery_display(0);
      }

      switch (action) {
#ifdef INCLUDE_FLAPS
        // Flaps UP button pushed
        case 4:
          if (flaps_active && flaps_mode_continuous == 0) {
            pthread_mutex_lock(&flaps_lock);
            flaps_put_action = FLAP_UP_BUTTON;
            ACTIVATE_FLAPS_THREAD(flaps_put_action);
            }

          break;

        // Flaps DOWN button pushed
        case 8:
          if (flaps_active && flaps_mode_continuous == 0) {
            pthread_mutex_lock(&flaps_lock);
            flaps_put_action = FLAP_DOWN_BUTTON;
            ACTIVATE_FLAPS_THREAD(flaps_put_action);
            }
          break;

        // The top of the flap setting bar was pushed. That flips the flap mode
        // from airspeed dependent or not
        case 3:
          if (flaps_active) {
            if (flaps_mode_airspeed == 0) {
              pthread_mutex_lock(&flaps_lock);
              flaps_mode_airspeed = 1;
              flaps_put_action = FLAP_MODE_AIRSPEED_ON;
            } else {
              pthread_mutex_lock(&flaps_lock);
              flaps_mode_airspeed = 0;
              flaps_put_action = FLAP_MODE_AIRSPEED_OFF;
              }
            ACTIVATE_FLAPS_THREAD(flaps_put_action);
            }
          break;

        // The bottom of the flap setting bar was pushed. That flips the flap mode
        // from continuous to incremental and back.
        case 7:
          if (flaps_active) {
            if (flaps_mode_continuous == 0) {
              pthread_mutex_lock(&flaps_lock);
              flaps_mode_continuous = 1;
              flaps_put_action = FLAP_MODE_CONTINUOUS;
            } else {
              pthread_mutex_lock(&flaps_lock);
              flaps_mode_continuous = 0;
              flaps_put_action = 0x8000000;       // Force it to do something
              }
            ACTIVATE_FLAPS_THREAD(flaps_put_action);
            }
          break;
#endif

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        case SWIPE_LEFT:
        case SWIPE_RIGHT:
        default:
          //index = action - 1;
          dl = dl->next;
          break;
        }
      break;


    case DS_DOORS:
      if (fake_door_countdown && (local_fake_data_ok || efi_fake_data_ok || efi_diag_data_ok)) {
        current_left = 0;
        eve_doors_page(0 | (current_right <<1) | (current_baggage <<2));
        fake_door_countdown--;
      } else {
        eve_doors_page(get_door_state());
      }
      if (allow_open_doors || (current_left && current_right && current_baggage)) {
        // Only allow the user out of here if all doors are locked
        if (eve_debounce_tagread(NULL, 0) > 0) {
          msec_entered_ready_to_start = msec_tick;
          dl = dl->next;
          break;
        }
      } else {
        if (eve_debounce_tagread(NULL, 0) > 0) {
          door_override++;
          if (door_override_count && door_override > door_override_count) {
            dbprintf("Door override granted, proceed with caution\n");
            tts_say("Door override granted, proceed with caution");
            msec_entered_ready_to_start = msec_tick;
            dl = dl->next;
            break;
          } else {
            tts_say("Door open, unable to proceed");
          }
          fake_door_countdown = 0;
          //current_left = 1;
        }
      }
      mo_usleep(100000);
      break;

    case DS_START:
      if (engine_is_running) {
        dbprintf("DS_START: Engine is running, as of msec_tick %d\n", msec_tick);
        DISABLE_START;                                      // Disable starter relay
        checklist_state = CL_LEFTCOIL;
        dl = dl->next;
      } else {
        ENABLE_START;                                       // Enable starter relay
        eve_ready_to_start(announce_ready_to_start);
        announce_ready_to_start = 0;
        mo_usleep(10000);                                   // Otherwise this is a loop
      }
      if ((local_fake_data_ok || efi_fake_data_ok || efi_diag_data_ok || efi_real_with_fake_engine)
        && eve_debounce_tagread(NULL, 1) > 0) {
        //dbprintf("DS_START: Read tag OK\n");
        if (simulate_engine_rpm) {
          fake_engine_rpm = simulate_engine_rpm;
          msec_when_engine_started = msec_tick;
          dbprintf("Doing fake engine start, %d rpm (%u %u)\n", fake_engine_rpm, msec_tick, msec_entered_ready_to_start);
          }
        DISABLE_START;
        checklist_state = CL_LEFTCOIL;
        dl = dl->next;
      } else {
        // A "real" engine start. However, the pilot may decide not to go ahead. As a getout
        // mechanism, three consecutive taps will escape to the checklists display.

        if (eve_debounce_tagread(NULL, 0) > 0) {
          if (++getout_taps >= 3) {
            dbprintf("DS_START: Pilot escaped from engine start at time msec_tick %d\n", msec_tick);
            //DISABLE_START;
            dl = dl->next;
            getout_taps = 0;
            }
          }
        }
      break;

    case DS_CHECKLISTS:
      switch (checklist_state) {
        case CL_LEFTCOIL:
          ret = eve_checklist("Left Coilpack", &do_efi_coilpack_test, 0, &efi_left_coil_disabled, 0);
          checklist_state = (ret == CL_NEXT) ? CL_RIGHTCOIL : CL_INJECTORS;
          break;

        case CL_RIGHTCOIL:
          ret = eve_checklist("Right Coilpack", &do_efi_coilpack_test, 1, &efi_right_coil_disabled, 0);
          checklist_state = (ret == CL_NEXT) ? CL_INJECTORS : CL_LEFTCOIL;
          break;

        case CL_INJECTORS:
          ret = eve_checklist("Injector power", &do_efi_injector_test, 0, &efi_injectors_disabled, 0);
          checklist_state = (ret == CL_NEXT) ? CL_LEFTCOIL : CL_RIGHTCOIL;
          if (ret == CL_NEXT)
            dl = dl->next;
          break;

        default:
          break;
        }
      break;

    case DS_SUMMARY:
      //detail_return = ds;
      action = eve_summary();
      switch (action) {
        case SWIPE_LEFT:
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          ret_code = SWIPE_LEFT;
          dl->prev->from = dl;              // Special case, DS_DIAGNOSTICS
          dl = dl->prev;
          break;

        case SWIPE_UP:
          if (dl->up)
            dl = dl->up;
          ret_code = SWIPE_UP;
          break;

        case SWIPE_DOWN:
          if (dl->down)
            dl = dl->down;
          ret_code = SWIPE_DOWN;
          break;

        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
          index = action - 1;
          dl = dl_extra(dl, DS_INJECTOR_DETAIL);
          break;

        case 7:
        case 8:
          index = action - 7;
          dl = dl_extra(dl, DS_COILPACK_DETAIL);
          break;

        case 9:
        case 10:
          index = action - 9;
          dl = dl_extra(dl, DS_FUELPUMP_DETAIL);
          break;

        default:
          //dbprintf("DS_SUMMARY: action %d\n", action);
          break;
      }
      break;

    case DS_COILPUMPS:
      //detail_return = ds;
      action = eve_coilpumps();
      switch (action) {
        case FORCE_INJECTOR_SUMMARY:
          dl = dl_find(DS_INJECTORS);
          break;

        case SWIPE_LEFT:
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          ret_code = SWIPE_LEFT;
          dl = dl->prev;
          break;

        case SWIPE_UP:
          if (dl->up)
            dl = dl->up;
          ret_code = SWIPE_UP;
          break;

        case SWIPE_DOWN:
          if (dl->down)
            dl = dl->down;
          ret_code = SWIPE_DOWN;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        case 1:
        case 2:
          index = action - 1;
          dl = dl_extra(dl, DS_COILPACK_DETAIL);
          break;

        default:
          index = action - 3;
          dl = dl_extra(dl, DS_FUELPUMP_DETAIL);
          break;
        }
      break;

    case DS_INJECTORS:
      //detail_return = ds;
      action = eve_injectors();
      switch (action) {
        case FORCE_INJECTOR_SUMMARY:
          dl = dl_find(DS_INJECTORS);
        break;

        case SWIPE_RIGHT:
          dl = dl->prev;
          break;

        case SWIPE_LEFT:
          dl = dl->next;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        case SWIPE_UP:
          if (dl->up)
            dl = dl->up;
          break;

        case SWIPE_DOWN:
          if (dl->down)
            dl = dl->down;
          break;

        default:
          index = action - 1;
          dl = dl_extra(dl, DS_INJECTOR_DETAIL);
          break;
        }
      break;

    case DS_BATTERY_BACKUP:
      if (options.flap_controller) {
        if (flaps_fd < 0) {                   // Don't get hung up
          dl = (last_action == SWIPE_RIGHT) ? dl->prev : dl->next;
          break;
          }
        action = eve_flaps_display(0);
      } else {
        if (bat_fd < 0) {                   // Don't get hung up
          dl = (last_action == SWIPE_RIGHT) ? dl->prev : dl->next;
          break;
          }
        action = eve_battery_display(0);
        }
      switch (action) {
        case SWIPE_LEFT:
          dl->next->from = dl;
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl->prev->from = dl;
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

#ifdef INCLUDE_FLAPS
        // Flaps UP button pushed
        case 4:
          if (flaps_mode_continuous == 0) {
            pthread_mutex_lock(&flaps_lock);
            flaps_put_action = FLAP_UP_BUTTON;
            pthread_cond_signal(&flaps_req_cond);
            pthread_mutex_unlock(&flaps_lock);
            }
          break;

        // Flaps DOWN button pushed
        case 8:
          if (flaps_mode_continuous == 0) {
            pthread_mutex_lock(&flaps_lock);
            flaps_put_action = FLAP_DOWN_BUTTON;
            pthread_cond_signal(&flaps_req_cond);
            pthread_mutex_unlock(&flaps_lock);
            }
          break;

        // The top of the flap setting bar was pushed. That flips the flap mode
        // from airspeed dependent or not
        case 3:
          if (flaps_mode_airspeed == 0) {
            pthread_mutex_lock(&flaps_lock);
            flaps_mode_airspeed = 1;
            flaps_put_action = FLAP_MODE_AIRSPEED_ON;
          } else {
            pthread_mutex_lock(&flaps_lock);
            flaps_mode_airspeed = 0;
            flaps_put_action = FLAP_MODE_AIRSPEED_OFF;
          }
          pthread_cond_signal(&flaps_req_cond);
          pthread_mutex_unlock(&flaps_lock);
          break;

        // The bottom of the flap setting bar was pushed. That flips the flap mode
        // from continuous to incremental and back.
        case 7:
          if (flaps_mode_continuous == 0) {
            pthread_mutex_lock(&flaps_lock);
            flaps_mode_continuous = 1;
            flaps_put_action = FLAP_MODE_CONTINUOUS;
          } else {
            pthread_mutex_lock(&flaps_lock);
            flaps_mode_continuous = 0;
            flaps_put_action = 0x8000000;       // Force it to do something
          }
          pthread_cond_signal(&flaps_req_cond);
          pthread_mutex_unlock(&flaps_lock);
          break;
#endif

        default:
          //index = action - 1;
          break;
        }
      break;

    case DS_ETX_FAULT:
      action = eve_etx1200_display();
      switch (action) {
        case SWIPE_LEFT:
          dl->next->from = dl;
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl->prev->from = dl;
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          //index = action - 1;
          break;
        }
      break;

    case DS_VIBRATION:
      action = eve_vibration_draw_page();
      //dbprintf("DS_VIBRATION: action %d\n", action);
      switch (action) {
        case 1:
          rev_adjust_mode = (rev_adjust_mode) ? 0 : 1;
          break;

        case 4:
            dbprintf("Writing vibration log\n");
            pthread_mutex_lock(&vib_lock);
            vib_write_snapshot = 1;
            pthread_cond_signal(&vib_wait);
            pthread_mutex_unlock(&vib_lock);
          break;

        case SWIPE_RIGHT:
          rev_adjust_mode = 0;
          dl->prev->from = dl;
          dl = dl->prev;
          break;

        case SWIPE_LEFT:
          rev_adjust_mode = 0;
          dl->next->from = dl;
          dl = dl->next;
          break;

        case SWIPE_UP:
          rotate_vib_options(1);
          break;

        case SWIPE_DOWN:
          rotate_vib_options(0);
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          break;
        }

      break;

    case DS_O2_SENSOR:
      action = eve_o2_sensor();
      switch (action) {
        case SWIPE_LEFT:
          dl->next->from = dl;
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl->prev->from = dl;
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          //index = action - 1;
          break;
        }
      break;


    case DS_AIRCON:
      action = eve_aircon();
      switch (action) {
        case SWIPE_LEFT:
          dl->next->from = dl;
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl->prev->from = dl;
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          //index = action - 1;
          break;
        }
      break;

    case DS_HFRADIO:
      action = eve_hfradio();
      //dbprintf("DS_HFRADIO: action %d\n", action);
      switch (action) {
        case SWIPE_LEFT:
          ret_code = SWIPE_RIGHT;
          dl->next->from = dl;
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl->prev->from = dl;
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          //index = action - 1;
          dl = dl->next;
          break;
        }
      break;

    case DS_DIAGNOSTIC:
      action = eve_diagnostic(ret_code);
      //dbprintf("DS_DIAGNOSTIC: action %d\n", action);
      switch (action) {
        case SWIPE_LEFT:
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl = dl->prev;
          break;

        case 1:                         // Allow checklist access at run-time
          checklist_state = CL_LEFTCOIL;
          for (int done = 0; !done;) {

            switch (checklist_state) {
              case CL_LEFTCOIL:
                ret = eve_checklist("Left Coilpack", &do_efi_coilpack_test, 0, &efi_left_coil_disabled, 1);
                checklist_state = (ret == CL_NEXT) ? CL_RIGHTCOIL : CL_INJECTORS;
                if (ret == CL_PREV)
                  done++;
                break;

              case CL_RIGHTCOIL:
                ret = eve_checklist("Right Coilpack", &do_efi_coilpack_test, 1, &efi_right_coil_disabled, 1);
                checklist_state = (ret == CL_NEXT) ? CL_INJECTORS : CL_LEFTCOIL;
                break;

              case CL_INJECTORS:
                ret = eve_checklist("Injector power", &do_efi_injector_test, 0, &efi_injectors_disabled, 1);
                checklist_state = (ret == CL_NEXT) ? CL_LEFTCOIL : CL_RIGHTCOIL;
                if (ret == CL_NEXT)
                  done++;
                break;
              }
            }
          checklist_state = CL_LEFTCOIL;
          break;

        case 2:                         // Statistics
          dl = dl_stat;                 // Go off into statistics chain
          break;

        case 6:
          dl = dl_extra2(dl, DS_CONFIGURE);
          break;

        case 7:                         // REBOOT
          exit(0);                      // Script will restart
          break;

        case 8:                         // BACK
          if (dl->from)
            dl = dl->from;              // from where we came, if it is set
          else
            dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        case 255:
          break;

        default:
          //index = action - 1;
          dl = dl->prev;
          break;
        }
      break;

    case DS_CONFIGURE:
      action = eve_configure(ret_code);
      switch (action) {
        case SWIPE_LEFT:
          dl = dl->prev;
          break;

        case SWIPE_RIGHT:
          dl = dl->prev;
          break;

#ifdef INCLUDE_FLAPS
        case 1:
          dl = dl_extra3(dl, DS_CONFIGURE_FLAPS);
          break;
#endif

        default:
          break;
        }
      break;

#ifdef INCLUDE_FLAPS
    case DS_CONFIGURE_FLAPS:
      action = eve_configure_flaps(ret_code);
      switch (action) {
        case SWIPE_LEFT:
          dl = dl->prev;
          break;

        case SWIPE_RIGHT:
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        case 6:
          mo_update_flap_config_settings();
          mo_write_config(config_writeback);
          dl = dl->prev->prev->prev;
          break;

        default:
          break;
        }
      break;
#endif

    case DS_STAT_EFI_RESPONSE:
      action = eve_stat_op_response(&efi_counts[0], num_efi_count, "EFI resp (msec)");
      switch (action) {
        case SWIPE_LEFT:
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl = dl->prev;
          break;

        case 6:
          reset_efi_counts();
          break;

        case 4:                 // Log for a period of time, log_snapshot_time (default 10 secs)
          switch (log_state) {
            case 0:
              do_logging = 1;
              log_time = log_snapshot_time;
              log_state++;
              break;

            case 1:
              log_time = 99999;     // Extend the time by a lot. Until stopped.
              log_state++;
              break;

            case 2:
              log_time = log_snapshot_time;     // Revert back to the snapshot time
              log_state = 0;
              break;

            default:
              break;
            }
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          break;
        }
      break;

    case DS_STAT_COMMS:
      action = eve_stat_comms(&efi_stats);
      //printf("DS_STAT_COMMS: action %d (current state %d\n", action, ds);
      switch (action) {
        case 9:                                             // Reset statistics
        case 10:
          //dbprintf("Resetting statistics\n");
          pthread_mutex_lock(&efi_stats_lock);
          memset(&efi_stats, 0, sizeof(efi_stats));
          pthread_mutex_unlock(&efi_stats_lock);
          break;

        case SWIPE_LEFT:
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          ret_code = SWIPE_LEFT;
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          break;
        }
      //printf("DS_STAT_COMMS: New state %d\n", ds);
      break;

    case DS_STAT_INJECTORS:
      action = eve_stat_injectors(&efi_stats);  // Strictly speacking efi_stats should be locked or copied. For
      switch (action) {                         // practical display purposes, doesn't really matter. Same below
        case SWIPE_LEFT:                        // as well...
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          break;
        }
      break;

    case DS_STAT_COILPUMPS:
      action = eve_stat_coilpumps(&efi_stats);
      //dbprintf("DS_STAT_COILPUMPS: action is %d\n", action);
      switch (action) {
        case SWIPE_LEFT:
          dl = dl->next;
          do_efi_enter_temp_mode();
          break;

        case SWIPE_RIGHT:
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          break;
        }
      break;

    case DS_STAT_JUNCTION_TEMPS:
      action = eve_stat_temps(&fp_temps, &gui_temps);
      switch (action) {
        case SWIPE_LEFT:
          do_efi_leave_temp_mode();
          dl = dl->next;
          break;

        case SWIPE_RIGHT:
          do_efi_leave_temp_mode();
          dl = dl->prev;
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          break;
        }
      break;

    case DS_SET_BACKLIGHT:
      tft_backlight = eve_set_backlight();
      dl = dl->prev;
      break;


    case DS_COILPACK_DETAIL:
      action = eve_coilpack_detail(index);
      //dbprintf("DS_COILPACK_DETAIL: action %d\n", action);
      switch (action) {
        case FORCE_INJECTOR_SUMMARY:
          dl = dl_find(DS_INJECTORS);
          break;

        case SWIPE_LEFT:
          if (index == 1) {
            dl->ds = DS_FUELPUMP_DETAIL;
            index = 0;
          } else {
            dl->ds = DS_COILPACK_DETAIL;
            index = 1;
          }
        break;

        case SWIPE_RIGHT:
          if (index == 0) {
            dl->ds = DS_FUELPUMP_DETAIL;
            index = 1;
          } else {
            dl->ds = DS_COILPACK_DETAIL;
            index = 0;
          }
        break;

        case DOUBLE_TOUCH:
        case 4:
          do_screenshot();
          break;

        default:
          dl = dl->prev;
          break;
        }
      break;

    case DS_FUELPUMP_DETAIL:
      action = eve_fuelpump_detail(index);
      switch (action) {
        case FORCE_INJECTOR_SUMMARY:
          dl = dl_find(DS_INJECTORS);
          break;

        case SWIPE_LEFT:
          if (index == 1) {
            dl->ds = DS_COILPACK_DETAIL;
            index = 0;
          } else {
            dl->ds = DS_FUELPUMP_DETAIL;
            index = 1;
          }
        break;

        case SWIPE_RIGHT:
          if (index == 0) {
            dl->ds = DS_COILPACK_DETAIL;
            index = 1;
          } else {
            dl->ds = DS_FUELPUMP_DETAIL;
            index = 0;
          }
        break;

        case DOUBLE_TOUCH:
        case 4:
          do_screenshot();
          break;

        case 2:
          dl->ds = DS_FUELPUMP_LASTSTART;
          break;

        default:
          dl = dl->prev;
          break;
        }
      break;

    case DS_FUELPUMP_LASTSTART:
      redraw = 0;
      do
        {
        if (efi_noise)
          dbprintf("DS_FUELPUMP_LASTSTART: top of loop, fscp_valid[%d] is %d, seq %d/%d, redraw = %d\n",
              index, fscp_valid[index], fuelpump_data_sequence[index], last_sequence, redraw);
        pthread_mutex_lock(&fuelpump_start_data_lock);
        last_sequence = fuelpump_data_sequence[index];      // Remember sequence # of pump start data
        eve_fuelpump_laststart_plot(index, &fuelpump_start_current_plot[index], redraw);
        pthread_mutex_unlock(&fuelpump_start_data_lock);

        for (action = 0; action == 0;) {
          mo_usleep(20000);
          check_screenshot();
          if ((action = check_touch(4, 70, 0, 0, 20)) > 0)          // The only reason for this is to support DOUBLE_TOUCH
            break;                                                  // Key hit

          pthread_mutex_lock(&fuelpump_start_data_lock);
          if (fuelpump_data_sequence[index] != last_sequence) {     // Start data changed
            pthread_mutex_unlock(&fuelpump_start_data_lock);
            redraw = 1;
            break;                  // Break out of this loop, to redraw display
            }
          pthread_mutex_unlock(&fuelpump_start_data_lock);
          }

        } while (action == 0);

      switch (action) {
        case FORCE_INJECTOR_SUMMARY:
          dl = dl_find(DS_INJECTORS);
          break;

        case DOUBLE_TOUCH:
          do_screenshot();
          break;

        default:
          dl->ds = DS_FUELPUMP_DETAIL;
          break;
        }
      break;

    case DS_INJECTOR_DETAIL:
      action = eve_injector_detail(index);
      switch (action) {
        case FORCE_INJECTOR_SUMMARY:
          dl = dl_find(DS_INJECTORS);
          break;

        case SWIPE_LEFT:
          if (++index > 5)
            index = 0;
        break;

        case SWIPE_RIGHT:
          if (--index < 0)
            index = 5;
        break;

        case DOUBLE_TOUCH:
        case 4:
          do_screenshot();
          break;

        default:
          dl = dl->prev;
          break;
        }
      break;

    default:
      dl = dl_find(DS_SUMMARY);
      break;
  }

  last_action = action;
}

////////////////////////////////////////////////////////////////////////////////
// Maintain a msec ticker
////////////////////////////////////////////////////////////////////////////////

void *tick_thread(void *marg)
{
  struct timespec ts_last, ts_now, ts_start;
  time_t secs;
  long   nsecs;
  uint32_t msec_pump_running;
  int idx;

  clock_gettime(CLOCK_MONOTONIC, &ts_start);

  clock_gettime(CLOCK_MONOTONIC, &ts_last);
  mo_usleep(100);

  dbprintf("tick_thread: started\n");

  while (!mo_abort && !tick_thread_abort) {
    clock_gettime(CLOCK_MONOTONIC, &ts_now);

    secs = ts_now.tv_sec - ts_start.tv_sec;
    nsecs = ts_now.tv_nsec - ts_start.tv_nsec;
    msec_tick = (uint32_t)((secs * 1000) + (nsecs / 1000000));

    if (msec_tick == last_msec_tick) {
      // Delay for next msec boundary, if we need one
      secs = ts_now.tv_sec - ts_last.tv_sec;
      nsecs = ts_now.tv_nsec - ts_last.tv_nsec;
      useconds_t sleep_usec = (useconds_t)1000 - (useconds_t) ((secs * 1000000) + (nsecs / 1000));

      if (sleep_usec <= 0) {
        //dbprintf("tick_thread: sleep_usec %d usec (secs %d, nsecs %d) ?!?\n", sleep_usec, secs, nsecs);
        sleep_usec = 1;
        }
      if (sleep_usec > 1000) {
        //dbprintf("tick_thread: sleep_usec %d usec (secs %d, nsecs %d) ?!?\n", sleep_usec, secs, nsecs);
        sleep_usec = 1000;
        }
      //dbprintf("tick_thread: sleep_usec %d\n", sleep_usec);
      mo_usleep(sleep_usec);
      continue;
    }
    //dbprintf("tick_thread: proceeding, msec_tick %d\n", msec_tick);

    ts_last.tv_sec = ts_now.tv_sec;     // For next time
    ts_last.tv_nsec = ts_now.tv_nsec;
    last_msec_tick = msec_tick;

    // Handle O2 sensor power, 30 seconds after engine start
    if (engine_is_running && (O2_SENSOR_POWER_ON == 0)) {
      if ((msec_tick - msec_when_engine_started) >= 30000) {
        dbprintf("Turning on O2 Sensor power, as of msec_tick %d\n", msec_tick);
        tts_say("O2 sensor on");
        ENABLE_O2_SENSOR_POWER;
        o2_sensor_power_is_on = 1;
        }
      }
    if (!engine_is_running && (O2_SENSOR_POWER_ON > 0)) {
      dbprintf("Engine stopped, turning off O2 Sensor power (%d)\n", O2_SENSOR_POWER_ON);
      tts_say("O2 sensor off");
      DISABLE_O2_SENSOR_POWER;
      o2_sensor_power_is_on = 0;
      }

    // Periodic stats collection
    if ((msec_tick % 500) == 499) {                               // Every 0.5 seconds
      if (enable_periodic_stats && efi_fd >= 0) {
        pthread_mutex_lock(&efi_lock);                  
        efi_get_stats = 1;                      // We don't block, there's no need
        pthread_cond_signal(&efi_req_cond);
        pthread_mutex_unlock(&efi_lock);
        }

      static uint8_t tick2 = 0;
      static FILE *fafr = NULL;
      struct tm t, *now = &t;
      struct timeval tv_now;

      if (tick2) {
        static uint8_t fafr_giveup = 0;
        tick2 = 0;

        // Every second, write out AFR, or 0.0 for invalid readings
        if (engine_is_running) {
          gettimeofday(&tv_now, NULL);
          localtime_r(&tv_now.tv_sec, &t);

          if (!fafr && (fafr_giveup < 5)) {
            char buf[256];

            snprintf(buf, sizeof(buf), "%s/afr%04d%02d%02d_%02d%02d%02d.txt", log_dir,
              now->tm_year + 1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);

            fafr = fopen(buf, "w");

            if (fafr)
              dbprintf("Create fafr succeeded, buf is %s\n", buf);
            else {
              dbprintf("Create fafr failed, buf is %s\n", buf);
              if (++fafr_giveup > 4)
                dbprintf("  .. giving up on fafr creation\n");
              }

            // First record in file is the current local date/time
            if (fafr) {
              strftime(buf, sizeof(buf), "%c", now);
              fprintf(fafr, "# %s\n", buf);
              }
            }

          if (fafr) {
            fprintf(fafr, "%02d.%02d.%02d: RPM %4d AFR %.1f c0 %4d c1 %4d i0 %5d\n", now->tm_hour, now->tm_min, now->tm_sec, engine_rpm, air_fuel_ratio,
                              saved_average_current[6], saved_average_current[7], saved_average_current[0]);
            }
          }

        if (fafr && !engine_is_running) {
          fflush(fafr);
          fclose(fafr);
          fafr = NULL;
          }
        } else {
          tick2 = 1;
        }
      }

    // Sense fuelpump OFF->ON transitions, delay at least 1 second and then collect startup data
    for (idx = 0; idx < FUELPUMPS; ++idx) {
      if (fuelpump_turned_on[idx] > 0) {
        secs = ts_now.tv_sec - ts_pumpstart[idx].tv_sec;
        nsecs = ts_now.tv_nsec - ts_pumpstart[idx].tv_nsec;
        msec_pump_running = (uint32_t)((secs * 1000) + (nsecs / 1000000));
        if (msec_pump_running > 1000) {             // One seconds after a fuelpump started...
          if (fuelpump_on[idx]) {                   // If the pump is still on...
            efi_get_fuelpump_laststart_data(idx);   // Request the new pump start data
            //dbprintf("tick_thread: %d: getting laststart data, msec_pump_running %d\n", idx, msec_pump_running);
            }
          fuelpump_turned_on[idx] = 0;              // Arm for next time
        }
      }
    }

  }

  return((void *) NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Poll the A/C controller, update local data
////////////////////////////////////////////////////////////////////////////////

aircon_data_t *get_ac_data(void)
{
  return(&aircon_data);
}


void *ac_thread(void *marg)
{
  aircon_data_t *a = &aircon_data;
  const char *ac_mode[] = {" OFF", " MAN", "AUTO", "VENT"};
  int sock;
  struct sockaddr_in server;
  const char *request = "GET /raw HTTP/1.1\r\nHost: 192.168.4.61\r\nConnection: keep-alive\r\n\r\n";
  char response[1024];
  char *p, *endptr;
  int nb;
  static int first = 1;

  while (!mo_abort) {
    a->valid = 0;

    if (first) {
      // Delay startup, let everything else happen first
      first = 0;
      for (int i = 0; i < 50 && !mo_abort && !ac_thread_abort; i++)
        mo_usleep(100000);
    }

    while (!ac_thread_abort) {
      sock = socket(AF_INET, SOCK_STREAM, 0);
      if (sock < 0)
        break;

      server.sin_addr.s_addr = inet_addr("192.168.4.61");
      server.sin_family = AF_INET;
      server.sin_port = htons(80);

      if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        close(sock); 
        break;
        }

      if (ac_noise)
        dbprintf("ac_thread: connected OK\n");
      while (!ac_thread_abort) {
        if (send(sock, request, strlen(request), 0) < 0)
          break;

        for (int i = 0; i < 10 && !mo_abort && !ac_thread_abort; i++)
          mo_usleep(100000);

        bzero(response, sizeof(response));
        if ((nb = recv(sock, response, sizeof(response), 0)) < 0)
          break;

        //printf("%s\n", response);

        // XXX Protect against overflows
        p = strstr(response, "<html>");
        if (!p)
          break;
        p += 6;
        while (*p && isspace(*p))
          p++;
        a->temp_evaporator = strtof(p, &endptr);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        a->temp_rear_vent_air = strtof(p, &endptr);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        a->temp_inside_left = strtof(p, &endptr);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        a->temp_inside_right = strtof(p, &endptr);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        a->pressure_evap_highside = strtof(p, &endptr);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        a->voltage_bilge_blower_pot = strtof(p, &endptr);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        a->bilge_adc_output = (uint32_t)strtoul(p, &endptr, 0);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        a->naca_pwm_width = (uint32_t)strtoul(p, &endptr, 0);
        for (p = endptr; (p-response) < nb && *p && isspace(*p); p++)
          ;
        if (!strncmp(p, "OFF", 3))
          a->mode = AM_OFF;
        else if (!strncmp(p, "MANUAL", 6))
          a->mode = AM_MANUAL;
        else if (!strncmp(p, "AUTOMATIC", 9))
          a->mode = AM_AUTO;
        else
          a->mode = AM_VENT;
        for (; (p-response) < nb && !isspace(*p); p++)        // Skip mode word
          ;
        for (; (p-response) < nb && isspace(*p); p++)         // Skip subsequent whitespace
          ;
        a->scan_time_usec = (uint32_t)strtoul(p, &endptr, 0);
        a->valid = 1;

        if (ac_noise)
          dbprintf("ac_thread: %s: evap %.1fC/%.1fpsi, air %.1f inside %.1f/%.1f pot %.1f (from resp %d bytes)\n",
            ac_mode[a->mode], a->temp_evaporator, a->pressure_evap_highside,
            a->temp_rear_vent_air, a->temp_inside_left, a->temp_inside_right, a->voltage_bilge_blower_pot, nb);

        // Done with a loop so we can be aborted quickly enough
        for (int i = 0; i < 40 && !mo_abort && !ac_thread_abort; i++)
          mo_usleep(100000);
        }

      a->valid = 0;
      close(sock);
      }
      mo_usleep(100000);
    }
  dbprintf("ac_thread: Done\n");
  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// EFI power board, writing and reading config data to Flash
// Normally a one-time thing to write configuration for each board,
// after Flash is programmed.
////////////////////////////////////////////////////////////////////////////////

void efi_configuration(int write, int read)
{
  static struct config_data config = {
    CONFIG_MAGIC,
    1,                          // Board revision, major
    3,                          // Board revision, minor
    6,                          // Cylinders
    2,                          // Coilpacks
    2,                          // Fuelpumps
    0,
    (0<<8)|42,                  // Firmware
    2,                          // Board serial number
    0, 0};

  if (write) {
    // Extract arguments from command line string, fill in config structure
    //
    // <Firmware M.N >,<Major>,<Minor>,<SN>
    //
    char *p, *next;

    p = &efi_write_config_arg[0];
    if ((next = strchr(p, '.')) != NULL) {
      *next++ = '\0';
      config.firmware_revision = (atoi(p) & 0xff) | ((atoi(next) & 0xff) << 8);

      if ((p = strchr(next, ',')) != NULL) {
        *p++ = '\0';
        config.board_revision_major = (uint8_t) atoi(p);
        if ((p = strchr(p, ',')) != NULL) {
          *p++ = '\0';
          config.board_revision_minor = (uint8_t) atoi(p);
          if ((p = strchr(p, ',')) != NULL) {
            *p++ = '\0';
            config.board_serial_number = (uint8_t) atoi(p);
            }
          }
        }
      }
    pthread_mutex_lock(&efi_lock);
    efi_config(&config);
    pthread_mutex_unlock(&efi_lock);
  } else if (read) {
    pthread_mutex_lock(&efi_lock);
    efi_config(NULL);
    pthread_mutex_unlock(&efi_lock);
  }
}

////////////////////////////////////////////////////////////////////////////////
// main, command line etc.
////////////////////////////////////////////////////////////////////////////////

#define LO_RUN_LIDAR            257
#define LO_RUN_EFI              258
#define LO_HELP                 259
#define LO_SCREENSHOTS          260
#define LO_ENGINE1              261
#define LO_ENGINE2              262
#define LO_DEBUG_COMMS          263
//#define LO_TEST                 264
#define LO_FAKE_OK              265
#define LO_TIME_DEBUG           266
#define LO_BAUDRATE             267
#define LO_HEARTBEAT            268
#define LO_EFI_FAKE_OK          269
#define LO_ALL_NOISE            270
#define LO_EFIDEV               271
#define LO_HFDEV                272
#define LO_BATDEV               273
#define LO_LCDOFF               274
#define LO_TEMPS                275
#define LO_INJECTOR_SCALE       276
#define LO_COILPACK_SCALE       277
#define LO_BACKLIGHT            278
#define LO_BAT_NOISE            279
#define LO_EFI_NOISE            280
#define LO_HF_NOISE             281
#define LO_AC_NOISE             283
#define LO_START_HF             284
#define LO_START_AC             285
#define LO_START_COILPUMP       286
#define LO_START_INJECTORS      287
#define LO_LOG_NOISE            288
#define LO_LOG_TIME             289
#define LO_WRITE_CONFIG         290
#define LO_READ_CONFIG          291
#define LO_BLANK                292
#define LO_LIDAR_NOISE          293
#define LO_LIDAR_TEST           294
#define LO_HAT2_NOISE           295
#define LO_START_SUMMARY        296
#define LO_START_AFR            297
#define LO_START_FLAPS          298
#define LO_FLAP_NOISE           299
#define LO_VIB_NOISE            300
#define LO_START_VIB            301


static struct option longopts[] =
{
 /* { *name             has_arg *flag val } */
    {"backlight",         1, 0, LO_BACKLIGHT},              // Set backlight, 0 - 100%
    {"debug-comms",       0, 0, LO_DEBUG_COMMS},
    {"baudrate",          1, 0, LO_BAUDRATE},
    {"efi",               0, 0, LO_RUN_EFI},                // Run the EFI thread
    {"efi-fake-ok",       0, 0, LO_EFI_FAKE_OK},
    {"fake-ok",           0, 0, LO_FAKE_OK},
    {"help",              0, 0, LO_HELP},
#ifdef OLD_HEARTBEAT_STUFF
    {"heart",             0, 0, LO_HEARTBEAT},
#endif
    {"lcdoff",            0, 0, LO_LCDOFF},                 // Turn off the LCD and exit
    {"lidar",             0, 0, LO_RUN_LIDAR},              // Run the LIDAR thread
    {"screenshots",       0, 0, LO_SCREENSHOTS},
    {"times",             0, 0, LO_TIME_DEBUG}, 
    {"efidev",            1, 0, LO_EFIDEV},
    {"batdev",            1, 0, LO_BATDEV},
    {"hfdev",             1, 0, LO_HFDEV},
    {"poll-temps",        0, 0, LO_TEMPS},
    {"injector-scale",    1, 0, LO_INJECTOR_SCALE},
    {"coilpack-scale",    1, 0, LO_COILPACK_SCALE},
    {"all-noise",         0, 0, LO_ALL_NOISE},
#ifdef INCLUDE_BAT
    {"bat-noise",         0, 0, LO_BAT_NOISE},
#endif
#ifdef INCLUDE_FLAPS
    {"flaps-noise",       0, 0, LO_FLAP_NOISE},
#endif
    {"vib-noise",         0, 0, LO_VIB_NOISE},
    {"efi-noise",         0, 0, LO_EFI_NOISE},
    {"hf-noise",          0, 0, LO_HF_NOISE},
    {"log-noise",         0, 0, LO_LOG_NOISE},
    {"log-time",          1, 0, LO_LOG_TIME},
#ifdef USE_HAT2
    {"hat2-noise",        0, 0, LO_HAT2_NOISE},
#endif
    {"ac-noise",          0, 0, LO_AC_NOISE},
    {"start-ac",          0, 0, LO_START_AC},
    {"start-afr",         0, 0, LO_START_AFR},
    {"start-hf",          0, 0, LO_START_HF},
    {"start-cp",          0, 0, LO_START_COILPUMP},
    {"start-summary",     0, 0, LO_START_SUMMARY},
    {"start-sum",         0, 0, LO_START_SUMMARY},
    {"start-coilpump",    0, 0, LO_START_COILPUMP},
    {"start-inj",         0, 0, LO_START_INJECTORS},
    {"start-flaps",       0, 0, LO_START_FLAPS},
    {"start-vib",         0, 0, LO_START_VIB},
    {"write-config",      1, 0, LO_WRITE_CONFIG},
    {"read-config",       0, 0, LO_READ_CONFIG},
    {"blank",             0, 0, LO_BLANK},
    {"lidar-noise",       0, 0, LO_LIDAR_NOISE},
    {"lidar-test",        0, 0, LO_LIDAR_TEST},
    {0, 0, 0, 0 }
};

static void usage(void) {
  struct option *l = longopts;

  printf("Usage:\n");
  printf("    mo [flags] [-d <debugflags>] [-v]\n");
  printf("Flags:\n");
  for (; l->name; l++)
    printf("    --%s%s\n", l->name, (l->has_arg) ? " <arg>" : "");
  printf("--write-config argument is of the form:\n");
  printf("        <Firmware M.N >,<Major>,<Minor>,<SN>\n");
  printf("    eg. 0.5,1,2,2\n");
}

void sig_handler(int sig)
{
  if (sig == SIGUSR1) {
    eve_do_screenshot = 1;
    }
  else if (sig == SIGINT) {
    mo_abort = 1;
    }
}

#ifdef USE_HAT2

void gpio_primary_tacho_alert(int e, lgGpioAlert_p evt, void *data)
{

  int i;
#if 0
  int userdata = *(int *)data;
#endif

   for (i=0; i<e; i++)
   {
#if 0
      printf("PRI: u=%d t=%"PRIu64" c=%d g=%d l=%d f=%d (%d of %d)\n",
         userdata, evt[i].report.timestamp, evt[i].report.chip,
         evt[i].report.gpio, evt[i].report.level,
         evt[i].report.flags, i+1, e);
#endif      
      tacho_count_primary++;
   }
}

void gpio_secondary_tacho_alert(int e, lgGpioAlert_p evt, void *data)
{

  int i;
#if 0
  int userdata = *(int *)data;
#endif

   for (i=0; i<e; i++)
   {
#if 0
      printf("SEC: u=%d t=%"PRIu64" c=%d g=%d l=%d f=%d (%d of %d)\n",
         userdata, evt[i].report.timestamp, evt[i].report.chip,
         evt[i].report.gpio, evt[i].report.level,
         evt[i].report.flags, i+1, e);
      tacho_count_secondary++;
#endif
   }
}

#endif

// Shutdown helper function
static void mo_request_shutdown(void)
{
  mo_abort = 1;

  lidar_thread_abort = 1;
  efi_thread_abort = 1;
  vib_thread_abort = 1;
  flaps_thread_abort = 1;
  etx_thread_abort = 1;
  hf_thread_abort = 1;
  tick_thread_abort = 1;
  log_thread_abort = 1;

#ifdef USE_HAT2
  hat2_thread_abort = 1;
  if (gpio_handle >= 0)
    lgGpiochipClose(gpio_handle);
#endif

  pthread_mutex_lock(&efi_lock);
  pthread_cond_broadcast(&efi_req_cond);
  pthread_cond_broadcast(&efi_done_cond);
  pthread_mutex_unlock(&efi_lock);

  pthread_mutex_lock(&flaps_lock);
  pthread_cond_broadcast(&flaps_req_cond);
  pthread_cond_broadcast(&flaps_done_cond);
  pthread_mutex_unlock(&flaps_lock);

  pthread_mutex_lock(&vib_lock);
  pthread_cond_broadcast(&vib_wait);
  pthread_mutex_unlock(&vib_lock);

  pthread_mutex_lock(&bat_lock);
  pthread_cond_broadcast(&bat_wait);
  pthread_mutex_unlock(&bat_lock);

  pthread_mutex_lock(&etx_lock);
  pthread_cond_broadcast(&etx_wait);
  pthread_mutex_unlock(&etx_lock);

  pthread_mutex_lock(&hf_lock);
  pthread_cond_broadcast(&hf_wait);
  pthread_mutex_unlock(&hf_lock);

  pthread_mutex_lock(&log_lock);
  pthread_cond_broadcast(&log_wait);
  pthread_mutex_unlock(&log_lock);
}


int main(int argc, char *argv[])
{
  int sts;
  int opt;

  pthread_t t_thread;
  pthread_t a_thread;

#ifdef USE_HAT2
  pthread_t h2_thread;
#endif
#ifdef INCLUDE_LIDAR
  pthread_t li_thread;
#endif
  pthread_t e_thread;
#ifdef INCLUDE_FLAPS
  pthread_t f_thread;
#endif
#ifdef INCLUDE_BAT
  pthread_t b_thread;
#endif
#ifdef INCLUDE_ETX
  pthread_t E_thread;
#endif
#ifdef INCLUDE_HF
  pthread_t h_thread;
#endif
#ifdef INCLUDE_LOGGING
  pthread_t lo_thread;
#endif
  pthread_t v_thread;


  int t_thread_created = 0;
  int a_thread_created = 0;
#ifdef USE_HAT2
  int h2_thread_created = 0;
#endif
#ifdef INCLUDE_LIDAR
  int li_thread_created = 0;
#endif
  int e_thread_created = 0;
  int f_thread_created = 0;
#ifdef INCLUDE_BAT
  int b_thread_created = 0;
#endif
  int E_thread_created = 0;
#ifdef INCLUDE_HF
  int h_thread_created = 0;
#endif
  int lo_thread_created = 0;
  int v_thread_created = 0;

  pthread_mutexattr_t mutex_attr;

  int ms;
  display_loop_t *d;

  atexit(restore_efi_attributes);

  // Create the default, NULL display loop
  d = dl_alloc();
  dl = dl_head = dl_last = dl_current = d;
  d->next = d;
  d->prev = d;
  d->ds = DS_DISCONNECTED;

  // Initial things that are never returned to
  dl_add(DS_IDLE);
  dl_add(DS_INITIAL_SCREEN);
  dl_add(DS_SECURITY);

  // Process arguments
  while ((opt = getopt_long(argc, argv, "cd:hmv", longopts, (int *)0)) != -1) {
    switch(opt) {
      case LO_BACKLIGHT:
        tft_backlight = (uint32_t) strtoul(optarg, 0, 0);
        if (tft_backlight > 100)
          tft_backlight = 100;
        break;

      case LO_DEBUG_COMMS:
      case 'c':
        debug_comms = 1;
        break;

      case LO_BAUDRATE:
        baudrate = strtoul(optarg, 0, 0);
        break;

      case 'd':
        debug_level = strtoul(optarg, 0, 0);
        break;

      case LO_RUN_LIDAR:
        run_lidar++;
        break;

      case LO_RUN_EFI:
        break;

      case LO_HELP:
      case 'h':
        usage();
        return(0);
        break;

#ifdef OLD_HEARTBEAT_STUFF
      case LO_HEARTBEAT:
        enable_heartbeat = 1;
        break;
#endif

      case LO_SCREENSHOTS:
        do_std_screenshots++;
        break;

      case LO_EFI_FAKE_OK:
        efi_fake_data_ok = 1;
        make_fake_pump_start_data();
        break;

      case LO_FAKE_OK:
        local_fake_data_ok = 1;
        log_while_running = 0;              // No point in wasting disk space
        fuelpump_on[0] = fake_pump_is_on[0];
        fuelpump_on[1] = fake_pump_is_on[1];
        break;

      case LO_LCDOFF:
        do_lcdoff = 1;
        break;

      case LO_TIME_DEBUG:
        print_time = 1;
        break;

      case LO_EFIDEV:
        snprintf(efi_device, sizeof(efi_device), "%s", optarg);
        break;

#ifdef INCLUDE_BAT
      case LO_BATDEV:
        snprintf(bat_device, sizeof(bat_device), "%s", optarg);
        break;
#endif

      case LO_HFDEV:
        snprintf(hf_device, sizeof(hf_device), "%s", optarg);
        break;

      case LO_TEMPS:
        poll_junction_temperatures = 1;
        break;

      case LO_INJECTOR_SCALE:
        injector_divisor = strtoul(optarg, 0, 0);
        break;

      case LO_COILPACK_SCALE:
        coilpack_divisor = strtoul(optarg, 0, 0);
        break;

#ifdef INCLUDE_BAT
      case LO_BAT_NOISE:
        bat_noise = 1;
        break;
#endif

#ifdef INCLUDE_FLAPS
      case LO_FLAP_NOISE:
        flaps_noise = 1;
        break;
#endif
      case LO_VIB_NOISE:
        vib_noise = 1;
        break;

      case LO_EFI_NOISE:
        efi_noise = 1;
        break;

      case LO_HF_NOISE:
        hf_noise = 1;
        break;

      case LO_LOG_NOISE:
        log_noise = 1;
        break;

      case LO_ALL_NOISE:
        flaps_noise = 1;
        vib_noise = 1;
        efi_noise = 1;
        hf_noise = 1;
        log_noise = 1;
#ifdef INCLUDE_BAT
        bat_noise = 1;
#endif
        break;

      case LO_LOG_TIME:
        log_time = strtoul(optarg, 0, 0);
        if (log_time > 0)
          do_logging = 1;
        break;

#ifdef USE_HAT2
      case LO_HAT2_NOISE:
        hat2_noise = 1;
        break;
#endif

      case LO_AC_NOISE:
        ac_noise = 1;
        break;

      case LO_START_HF:
        initial_screen = DS_HFRADIO;
        break;

      case LO_START_AC:
        initial_screen = DS_AIRCON;
        break;

      case LO_START_AFR:
        initial_screen = DS_O2_SENSOR;
        break;

      case LO_START_SUMMARY:
        initial_screen = DS_SUMMARY;
        break;

      case LO_START_COILPUMP:
        initial_screen = DS_COILPUMPS;
        break;

      case LO_START_INJECTORS:
        initial_screen = DS_INJECTORS;
        break;

      case LO_START_FLAPS:
        initial_screen = DS_BATTERY_BACKUP;
        break;

      case LO_START_VIB:
        initial_screen = DS_VIBRATION;
        break;

      case LO_WRITE_CONFIG:
        snprintf(efi_write_config_arg, sizeof(efi_write_config_arg), "%s", optarg);
        enable_periodic_stats = 0;
        efi_write_config = 1;
        break;

      case LO_READ_CONFIG:
        enable_periodic_stats = 0;
        efi_read_config = 1;
        break;

      case LO_BLANK:
        do_lcd_blank = 1;
        break;

      case LO_LIDAR_NOISE:
        lidar_noise = 1;
        break;

      case LO_LIDAR_TEST:
        lidar_testmode = 1;
        break;

      case 'v':
        verbose++;
        break;

      default:
        break;
    }
  }

  while (optind < argc) {
    optind++;
    // or really, process argv[optind++]
  }

  openlog("mo", LOG_NDELAY|LOG_PID, LOG_LOCAL0);
  syslog(LOG_INFO, "Log opened");

  // Read in configuration data
  mo_load_config();

  // Now the config is loaded, "options" is set, so we can set up the display loop
  mo_setup_display_loop();

#ifdef ARM
  #ifdef USE_LGPIO
    gpio_handle = lgGpiochipOpen(GPIOCHIP);
    if (gpio_handle < 0) {
      fprintf(stderr, "lpGpiochipOpen(4) failed\n");
      return -1;
      }
    dbprintf("GPIOCHIP is %d, gpio_handle = 0x%x\n", GPIOCHIP, gpio_handle);
  #else
    if (wiringPiSetup() < 0) {
      fprintf(stderr, "wiringPiSetup() failed\n");
      return -1;
    }
  #endif
#endif

#if 0
  struct sched_param rt_param;
  int rt_prio = 60; // realtime scheduling priority for Linux kernel

  // Enable realtime fifo scheduling
  rt_param.sched_priority = rt_prio;
  if (sched_setscheduler(0, SCHED_FIFO, &rt_param) == -1) {
     perror("sched_setscheduler failed");
     return -1;
  }

  // Lock memory allocations for realtime performance
  if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
      perror("mlockall failed");
      return -2;
  }
#endif

  signal(SIGUSR1, sig_handler);
  signal(SIGINT, sig_handler);

#ifdef ARM
  #ifdef USE_LGPIO
    // Cycle LCD Power
    
    lgGpioClaimOutput(gpio_handle, 0, GPIO16, 0);
    lgGpioWrite(gpio_handle, GPIO16, 0);
    if (do_lcdoff) {
      exit(0);
    }
    mo_usleep(1000000);
    lgGpioWrite(gpio_handle, GPIO16, 1);

    lgGpioClaimInput(gpio_handle, LG_SET_PULL_UP, LEFT_BATTERY_ALARM_PIN);
    lgGpioClaimInput(gpio_handle, LG_SET_PULL_UP, RIGHT_BATTERY_ALARM_PIN);
    lgGpioClaimInput(gpio_handle, LG_SET_PULL_UP, BAGGAGE_DOOR_SWITCH);

    #ifdef USE_HAT2
      lgGpioClaimInput(gpio_handle, 0, LEFT_DOOR_SWITCH);
      lgGpioClaimInput(gpio_handle, 0, RIGHT_DOOR_SWITCH);
      lgGpioClaimInput(gpio_handle, 0, BAGGAGE_DOOR_SWITCH);
      lgGpioClaimInput(gpio_handle, LG_SET_PULL_NONE, PRIMARY_TACHO_GPIO);
      lgGpioClaimInput(gpio_handle, LG_SET_PULL_NONE, SECONDARY_TACHO_GPIO);

      lgGpioClaimOutput(gpio_handle, 0, ENABLE_O2_SENSOR_GPIO, 0);
#if 0
      lgGpioSetSamplesFunc(gpio_events, &gpio_user_data);
#endif
      const static int ud=123;
      int lsts1, lsts2, lsts3, lsts4;
      lsts1 = lgGpioSetAlertsFunc(gpio_handle, PRIMARY_TACHO_GPIO, gpio_primary_tacho_alert, (void *) &ud);
      lsts2 = lgGpioSetAlertsFunc(gpio_handle, SECONDARY_TACHO_GPIO, gpio_secondary_tacho_alert, (void *) &ud);
      // Max rate of these is 2700 rpm / 60 * 3 = 135 pulses per second
      lsts3 = lgGpioClaimAlert(gpio_handle, 0, LG_FALLING_EDGE, PRIMARY_TACHO_GPIO, -1);    // primary_rpm pulses
      lsts4 = lgGpioClaimAlert(gpio_handle, 0, LG_FALLING_EDGE, SECONDARY_TACHO_GPIO, -1);  // secondary_rpm pulses  
      if (lsts1 || lsts2 || lsts3 || lsts4) {
        dbprintf("lgGpio trouble with Alerts: lsts 1-4 %d %d %d %d\n", lsts1, lsts2, lsts3, lsts4);
        dbprintf("lsts1 is %s\n", lguErrorText(lsts1));
        dbprintf("lsts2 is %s\n", lguErrorText(lsts2));
        dbprintf("lsts3 is %s\n", lguErrorText(lsts3));
        dbprintf("lsts4 is %s\n", lguErrorText(lsts4));
        }
    #endif
  #endif
  mo_usleep(500000);                      // Wait for the LCD to initialize
#endif

  // Create threads
  pthread_mutexattr_init(&mutex_attr);

  if ((sts = pthread_mutex_init(&efi_lock, &mutex_attr)) != 0) {
    fprintf(stderr, "warning: pthread_mutex_init failed: %d\n", sts);
    perror("pthread_mutex_init");
  }
  if ((sts = pthread_cond_init(&efi_req_cond, 0)) != 0) {
    fprintf(stderr, "warning: pthread_cond_init failed: %d\n", sts);
    perror("pthread_cond_init");
  }
  if ((sts = pthread_cond_init(&efi_done_cond, 0)) != 0) {
    fprintf(stderr, "warning: pthread_cond_init failed: %d\n", sts);
    perror("pthread_cond_init");
  }

  if (pthread_create(&t_thread, NULL, tick_thread, &ms)) {
    fprintf(stderr, "Error creating ticker thread\n");
    return 1;
  }
  t_thread_created = 1;

  if (options.ac_display == 1) {
    if (pthread_create(&a_thread, NULL, ac_thread, &ms)) {
      fprintf(stderr, "Error creating A/C thread\n");
      return 1;
      }
    a_thread_created = 1;
    }

#ifdef INCLUDE_LIDAR
  if (run_lidar) {
      if (pthread_create(&li_thread, NULL, lidar_thread, &ms)) {
        fprintf(stderr, "Error creating Lidar thread\n");
        return 1;
      }
    li_thread_created = 1;
    }
#endif

#ifdef USE_HAT2
  if (pthread_create(&h2_thread, NULL, hat2_thread, &ms)) {
    fprintf(stderr, "Error creating HAT2 thread\n");
    return 1;
  }
  h2_thread_created = 1;
#endif

#ifdef INCLUDE_EFI
  if (pthread_create(&e_thread, NULL, efi_thread, &ms)) {
    fprintf(stderr, "Error creating Efi thread\n");
    return 1;
  }
  e_thread_created = 1;
#endif

  if (options.flap_controller == 1) {
    if (pthread_create(&f_thread, NULL, flaps_thread, &ms)) {
      fprintf(stderr, "Error creating flaps thread\n");
      return 1;
      }
    f_thread_created = 1;
    }

  if (options.vibration_sensor == 1) {
    if (pthread_create(&v_thread, NULL, vib_thread, &ms)) {
      fprintf(stderr, "Error creating vib thread\n");
      return 1;
      }
    v_thread_created = 1;
    }

  if (options.old_battery_backup_board == 1) {
    if (pthread_create(&b_thread, NULL, bat_thread, &ms)) {
      fprintf(stderr, "Error creating bat thread\n");
      return 1;
      }
    b_thread_created = 1;
    }

  if (options.battery_display == 1) {
    if (pthread_create(&E_thread, NULL, etx_thread, &ms)) {
      fprintf(stderr, "Error creating etx thread\n");
      return 1;
      }
    E_thread_created = 1;
    }

  if (options.hf_display == 1) {
    if (pthread_create(&h_thread, NULL, hf_thread, &ms)) {
      fprintf(stderr, "Error creating HF thread\n");
      return 1;
      }
    h_thread_created = 1;
    }

#ifdef INCLUDE_LOGGING
  if (pthread_create(&lo_thread, NULL, log_thread, &ms)) {
    fprintf(stderr, "Error creating log thread\n");
    return 1;
  }
  lo_thread_created = 1;
  atexit(&efi_log_close);
#endif

  mo_usleep(400000);

#ifdef INCLUDE_EFI
  if (efi_write_config || efi_read_config) {
    efi_configuration(efi_write_config, efi_read_config);
    mo_usleep(1000000);                             // Give the writes time to happen...
    mo_request_shutdown();
    return 0;
  }
#endif

#ifdef INCLUDE_FLAPS
  if (flaps_config_good) {
    flaps_put_flap_settings();                      // Do this once, at startup
    }
#endif

  check_for_crash_recovery();                      // See if we're doing a crash recovery

  dbprintf("TFT: Entering eve_handler\n");
  while (!mo_abort) {
    // Handle the TFT display
    eve_handler();
    }

  if (tft_present) {
    eve_blank();
    mo_usleep(10000);
    }

  mo_request_shutdown();
  mo_usleep(10000);

  if (lo_thread_created) pthread_join(lo_thread, NULL);
#ifdef INCLUDE_HF
  if (h_thread_created)  pthread_join(h_thread, NULL);
#endif
  if (E_thread_created)  pthread_join(E_thread, NULL);
#ifdef INCLUDE_BAT
  if (b_thread_created)  pthread_join(b_thread, NULL);
#endif
  if (v_thread_created)  pthread_join(v_thread, NULL);
  if (f_thread_created)  pthread_join(f_thread, NULL);
  if (e_thread_created)  pthread_join(e_thread, NULL);
#ifdef USE_HAT2
  if (h2_thread_created) pthread_join(h2_thread, NULL);
#endif
  if (t_thread_created)  pthread_join(t_thread, NULL);
  if (a_thread_created)  pthread_join(a_thread, NULL);
#ifdef INCLUDE_LIDAR
  if (li_thread_created) pthread_join(li_thread, NULL);
#endif

#ifdef USE_LGPIO
  lgGpiochipClose(gpio_handle);
#endif

  mo_usleep(100000);

  return 0;
  }



////////////////////////////////////////////////////////////////////////////////
// Load configuration data, from config file
////////////////////////////////////////////////////////////////////////////////

#define CONFIG_FILE "config/mo.config"

struct display_data display = {0};

#define mo_COLOR_RGB(r,g,b) ((4<<24)|((r)<<16)|((g)<<8)|(b))
#define mo_LINEWIDTH(x)     ((14<<24)|((x)&8191))

static void do_color_rgb_string(config_setting_t *c, char *p, uint32_t *rgb) {
  const char *s;
  uint32_t r=0, g=255, b=255;

  config_setting_lookup_string(c, p, &s);
  if (s) {
    r = atoi(s);
    if ((s = strchr(s, ',')) != NULL) {
      s++;
      g = atoi(s);
      }
    if ((s = strchr(s, ',')) != NULL) {
      s++;
      b = atoi(s);
      }
    }

  *rgb = mo_COLOR_RGB(r,g,b);
  }

static void do_linewidth(config_setting_t *c, char *p, int *val)
{
  int tmp;
  config_setting_lookup_int(c, p, &tmp);

  *val = mo_LINEWIDTH(tmp);
}


config_t cfg = {0};

static void mo_load_config()
{
  config_setting_t *c;
  struct pilot_data *p;

  config_init(&cfg);

  if (! config_read_file(&cfg, CONFIG_FILE)) {
    config_destroy(&cfg);
    fprintf(stderr, "Error reading configuration file\n");
    return;
    }

  ////////////////////////////////////////////////////////////////////////////////
  // File names to use
  ////////////////////////////////////////////////////////////////////////////////
  c = config_lookup(&cfg, "filenames");
  if (c) {
   const char *str = NULL;
   config_setting_lookup_string(c, "initial_screen", &str);
   if (str)
     snprintf(initial_splash, sizeof(initial_splash), "%s", str);
   str = NULL;
   config_setting_lookup_string(c, "config_writeback", &str);
   if (str)
     snprintf(config_writeback, sizeof(config_writeback), "%s", str);
  }
  ////////////////////////////////////////////////////////////////////////////////
  // Options
  ////////////////////////////////////////////////////////////////////////////////
  c = config_lookup(&cfg, "options");
  if (c) {
    int val;

    config_setting_lookup_int(c, "door_switches", &val);
    options.door_switches = (uint8_t) val;
    config_setting_lookup_int(c, "engine_start_interlock", &val);
    options.engine_start_interlock = (uint8_t) val;
    config_setting_lookup_int(c, "flap_controller", &val);
    options.flap_controller = (uint8_t) val;
    config_setting_lookup_int(c, "old_battery_backup_board", &val);
    options.old_battery_backup_board = (uint8_t) val;
    config_setting_lookup_int(c, "battery_display", &val);
    options.battery_display = (uint8_t) val;
    config_setting_lookup_int(c, "o2_sensor", &val);
    options.o2_sensor = (uint8_t) val;
    config_setting_lookup_int(c, "vibration_sensor", &val);
    options.vibration_sensor = (uint8_t) val;
    config_setting_lookup_int(c, "vib_source", &val);
    options.vib_source = (uint8_t) val;
    config_setting_lookup_int(c, "vib_live_rotations", &val);
    options.vib_live_rotations = (uint8_t) val;
    config_setting_lookup_int(c, "vib_snapshot_rotations", &val);
    options.vib_snapshot_rotations = (uint8_t) val;
    config_setting_lookup_int(c, "ac_display", &val);
    options.ac_display = (uint8_t) val;
    config_setting_lookup_int(c, "hf_display", &val);
    options.hf_display = (uint8_t) val;
    }

  ////////////////////////////////////////////////////////////////////////////////
  // Pilots
  ////////////////////////////////////////////////////////////////////////////////
  c = config_lookup(&cfg, "pilots");
  if (c) {
    int num_pilots;
    int i;

    num_pilots = config_setting_length(c);
    if (num_pilots > 0) {
      pilots = (struct pilot_data *) calloc(num_pilots, sizeof(struct pilot_data));
      if (!pilots) {
        dbprintf("calloc for pilot data (%d pilots) failed\n", num_pilots);
        }

      for (i = 0, p = pilots; p && i < num_pilots; i++, p++) {
        const char *name, *fullname, *code;
        int admin, option;

        config_setting_t *pilot = config_setting_get_elem(c, i);

        if (!(config_setting_lookup_string(pilot, "name", &name)
           && config_setting_lookup_string(pilot, "fullname", &fullname)
           && config_setting_lookup_string(pilot, "code", &code)
           && config_setting_lookup_int(pilot, "administrator", &admin)))
          continue;

        //dbprintf("Pilot: %-30s %-30s %-8s %4d\n", name, fullname, code, admin);
        if ((p->name = malloc(strlen(name)+1)) != NULL)
          strcpy(p->name, name);
        if ((p->fullname = malloc(strlen(fullname)+1)) != NULL)
          strcpy(p->fullname, fullname);
        p->code_length = snprintf((char *) &p->code[0], sizeof(p->code), "%s", code);
        if (p->code_length > sizeof(p->code))
          p->code_length = sizeof(p->code);
        p->idx = 0;
        p->administrator = admin;
        p->options = 0;

        option = 0;
        config_setting_lookup_int(pilot, "local_fake_data", &option);
        if (option)
          p->options |= SEC_OPTION_LOCAL_FAKE_DATA;

        option = 0;
        config_setting_lookup_int(pilot, "efi_fake_data", &option);
        if (option)
          p->options |= SEC_OPTION_EFI_FAKE_DATA;

        option = 0;
        config_setting_lookup_int(pilot, "efi_diag_data", &option);
        if (option)
          p->options |= SEC_OPTION_EFI_DIAG_DATA;

        option = 0;
        config_setting_lookup_int(pilot, "real_with_fake_engine", &option);
        if (option)
          p->options |= SEC_OPTION_REAL_WITH_FAKE_ENGINE;

      }
      sec_num_pilots = num_pilots;
    }

  ////////////////////////////////////////////////////////////////////////////////
  // Display parameters
  ////////////////////////////////////////////////////////////////////////////////
  c = config_lookup(&cfg, "display");
  if (c) {
    double x;
    int val;

    config_setting_lookup_int(c, "width", &display.width);
    config_setting_lookup_int(c, "height", &display.height);
    config_setting_lookup_int(c, "plot_height", &display.plot_height);
    config_setting_lookup_int(c, "y_over_fs", &display.y_over_fs);
    config_setting_lookup_int(c, "y_fs", &display.y_fs);
    config_setting_lookup_int(c, "y_fault", &display.y_fault);
    config_setting_lookup_int(c, "main_label_font", &display.main_label_font);
    config_setting_lookup_int(c, "small_axis_font", &display.small_axis_font);
    config_setting_lookup_int(c, "pumpstart_xscale", &display.pumpstart_xscale);

    config_setting_lookup_float(c, "air_fuel_ratio_correction", &x);
    air_fuel_ratio_correction = (float)x;

    if ((config_setting_lookup_int(c, "left_coilpack_alternator_threshold", &display.coilpack_alternator_threshold[0])) == 0)
      display.coilpack_alternator_threshold[0] = LEFT_COILPACK_ALTERNATOR_THRESHOLD;
    if ((config_setting_lookup_int(c, "right_coilpack_alternator_threshold", &display.coilpack_alternator_threshold[1])) == 0)
      display.coilpack_alternator_threshold[1] = RIGHT_COILPACK_ALTERNATOR_THRESHOLD;

    if (display.pumpstart_xscale < 1 || display.pumpstart_xscale > 8) {
      dbprintf("Illegal display.pumpstart_xscale %d, defaulting to 1\n", display.pumpstart_xscale);
      display.pumpstart_xscale = 1;
      }

    do_linewidth(c, "normal_line_width", &display.normal_line_width);
    do_linewidth(c, "fault_line_width", &display.fault_line_width);
    do_linewidth(c, "caution_line_width", &display.caution_line_width);
    do_linewidth(c, "pumpstart_line_width", &display.pumpstart_line_width);

    do_color_rgb_string(c, "axis_color", &display.axis_color);
    do_color_rgb_string(c, "clear_color", &display.clear_color);
    do_color_rgb_string(c, "normal_label_color", &display.normal_label_color);
    do_color_rgb_string(c, "dim_label_color", &display.dim_label_color);
    do_color_rgb_string(c, "fault_label_color", &display.fault_label_color);
    do_color_rgb_string(c, "fault_line_color", &display.fault_line_color);
    do_color_rgb_string(c, "caution_line_color", &display.caution_line_color);
    do_color_rgb_string(c, "pumpstart_line_color", &display.pumpstart_line_color);
    do_color_rgb_string(c, "normal_line_color", &display.normal_line_color);
    do_color_rgb_string(c, "vib_line1_color", &display.vib_line1_color);
    do_color_rgb_string(c, "vib_line2_color", &display.vib_line2_color);
    do_color_rgb_string(c, "normal_title_color", &display.normal_title_color);

    if ((config_setting_lookup_int(c, "screenshot_options", &val)) == 0) {
      //dbprintf("default screenshot_options %d\n", display.screenshot_options);
      }
    else {
      display.screenshot_options = (uint8_t) val;
      //dbprintf("screenshot_options %d\n", display.screenshot_options);
      }

#if 0
    dbprintf("display.width is %d, height %d, plot_height %d, y_over_fs %d, y_fs %d, y_fault %d\n",
      display.width, display.height, display.plot_height, display.y_over_fs, display.y_fs, display.y_fault);
    dbprintf("display.main_label_font %d small_axis_font %d normal_line_width 0x%x fault_line_width 0x%x\n",
        display.main_label_font, display.small_axis_font, display.normal_line_width, display.fault_line_width);
    dbprintf("display.caution_line_width 0x%x pumpstart_line_width 0x%x\n",
        display.caution_line_width, display.pumpstart_line_width);
    dbprintf("display.axis_color is 0x%06x\n", display.axis_color);
    dbprintf("display.clear_color is 0x%06x\n", display.clear_color);
    dbprintf("display.normal_label_color is 0x%06x\n", display.normal_label_color);
    dbprintf("display.fault_label_color is 0x%06x\n", display.fault_label_color);
    dbprintf("display.fault_line_color is 0x%06x\n", display.fault_line_color);
    dbprintf("display.caution_line_color is 0x%06x\n", display.caution_line_color);
    dbprintf("display.pumpstart_line_color is 0x%06x\n", display.pumpstart_line_color);
    dbprintf("display.normal_line_color is 0x%06x\n", display.normal_line_color);
    dbprintf("display.pumpstart_xscale is %d\n", display.pumpstart_xscale);
#endif
    }
  }

#ifdef INCLUDE_FLAPS
  ////////////////////////////////////////////////////////////////////////////////
  // Flaps parameters
  ////////////////////////////////////////////////////////////////////////////////
  c = config_lookup(&cfg, "flaps");
  if (c) {
    int val;
    int sts;
    int allsts = 1;

    sts = config_setting_lookup_int(c, "reflex", &val);
    if (sts)
      flaps.reflex_position = (uint16_t)val;
    else
      allsts = 0;
    sts = config_setting_lookup_int(c, "zero", &val);
    if (sts)
      flaps.zero_position = (uint16_t)val;
    else
      allsts = 0;
    sts = config_setting_lookup_int(c, "half", &val);
    if (sts)
      flaps.half_position = (uint16_t)val;
    else
      allsts = 0;
    sts = config_setting_lookup_int(c, "full", &val);
    if (sts)
      flaps.full_position = (uint16_t)val;
    else
      allsts = 0;
    sts = config_setting_lookup_int(c, "deadband", &val);
    if (sts)
      flaps.deadband = (uint16_t)val;
    else
      allsts = 0;
    sts = config_setting_lookup_int(c, "sensor_reversed", &val);
    if (sts)
      flaps_sensor_reverse = (val) ? 1 : 0;
    else
      allsts = 0;

    if (allsts) 
      flaps_config_good = 1;
  }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  // Logging
  ////////////////////////////////////////////////////////////////////////////////
  c = config_lookup(&cfg, "logging");
  if (c) {
    int val;
    int sts;
    const char *str;

    sts = config_setting_lookup_int(c, "enable", &val);
    if (sts)
      do_logging = (val) ? 1 : 0;
    else
      do_logging = 0;
    if (config_setting_lookup_int(c, "log_time", &val))
      log_time = val;
    else
      log_time = 0;
    if (config_setting_lookup_int(c, "log_snapshot_time", &val))
      log_snapshot_time = val;
    else
      log_snapshot_time = 10;
    if (config_setting_lookup_int(c, "log_above_rpm", &val))
      log_above_rpm = val;
    else
      log_above_rpm = 0;
    if (config_setting_lookup_int(c, "log_above_rpm_post_seconds", &val))
      log_above_rpm_post_seconds = val;
    else
      log_above_rpm_post_seconds = 0;

     config_setting_lookup_string(c, "directory_default", &str);
     if (str)
       snprintf(log_dir, sizeof(log_dir), "%s", str);
#ifdef ARM
     config_setting_lookup_string(c, "directory_arm", &str);
#else
     config_setting_lookup_string(c, "directory_other", &str);
#endif
     if (str)
       snprintf(log_dev, sizeof(log_dev), "%s", str);
#if 1
    dbprintf("Logging flash card device is \"%s\"\n", log_dev);
    dbprintf("Logging default directory is \"%s\"\n", log_dir);
#endif
  }

  //////////////////////////////////////////////////////////////////////////////// 
  // Lidar settings
  //////////////////////////////////////////////////////////////////////////////// 
  int lidar_ok = 1;
  int enabled = 0;

  c = config_lookup(&cfg, (lidar_testmode) ? "lidar_test" : "lidar");
  if (c) {
    config_setting_t *cc;

    if (!(config_setting_lookup_int(c, "enabled", &enabled)
       && config_setting_lookup_float(c, "height_above_wheels", &height_above_wheels)
       && config_setting_lookup_float(c, "takeoff_margin", &takeoff_margin)
       && config_setting_lookup_float(c, "callout_margin", &callout_margin)
       && config_setting_lookup_int(c, "log_points", &lidar_logpoints)
       && config_setting_lookup_string(c, "takeoff_filename", &lidar_takeoff_filename)
       && config_setting_lookup_int(c, "landing_log_runon_seconds", &lidar_landing_runon_seconds)
       && config_setting_lookup_string(c, "landing_filename", &lidar_landing_filename))) {
      dbprintf("lidar: Configuration file error, lidar area\n");
      lidar_ok = 0;
    }

    cc = config_lookup(&cfg, (lidar_testmode) ? "lidar_test.callouts" : "lidar.callouts");
    if (cc) {
      lidar_num_callouts = config_setting_length(cc);
      if (lidar_num_callouts > 0) {
        lidar_callouts = malloc(lidar_num_callouts * sizeof(struct callouts));

        if (lidar_noise)
          dbprintf("cc is valid, ncallouts %d!\n", lidar_num_callouts);

        if (lidar_callouts) {
          struct callouts *l;
          int i;

          for (i = 0, l = lidar_callouts; i < lidar_num_callouts; i++, l++) {
            config_setting_t *callout = config_setting_get_elem(cc, i);
            
            if (!(config_setting_lookup_float(callout, "limit", &l->limit)
               && config_setting_lookup_int(callout, "say_feet", &l->say_feet)
               && config_setting_lookup_string(callout, "text", &l->text))) {
                dbprintf("Error in configuration file, lidar_callout element %d\n", i);
                lidar_ok = 0;
                break;
              }
            if (i == 3)
              lidar_callouts_declare_takeoff = l;
            if (lidar_noise)
              dbprintf("lidar: %d: Adding limit \"%.1f\", say_feet \"%d\", text \"%s\"\n",
                    i, l->limit, l->say_feet, l->text);
            }
          }
        }
      }
    } else {
      lidar_ok = 0;
    }

  if (lidar_ok && enabled) {
    lidar_init();
  }
}

#ifdef INCLUDE_FLAPS
static void mo_update_flap_config_settings()
{
  config_setting_t *c;

  if ((c = config_lookup(&cfg, "flaps.reflex")))
    config_setting_set_int(c, (int)flaps.reflex_position);
  if ((c = config_lookup(&cfg, "flaps.zero")))
    config_setting_set_int(c, (int)flaps.zero_position);
  if ((c = config_lookup(&cfg, "flaps.half")))
    config_setting_set_int(c, (int)flaps.half_position);
  if ((c = config_lookup(&cfg, "flaps.full")))
    config_setting_set_int(c, (int)flaps.full_position);
  if ((c = config_lookup(&cfg, "flaps.deadband")))
    config_setting_set_int(c, (int)flaps.deadband);
  if ((c = config_lookup(&cfg, "flaps.sensor_reversed")))
    config_setting_set_int(c, (int)flaps_sensor_reverse);
}

static void mo_write_config(char *fname)
{
  if (!config_write_file(&cfg, fname)) {
    dbprintf("Error writing to new config file %s\n", fname);
  }
}
#endif


////////////////////////////////////////////////////////////////////////////////
// If the TFT display dies, we remember the state we are in, and restart
// back at the same point.
////////////////////////////////////////////////////////////////////////////////

#define RESTART_MAGIC 0x42135a78

typedef struct restart_state {
  uint32_t magic;
  dstate_t state;
  cltype_t checklist_state;
  int pilot_in_command;
  uint8_t efi_left_coil_disabled;
  uint8_t efi_right_coil_disabled;
  uint8_t efi_injectors_disabled;
  uint32_t crc;
  } restart_state_t;

static restart_state_t restart_state = {0};

void die_and_restart(void)
{
  int fd;

  dbprintf("die_and_restart: Entered\n");

  lidar_thread_abort = 1;
  efi_thread_abort = 1;
  vib_thread_abort = 1;
  etx_thread_abort = 1;
  hf_thread_abort = 1;
  tick_thread_abort = 1;
  flaps_thread_abort = 1;

  mo_usleep(10000);

  // Save the current state, for use in the restart
  fd = open(restart_state_filename, O_CREAT|O_WRONLY|O_TRUNC, S_IRWXU|S_IRWXG);
  if (fd >= 0) {
    restart_state.magic = RESTART_MAGIC;
    restart_state.state = dl->ds;
    restart_state.checklist_state = checklist_state;
    restart_state.pilot_in_command = pilot_in_command;
    restart_state.efi_left_coil_disabled = efi_left_coil_disabled;
    restart_state.efi_right_coil_disabled = efi_right_coil_disabled;
    restart_state.efi_injectors_disabled = efi_injectors_disabled;
    restart_state.crc = mo_crc32((unsigned char *)&restart_state, sizeof(struct restart_state) - 4, 0);
    write(fd, &restart_state, sizeof(restart_state));
    close(fd);
    dbprintf("die_and_restart: Saved current state as %d\n", restart_state.state);
  }

  exit(0);
}

static void check_for_crash_recovery()
{
  int fd;
  int n;

  fd = open(restart_state_filename, O_RDONLY);
  if (fd >= 0) {
    if ((n = read(fd, &restart_state, sizeof(restart_state))) == sizeof(restart_state)) {
      if (restart_state.magic == RESTART_MAGIC) {
        if (restart_state.crc == mo_crc32((unsigned char *)&restart_state, sizeof(struct restart_state) - 4, 0)) {
          initial_screen = restart_state.state;
          checklist_state = restart_state.checklist_state;
          pilot_in_command = restart_state.pilot_in_command;
          efi_left_coil_disabled = restart_state.efi_left_coil_disabled;
          efi_right_coil_disabled = restart_state.efi_right_coil_disabled;
          efi_injectors_disabled = restart_state.efi_injectors_disabled;
          dbprintf("check_for_crash_recovery: restart file exists, setting initial state to %d\n", initial_screen);
          }
        }
      }
    close(fd);
    n = rename(restart_state_filename, "old_restart_file");         // Get rid of it, but save a copy
    if (n != 0) {
      perror("rename in check_for_crash_recovery");
      dbprintf("check_for_crash_recovery: File rename failed, returned %d\n", n);
      }
    }
}

