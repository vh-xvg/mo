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

//////////////////////////////////////////////////////////////////////////////
//
// A collection of functions for VH-XVG's checklists and operations
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <errno.h>

#include "eve_config.h"
#include "eve_lib.h"
#include "hw_api.h"
#include "eve_xvg.h"
#include "tts.h"
#include "efi_protocol.h"
#include "eve_vibration_display.h"

// Time in msec to lock out touch polling after each successful swipe/action.
// Avoids accidental double display changes etc.
#define LOCKOUT_MSEC    1

// Debounce time in msec
#define EVE_DEBOUNCE_MS  ((double)1.0)

// Default swipe gaps, minimum pixes before a swipe is recognized
#define SWIPE_GAP_X         80
#define SWIPE_GAP_Y         30

#if 0
/*
 * Minimum total finger travel before a gesture is accepted as a swipe.
 *
 * The caller-provided swipe_gap values are still used, but these values are
 * hard lower limits.  This prevents small finger movement during a tap from
 * being misclassified as a swipe.  Override these from the build if needed.
 */
#ifndef TOUCH_MIN_HORIZONTAL_SWIPE_DISTANCE_PIXELS
#define TOUCH_MIN_HORIZONTAL_SWIPE_DISTANCE_PIXELS  (DWIDTH / 4)
#endif

#ifndef TOUCH_MIN_VERTICAL_SWIPE_DISTANCE_PIXELS
#define TOUCH_MIN_VERTICAL_SWIPE_DISTANCE_PIXELS    (DHEIGHT / 4)
#endif
#endif


// Some colours
#define EGREEN              0x148026
//#define EGREEN_PRESSED      RGB(78,115,62)
#define EGREEN_PRESSED      RGB(0,255,0)

#define ECYAN                RGB(0,255,255)

#define ERED                RGB(255,0,0)
#define EHALFRED            RGB(127,0,0)
#define EDARKRED            RGB(80,0,0)
#define ECRIMSON            RGB(102,20,60)
#define EAMBER              RGB(0xff,0xbf,0x0)

#define EBANANA             RGB(0xb9,0xb9,0)
#define EORANGE             RGB(0xb9,0x73,0)
#define EORANGE_PRESSED     RGB(0xb9,0x73,0xff)
#define EGRAY               RGB(0x80,0x80,0x80)
#define EDARK_GRAY          RGB(0x20,0x20,0x20)
#define EBLUE               RGB(0,0,255)
#define EBLUE_PRESSED       RGB(37,37,166)

#define DIMENSION(x) (sizeof(x)/sizeof(x[0]))

static int eve_checklist_signals(void);

extern plot_data_t *efi_get_plot_data(uint8_t, uint32_t, uint32_t, uint32_t);
extern void efi_get_stats_update();
extern void flaps_request_data();
extern char *mo_software_version;
extern struct config_data efi_board_config;
extern struct display_data display;
extern _Atomic uint8_t fuelpump_on[FUELPUMPS];
extern uint8_t implied_ecu_failure;            // 0x1 == primary ECU, 0x2 == secondary ECU
extern _Atomic uint8_t left_battery_fault;
extern _Atomic uint8_t right_battery_fault;
extern uint32_t msec_tick;
extern float avg_lidar_distance;
extern sig_atomic_t mo_abort;
extern _Atomic int vib_thread_abort;
extern uint8_t vib_live_rotations;
extern uint8_t vib_snapshot_rotations;
extern int vib_rotations;
extern struct efi_options options;

extern _Atomic float air_fuel_ratio;
extern _Atomic uint8_t o2_sensor_power_is_on;
extern _Atomic uint8_t o2_sensor_not_ready;
extern _Atomic uint8_t o2_sensor_error;
extern _Atomic uint32_t primary_rpm;                   // As measured from SDSEFI pulses
extern _Atomic uint32_t secondary_rpm;
extern uint16_t saved_average_current[NUM_CURRENTS];

volatile sig_atomic_t eve_do_screenshot = 0;

/*
 * Screenshot output selection.  This is intended to be set from the
 * configuration file by the main program.
 */

void check_screenshot() 
{
  struct timeval tv_now;
  struct tm t, *now = &t;
  char buf[64];

  if (eve_do_screenshot) {
    eve_do_screenshot = 0;
    gettimeofday(&tv_now, NULL);
    localtime_r(&tv_now.tv_sec, &t);
    snprintf(buf, sizeof(buf), "runtime/screenshots/screenshot_%04d%02d%02d_%02d%02d%02d", 
      now->tm_year + 1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);

    eve_screenshot(buf);
    }
}

void do_screenshot(void)
{
  eve_do_screenshot = 1;
  check_screenshot();
}

// Calibration data:
// "Good" MO39:
//      Cal: stamp 382e352e_31203931_2d37302d_37313032 TransMatrix[A-F] are 0x000102fb ffffef1d fff74f53 000001ff 0000f698 000010be
// "Bad" MO39:
//      Cal: stamp 382e352e_31203931_2d37302d_37313032 TransMatrix[A-F] are 0x00010397 000001f3 fff6b5de fffffe75 0000fffd fffd7bdf

void Calibrate(void)
{
#if defined TOUCH_CAPACITIVE
  wr32(REG_CTOUCH_EXTEND + RAM_REG, 0x1);           // Disable extended mode
#endif
  Calibrate_Manual(display.width, display.height, PIXVOFFSET, PIXHOFFSET);
#if defined TOUCH_CAPACITIVE
  wr32(REG_CTOUCH_EXTEND + RAM_REG, 0x0);           // Enable extended mode
#endif
}


////////////////////////////////////////////////////////////////////////////////
// Load initial screen
////////////////////////////////////////////////////////////////////////////////


void eve_blank()
{
  accrue_Send_CMD(CMD_DLSTART);                   // Start a new display list
  accrue_Send_CMD(VERTEXFORMAT(0));               // setup VERTEX2F to take pixel coordinates
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      // Determine the clear screen color
  accrue_Send_CMD(CLEAR(1, 1, 1));	          // Clear the screen and the curren display list
  accrue_Send_CMD(DISPLAY());                     // End the display list
  accrue_Send_CMD(CMD_SWAP);                      // Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
}


void eve_initial_screen()
{
  extern char initial_splash[];

  eve_send_image(initial_splash, 0, 0, 10);
}


////////////////////////////////////////////////////////////////////////////////
// DOORS open/closed functions
////////////////////////////////////////////////////////////////////////////////

struct door_pts {
  int16_t x, y, w, h, font;
  } door_points[] = {
    {30, (128-80)/2, 120, 80, 28},
    {180, (128-80)/2, 120, 80, 28},
    {350, (128-80)/2, 100, 80, 28}
  };

void eve_doors_page(uint8_t door_state)
{
  int i, idx;
  struct door_pts *d;
  const char *door_text[] = {
        "Left Door\nOPEN", "Right Door\nOPEN", "Baggage\nOPEN",
        "Left Door\nCLOSED", "Right Door\nCLOSED", "Baggage\nCLOSED" };

  Wait4CoProFIFOEmpty();
  accrue_Send_CMD(CMD_DLSTART);                   // Start a new display list
  accrue_Send_CMD(VERTEXFORMAT(0));               // Setup VERTEX2F to take pixel coordinates
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      // Determine the clear screen color
  accrue_Send_CMD(CLEAR(1, 1, 1));	          // Clear the screen and the curren display list

  for (i = 0, d = door_points; i < 3; i++, d++) {
    if (door_state & (1<<i)) {
      idx = 3+i;
      Cmd_FGcolor(0x148026);                    // A green you can see white text on
      } else {
      idx = i;
      Cmd_FGcolor(0xb90007);                    // Red-ish
      }
    accrue_Send_CMD(TAG(i));
    Cmd_Button(d->x, d->y, d->w, d->h, d->font, OPT_FLAT, door_text[idx]);
    release_wr32();
    UpdateFIFO();
    }

  accrue_Send_CMD(DISPLAY());                     // End the display list
  accrue_Send_CMD(CMD_SWAP);                      // Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
  check_screenshot();
}

////////////////////////////////////////////////////////////////////////////////
//
// Security page functions
//
////////////////////////////////////////////////////////////////////////////////

void eve_draw_security_page(uint8_t pressed)
{
  uint16_t opt_toprow, opt_botrow;

  if ('0' <= pressed && pressed <= '4')
    opt_toprow = pressed;
  else
    opt_toprow = 0;
  if ('5' <= pressed && pressed <= '9')
    opt_botrow = pressed;
  else
    opt_botrow = 0;

  accrue_Send_CMD(CMD_DLSTART);                   //Start a new display list
  accrue_Send_CMD(VERTEXFORMAT(0));               //setup VERTEX2F to take pixel coordinates
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      //Determine the clear screen color
  accrue_Send_CMD(CLEAR(1, 1, 1));	             //Clear the screen and the curren display list

  //accrue_Send_CMD(COLOR_RGB(26, 26, 192));        // change colour to blue
  Cmd_FGcolor((26<<16)|(26<<8)|192);

  Cmd_Keys(180, 5, 290, 55, 30, opt_toprow, "01234");
  Cmd_Keys(180, 65, 290, 55, 30, opt_botrow, "56789");

  accrue_Send_CMD(display.normal_label_color);      //Change color to white for text
  Cmd_Text(5, display.height / 2, 30, OPT_CENTERY, "Enter code:");

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
}

extern struct pilot_data *pilots;           // Array of pilot_data structures
extern int sec_num_pilots;                  // Number of pilots in security database
extern int pilot_in_command;                // Index of PIC

int eve_security_page()
{
  int i;
  struct pilot_data *p;


  // Initialize all counters
  for (i = 0, p = pilots; i < sec_num_pilots; i++, p++)
    p->idx = 0;

  eve_draw_security_page(0);
  tts_say("Enter security code");
  while (!mo_abort)
    {
    uint8_t tag = eve_debounce_tagread(&eve_draw_security_page, 1);
    if (tag > 0)
      if ('0' <= tag && tag <= '9') {
        mo_usleep(20000);
        eve_draw_security_page(0);

        for (i = 0, p = pilots; i < sec_num_pilots; i++, p++) {
          //dbprintf("%-20s: idx %d code %c tag %c\n", p->name, p->idx, p->code[p->idx], tag);
          if (p->code[p->idx] == tag) {
            if (++p->idx == p->code_length) {
              return(i);
              break;
              }
          } else {
            p->idx = 0;
          }
        }
      }
    check_screenshot();
    mo_usleep(20000);
    }

  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Ready to START page
////////////////////////////////////////////////////////////////////////////////

uint8_t eve_ready_to_start(int announce)
{
  accrue_Send_CMD(CMD_DLSTART);
  accrue_Send_CMD(VERTEXFORMAT(0));
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  accrue_Send_CMD(CLEAR(1, 1, 1));

  accrue_Send_CMD(COLOR_RGB(0, 255, 0));            // GREEN
  //accrue_Send_CMD(TAG(1));
  Cmd_Text(display.width / 2, display.height / 2, 31, OPT_CENTER, "Ready to START");

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  if (announce)
    tts_say("Ready to start");
  Wait4CoProFIFOEmpty();
  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Checklist controls
////////////////////////////////////////////////////////////////////////////////

#define CHKW 220

struct checklist_pts {
  int16_t x, y, w, h, font;
  } checklist_points[] = {
    { 10,             (DHEIGHT-80)/2+20,   100, 80, 30},
    {DWIDTH/2-CHKW/2, (DHEIGHT-80)/2+20, CHKW, 80, 30},
    {370,             (DHEIGHT-80)/2+20,   100, 80, 30}
  };


uint8_t eve_checklist_display(char *name, int disabled)
{
  int i;
  char *p;
  struct checklist_pts *c;
  char buf[256];

  accrue_Send_CMD(CMD_DLSTART);
  accrue_Send_CMD(VERTEXFORMAT(0));
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  accrue_Send_CMD(CLEAR(1, 1, 1));
  accrue_Send_CMD(COLOR_RGB(255, 255, 255));      //Change color to white for text

  for (i = 0, c = checklist_points; i < 3; i++, c++) {
    p = (i == 0) ? "PREV" : (i == 1) ? buf : "NEXT";
    if (i == 1 && disabled) {
      snprintf(buf, sizeof(buf), "%s\n%s", name, "DISABLED");
      Cmd_FGcolor(0x550010);
    } else {
      snprintf(buf, sizeof(buf), "%s", name);
      Cmd_FGcolor(0x148026);                    // A green you can see white text on
    }
    accrue_Send_CMD(TAG(i+1));
    Cmd_Button(c->x, c->y, c->w, c->h, c->font, 0, p);
    release_wr32();
    UpdateFIFO();
    }

  eve_checklist_signals();

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
  return(0);
}

uint8_t eve_checklist(char *name, void (*action)(uint8_t, uint8_t), uint8_t arg, uint8_t *disabled, uint8_t allow_disable)
{
  int do_speech = 0;
  int loops;
  uint8_t tag;
  char buf[64];

  for (loops = 0; !mo_abort; loops++) {
    if (do_speech || loops == 0 || (loops % 10) == 9) 
      eve_checklist_display(name, *disabled);       // So we update around 5 times per second

    if (do_speech) {
      tts_say(buf);
      do_speech = 0;
      }

    tag = eve_debounce_tagread(NULL, 0);
    //if (tag != 0)
    //  dbprintf("checklist tag is %d, allow_disable %d\n", tag, allow_disable);
    switch (tag) {
      case 1:
        if (!allow_disable && *disabled)               // Not allowed to return disabled
          continue;
        return CL_PREV;
        break;
      
      case 2:
        snprintf(buf, sizeof(buf), "%s %s", name, (*disabled) ? "enabled" : "disabled");
        if (*disabled)
          (*action)(arg, 1);        // Enable it

        else
          (*action)(arg, 0);        // Disable it
        *disabled = (*disabled) ? 0 : 1;
        do_speech = 1;
        mo_usleep(10000);
        break;

      case 3:
        if (!allow_disable && *disabled)
          continue;
        return CL_NEXT;
        break;

      default:
        mo_usleep(10000);
        break;
      }

    check_screenshot();
    }

  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Sending an inline image
////////////////////////////////////////////////////////////////////////////////

void eve_send_image(char *filename, uint32_t ram_g_address, uint32_t x_pos, uint32_t y_pos)
{
  uint32_t *crap, *p;
  int fd;
  int i, n;

#define MAX_FILESIZE 65536

  crap = malloc(MAX_FILESIZE);
  if (!crap)
    return;
  memset(crap, 0, MAX_FILESIZE);
  fd = open(filename, 0);
  if (fd < 0) {
    free(crap);
    return;
    }
  n = read(fd, crap, MAX_FILESIZE);
  if (n <= 0 || n == MAX_FILESIZE) {
    fprintf(stderr, "eve_send_image, problem with file \"%s\": n %d\n", filename, n);
    close(fd);
    free(crap);
    return;
  }

  Wait4CoProFIFOEmpty();

  Send_CMD(CMD_DLSTART);                  // Start a new display list
  Send_CMD(VERTEXFORMAT(0));              // Setup VERTEX2F to take pixel coordinates
  //Send_CMD(BITMAP_HANDLE(1));
  Send_CMD(CMD_LOADIMAGE);
  Send_CMD(ram_g_address);                // RAM_G Address
  Send_CMD(0);
  //Send_CMD(OPT_NODL);
  UpdateFIFO();

  for (i = 0, p = crap; i < n; i += 4, p++) {
    if ((i % 2048) == 2044) {
       release_Send_CMD();                // Send ~2KB chunks
       UpdateFIFO();
       Wait4CoProFIFOEmpty();
      }
    accrue_Send_CMD(*p);
    }
  release_Send_CMD();
  UpdateFIFO();
  close(fd);
  free(crap);
  Wait4CoProFIFOEmpty();

  accrue_Send_CMD(BEGIN(BITMAPS));
  accrue_Send_CMD(VERTEX2F(x_pos, y_pos));
  accrue_Send_CMD(DISPLAY());              // End the display list
  accrue_Send_CMD(CMD_SWAP);               // Swap commands into RAM
  release_Send_CMD();
  UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO
  Wait4CoProFIFOEmpty();

}

////////////////////////////////////////////////////////////////////////////////
//
// Key/Button debounce
//
////////////////////////////////////////////////////////////////////////////////

uint8_t eve_debounce_tagread(void (*callback)(uint8_t), int wait_forever) {
  static struct timespec now, last_debounce_time;
  uint8_t last_tag = 0;
  uint8_t tag, got_keystroke;
  uint8_t first = 1;

  clock_gettime(CLOCK_MONOTONIC, &now);
  last_debounce_time.tv_sec = now.tv_sec;
  last_debounce_time.tv_nsec = now.tv_nsec;

  got_keystroke = 0;
  while (!mo_abort && (first || wait_forever || got_keystroke)) {
    check_screenshot();
    first = 0;
    tag = rd8(REG_TOUCH_TAG + RAM_REG);                     // Check for touches
    uint32_t sxy0 = rd32(REG_CTOUCH_TOUCH_XY + RAM_REG);

    //printf("eve_debounce_tagread: tag %d, sxy0 0x%08x\n", tag, sxy0);
    if (tag == 0 && sxy0 != 0x80008000)
      tag = 255;

    clock_gettime(CLOCK_MONOTONIC, &now);
    long seconds = now.tv_sec - last_debounce_time.tv_sec;
    long nanoseconds = now.tv_nsec - last_debounce_time.tv_nsec;
    double elapsed_msec = seconds*1e3 + nanoseconds*1e-6;

    if (tag > 0) {
      //printf("eve_debounce_tagread: tag %d, elapsed %.1f\n", tag, elapsed_msec);
      got_keystroke = 1;

      if (tag != last_tag) {
        last_debounce_time.tv_sec = now.tv_sec;
        last_debounce_time.tv_nsec = now.tv_nsec;
        last_tag = tag;
        elapsed_msec = 0.0;
        }
    } else {
      //printf("eve_debounce_tagread: tag %d, got_keystroke %d elapsed %.1f debounce %.1f\n", tag, got_keystroke, elapsed_msec, EVE_DEBOUNCE_MS);
      if (got_keystroke && (elapsed_msec > EVE_DEBOUNCE_MS)) {
        play_sound("bleep");
        if (callback) {
          (*callback)(last_tag);
          }
        return(last_tag);
      }
    }
    //if (wait_forever || got_keystroke)
    mo_usleep(5000);
  }

  return(0);                            // No tag
}

////////////////////////////////////////////////////////////////////////////////
//
// Swipe/Key detection
//
////////////////////////////////////////////////////////////////////////////////

#define CT_PTS 4
#define CT_HIS 8

typedef struct ctxy {
  int16_t x;
  int16_t y;
} ctxy_t;

typedef struct ct_all {
  uint32_t valid_bits;
  struct ctxy xy[CT_PTS][CT_HIS];
} ct_all_t;

static int wait_more_passes = 0;
static struct ct_all ct = {0};

//#define TOUCH_DEBUG

#ifdef TOUCH_DEBUG
static struct timespec touch_time[CT_HIS];
int swipe_debug = 1;
#else
int swipe_debug = 0;
#endif

static void print_ct()
{
  int i, h;
#ifdef TOUCH_DEBUG
  struct timespec *t, *last_t;
  int t_valid = 0, last_t_valid = 0;
#endif

  dbprintf("print_ct: valid_bits = 0x%x\n", ct.valid_bits);
  for (h = 0; h < CT_HIS; ++h) {
    printf("     %d: ", (ct.valid_bits & (1<<h)) ? 1 : 0);
    for (i = 0; i < CT_PTS; i++)
      dbprintf("[%d] = %6d,%6d  ", i, ct.xy[i][h].x, ct.xy[i][h].y);
#ifdef TOUCH_DEBUG
    t = &touch_time[h];
    t_valid = (ct.valid_bits & (1<<h)) ? 1 : 0;
    if (h > 0) {
      if (t_valid && last_t_valid) {
        long seconds = last_t->tv_sec - t->tv_sec;
        long nanoseconds = last_t->tv_nsec - t->tv_nsec;
        double elapsed_msec = seconds*1e3 + nanoseconds*1e-6;
        printf("   +%.2f msec", elapsed_msec);
        }
      }
    if (t_valid) {
      last_t = t;
      last_t_valid = t_valid;
      }
#endif
    printf("\n");
  }
}

static void shift_ct_left(struct ct_all *c)
{
  int i, h;

  uint32_t new_valid_bits = (c->valid_bits << 1) & ((0x1<<CT_HIS)-1);

#ifdef TOUCH_DEBUG
  if (swipe_debug)
    printf("shift_ct_left: valid_bits 0x%x, will become 0x%x\n", c->valid_bits, new_valid_bits);
#endif

  c->valid_bits = new_valid_bits;
  for (h = CT_HIS-1; h > 0; h--) {
    for (i = 0; i < CT_PTS; i++) {
      c->xy[i][h].x = c->xy[i][h-1].x;
      c->xy[i][h-1].x = 0;
      c->xy[i][h].y = c->xy[i][h-1].y;
      c->xy[i][h-1].y = 0;
      }
#ifdef TOUCH_DEBUG
    touch_time[h] = touch_time[h-1];
#endif
    }

#ifdef TOUCH_DEBUG
  printf("end of shift_ct_left, ct is:\n");
  print_ct();
#endif
}

static void shift_ct_right(struct ct_all *c)
{
  int i, h;

#ifdef TOUCH_DEBUG
  if (swipe_debug)
    printf("shift_ct_right: valid_bits 0x%x, will become 0x%x\n", c->valid_bits, c->valid_bits >> 1);
#endif

  c->valid_bits >>= 1;
  for (h = 0; h <= CT_HIS-2; h++) {
    for (i = 0; i < CT_PTS; i++) {
      c->xy[i][h].x = c->xy[i][h+1].x;
      c->xy[i][h].y = c->xy[i][h+1].y;
      }
#ifdef TOUCH_DEBUG
    touch_time[h] = touch_time[h+1];
#endif
    }
  for (i = 0; i < CT_PTS; i++) {
    c->xy[i][CT_HIS-1].x = 0;
    c->xy[i][CT_HIS-1].y = 0;
    }
#ifdef TOUCH_DEBUG
    touch_time[CT_HIS-1].tv_sec = 0;
    touch_time[CT_HIS-1].tv_nsec = 0;

    printf("end of shift_ct_right, ct is:\n");
    print_ct();
#endif

}

static inline void remove_ct_hole(struct ct_all *c, int idx)
{
  int i, h;

  uint32_t keep_mask = (0x1 << idx) - 1;
  uint32_t shift_mask = ~keep_mask & ((unsigned)(0x1<<CT_HIS)-1);

#ifdef TOUCH_DEBUG
  if (swipe_debug)
    printf("remove_ct_hole, idx %d, keep_mask 0x%08x shift_mask 0x%08x c->valid_bits 0x%x\n", idx, keep_mask, shift_mask, c->valid_bits);
#endif
  c->valid_bits = (c->valid_bits & keep_mask) | ((c->valid_bits & shift_mask) >> 1);
#ifdef TOUCH_DEBUG
  if (swipe_debug)
    printf("remove_ct_hole, c->valid_bits is now 0x%x\n", c->valid_bits);
#endif
  for (h = idx; h < CT_HIS-2; h++) {
    for (i = 0; i < CT_PTS; i++) {
      c->xy[i][h].x = c->xy[i][h+1].x;
      c->xy[i][h].y = c->xy[i][h+1].y;
      }
#ifdef TOUCH_DEBUG
    touch_time[h] = touch_time[h+1];
#endif
    }
  // Empty slot at end
  for (i = 0; i < CT_PTS; i++) {
    c->xy[i][CT_HIS-1].x = 0;
    c->xy[i][CT_HIS-1].y = 0;
    }
}

static void clear_ct(void)
{
  memset(&ct, 0, sizeof(ct));
#ifdef TOUCH_DEBUG
  memset(&touch_time[0], 0, sizeof(touch_time));
#endif
}


static inline int ct_point_valid(const struct ctxy *p)
{
  return (p->x != -32768 && p->y != -32768);
}

static int ct_count_valid_points_at(int h)
{
  int i, n = 0;

  for (i = 0; i < CT_PTS; i++)
    if (ct_point_valid(&ct.xy[i][h]))
      n++;

  return n;
}

static int ct_centroid_for_first_two_points(int h, int *x, int *y)
{
  int i, n = 0;
  int sx = 0, sy = 0;

  for (i = 0; i < CT_PTS; i++) {
    if (ct_point_valid(&ct.xy[i][h])) {
      sx += ct.xy[i][h].x;
      sy += ct.xy[i][h].y;
      if (++n == 2)
        break;
    }
  }

  if (n < 2)
    return 0;

  *x = sx / 2;
  *y = sy / 2;
  return 1;
}

static int ct_is_double_touch(int touch_debug)
{
  int h;
  int entries = 0;
  int first = 1;
  int min_x = 0, max_x = 0, min_y = 0, max_y = 0;
  int max_centroid_move;

  /*
   * A double-touch is intended as a deliberate, non-navigation gesture:
   * two fingers present for several samples, with little centroid movement.
   * This avoids stealing single-touch plot selections and avoids confusing
   * two-finger movement with the normal one-finger swipe classifier.
   */
  max_centroid_move = display.height / 6;
  if (max_centroid_move < 16)
    max_centroid_move = 16;
  if (max_centroid_move > 30)
    max_centroid_move = 30;

  for (h = 0; h < CT_HIS; h++) {
    int x, y;

    if (!(ct.valid_bits & (1 << h)))
      continue;

    if (ct_count_valid_points_at(h) < 2)
      continue;

    if (!ct_centroid_for_first_two_points(h, &x, &y))
      continue;

    if (first) {
      min_x = max_x = x;
      min_y = max_y = y;
      first = 0;
    } else {
      if (x < min_x) min_x = x;
      if (x > max_x) max_x = x;
      if (y < min_y) min_y = y;
      if (y > max_y) max_y = y;
    }

    entries++;
  }

  if (touch_debug)
    dbprintf("double-touch check: entries %d, centroid span %d,%d limit %d\n",
             entries, max_x - min_x, max_y - min_y, max_centroid_move);

  if (entries < 3)
    return 0;

  if ((max_x - min_x) > max_centroid_move)
    return 0;

  if ((max_y - min_y) > max_centroid_move)
    return 0;

  return 1;
}

int check_touch(int even_sections, int swipe_gap, int touch_debug,
                int row2_sections, int row2_swipe_gap)
{
  static uint8_t forced_injector_summary = 0;
  static struct timespec begin_lockout = {0};
  static struct timespec last_touch_check = {0};

  struct timespec now;
  uint32_t sxy0, sxyA, sxyB, sxyC;
  int i, j;
  int evaluate_swipe = 0;
  int evaluate_single = 0;

  int section_width = (even_sections > 0) ? display.width / even_sections : 0;
  int row2_section_width = (row2_sections > 0) ? display.width / row2_sections : 0;

  /*
   * Horizontal and vertical thresholds.
   * swipe_gap keeps its old meaning for left/right swipes.
   * row2_swipe_gap is reused as the vertical swipe threshold if supplied.
   */
  int x_swipe_gap = (swipe_gap > 0) ? swipe_gap : display.width / 10;
  int y_swipe_gap = (row2_swipe_gap > 0) ? row2_swipe_gap : display.height / 5;
  int tap_max_move = display.height / 8;

  if (tap_max_move < 8)
    tap_max_move = 8;

  clock_gettime(CLOCK_MONOTONIC, &now);

  long seconds = now.tv_sec - begin_lockout.tv_sec;
  long nanoseconds = now.tv_nsec - begin_lockout.tv_nsec;
  double elapsed_msec = seconds * 1e3 + nanoseconds * 1e-6;

  if (implied_ecu_failure && !forced_injector_summary) {
    forced_injector_summary = 1;
    return FORCE_INJECTOR_SUMMARY;
  }

  if (elapsed_msec < LOCKOUT_MSEC)
    return 0;

  seconds = now.tv_sec - last_touch_check.tv_sec;
  nanoseconds = now.tv_nsec - last_touch_check.tv_nsec;
  elapsed_msec = seconds * 1e3 + nanoseconds * 1e-6;

  if (elapsed_msec < 20)
    return 0;

  last_touch_check = now;

  sxy0 = rd32(REG_CTOUCH_TOUCH_XY + RAM_REG);

  if (sxy0 != 0x80008000) {
    if (touch_debug) {
      dbprintf("sxy0 is 0x%08x, ct is:\n", sxy0);
      print_ct();
    }

    if (ct.valid_bits & 0x1)
      shift_ct_left(&ct);

    sxyA = rd32(REG_CTOUCH_TOUCH1_XY + RAM_REG);
    sxyB = rd32(REG_CTOUCH_TOUCH2_XY + RAM_REG);
    sxyC = rd32(REG_CTOUCH_TOUCH3_XY + RAM_REG);

    ct.xy[0][0].x = sxy0 >> 16;
    ct.xy[0][0].y = sxy0;
    ct.xy[1][0].x = sxyA >> 16;
    ct.xy[1][0].y = sxyA;
    ct.xy[2][0].x = sxyB >> 16;
    ct.xy[2][0].y = sxyB;
    ct.xy[3][0].x = sxyC >> 16;
    ct.xy[3][0].y = sxyC;

    if (
        ct.xy[0][0].x == -32768 &&
        ct.xy[0][0].y == -32768 &&
        ct.xy[1][0].x == -32768 &&
        ct.xy[1][0].y == -32768 &&
        ct.xy[2][0].x == -32768 &&
        ct.xy[2][0].y == -32768) {
      if (touch_debug)
        dbprintf("sxy0 is 0x%08x, but no points\n", sxy0);
    } else {
      ct.valid_bits |= 0x1;

#ifdef TOUCH_DEBUG
      if (touch_debug)
        clock_gettime(CLOCK_MONOTONIC, &touch_time[0]);
#endif

      if (touch_debug) {
        dbprintf("touch sample: valid_bits %x, xy0 %d,%d\n",
                 ct.valid_bits, ct.xy[0][0].x, ct.xy[0][0].y);
        print_ct();
      }

      wait_more_passes = 2;
    }
  } else {
    if (wait_more_passes && touch_debug) {
      dbprintf("wait_more_passes %d, ct is:\n", wait_more_passes);
      print_ct();
    }

    if (wait_more_passes > 0 && --wait_more_passes == 0) {
      if (touch_debug) {
        dbprintf("--wait_more_passes is now zero, ct is:\n");
        print_ct();
      }

      /*
       * Rotate history so index zero is the most recent valid sample.
       */
      for (j = 0; j < CT_HIS - 1; j++) {
        if (ct.valid_bits & (0x1 << j))
          break;
      }

      if (j > 0)
        while (j--)
          shift_ct_right(&ct);

      /*
       * Remove empty holes between valid samples.
       */
      for (j = 1; j < CT_HIS - 1;) {
        if (!(ct.valid_bits & (0x1 << j))) {
          remove_ct_hole(&ct, j);
          if (ct.valid_bits & ~(((uint32_t)0x1 << j) - 1))
            continue;
          else
            break;
        } else {
          j++;
        }
      }

      if (touch_debug) {
        dbprintf("after debounce, ct is:\n");
        print_ct();
      }

      if (ct.valid_bits & 0x1) {
        if (ct.valid_bits & 0x2)
          evaluate_swipe = 1;
        else
          evaluate_single = 1;
      }
    }
  }

  if (evaluate_swipe && ct_is_double_touch(touch_debug)) {
    clock_gettime(CLOCK_MONOTONIC, &begin_lockout);
    clear_ct();
    play_sound("double_touch");
    return DOUBLE_TOUCH;
  }

  if (evaluate_swipe) {
    int entries = 0;
    int newest, oldest;
    int x0, y0, x1, y1;
    int dx, dy, adx, ady;

    for (i = 0; i < CT_HIS; i++)
      if (ct.valid_bits & (0x1 << i))
        entries++;

    newest = 0;
    oldest = entries - 1;

    x0 = ct.xy[0][oldest].x;
    y0 = ct.xy[0][oldest].y;
    x1 = ct.xy[0][newest].x;
    y1 = ct.xy[0][newest].y;

    if (x0 == -32768 || y0 == -32768 || x1 == -32768 || y1 == -32768) {
      clear_ct();
      return 0;
    }

    dx = x1 - x0;
    dy = y1 - y0;
    adx = abs(dx);
    ady = abs(dy);

    if (touch_debug) {
      dbprintf("gesture: entries %d, old %d,%d new %d,%d dx %d dy %d\n",
               entries, x0, y0, x1, y1, dx, dy);
    }

    /*
     * Decide swipe direction from the dominant axis.
     * Require the dominant axis to be at least 1.5x the other axis,
     * to avoid turning diagonal gestures into accidental navigation.
     */
    if (adx >= x_swipe_gap && adx * 100 >= ady * 150) {
      clock_gettime(CLOCK_MONOTONIC, &begin_lockout);
      clear_ct();

      if (dx < 0) {
        play_sound("swipe_left");
        return SWIPE_LEFT;
      } else {
        play_sound("swipe_right");
        return SWIPE_RIGHT;
      }
    }

    if (ady >= y_swipe_gap && ady * 100 >= adx * 150) {
      clock_gettime(CLOCK_MONOTONIC, &begin_lockout);
      clear_ct();

      if (dy < 0) {
        play_sound("swipe_up");
        return SWIPE_UP;
      } else {
        play_sound("swipe_down");
        return SWIPE_DOWN;
      }
    }

    /*
     * Not a swipe. If it barely moved, evaluate as a tap.
     * Otherwise ignore it.
     */
    if (adx <= tap_max_move && ady <= tap_max_move)
      evaluate_single = 1;
    else {
      if (touch_debug)
        dbprintf("gesture ignored: not swipe, too much movement for tap\n");
      clear_ct();
      return 0;
    }
  }

  if (evaluate_single) {
    int x = ct.xy[0][0].x;
    int y = ct.xy[0][0].y;

    if (touch_debug)
      dbprintf("evaluate_single, x %d y %d\n", x, y);

    if (x == -32768 || y == -32768)
      return 0;

    /*
     * Optional second row: bottom half of the screen returns
     * even_sections + row2_section_number.
     */
    if (row2_sections > 0 && y > display.height / 2) {
      for (i = 1; i <= row2_sections; i++) {
        if (x < i * row2_section_width) {
          clock_gettime(CLOCK_MONOTONIC, &begin_lockout);
          clear_ct();
          play_sound("bleep");
          return even_sections + i;
        }
      }
    }

    if (even_sections > 0) {
      for (i = 1; i <= even_sections; i++) {
        if (x < i * section_width) {
          clock_gettime(CLOCK_MONOTONIC, &begin_lockout);
          clear_ct();
          play_sound("bleep");
          return i;
        }
      }
    }
  }

  return 0;
}

static int check_touch_while_wait_fifo_empty(int even_sections, int swipe_gap, int touch_debug, int row2_sections, int row2_swipe_gap)
{
  uint16_t free_space;
  int tag;

  while (!mo_abort)
     {
     if ((tag = check_touch(even_sections, swipe_gap, touch_debug, row2_sections, row2_swipe_gap)) > 0)
       return(tag);
     free_space = CoProFIFO_FreeSpace();
     if (free_space < 4092)
       mo_usleep(100);
     else
       return(0);
     }

  return(0);
}


////////////////////////////////////////////////////////////////////////////////
//
// Fake data generator
//
////////////////////////////////////////////////////////////////////////////////

// Fake injector baseline data, AMPS * 100
// 72 points across 12 msec
static uint8_t fake_injector_data[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 3, 8, 12, 16, 19, 22, 23, 24,
        22, 19, 25, 31, 36, 40, 44, 47, 50, 53, 56, 58,
        59, 60, 61, 61, 62, 62, 62, 62, 62, 62, 62, 62,
        62, 30, 10, 2, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static uint8_t noisy_injector_data[sizeof(fake_injector_data)];

static uint8_t fake_coilpack_data[480/4-8];
static uint8_t noisy_coilpack_data[480/4-8];

static uint8_t fake_fuelpump_data[480/4-8];
static uint8_t noisy_fuelpump_data[480/4-8];

uint8_t *get_fake_data(int type)
{
  int i, limit;
  uint8_t *p, *q, *r;
  uint8_t rmask;
  static int initialized = 0;
  int coil_peak = 60;

  if (!initialized) {
    int low = sizeof(fake_coilpack_data)/4;
    int high = 3 * low;
    for (i = low; i <= high; i++)
      fake_coilpack_data[i] = (coil_peak * (i - low))/(high - low);
    fake_coilpack_data[i++] = coil_peak * 2 / 3;
    fake_coilpack_data[i++] = coil_peak / 3;

    for (i = 0; i < sizeof(fake_fuelpump_data); i++)
      fake_fuelpump_data[i] = 50 + 8 * sin(((double)i * 2 * 3.14)/(double) (sizeof(fake_fuelpump_data)/8));

    initialized = 1;
  }

  // Make the data noisy, so we can see the updates
  if (type == DT_INJECTOR) {
    p = &fake_injector_data[0];
    q = &noisy_injector_data[0];
    limit = sizeof(fake_injector_data);
    rmask = 0x7;
  } else if (type == DT_COILPACK) {
    p = &fake_coilpack_data[0];
    q = &noisy_coilpack_data[0];
    limit = sizeof(fake_coilpack_data);
    rmask = 0x3;
  } else {
    p = &fake_fuelpump_data[0];
    q = &noisy_fuelpump_data[0];
    limit = sizeof(fake_fuelpump_data);
    rmask = 0x3;
  }

  r = q;
  for (i = 0; i < limit; i++, p++, q++) {
    if (*p)
      *q = *p + (rand() & rmask);
    else
      *q = 0;
    }
  return(r);
}


////////////////////////////////////////////////////////////////////////////////
//
// Graph data
//
////////////////////////////////////////////////////////////////////////////////


static void put_fault_cross(int x_center, int y_center, int x_width, int y_height, uint32_t color_rgb_cmd)
{

  accrue_Send_CMD(BEGIN(LINES));
  accrue_Send_CMD(color_rgb_cmd);
  accrue_Send_CMD(LINE_WIDTH(16*2));
  accrue_Send_CMD(VERTEX2F(x_center-x_width/2, y_center-y_height/2));
  accrue_Send_CMD(VERTEX2F(x_center+x_width/2, y_center+y_height/2));
  accrue_Send_CMD(VERTEX2F(x_center-x_width/2, y_center+y_height/2));
  accrue_Send_CMD(VERTEX2F(x_center+x_width/2, y_center-y_height/2));

}

// Fill in individual detail parts of single live displays

static int      eve_individual_index = 0;
static uint16_t latch_fault_settings = 0;
static int      reset_button_highlight = 0;

static void eve_individual_detail(uint32_t mask, plot_data_t *pd, uint16_t x_offset, char *but2_text)
{
  uint16_t font = 18;
  uint16_t button_font;
  uint16_t y_pos = 15;
  int index;
  char buf[100];

  // Stuff about some buttons
  int nbuttons = (but2_text) ? 2 : 1;
  int but_space = 15;
  int but_w = ((display.width - x_offset) - (nbuttons+1) * but_space) / nbuttons;
  int but_h = 50;
  int but1_x = x_offset + but_space;
  int but2_x = x_offset + nbuttons * but_space + ((nbuttons > 1) ? but_w : 0);
  //int but_y = display.height - but_h - 1;
  int but_y = display.height - but_h;

  for (index = 0; index < 10 && !(mask & (1<<index)); index++)
    ;
  eve_individual_index = index;

  // index now points to whatever this is
  accrue_Send_CMD(display.normal_label_color);                    // White
  if (index < CYLINDERS) {
    float duration = (float)pd->duration / 1000;
    float peak_current = (float)(pd->current_ma & PD_CURRENT_MASK) / 1000;

    button_font = 24;
    if (pd->fault == FAULT_WIRING) {
      snprintf(buf, sizeof(buf), "Fault: %s", "OVER-CURRENT");
      accrue_Send_CMD(COLOR_RGB(255, 0, 0));
      Cmd_Text(x_offset+but_space, y_pos, font, OPT_CENTERY, buf);
      accrue_Send_CMD(display.normal_label_color);
      if (reset_button_highlight)
        Cmd_FGcolor(EDARKRED);
      else
        Cmd_FGcolor(ERED);
    } else {
      if (pd->current_ma & PD_CURRENT_AVERAGE)
        accrue_Send_CMD(COLOR_RGB(255, 0, 0));
      if (pd->no_update)
        snprintf(buf, sizeof(buf), "Peak current X.XX Amps");
      else {
        if (pd->fault == FAULT_DEVICE) {
          accrue_Send_CMD(COLOR_RGB(255, 0, 0));
          snprintf(buf, sizeof(buf), "DEVICE FAULT");
          }
        else if (pd->current_ma & PD_CURRENT_AVERAGE)
          snprintf(buf, sizeof(buf), "Avg  current %4.2f Amps", peak_current);
        else
          snprintf(buf, sizeof(buf), "Peak current %4.2f Amps", peak_current);
        }
      Cmd_Text(x_offset+but_space-((nbuttons>1)?0:10), y_pos, font, OPT_CENTERY, buf);      // -10 to squeeze it in

      if (pd->current_ma & PD_CURRENT_AVERAGE) {
        Cmd_Text(x_offset+but_space-((nbuttons>1)?0:10), y_pos+25, font, OPT_CENTERY, "DC OFFSET DETECTED");
        Cmd_Text(x_offset+but_space-((nbuttons>1)?0:10), y_pos+50, font, OPT_CENTERY, "Check Wiring/Injector");
      } else {
        if (pd->no_update)
          snprintf(buf, sizeof(buf), "Duration      XX.XX msec");
        else
          snprintf(buf, sizeof(buf), "Duration    %5.2f msec", duration);
        Cmd_Text(x_offset+but_space-((nbuttons>1)?0:10), y_pos+25, font, OPT_CENTERY, buf);   // -10 to squeeze it in
        Cmd_FGcolor(EDARK_GRAY);
      }
    accrue_Send_CMD(display.normal_label_color);
    Cmd_FGcolor(EDARK_GRAY);
    }
  } else if (index < CYLINDERS+COILPACKS) {
    float dwell_time = (float)pd->duration / 1000;
    float peak_current = (float)(pd->current_ma & PD_CURRENT_MASK) / 1000;

    button_font = 23;
    if (pd->fault == FAULT_WIRING) {
      snprintf(buf, sizeof(buf), "Fault: %s", "OVER-CURRENT");
      accrue_Send_CMD(COLOR_RGB(255, 0, 0));
      Cmd_Text(x_offset+but_space, y_pos, font, OPT_CENTERY, buf);
      accrue_Send_CMD(display.normal_label_color);
      if (reset_button_highlight)
        Cmd_FGcolor(EDARKRED);
      else
        Cmd_FGcolor(ERED);
    } else {
      if (pd->current_ma & PD_CURRENT_AVERAGE)
        accrue_Send_CMD(COLOR_RGB(255, 0, 0));
      if (pd->fault == FAULT_DEVICE) {
        accrue_Send_CMD(COLOR_RGB(255, 0, 0));
        snprintf(buf, sizeof(buf), "DEVICE FAULT");
        }
      else if (pd->current_ma & PD_CURRENT_AVERAGE)
        snprintf(buf, sizeof(buf), "Avg  current : %.1f Amps", peak_current);
      else
        snprintf(buf, sizeof(buf), "Peak current : %.1f Amps", peak_current);
      Cmd_Text(x_offset+but_space, y_pos, font, OPT_CENTERY, buf);

      if (pd->current_ma & PD_CURRENT_AVERAGE) {
        Cmd_FGcolor(EAMBER);
        Cmd_Text(x_offset+but_space, y_pos+25, font, OPT_CENTERY, "DC OFFSET DETECTED");
        Cmd_Text(x_offset+but_space, y_pos+50, font, OPT_CENTERY, "Check Wiring/Coilpack");
      } else {
        snprintf(buf, sizeof(buf), "Dwell time   : %.1f msec", dwell_time);
        Cmd_Text(x_offset+but_space, y_pos+25, font, OPT_CENTERY, buf);

        if (!pd->fault) {
          snprintf(buf, sizeof(buf), "Implied RPM  : %4d", pd->derived_rpm);
          Cmd_Text(x_offset+but_space, y_pos+50, font, OPT_CENTERY, buf);
        }
      }
      accrue_Send_CMD(display.normal_label_color);
      Cmd_FGcolor(EDARK_GRAY);
    }
  } else {
    float run_time = (float)pd->duration / 60;
    float avg_current = (float)(pd->current_ma & PD_CURRENT_MASK) / 1000;
    float surge_current = (float)(pd->surge_ma) / 1000;
    float ripple = (float)(pd->ripple_ma) / 1000;

    button_font = 23;
    if (pd->fault == FAULT_WIRING) {
      snprintf(buf, sizeof(buf), "Fault: %s", "OVER-CURRENT");
      accrue_Send_CMD(COLOR_RGB(255, 0, 0));
      Cmd_Text(x_offset+but_space, y_pos, font, OPT_CENTERY, buf);
      accrue_Send_CMD(display.normal_label_color);
      if (reset_button_highlight)
        Cmd_FGcolor(EDARKRED);
      else
        Cmd_FGcolor(ERED);
    } else {
      if (pd->no_update)
        ;
      else if (pd->fault == FAULT_DEVICE) {
        accrue_Send_CMD(COLOR_RGB(255, 0, 0));
        snprintf(buf, sizeof(buf), "DEVICE FAULT");
        Cmd_Text(x_offset+but_space, y_pos, font, OPT_CENTERY, buf);
        }
      else {
        if (run_time > 0) {
          snprintf(buf, sizeof(buf), "Average current %.1f Amps", avg_current);
          Cmd_Text(x_offset+but_space, y_pos, font, OPT_CENTERY, buf);
          }
        }

      if (!pd->no_update && run_time > 0) {
        snprintf(buf, sizeof(buf), "Start inrush %.1f Amps", surge_current);
        Cmd_Text(x_offset+but_space, y_pos+17, font, OPT_CENTERY, buf);

        snprintf(buf, sizeof(buf), "Ripple %.2f Amps", ripple);
        Cmd_Text(x_offset+but_space, y_pos+34, font, OPT_CENTERY, buf);

        snprintf(buf, sizeof(buf), "Run time %.1f minutes", run_time);
        Cmd_Text(x_offset+but_space, y_pos+51, font, OPT_CENTERY, buf);
        }

      accrue_Send_CMD(display.normal_label_color);
      Cmd_FGcolor(EDARK_GRAY);
    }
  }

  accrue_Send_CMD(TAG_MASK(1));
  if (latch_fault_settings & (1 << index)) {
    Cmd_FGcolor(EORANGE);
    accrue_Send_CMD(TAG(3));
    Cmd_Button(but1_x, but_y, but_w, but_h, button_font, 0, "Latch fault");
    }
  else {
    Cmd_FGcolor(EGREEN);
    accrue_Send_CMD(TAG(3));
    Cmd_Button(but1_x, but_y, but_w, but_h, button_font, 0, "Auto fault");
    }

  if (but2_text != NULL) {
    Cmd_FGcolor(EBLUE);
    accrue_Send_CMD(TAG(2));
    Cmd_Button(but2_x, but_y, but_w, but_h, button_font, 0, but2_text);
    }

  if (reset_button_highlight > 0)
    reset_button_highlight--;
}

extern int simulate_injector_fault;
extern int simulate_left_coilpack_fault;
extern int simulate_left_fuelpump_fault;

static void eve_individual_button_pressed(uint8_t tag)
{
  // Maps Coilpack, Fuelpump display position back to hardware positions

  //dbprintf("eve_individual_button_pressed: tag %d\n", tag);
  if (tag == 3) {
    // Change state of "Latch fault"
    if (eve_individual_index < CYLINDERS) {
      // Changing to Latch fault mode is deliberately not supported in
      // the hardware for fuel injectors, because the associated wiring
      // represents an (extremely unlikely, but non-zero) single point
      // of failure.
      dbprintf("Change latch fault setting not allowed for injector (cylinder %d)\n", eve_individual_index+1);
    } else {
      // It is allowed for Coilpacks and Fuelpumps, because there are
      // two of each, i.e. redundancy. It *could* be useful in the event
      // of a short in-flight to latch the fault, to stop the associated
      // power channel from beating its brains out with a 15 Amp short.
      dbprintf("Change latch fault setting for index %d\n", eve_individual_index);
      latch_fault_settings ^= (1 << eve_individual_index);
      do_efi_write_latch_settings(latch_fault_settings);
    }
  } else {
    //dbprintf("eve_individual_button_pressed: tag %d\n", tag);
  }
}

static void do_red_arrow(int x1, int y1, int x2, int y2)
{
  int down = (y2 > y1) ? -1 : 1;

  accrue_Send_CMD(BEGIN(LINES));
  accrue_Send_CMD(COLOR_RGB(255, 0, 0));
  accrue_Send_CMD(LINE_WIDTH(32));

  accrue_Send_CMD(VERTEX2F(x1, y1)); 
  accrue_Send_CMD(VERTEX2F(x2, y2)); 

  accrue_Send_CMD(VERTEX2F(x2, y2)); 
  accrue_Send_CMD(VERTEX2F(x2-5, y2+down*20)); 

  accrue_Send_CMD(VERTEX2F(x2-5, y2+down*20)); 
  accrue_Send_CMD(VERTEX2F(x2+5, y2+down*20)); 

  accrue_Send_CMD(VERTEX2F(x2+5, y2+down*20)); 
  accrue_Send_CMD(VERTEX2F(x2, y2)); 
}

static void invisible_button(int tag, int x1, int y1, int x2, int y2)
{
  /*
   * Invisible touch target
   */
  accrue_Send_CMD(TAG_MASK(1));
  accrue_Send_CMD(TAG(tag));
  accrue_Send_CMD(COLOR_MASK(0, 0, 0, 0));
  accrue_Send_CMD(BEGIN(RECTS));
  accrue_Send_CMD(LINE_WIDTH(16));
  accrue_Send_CMD(VERTEX2F(x1, y1));
  accrue_Send_CMD(VERTEX2F(x2, y2));
  accrue_Send_CMD(COLOR_MASK(1, 1, 1, 1));
  accrue_Send_CMD(TAG_MASK(0));
  accrue_Send_CMD(display.normal_label_color);
}



int eve_even_live(uint8_t sections, uint32_t mask, char *namstr[], uint16_t total_width)
{
  // X, Y dimensions of the plot, in pixels
  int x_width = total_width / sections;
  int y_height = 112;
  int y_pos = display.height - 1;

  int x_pos;
  int i, k;
  uint8_t *p;
  uint8_t dc_offset_fault = 0;
  uint8_t both_fuelpumps_off = 0;

  plot_data_t *pd;

  struct timespec begin, release, end;

  double cumulative_total_msec = 0.0;
  double average_msec = 0.0;
  double loops = 0;
  char dbuf[16], pbuf[16];
  char *but2_text = NULL;
  uint8_t detail_tag4_enabled = 0;

  if (sections == 1 && ((mask & 0xff) == 0))
    but2_text = "Show start";

  if (sections == 1) {
    int detail_index;

    for (detail_index = 0; detail_index < NUM_CURRENTS; detail_index++) {
      if (mask == (1UL << detail_index)) {
        detail_tag4_enabled = 1;
        break;
      }
    }
  }

  while (!mo_abort)
    {
    if (sections > 1) {
      if ((i = check_touch_while_wait_fifo_empty(sections, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        //dbprintf("eve_even_live: Top: Average time %.2f msec\n", average_msec);
        return(i);
        }
      }
    clock_gettime(CLOCK_MONOTONIC, &begin);

    accrue_Send_CMD(CMD_DLSTART);                   // Start a new display list
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      // Determine the clear screen color
    accrue_Send_CMD(CLEAR(1, 1, 1));	            // Clear the screen and the current display list
    accrue_Send_CMD(VERTEXFORMAT(0));               // Setup VERTEX2F to take pixel coordinates
    accrue_Send_CMD(TAG_MASK(0));                   // Don't assign tags to objects

    // Put battery fault indicators up on top left, if appropriate
    if (left_battery_fault != ETX_NORMAL || right_battery_fault != ETX_NORMAL) {
      if ((msec_tick % 2000) > 500) {               // 1/2 second off, 1.5 seconds on blink
        if (left_battery_fault != ETX_NORMAL) {
            accrue_Send_CMD(BEGIN(RECTS));
            accrue_Send_CMD(LINE_WIDTH(8));                   // Line width in 1/16 pixels
            if (left_battery_fault == ETX_BLINK_5SEC)
              accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0x0));    // Amber
            else
              accrue_Send_CMD(COLOR_RGB(0xff, 0x0, 0x0));     // Red
            accrue_Send_CMD(VERTEX2F(0, 0));
            accrue_Send_CMD(VERTEX2F(7, 10));
        }
        if (right_battery_fault != ETX_NORMAL) {
          if (left_battery_fault == ETX_NORMAL) {
            accrue_Send_CMD(BEGIN(RECTS));
            accrue_Send_CMD(LINE_WIDTH(10));                       // Line width in 1/16 pixels
            }
          if (right_battery_fault == ETX_BLINK_5SEC)
            accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0x0));    // Amber
          else
            accrue_Send_CMD(COLOR_RGB(0xff, 0x0, 0x0));     // Red
          accrue_Send_CMD(VERTEX2F(8, 0));
          accrue_Send_CMD(VERTEX2F(15, 10));
        }
      }
    }

    if (sections == 4 && mask == ((((1<<(CYLINDERS+COILPACKS+FUELPUMPS))-1)) - ((1<<CYLINDERS)-1))) {
      // The coilpacks + fuelpumps display.
      if (fuelpump_on[0] == 0 && fuelpump_on[1] == 0)
        both_fuelpumps_off = 1;                     // Causes some changed actions below
      else
        both_fuelpumps_off = 0;
      }

    // Draw all displays
    for (k = 0; k < sections; k++) {
      dc_offset_fault = 0;

      // x_pos for each injector
      x_pos = (total_width / sections) * k;

      // Draw the Axes
      accrue_Send_CMD(BEGIN(LINES));
      accrue_Send_CMD(COLOR_RGB(26, 26, 192));              // change colour to blue
      accrue_Send_CMD(LINE_WIDTH(8));                       // Line width in 1/16 pixels
      accrue_Send_CMD(VERTEX2F(x_pos, (y_pos-y_height)));   // Y Axis
      accrue_Send_CMD(VERTEX2F(x_pos, y_pos));
      accrue_Send_CMD(VERTEX2F(x_pos, y_pos));              // X Axis
      accrue_Send_CMD(VERTEX2F((x_pos+x_width), y_pos));    //

      // Top labels
      if (k > 1 && both_fuelpumps_off == 1)
        accrue_Send_CMD(display.fault_label_color); 
      else
        accrue_Send_CMD(display.normal_label_color);
      Cmd_Text(x_pos + x_width/2, 10, 27, OPT_CENTER, namstr[k]);

      if (implied_ecu_failure && sections == 6) {
        if (implied_ecu_failure == 1 && k == 1) {
          do_red_arrow(230, 20, 230, 100);
          accrue_Send_CMD(display.fault_label_color); 
          Cmd_Text(x_pos + x_width/2, 60, 28, OPT_CENTER, "Primary ECU Failure");
        } else if (implied_ecu_failure == 2 && k == 4) {
          do_red_arrow(470, 100, 470, 20);
          accrue_Send_CMD(display.fault_label_color); 
          Cmd_Text(x_pos + x_width/2, 60, 28, OPT_CENTER, "Secondary ECU Failure");
        }
      }

      pd = efi_get_plot_data(k, mask, x_width - 8, x_width - 8);
      if (!pd) {
        // Something's wrong - no data
        put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(252,186,3));
        release_Send_CMD();
        UpdateFIFO();
        continue;
        }

      p = pd->plot_data;
      //dbprintf("k %d, p 0x%08x (%d)\n", k, p, p);
      if (!p) {
        // Something's wrong - no data
        put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(252,186,3));
        if (sections == 1) {
          eve_individual_detail(mask, pd, total_width, but2_text);
          if (detail_tag4_enabled) {
            invisible_button(4, 247, 0, 464, 78);
          }
        }
        release_Send_CMD();
        UpdateFIFO();
        continue;
        }

      if (pd->fault == FAULT_WIRING) {
        // There's a fault code. Indicate the fault.
        put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(255,0,0));
        if (sections == 1) {
          eve_individual_detail(mask, pd, total_width, but2_text);
          if (detail_tag4_enabled) {
            invisible_button(4, 247, 0, 464, 78);
          }
        }
        release_Send_CMD();
        UpdateFIFO();
        continue;
        }

      // Inefficient, but simple way to identify device faults
      int l;
      uint8_t *q;
      for (q = p, l = 0; l < pd->points; l++, q++) {
        if (*q == MO_FAULT) {
          pd->fault = FAULT_DEVICE;
#if 0
          FILE *f = fopen("fault_dump.dat", "w+");
          if (f) {
            fprintf(f, "Dump of fault data, %d points, fault %d\n", pd->points, pd->fault);
            for (q = p, l = 0; l < pd->points; l++, q++) {
              fprintf(f, "     %3d: 0x%02x (%d)\n", l, *q, *q);
            }
          fclose(f);
          }
#endif
          break;
        }
      }

#if 0
  static uint did_dump = 0;
  if (!did_dump && sections == 1) {
    did_dump = 1;
    FILE *f = fopen("data_dump.dat", "w+");
    if (f) {
      fprintf(f, "Dump of data, %d points, fault %d\n", pd->points, pd->fault);
      for (q = p, l = 0; l < pd->points; l++, q++) {
        fprintf(f, "     %3d: 0x%02x (%d)\n", l, *q, *q);
      }
    fclose(f);
    }
  }
#endif


      if (mask == ((1<<CYLINDERS)-1)) {
        // The 6 injector display. Stick the current and duration in
        if (pd->no_update) {
          // Leave blank
        } else {
          accrue_Send_CMD(display.normal_label_color);

          if (pd->duration) {
            snprintf(dbuf, sizeof(dbuf), "%5.1fms", (float)pd->duration / 1000);
            Cmd_Text(x_pos + x_width - 20, 103, 20, OPT_RIGHTX, dbuf);
            }

          if (pd->fault == FAULT_DEVICE)
            snprintf(pbuf, sizeof(pbuf), "FAULT");
          else
            snprintf(pbuf, sizeof(pbuf), "%5.1fA", (float)(pd->current_ma & PD_CURRENT_MASK) / 1000);
          Cmd_Text(x_pos + x_width - 26, 90, 20, OPT_RIGHTX, pbuf);

          if (pd->current_ma & PD_CURRENT_AVERAGE)
            dc_offset_fault = 1;
        }
      } else if (mask == ((((1<<(CYLINDERS+COILPACKS+FUELPUMPS))-1)) - ((1<<CYLINDERS)-1))) {
        // The coilpacks + fuelpumps display. Stick coilpack data in
        if (k < 2) {
          if (pd->no_update) {
            // Leave blank
          } else {
            accrue_Send_CMD(display.normal_label_color);
            if (pd->duration > 0) {
              snprintf(dbuf, sizeof(dbuf), "%5.1fms", (float)pd->duration / 1000);      // Dwell time
              Cmd_Text(x_pos + x_width - 40, 103, 20, OPT_RIGHTX, dbuf);
            }

            if (pd->fault == FAULT_DEVICE)
              snprintf(pbuf, sizeof(pbuf), "FAULT");
            else
              snprintf(pbuf, sizeof(pbuf), "%5.1fA", (float)(pd->current_ma & PD_CURRENT_MASK) / 1000);     // Peak current
            Cmd_Text(x_pos + x_width - 46, 90, 20, OPT_RIGHTX, pbuf);

            if (pd->current_ma & PD_CURRENT_AVERAGE)
              dc_offset_fault = 1;

            if (pd->derived_rpm) {
              snprintf(dbuf, sizeof(dbuf), "%4d rpm", pd->derived_rpm);
              Cmd_Text(x_pos + x_width - 110, 30, 20, 0, dbuf);
              }
          }
        } else {
          // Fuelpumps
          if (pd->no_update) {
            // Leave blank
          } else {
            if (k > 1 && both_fuelpumps_off == 1)
              accrue_Send_CMD(display.fault_label_color); 
            else
              accrue_Send_CMD(display.normal_label_color);

            if (pd->fault == FAULT_DEVICE)
              snprintf(pbuf, sizeof(pbuf), "FAULT");
            else
              snprintf(pbuf, sizeof(pbuf), "%5.1fA", (float)(pd->current_ma & PD_CURRENT_MASK) / 1000);     // Average current
            Cmd_Text(x_pos + x_width - 46, 90, 20, OPT_RIGHTX, pbuf);

            if (pd->duration > 0) {
              snprintf(dbuf, sizeof(dbuf), "%5.1fm", (float)pd->duration / 60);         // Run time
              Cmd_Text(x_pos + x_width - 45, 103, 20, OPT_RIGHTX, dbuf);
              }

          }
        }
      } else {
        // Individual plot
        if (pd->current_ma & PD_CURRENT_AVERAGE)
          dc_offset_fault = 1;
      }


      if (sections == 1) {
        // This is an individual display, there will usually be right hand data/detail to add.
        eve_individual_detail(mask, pd, total_width, but2_text);
          if (detail_tag4_enabled) {
            invisible_button(4, 247, 0, 464, 78);
          }
      }

      uint8_t doing_green;

      accrue_Send_CMD(BEGIN(LINES));
      if (dc_offset_fault > 0) {
        accrue_Send_CMD(display.caution_line_color);
        accrue_Send_CMD(display.caution_line_width);
      } else {
        if (k > 1 && both_fuelpumps_off == 1) {
          doing_green = 1;      // Fake it
          accrue_Send_CMD(display.fault_line_color);           // Red
          accrue_Send_CMD(display.fault_line_width);
        } else {
          doing_green = 1;
          accrue_Send_CMD(display.normal_line_color);
          accrue_Send_CMD(display.normal_line_width);                   // Line width in 1/16 pixels
        }
      }

#if 0
      // Temporary stuff to get fft data input
      if (sections == 1) {
        static int first = 1;
        if (first) {
          FILE *f;

          first = 0;
          f = fopen("data.dat", "w");
          if (f) {
            int l, pad;
            fprintf(f, "256\n85858\n");
            pad = (256 - pd->points)/2;
            for (l = 0; l < pad; l++)
              fprintf(f, "0 0\n");
            for (l = 0; l < pd->points; l++)
              fprintf(f, "%.1f 0\n", (double) pd->plot_data[l]);
            for (l = pad + pd->points; l < 256; l++)
              fprintf(f, "0 0\n");
            fclose(f);
          }
        }
      }
#endif

      int x1, y1, x2, y2;
      int coalescing;
      uint8_t p_val, pm1_val;
      //dbprintf("Plotting, k %d, pd->points %d\n", k, pd->points);
      p++;          // Skip first point, for coalescing algorithm
      for (i = 1, coalescing = 0; i < pd->points; i++, p++) {
        if (*p == MO_FAULT)
          p_val = 95;
        else
          p_val = *p;
        if (*(p-1) == MO_FAULT)
          pm1_val = 95;
        else
          pm1_val = *(p-1);

        if (coalescing) {
          if (pm1_val == p_val && i < pd->points-1)
            continue;
          coalescing = 0;
          //if (k == 2)
          //  dbprintf("Exit coalescing, i %d, val-1 %d, val %d\n", i, *(p-1), *p);
          // Finish the first line through to end point *(p-1)
          x2 = x_pos+1+(i-1);
          y2 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x2, y2));

          if (dc_offset_fault == 0) {
            if (*(p-1) == MO_FAULT) {                         // Fault
              if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
                }
              else if (doing_green) {
                accrue_Send_CMD(display.fault_line_color);           // Red
                accrue_Send_CMD(display.fault_line_width);
                doing_green = 0;
                }
            } else {
              if (k > 1 && both_fuelpumps_off) {
                accrue_Send_CMD(display.fault_line_color);           // Red
                accrue_Send_CMD(display.fault_line_width);
                doing_green = 1;            // Fake it
              } else if (!doing_green) {
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
              }
            }
          }

          // Send a line between these two dis-similar points
          x1 = x_pos+1+(i-1);
          y1 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x1, y1));

          x2 = x_pos+1+(i);
          y2 = y_pos-1-p_val-10;
          accrue_Send_CMD(VERTEX2F(x2, y2));
        } else if (pm1_val == p_val) {
          coalescing = 1;

          //if (k == 2)
          //  dbprintf("Enter Coalescing, i %d, val-1 %d, val %d\n", i, *(p-1), *p);
          // Send the first point for a coalesced line
          if (dc_offset_fault == 0) {
            if (*(p-1) == MO_FAULT) {                         // Fault
              if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
                }
              else if (doing_green) {
                accrue_Send_CMD(display.fault_line_color);               // Red
                accrue_Send_CMD(display.fault_line_width);
                doing_green = 0;
                }
            } else if (!doing_green) {
              accrue_Send_CMD(display.normal_line_color);
              accrue_Send_CMD(display.normal_line_width);
              doing_green = 1;
            }
          }

          x1 = x_pos+1+(i-1);
          y1 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x1, y1));
        } else {

          if (dc_offset_fault == 0) {
            if (*(p-1) == MO_FAULT) {                         // Fault
              if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
                }
              else if (doing_green) {
                accrue_Send_CMD(display.fault_line_color);               // Red
                accrue_Send_CMD(display.fault_line_width);
                doing_green = 0;
                }
            } else if (!doing_green) {
              accrue_Send_CMD(display.normal_line_color);
              accrue_Send_CMD(display.normal_line_width);
              doing_green = 1;
            }
          }

          // Send a line between the two points
          x1 = x_pos+1+(i-1);
          y1 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x1, y1));

          x2 = x_pos+1+(i);
          y2 = y_pos-1-p_val-10;
          accrue_Send_CMD(VERTEX2F(x2, y2));
          //if (k == 2)
          //  dbprintf("%1d,%1d -> %1d,%1d\n", x1, y1, x2, y2);
        }
      }

      release_Send_CMD();
      UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO

      if (sections > 1)
        if ((i = check_touch(sections, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
          //dbprintf("eve_even_live: bottom1: Average time %.2f msec\n", average_msec);
          return(i);
          }
      }

    //accrue_Send_CMD(CMD_LOADIDENTITY);            // XXX No idea how this works
    //accrue_Cmd_Translate(50, 50);
    //accrue_Send_CMD(CMD_SETMATRIX);

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_Send_CMD();
    UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO

    clock_gettime(CLOCK_MONOTONIC, &release);
    long seconds = release.tv_sec - begin.tv_sec;
    long nanoseconds = release.tv_nsec - begin.tv_nsec;
    double elapsed_msec = seconds*1e3 + nanoseconds*1e-6;

    if (sections == 1) {
      uint8_t tag;
      Wait4CoProFIFOEmpty();
      if ((tag = eve_debounce_tagread(&eve_individual_button_pressed, 0)) > 0 && tag != 3) {
        //dbprintf("eve_even_live: bottom2: eve_debounce_tagread returned %d\n", tag);
        return(tag);
        }
      }
    else if ((i = check_touch_while_wait_fifo_empty(sections, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
      dbprintf("eve_even_live: bottom3: Average time %.2f msec\n", average_msec);
      return(i);
      }

    check_screenshot();

    clock_gettime(CLOCK_MONOTONIC, &end);
    seconds = end.tv_sec - begin.tv_sec;
    nanoseconds = end.tv_nsec - begin.tv_nsec;
    double total_msec = seconds*1e3 + nanoseconds*1e-6;

    cumulative_total_msec += total_msec;
    loops += 1.0;
    average_msec = cumulative_total_msec / loops;

    //dbprintf("time: release %.2f, total %.2f msec\n", elapsed_msec, total_msec);            // Usually around 54 msec

    // Make up to 100 msec, for updates @ 10 times per second
    if (elapsed_msec < 100.0) {
      int delay_usec = 100000 - (int)(elapsed_msec*1000);
      //dbprintf("eve_even_live: delay for %d usec\n", delay_usec);

      if (sections > 1)
        while (delay_usec > 10000) {
        // Keep checking for input while waiting
          if ((i = check_touch(sections, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
            //dbprintf("eve_even_live: release: Average time %.2f msec\n", average_msec);
            return(i);
            }
          mo_usleep(8000);      // A bit less than 10000 coz more delay in check_touch()
          delay_usec -= 10000;
        }

      mo_usleep(delay_usec);
      }
      
    }
  return(0);
}

uint8_t eve_injectors()
{
  uint32_t cylmask = 0x3f;
  char *cylstr[] = {"1", "2", "3", "4", "5", "6"};

  return(eve_even_live(6, cylmask, cylstr, display.width));
}


uint8_t eve_coilpumps()
{
  uint32_t cfmask = 0x3c0;
  char *cfstr[] = {"Coilpack-L", "Coilpack-R", "Fuelpump-L", "Fuelpump-R"};
  
  return(eve_even_live(4, cfmask, cfstr, display.width));
}


uint8_t eve_injector_detail(uint8_t index)
{
  uint32_t injmask = (0x1 << index);
  char buf[32], *p = buf;

  snprintf(buf, sizeof(buf), "Cylinder %1d Injector", index+1);

  return(eve_even_live(1, injmask, &p, 288 + 8));
}

uint8_t eve_coilpack_detail(uint8_t index)
{
  uint32_t cpmask = (index) ? 0x80 : 0x40;
  char *cpstr[] = {"Left Coilpack (top plugs)", "Right Coilpack (bottom plugs)"};

  return(eve_even_live(1, cpmask, &cpstr[index], 224 + 8));
}


uint8_t eve_fuelpump_detail(uint8_t index)
{
  uint32_t fpmask = (index) ? 0x200 : 0x100;
  char *fpstr[] = {"Left Fuelpump", "Right Fuelpump"};

  return(eve_even_live(1, fpmask, &fpstr[index], 224 + 8));
}


uint8_t eve_fuelpump_laststart_plot(uint8_t index, struct pump_start_record *ps, int redraw)
{
  int x_width = 400;
  int y_height = 112;
  int y_pos = display.height - 1;
  int xscale = display.pumpstart_xscale;            // Magnify the x scale if > 1
                                                    // xscale = 1, x max is 1 second
                                                    // xscale = 2, x max is 0.5 seconds
                                                    // etc.
  int x_pos = 25;
  int i, x, y;

  uint16_t *p, peak, div;
  float fpeak;
  char buf[64];

  //dbprintf("laststart_plot: entered, tick %d data %d,%d,%d...\n", ps->fuelpump_start_tick, ps->data[0], ps->data[1], ps->data[3]);

  if (redraw) {
    // Clear the screen for 100 msec, so we can see the redraw happened
    accrue_Send_CMD(CMD_DLSTART);                   // Start a new display list
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      // Determine the clear screen color
    accrue_Send_CMD(CLEAR(1, 1, 1));	            // Clear the screen and the current display list
    accrue_Send_CMD(DISPLAY());                     // End the display list
    accrue_Send_CMD(CMD_SWAP);                      // Swap commands into RAM
    release_Send_CMD();
    UpdateFIFO();                                   // Trigger the CoProcessor to start processing the FIFO
    mo_usleep(200000);
  }

  accrue_Send_CMD(CMD_DLSTART);                   // Start a new display list
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      // Determine the clear screen color
  accrue_Send_CMD(CLEAR(1, 1, 1));	            // Clear the screen and the current display list
  accrue_Send_CMD(VERTEXFORMAT(0));               // Setup VERTEX2F to take pixel coordinates
  accrue_Send_CMD(TAG_MASK(0));                   // Don't assign tags to objects

  // Figure the scale, based on the peak
  for (i = 0, peak=0, p = &ps->data[0], div = 1; i < FPS_RECORDS; i++, p++) {
    if (*p > peak)
      peak = *p;
  while ((*p / div) > y_height)         // Tamp it down in case there's no inrush in data
    div++;
  }

  // Find the next round number for Y full scale. Make it even
  uint32_t amps_peak = (div * y_height * 2900 * 5) / 4095 / 1000;         // Amps
  if (++amps_peak & 0x1)
    amps_peak++;
  div = (amps_peak * 1000 * 4095) / 5 / y_height / 2900;

  // Adjust the scale if the final value is > 50% of scale
  if (ps->data[FPS_RECORDS-1] > 100) {            // Unless it is silly data
    while (ps->data[FPS_RECORDS-1] / div > y_height / 3)
      div++;
    // Gotta re-compute amps_peak
    amps_peak = (uint32_t)div * 2900 * y_height * 5 / 1000 / 4095;
    }

  fpeak = (float)(((uint32_t)peak * 2900 * 5) / 4095)/1000.0;
  //dbprintf("laststart_plot: peak %d (%.1f Amps) div %d\n", peak, fpeak, div);

  if (1 || peak > 0) {                                  // XXX Draw anyway
    // Draw the Axes
    accrue_Send_CMD(BEGIN(LINES));
    accrue_Send_CMD(display.axis_color);
    accrue_Send_CMD(display.normal_line_width);                   // Line width in 1/16 pixels

    // Axis lines
    for (x = 0; x <= x_width; x += x_width/4) {
      accrue_Send_CMD(VERTEX2F(x+x_pos, (x > x_width/2) ? (y_pos - y_height/2) : (y_pos-y_height)));
      accrue_Send_CMD(VERTEX2F(x+x_pos, y_pos));
      }
    for (y = y_pos; y > 0; y -= y_height/4) {
      accrue_Send_CMD(VERTEX2F(x_pos, y));
      accrue_Send_CMD(VERTEX2F((y < y_height/2) ? (x_pos+x_width/2) : (x_pos+x_width), y));
      }

    accrue_Send_CMD(display.pumpstart_line_color);
    accrue_Send_CMD(display.pumpstart_line_width); 

    int x1, y1, x2, y2;
    uint8_t p1_val, p2_val;
    uint8_t doing_normal = 1;

    for (i = 0, p = &ps->data[0]; i < (FPS_RECORDS/xscale)-1; i++, p += xscale) {
      p1_val = (uint8_t)((*p) / div);
      p2_val = (uint8_t)((*(p+xscale)) / div);
      if (*(p+xscale) >= 3300) {                                     // XXX Get a define ADC_FAULT_THRESHOLD
        doing_normal = 0;
        accrue_Send_CMD(display.fault_line_color);
      } else {
        if (!doing_normal) {
          doing_normal = 1;
          accrue_Send_CMD(display.pumpstart_line_color);
        }
      }
      x1 = x_pos+xscale*i;
      x2 = x_pos+xscale*(i+1);
      y1 = y_pos-p1_val;
      y2 = y_pos-p2_val;
      accrue_Send_CMD(VERTEX2F(x1, y1));
      accrue_Send_CMD(VERTEX2F(x2, y2));
    }


    x = x_pos+x_width/2+20;
    y = 20;
    accrue_Send_CMD(display.normal_label_color);                    // White
    snprintf(buf, sizeof(buf), "%s fuel pump startup", (index) ? "Right" : "Left");
    Cmd_Text(x, y, 27, 0, buf);
    snprintf(buf, sizeof(buf), "Start inrush current %.1f Amps", fpeak);
    Cmd_Text(x, y+25, 27, 0, buf);

    // X axis labels
    for (i = 0, x = 0; x <= x_width; i++, x += x_width/4) {
      if (x == 0)
        strncpy(buf, "0", sizeof(buf));
      else
        snprintf(buf, sizeof(buf), "%.*f%s", (xscale > 3) ? 4 : ((xscale > 1) ? 3 : 2), (float)(i)/4/(float)xscale, (x == x_width) ? " sec" : "");
      Cmd_Text(x+x_pos+4, y_pos-13, 20, 0, buf);
      }
    
    // Y axis labels
    snprintf(buf, sizeof(buf), "%1dA", amps_peak);
    Cmd_Text(x_pos-3, display.height-y_height, 20, OPT_RIGHTX|OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "%1dA", amps_peak/2);
    Cmd_Text(x_pos-3, display.height-y_height+(y_height/2), 20, OPT_RIGHTX|OPT_CENTERY, buf);
    //Cmd_Text(x_pos-3, y_pos-13, 20, OPT_RIGHTX, "0A");
    
  }

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_Send_CMD();
  UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO

  Wait4CoProFIFOEmpty();

  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Summary of everything, coilpacks, fuelpumps and injectors
////////////////////////////////////////////////////////////////////////////////

static void show_afr(int x, int y, int font)
{
  char dbuf[32];

  int rl_offset = (font == 20) ? 55 : 70;  // Burned in offset

  if (o2_sensor_power_is_on && !o2_sensor_not_ready && !o2_sensor_error) {
    snprintf(dbuf, sizeof(dbuf), "%s  %4.1f", "AFR", air_fuel_ratio);
    Cmd_Text(x, y, font, 0, dbuf);
    if (air_fuel_ratio < 14.5) {
      accrue_Send_CMD(COLOR_RGB(255, 0, 0));
      Cmd_Text(x + rl_offset, y, font, 0, "RICH");
      }
    else if (air_fuel_ratio > 14.9) {
      accrue_Send_CMD(COLOR_RGB(0x0, 0xff, 0x0));
      Cmd_Text(x + rl_offset, y, font, 0, "LEAN");
      }
    }
  else {
    accrue_Send_CMD(COLOR_RGB(128, 128, 128));
    Cmd_Text(x, y, font, 0, "AFR  N/A");
  }
}



int eve_summary()
{
  int total_width = display.width;
  int cylinders = 6;                                    // One day ...
  int mask = (cylinders == 6) ? 0x3ff : 0x3cf;

  // X, Y dimensions of the injector plot, in pixels
  int x1_width = total_width / cylinders;
  int y1_height = display.height/2;
  int y1_pos = y1_height - 1;

  // X, Y dimensions of the Coilpacks, Fuelpumps plot, in pixels
  int x2_width = total_width / 4;
  int y2_height = display.height/2;
  int y2_pos = 2*y2_height - 1;

  int row;

  int x_pos;
  int i, k;
  uint8_t *p;
  uint8_t dc_offset_fault = 0;
  uint8_t both_fuelpumps_off = 0;

  plot_data_t *pd;

  struct timespec begin, release, end;

  double cumulative_total_msec = 0.0;
  double average_msec = 0.0;
  double loops = 0;
  char dbuf[16], pbuf[16];
  uint8_t no_injector_updates[6];

  for (i = 0; i < DIMENSION(no_injector_updates); i++)
    no_injector_updates[i] = 0;

  while (!mo_abort)
    {
    if ((i = check_touch_while_wait_fifo_empty(6, SWIPE_GAP_X, swipe_debug, 4, SWIPE_GAP_Y)) > 0) {
      //dbprintf("eve_summary: Top: Average time %.2f msec\n", average_msec);
      if (i == DOUBLE_TOUCH) 
        eve_do_screenshot = 1;
      else {
        return(i);
        }
      }

    clock_gettime(CLOCK_MONOTONIC, &begin);

    accrue_Send_CMD(CMD_DLSTART);                   // Start a new display list
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      // Determine the clear screen color
    accrue_Send_CMD(CLEAR(1, 1, 1));	            // Clear the screen and the current display list
    accrue_Send_CMD(VERTEXFORMAT(0));               // Setup VERTEX2F to take pixel coordinates
    accrue_Send_CMD(TAG_MASK(0));                   // Don't assign tags to objects

    // Put battery fault indicators up on top left, if appropriate
    if (left_battery_fault != ETX_NORMAL || right_battery_fault != ETX_NORMAL) {
      if ((msec_tick % 2000) > 500) {               // 1/2 second off, 1.5 seconds on blink
        if (left_battery_fault != ETX_NORMAL) {
            accrue_Send_CMD(BEGIN(RECTS));
            accrue_Send_CMD(LINE_WIDTH(8));                   // Line width in 1/16 pixels
            if (left_battery_fault == ETX_BLINK_5SEC)
              accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0x0));    // Amber
            else
              accrue_Send_CMD(COLOR_RGB(0xff, 0x0, 0x0));     // Red
            accrue_Send_CMD(VERTEX2F(0, 0));
            accrue_Send_CMD(VERTEX2F(7, 10));
        }
        if (right_battery_fault != ETX_NORMAL) {
          if (left_battery_fault == ETX_NORMAL) {
            accrue_Send_CMD(BEGIN(RECTS));
            accrue_Send_CMD(LINE_WIDTH(10));                       // Line width in 1/16 pixels
            }
          if (right_battery_fault == ETX_BLINK_5SEC)
            accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0x0));    // Amber
          else
            accrue_Send_CMD(COLOR_RGB(0xff, 0x0, 0x0));     // Red
          accrue_Send_CMD(VERTEX2F(8, 0));
          accrue_Send_CMD(VERTEX2F(15, 10));
        }
      }
    }

    // The coilpacks + fuelpumps display half
    if (fuelpump_on[0] == 0 && fuelpump_on[1] == 0)
      both_fuelpumps_off = 1;                     // Causes some changed actions below
    else
      both_fuelpumps_off = 0;

    // First row is injectors
    // Second row is coilpacks and fuelpumps
    for (row = 0; row < 2; row++) {
      int sections = (row == 0) ? cylinders : 4;
      int x_width = (row == 0) ? x1_width : x2_width;
      int y_height = (row == 0) ? y1_height : y2_height;
      int y_pos = (row == 0) ? y1_pos : y2_pos;

      //dbprintf("eve_summary: row %d, sections %d x_width %d y_height %d y_pos %d\n", row, sections, x_width, y_height, y_pos);

      // Draw the top section
      for (k = 0; k < sections; k++) {
        dc_offset_fault = 0;

        // First thing - get the plot data. The first call will result in a delay, so we do
        // this before clearing the screen so that there is no "blinking numbers" while the
        // screen is cleared waiting for the data.
        pd = efi_get_plot_data(row*cylinders+k, mask, x1_width - 8, x2_width - 8);
        //dbprintf("eve_summary: Back from getting plot data, pd 0x%x\n", pd);

        // x_pos for each injector
        x_pos = (total_width / sections) * k;

        // Draw the Axes
        accrue_Send_CMD(BEGIN(LINES));
        accrue_Send_CMD(COLOR_RGB(26, 26, 192));              // change colour to blue
        accrue_Send_CMD(LINE_WIDTH(8));                       // Line width in 1/16 pixels
        accrue_Send_CMD(VERTEX2F(x_pos, (y_pos-y_height)));   // Y Axis
        accrue_Send_CMD(VERTEX2F(x_pos, y_pos));
        accrue_Send_CMD(VERTEX2F(x_pos, y_pos));              // X Axis
        accrue_Send_CMD(VERTEX2F((x_pos+x_width), y_pos));    //

        if (k == sections-1) {
          // RHS Y line, and top box line
          int x_rhs = total_width-1;

          accrue_Send_CMD(VERTEX2F(x_rhs, (y_pos-y_height)));   // Y Axis
          accrue_Send_CMD(VERTEX2F(x_rhs, y_pos));

          if (row == 0) {
            accrue_Send_CMD(VERTEX2F(0, 0));
            accrue_Send_CMD(VERTEX2F(total_width-1, 0));
            }
        }


  #if 1
        // Top labels
        if (row > 0 && k > 1 && both_fuelpumps_off == 1)
          accrue_Send_CMD(display.fault_label_color); 
        else
          accrue_Send_CMD(display.dim_label_color);
        const char *str[] = {"1", "2", "3", "4", "5", "6"};
        //const char *lr[] = {"L", "R"};
        if (row == 0) {
          if (implied_ecu_failure != 1 && k < cylinders/2)
            Cmd_Text(x_pos + 5, 7, 20, OPT_CENTER, str[k]);
          if (implied_ecu_failure != 2 && k >= cylinders/2)
            Cmd_Text(x_pos + 5, 7, 20, OPT_CENTER, str[k]);
        //} else {
        //  Cmd_Text(x_pos + 5, y1_height+7, 20, OPT_CENTER, lr[(k&0x1)]);
        }
  #endif
        if (implied_ecu_failure && row == 0) {
          if (implied_ecu_failure == 1 && k == 1) {
            do_red_arrow(230, 5, 230, 55);
            accrue_Send_CMD(display.fault_label_color); 
            Cmd_Text(x_pos + x_width/2, 10, 28, OPT_CENTER, "Primary ECU Failure");
          } else if (implied_ecu_failure == 2 && k == 4) {
            do_red_arrow(470, 55, 470, 20);
            accrue_Send_CMD(display.fault_label_color); 
            Cmd_Text(x_pos + x_width/2, 10, 28, OPT_CENTER, "Secondary ECU Failure");
          }
        }

        if (!pd) {
          // Something's wrong - no data
          put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(252,186,3));
          release_Send_CMD();
          UpdateFIFO();
          continue;
          }

        p = pd->plot_data;
        //dbprintf("k %d, p 0x%08x (%d)\n", k, p, p);
        if (!p) {
          // Something's wrong - no data
          put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(252,186,3));
          release_Send_CMD();
          UpdateFIFO();
          continue;
          }

        if (pd->fault == FAULT_WIRING) {
          // There's a fault code. Indicate the fault.
          put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(255,0,0));
          release_Send_CMD();
          UpdateFIFO();
          continue;
          }

        // Inefficient, but simple way to identify device faults
        int l;
        uint8_t *q;
        for (q = p, l = 0; l < pd->points; l++, q++) {
          if (*q == MO_FAULT) {
            pd->fault = FAULT_DEVICE;
            break;
          }
        }

        if (row == 0) {
          // The 6 injector display. Stick the current and duration in
          // We allow a few "no_updates" through and continue to draw/display the old
          // data. Without this, the printed data for cylinder's 1 and 2 could "blink"
          // with the odd no_update cycle, at fast display refresh intervals. After a few
          // consecutive "no_update" cycles, we do blank the numbers.
          if (pd->no_update && no_injector_updates[k] > 3) {
            // Blank the numbers out
          } else {
            if (pd->no_update)
              no_injector_updates[k]++;
            else
              no_injector_updates[k] = 0;           // Reset for next time

            accrue_Send_CMD(display.dim_label_color);

#if 1
            if (pd->duration) {
              snprintf(dbuf, sizeof(dbuf), "%5.1fms", (float)pd->duration / 1000);
              Cmd_Text(x_pos + x_width - 25, 43, 20, OPT_RIGHTX, dbuf);
              }

            if (pd->fault == FAULT_DEVICE)
              snprintf(pbuf, sizeof(pbuf), "FAULT");
            else
              snprintf(pbuf, sizeof(pbuf), "%5.1fA", (float)(pd->current_ma & PD_CURRENT_MASK) / 1000);
            Cmd_Text(x_pos + x_width - 30, 30, 20, OPT_RIGHTX, pbuf);
#endif
            if (pd->current_ma & PD_CURRENT_AVERAGE)
              dc_offset_fault = 1;
          }
        } else {
          // The coilpacks + fuelpumps display. Stick coilpack data in
          if (k < 2) {
            if (pd->no_update) {
              // Leave blank
            } else {
              accrue_Send_CMD(display.dim_label_color);

              if (pd->duration > 0) {
                snprintf(dbuf, sizeof(dbuf), "%5.1fms", (float)pd->duration / 1000);      // Dwell time
                Cmd_Text(x_pos + 30, 77, 20, OPT_RIGHTX, dbuf);
              }

              if (pd->fault == FAULT_DEVICE)
                snprintf(pbuf, sizeof(pbuf), "FAULT");
              else
                snprintf(pbuf, sizeof(pbuf), "%5.1fA", (float)(pd->current_ma & PD_CURRENT_MASK) / 1000);     // Peak current
              Cmd_Text(x_pos + 24, 90, 20, OPT_RIGHTX, pbuf);
              if (pd->current_ma & PD_CURRENT_AVERAGE)
                dc_offset_fault = 1;
#if 1
              if (pd->derived_rpm) {
                snprintf(pbuf, sizeof(pbuf), "%4d", pd->derived_rpm);
                Cmd_Text(x_pos + 3, 64, 20, 0, pbuf);
                }
#endif
            // For an active coilpack, try to guess if the Alternator is off, and warn the user
            if (pd->duration > 0) {
              if (saved_average_current[k+CYLINDERS] > 0 && saved_average_current[k+CYLINDERS] < display.coilpack_alternator_threshold[k]) {
                accrue_Send_CMD(display.fault_label_color); 
                Cmd_Text(x_pos + 40, 70, 21, 0, "ALT");
                }
              }
            }
          } else {
            // Fuelpumps
            if (pd->no_update) {
              // Leave blank
            } else {
              if (row > 0 && k > 1 && both_fuelpumps_off == 1)
                accrue_Send_CMD(display.fault_label_color); 
              else
                accrue_Send_CMD(display.dim_label_color);

              if (pd->fault == FAULT_DEVICE) {
                Cmd_Text(x_pos + x_width - 46, 95, 20, OPT_RIGHTX, "FAULT");
              } else if (fuelpump_on[k-2] == 1) {
                snprintf(pbuf, sizeof(pbuf), "%5.1fA", (float)(pd->current_ma & PD_CURRENT_MASK) / 1000);     // Average current
                Cmd_Text(x_pos + x_width - 46, 95, 20, OPT_RIGHTX, pbuf);
                }

              if (pd->duration > 0) {
                snprintf(dbuf, sizeof(dbuf), "%5.1fm", (float)pd->duration / 60);         // Run time
                Cmd_Text(x_pos + x_width - 45, 108, 20, OPT_RIGHTX, dbuf);
                }
            }

            // Sneak an O2 air/fuel ratio number in
            if (fuelpump_on[0] == 1 && fuelpump_on[1] == 1) {
              // Both pumps are on, squeeze it into the top of "left" fuel pump section
              if (k == sections-2)
                show_afr(x_pos+3, 64, 20);
              }
            else if (fuelpump_on[0] == 0) {
              // Right pump on, put AFR in the "left" position
              if (k == sections-2)
                show_afr(x_pos+10, 80, 21);
              }
            else if (fuelpump_on[1] == 0) {
              // Left pump on, put AFR in the "right" position
              if (k == sections-1)
                show_afr(x_pos+10, 80, 21);
              }
          }
        }

        uint8_t doing_green;

        accrue_Send_CMD(BEGIN(LINES));
        if (dc_offset_fault > 0) {
          accrue_Send_CMD(display.caution_line_color);
          accrue_Send_CMD(display.caution_line_width);
        } else {
          if (row > 0 && k > 1 && both_fuelpumps_off == 1) {
            doing_green = 1;      // Fake it
            accrue_Send_CMD(display.fault_line_color);           // Red
            accrue_Send_CMD(display.fault_line_width);
          } else {
            doing_green = 1;
            accrue_Send_CMD(display.normal_line_color);
            accrue_Send_CMD(display.normal_line_width);                   // Line width in 1/16 pixels
          }
        }

        int x1, y1, x2, y2;
        int coalescing;
        uint8_t p_val, pm1_val;
        //dbprintf("Plotting, row %d, k %d, pd->points %d\n", row, k, pd->points);
        p++;          // Skip first point, for coalescing algorithm
        for (i = 1, coalescing = 0; i < pd->points; i++, p++) {
          if (*p == MO_FAULT)
            p_val = 95/2;
          else
            p_val = *p/2;
          if (*(p-1) == MO_FAULT)
            pm1_val = 95/2;
          else
            pm1_val = *(p-1)/2;

          if (coalescing) {
            if (pm1_val == p_val && i < pd->points-1)
              continue;
            coalescing = 0;
            //if (k == 2)
            //  dbprintf("Exit coalescing, i %d, val-1 %d, val %d\n", i, *(p-1), *p);
            // Finish the first line through to end point *(p-1)
            x2 = x_pos+1+(i-1);
            y2 = y_pos-1-pm1_val-10;
            accrue_Send_CMD(VERTEX2F(x2, y2));

            if (dc_offset_fault == 0) {
              if (*(p-1) == MO_FAULT) {                         // Fault
                if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                  accrue_Send_CMD(display.normal_line_color);
                  accrue_Send_CMD(display.normal_line_width);
                  doing_green = 1;
                  }
                else if (doing_green) {
                  accrue_Send_CMD(display.fault_line_color);           // Red
                  accrue_Send_CMD(display.fault_line_width);
                  doing_green = 0;
                  }
              } else {
                if (row > 0 && k > 1 && both_fuelpumps_off) {
                  accrue_Send_CMD(display.fault_line_color);           // Red
                  accrue_Send_CMD(display.fault_line_width);
                  doing_green = 1;            // Fake it
                } else if (!doing_green) {
                  accrue_Send_CMD(display.normal_line_color);
                  accrue_Send_CMD(display.normal_line_width);
                  doing_green = 1;
                }
              }
            }

            // Send a line between these two dis-similar points
            x1 = x_pos+1+(i-1);
            y1 = y_pos-1-pm1_val-10;
            accrue_Send_CMD(VERTEX2F(x1, y1));

            x2 = x_pos+1+(i);
            y2 = y_pos-1-p_val-10;
            accrue_Send_CMD(VERTEX2F(x2, y2));
          } else if (pm1_val == p_val) {
            coalescing = 1;

            //if (k == 2)
            //  dbprintf("Enter Coalescing, i %d, val-1 %d, val %d\n", i, *(p-1), *p);
            // Send the first point for a coalesced line
            if (dc_offset_fault == 0) {
              if (*(p-1) == MO_FAULT) {                         // Fault
                if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                  accrue_Send_CMD(display.normal_line_color);
                  accrue_Send_CMD(display.normal_line_width);
                  doing_green = 1;
                  }
                else if (doing_green) {
                  accrue_Send_CMD(display.fault_line_color);               // Red
                  accrue_Send_CMD(display.fault_line_width);
                  doing_green = 0;
                  }
              } else if (!doing_green) {
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
              }
            }

            x1 = x_pos+1+(i-1);
            y1 = y_pos-1-pm1_val-10;
            accrue_Send_CMD(VERTEX2F(x1, y1));
          } else {

            if (dc_offset_fault == 0) {
              if (*(p-1) == MO_FAULT) {                         // Fault
                if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                  accrue_Send_CMD(display.normal_line_color);
                  accrue_Send_CMD(display.normal_line_width);
                  doing_green = 1;
                  }
                else if (doing_green) {
                  accrue_Send_CMD(display.fault_line_color);               // Red
                  accrue_Send_CMD(display.fault_line_width);
                  doing_green = 0;
                  }
              } else if (!doing_green) {
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
              }
            }

            // Send a line between the two points
            x1 = x_pos+1+(i-1);
            y1 = y_pos-1-pm1_val-10;
            accrue_Send_CMD(VERTEX2F(x1, y1));

            x2 = x_pos+1+(i);
            y2 = y_pos-1-p_val-10;
            accrue_Send_CMD(VERTEX2F(x2, y2));
            //if (k == 2)
            //  dbprintf("%1d,%1d -> %1d,%1d\n", x1, y1, x2, y2);
          }
        }

        release_Send_CMD();
        UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO

        if ((i = check_touch(sections, SWIPE_GAP_X, swipe_debug, 4, SWIPE_GAP_Y)) > 0) {
          //dbprintf("eve_summary: bottom1: Average time %.2f msec\n", average_msec);
          if (i == DOUBLE_TOUCH)
            eve_do_screenshot = 1;
          else {
            return(i);
            }
          }
        }
      }
      //dbprintf("eve_summary: Bottom, done all rows\n");

      accrue_Send_CMD(DISPLAY());                     //End the display list
      accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
      release_Send_CMD();
      UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO

      clock_gettime(CLOCK_MONOTONIC, &release);
      long seconds = release.tv_sec - begin.tv_sec;
      long nanoseconds = release.tv_nsec - begin.tv_nsec;
      double elapsed_msec = seconds*1e3 + nanoseconds*1e-6;

      if ((i = check_touch_while_wait_fifo_empty(6, SWIPE_GAP_X, swipe_debug, 4, SWIPE_GAP_Y)) > 0) {            // 6 sections
        dbprintf("eve_summary: bottom3: Average time %.2f msec\n", average_msec);
        if (i == DOUBLE_TOUCH) {
          eve_do_screenshot = 1;
        } else {
          return(i);
          }
        }

      check_screenshot();

      clock_gettime(CLOCK_MONOTONIC, &end);
      seconds = end.tv_sec - begin.tv_sec;
      nanoseconds = end.tv_nsec - begin.tv_nsec;
      double total_msec = seconds*1e3 + nanoseconds*1e-6;

      cumulative_total_msec += total_msec;
      loops += 1.0;
      average_msec = cumulative_total_msec / loops;

      //dbprintf("time: release %.2f, total %.2f msec\n", elapsed_msec, total_msec);            // Usually around 54 msec

      // Make up to 100 msec, for updates @ 20 times per second
      if (elapsed_msec < 100.0) {
        int delay_usec = 100000 - (int)(elapsed_msec*1000);
        //dbprintf("eve_summary: delay for %d usec\n", delay_usec);

        while (delay_usec > 10000) {
        // Keep checking for input while waiting
          if ((i = check_touch(6, SWIPE_GAP_X, swipe_debug, 4, SWIPE_GAP_Y)) > 0) {              // 6 was sections
            //dbprintf("eve_summary: release: Average time %.2f msec\n", average_msec);
            if (i == DOUBLE_TOUCH) 
              eve_do_screenshot = 1;
            else
              return(i);
            }
          mo_usleep(8000);      // A bit less than 10000 coz more delay in check_touch()
          delay_usec -= 10000;
        }

        mo_usleep(delay_usec);
        }
      }

    return(0);
  }

////////////////////////////////////////////////////////////////////////////////
// Summary of injectors + coilpacks, used in checklists
////////////////////////////////////////////////////////////////////////////////

struct ck_signal_pts {
  int16_t x, y, x_points, h;
  } checklist_signal_points[] = {
    {80+120,      40, 80/2, 40},                        // Cylinder 1
    {80+120+1*40, 40, 80/2, 40},
    {80+120+2*40, 40, 80/2, 40},
    {80+120+3*40, 40, 80/2, 40},
    {80+120+4*40, 40, 80/2, 40},
    {80+120+5*40, 40, 80/2, 40},
    {40+0,        40, 120/2, 40},                        // Left Coilpack
    {40+60,       40, 120/2, 40}                         // Right Coilpack
  };

static int eve_checklist_signals()
{
  int cylinders = 6;                                    // One day ...
  int mask = (cylinders == 6) ? 0xff : 0xcf;

  int x_pos, y_pos, x_width, y_height;
  int i, k;
  uint8_t *p;
  uint8_t dc_offset_fault = 0;

  plot_data_t *pd;

  //accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      // Determine the clear screen color
  //accrue_Send_CMD(CLEAR(1, 1, 1));	            // Clear the screen and the current display list
  //accrue_Send_CMD(VERTEXFORMAT(0));               // Setup VERTEX2F to take pixel coordinates
  //accrue_Send_CMD(TAG_MASK(0));                   // Don't assign tags to objects

    //dbprintf("eve_checklist_signals: row %d, sections %d x_width %d y_height %d y_pos %d\n", row, sections, x_width, y_height, y_pos);

    // Draw the top section
    for (k = 0; k < cylinders+2; k++) {
      dc_offset_fault = 0;

      x_pos = checklist_signal_points[k].x;
      y_pos = checklist_signal_points[k].y;
      x_width = checklist_signal_points[k].x_points;
      y_height = checklist_signal_points[k].h;

      pd = efi_get_plot_data(k, mask, checklist_signal_points[0].x_points-4, checklist_signal_points[cylinders].x_points-4);
      //dbprintf("eve_checklist_signals: Back from getting plot data, pd 0x%x\n", pd);

      if (!pd) {
        // Something's wrong - no data
        put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(252,186,3));
        release_Send_CMD();
        UpdateFIFO();
        continue;
        }

      p = pd->plot_data;
      //dbprintf("k %d, p 0x%08x (%d)\n", k, p, p);
      if (!p) {
        // Something's wrong - no data
        put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(252,186,3));
        release_Send_CMD();
        UpdateFIFO();
        continue;
        }

      if (pd->fault == FAULT_WIRING) {
        // There's a fault code. Indicate the fault.
        put_fault_cross(x_pos + x_width/2, y_pos - y_height/2 + 40/2 - 10, x_width-30, y_height-40, COLOR_RGB(255,0,0));
        release_Send_CMD();
        UpdateFIFO();
        continue;
        }

      // Inefficient, but simple way to identify device faults
      int l;
      uint8_t *q;
      for (q = p, l = 0; l < pd->points; l++, q++) {
        if (*q == MO_FAULT) {
          pd->fault = FAULT_DEVICE;
          break;
        }
      }

      uint8_t doing_green = 0;

      accrue_Send_CMD(BEGIN(LINES));
      if (dc_offset_fault > 0) {
        accrue_Send_CMD(display.caution_line_color);
        accrue_Send_CMD(display.caution_line_width);
      } else {
        doing_green = 1;
        accrue_Send_CMD(display.normal_line_color);
        accrue_Send_CMD(display.normal_line_width);                   // Line width in 1/16 pixels
      }

      int x1, y1, x2, y2;
      int coalescing;
      uint8_t p_val, pm1_val;
      //dbprintf("Plotting, row %d, k %d, pd->points %d\n", row, k, pd->points);
      p++;          // Skip first point, for coalescing algorithm
      for (i = 1, coalescing = 0; i < pd->points; i++, p++) {
        if (*p == MO_FAULT)
          p_val = 95/4;
        else
          p_val = *p/4;
        if (*(p-1) == MO_FAULT)
          pm1_val = 95/4;
        else
          pm1_val = *(p-1)/4;

        if (coalescing) {
          if (pm1_val == p_val && i < pd->points-1)
            continue;
          coalescing = 0;
          //if (k == 2)
          //  dbprintf("Exit coalescing, i %d, val-1 %d, val %d\n", i, *(p-1), *p);
          // Finish the first line through to end point *(p-1)
          x2 = x_pos+1+(i-1);
          y2 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x2, y2));

          if (dc_offset_fault == 0) {
            if (*(p-1) == MO_FAULT) {                         // Fault
              if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
                }
              else if (doing_green) {
                accrue_Send_CMD(display.fault_line_color);           // Red
                accrue_Send_CMD(display.fault_line_width);
                doing_green = 0;
                }
            } else {
              accrue_Send_CMD(display.normal_line_color);
              accrue_Send_CMD(display.normal_line_width);
              doing_green = 1;
            }
          }

          // Send a line between these two dis-similar points
          x1 = x_pos+1+(i-1);
          y1 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x1, y1));

          x2 = x_pos+1+(i);
          y2 = y_pos-1-p_val-10;
          accrue_Send_CMD(VERTEX2F(x2, y2));
        } else if (pm1_val == p_val) {
          coalescing = 1;

          //if (k == 2)
          //  dbprintf("Enter Coalescing, i %d, val-1 %d, val %d\n", i, *(p-1), *p);
          // Send the first point for a coalesced line
          if (dc_offset_fault == 0) {
            if (*(p-1) == MO_FAULT) {                         // Fault
              if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
                }
              else if (doing_green) {
                accrue_Send_CMD(display.fault_line_color);               // Red
                accrue_Send_CMD(display.fault_line_width);
                doing_green = 0;
                }
            } else if (!doing_green) {
              accrue_Send_CMD(display.normal_line_color);
              accrue_Send_CMD(display.normal_line_width);
              doing_green = 1;
            }
          }

          x1 = x_pos+1+(i-1);
          y1 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x1, y1));
        } else {

          if (dc_offset_fault == 0) {
            if (*(p-1) == MO_FAULT) {                         // Fault
              if (*p <= 105 && !doing_green) {                // 105 to allow a bit of overrun
                accrue_Send_CMD(display.normal_line_color);
                accrue_Send_CMD(display.normal_line_width);
                doing_green = 1;
                }
              else if (doing_green) {
                accrue_Send_CMD(display.fault_line_color);               // Red
                accrue_Send_CMD(display.fault_line_width);
                doing_green = 0;
                }
            } else if (!doing_green) {
              accrue_Send_CMD(display.normal_line_color);
              accrue_Send_CMD(display.normal_line_width);
              doing_green = 1;
            }
          }

          // Send a line between the two points
          x1 = x_pos+1+(i-1);
          y1 = y_pos-1-pm1_val-10;
          accrue_Send_CMD(VERTEX2F(x1, y1));

          x2 = x_pos+1+(i);
          y2 = y_pos-1-p_val-10;
          accrue_Send_CMD(VERTEX2F(x2, y2));
          //if (k == 2)
          //  dbprintf("%1d,%1d -> %1d,%1d\n", x1, y1, x2, y2);
        }
      }
    }

  return(0);
  }

////////////////////////////////////////////////////////////////////////////////
// Flaps controller page
////////////////////////////////////////////////////////////////////////////////

extern uint8_t flaps_mode_continuous;
extern uint8_t flaps_mode_airspeed;
extern struct flap_settings flaps;
static uint8_t configuring_flaps = 0;

#define YBAR_MIN            1
#define FLAP_HILITE_PASSES  10

#define FLAPS_BUT_WIDTH     ((uint16_t)100)
#define FLAPS_BUT_HEIGHT    ((uint16_t)34)
#define FLAP_BUT_Y          50
#define FLAP_BUT_X          120
#define NOFLAP_BUT_HEIGHT   ((uint16_t)100)

#define VOLTAGE_THRESHOLD   1250

// Where the ticks are for each position
static uint8_t f_reflex_tick = 0;
static uint8_t f_zero_tick = 10;
static uint8_t f_half_tick = 65;
static uint8_t f_full_tick = 120;

static uint32_t compute_flap_bar(struct bpower_data *f, int ymax)
{
  uint32_t ybar;

  // Straight linear interpolation between any two flap position markers
  if (f->current_flap_position <= flaps.reflex_position)
    ybar = YBAR_MIN;
  else if (f->current_flap_position <= flaps.zero_position)
    ybar = f_reflex_tick +
      (uint32_t)(f_zero_tick - f_reflex_tick) * (uint32_t)(f->current_flap_position - flaps.reflex_position) / (flaps.zero_position - flaps.reflex_position);
  else if (f->current_flap_position <= flaps.half_position)
    ybar = f_zero_tick +
      (uint32_t)(f_half_tick - f_zero_tick) * (uint32_t)(f->current_flap_position - flaps.zero_position) / (flaps.half_position - flaps.zero_position);
  else
    ybar = f_half_tick +
      (uint32_t)(f_full_tick - f_half_tick) * (uint32_t)(f->current_flap_position - flaps.half_position) / (flaps.full_position - flaps.half_position);

  if (ybar == 0)
    ybar = YBAR_MIN;            // Minimum
  if (ybar > f_full_tick)
    ybar = f_full_tick;         // Clip to maximum

  return(ybar);
}


uint8_t eve_flaps_display(int timeout)
  {
    //struct timeval ts_both_on, ts_now;
    int i, y, action;
    int xpos;
    int ypos;
    char buf[64];
    //long timeout_msec = (long)timeout * 1000;
    float bat_voltage, primary_voltage, secondary_voltage;
    static uint8_t passes_after_up_button = 0;
    static uint8_t passes_after_down_button = 0;
    static uint32_t accrued_flap_motor_current = 0;
    const char *lbl[] = {
                          "GPS", "COM", "XPDR", "INTCM", "OTHER", "FLAPS"
                        };

    extern struct bpower_data *flaps_data;
    extern int flaps_data_valid;

    struct bpower_data *f = flaps_data;

    //ts_both_on.tv_sec = 0;
    //ts_both_on.tv_usec = 0;
    accrued_flap_motor_current = 0;

    while (!mo_abort) {
      flaps_request_data();         // Keep requesting data

      for (i = 0; i < 50; i++) {
        if ((action = check_touch(4, SWIPE_GAP_X, swipe_debug, 4, SWIPE_GAP_Y)) > 0) {
          //dbprintf("eve_flaps_display: TOP: returning %d\n", action);
          if (action == 4)
            passes_after_up_button = FLAP_HILITE_PASSES;
          if (action == 8)
            passes_after_down_button = FLAP_HILITE_PASSES;
          return(action);
          }
        mo_usleep(1000);            // Update rate around 50 msec, i.e. 20 times per second
        }

      accrue_Send_CMD(CMD_DLSTART);
      accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
      accrue_Send_CMD(CLEAR(1, 1, 1));
      accrue_Send_CMD(VERTEXFORMAT(0));

      if (flaps_data_valid) {
        const char *fmt = "%.1f Volts";

        //////////////////////////////////////////////////////////////////////////////// 
        // Voltage buttons
        //////////////////////////////////////////////////////////////////////////////// 
        bat_voltage = FLAPS_VOLTAGE(f->battery_voltage & 0xfff);
        secondary_voltage = FLAPS_VOLTAGE(f->secondary_voltage & 0xfff);
        primary_voltage = FLAPS_VOLTAGE(f->primary_voltage & 0xfff);

        snprintf(buf, sizeof(buf), fmt, primary_voltage);
        if (f->primary_voltage > VOLTAGE_THRESHOLD)
          Cmd_FGcolor(EGREEN);
        else
          Cmd_FGcolor(ERED);                    // Red-ish
        y = 4;
        Cmd_Text(5, y+10, 26, 0, "PRI");
        Cmd_Button(40, y, FLAPS_BUT_WIDTH, FLAPS_BUT_HEIGHT, 23, 0, buf);

        snprintf(buf, sizeof(buf), fmt, secondary_voltage);
        if (f->secondary_voltage > VOLTAGE_THRESHOLD)
          Cmd_FGcolor(EGREEN);
        else
          Cmd_FGcolor(ERED);                    // Red-ish
        y = 4+8+FLAPS_BUT_HEIGHT;
        Cmd_Text(5, y+10, 26, 0, "SEC");
        Cmd_Button(40, y, FLAPS_BUT_WIDTH, FLAPS_BUT_HEIGHT, 23, 0, buf);

        snprintf(buf, sizeof(buf), fmt, bat_voltage);
        if (f->battery_voltage > VOLTAGE_THRESHOLD) {
            if (f->secondary_voltage > VOLTAGE_THRESHOLD)
              Cmd_FGcolor(EAMBER);                  // Battery is in standby
            else
              Cmd_FGcolor(EGREEN);
          }
        else
          Cmd_FGcolor(ERED);                    // Red-ish
        y = 4+2*8+2*FLAPS_BUT_HEIGHT;
        Cmd_Text(5, y+10, 26, 0, "BAT");
        Cmd_Button(40, y, FLAPS_BUT_WIDTH, FLAPS_BUT_HEIGHT, 23, 0, buf);

        //////////////////////////////////////////////////////////////////////////////// 
        // Middle section
        //////////////////////////////////////////////////////////////////////////////// 
        xpos = 170;
        ypos = 4;

        for (i = 0; i < 6; i++) {
          Cmd_Text(xpos, 4+i*20, 26, 0, lbl[i]);
          }

        xpos += 50;

        accrue_Send_CMD(LINE_WIDTH(8*1));
        accrue_Send_CMD(BEGIN(LINES));
        for (i = 0; i < 3; i++) {
          accrue_Send_CMD(VERTEX2F(xpos+i*20, ypos));
          accrue_Send_CMD(VERTEX2F(xpos+i*20, 128-ypos));
          }
        for (i = 0; i < 7; i++) {
          accrue_Send_CMD(VERTEX2F(xpos, ypos+i*20));
          accrue_Send_CMD(VERTEX2F(xpos+2*20, ypos+i*20));
          }

        ////////////////////////////////////////////////////////////////////////////////
        // Fill in the blobs
        ////////////////////////////////////////////////////////////////////////////////
        accrue_Send_CMD(COLOR_RGB(0, 128, 0));
        accrue_Send_CMD(LINE_WIDTH(16*1));
        accrue_Send_CMD(BEGIN(RECTS));

        // GPS and COM are different - we have an indication of both supplies
        if (f->flags & BF_GPS_PRIMARY)
          accrue_Send_CMD(COLOR_RGB(0, 128, 0));
        else
          accrue_Send_CMD(COLOR_RGB(128, 0, 0));
        accrue_Send_CMD(VERTEX2F(xpos+2, ypos+2));
        accrue_Send_CMD(VERTEX2F(xpos+18, ypos+18));

        if (f->flags & BF_GPS_SECONDARY)
          accrue_Send_CMD(COLOR_RGB(0, 128, 0));
        else
          accrue_Send_CMD(COLOR_RGB(128, 0, 0));

        accrue_Send_CMD(VERTEX2F(xpos+2+20, ypos+2));
        accrue_Send_CMD(VERTEX2F(xpos+18+20, ypos+18));

        if (f->flags & BF_COM_PRIMARY)
          accrue_Send_CMD(COLOR_RGB(0, 128, 0));
        else
          accrue_Send_CMD(COLOR_RGB(128, 0, 0));
        accrue_Send_CMD(VERTEX2F(xpos+2, 20+ypos+2));
        accrue_Send_CMD(VERTEX2F(xpos+18, 20+ypos+18));

        if (f->flags & BF_COM_SECONDARY)
          accrue_Send_CMD(COLOR_RGB(0, 128, 0));
        else
          accrue_Send_CMD(COLOR_RGB(128, 0, 0));
        accrue_Send_CMD(VERTEX2F(20+xpos+2, 20+ypos+2));
        accrue_Send_CMD(VERTEX2F(20+xpos+18, 20+ypos+18));

        // For the rest, we just have an indication of which supply is being used
        accrue_Send_CMD(COLOR_RGB(0, 128, 0));
        if (f->flags & BF_XPNDR_STATUS) {
          accrue_Send_CMD(VERTEX2F(xpos+2, 40+ypos+2));
          accrue_Send_CMD(VERTEX2F(xpos+18, 40+ypos+18));
        } else {
          accrue_Send_CMD(VERTEX2F(20+xpos+2, 40+ypos+2));
          accrue_Send_CMD(VERTEX2F(20+xpos+18, 40+ypos+18));
        }

        if (f->flags & BF_INTCM_STATUS) {
          accrue_Send_CMD(VERTEX2F(xpos+2, 60+ypos+2));
          accrue_Send_CMD(VERTEX2F(xpos+18, 60+ypos+18));
        } else {
          accrue_Send_CMD(VERTEX2F(20+xpos+2, 60+ypos+2));
          accrue_Send_CMD(VERTEX2F(20+xpos+18, 60+ypos+18));
        }

        if (f->flags & BF_OTHER_STATUS) {
          accrue_Send_CMD(VERTEX2F(xpos+2, 80+ypos+2));
          accrue_Send_CMD(VERTEX2F(xpos+18, 80+ypos+18));
        } else {
          accrue_Send_CMD(VERTEX2F(20+xpos+2, 80+ypos+2));
          accrue_Send_CMD(VERTEX2F(20+xpos+18, 80+ypos+18));
        }

        if ((f->flags & BF_FLAPS_SOURCE) == 0) {
          accrue_Send_CMD(VERTEX2F(xpos+2, 100+ypos+2));
          accrue_Send_CMD(VERTEX2F(xpos+18, 100+ypos+18));
        } else {
          accrue_Send_CMD(VERTEX2F(20+xpos+2, 100+ypos+2));
          accrue_Send_CMD(VERTEX2F(20+xpos+18, 100+ypos+18));
        }

        //////////////////////////////////////////////////////////////////////////////// 
        // Flap position bargraph
        //////////////////////////////////////////////////////////////////////////////// 
        xpos += 70;
        ypos = 4;
        uint8_t xtick = 10;
        uint8_t fbar_width = 50;

        // Fill in the bar
        accrue_Send_CMD(COLOR_RGB(0, 128, 0));  
        accrue_Send_CMD(LINE_WIDTH(16*1));
        accrue_Send_CMD(BEGIN(RECTS));

        uint32_t ybar = compute_flap_bar(f, 120);

        accrue_Send_CMD(VERTEX2F(xpos+2, ypos+2));
        accrue_Send_CMD(VERTEX2F(xpos+fbar_width-2, ypos+ybar));

        accrue_Send_CMD(COLOR_RGB(255, 255, 255));  
        accrue_Send_CMD(LINE_WIDTH(8*1));
        accrue_Send_CMD(BEGIN(LINES));
        for (i = 0; i < 2; i++) {                           // Vertical bars
          accrue_Send_CMD(VERTEX2F(xpos+i*fbar_width, ypos));
          accrue_Send_CMD(VERTEX2F(xpos+i*fbar_width, 128-ypos));
          }
        for (i = 0; i < 2; i++) {                           // Top and bottom bars
          accrue_Send_CMD(VERTEX2F(xpos-xtick, ypos+i*120));
          accrue_Send_CMD(VERTEX2F(xpos+fbar_width, ypos+i*120));
          }

        accrue_Send_CMD(VERTEX2F(xpos-xtick, ypos+f_zero_tick));          // Other bars
        accrue_Send_CMD(VERTEX2F(xpos+fbar_width, ypos+f_zero_tick));
        accrue_Send_CMD(VERTEX2F(xpos-xtick, ypos+f_half_tick));
        accrue_Send_CMD(VERTEX2F(xpos+fbar_width, ypos+f_half_tick));

        // Fill in the airspeed
        if (f->efis_airspeed > 0) {
          if (flaps_mode_airspeed > 0)
            accrue_Send_CMD(COLOR_RGB(255, 0, 0));
          else
            accrue_Send_CMD(display.dim_label_color);
          snprintf(buf, sizeof(buf), "%3d", f->efis_airspeed);
          Cmd_Text(xpos+fbar_width/2, ypos+f_half_tick/2, 26, OPT_CENTER, buf);
          if (flaps_mode_airspeed > 0)
            accrue_Send_CMD(display.normal_label_color);       // Put it back to normal
          }

        // If the flaps are moving, fill in the current
        if (f->flap_motor_current > 100) {                          // Non-zero
          // Running average over 16 samples
          accrued_flap_motor_current = accrued_flap_motor_current - accrued_flap_motor_current / 16 + f->flap_motor_current;

          double current = FLAPS_CURRENT(accrued_flap_motor_current/16);

          snprintf(buf, sizeof(buf), "%.1fA", current);
          Cmd_Text(xpos+fbar_width/2, ypos+f_half_tick+20, 26, OPT_CENTER, buf);
          }
        else
          accrued_flap_motor_current = 0;

        if (configuring_flaps) {                // Doing Flaps calibration
          snprintf(buf, sizeof(buf), "%4d", f->current_flap_position);
          Cmd_Text(xpos+fbar_width/2, ypos+f_half_tick+45, 26, OPT_CENTER, buf);
          }


        //////////////////////////////////////////////////////////////////////////////// 
        // Flap UP / DOWN buttons
        //////////////////////////////////////////////////////////////////////////////// 
 
        xpos = 350;
        ypos = 4;

        if (flaps_mode_continuous) {
          Cmd_FGcolor(EGRAY);           // Buttons not available
          Cmd_Button(xpos, ypos, FLAP_BUT_X, FLAP_BUT_Y, 24, 0, "UP");
          Cmd_Button(xpos, 128-50-4, FLAP_BUT_X, FLAP_BUT_Y, 24, 0, "DOWN");
        } else {
          if (passes_after_up_button > 0) {
            Cmd_FGcolor(EORANGE);
            passes_after_up_button--;
          } else
            Cmd_FGcolor(EAMBER);
          Cmd_Button(xpos, ypos, FLAP_BUT_X, FLAP_BUT_Y, 24, 0, "UP");

          if (passes_after_down_button > 0) {
            Cmd_FGcolor(EORANGE);
            passes_after_down_button--;
          } else
            Cmd_FGcolor(EAMBER);
          Cmd_Button(xpos, 128-50-4, FLAP_BUT_X, FLAP_BUT_Y, 24, 0, "DOWN");
          }

      } else {
        snprintf(buf, sizeof(buf), "Flaps module\nstatus not available");
        Cmd_FGcolor(ERED);                    // Red-ish
        Cmd_Button(10, (display.height-NOFLAP_BUT_HEIGHT)/2, 460, NOFLAP_BUT_HEIGHT, 24, 0, buf);
      }

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();

    if ((i = check_touch_while_wait_fifo_empty(4, SWIPE_GAP_X, 0, 4, SWIPE_GAP_Y)) > 0) {
      //dbprintf("eve_flaps_display: BOT: returning %d\n", i);
      if (i == 4)
        passes_after_up_button = FLAP_HILITE_PASSES;
      if (i == 8)
        passes_after_down_button = FLAP_HILITE_PASSES;
      return(i);
      }

    check_screenshot();

    }
  return(0);
}



uint8_t eve_battery_display(int timeout)
  {
    struct timeval ts_both_on, ts_now;
    int i, action;
    char buf[64];
    long timeout_msec = (long)timeout * 1000;
    float bat_voltage, right_voltage, bat_internal_voltage;

    extern bat_response_t bat_status;
    extern int bat_status_valid;

    bat_response_t *br = &bat_status;

    ts_both_on.tv_sec = 0;
    ts_both_on.tv_usec = 0;

    while (!mo_abort) {

      for (i = 0; i < 50; i++) {
        if ((action = check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
          //dbprintf("eve_battery_display: returning %d\n", action);
          return(action);
          }
        mo_usleep(100);
        }

      accrue_Send_CMD(CMD_DLSTART);
      accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
      accrue_Send_CMD(CLEAR(1, 1, 1));
      accrue_Send_CMD(VERTEXFORMAT(0));

  #define BAT_BUT_WIDTH   ((uint16_t)170)
  #define BAT_BUT_HEIGHT   ((uint16_t)100)

      if (bat_status_valid) {
        const char *fmt = "TCW IBBS\nBattery\n%.1f Volts";
        const char *tcwofl = "TCW IBBS\nBattery\nOFFLINE";
        const char *rbatofl = "Right system\nOFFLINE";
        // TCW IBBS Battery
        bat_voltage = BAT_VOLTAGE(br->battery_voltage_adc_status & 0xfff);
        right_voltage = BAT_VOLTAGE(br->right_voltage_adc & 0xfff);
        bat_internal_voltage = BAT_VOLTAGE(br->battery_vadc1 & 0xfff);

        if (BAT_RELAY_ON_RIGHT(br)) {
          // Relay is on right battery
          if (bat_internal_voltage > 5.0)
            snprintf(buf, sizeof(buf), fmt, bat_internal_voltage);
          else
            snprintf(buf, sizeof(buf), tcwofl);
          if (BAT_BATTERY_UP(br))
            Cmd_FGcolor(EAMBER);                  // Battery is in standby
          else
            Cmd_FGcolor(ERED);                    // Red-ish
          } else {
          // Relay is on battery
            snprintf(buf, sizeof(buf), fmt, bat_voltage);
            if (bat_voltage >= 11.0)
              Cmd_FGcolor(EGREEN);                // A green you can see white text on
            else
              Cmd_FGcolor(ERED);                  // We're getting into trouble
          }
        Cmd_Button(50, (display.height-BAT_BUT_HEIGHT)/2, BAT_BUT_WIDTH, BAT_BUT_HEIGHT, 24, 0, buf);

        // Right system
        if (right_voltage > 5.0)
          snprintf(buf, sizeof(buf), "Right system\n%.1f Volts", right_voltage);
        else
          snprintf(buf, sizeof(buf), rbatofl);
        if (BAT_RELAY_ON_RIGHT(br)) {
          if (BAT_RIGHT_UP(br))
            Cmd_FGcolor(EGREEN);
          else
            Cmd_FGcolor(ERED);
        } else {
          // Should be just a transition state
          if (BAT_RIGHT_UP(br))
            Cmd_FGcolor(EAMBER);
          else
            Cmd_FGcolor(ERED);
        }
        Cmd_Button(480-50-BAT_BUT_WIDTH, (display.height-BAT_BUT_HEIGHT)/2, BAT_BUT_WIDTH, BAT_BUT_HEIGHT, 24, 0, buf);

        } else {
          snprintf(buf, sizeof(buf), "Battery backup\nstatus not available");
        Cmd_FGcolor(ERED);                    // Red-ish
        Cmd_Button(10, (display.height-BAT_BUT_HEIGHT)/2, 460, BAT_BUT_HEIGHT, 24, 0, buf);
      }
    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();

    if ((i = check_touch_while_wait_fifo_empty(20, SWIPE_GAP_X, 0, 0, SWIPE_GAP_Y)) > 0) {
      return(i);
      }

    // if BOTH the battery and right systems have been on for five continuous
    // seconds, we return a different code, to automagically switch to the DOORS
    // display - if we were in the "initial" 
    if (timeout > 0 && ts_both_on.tv_sec) {
      gettimeofday(&ts_now, NULL);
      long secs = ts_now.tv_sec - ts_both_on.tv_sec;
      long usecs = ts_now.tv_usec - ts_both_on.tv_usec;

      long msecs = secs*1000 + usecs/1000;

      if (msecs >= timeout_msec)
        return(BOTH_ON_FOR_TIMEOUT);
      }

    if (BAT_BATTERY_UP(br) && BAT_RIGHT_UP(br)) {
      if (ts_both_on.tv_sec == 0)
        gettimeofday(&ts_both_on, NULL);            // Remember when this happened
      } else {
        ts_both_on.tv_sec = 0;
        }

    check_screenshot();

    }

  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Battery fault display
////////////////////////////////////////////////////////////////////////////////

uint8_t eve_etx1200_display()
{
  int i, action;
  char buf[64];
  char *etx_state[] = {"Normal", "2 sec blink", "5 sec blink", "Solid"};
  char *etx_line4[] = {"", "Temp > 75C", "Charge problem", "BMS Problem"};

  while (!mo_abort) {
    for (i = 0; i < 50; i++) {
      if ((action = check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        return(action);
        }
      mo_usleep(100);
      }

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));
    accrue_Send_CMD(VERTEXFORMAT(0));

#define ETX_BUT_WIDTH   ((uint16_t)170)
#define ETX_BUT_HEIGHT   ((uint16_t)100)

    snprintf(buf, sizeof(buf), "Left/Primary\nETX1200 Battery\n%s\n%s",
            etx_state[left_battery_fault], etx_line4[left_battery_fault]);
    if (left_battery_fault == ETX_NO_BLINKING)
      Cmd_FGcolor(EGREEN);
    else if (left_battery_fault == ETX_BLINK_2SEC || left_battery_fault == ETX_SOLID)
      Cmd_FGcolor(ERED);
    else if (left_battery_fault == ETX_BLINK_5SEC)
      Cmd_FGcolor(EAMBER);
    else
      Cmd_FGcolor(EGREEN);
    Cmd_Button(50, (display.height-ETX_BUT_HEIGHT)/2, ETX_BUT_WIDTH, ETX_BUT_HEIGHT, 23, 0, buf);

    // Right system
    snprintf(buf, sizeof(buf), "Right/Secondary\nETX1200 Battery\n%s\n%s",
            etx_state[right_battery_fault], etx_line4[right_battery_fault]);
    if (right_battery_fault == ETX_NO_BLINKING)
      Cmd_FGcolor(EGREEN);
    else if (right_battery_fault == ETX_BLINK_2SEC || right_battery_fault == ETX_SOLID)
      Cmd_FGcolor(ERED);
    else if (right_battery_fault == ETX_BLINK_5SEC)
      Cmd_FGcolor(EAMBER);
    else
      Cmd_FGcolor(EGREEN);
    Cmd_Button(480-50-ETX_BUT_WIDTH, (display.height-ETX_BUT_HEIGHT)/2, ETX_BUT_WIDTH, ETX_BUT_HEIGHT, 23, 0, buf);

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();

    if ((i = check_touch_while_wait_fifo_empty(20, SWIPE_GAP_X, 0, 0, SWIPE_GAP_Y)) > 0) {
      return(i);
      }

    check_screenshot();
    }

  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Diagnostic entry points
////////////////////////////////////////////////////////////////////////////////

int const num_diag_buttons = 8;           // Number of diagnostic buttons
int const num_diag_columns = 4;           // Number of diagnostic buttons
int const gap_diag = 20;                  // Gap between buttons

int const diag_button_width = (DWIDTH - (num_diag_columns-1)*gap_diag) / num_diag_columns;
int const diag_button_height = 50;
int const diag_xinc = diag_button_width + gap_diag;

struct button_array diag_buttons[] = {
    {            1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EBLUE, "Checklists"},
    {1*diag_xinc+1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EBLUE, "Statistics"},
    {2*diag_xinc+1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EBLUE, "Backlight"},
    {3*diag_xinc+1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EBLUE, "About"},
    {            1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EBLUE, "Doors"},
    {1*diag_xinc+1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EBLUE, "Configure"},
    {2*diag_xinc+1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, ERED, "REBOOT"},
    {3*diag_xinc+1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGREEN, "BACK"}
  };

static uint8_t eve_diagnostic_display(uint8_t pressed)
{
  int i;
  struct button_array *b;

  accrue_Send_CMD(CMD_DLSTART);
  accrue_Send_CMD(VERTEXFORMAT(0));
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  accrue_Send_CMD(CLEAR(1, 1, 1));
  accrue_Send_CMD(display.normal_label_color);      // Change color to white for text

  Cmd_FGcolor(EBLUE);
  for (i = 0, b = diag_buttons; i < DIMENSION(diag_buttons); i++, b++) {
    if (b->text == NULL)
      continue;
    if (pressed & (0x1 << i))
      Cmd_FGcolor(EBLUE_PRESSED);
    else {
      Cmd_FGcolor(b->color);
      }
    accrue_Send_CMD(TAG(i+1));
    Cmd_Button(b->x, b->y, b->w, b->h, b->font, 0, b->text);
    release_wr32();
    UpdateFIFO();
    }

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();

  return(0);
}

struct button_array config_buttons[] = {
    {            1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EBLUE, "Flaps"},
    {1*diag_xinc+1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGRAY, NULL},
    {2*diag_xinc+1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGRAY, NULL},
    {3*diag_xinc+1,    (64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGRAY, NULL},
    {            1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGRAY, NULL},
    {1*diag_xinc+1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGRAY, NULL},
    {2*diag_xinc+1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGRAY, NULL},
    {3*diag_xinc+1, 64+(64-diag_button_height)/2, diag_button_width, diag_button_height, 28, EGREEN, "BACK"}
  };


static uint8_t eve_configure_display(uint8_t pressed)
{
  int i;
  struct button_array *b;

  accrue_Send_CMD(CMD_DLSTART);
  accrue_Send_CMD(VERTEXFORMAT(0));
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  accrue_Send_CMD(CLEAR(1, 1, 1));
  accrue_Send_CMD(display.normal_label_color);      // Change color to white for text

  Cmd_FGcolor(EBLUE);
  for (i = 0, b = config_buttons; i < DIMENSION(config_buttons); i++, b++) {
    if (b->text == NULL)
      continue;
    if (pressed & (0x1 << i))
      Cmd_FGcolor(EBLUE_PRESSED);
    else {
      Cmd_FGcolor(b->color);
      }
    accrue_Send_CMD(TAG(i+1));
    Cmd_Button(b->x, b->y, b->w, b->h, b->font, 0, b->text);
    release_wr32();
    UpdateFIFO();
    }

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();

  return(0);
}

int const num_cflap_buttons = 6;           // Number of flap buttons
int const num_cflap_columns = 3;           // Number of columns
int const gap_cflap = 20;                  // Gap between buttons

int const cflap_button_width = (DWIDTH - (num_cflap_columns-1)*gap_cflap) / num_cflap_columns;
int const cflap_button_height = 55;
int const cflap_xinc = cflap_button_width + gap_cflap;

struct button_array 
  flap_config_buttons[] = {
    {             1,    (64-cflap_button_height)/2, cflap_button_width, cflap_button_height, 28, EGRAY, "Reflex"},
    {1*cflap_xinc+1,    (64-cflap_button_height)/2, cflap_button_width, cflap_button_height, 28, EGRAY, "Zero"},
    {2*cflap_xinc+1,    (64-cflap_button_height)/2, cflap_button_width, cflap_button_height, 28, EGRAY, "Half"},
    {             1, 64+(64-cflap_button_height)/2, cflap_button_width, cflap_button_height, 28, EGRAY, "Full"},
    {1*cflap_xinc+1, 64+(64-cflap_button_height)/2, cflap_button_width, cflap_button_height, 28, EGRAY, "Reverse"},
    {2*cflap_xinc+1, 64+(64-cflap_button_height)/2, cflap_button_width, cflap_button_height, 28, EGRAY, "SET"}
  };


static uint8_t set = 0;
extern struct bpower_data *flaps_data;
extern uint8_t flaps_sensor_reverse;

static uint8_t eve_configure_flaps_display(uint8_t pressed)
{
  int i;
  struct button_array *b;
  char buf[16];

  accrue_Send_CMD(CMD_DLSTART);
  accrue_Send_CMD(VERTEXFORMAT(0));
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  accrue_Send_CMD(CLEAR(1, 1, 1));
  accrue_Send_CMD(display.normal_label_color);      // Change color to white for text

  Cmd_FGcolor(EBLUE);
  for (i = 0, b = flap_config_buttons; i < DIMENSION(flap_config_buttons); i++, b++) {
    if (b->text == NULL)
      continue;
    if (pressed & (0x1 << i))
      Cmd_FGcolor(EGREEN_PRESSED);
    else {
      if (i == 4) {
        // Reverse switch
        if (flaps_sensor_reverse)
          Cmd_FGcolor(ERED);
        else
          Cmd_FGcolor(EGREEN);
      } else {
        if (set & (0x1 << i))
          Cmd_FGcolor(EGREEN);
        else
          Cmd_FGcolor(b->color);
        }
      }
    accrue_Send_CMD(TAG(i+1));
    Cmd_Button(b->x, b->y, b->w, b->h, b->font, 0, b->text);
    release_wr32();
    UpdateFIFO();
    }

  snprintf(buf, sizeof(buf), "ADC=%4d", flaps_data->current_flap_position);
  Cmd_Text(240, 115, 26, OPT_CENTER, buf);

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();

  return(0);
}


uint8_t eve_configure_flaps(uint8_t ret_code)
{

  uint8_t tag;
  struct flap_settings new;

  set = 0;
  configuring_flaps = 1;
  while (!mo_abort) {
    flaps_request_data();
    eve_configure_flaps_display(0);
    if ((tag = eve_debounce_tagread(NULL, 0)) > 0) {
      dbprintf("eve_configure_flaps: tag %d\n", tag);
      eve_configure_flaps_display(1<<(tag-1));
      mo_usleep(250000);
      switch (tag) {
        case 1:
          new.reflex_position = flaps_data->current_flap_position;
          set |= 0x1;
          break;

        case 2:
          new.zero_position = flaps_data->current_flap_position;
          set |= 0x2;
          break;

        case 3:
          new.half_position = flaps_data->current_flap_position;
          set |= 0x4;
          break;

        case 4:
          new.full_position = flaps_data->current_flap_position;
          set |= 0x8;
          break;

        case 5:
          flaps_sensor_reverse = (flaps_sensor_reverse) ? 0 : 1;
          break;

        case 6:
          if (set & 0x1)
            flaps.reflex_position = new.reflex_position;
          if (set & 0x2)
            flaps.zero_position = new.zero_position;
          if (set & 0x4)
            flaps.half_position = new.half_position;
          if (set & 0x8)
            flaps.full_position = new.full_position;
          flaps_put_flap_settings();
          set = 0;
          configuring_flaps = 0;
          return(tag);
          break;

        default:
          configuring_flaps = 0;
          return(tag);
          break;
       }
    } else {
      mo_usleep(5000);
    }
  }
  configuring_flaps = 0;
  return(0);
}

static uint8_t eve_about_display()
{
  struct config_data *c = &efi_board_config;
  const char *labels[] = {"EFI Board S/N:", "EFI Board Firmware:", "System Software:"};
  char buf[64];

  int i;
  int x_pos1 = 50;
  int x_pos2 = (display.width*3)/4-x_pos1;
  int y_base = 45, y_offset = 20;
  int font = 23;

  accrue_Send_CMD(CMD_DLSTART);
  accrue_Send_CMD(VERTEXFORMAT(0));
  accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
  accrue_Send_CMD(CLEAR(1, 1, 1));
  accrue_Send_CMD(display.normal_label_color);      //Change color to white for text

  Cmd_Text(x_pos1, 15, font, 0, "Configuration information:");

  for (i = 0; i < DIMENSION(labels); i++)
    Cmd_Text(x_pos1, y_base+y_offset*i, font, 0, labels[i]);
  for (i = 0; i < DIMENSION(labels); i++) {
    switch (i) {
      case 0:
        snprintf(buf, sizeof(buf), "%1d", c->board_serial_number);
        break;

      case 1:
        snprintf(buf, sizeof(buf), "%1d.%1d", c->board_revision_major, c->board_revision_minor);
        break;

      case 2:
        strncpy(buf, mo_software_version, sizeof(buf));
        break;

      default:
        buf[0] = 0;
        break;
      }
    Cmd_Text(x_pos2, y_base+y_offset*i, font, 0, buf);
    }

  accrue_Send_CMD(DISPLAY());                     //End the display list
  accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
  release_wr32();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();

  return(0);
}

uint8_t eve_diagnostic(uint8_t ret_code)
{
  uint8_t tag;

  while (!mo_abort) {
    eve_diagnostic_display(0);
    //if ((tag = check_touch_while_wait_fifo_empty(num_diag_buttons, SWIPE_GAP_X, 0, 0, SWIPE_GAP_Y)) > 0) {
    if ((tag = eve_debounce_tagread(NULL, 1)) > 0) {
      //dbprintf("eve_diagnostic: tag %d\n", tag);
      eve_diagnostic_display(1<<(tag-1));
      mo_usleep(250000);
      switch (tag) {
        case 1:                                 // Go into checklists
          return(tag);
          break;

        case 2:
          return(tag);
          break;

        case 3:
          eve_set_backlight();
          break;

        case 4:
          eve_about_display();
          while (!mo_abort && ((tag = check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) == 0)) {
            check_screenshot();
            mo_usleep(5000);
            }
          return(255);
          break;

        case 5:
          do
            {
            eve_doors_page(get_door_state());
            check_screenshot();
            } while (!mo_abort && (check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y) == 0 && !mo_usleep(50000)));
          return(255);
          break;

        case 6:
          return(tag);
          break;

        case 7:                                 // Restart
          return(tag);
          break;

        case 8:                                 // Back
          return(tag);
          break;

        default:
          return(tag);                          // Swipe left or right
          break;
        }
      }
    }
  return(0);
}


uint8_t eve_configure(uint8_t ret_code)
{
  uint8_t tag;

  while (!mo_abort) {
    eve_configure_display(0);
    if ((tag = eve_debounce_tagread(NULL, 1)) > 0) {
      dbprintf("eve_configure: tag %d\n", tag);
      eve_configure_display(1<<(tag-1));
      mo_usleep(250000);

      switch (tag) {
        case 1:                                 // Go into Flaps configuration
          return(tag);
          break;

        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
          return(tag);
          break;

        case 8:
          return(ret_code);
          break;

        default:
          return(tag);                          // Swipe left or right
          break;
        }
      }
    }
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Air Conditioning status
////////////////////////////////////////////////////////////////////////////////

uint8_t eve_aircon()
{
  aircon_data_t *a = NULL;

  int i, action;
  uint16_t font = 18;
  uint16_t but_width = 180;
  uint16_t but_space = 20;
  uint16_t v = display.height/6;       // Vertical spacing, for 5 rows of text
  uint16_t x_left = but_width + 2 * but_space;
  char *ac_mode[] = {"OFF", "MANUAL", "AUTO", "VENT"};
  uint32_t ac_colour[] = {EDARK_GRAY, EORANGE, EGREEN, EBLUE};
  char buf[64];


  a = get_ac_data();                    // Gets updated in place

  while (!mo_abort) {
    for (i = 0; i < 50; i++) {
      if ((action = check_touch(10, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        //printf("eve_aircon: action %d\n", action);
        if (action == 1) {
#if 0
          if (a->mode == AM_VENT)
            a->mode = AM_OFF;
          else
            a->mode++;
#endif
          break;
          }
        else if (action > 2)            // Allow swipes
          return(action);
        }
      mo_usleep(100);
      }

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    snprintf(buf, sizeof(buf), "A/C mode\n%s", ac_mode[a->mode]);
    Cmd_FGcolor(ac_colour[a->mode]);
    accrue_Send_CMD(TAG(1));
    Cmd_Button(but_space, 15, but_width, display.height-30, 30, 0, buf);

    accrue_Send_CMD(COLOR_RGB(255, 255, 255));
    snprintf(buf, sizeof(buf), "Evap high side temp: %.1f C", a->temp_evaporator);
    Cmd_Text(x_left, v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "Evap high pressure : %.1f psi", a->pressure_evap_highside);
    Cmd_Text(x_left, 2*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "Rear vent air temp : %.1f C", a->temp_rear_vent_air);
    Cmd_Text(x_left, 3*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "Cabin temp, left   : %.1f C", a->temp_inside_left);
    Cmd_Text(x_left, 4*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "Cabin temp, right  : %.1f C", a->temp_inside_right);
    Cmd_Text(x_left, 5*v, font, OPT_CENTERY, buf);
    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    check_screenshot();
    }
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Statistics
////////////////////////////////////////////////////////////////////////////////

uint8_t eve_stat_comms(efi_stats_t *b)
{
  int i, action;
  uint16_t font = 18;
  uint16_t v = display.height/(7+1);       // Vertical spacing, for 7 rows of text
  uint16_t x_left = 1;
  uint16_t x_indent = 30;
  const int16_t but_width = display.width/4-1;
  const int16_t but_height = display.height/2;
  const int16_t but_x = display.width*3/4;
  const int16_t but_y = display.height/2-but_height/2;
  char buf[64];

  while (!mo_abort) {
    for (i = 0; i < 10; i++) {
      if ((action = check_touch(10, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        //dbprintf("eve_stat_comms: action %d\n", action);
        return(action);
        }
      mo_usleep(1000);
      }

    efi_get_stats_update();

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    Cmd_FGcolor(EGREEN);
    accrue_Send_CMD(TAG(10));
    Cmd_Button(but_x, but_y, but_width, but_height, font, 0, "RESET");

    accrue_Send_CMD(display.normal_label_color);
    Cmd_Text(x_left, v, font, OPT_CENTERY, "EFI Communication Stats:");

    snprintf(buf, sizeof(buf), "rx_length errors : %10d", b->rx_length_errors);
    Cmd_Text(x_left+x_indent, 2*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "preamble errors  : %10d", b->header_crc_errors);
    Cmd_Text(x_left+x_indent, 3*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "sequence errors  : %10d", b->sequence_errors);
    Cmd_Text(x_left+x_indent, 4*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "oversampled      : %10d", b->oversampled);
    Cmd_Text(x_left+x_indent, 5*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "single reversals : %10d", b->tracking_single_reversals);
    Cmd_Text(x_left+x_indent, 6*v, font, OPT_CENTERY, buf);
    snprintf(buf, sizeof(buf), "double reversals : %10d", b->tracking_double_reversals);
    Cmd_Text(x_left+x_indent, 7*v, font, OPT_CENTERY, buf);

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    check_screenshot();
  }

  return(0);
}


uint8_t eve_stat_op_response(response_counts_t *resp, int num, const char *title)
{
  extern int log_snapshot_time;
  extern uint8_t log_state;

  int i, action;
  uint16_t font = 18;
  uint16_t v = display.height/(7+1);       // Vertical spacing, for 7 rows of text
  uint16_t x_left = 1;
  const int16_t but_width = display.width/4-1;
  const int16_t but_height = 50;
  const int16_t but_x = display.width*3/4;
  const int16_t but_y = 0;
  response_counts_t *r;
  char buf[64];

  while (!mo_abort) {
    for (i = 0; i < 10; i++) {
      if ((action = check_touch(4, SWIPE_GAP_X, swipe_debug, 2, SWIPE_GAP_Y)) > 0) {
        //dbprintf("op_response: action %d\n", action);
        return(action);
        }
      mo_usleep(1000);
      }

    if (log_state == 0)
      snprintf(buf, sizeof(buf), "Log %d secs", log_snapshot_time);
    else if (log_state == 1)
      snprintf(buf, sizeof(buf), "%s", "Keep logging");
    else if (log_state == 2)
      snprintf(buf, sizeof(buf), "%s", "Revert");
    else
      snprintf(buf, sizeof(buf), "%s", "?");

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    Cmd_FGcolor(EGREEN);
    Cmd_Button(but_x, but_y+77, but_width, but_height, font, 0, "RESET");

    Cmd_FGcolor(EBLUE);
    Cmd_Button(but_x, but_y+1, but_width, but_height, font, 0, buf);

    accrue_Send_CMD(display.normal_title_color);
    snprintf(buf, sizeof(buf), "%-18s count  min   max   avg", title);
    Cmd_Text(x_left, v, font, OPT_CENTERY, buf);

    if (num > 6)
      num = 6;
    accrue_Send_CMD(display.normal_label_color);

    for (i = 0, r = resp; i < num; i++, r++) {
      if (r->num == 0)
        snprintf(buf, sizeof(buf), "%14s: %8d", r->name, r->num);
      else
        snprintf(buf, sizeof(buf), "%14s: %8d %5.3f %5.3f %5.3f", r->name, r->num, r->min, r->max, ((r->num)?(r->tot/r->num):0.0));
      Cmd_Text(x_left, (i+2)*v, font, OPT_CENTERY, buf);
    }

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    check_screenshot();
    }

  return(0);
}



uint8_t eve_stat_injectors(efi_stats_t *b)
{
  int i, j, k, action;
  uint16_t font = 18;
  uint16_t v = display.height/(6+1);       // Vertical spacing, for 6 rows of text
  uint16_t x_left = 1;
  const uint16_t x_indent1 = 10;
  const uint16_t x_varwidth = 330;
  const uint16_t x_indent2 = display.width - x_varwidth;
  const uint16_t x_inc = x_varwidth / CYLINDERS;

  struct stat_items {
    const char *text;
    const uint32_t *data;
  } items[] = {
    {"Pulses          :", &b->pulses[0]},
    {"Single glitches :", &b->single_glitches[0]},
    {"Double glitches :", &b->double_glitches[0]},
    {"Overrun errors  :", &b->overrun[0]},
    {"Faults          :", &b->faults[0]}
    };

  struct stat_items *si = &items[0];

  while (!mo_abort) {
    for (i = 0; i < 50; i++) {
      if ((action = check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        return(action);
        }
      mo_usleep(100);
      }

    efi_get_stats_update();

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    accrue_Send_CMD(COLOR_RGB(0, 0, 255));
    for (j = 0; j < CYLINDERS; j++)
      Cmd_Number(x_left+x_indent2+(j+1)*x_inc-1, v, 20, OPT_RIGHTX|OPT_CENTERY, j+1);
    Cmd_Text(x_indent2+x_varwidth/2+25, v/2-5, 20, OPT_CENTER, "CYLINDER");

    accrue_Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text(x_left, v, font, OPT_CENTERY, "EFI Injector Stats:");

    for (k = 0, si = &items[0]; k < DIMENSION(items); k++, si++) {
      Cmd_Text(x_left+x_indent1, (k+2)*v, font, OPT_CENTERY, si->text);
      for (j = 0; j < CYLINDERS; j++)
        Cmd_Number(x_indent2+(j+1)*x_inc-1, (k+2)*v, 20, OPT_RIGHTX|OPT_CENTERY, si->data[j]);
      }

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    check_screenshot();
  }
  return(0);
}

#define SCF_NUM     (COILPACKS+FUELPUMPS)
#define SCF_OFFSET  CYLINDERS

uint8_t eve_stat_coilpumps(efi_stats_t *b)
{
  int i, j, k, action;
  uint16_t font = 18;
  uint16_t v = display.height/(6+1);       // Vertical spacing, for 6 rows of text
  uint16_t x_left = 1;
  const uint16_t x_indent1 = 10;
  const uint16_t x_varwidth = 330;
  const uint16_t x_indent2 = display.width - x_varwidth;
  const uint16_t x_inc = x_varwidth / SCF_NUM;

  struct stat_items {
    const char *text;
    const uint32_t *data;
  } items[] = {
    {"Pulses/Samples  :", &b->pulses[SCF_OFFSET]},
    {"Single glitches :", &b->single_glitches[SCF_OFFSET]},
    {"Double glitches :", &b->double_glitches[SCF_OFFSET]},
    {"Overrun errors  :", &b->overrun[SCF_OFFSET]},
    {"Faults          :", &b->faults[SCF_OFFSET]}
    };

  struct stat_items *si = &items[0];

  while (!mo_abort) {
    for (i = 0; i < 50; i++) {
      if ((action = check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        return(action);
        }
      mo_usleep(2000);
      }

    efi_get_stats_update();

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    accrue_Send_CMD(COLOR_RGB(0, 0, 255));
    for (j = 0; j < SCF_NUM; j++)
      Cmd_Text(x_left+x_indent2+(j+1)*x_inc-1, v, 20, OPT_RIGHTX|OPT_CENTERY, ((j==0 || j==2) ? "L" : "R"));
    Cmd_Text(x_indent2+x_varwidth/4+35, v/2-5, 20, OPT_CENTER, "COILPACKS");
    Cmd_Text(x_indent2+3*x_varwidth/4+35, v/2-5, 20, OPT_CENTER, "FUELPUMPS");

    accrue_Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text(x_left, v, font, OPT_CENTERY, "EFI Coil/Pump Stats:");

    for (k = 0, si = &items[0]; k < DIMENSION(items); k++, si++) {
      Cmd_Text(x_left+x_indent1, (k+2)*v, font, OPT_CENTERY, si->text);
      for (j = 0; j < SCF_NUM; j++)
        Cmd_Number(x_indent2+(j+1)*x_inc-1, (k+2)*v, 20, OPT_RIGHTX|OPT_CENTERY, si->data[j]);
      }

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    check_screenshot();
    mo_usleep(10000);
  }
  return(0);
}

uint8_t eve_stat_temps(struct fp_temps *t, struct gui_temps *g)
{
  int i, j, action;
  uint16_t font = 18;
  uint16_t v = display.height/(5+1);       // Vertical spacing, for 5 rows of text
  uint16_t x_left = 1;
  const uint16_t x_varwidth = 450;
  const uint16_t x_indent2 = display.width - x_varwidth;
  const uint16_t x_inc = x_varwidth / (NUM_CURRENTS/2);
  const char *name[] = {"Inj1", "Inj2", "Inj3", "Inj4", "Inj5", "Inj6", "L Coil", "R Coil", "L Pump", "R Pump"};

  while (!mo_abort) {
    for (i = 0; i < 50; i++) {
      if ((action = check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        return(action);
        }
      mo_usleep(4000);
      }

    efi_get_temps_update();

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    accrue_Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text(x_left, v, font, OPT_CENTERY, "EFI Power junction temps:");

    for (j = 0; j < NUM_CURRENTS; j++) {
      int xpos = x_indent2 + (j%(NUM_CURRENTS/2)) * x_inc;
      int ypos = (j < NUM_CURRENTS/2) ? display.height/3 : display.height*2/3;
      float x = t->junction_temperature[j];

      accrue_Send_CMD(COLOR_RGB(0, 0, 255));
      Cmd_Text(xpos, ypos, 21, OPT_CENTER, name[j]);

      if (x < 60.0)
        accrue_Send_CMD(COLOR_RGB(0, 255, 0));              // Green
      else if (x < 70.0)
        accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0));          // Amber
      else
        accrue_Send_CMD(COLOR_RGB(0xff, 0, 0));             // Red
      Cmd_Text(xpos, ypos+20, 23, OPT_CENTER, g->junction_temperature[j]);
      }

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    check_screenshot();
    mo_usleep(10000);
  }
  return(0);
}



////////////////////////////////////////////////////////////////////////////////
// O2 Sensor
////////////////////////////////////////////////////////////////////////////////

uint8_t eve_o2_sensor()
{
  int i, action;
  uint16_t font = 31;
  uint16_t v = display.height/2;       // Vertical spacing, for 2 rows of text
  char buf[64];

  while (!mo_abort) {
    for (i = 0; i < 50; i++) {
      if ((action = check_touch(20, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
        //printf("eve_o2_sensor: returning %d\n", action);
        return(action);
        }
      mo_usleep(100);
      }

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    uint16_t major_ticks = 10;
    uint16_t minor_ticks = 2;
    uint16_t val;
    uint16_t range = 180 - 80;

    uint16_t gauge_0 = OPT_NOPOINTER;
    uint16_t gauge_1 = OPT_NOBACK | OPT_NOTICKS;

    val = (uint16_t)(air_fuel_ratio * 10.0) - 80;

    Cmd_FGcolor((26<<16)|(26<<8)|192);
    Cmd_Gauge(100, display.height/2, display.height/2-5, gauge_0, major_ticks, minor_ticks, val, range);

    accrue_Send_CMD(COLOR_RGB(100, 100, 100));
    Cmd_Gauge(100, display.height/2, display.height/2-5, gauge_1, major_ticks, minor_ticks, (147 - 80), range);     // Green tick is stoichiometric

    if (o2_sensor_power_is_on && !o2_sensor_not_ready && !o2_sensor_error) {
      if (air_fuel_ratio <= 14.7)
        accrue_Send_CMD(COLOR_RGB(255, 0, 0));          // RED for rich
      else
        accrue_Send_CMD(COLOR_RGB(0x0, 0xff, 0x0));     // GREEN for lean

      Cmd_Gauge(100, display.height/2, display.height/2-5, gauge_1, major_ticks, minor_ticks, val, range);
      }

    accrue_Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text(100-55, 115, 18, OPT_CENTER, "8");
    Cmd_Text(100+55, 115, 18, OPT_CENTER, "18");
    Cmd_Text(100+55, 17, 18, OPT_CENTER, "15");
    Cmd_Text(100-55, 17, 18, OPT_CENTER, "11");

    const char *aft = "Air/Fuel: ";
    if (o2_sensor_power_is_on == 0) {
      Cmd_Text(190, v-15, font, OPT_CENTERY, aft);
      accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0x0));    // Amber
      Cmd_Text(370, v-38, 23, 0, "Sensor");
      Cmd_Text(370, v-15, 23, 0, "power OFF");
    } else if (o2_sensor_not_ready) {
      Cmd_Text(190, v-15, font, OPT_CENTERY, aft);
      accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0x0));    // Amber
      Cmd_Text(370, v-38, 23, 0, "Sensor");
      Cmd_Text(370, v-15, 23, 0, "Not Ready");
    } else if (o2_sensor_error) {
      Cmd_Text(190, v-15, font, OPT_CENTERY, aft);
      accrue_Send_CMD(COLOR_RGB(0xff, 0xbf, 0x0));    // Amber
      Cmd_Text(370, v-38, 23, 0, "Sensor");
      Cmd_Text(370, v-15, 23, 0, "Error");
    } else {
      snprintf(buf, sizeof(buf), "%s%4.1f", aft, air_fuel_ratio);
      Cmd_Text(190, v-15, font, OPT_CENTERY, buf);

      if (air_fuel_ratio < 14.5) {
        accrue_Send_CMD(COLOR_RGB(255, 0, 0));
        Cmd_Text(350, v+10, font, 0, "RICH");
        }
      else if (air_fuel_ratio > 14.9) {
        accrue_Send_CMD(COLOR_RGB(0x0, 0xff, 0x0));
        Cmd_Text(350, v+10, font, 0, "LEAN");
        }

      }

#if 0
    accrue_Send_CMD(display.normal_label_color);  
    snprintf(buf, sizeof(buf), "PRI %4d rpm   SEC %4d rpm", primary_rpm, secondary_rpm);
    Cmd_Text(203, v+30, 27, OPT_CENTERY, buf);

    snprintf(buf, sizeof(buf), "Raw Lidar height %.1f feet", avg_lidar_distance);
    Cmd_Text(203, v+50, 27, OPT_CENTERY, buf);
#endif


    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    check_screenshot();
    }
  return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Take a screenshot.
////////////////////////////////////////////////////////////////////////////////

#define ARGB4_SIZE      (2*display.width*display.height)
#define RGB565_SIZE     (2*display.width*display.height)

#define SS_SIZE     (2*display.width*display.height)
#define SS_ADDR     1024*1024/2
#define SS_BUFSZ    1024

extern void HAL_Recover_SPI(void);

static void screenshot_make_name(const char *root_or_565, const char *suffix,
                                 char *out, size_t outsz)
{
  size_t len;

  if (!root_or_565 || !out || outsz == 0)
    return;

  len = strlen(root_or_565);
  if (len > 4 && strcmp(root_or_565 + len - 4, ".565") == 0)
    len -= 4;

  snprintf(out, outsz, "%.*s.%s", (int)len, root_or_565, suffix);
}

static int screenshot_rgb565_to_png(const char *raw565_name, const char *png_name)
{
  char size_arg[32];
  int status;
  pid_t pid;

  snprintf(size_arg, sizeof(size_arg), "%dx%d", display.width, display.height);

  pid = fork();
  if (pid < 0) {
    perror("fork ffmpeg");
    return -1;
  }

  if (pid == 0) {
    execlp("ffmpeg", "ffmpeg",
           "-hide_banner", "-loglevel", "error", "-y",
           "-vcodec", "rawvideo",
           "-f", "rawvideo",
           "-pix_fmt", "rgb565",
           "-s", size_arg,
           "-i", raw565_name,
           "-frames:v", "1",
           "-f", "image2",
           "-vcodec", "png",
           png_name,
           (char *)NULL);
    perror("exec ffmpeg");
    _exit(127);
  }

  if (waitpid(pid, &status, 0) < 0) {
    perror("waitpid ffmpeg");
    return -1;
  }

  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    dbprintf("eve_screenshot: ffmpeg failed, status 0x%x\n", status);
    return -1;
  }

  return 0;
}

int eve_screenshot(char *fname)
{
  int i, n, fd, amt;
  uint8_t opts = display.screenshot_options;
  char raw565_name[256];
  char png_name[256];
  static uint32_t d[SS_BUFSZ/sizeof(uint32_t)];

  if ((opts & (SCREENSHOT_OPTION_565 | SCREENSHOT_OPTION_PNG)) == 0)
    opts = SCREENSHOT_OPTION_565;     // Preserve the old behaviour if unset/invalid.

  screenshot_make_name(fname, "565", raw565_name, sizeof(raw565_name));
  screenshot_make_name(fname, "png", png_name, sizeof(png_name));

  dbprintf("Screen shot to address %d (0x%x), size %d, opts %d\n", SS_ADDR, SS_ADDR, SS_SIZE, opts);
  Cmd_Snapshot2(RGB565, SS_ADDR, 0, 0, display.width, display.height);
  /*
   * Note: After CMD_SNAPSHOT/CMD_SNAPSHOT2, BT815/Matrix Orbital/FTDI link
   * reliably wedges on SPI RAM_G reads larger than 8 bytes, even from
   * unrelated RAM_G addresses. Normal RAM_G large reads work before
   * snapshot. Keep screenshot readback <= 8 bytes. Unfortunately, this
   * means it takes quite a while to read all the data.
   */

  /*
   * Always capture to a temporary/intermediate RGB565 file first.  PNG output
   * is then generated from that file.  If only PNG is requested, the raw file
   * is removed after conversion.
   */
  fd = open(raw565_name, O_CREAT|O_TRUNC|O_WRONLY, S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP|S_IWOTH|S_IWOTH);
  if (fd >= 0) {
    uint32_t amount = 8;                      // XXX Much above this and the FTDI bits lock up
    int ret;
    int retries = 0;

#define MAX_RETRIES 10

    for (i = 0; i < SS_SIZE; i += amount) {
      if (i + amount <= SS_SIZE)
        amt = amount;
      else
        amt = SS_SIZE - i;
      do
	{
	ret = rdxx(i + RAM_G + SS_ADDR, &d[0], amt);
	if (ret != amt) {
	  if (++retries > MAX_RETRIES) {
	    dbprintf("eve_screenshot: exceeded MAX_RETRIES, aborting\n");
            close(fd);
	    return(0);
	    }
	  //dbprintf("eve_screenshot: retrying\n");
	  mo_usleep(20000);
	  }
        } while (ret != amt);

      retries = 0;
      n = write(fd, &d[0], amt);
      //printf("a: %8d n %d\n", i + RAM_G, n);
      if (n != amt) {
        perror("eve_screenshot");
        dbprintf("eve_screenshot: write() returned n %d for write of %d bytes, aborting\n", n, amt);
        close(fd);
        return(0);
      }
    }
    close(fd);
  } else {
    perror("eve_screenshot");
    return(0);
  }

  if (opts & SCREENSHOT_OPTION_PNG) {
    if (screenshot_rgb565_to_png(raw565_name, png_name) == 0) {
      dbprintf("Screen shot PNG done, file %s\n", png_name);
    } else {
      dbprintf("Screen shot PNG conversion failed for %s\n", raw565_name);
      opts |= SCREENSHOT_OPTION_565;   // Keep the raw file if conversion failed.
    }
  }

  if (!(opts & SCREENSHOT_OPTION_565))
    unlink(raw565_name);
  else
    dbprintf("Screen shot RGB565 done, file %s\n", raw565_name);

  return(SS_SIZE);
}

////////////////////////////////////////////////////////////////////////////////
// Set screen brightness
////////////////////////////////////////////////////////////////////////////////

void eve_backlight(uint8_t percent)
{
  static uint8_t hz_set = 0;
  int pwm = ((int)percent * 128 ) / 100;

  if (percent > 100)
    return;
  if (pwm > 128)
    pwm = 128;

  //printf("Setting pwm duty to %d\n", pwm);

  if (!hz_set) {
    wr16(REG_PWM_HZ + RAM_REG, 5000);               // Low duty values don't work unless freq is high
    hz_set = 1;
    }
  wr8(REG_PWM_DUTY + RAM_REG, (uint8_t)pwm);        // Set backlight PWM duty

}

uint8_t eve_set_backlight()
{
  int draw_pressed_and_return = 0;
  //uint16_t font = 31;
  //uint16_t v = display.height/2;       // Vertical spacing, for 2 rows of text

  uint8_t t, tag = 42, tag_done = 43;
  uint16_t x = 50, y = 80, x_w = 250, y_w = 15;
  uint16_t val = 50;
  uint16_t range = 100;

  while (!mo_abort) {
    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    accrue_Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text(x, 30, 30, OPT_CENTERY, "Set brightness:");

    if (draw_pressed_and_return)
      Cmd_FGcolor(EGREEN_PRESSED);
    else
      Cmd_FGcolor(EGREEN);
    accrue_Send_CMD(TAG(tag_done));
    Cmd_Button(350, 20, 100, 88, 30, 0, "Done");

    Cmd_Track(x, y, x_w, y_w, tag);
    Cmd_FGcolor((26<<16)|(26<<8)|192);
    accrue_Send_CMD(TAG(tag));
    Cmd_Slider(x, y, x_w, y_w, 0, val, range);

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    if (draw_pressed_and_return) {
      mo_usleep(400000);                            // De-bounce time
      while (rd8(REG_TOUCH_TAG + RAM_REG) != 0)
        mo_usleep(1000);
      return(val);
      }

    uint32_t tracker = rd32(REG_TRACKER + RAM_REG);
    if ((tracker & 0xff) == tag) {
      val = (uint16_t)((float)(tracker >> 16) / (float)655.35);
      if (val > 100)
        val = 100;
      if (val == 0)
        val = 1;
      //printf("eve_set_backlight: tracker, val set to %d\n", val);
      eve_backlight(val);
      }
    t = rd8(REG_TOUCH_TAG + RAM_REG);           // Check for DONE
    if (t == tag_done) {
      draw_pressed_and_return = 1;
      }

    check_screenshot();
    }
  return(0);
}

//
// HF Radio controls
//

#define BAR_STEPS       20
#define SM_TICK_HEIGHT   5
#define SM_THIN_TICK     1
#define SM_THICK_TICK    3
#define SM_FONT          20

typedef struct bargraph {
  char *label;
  uint8_t thick_tick;            // Otherwise thin_tick
  } bargraph_t;

// S-Meter
const bargraph_t bg_sm[] = {
  {"S1", 1}, {NULL, 0}, {"3", 1}, {NULL, 0}, {"5", 1}, {NULL, 0}, {"7", 1}, {NULL, 0}, {"9", 1},
  {NULL, 0}, {NULL, 0}, {NULL, 0}, {"+20", 1},
  {NULL, 0}, {NULL, 0}, {NULL, 0}, {"40", 1},
  {NULL, 0}, {NULL, 0}, {NULL, 0}, {"60", 1}};

// SWR
const bargraph_t bg_swr[] = {
  {"SWR", 0}, {NULL, 0}, {NULL, 0}, {NULL, 0},
  {"2", 1}, {NULL, 0}, {"3", 1}, {NULL, 0}};

// Transmit power
const bargraph_t bg_tp[] = {
  {"RF", 1}, {NULL, 0}, {NULL, 0}, {NULL, 0}, {NULL, 0},
  {"50", 1}, {NULL, 0}, {NULL, 0}, {NULL, 0}, {NULL, 0},
  {"100", 1}, {NULL, 0}};


static void eve_bargraph(int base_x, int base_y, int step, int direction, int bar_offset, int bar_width, const bargraph_t *bg, int dimension, int bar_percentage, int bar_dimension)
{
  int i;

  if (bar_width)
    accrue_Send_CMD(display.normal_label_color);  
  else
    accrue_Send_CMD(COLOR_RGB(50, 50, 50));         // Dim scale not in use

  // Horizontal line
  accrue_Send_CMD(LINE_WIDTH(8*1));
  accrue_Send_CMD(VERTEX2F(base_x, base_y));
  accrue_Send_CMD(VERTEX2F(base_x+(dimension-1)*step, base_y));
  // Vertical ticks
  for (i = 0; i < dimension; i++, bg++) {
    int x = base_x+i*step;
    int y_top = base_y-direction*SM_TICK_HEIGHT;

    if (bg->thick_tick)
      accrue_Send_CMD(LINE_WIDTH(16*1));
    else
      accrue_Send_CMD(LINE_WIDTH(8*1));
    accrue_Send_CMD(VERTEX2F(base_x+i*step, base_y));
    accrue_Send_CMD(VERTEX2F(x, y_top));
    if (bg->label) {
      Cmd_Text(x, y_top-direction*8, SM_FONT, OPT_CENTER, bg->label);
      accrue_Send_CMD(BEGIN(LINES));
      }
    }

  // The bar-graph
  if (bar_width) {

    //printf("step %d bar_dimension %d bar_percentage %d, x1 %d, x2 %d\n", step, bar_dimension, bar_percentage, base_x, base_x+(step*bar_percentage*bar_dimension)/100);

    accrue_Send_CMD(COLOR_RGB(128, 128, 0));  
    accrue_Send_CMD(LINE_WIDTH(16*1));
    accrue_Send_CMD(BEGIN(RECTS));
    accrue_Send_CMD(VERTEX2F(base_x, base_y+direction*bar_offset));
    accrue_Send_CMD(VERTEX2F(base_x+(step*(bar_percentage*bar_dimension))/100, base_y+direction*(bar_offset+bar_width)));
    }

}

#define BAR_BELOW        1
#define BAR_ABOVE        -1

static void eve_bargraphs(int transmitting, int s_percent, float swr_percent, int tp_percent)
{
  const int sm_step = 8;
  const int sm_base_x = 20;
  const int sm_base_y = 20;

  const int sw_step = sm_step;
  const int sw_base_x = sm_base_x;
  const int sw_base_y = sm_base_y+20;

  //int direction = 1;            // 1 for "down" the screen, -1 for "up" the screen
  int bar_offset = 5;
  int bar_width = 10;

  int receiving = (transmitting) ? 0 : 1;

  accrue_Send_CMD(BEGIN(LINES));
  accrue_Send_CMD(VERTEXFORMAT(0));               // Setup VERTEX2F to take pixel coordinates

  // S-Meter
  eve_bargraph(sm_base_x, sm_base_y, sm_step, BAR_BELOW, bar_offset, receiving*bar_width, &bg_sm[0], DIMENSION(bg_sm), s_percent, DIMENSION(bg_sm)-1);

  // SWR
  eve_bargraph(sw_base_x, sw_base_y, sw_step, BAR_ABOVE, bar_offset, transmitting*bar_width, &bg_swr[0], DIMENSION(bg_swr), swr_percent, DIMENSION(bg_swr)-1);

  // Transmit power
  int tp_base_x = sw_base_x + DIMENSION(bg_swr)*sw_step+10;
  int tp_base_y = sw_base_y;
  int tp_step = sw_step;
  eve_bargraph(tp_base_x, tp_base_y, tp_step, BAR_ABOVE, bar_offset, transmitting*bar_width, &bg_tp[0], DIMENSION(bg_tp), tp_percent, DIMENSION(bg_tp)-2);

}

const char *region_names[] = {"Southern", "NorthEast", "NorthWest", "MWARA INO-1", "MWARA SEA-3", "MWARA SP-6", "160m", "80m", "40m", "30m", "20m", "17m", "15m", "12m", "10m" };
#define NUM_REGIONS DIMENSION(region_names)
#define MAX_FREQ    6

uint8_t eve_hfradio()
{
  const int khz_regions[NUM_REGIONS][MAX_FREQ] = {
                            {3461, 6565, 8822, 0},
                            {3452, 6541, 8843, 0},
                            {3452, 6610, 8831, 0},
                            {3476, 5634, 8879, 13306, 17961, 0},
                            {3470, 6556, 11396, 13318, 17907, 0},
                            {3467, 5643, 8867, 13261, 17904, 0},
                            {1800, 0},
                            {3500, 0},
                            {7000, 0},
                            {10100, 0},
                            {14000, 0},
                            {18068, 0},
                            {21000, 0},
                            {24890, 0},
                            {28000, 0}
                          };
  const uint8_t usb[NUM_REGIONS] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t vfoA_font = 29;
  const uint8_t region_font = 26;

  int draw_pressed_and_return = 0;
  int draw_pressed_region = 0;
  int draw_pressed_tune = 0;
  //uint16_t font = 31;
  //uint16_t v = display.height/2;       // Vertical spacing, for 2 rows of text

  uint8_t t, tag = 42, tag_done = 43, tag_freq = 44, tag_region = 45, tag_tune = 46;
  const uint16_t range = 200;
  //uint8_t band_change = 0;

  static int val = range/2;
  static int freq_index = 0;
  static int region_index = 0;

  static hf_data_t hf_data;

  int32_t freq_base = khz_regions[region_index][freq_index] * 1000;
  int32_t freq_trim = 0;
  static int32_t last_freq = 0;

  char buf[32];
  //int nret;
  //int sts;
  int loops;
  int transmitting;

  loops = 0;
  while (!mo_abort) {
    if ((loops++ % 20) == 0) {
      hf_get_data(&hf_data);
      transmitting = (hf_data.flags & HF_TRANSMITTING);
      }

    accrue_Send_CMD(CMD_DLSTART);
    accrue_Send_CMD(VERTEXFORMAT(0));
    accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(CLEAR(1, 1, 1));

    float fswr;
    if (hf_data.swr > 3.5)
      fswr = 3.5;
    else if (hf_data.swr < 1.0)
      fswr = 1.0;
    else
      fswr = hf_data.swr;

    int swr_percent = (fswr <= 2.0) ?
                    (int)((fswr - 1.0)*100*(4/7)) :
                    (int)((fswr - 2.0)/1.5*100*(3/7)) + (400/7);

    //printf("hf_data.swr = %.1f, fswr = %.1f, swr_percent = %d\n", hf_data.swr, fswr, swr_percent);

    eve_bargraphs(transmitting, hf_data.s_meter, swr_percent, (int)(hf_data.forward_power));

    accrue_Send_CMD(COLOR_RGB(255, 255, 255));  

    const int x_done = 370;
    const int y_done = 1;
    const int x_region = 250;
    const int y_region = y_done;

    const int x_vfoA = 280;
    const int y_vfoA = 75;

    if (draw_pressed_and_return)
      Cmd_FGcolor(EGREEN_PRESSED);
    else
      Cmd_FGcolor(EGREEN);
    accrue_Send_CMD(TAG(tag_done));
    Cmd_Button(x_done, y_done, 100, 40, 30, 0, "Done");

    if (draw_pressed_region)
      Cmd_FGcolor(EGREEN_PRESSED);
    else
      Cmd_FGcolor(EORANGE);
    accrue_Send_CMD(TAG(tag_region));
    Cmd_Button(x_region, y_region, 100, 40, region_font, 0, region_names[region_index]);

    const int x_tune = 10;
    const int y_tune = 80;
    if (draw_pressed_tune)
      Cmd_FGcolor(EGREEN_PRESSED);
    else
      Cmd_FGcolor(EORANGE);
    accrue_Send_CMD(TAG(tag_tune));
    Cmd_Button(x_tune, y_tune, 100, 40, 30, 0, "TUNE");

    uint16_t x_slider = 250, y_slider = 110, xw_slider = 200, yw_slider = 10;

    Cmd_Track(x_slider, y_slider, xw_slider, yw_slider, tag);
    Cmd_FGcolor((26<<16)|(26<<8)|192);
    accrue_Send_CMD(TAG(tag));
    Cmd_Slider(x_slider, y_slider, xw_slider, yw_slider, 0, val, range);

    int32_t freq = freq_base + freq_trim;
    accrue_Send_CMD(COLOR_RGB(255, 255, 255));
    snprintf(buf, sizeof(buf), "%1d.%03d%03d MHz", freq/1000000, (freq / 1000) % 1000, freq % 1000);
    accrue_Send_CMD(TAG(tag_freq));
    Cmd_Text(x_vfoA, y_vfoA, vfoA_font, OPT_CENTERY, buf);

    accrue_Send_CMD(DISPLAY());                     //End the display list
    accrue_Send_CMD(CMD_SWAP);                      //Swap commands into RAM
    release_wr32();
    UpdateFIFO();
    Wait4CoProFIFOEmpty();

    if (freq > 0 && freq != last_freq) {
      // Update the VFO frequency
      //printf("Updating VFO frequencies\n");
      hf_data.vfoA = freq;
      hf_data.vfoB = freq;
      if (usb[region_index])
        hf_data.flags |= HF_USB;
      else
        hf_data.flags &= ~HF_USB;
      /* sts = */ hf_operation(&hf_data);

      if (abs(freq - last_freq) > 10000) {
        // Assume a band change just happened
        //printf("Band change delay\n");
        mo_usleep(500000);     // XXX Do something better here
      }
      last_freq = freq;
    }

    if (draw_pressed_and_return) {
      mo_usleep(400000);                            // De-bounce time
      while (rd8(REG_TOUCH_TAG + RAM_REG) != 0)
        mo_usleep(1000);
      return(val);
      }
    if (draw_pressed_region) {
      mo_usleep(400000);                            // De-bounce time
      while (rd8(REG_TOUCH_TAG + RAM_REG) != 0)
        mo_usleep(1000);
      draw_pressed_region = 0;
    }

    if (!transmitting) {
      uint32_t tracker = rd32(REG_TRACKER + RAM_REG);
      if ((tracker & 0xff) == tag) {
        val = (int)((float)(tracker >> 16) / (float)(655.35 / 2));
        if (val > range)
          val = range;
        if (val < 0)
          val = 0;
        //printf("eve_hfradio: tracker, val set to %d\n", val);
        //eve_backlight(val);
        }
      freq_trim = (val - range/2) * 5;                        // 10 Hz increments, 1000 Hz total
      }

    t = rd8(REG_TOUCH_TAG + RAM_REG);           // Check for DONE
    if (t == tag_freq && !transmitting) {
      // Make sure
      mo_usleep(5000);
      t = rd8(REG_TOUCH_TAG + RAM_REG);
      if (t == tag_freq) {
        if (val == range/2) {
          ++freq_index;
          if (khz_regions[region_index][freq_index] == 0)
            freq_index = 0;
          } 
        freq_base = khz_regions[region_index][freq_index] * 1000;
        val = range/2;
        mo_usleep(100000);
        }
      }
    if (t == tag_done) {
      draw_pressed_and_return = 1;
      }
    if (t == tag_tune) {
      if (draw_pressed_tune == 1) {
          draw_pressed_tune = 0;
          hf_tune(0);
        } else {
          draw_pressed_tune = 1;
          hf_tune(1);
        }
        loops = 0;
        mo_usleep(200000);
      }
    if (t == tag_region && !transmitting) {
      draw_pressed_region = 1;
      if (++region_index > NUM_REGIONS - 1)
        region_index = 0;
      freq_index = 0;
      val = range/2;
      freq_base = khz_regions[region_index][freq_index] * 1000;
      }

    check_screenshot();
    }
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Engine/Prop Vibration display
////////////////////////////////////////////////////////////////////////////////

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#ifndef RGB
#define RGB(r,g,b) ((((uint32_t)(r) & 255u) << 16) | (((uint32_t)(g) & 255u) << 8) | ((uint32_t)(b) & 255u))
#endif

#define VIB_COL_GREEN   RGB(0x14,0x80,0x26)
#define VIB_COL_AMBER   RGB(0xff,0xbf,0x00)
#define VIB_COL_RED     RGB(0xff,0x00,0x00)
#define VIB_COL_GRAY    RGB(0x80,0x80,0x80)
#define VIB_COL_DIM     RGB(0x70,0x70,0x70)
#define VIB_COL_DARK    RGB(0x40,0x40,0x40)
#define VIB_COL_WHITE   RGB(0xff,0xff,0xff)
#define VIB_COL_CYAN    RGB(0x00,0xff,0xff)

static void draw_circle_lines(int cx, int cy, int radius, uint32_t color)
{
    int line_px = 2;   // ring thickness

    accrue_Send_CMD(BEGIN(POINTS));

    // Outer disk
    accrue_Send_CMD(COLOR_RGB((color >> 16) & 255u,
                              (color >> 8) & 255u,
                               color & 255u));
    accrue_Send_CMD(POINT_SIZE(radius * 16));
    accrue_Send_CMD(VERTEX2F(cx, cy));

    // Inner cut-out
    accrue_Send_CMD(COLOR_RGB(0, 0, 0));
    accrue_Send_CMD(POINT_SIZE((radius - line_px) * 16));
    accrue_Send_CMD(VERTEX2F(cx, cy));

    accrue_Send_CMD(END());
}


static void draw_line(int x0, int y0, int x1, int y1, uint32_t color, int width_px)
{
    accrue_Send_CMD(COLOR_RGB((color >> 16) & 255u, (color >> 8) & 255u, color & 255u));
    accrue_Send_CMD(LINE_WIDTH(width_px * 16));
    accrue_Send_CMD(BEGIN(LINES));
    accrue_Send_CMD(VERTEX2F(x0, y0));
    accrue_Send_CMD(VERTEX2F(x1, y1));
    accrue_Send_CMD(END());
}

static const char *source_name(uint8_t source)
{
    switch (source)
    {
    case VIB_ACCEL_SOURCE_ADXL355:   return "ADXL";
    case VIB_ACCEL_SOURCE_IIS3DWBG1: return "IIS3";
    case VIB_ACCEL_SOURCE_BOTH:      return "BOTH";
    default:                         return "--";
    }
}

int eve_vibration_draw_page()
{
    const int w = display.width ? display.width : DWIDTH;
    const int h = display.height ? display.height : DHEIGHT;

    const int left_x = 6;
    const int indent_x = 10;
    const int indent2_x = 100;
    const int center_x = w / 2;
    const int center_y = h / 2 + 4;
    const int polar_r = (h < 140) ? 50 : 58;
    const int right_x = w - 140;

    int i, action;
    uint8_t snapshot_pressed;
    char buf[64];
    vibsense_live_metrics_t metrics[2], *m, *ma;

    while (!mo_abort && !vib_thread_abort) {
      snapshot_pressed = 0;
      if (vib_get_data_for_display(&metrics[0])) {
        m = &metrics[0];        // Single source data, or IIS data if both
        if (options.vib_source == VIB_ACCEL_SOURCE_BOTH)
          ma = &metrics[1];     // This will be ADXL data
        else
          ma = NULL;
        }
      else {
        m = NULL;
        ma = NULL;
        }

      for (i = 0; i < 50; i++) {
        if ((action = check_touch(4, SWIPE_GAP_X, swipe_debug, 0, SWIPE_GAP_Y)) > 0) {
          //dbprintf("eve_vibration_draw_page: TOP: returning %d\n", action);
          if (action == 3) {
            eve_do_screenshot = 1;
            break;
            }
          else if (action == 4) {
            //passes_after_snapshot_button = SNAPSHOT_HILITE_PASSES;
            snapshot_pressed = 1;
            break;
            }
          else {
            return(action);         // Must be a swipe
            }
          }
        mo_usleep(1000);            // Update rate around 50 msec, i.e. 20 times per second
        }

      Wait4CoProFIFOEmpty();
      accrue_Send_CMD(CMD_DLSTART);
      accrue_Send_CMD(VERTEXFORMAT(0));
      accrue_Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
      accrue_Send_CMD(CLEAR(1, 1, 1));

      /* Left numerical summary. */
      accrue_Send_CMD(COLOR_RGB(255, 255, 255));
      Cmd_Text(left_x, 8, 18, 0, "ENGINE/PROP VIBRATION:");

      uint32_t rpm = primary_rpm ? primary_rpm : secondary_rpm;
      if (m && m->rpm > 1.0)
          rpm = (uint32_t)lrint(m->rpm);

      snprintf(buf, sizeof(buf), "RPM %4u", rpm);
      Cmd_Text(left_x+indent_x, 28, 18, 0, buf);

      snprintf(buf, sizeof(buf), "%u revs", vib_rotations);
      Cmd_Text(left_x+indent2_x, 28, 18, 0, buf);

      if (m && m->valid)
      {
          uint32_t status_col = VIB_COL_GREEN;
          if (m->ips_xy >= 0.25)
              status_col = VIB_COL_RED;
          else if (m->ips_xy >= 0.15)
              status_col = VIB_COL_AMBER;

          accrue_Send_CMD(COLOR_RGB((status_col >> 16) & 255u, (status_col >> 8) & 255u, status_col & 255u));
          snprintf(buf, sizeof(buf), "IPS %.3f", m->ips_xy);
          Cmd_Text(left_x+indent_x, 48, 18, 0, buf);
          accrue_Send_CMD(COLOR_RGB(255, 255, 255));
          snprintf(buf, sizeof(buf), "X   %.3f", m->ips_x);
          Cmd_Text(left_x+indent_x, 68, 18, 0, buf);
          snprintf(buf, sizeof(buf), "Y   %.3f", m->ips_y);
          Cmd_Text(left_x+indent_x, 88, 18, 0, buf);
          snprintf(buf, sizeof(buf), "PH  %03.0f", m->phase_deg);
          Cmd_Text(left_x+indent_x, 108, 18, 0, buf);

          accrue_Send_CMD(COLOR_RGB((display.vib_line1_color >> 16) & 255u, (display.vib_line1_color >> 8) & 255u, display.vib_line1_color & 255u));
          snprintf(buf, sizeof(buf), "%s %u revs", source_name(m->accel_source), m->revolutions);
          Cmd_Text(360, 95, 18, 0, buf);

          if (ma && ma->valid) {
            status_col = VIB_COL_GREEN;
            if (ma->ips_xy >= 0.25)
                status_col = VIB_COL_RED;
            else if (ma->ips_xy >= 0.15)
                status_col = VIB_COL_AMBER;
            accrue_Send_CMD(COLOR_RGB((status_col >> 16) & 255u, (status_col >> 8) & 255u, status_col & 255u));
            snprintf(buf, sizeof(buf), "%.3f", ma->ips_xy);
            Cmd_Text(left_x+indent2_x, 48, 18, 0, buf);
            accrue_Send_CMD(COLOR_RGB(255, 255, 255));
            snprintf(buf, sizeof(buf), "%.3f", ma->ips_x);
            Cmd_Text(left_x+indent2_x, 68, 18, 0, buf);
            snprintf(buf, sizeof(buf), "%.3f", ma->ips_y);
            Cmd_Text(left_x+indent2_x, 88, 18, 0, buf);
            snprintf(buf, sizeof(buf), "%03.0f", ma->phase_deg);
            Cmd_Text(left_x+indent2_x, 108, 18, 0, buf);

            accrue_Send_CMD(COLOR_RGB((display.vib_line2_color >> 16) & 255u, (display.vib_line2_color >> 8) & 255u, display.vib_line2_color & 255u));
            snprintf(buf, sizeof(buf), "%s %u revs", source_name(ma->accel_source), ma->revolutions);
            Cmd_Text(360, 110, 18, 0, buf);
          }
      }
      else
      {
          accrue_Send_CMD(COLOR_RGB(255, 191, 0));
          Cmd_Text(left_x+indent_x, 50, 18, 0, "NO DATA");
      }

      /* Central polar display. */
      draw_circle_lines(center_x, center_y, polar_r, VIB_COL_DIM);
      draw_circle_lines(center_x, center_y, polar_r / 2, VIB_COL_DARK);
      draw_line(center_x - polar_r, center_y, center_x + polar_r, center_y, VIB_COL_DIM, 1);
      draw_line(center_x, center_y - polar_r, center_x, center_y + polar_r, VIB_COL_DIM, 1);

      accrue_Send_CMD(COLOR_RGB(160, 160, 160));
      Cmd_Text(center_x, center_y - polar_r - 7, 16, OPT_CENTER, "0");
      Cmd_Text(center_x + polar_r + 10, center_y, 16, OPT_CENTER, "90");
      Cmd_Text(center_x, center_y + polar_r + 7, 16, OPT_CENTER, "180");
      Cmd_Text(center_x - polar_r - 14, center_y, 16, OPT_CENTER, "270");

      if (m && m->valid)
      {
          int dx = (int)lrint(m->dot_x_norm * (double)polar_r);
          int dy = (int)lrint(m->dot_y_norm * (double)polar_r);
          int dot_x = center_x + dx;
          int dot_y = center_y + dy;

          draw_line(center_x, center_y, dot_x, dot_y, display.vib_line1_color, 2);
          accrue_Send_CMD(COLOR_RGB((display.vib_line1_color >> 16) & 255u, (display.vib_line1_color >> 8) & 255u, display.vib_line1_color & 255u));
          accrue_Send_CMD(POINT_SIZE(5 * 16));
          accrue_Send_CMD(BEGIN(POINTS));
          accrue_Send_CMD(VERTEX2F(dot_x, dot_y));
          accrue_Send_CMD(END());

          accrue_Send_CMD(COLOR_RGB(180, 180, 180));
          snprintf(buf, sizeof(buf), "%.2fips", m->display_full_scale_ips);
          Cmd_Text(center_x+30, 15, 16, 0, buf);
      }
      if (ma && ma->valid)
      {
          int dx = (int)lrint(ma->dot_x_norm * (double)polar_r);
          int dy = (int)lrint(ma->dot_y_norm * (double)polar_r);
          int dot_x = center_x + dx;
          int dot_y = center_y + dy;

          draw_line(center_x, center_y, dot_x, dot_y, display.vib_line2_color, 2);
          accrue_Send_CMD(COLOR_RGB((display.vib_line2_color >> 16) & 255u, (display.vib_line2_color >> 8) & 255u, display.vib_line2_color & 255u));
          //draw_line(center_x, center_y, dot_x, dot_y, VIB_COL_RED, 2);
          //accrue_Send_CMD(COLOR_RGB(255, 0, 0));
          accrue_Send_CMD(POINT_SIZE(5 * 16));
          accrue_Send_CMD(BEGIN(POINTS));
          accrue_Send_CMD(VERTEX2F(dot_x, dot_y));
          accrue_Send_CMD(END());

          //accrue_Send_CMD(COLOR_RGB(180, 180, 180));
          //snprintf(buf, sizeof(buf), "%.2fips", m->display_full_scale_ips);
          //Cmd_Text(center_x+30, 15, 16, 0, buf);
      }

      accrue_Send_CMD(COLOR_RGB(255, 255, 255));
      /* Right snapshot button. */
      Cmd_FGcolor(snapshot_pressed ? VIB_COL_AMBER : VIB_COL_GREEN);
      //accrue_Send_CMD(TAG(EVE_VIBRATION_TAG_SNAPSHOT));
      Cmd_Button(right_x, 26, 126, 54, 26, 0, "SNAPSHOT");

      accrue_Send_CMD(COLOR_RGB(160, 160, 160));
      snprintf(buf, sizeof(buf), "Log %d rev csv", vib_snapshot_rotations);
      Cmd_Text(right_x + 63, 70, 16, OPT_CENTER, buf);

      accrue_Send_CMD(TAG(0));
      accrue_Send_CMD(DISPLAY());
      accrue_Send_CMD(CMD_SWAP);
      release_wr32();
      UpdateFIFO();
      Wait4CoProFIFOEmpty();

      if ((i = check_touch_while_wait_fifo_empty(4, SWIPE_GAP_X, 0, 0, SWIPE_GAP_Y)) > 0) {
        //dbprintf("eve_vibration_draw_page: BOTTOM: returning %d\n", i);
        if (i == 3 || eve_do_screenshot)
          eve_do_screenshot = 1;
        else if (i == 4) {
          //passes_after_snapshot_button = SNAPSHOT_HILITE_PASSES;
          snapshot_pressed = 1;
          }
        else {
          return(i);
        }
      }

    check_screenshot();

    if (snapshot_pressed)
      return(4);
    }
  return(0);
}

/*
uint8_t eve_vibration_poll_touch(void)
{
    uint8_t tag = eve_debounce_tagread(NULL, 0);
    if (tag == EVE_VIBRATION_TAG_SNAPSHOT)
        return EVE_VIBRATION_ACTION_SNAPSHOT;
    return EVE_VIBRATION_ACTION_NONE;
}
*/

