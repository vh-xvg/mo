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
// Functions for XVG
//

#include <efi_protocol.h>
#include "eve_vibration_display.h"

void speech(char *);
void play_sound(char *);
void eve_blank(void);
void eve_initial_screen(void);
void flaps_request_data(void);
void flaps_put_flap_settings(void);

uint8_t eve_debounce_tagread(void (*)(uint8_t), int);
void eve_doors_page(uint8_t);
void eve_draw_security_page(uint8_t);
int eve_security_page(void);
void eve_send_image(char *, uint32_t, uint32_t, uint32_t);
int check_touch(int, int, int, int, int);
uint8_t *get_fake_data(int);
uint8_t get_door_state(void);
int eve_even_live(uint8_t, uint32_t, char *namstr[], uint16_t);
uint8_t eve_injectors();
uint8_t eve_coilpumps();
uint8_t eve_battery_display(int);
uint8_t eve_flaps_display(int);
uint8_t eve_etx1200_display();
uint8_t eve_aircon(void);
uint8_t eve_o2_sensor(void);
uint8_t eve_injector_detail(uint8_t);
uint8_t eve_coilpack_detail(uint8_t);
uint8_t eve_fuelpump_detail(uint8_t);
uint8_t eve_fuelpump_laststart_plot(uint8_t, struct pump_start_record *, int);
uint8_t eve_ready_to_start(int);
uint8_t eve_diagnostic(uint8_t);
uint8_t eve_configure(uint8_t);
uint8_t eve_configure_flaps(uint8_t);
uint8_t eve_checklist(char *, void (*)(uint8_t, uint8_t), uint8_t, uint8_t *, uint8_t);
void check_screenshot(void);
void do_screenshot(void);
int eve_screenshot(char *);
void eve_backlight(uint8_t);
uint8_t eve_set_backlight(void);
uint8_t eve_hfradio(void);
uint8_t eve_stat_comms(efi_stats_t *);
uint8_t eve_stat_op_response(response_counts_t *, int, const char *);
uint8_t eve_stat_injectors(efi_stats_t *);
uint8_t eve_stat_coilpumps(efi_stats_t *);
uint8_t eve_stat_temps(struct fp_temps *, struct gui_temps *);
int eve_summary(void);
int vib_get_data_for_display(vibsense_live_metrics_t *);
//int vib_get_data_for_display(vibsense_live_metrics_t (*)[2]);

// Return values for swipe touchpad, and other events
#define DOUBLE_TOUCH                248
#define FORCE_INJECTOR_SUMMARY      249
#define SWIPE_LEFT                  250
#define SWIPE_RIGHT                 251
#define BOTH_ON_FOR_TIMEOUT         252
#define SWIPE_UP                    253
#define SWIPE_DOWN                  254

// Bitmasks for door states
#define LEFT_DOOR       0x1
#define RIGHT_DOOR      0x2
#define BAGGAGE_DOOR    0x4

// States for the door fault variables
#define ETX_NO_BLINKING     0
#define ETX_NORMAL          0
#define ETX_BLINK_2SEC      1
#define ETX_BLINK_5SEC      2
#define ETX_SOLID           3

// Checklist actions
typedef enum {
  CL_PREV,
  CL_NEXT
  } clret_t;

// Pilot data for the security page
#define SEC_MAXLENGTH   16

#define SEC_OPTION_LOCAL_FAKE_DATA          0x1
#define SEC_OPTION_EFI_FAKE_DATA            0x2
#define SEC_OPTION_EFI_DIAG_DATA            0x4
#define SEC_OPTION_REAL_WITH_FAKE_ENGINE    0x8

#define SCREENSHOT_OPTION_565               0x01u
#define SCREENSHOT_OPTION_PNG               0x02u

struct pilot_data {
  char *name;
  char *fullname;
  int idx;                      // Tracks code entry
  int code_length;              // Length of security code, max 16
  char code[SEC_MAXLENGTH];
  int administrator;            // 1 == Administrator
  int options;                  // Other options
};

// Display information structure
struct display_data {
  int width;
  int height;
  int plot_height;
  int y_over_fs;
  int y_fs;
  int y_fault;
  int main_label_font, small_axis_font;
  int normal_line_width, fault_line_width, caution_line_width, pumpstart_line_width, pumpstart_xscale;
  uint32_t clear_color, axis_color, normal_label_color, dim_label_color, fault_label_color, normal_line_color, normal_title_color, fault_line_color, caution_line_color, pumpstart_line_color, vib_line1_color, vib_line2_color;
  int coilpack_alternator_threshold[2];
  uint8_t screenshot_options;
};

// A structure used in button array descriptions
struct button_array {
  int16_t x, y, w, h, font;
  uint32_t color;
  char *text;
  };


struct logging_config_data {
  char *directory_usb;
  char *directory_default;
  char *log_names[10];
};

// Display states
typedef enum {
  DS_DISCONNECTED,
  DS_IDLE,
  DS_INITIAL_SCREEN,
  DS_INITIAL_BATTERY_BACKUP,
  DS_SET_BACKLIGHT,
  DS_SECURITY,
  DS_DOORS,
  DS_START,
  DS_CHECKLISTS,
  DS_SUMMARY,
  DS_COILPUMPS,
  DS_INJECTORS,
  DS_BATTERY_BACKUP,
  DS_VIBRATION,
  DS_AIRCON,
  DS_O2_SENSOR,
  DS_COILPACK_DETAIL,
  DS_FUELPUMP_DETAIL,
  DS_INJECTOR_DETAIL,
  DS_HFRADIO,
  DS_DIAGNOSTIC,
  DS_STAT_COMMS,
  DS_STAT_EFI_RESPONSE,
  DS_STAT_INJECTORS,
  DS_STAT_COILPUMPS,
  DS_STAT_JUNCTION_TEMPS,
  DS_STAT_EFI,
  DS_FUELPUMP_LASTSTART,
  DS_ETX_FAULT,
  DS_CONFIGURE,
  DS_CONFIGURE_FLAPS,
  DS_NO_CONFIGURATION
  } dstate_t;

// Structures for A/C data
typedef enum {
  AM_OFF = 0,
  AM_MANUAL,
  AM_AUTO,
  AM_VENT
  } aircon_mode_t;

typedef struct aircon_data {
  aircon_mode_t mode;
  float temp_evaporator;
  float temp_rear_vent_air;
  float temp_inside_left;
  float temp_inside_right;
  float pressure_evap_highside;
  float voltage_bilge_blower_pot;
  uint32_t bilge_adc_output;
  uint32_t naca_pwm_width;
  uint32_t scan_time_usec;
  uint32_t valid;
  } aircon_data_t;

typedef struct display_loop {
  struct display_loop *next;
  struct display_loop *prev;
  struct display_loop *up;
  struct display_loop *down;
  struct display_loop *detail;
  struct display_loop *from;
  dstate_t ds;
  } display_loop_t;


aircon_data_t *get_ac_data(void);
