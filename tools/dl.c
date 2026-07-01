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
// Convert log data to CSV files
//

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include "efi_protocol.h"

double sample_period_usec = (double) 52.0;               // Default ADC period in usec, if there is no correction

#define DIMENSION(x) (sizeof(x)/sizeof((x)[0]))

// File names
char *fname[] = {
                "injector_0",
                "injector_1",
                "injector_2",
                "injector_3",
                "injector_4",
                "injector_5",
                "coil_left",
                "coil_right",
                "fuelpump_left",
                "fuelpump_right"};

char *fname_pumpstart[] = {
  "left_pumpstart_detail",
  "right_pumpstart_detail"};

// Conversion factors
int amps_per_volt[] = {1,1,1,1,1,1,5,5,5,5};
int idx;

int debug = 0;
static uint32_t where;
char fnbuf[256];

uint8_t buf[2048];
int abort_this_file = 1;

#define ADC_REF_MV              2900
#define ADC_RESULT_MV(x)        (((x)*ADC_REF_MV)/4095)
#define ADC_FAULT_THRESHOLD     3300

int skip_distance = 0;

void pout(FILE *f, uint32_t sample, uint16_t val)
{
  double real_sample, real_value;
  static int skip_bad = 0, skipping_bad = 0;
  static double last_sample = 0.0;

  real_sample = (double)sample * sample_period_usec / (double)1000000;

  if (val > 4095) {
    if (debug) {
      printf("pout: val %d (0x%x) not possible\n", val, val);
      printf("File: %s  Offset: %d\n", fnbuf, where);
      abort_this_file = 1;
      return;
    } else {
      fprintf(stderr, "pout: val %d (0x%x) not possible\n", val, val);
      fprintf(stderr, "File: %s  Offset: %d\n", fnbuf, where);
    }
    exit(1);
    }

  if (skip_bad > 0 || val > ADC_FAULT_THRESHOLD) {
    if (val > ADC_FAULT_THRESHOLD) {
      skip_bad = 15;
      skipping_bad = 1;
      fprintf(f, "%12.9f,%8.6f\n", real_sample, (float)0.0);
    } else {
      skip_bad--;
    }
    last_sample = real_sample;
    return;
  }
  if (skipping_bad) {
    skipping_bad = 0;
    if (last_sample > 0.0)
      fprintf(f, "%12.9f,%8.6f\n", last_sample, (float)0.0);
  }

  real_value = (double)(ADC_RESULT_MV(val)) / 1000.0 * (double)amps_per_volt[idx];

  if (debug)
    printf("                                        %12.9f %8.6f\n", real_sample, real_value);
  fprintf(f, "%12.9f,%8.6f\n", real_sample, real_value);
}

int process_log(int fd, FILE *f)
{
  struct log_header header;
  int nb;
  uint16_t val;
  static uint16_t last_val;
  int8_t *p;
  uint8_t repeat_count;
  uint32_t sample;
  int discard;
  int sample_number;
  int do_output;

  if (where == 0)
    discard = 1;
  else
    discard = 0;

  if (read(fd, &header, sizeof(header)) != sizeof(header))
    return(0);
  where += sizeof(header);
  nb = header.nbytes;
  sample = header.start_sample;
  if (debug)
    printf("%10d: start_sample %d, nb %d\n", where, sample, nb);
  if (0 && sample > 1000000) {
    if (debug)
      printf("%10d: discarding, bad sample\n", where);
    return(1);
    }
  if (read(fd, buf, nb) <= 0)
    return(0);

  p = (int8_t *)buf;
  for (sample_number = 0; nb > 0 && !abort_this_file; sample_number++) {
    if (skip_distance == 0 || ((sample_number % skip_distance) == 0))
      do_output = 1;
    else
      do_output = 0;
    switch (*p) {
      case LOG_TYPE_NEWVALUE:
        ++p;
        nb -= 3;
        where += 3;
        val = (uint8_t)(*p++);
        val = (val << 8) | (uint8_t)(*p++);
        if (debug)
          printf("%10d: new value %d\n", where, val);
        last_val = val;
        if (!discard && (do_output || nb == 0))
          pout(f, sample++, val);
        break;

      case LOG_TYPE_REPEAT:
        ++p;
        repeat_count = (uint8_t)(*p++);
        nb -= 2;
        where += 2;
        if (debug)
          printf("%10d: repeat %d times\n", where, repeat_count);
        for (int i = 0; i < repeat_count; i++) {
          if (!discard && (do_output || nb == 0))
            pout(f, sample++, last_val);
          }
        break;

      default:
        val = (uint16_t)((int)last_val + (int)(*p));
        if (debug)
          printf("%10d: Delta %d -> value %d\n", where, (int8_t)(*p), val);
        ++p;
        nb--;
        where++;
        if (!discard && (do_output || nb == 0))
          pout(f, sample++, val);
        last_val = val;
        break;
    }
  }
  return(1);
}



// buf needs to store 30 characters
int timespec2str(char *buf, uint len, const struct timespec *ts) {
    int ret;
    struct tm t;

    tzset();
    if (localtime_r(&(ts->tv_sec), &t) == NULL)
        return 1;

    ret = strftime(buf, len, "%F %T", &t);
    if (ret == 0)
        return 2;
    len -= ret - 1;

    ret = snprintf(&buf[strlen(buf)], len, ".%09ld", ts->tv_nsec);
    if (ret >= len)
        return 3;

    return 0;
}


struct tspsr {
  struct timespec time;
  struct pump_start_record data;
  } __attribute__((packed,aligned(4)));

int main()
{
  int fd;
  int i;
  FILE *f;
  char buf[256];
  char *prefix = "";
  struct timespec start, end;
  uint32_t num_samples;

  //printf("sizeof(struct log_header) is %d\n", sizeof(struct log_header));
  //exit(0);

  // Find the times file
  prefix = "";
  if (!(f = fopen("log_times.txt", "r"))) {
    prefix = "runtime/logs/test/";
    snprintf(fnbuf, sizeof(fnbuf), "%slog_times.txt", prefix);
    f = fopen(fnbuf, "r");
    if (f) {
      if ((i = fscanf(f, "%ld%ld%ld%ld%u", &start.tv_sec, &start.tv_nsec, &end.tv_sec, &end.tv_nsec, &num_samples)) == 5) {
        time_t secs = end.tv_sec - start.tv_sec;
        long nsecs = end.tv_nsec - start.tv_nsec;
        sample_period_usec = ((double)secs * 1e6 + (double)(nsecs / 1000)) / (double)num_samples;
        printf("num_samples %u, sample_period_usec %.4f\n", num_samples, sample_period_usec);
      } else {
        printf("fscanf returned %d, expecting 5\n", i);
      }
      fclose(f);
    }
  }

  for (i = 0; i < DIMENSION(fname); i++) {
    idx = i;
    prefix = "";
    snprintf(fnbuf, sizeof(fnbuf), "log_%s.bin", fname[i]);
    fd = open(fnbuf, O_RDONLY);
    if (fd < 0) {
      prefix = "runtime/logs/test/";
      snprintf(fnbuf, sizeof(fnbuf), "%slog_%s.bin", prefix, fname[i]);
      fd = open(fnbuf, O_RDONLY);
      if (fd < 0) {
        perror(buf);
        exit(1);
      }
    }

    snprintf(buf, sizeof(buf), "%slog_%s.dat", prefix, fname[i]);
    if (!(f = fopen(buf, "w"))) {
      perror(buf);
      exit(1);
    }

    if (debug)
      printf("Starting file %s\n", buf);
    where = 0;
    abort_this_file = 0;
    while (process_log(fd, f))
      if (abort_this_file)
        break;

    fclose(f);
    close(fd);
    }

  // Process pump start logs
  for (i = 0; i < DIMENSION(fname_pumpstart); i++) {
    idx = i;
    prefix = "";
    snprintf(fnbuf, sizeof(fnbuf), "log_%s.bin", fname_pumpstart[i]);
    fd = open(fnbuf, O_RDONLY);
    if (fd < 0) {
      prefix = "runtime/logs/test/";
      snprintf(fnbuf, sizeof(fnbuf), "%slog_%s.bin", prefix, fname_pumpstart[i]);
      fd = open(fnbuf, O_RDONLY);
      if (fd < 0) {
        //perror(fnbuf);
        continue;           // It's OK for the file not to exist
        }
      }

    struct tspsr *ts, *tsbase;

    int j, k, nb;
    int nalloc;
    int nrec;

    if (i == 0) {
      // Initial allocation
      nalloc = 16;
      tsbase = (struct tspsr *) calloc(nalloc, sizeof(struct tspsr));
      }

    // Read in all the start records
    nrec = 0;
    ts = tsbase;
    for (j = 0; ;j++) {
      nb = read(fd, (char *) ts, sizeof(struct tspsr));

      //printf("pumpstart: i %d, j %d: nb %d ts 0x%08lx\n", i, j, nb, (uint64_t) ts);
      if (nb < 0)
        perror("read");

      if (nb != sizeof(struct tspsr))
        break;

      if (j == nalloc-1) {
        nalloc += 16;
        ts = tsbase = (struct tspsr *) realloc(tsbase, nalloc * sizeof(struct tspsr));
        ts += j;
        }
      ts++;
      nrec++;
      }

      //printf("pumpstart: i %d: nrec %d\n", i, nrec);

      // Create the output file
      snprintf(buf, sizeof(buf), "%slog_%s.dat", prefix, fname_pumpstart[i]);
      if (!(f = fopen(buf, "w"))) {
        perror(buf);
        exit(1);
        }

      for (k = 0; k < FPS_RECORDS; k++) {
        fprintf(f, "%.2f", (float)k/0.4);
        for (j = 0, ts = tsbase; j < nrec; j++, ts++)
          fprintf(f, " %.4f", (float)(((uint32_t)ts->data.data[k] * 2900 * 5) / 4095)/1000.0);
        fprintf(f, "\n");
        }

      fclose(f);
      close(fd);

      // Make a GNUplot script
      if (!(f = fopen((i) ? "rpump.gpl" : "lpump.gpl", "w"))) {
        perror("gnuplot file");
        exit(1);
        }
      fprintf(f, "set xlabel \"Time (msec)\"\n");
      fprintf(f, "set ylabel \"Current (A)\"\n");
      fprintf(f, "set xtics 100\n");
      fprintf(f, "set ytics 2\n");
      fprintf(f, "set grid\n");
      fprintf(f, "plot [0:1000] [0:10] \\\n");
      for (k = 0, ts = tsbase; k < nrec; k++, ts++) {
        char tbuf[50];

        /*
         * struct tspsr is packed, so taking &ts->time directly can produce
         * an unaligned struct timespec pointer. Copy the member value into a
         * normally-aligned local object before passing it by address.
         */
        struct timespec record_time = ts->time;
        timespec2str(tbuf, sizeof(tbuf), &record_time);
        fprintf(f, "   '%s' using 1:%d title \"Pump start at %s\" with lines lw 2, \\\n", buf, k+2, tbuf);
        }
      fprintf(f, "\n\n");
      fprintf(f, "pause 9999 \"Cntrl-C to exit\"\n");

      fclose(f);
    }

  return(0);
}
