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
// VH-XVG load board command tool.
//
// This thing talks to the load board via the virtual comm port on
// the Nucleo ST-LINK interface, to allow run-time change of loading
// conditions while the system is under test.
//
// It uses the same framing protocol as the EFI power board interface
// uses, because I'm too lazy to invent something more fit for purpose.
//
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/select.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <sched.h>
#include <sys/mman.h>
#include <math.h>
#include <getopt.h>

#define __USE_GNU
#include <pthread.h>
#include <efi_protocol.h>

int verbose = 0;
int debug_level = 0;
int lfd = -1;
int do_pwm_command = 0;
int do_echo_command = 0;

static void do_load_transaction(int, struct efi_header *, struct efi_frame *);

#define DIMENSION(a) (sizeof(a)/sizeof(a[0]))

#define BAUDRATE B115200
#define LOAD0_DEFAULT "/dev/ttyACM0"             /* This is the debugger USB virtual comm port */

#define DEVSIZE 256

char load_device[DEVSIZE] = LOAD0_DEFAULT;

static struct termios saved_lfd_attributes;

static void restore_lfd_attributes(void) 
{
  if (lfd >= 0)
    tcsetattr(lfd,TCSANOW,&saved_lfd_attributes);
}

//
// CRC-32 calculator
//
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

static void efi_dump_frame(struct efi_header *h, char *t)
{
  printf("%8s: %02x %02x %02x %02x %08x %08x %08x\n",
    t,
    h->preamble,
    h->opcode,
    h->status,
    h->sequence,
    h->opdata1,
    h->opdata2,
    h->crc32);

}

//
// Get a frame, with timeouts
//

static int load_get_frame(int fd, struct efi_frame *h)
{
  int read_amount = sizeof(h->header);
  int total_amount;
  uint8_t *result = (uint8_t *)h;
  struct timeval timeout;
  ssize_t ret = 0;
  uint32_t fcrc;
  fd_set rd;

  // Read 1 byte until we get a valid preamble
  timeout.tv_sec = 0;
  do {
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    // if [[unlikely]] (ret < 0)
    //if (unlikely(ret < 0))
    if (ret < 0)
      {
      //applog(LOG_WARNING, "load: Error %d on select in efi_get_header", errno);
      return(ret);
      }
    if (!ret) {
      if (verbose)
        fprintf(stderr, "ERROR: Timeout waiting for preamble byte\n");
      return(0);                    // Timeout
      }
    ret = read(fd, result, 1);      // Read 1 byte
    if (ret <= 0)
      return(ret);                  // Error or timeout
    } while (*result != EFI_PREAMBLE);

  // Found a preamble byte, now read the rest of the frame header
  result++;
  read_amount--;
  do {
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (ret <= 0) {
      if (verbose)
        fprintf(stderr, "ERROR: Waiting on rest of header, ret %ld\n", ret);
      return(ret);                                  // Error or timeout
      }

    ret = read(fd, result, read_amount);
    //if (unlikely(ret < 0))
    if (ret < 0)
      {
      //applog(LOG_WARNING, "load: Error %d on read in efi_get_header", errno);
      if (verbose)
        fprintf(stderr, "ERROR: Read on rest of header, ret %ld against read_amount %d\n", ret, read_amount);
      return(ret);
      }
    result += ret;
    read_amount -= ret;
  } while (read_amount > 0);

  // Check the header CRC and status
  fcrc = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);
  if (fcrc != h->header.crc32)
    return(EBADHEADERCRC);

  if (h->header.status != 0)
    return(EBADHEADERSTS);

  // If there's a data frame, get it
  switch (h->header.opcode) {
    case OP_CURRENTS:
      //if ((h->header.opdata2 & OP_CURRENT_ALL_MASK) == 0)
      //  read_amount = sizeof(struct efi_currents) + 4;

      break;

    default:
      return(sizeof(h->header));            // No data, return
      break;
  }

  total_amount = sizeof(h->header);

  do {
    timeout.tv_usec = EFI_TIMEOUT_USEC;
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ret = select(fd+1, &rd, NULL, NULL, &timeout);
    if (ret <= 0) {
      if (verbose)
        fprintf(stderr, "ERROR: Select on getting data, ret %ld\n", ret);
      return(ret);                                  // Error or timeout
      }

    ret = read(fd, result, read_amount);
    //if (unlikely(ret < 0))
    if (ret < 0)
      {
      //applog(LOG_WARNING, "load: Error %d on read in efi_get_header", errno);
      if (verbose)
        fprintf(stderr, "ERROR: Read rest of data ret %ld, read_amount %d\n", ret, read_amount);
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
    return(EBADDATACRC);
    }

  return(total_amount - 4);
  }


void do_echo()
{
  struct efi_header header, *h = &header;
  struct efi_frame frame, *f = &frame;

  h->opcode = OP_ECHO;
  h->opdata1 = 0;
  h->opdata2 = 0;

  do_load_transaction(lfd, h, f);
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

static void efi_injector_test(int enable)
{
  struct efi_header header, *h = &header;
  struct efi_frame frame, *f = &frame;

  h->opcode = OP_TEST;
  if (enable)
    h->opdata1 = 0;
  else
    h->opdata1 = TEST_INJECTORS;      // Disable all injectors

  //h->opdata1 |= TEST_LOAD_PWM;
  //h->opdata2 = PWM_SPEC(30,1000);

  do_load_transaction(lfd, h, f);
}

//
// Do a transaction with the load box.
// opcode, opdata* fields filled in.
//
static void do_load_transaction(int fd, struct efi_header *h, struct efi_frame *f)
{
  static uint8_t sequence = 1;
  int n, sts;
  int i, j;

  h->preamble = EFI_PREAMBLE;
  h->sequence = sequence++;
  h->status = 0;
  h->crc32 = mo_crc32((unsigned char *)h, sizeof(struct efi_header) - 4, 0);

  if (verbose)
    efi_dump_frame(h, "TO");
  
  n = write(fd, (char *)h, sizeof(*h));
  if (n != sizeof(*h)) {
    // XXX
  }

  sts = load_get_frame(fd, f);
  if (sts > 0 && sts >= sizeof(*h)) {
    if (verbose) {
      efi_dump_frame(&f->header, "FROM");
      printf("\n");
      }
    if (f->header.opcode == OP_CURRENTS) {
      if (verbose)
        ;
        //efi_dump_currents(f);

      }
    } else {
      if (sts == 0)
        fprintf(stderr, "load: TIMEOUT\n");
      else {
        fprintf(stderr, "load: read returned sts %d\n", sts);
        perror("read");
        }
    }
}

#define LO_HELP                 257
#define LO_PWM                  258
#define LO_EFIDEV               259
#define LO_ECHO                 260


static struct option longopts[] =
{
 /* { *name             has_arg *flag val } */
    {"echo",              0, 0, LO_ECHO},
    {"help",              0, 0, LO_HELP},
    {"pwm",               1, 0, LO_PWM},
    {"device",            1, 0, LO_EFIDEV},

    {0, 0, 0, 0 }
};

static void usage(void) {
  struct option *l = longopts;

  printf("Usage:\n");
  printf("    load [flags] [-d <debugflags>] [-v]\n");
  printf("Flags:\n");
  for (; l->name; l++)
    printf("    --%s%s\n", l->name, (l->has_arg) ? " <arg>" : "");
}

static struct efi_header pwm_request = {0};
static struct efi_frame pwm_response = {0};

static void make_pwm_request(char *optarg)
{
  struct efi_header *h = &pwm_request;

  char *c, *b, *f;
  char ch;
  int i, n, low, high, val, pwm_frequency;
  double pwm_width;

  static char *pwm_target[] = {
    "Injector 0",
    "Injector 1",
    "Injector 2",
    "Injector 3",
    "Injector 4",
    "Injector 5",
    "Coilpack A",
    "Coilpack B",
    "Coilpack C"
    };

  h->opcode = OP_PWM;
  h->opdata1 = 0;
  h->opdata2 = 0;

  if (f = strchr(optarg, ':')) {
    b = optarg;
    *f++ = '\0';
    // Process injector, coilpack spec string
    while (b && *b) {
      //n = sscanf(b, "i%d-%d", &low, &high);
      //printf("n = %d, low %d, high %d\n", n, low, high);

      if (sscanf(b, "i%d-%d", &low, &high) == 2) {
        if (low >= 0 && high <= 5) {
          for (i = low; i <= high; i++)
            h->opdata1 |= (1<<i);
          //printf("h->opdata1 now 0x%x\n", h->opdata1);
        } else {
          usage();
          return;
        }
      }
      if (sscanf(b, "c%d-%d", &low, &high) == 2) {
        if (low >= 0 && high <= 2) {
          for (i = low; i <= high; i++)
            h->opdata1 |= (1<<(i+CYLINDERS));
        } else {
          usage();
          return;
        }
      }
      if (sscanf(b, "i%d", &val) == 1) {
        if (val >= 0 && val <= 5) {
            h->opdata1 |= (1<<val);
        } else {
          usage();
          return;
        }
      }
      if (sscanf(b, "c%d", &val) == 1) {
        if (val >= 0 && val <= 5) {
            h->opdata1 |= (1<<(val+CYLINDERS));
        } else {
          usage();
          return;
        }
      }
      if (c = strchr(b, ',')) {
        b = ++c;
      } else {
        b = NULL;
      }
    }
    // Process PWM value
    if (f && *f) {
      if (!strcmp(f, "off") || !strcmp(f, "OFF")) {
        h->opdata2 = 0;
      } else {
        if (c = strchr(f, '/')) {
          b = ++c;

        pwm_width = atof(f);                    // Width in msec
        pwm_frequency = strtoul(b, 0, 0);       // Frequency in Hz

        if (pwm_frequency == 0) {
          fprintf(stderr, "pwm frequency cannot be 0\n");
          usage();
          return;
        }

        val = 1000 / pwm_frequency;             // Period in msec
        if (val > 655) {
          fprintf(stderr, "ERROR: Can't do such a low PWM period\n");
          return;
        }
        if (pwm_width < 1.0 || val < 5 || pwm_width >= val) {
          fprintf(stderr, "ERROR: Bad pwm spec\n");
          return;
        }
        h->opdata2 = PWM_SPEC(((int)(pwm_width*100.0)),val*100);           // Timer counts are in 10 usec increments
      }
    }
  } else {
    usage();
  }

  if (verbose) {
    printf("Targets:\n");
    for (i = 0; i < (int)DIMENSION(pwm_target); i++) {
        if (h->opdata1 & (1<<i)) {
          printf("     %s\n", pwm_target[i]);
        }
      }
    printf("PWM width %d, PWM period %d\n", PWM_PULSE(h->opdata2), PWM_PERIOD(h->opdata2));
    }

  do_pwm_command = 1;
  }
}

int main(int argc, char *argv[])
{
  struct termios term;
  int tick = 0;
  int sts;
  int opt;

  int ms;

  while ((opt = getopt_long(argc, argv, "d:ep:v", longopts, (int *)0)) != -1) {
    switch(opt) {

      case 'd':
        debug_level = strtoul(optarg, 0, 0);
        break;

      case 'e':
      case LO_ECHO:
        do_echo_command = 1;
        break;

      case LO_HELP:
      case 'h':
        usage();
        return(0);
        break;

      case LO_PWM:
      case 'p':
        make_pwm_request(optarg);
        break;

      case LO_EFIDEV:
        strncpy(load_device, optarg, sizeof(load_device));
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

  //Open the serial port
  lfd = open(load_device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (lfd < 0) {
    fprintf(stderr, "Error opening Load device serial port %s\n", load_device);
    return -1;
    }
  else {
    fprintf(stderr, "Opened serial port %s OK\n", load_device);
  }

  sts = tcgetattr(lfd,&term);
  if (sts < 0) {
    fprintf(stderr, "Error %d (%s) from tcgetattr\n", errno, strerror(errno));
    return 1;
  }
  sts = tcgetattr(lfd,&saved_lfd_attributes);
  atexit(restore_lfd_attributes);
  cfmakeraw(&term);
  cfsetispeed(&term, BAUDRATE);
  cfsetospeed(&term, BAUDRATE);
  if (tcsetattr(lfd,TCSANOW,&term) != 0) {
    fprintf(stderr, "Error %d (%s) from tcsetattr\n", errno, strerror(errno));
    return 1;
  }
  //tcflush(lfd, TCIOFLUSH);

  // Do whatever was asked
  //do_command();
  //do_echo();
  if (do_pwm_command)
    do_load_transaction(lfd, &pwm_request, &pwm_response);

  if (do_echo_command)
    do_echo();

  restore_lfd_attributes();
  close(lfd);
  return 0;
}
