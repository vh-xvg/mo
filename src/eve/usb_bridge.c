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


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "efi_protocol.h"

extern void dbprintf(const char *, ...);
//#define dbprintf(...) debugPrintf(__VA_ARGS__)

int debug_ft = 0;

/* Based on example by bjorn vaktaren at
 * https://gist.github.com/bjornvaktaren/d2461738ec44e3ad8b3bae4ce69445b4 */
#include <libftdi1/ftdi.h>
#include <stdbool.h>

#define BUS_SK 0x01 // ADBUS0, SPI data clock
#define BUS_DO 0x02 // ADBUS1, SPI data out
#define BUS_DI 0x04 // ADBUS2, SPI data in
#define BUS_CS 0x08 // ADBUS3, SPI chip select
#define BUS_L0 0x10 // ADBUS4, general-ourpose i/o, GPIOL0
#define BUS_L1 0x20 // ADBUS5, general-ourpose i/o, GPIOL1
#define BUS_L2 0x40 // ADBUS6, general-ourpose i/o, GPIOL2
#define BUS_L3 0x80 // ADBUS7, general-ourpose i/o, GPIOL3

#define FT800_RST BUS_L3
// Set these pins high
#define pinInitialState (BUS_CS | BUS_L0 | BUS_L1 | FT800_RST)
#define pinDirection (BUS_SK | BUS_DO | BUS_CS | BUS_L0 | BUS_L1 | FT800_RST)

void HAL_Delay(uint32_t);

struct ftdi_context *ftdi = NULL;

void HAL_Close(void)
{
  if (debug_ft)
    printf("Closing bridge\n");
  HAL_Delay(200);
  if (ftdi)
  {
    ftdi_usb_close(ftdi);
  }
}

void HAL_RST_Enable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState & ~FT800_RST;
  buf[icmd++] = pinDirection;
  if (debug_ft)
    printf("HAL_RST_Enable  : write 0x%02x %02x %02x\n", buf[0], buf[1], buf[2]);
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_RST_Enable write failed\n");
  }
}

void HAL_RST_Disable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState | FT800_RST;
  buf[icmd++] = pinDirection;
  if (debug_ft)
    printf("HAL_RST_Disable : write 0x%02x %02x %02x\n", buf[0], buf[1], buf[2]);
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_RST_Disable write failed\n");
  }
}

void HAL_SPI_Enable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState & ~BUS_CS;
  buf[icmd++] = pinDirection;
  if (debug_ft)
    printf("HAL_SPI_Enable  : write 0x%02x %02x %02x\n", buf[0], buf[1], buf[2]);
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Enable write failed\n");
  }
}

void HAL_SPI_Disable(void)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = SET_BITS_LOW;
  buf[icmd++] = pinInitialState | BUS_CS;
  buf[icmd++] = pinDirection;
  if (debug_ft)
    printf("HAL_SPI_Disable : write 0x%02x %02x %02x\n\n", buf[0], buf[1], buf[2]);
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Disable write failed\n");
  }
}

void HAL_Recover_SPI(void)
{
    uint8_t buf[16];
    int n = 0;

    ftdi_tciflush(ftdi);

    buf[n++] = SET_BITS_LOW;
    buf[n++] = pinInitialState | BUS_CS;   // CS high
    buf[n++] = pinDirection;

    buf[n++] = SEND_IMMEDIATE;

    ftdi_write_data(ftdi, buf, n);
    mo_usleep(10000);
    ftdi_tciflush(ftdi);
}


uint8_t HAL_SPI_Write(uint8_t data)
{
  int icmd = 0;
  uint8_t buf[8];
  buf[icmd++] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
  buf[icmd++] = 0x00; // length low byte, 0x0000 ==> 1 byte
  buf[icmd++] = 0x00; // length high byte
  buf[icmd++] = data; // byte to send
  if (debug_ft)
    printf("HAL_SPI_Write   : write 0x%02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3]);
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Write failed\n");
  }
  return 0;
}

void HAL_SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length)
{
  int icmd = 0;

  if (Length == 0)
    {
    printf("ERROR: Call to HAL_SPI_WriteBuffer with Length 0\n");
    return;
    }

  uint8_t *buf = malloc(Length + 16);

  buf[icmd++] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
  buf[icmd++] = (Length - 1) & 0xff;
  buf[icmd++] = ((Length - 1) >> 8) & 0xff; // length high byte
  memcpy(&buf[icmd], Buffer, Length);
  icmd += Length;
  if (debug_ft)
    printf("HAL_SPI_WriteBuf: write 0x%02x %02x %02x %02x %02x... (%d)\n", buf[0], buf[1], buf[2], buf[3], buf[4], Length);
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    printf("HAL_SPI_Write failed\n");
  }
  free(buf);
}

static int ftdi_read_exact(uint8_t *buf, int len)
{
    int got = 0;
    int idle = 0;

    while (got < len) {
        int n = ftdi_read_data(ftdi, buf + got, len - got);

        if (n > 0) {
            got += n;
            idle = 0;
            continue;
        }

        if (++idle > 50)
            return got;   // timeout/short read

        mo_usleep(1000);
    }

    return got;
}

int rdxx(uint32_t address, uint32_t *buf, uint32_t length)
{
    uint8_t cmd[64];
    int n = 0;

    uint8_t *rx = (uint8_t *)buf;

    // CS low
    cmd[n++] = SET_BITS_LOW;
    cmd[n++] = pinInitialState & ~BUS_CS;
    cmd[n++] = pinDirection;

    // 3-byte BT815 read address
    uint8_t addr[4];
    addr[0] = (address >> 16) & 0x3f;
    addr[1] = (address >> 8) & 0xff;
    addr[2] = address & 0xff;
    addr[3] = 0x00;   // BT815 dummy byte

    cmd[n++] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
    cmd[n++] = 4 - 1;
    cmd[n++] = 0;
    memcpy(&cmd[n], addr, 4);
    n += 4;

    // Read bytes. For SPI mode 0, try plain MPSSE_DO_READ first.
    cmd[n++] = MPSSE_DO_READ;
    cmd[n++] = (length - 1) & 0xff;
    cmd[n++] = ((length - 1) >> 8) & 0xff;

    cmd[n++] = SEND_IMMEDIATE;

    if (ftdi_write_data(ftdi, cmd, n) != n)
        return -1;

    int got = ftdi_read_exact(rx, length);

    // CS high, after read has completed
    uint8_t cs_hi[3];
    cs_hi[0] = SET_BITS_LOW;
    cs_hi[1] = pinInitialState | BUS_CS;
    cs_hi[2] = pinDirection;
    ftdi_write_data(ftdi, cs_hi, 3);

    return got;
}

int HAL_SPI_ReadBuffer(uint8_t *Buffer, uint32_t Length)
{
  int icmd = 0;
  uint8_t buf[256];

  HAL_SPI_Write(0);

  //buf[icmd++] = MPSSE_WRITE_NEG | MPSSE_DO_READ;
  buf[icmd++] = MPSSE_DO_READ;
  buf[icmd++] = (Length - 1) & 0xff;
  buf[icmd++] = ((Length - 1) >> 8) & 0xff; // length high byte
  buf[icmd++] = SEND_IMMEDIATE;
  if (debug_ft)
  //if (Length >= 16)
    printf("HAL_SPI_Readbuf :    write 0x%02x %02x %02x %02x (len %d)\n", buf[0], buf[1], buf[2], buf[3], Length);
  if (ftdi_write_data(ftdi, buf, icmd) != icmd)
  {
    dbprintf("HAL_SPI_ReadBuffer ftdi_write_data failed\n");
    return(-1);
  }

  int res;
  //res = ftdi_read_data(ftdi, Buffer, Length);
  res = ftdi_read_exact(Buffer, Length);
  if (debug_ft)
  //if (Length >= 16)
    printf("HAL_SPI_Readbuf :                           read 0x%02x %02x %02x %02x (res %d)\n", Buffer[0], Buffer[1], Buffer[2], Buffer[3], res);
  if (res != Length) {
    dbprintf("HAL_SPI_ReadBuffer failed, Length %d res %d\n", Length, res);
  }
  return(res);
}

void HAL_Delay(uint32_t milliSeconds)
{
  if (debug_ft)
    printf("HAL_Delay       :    %d msec\n", milliSeconds);
  mo_usleep(milliSeconds * 1000);
}

int HAL_Eve_Reset_HW(void)
{
  static int consecutive_errors = 0;

  ftdi = ftdi_new();
  if (!ftdi) {
    dbprintf("TFT: Failed to initialize USB bridge\n");
    return -1;
    }
  //dbprintf("After ftdi_new(), ftdi->usb_read_timeout is %d\n", ftdi->usb_read_timeout);

  int ftdi_status = ftdi_usb_open(ftdi, 0x1b3d, 0x200);
  if (ftdi_status != 0) {
    if (++consecutive_errors < 3)
      dbprintf("TFT: Can't open USB bridge, error %s\n", ftdi_get_error_string(ftdi));
    return -1;
    }
  //dbprintf("After ftdi_usb_open(), ftdi->usb_read_timeout is %d\n", ftdi->usb_read_timeout);
  //dbprintf("Bridge opened successfully!\n");
  consecutive_errors = 0;
  ftdi_usb_reset(ftdi);
  ftdi_set_interface(ftdi, INTERFACE_ANY);
  ftdi_set_bitmode(ftdi, 0, 0);
  ftdi_set_bitmode(ftdi, 0, BITMODE_MPSSE);
  ftdi_set_latency_timer(ftdi, 2);                  // Change from default of 16ms to 2ms
  ftdi_read_data_set_chunksize(ftdi, 4096);         // These are the defaults anyway...
  ftdi_write_data_set_chunksize(ftdi, 4096);
  ftdi_tcioflush(ftdi);
  mo_usleep(200000);

  unsigned int icmd = 0;
  unsigned char buf[32] = {0};
  buf[icmd++] = DIS_DIV_5;      // opcode:   disable prescaler, make 60 MHz rather than 12 MHz
  // OR... buf[icmd++] = EN_DIV_5;       // opcode:   enable prescaler, make 12 MHz rather than 60 MHz
  buf[icmd++] = TCK_DIVISOR;    // opcode:   set clk divisor
  buf[icmd++] = 0x05;           // argument: low bit. 60 MHz / (2 * (5+1)) = around 5 MHz
  //buf[icmd++] = 29;           // argument: low bit. 60 MHz / 2*(29+1) = around 1 MHz
  buf[icmd++] = 0;              // argument: high bits.
  buf[icmd++] = DIS_ADAPTIVE;    // opcode:   disable adaptive clocking
  buf[icmd++] = DIS_3_PHASE;     // opcode:   disable 3-phase clocking
  buf[icmd++] = SET_BITS_LOW;    // opcode:   set low bits (ADBUS[0-7])
  buf[icmd++] = pinInitialState; // argument: inital pin states
  buf[icmd++] = pinDirection;    // argument: pin direction
  if (ftdi_write_data(ftdi, buf, icmd) != icmd) {
    dbprintf("TFT: Bridge setup failed, error %s\n", ftdi_get_error_string(ftdi));
    ftdi_usb_close(ftdi);
    return -1;
    }
  //dbprintf("Setup complete!\n");
  HAL_RST_Enable();
  HAL_Delay(20);
  HAL_RST_Disable();
  HAL_Delay(20);

  return(0);
}


int eve_open(void) {
  return(1);
}
