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


#include <stdint.h>
#include <limits.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "ad5592.h"
#include "efi_protocol.h"

#define SWAP_BYTES(x)           ((((x)>>8)&0xff) | (((x)<<8)&0xff00))

// Set this to up to 4, depending on how many channels are in use
// Only ADC0 is in use right now, for air/fuel ratio input
#define ADC_ACTIVE_CHANNELS     1

extern uint8_t hat2_noise;
extern float air_fuel_ratio;
extern float air_fuel_ratio_correction;
extern uint8_t o2_sensor_not_ready;
extern uint8_t o2_sensor_error;
extern uint8_t  o2_sensor_power_is_on;

extern int local_fake_data_ok;

// Use these externally to do a DAC operation
uint8_t write_dac[2] = {0,0};                      // 0 == not busy
uint16_t dac_data[2] = {0x0, 0x0};

static ad5592_cxt_t ad5592_context;

static inline int ipow(int x, int n)
{
  int res = 1;

  for (int i = 0; i < n; ++i)
    res *= x;

  return(res);
}

static int ad5592_init(ad5592_cxt_t *c, const char *spi_device, uint32_t spi_freq)
{
  struct spi_ioc_transfer init[1];
  char buf[8];
  uint16_t *pbuf = (uint16_t *) buf;
  int sts;

  if ((c->spi_fd = open(spi_device, O_RDWR)) < 0)
    {
    dbprintf("ERROR: Can't open %s (%s)\n", spi_device, strerror(errno));
    return -1;
    }

  // Mode bits: SPI_LOOP SPI_CPHA SPI_CPOL SPI_LSB_FIRST SPI_CS_HIGH SPI_3WIRE SPI_NO_CS SPI_READY
  uint32_t spi_mode = SPI_CPOL;
  if (ioctl(c->spi_fd, SPI_IOC_WR_MODE32, &spi_mode) < 0) {
    dbprintf("ad5592_init: Failed to set SPI mode: %i: %s\n", errno, strerror(errno));
    return -2;
  }

  uint8_t spi_bits = 8;
  if (ioctl(c->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) < 0) {
    dbprintf("ad5592_init: Failed to set SPI bits per word: %i: %s\n", errno, strerror(errno));
    return -3;
  }

  if (ioctl(c->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_freq) < 0) {
    dbprintf("ad5592_init: Failed to set SPI speed: %i: %s\n", errno, strerror(errno));
    return -4;
  }

  // Enable internal reference
  memset(init, 0, sizeof init);
  memset(buf, 0, sizeof buf);
  init[0].tx_buf = (unsigned long) buf;
  init[0].len = 2;
  *pbuf = SWAP_BYTES((PD_REF_CTRL << CMD_SHIFT) | 0x200);       // bit 9 = power up internal reference

  sts = ioctl(c->spi_fd, SPI_IOC_MESSAGE(1), init);
  if (sts < 0) {
    perror("SPI_IOC_MESSAGE");
    return -1;
    }

  // Set ADC range to be 2 * Vref
  memset(init, 0, sizeof init);
  memset(buf, 0, sizeof buf);
  init[0].tx_buf = (unsigned long) buf;
  init[0].len = 2;
  *pbuf = SWAP_BYTES((GEN_CTRL_REG << CMD_SHIFT) | 0x300 | 0x20 | 0x10);    // bit 9 = Enable ADC Buffer precharge
                                                                            // bit 8 = Enable ADC buffer
                                                                            // bit 5 = Set ADC gain as 0 to 2 x Vref
                                                                            // bit 4 = Set DAC range as 0 to 2 * Vreg

  sts = ioctl(c->spi_fd, SPI_IOC_MESSAGE(1), init);
  if (sts < 0) {
    perror("SPI_IOC_MESSAGE");
    return -1;
    }

  // Configure channels 0-3 as ADC's
  uint16_t channel_mask = ipow(2, ADC_ACTIVE_CHANNELS) - 1;

  memset(init, 0, sizeof init);
  memset(buf, 0, sizeof buf);
  init[0].tx_buf = (unsigned long) buf;
  init[0].len = 2;
  *pbuf = SWAP_BYTES((ADC_CONFIG << CMD_SHIFT) | channel_mask);

  sts = ioctl(c->spi_fd, SPI_IOC_MESSAGE(1), init);
  if (sts < 0) {
    perror("SPI_IOC_MESSAGE");
    return -1;
  }


  // Configure channels 4-5 as DAC's
  *pbuf = SWAP_BYTES((DAC_CONFIG << CMD_SHIFT) | 0x30);

  sts = ioctl(c->spi_fd, SPI_IOC_MESSAGE(1), init);
  if (sts < 0) {
    perror("SPI_IOC_MESSAGE");
    return -1;
  }

  // Configure ADC sequence, channels 0 up to 3, enable repetition
  *pbuf = SWAP_BYTES((ADC_SEQ << CMD_SHIFT) | (1 << 9) | channel_mask);
  sts = ioctl(c->spi_fd, SPI_IOC_MESSAGE(1), init);
  if (sts < 0) {
    perror("SPI_IOC_MESSAGE");
    return -1;
  }

  dbprintf("ad5592_init completed successfully\n");

  return(0);
}

int ad5592_main()
{
  ad5592_cxt_t *c = &ad5592_context;
  static int done_init = 0;
  static uint8_t dac_channel = 0;
  static float o2_emaval;            // Exponential moving average value


  int adc;
  struct spi_ioc_transfer xfer[2];
  char buf[16];
  uint16_t *pbuf = (uint16_t *) buf;
  int sts;

  if (!done_init) {
    memset(&ad5592_context, 0, sizeof(ad5592_context));
    if (ad5592_init(&ad5592_context, "/dev/spidev0.0", 500000) < 0)
      return(-1);
    done_init = 1;

#if 0
    // Read back register 2
    memset(xfer, 0, sizeof xfer);
    memset(buf, 0xff, sizeof buf);
    *pbuf = SWAP_BYTES((CONFIG_READ_AND_LDAC << CMD_SHIFT) | 0x40 | (0x4 << 2));      // 0x40 = readback, 0x2 = reg to read
    xfer[0].tx_buf = (unsigned long) buf;
    xfer[0].len = 2;
    xfer[1].rx_buf = (unsigned long) buf;
    xfer[1].len = 2;
    sts = ioctl(c->spi_fd, SPI_IOC_MESSAGE(2), xfer);
    if (sts < 0) {
      perror("SPI_IOC_MESSAGE");
      return -1;
      }
    uint16_t val = SWAP_BYTES(*pbuf);
    dbprintf("REG2 readback, value 0x%04x\n", val);
    char *p = buf;
    dbprintf("chars: 0x%02x %02x %02x %02x %02x %02x %02x %02x\n",
        *(p), *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
    return(0);
#endif
  }

  // Read ADC's back
  for (adc = 0; adc < ADC_ACTIVE_CHANNELS; adc++) {
    memset(xfer, 0, sizeof xfer);
    memset(buf, 0xff, sizeof buf);

    // Switch if only one channel is to be written, to make sure we pick it up
    if (write_dac[0] == 0 && write_dac[1] == 1)
      dac_channel = 1;
    else if (write_dac[0] == 1 && write_dac[1] == 0)
      dac_channel = 0;

    if (write_dac[dac_channel]) {
      write_dac[dac_channel] = 0;
      *pbuf = SWAP_BYTES(0x8000 | ((dac_channel + DAC_BASE_CHANNEL) << 12) | (dac_data[dac_channel] & 0xfff));
      dac_channel = (dac_channel) ? 0 : 1;      // Flip channel
    } else {
      *pbuf = SWAP_BYTES((NOP << CMD_SHIFT));
    }
    xfer[0].tx_buf = (unsigned long) buf;
    xfer[0].len = 2;
    xfer[0].rx_buf = (unsigned long) buf;
    //xfer[1].len = 2;
    sts = ioctl(c->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (sts < 0) {
      perror("SPI_IOC_MESSAGE");
      return -1;
      }
    uint16_t val = SWAP_BYTES(*pbuf);
    if (adc == 0) {
      float volts = ADC_VOLTS(val);
      #define ALPHA ((float)0.2)         /* exponential moving average smoothing value */

      o2_emaval = volts * ALPHA + o2_emaval * ((float)(1.0 - ALPHA));

      if (local_fake_data_ok) {
        air_fuel_ratio = 12.5;
        o2_sensor_not_ready = 0;
        o2_sensor_error = 0;
        o2_sensor_power_is_on = 1;
      } else {
        // From AEM manual
        air_fuel_ratio = (7.3125 + 2.375 * o2_emaval) + air_fuel_ratio_correction;
        if (volts < 0.5) {
          o2_sensor_not_ready = 1;
          air_fuel_ratio = 0.0;
          }
        else
          o2_sensor_not_ready = 0;
        if (volts > 4.5) {
          o2_sensor_error = 1;
          air_fuel_ratio = 0.0;
          }
        else
          o2_sensor_error = 0;
        if (hat2_noise)
          dbprintf("ADC readback %d, chan %d value %d (0x%x) (%.2f Volts, ema %.2f) (not_ready %d sensor_error %d)\n",
              adc, (val >> 12) & 0x7, val & 0xfff, val & 0xfff, volts, o2_emaval, o2_sensor_not_ready, o2_sensor_error);
        }
      }
  }

  return(0);
}
