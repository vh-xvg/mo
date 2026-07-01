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

#ifndef AD5592_H
#define AD5592_H

typedef struct {
  int spi_fd;
  int freq;
  } ad5592_cxt_t;

#define NOP                     0
#define DAC_RD                  1
#define ADC_SEQ                 2
#define GEN_CTRL_REG            3
#define ADC_CONFIG              4
#define DAC_CONFIG              5
#define CONFIG_READ_AND_LDAC    7
#define PD_REF_CTRL             0xb

#define CMD_SHIFT               11

// ADC FS is 2.5 (Vref) * 2
// Resistive divider is a further * 2, to give a range of 0 - 10V.
#define ADC_VOLTS(x)   (((double)(x & 0xfff)) * 2.5 * 2 * 2 / 0xfff)

#define DAC_BASE_CHANNEL        4

int ad5592_main(void);


#endif
