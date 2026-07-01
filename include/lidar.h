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

#ifndef __lidar_h_
#define __lidar_h_

// LIDAR-Lite default I2C device address
#define LIDARLITE_ADDR_DEFAULT 0x62

// LIDAR-Lite internal register addresses
#define LLv3_ACQ_CMD       0x00
#define LLv3_STATUS        0x01
#define LLv3_SIG_CNT_VAL   0x02
#define LLv3_ACQ_CONFIG    0x04
#define LLv3_DISTANCE      0x0f
#define LLv3_REF_CNT_VAL   0x12
#define LLv3_UNIT_ID_HIGH  0x16
#define LLv3_UNIT_ID_LOW   0x17
#define LLv3_I2C_ID_HIGH   0x18
#define LLv3_I2C_ID_LOW    0x19
#define LLv3_I2C_SEC_ADR   0x1a
#define LLv3_THRESH_BYPASS 0x1c
#define LLv3_I2C_CONFIG    0x1e
#define LLv3_COMMAND       0x40
#define LLv3_CORR_DATA     0x52
#define LLv3_ACQ_SETTINGS  0x5d

int       lidar_i2c_init    (void);
int       lidar_i2c_connect (uint8_t);
void      lidar_configure   (uint8_t, uint8_t);
void      lidar_setI2Caddr  (uint8_t, uint8_t, uint8_t);
uint16_t  lidar_readDistance(uint8_t);
void      lidar_waitForBusy (uint8_t);
uint8_t   lidar_getBusyFlag (uint8_t);
void      lidar_takeRange   (uint8_t);
uint32_t  lidar_i2cWrite    (uint8_t, uint8_t *, uint8_t, uint8_t);
int       lidar_i2cRead     (uint8_t, uint8_t *, uint8_t, uint8_t);
void      lidar_correlationRecordRead (uint16_t *, uint16_t, uint8_t);
uint8_t   lidar_getStatus(uint8_t);
void 	  lidar_set_threshold_bypass(uint8_t, uint8_t);

#endif
