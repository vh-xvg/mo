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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>              
#include <stdbool.h>            

void HAL_SPI_Enable(void);
void HAL_SPI_Disable(void);
uint8_t HAL_SPI_Write(uint8_t data);
uint8_t HAL_SPI_WriteByte(uint8_t data);
void HAL_SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length);
int HAL_SPI_ReadBuffer(uint8_t *Buffer, uint32_t Length);
void HAL_Delay(uint32_t milliSeconds);
int  HAL_Eve_Reset_HW(void);
void HAL_Close(void);

#ifdef __cplusplus
}
#endif

