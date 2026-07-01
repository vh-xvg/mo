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

// Eve2 Processor Agnostic Library (Condensed)
//
// This "library" consists of the files "Eve2_81x.c" and "Eve2_81x.h".
//
// In persuit of the common goal of simplicity and understandability I find that I am unable to
// make function prototypes that match Bridgetek example code.  I draw the line between the 
// Eve and all other hardware. The library is "clean" and includes no abstraction at all, unlike 
// much of the example code on the Internet which is sort of application and abstraction mixed 
// together in a confusing abuse of my eye-holes.  
// My intent is to be as straight forward and understandable as possible, so while function 
// names and parameter lists are different than Bridgetek code examples, they should be easily 
// recognizable.  I have also made every attempt to reference Bridgetek documentation against 
// the code to act as a translation to help in understanding.

// Notes on the operation of the Eve command processing engine - THE FIFO
//
// First be aware that the FTDI documentation variously refers to you as "User", "MCU", "Host".
// 
// The FIFO, like all FIFO's needs pointers to indicate the starting address of buffered data and
// the end address of buffered data.  There is wrapping involved, but the basic idea is clear.
// Eve takes data into it's FIFO using a fully defined write operation to a memory address - that 
// is, you need to take care of the wrapping - to you, it is not a FIFO - it is a piece of memory.
// Eve keeps track of it's own read address location, but relies on you to write the address
// of the end of buffered data.
// 
// So as commands are loaded into RAM - into the FIFO space - Eve will do nothing in response.
// Eve is happy to take your data and store it for you while it sits with it's read address and 
// write address set to the same value.  Once the commands are loaded, the next available address
// is manually written (by you) to the register in which Eve stores the FIFO write pointer
// (REG_CMD_WRITE).  
//
// Following this, Eve discovers that the addresses are different and begins processing commands while
// updating it's own read pointer until the read and write pointers are the same.
// 
// Be aware that Eve stores only the offset into the "FIFO" as 16 bits, so any use of the offset 
// requires adding the base address (RAM_CMD 0x308000) to the resultant 32 bit value.

#include <stdio.h>
#include <stdint.h>              // Find integer types like "uint8_t"  
#include <stdbool.h>			 // for true/false
#include <unistd.h>
#include "eve_lib.h"            // Header for this file with prototypes, defines, and typedefs
#include "eve_config.h"         // Header for display selection 
#include "hw_api.h"		// for spi abstraction 
#include <efi_protocol.h>

#define WorkBuffSz 512
#define Log printf

extern void die_and_restart(void);
extern int rdxx(uint32_t, uint32_t *, uint32_t );       // This is in usb_bridge.c now

// Global Variables 
uint16_t FifoWriteLocation = 0;
char LogBuf[WorkBuffSz];         // The singular universal data array used for all things including logging

// Call this function once at powerup to reset and initialize the Eve chip
int BT81x_Init(void)
{  
  uint32_t Ready = false;
  static uint32_t loops = 0;
  
  //dbprintf("Eve_Reset\n");
  if (Eve_Reset())               // Hard reset of the Eve chip
    return -1;

  // Wakeup Eve
#if defined(EVE3)
  //printf("HCMD_CLKEXT\n");
  HostCommand(HCMD_CLKEXT);
  //printf("HCMD_CORERESET\n");
  HostCommand(HCMD_CORERESET);
  HAL_Delay(200);
#endif
  //printf("HCMD_ACTIVE\n");
  HostCommand(HCMD_ACTIVE);
  HAL_Delay(300);
  
  //printf("Cmd_READ_REG_ID\n");
  for (loops = 0; !Ready && loops < 5; loops++)
  {
    Ready = Cmd_READ_REG_ID();
  }
  if (!Ready) {
    fprintf(stderr, "EVE not ready after %d retries\n", loops+1);
    exit(1);
  }

  Ready = rd32(REG_CHIP_ID);
  uint16_t ValH = Ready >> 16;
  uint16_t ValL = Ready & 0xFFFF;
  dbprintf("Chip ID = 0x%04x%04x, FLASH size %d KB\n", ValH, ValL, rd32(REG_FLASH_SIZE + RAM_REG) / 1024);

  wr32(REG_FREQUENCY + RAM_REG, 0x3938700); // Configure the system clock to 60MHz

  // Before we go any further with Eve, it is a good idea to check to see if she is wigging out about something 
  // that happened before the last reset.  If Eve has just done a power cycle, this would be unnecessary.
  if( rd16(REG_CMD_READ + RAM_REG) == 0xFFF )
  {
    // Eve is unhappy - needs a paddling.
    uint32_t Patch_Add = rd32(REG_COPRO_PATCH_PTR + RAM_REG);
    wr8(REG_CPU_RESET + RAM_REG, 1);
    wr16(REG_CMD_READ + RAM_REG, 0);
    wr16(REG_CMD_WRITE + RAM_REG, 0);
    wr16(REG_CMD_DL + RAM_REG, 0);
    wr8(REG_CPU_RESET + RAM_REG, 0);
    wr32(REG_COPRO_PATCH_PTR + RAM_REG, Patch_Add);
  }
  
  // turn off screen output during startup
  wr8(REG_GPIOX + RAM_REG, 0);             // Set REG_GPIOX to 0 to turn off the LCD DISP signal
  wr8(REG_PCLK + RAM_REG, 0);              // Pixel Clock Output disable

  // load parameters of the physical screen to the Eve
  // All of these registers are 32 bits, but most bits are reserved, so only write what is actually used
  wr16(REG_HCYCLE + RAM_REG, HCYCLE);         // Set H_Cycle to 548
  wr16(REG_HOFFSET + RAM_REG, HOFFSET);       // Set H_Offset to 43
  wr16(REG_HSYNC0 + RAM_REG, HSYNC0);         // Set H_SYNC_0 to 0
  wr16(REG_HSYNC1 + RAM_REG, HSYNC1);         // Set H_SYNC_1 to 41
  wr16(REG_VCYCLE + RAM_REG, VCYCLE);         // Set V_Cycle to 292
  wr16(REG_VOFFSET + RAM_REG, VOFFSET);       // Set V_OFFSET to 12
  wr16(REG_VSYNC0 + RAM_REG, VSYNC0);         // Set V_SYNC_0 to 0
  wr16(REG_VSYNC1 + RAM_REG, VSYNC1);         // Set V_SYNC_1 to 10
  wr8(REG_SWIZZLE + RAM_REG, SWIZZLE);        // Set SWIZZLE to 0
  wr8(REG_PCLK_POL + RAM_REG, PCLK_POL);      // Set PCLK_POL to 1
  wr16(REG_HSIZE + RAM_REG, HSIZE);           // Set H_SIZE to 480
  wr16(REG_VSIZE + RAM_REG, VSIZE);           // Set V_SIZE to 272
  wr8(REG_CSPREAD + RAM_REG, CSPREAD);        // Set CSPREAD to 1    (32 bit register - write only 8 bits)
  wr8(REG_DITHER + RAM_REG, DITHER);          // Set DITHER to 1     (32 bit register - write only 8 bits)

  // configure touch & audio
#ifdef TOUCH_RESISTIVE
  wr16(REG_TOUCH_CONFIG + RAM_REG, 0x8381);
#elif defined TOUCH_CAPACITIVE
  // Settings for the Goodix touch controller
  wr16(REG_TOUCH_CONFIG + RAM_REG, 0x5d0);
#endif

  wr16(REG_TOUCH_RZTHRESH + RAM_REG, 1200);          // set touch resistance threshold
  wr8(REG_TOUCH_MODE + RAM_REG, 0x02);               // set touch on: continous - this is default
  wr8(REG_TOUCH_ADC_MODE + RAM_REG, 0x01);           // set ADC mode: differential - this is default
  wr8(REG_TOUCH_OVERSAMPLE + RAM_REG, 15);           // set touch oversampling to max

#if defined TOUCH_CAPACITIVE
  wr32(REG_CTOUCH_MODE + RAM_REG, 0x3);             // Continuous mode
  wr32(REG_CTOUCH_EXTEND + RAM_REG, 0x0);           // Enable extended mode
#endif

  wr16(REG_GPIOX_DIR + RAM_REG, 0x8000 | ENABLE_HAPTIC_GPIO3_DIR);    // Set Disp GPIO Direction 
  wr16(REG_GPIOX + RAM_REG, 0x8000 | DISABLE_HAPTIC);                 // Enable Disp (if used)

  wr16(REG_PWM_HZ + RAM_REG, 0x00FA);                // Backlight PWM frequency
  wr8(REG_PWM_DUTY + RAM_REG, 128);                  // Backlight PWM duty (on)   

  //Cmd_SetRotate(0);

  // write first display list (which is a clear and blank screen)
  wr32(RAM_DL+0, CLEAR_COLOR_RGB(0,0,0));
  wr32(RAM_DL+4, CLEAR(1,1,1));
  wr32(RAM_DL+8, DISPLAY());
  wr8(REG_DLSWAP + RAM_REG, DLSWAP_FRAME);          // swap display lists
  wr8(REG_PCLK + RAM_REG, PCLK);                       // after this display is visible on the LCD

  return(0);
}

// Reset Eve chip via the hardware PDN line
int Eve_Reset(void)
{
  if (HAL_Eve_Reset_HW())
    return(-1);

  wr8(REG_CPU_RESET + RAM_REG, 1);
  wr8(REG_CMD_READ + RAM_REG, 0);
  wr8(REG_CMD_WRITE + RAM_REG, 0);
  wr8(REG_CMD_DL + RAM_REG, 0);
  wr8(REG_CPU_RESET + RAM_REG, 0);

  return(0);

}

// *** Host Command - BT81X Embedded Video Engine Datasheet - 4.1.5 **********************************************
// Host Command is a function for changing hardware related parameters of the Eve chip.  The name is confusing.
// These are related to power modes and the like.  All defined parameters have HCMD_ prefix
void HostCommand(uint8_t HCMD) 
{

  HAL_SPI_Enable();
  
/*  HAL_SPI_Write(HCMD | 0x40); // In case the manual is making you believe that you just found the bug you were looking for - no. */       
  HAL_SPI_Write(HCMD);        
  HAL_SPI_Write(0x00);          // This second byte is set to 0 but if there is need for fancy, never used setups, then rewrite.  
  HAL_SPI_Write(0x00);   
  
  HAL_SPI_Disable();
}

// *** Eve API Reference Definitions *****************************************************************************
// BT81X Embedded Video Engine Datasheet 1.3 - Section 4.1.4, page 16
// These are all functions related to writing / reading data of various lengths with a memory address of 32 bits
// ***************************************************************************************************************
void wr32(uint32_t address, uint32_t parameter)
{
  HAL_SPI_Enable();
  
#if 0
  HAL_SPI_Write((uint8_t)((address >> 16) | 0x80));   // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  HAL_SPI_Write((uint8_t)(address >> 8));             // Next byte of the register address   
  HAL_SPI_Write((uint8_t)address);                    // Low byte of register address - usually just the 1 byte offset
  
  HAL_SPI_Write((uint8_t)(parameter & 0xff));         // Little endian (yes, it is most significant bit first and least significant byte first)
  HAL_SPI_Write((uint8_t)((parameter >> 8) & 0xff));
  HAL_SPI_Write((uint8_t)((parameter >> 16) & 0xff));
  HAL_SPI_Write((uint8_t)((parameter >> 24) & 0xff));
#else
  uint8_t buf[7];

  buf[0] = (uint8_t)((address >> 16) | 0x80);   // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  buf[1] = (uint8_t)(address >> 8);             // Next byte of the register address   
  buf[2] = (uint8_t)address;                    // Low byte of register address - usually just the 1 byte offset
  
  buf[3] = (uint8_t)(parameter & 0xff);         // Little endian (yes, it is most significant bit first and least significant byte first)
  buf[4] = (uint8_t)((parameter >> 8) & 0xff);
  buf[5] = (uint8_t)((parameter >> 16) & 0xff);
  buf[6] = (uint8_t)((parameter >> 24) & 0xff);

  HAL_SPI_WriteBuffer(buf, 7);
#endif
  
  HAL_SPI_Disable();
}

static uint8_t aw_spibuf[2048];
static int aw_size = 0;

void accrue_wr32(uint32_t address, uint32_t parameter)
{

  if (aw_size == 0) {
    aw_spibuf[aw_size++] = (uint8_t)((address >> 16) | 0x80);   // RAM_REG = 0x302000 and high bit is set - result always 0xB0
    aw_spibuf[aw_size++] = (uint8_t)(address >> 8);             // Next byte of the register address   
    aw_spibuf[aw_size++] = (uint8_t)address;                    // Low byte of register address - usually just the 1 byte offset
    }
  
  aw_spibuf[aw_size++] = (uint8_t)(parameter & 0xff);         // Little endian (yes, it is most significant bit first and least significant byte first)
  aw_spibuf[aw_size++] = (uint8_t)((parameter >> 8) & 0xff);
  aw_spibuf[aw_size++] = (uint8_t)((parameter >> 16) & 0xff);
  aw_spibuf[aw_size++] = (uint8_t)((parameter >> 24) & 0xff);

  if ((aw_size > sizeof(aw_spibuf) - 10) || (address == (RAM_CMD + FT_CMD_FIFO_SIZE - FT_CMD_SIZE)))  {
    release_wr32();
  }
}

void release_wr32()
{
  if (aw_size)
    {
    HAL_SPI_Enable();
    HAL_SPI_WriteBuffer(aw_spibuf, aw_size);
    HAL_SPI_Disable();
    aw_size = 0;
    }
}


void wr16(uint32_t address, uint16_t parameter)
{
  //printf("wr16: 0x%08x %04x (%d)\n", address, parameter, parameter);
  HAL_SPI_Enable();
  
  HAL_SPI_Write((uint8_t)((address >> 16) | 0x80)); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  HAL_SPI_Write((uint8_t)(address >> 8));           // Next byte of the register address   
  HAL_SPI_Write((uint8_t)address);                  // Low byte of register address - usually just the 1 byte offset
  
  HAL_SPI_Write((uint8_t)(parameter & 0xff));       // Little endian (yes, it is most significant bit first and least significant byte first)
  HAL_SPI_Write((uint8_t)(parameter >> 8));
  
  HAL_SPI_Disable();
}

void wr8(uint32_t address, uint8_t parameter)
{
  HAL_SPI_Enable();
  
  HAL_SPI_Write((uint8_t)((address >> 16) | 0x80)); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  HAL_SPI_Write((uint8_t)(address >> 8));           // Next byte of the register address   
  HAL_SPI_Write((uint8_t)(address));                // Low byte of register address - usually just the 1 byte offset
  
  HAL_SPI_Write(parameter);             
  
  HAL_SPI_Disable();
}

uint32_t rd32(uint32_t address)
{
  uint8_t buf[4];
  uint32_t Data32;
  int sts;
  int retries = 0;
  
  do
    {
    HAL_SPI_Enable();
    HAL_SPI_Write((address >> 16) & 0x3F);    
    HAL_SPI_Write((address >> 8) & 0xff);    
    HAL_SPI_Write(address & 0xff);
    sts = HAL_SPI_ReadBuffer(buf, 4);
    HAL_SPI_Disable();
    } while (sts != 4 && ++retries < 10 && !mo_usleep(2000));
  
  if (sts == 4)
    Data32 = buf[0] + ((uint32_t)buf[1] << 8) + ((uint32_t)buf[2] << 16) + ((uint32_t)buf[3] << 24);
  else {
    // TFT is dead, restart
    die_and_restart();
    Data32 = 0;
    }

  return (Data32);  
}

int old_rdxx(uint32_t address, uint32_t *buf, uint32_t length)
{
  int ret;

  HAL_SPI_Enable();
  
  HAL_SPI_Write((address >> 16) & 0x3F);    
  HAL_SPI_Write((address >> 8) & 0xff);    
  HAL_SPI_Write(address & 0xff);

  ret = HAL_SPI_ReadBuffer((uint8_t *)buf, length);
  
  HAL_SPI_Disable();

  return(ret);
}


uint16_t rd16(uint32_t address)
{
  uint16_t Data16;
  uint8_t buf[2] = { 0,0 };
  int sts;
  int retries = 0;

  do
    {
    HAL_SPI_Enable();
    HAL_SPI_Write((address >> 16) & 0x3F);    
    HAL_SPI_Write((address >> 8) & 0xff);    
    HAL_SPI_Write(address & 0xff);
    sts = HAL_SPI_ReadBuffer(buf, 2);
    HAL_SPI_Disable();
    } while (sts != 2 && ++retries < 10 && !mo_usleep(2000));

  
  if (sts == 2)
    Data16 = buf[0] + ((uint16_t)buf[1] << 8);
  else {
    // TFT is dead, restart
    die_and_restart();
    Data16 = 0;
    }

  return (Data16);  
}

uint8_t rd8(uint32_t address)
{
  uint8_t buf[1] = {0};
  int sts;
  int retries = 0;

  do
    {
    HAL_SPI_Enable();
    HAL_SPI_Write((address >> 16) & 0x3F);    
    HAL_SPI_Write((address >> 8) & 0xff);    
    HAL_SPI_Write(address & 0xff);
    sts = HAL_SPI_ReadBuffer(buf, 1);
    HAL_SPI_Disable();
    } while (sts != 1 && ++retries < 10 && !mo_usleep(2000));

  if (sts != 1) {
    // TFT is dead, restart
    die_and_restart();
    }
  
  return (buf[0]);  
}

// *** Send_Cmd() - this is like cmd() in (some) Eve docs - sends 32 bits but does not update the write pointer ***
// BT81x Series Programmers Guide Section 5.1.1 - Circular Buffer (AKA "the FIFO" and "Command buffer" and "CoProcessor")
// Don't miss section 5.3 - Interaction with RAM_DL
void Send_CMD(uint32_t data)
{
  if (aw_size)
    release_wr32();

  wr32(FifoWriteLocation + RAM_CMD, data);                         // write the command at the globally tracked "write pointer" for the FIFO

  FifoWriteLocation += FT_CMD_SIZE;                                // Increment the Write Address by the size of a command - which we just sent
  FifoWriteLocation %= FT_CMD_FIFO_SIZE;                           // Wrap the address to the FIFO space
}

void accrue_Send_CMD(uint32_t data)
{
  //printf("                           0x%8x:  0x%08x:\n", FifoWriteLocation + RAM_CMD, data);
  accrue_wr32(FifoWriteLocation + RAM_CMD, data);                   // write the command at the globally tracked "write pointer" for the FIFO

  FifoWriteLocation += FT_CMD_SIZE;                                 // Increment the Write Address by the size of a command - which we just sent
  FifoWriteLocation %= FT_CMD_FIFO_SIZE;                            // Wrap the address to the FIFO space
}

void release_Send_CMD()
{
  release_wr32();
}


// UpdateFIFO - Cause the CoProcessor to realize that it has work to do in the form of a 
// differential between the read pointer and write pointer.  The CoProcessor (FIFO or "Command buffer") does
// nothing until you tell it that the write position in the FIFO RAM has changed
void UpdateFIFO(void)
{
  wr16(REG_CMD_WRITE + RAM_REG, FifoWriteLocation);               // We manually update the write position pointer
}

// Read the specific ID register and return TRUE if it is the expected 0x7C otherwise.
uint8_t Cmd_READ_REG_ID(void)
{
  uint8_t readData[2];
  
  HAL_SPI_Enable();
  HAL_SPI_Write(0x30);                   // Base address RAM_REG = 0x302000
  HAL_SPI_Write(0x20);    
  HAL_SPI_Write(REG_ID);                 // REG_ID offset = 0x00
  HAL_SPI_ReadBuffer(readData, 1);       // There was a dummy read of the first byte in there
  HAL_SPI_Disable();
  
  if (readData[0] == 0x7C) {
    //printf("Good ID\n");
    return 1;
  } else {
    printf("Bad ID 0x%02x\n", readData[0]);
    return 0;
  }
}

void Cmd_Snapshot2(uint32_t fmt, uint32_t ptr, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  accrue_Send_CMD(CMD_SNAPSHOT2);
  accrue_Send_CMD(fmt);
  accrue_Send_CMD(ptr);
  accrue_Send_CMD(((uint32_t)x & 0xffff) | (((uint32_t)y) << 16));
  accrue_Send_CMD(((uint32_t)w & 0xffff) | (((uint32_t)h) << 16));
  release_Send_CMD();
  UpdateFIFO();    
  Wait4CoProFIFOEmpty();
}


// Take a screenshot, addr is address to place bitmap in RAM_G
void Cmd_Screenshot(uint32_t addr)
{
#if 1
  wr8(REG_PCLK + RAM_REG, 0);                           // Turn display off
  wr16(RAM_REG+REG_HSIZE,DWIDTH);
  wr16(RAM_REG+REG_VSIZE,DHEIGHT);
  accrue_Send_CMD(CMD_SNAPSHOT);
  accrue_Send_CMD(addr);
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
  wr8(REG_PCLK + RAM_REG, PCLK);                        // Turn display back on
#else
  // A bit of old test junk
  uint8_t old_pclk = rd8(RAM_REG + REG_PCLK);
  uint16_t old_hsize = rd16(RAM_REG + REG_HSIZE);
  uint16_t old_vsize = rd16(RAM_REG + REG_VSIZE);

  wr8(RAM_REG + REG_PCLK, 0);
  wr16(RAM_REG + REG_HSIZE, DWIDTH);
  wr16(RAM_REG + REG_VSIZE, DHEIGHT);

  accrue_Send_CMD(CMD_SNAPSHOT);
  accrue_Send_CMD(addr);
  release_Send_CMD();
  UpdateFIFO();
  Wait4CoProFIFOEmpty();

  wr16(RAM_REG + REG_HSIZE, old_hsize);
  wr16(RAM_REG + REG_VSIZE, old_vsize);
  wr8(RAM_REG + REG_PCLK, old_pclk);
#endif
}


// **************************************** Co-Processor/GPU/FIFO/Command buffer Command Functions ***************
// These are discussed in BT81x Series Programmers Guide, starting around section 5.10
// While display list commands can be sent to the CoPro, these listed commands are specific to it.  They are 
// mostly widgets like graphs, but also touch related functions like cmd_track() and memory operations. 
// Essentially, these commands set up parameters for CoPro functions which expand "macros" using those parameters
// to then write a series of commands into the Display List to create all the primitives which make that widget.
// ***************************************************************************************************************

// ******************** Screen Object Creation CoProcessor Command Functions ******************************

// *** Draw Slider - BT81x Series Programmers Guide Section 5.38 *************************************************
void Cmd_Slider(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t options, uint16_t val, uint16_t range)
{
  accrue_Send_CMD(CMD_SLIDER);
  accrue_Send_CMD( ((uint32_t)y << 16) | x );
  accrue_Send_CMD( ((uint32_t)h << 16) | w );
  accrue_Send_CMD( ((uint32_t)val << 16) | options );
  accrue_Send_CMD( (uint32_t)range );
}

// *** Draw Spinner - BT81x Series Programmers Guide Section 5.54 *************************************************
void Cmd_Spinner(uint16_t x, uint16_t y, uint16_t style, uint16_t scale)
{    
  accrue_Send_CMD(CMD_SPINNER);
  accrue_Send_CMD( ((uint32_t)y << 16) | x );
  accrue_Send_CMD( ((uint32_t)scale << 16) | style );
}

// *** Draw Gauge - BT81x Series Programmers Guide Section 5.33 **************************************************
void Cmd_Gauge(uint16_t x, uint16_t y, uint16_t r, uint16_t options, uint16_t major, uint16_t minor, uint16_t val, uint16_t range)
{
  accrue_Send_CMD(CMD_GAUGE);
  accrue_Send_CMD(((uint32_t)y << 16) | x );
  accrue_Send_CMD(((uint32_t)options << 16) | r );
  accrue_Send_CMD(((uint32_t)minor << 16) | major );
  accrue_Send_CMD(((uint32_t)range << 16) | val );
}

// *** Draw Dial - BT81x Series Programmers Guide Section 5.39 **************************************************
// This is much like a Gauge except for the helpful range parameter.  For some reason, all dials are 65535 around.
void Cmd_Dial(uint16_t x, uint16_t y, uint16_t r, uint16_t options, uint16_t val)
{
  accrue_Send_CMD(CMD_DIAL);
  accrue_Send_CMD(((uint32_t)y << 16) | x );
  accrue_Send_CMD(((uint32_t)options << 16) | r );
  accrue_Send_CMD((uint32_t)val );
}

// *** Make Track (for a slider) - BT81x Series Programmers Guide Section 5.62 ************************************
// tag refers to the tag # previously assigned to the object that this track is tracking.
void Cmd_Track(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t tag)
{
  accrue_Send_CMD(CMD_TRACK);
  accrue_Send_CMD(((uint32_t)y << 16) | x );
  accrue_Send_CMD(((uint32_t)h << 16) | w );
  accrue_Send_CMD((uint32_t)tag );
}

// *** Draw Number - BT81x Series Programmers Guide Section 5.43 *************************************************
void Cmd_Number(uint16_t x, uint16_t y, uint16_t font, uint16_t options, uint32_t num)
{
  accrue_Send_CMD(CMD_NUMBER);
  accrue_Send_CMD(((uint32_t)y << 16) | x );
  accrue_Send_CMD(((uint32_t)options << 16) | font );
  accrue_Send_CMD(num);
}

// *** Draw Smooth Color Gradient - BT81x Series Programmers Guide Section 5.34 **********************************
void Cmd_Gradient(uint16_t x0, uint16_t y0, uint32_t rgb0, uint16_t x1, uint16_t y1, uint32_t rgb1)
{
  accrue_Send_CMD(CMD_GRADIENT);
  accrue_Send_CMD( ((uint32_t)y0<<16)|x0 );
  accrue_Send_CMD(rgb0);
  accrue_Send_CMD( ((uint32_t)y1<<16)|x1 );
  accrue_Send_CMD(rgb1);
}

// *** Draw Button - BT81x Series Programmers Guide Section 5.28 **************************************************
void Cmd_Button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t font, uint16_t options, const char* str)
{ 
  int i;
  const char *p;
  int len = strlen(str);
  uint32_t keystr;

  if (!len)
    return;

  accrue_Send_CMD(CMD_BUTTON);
  accrue_Send_CMD( ((uint32_t)y << 16) | x ); // Put two 16 bit values together into one 32 bit value - do it little endian
  accrue_Send_CMD( ((uint32_t)h << 16) | w );
  accrue_Send_CMD( ((uint32_t)options << 16) | font );

  keystr = 0;
  for (i = 0, p = str; i <= len; i++, p++) {        // <= to include the terminating NULL
    keystr |= ((uint32_t)(*p)) << ((i&0x3)*8);

    if ((i && (i%4) == 3) || (i == len)) {
      accrue_Send_CMD(keystr);
      keystr = 0;
      }
    }
}

void Cmd_Keys(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t font, uint16_t options, const char* str)
{
  int i;
  const char *p;
  int len = strlen(str);
  uint32_t keystr;

  if (!len)
    return;

  accrue_Send_CMD(CMD_KEYS);
  accrue_Send_CMD( ((uint32_t)y << 16) | x );
  accrue_Send_CMD( ((uint32_t)h << 16) | w );
  accrue_Send_CMD( ((uint32_t)options << 16) | font );

  keystr = 0;
  for (i = 0, p = str; i <= len; i++, p++) {        // <= to include the terminating NULL
    keystr |= ((uint32_t)(*p)) << ((i&0x3)*8);

    if ((i && (i%4) == 3) || (i == len)) {
      accrue_Send_CMD(keystr);
      keystr = 0;
      }
    }
}

#define MAX_STR 1024

// *** Draw Text - BT81x Series Programmers Guide Section 5.41 ***************************************************
void Cmd_Text(uint16_t x, uint16_t y, uint16_t font, uint16_t options, const char* str)
{
  uint16_t i, j, idx;
  uint32_t data[MAX_STR/4+1];
  
  uint16_t length = (uint16_t) strlen(str);

  //printf("Cmd_Text: %d: |%s|\n", length, str);
  if (!length) 
    return; 
  if (length > MAX_STR) {
    fprintf(stderr, "BUG: Increase MAX_STR\n");
    exit(1);
  }
  
  idx = 0;
  memset(data, 0, sizeof(data));
  for (j = 0; j < (length/4); ++j, idx += 4)
    data[j] = (uint32_t)str[idx+3]<<24 | (uint32_t)str[idx+2]<<16 | (uint32_t)str[idx+1]<<8 | (uint32_t)str[idx];

  for (i = 0, data[j] = 0; i < (length%4); ++i, ++idx)      // Residual
    data[j] |= (uint32_t)str[idx] << (i*8);

  // Set up the command
  accrue_Send_CMD(CMD_TEXT);
  accrue_Send_CMD(((uint32_t)y << 16) | x );
  accrue_Send_CMD(((uint32_t)options << 16) | font );

  // Send out the text
  for (i = 0; i <= length/4; i++) {
    accrue_Send_CMD(data[i]);
    //printf("Cmd_Text: %d: 0x%08x\n", i, data[i]);
    }

  // Something will always do a release...
}

// ******************** Miscellaneous Operation CoProcessor Command Functions ******************************

// *** Cmd_SetBitmap - generate DL commands for bitmap parms - BT81x Series Programmers Guide Section 5.65 *******
void Cmd_SetBitmap(uint32_t addr, uint16_t fmt, uint16_t width, uint16_t height)
{
  accrue_Send_CMD( CMD_SETBITMAP );
  accrue_Send_CMD( addr );
  accrue_Send_CMD( ((uint32_t)width << 16) | fmt );
  accrue_Send_CMD( (uint32_t)height);
}

// *** Cmd_Memcpy - background copy a block of data - BT81x Series Programmers Guide Section 5.27 ****************
void Cmd_Memcpy(uint32_t dest, uint32_t src, uint32_t num)
{
  Send_CMD(CMD_MEMCPY);
  Send_CMD(dest);
  Send_CMD(src);
  Send_CMD(num);
}

// *** Cmd_GetPtr - Get the last used address from CoPro operation - BT81x Series Programmers Guide Section 5.47 *
void Cmd_GetPtr(void)
{
  Send_CMD(CMD_GETPTR);
  Send_CMD(0);
}

// *** Set Highlight Gradient Color - BT81x Series Programmers Guide Section 5.32 ********************************
void Cmd_GradientColor(uint32_t c)
{
  Send_CMD(CMD_GRADCOLOR);
  Send_CMD(c);
}

// *** Set FG color - BT81x Series Programmers Guide Section 5.30 ************************************************
void Cmd_FGcolor(uint32_t c)
{
  accrue_Send_CMD(CMD_FGCOLOR);
  accrue_Send_CMD(c);
}

// *** Set BG color - BT81x Series Programmers Guide Section 5.31 ************************************************
void Cmd_BGcolor(uint32_t c)
{
  accrue_Send_CMD(CMD_BGCOLOR);
  accrue_Send_CMD(c);
}

// *** Translate Matrix - BT81x Series Programmers Guide Section 5.51 ********************************************
void accrue_Cmd_Translate(uint32_t tx, uint32_t ty)
{
  accrue_Send_CMD(CMD_TRANSLATE);
  accrue_Send_CMD(tx);
  accrue_Send_CMD(ty);
}

// *** Rotate Matrix - BT81x Series Programmers Guide Section 5.50 ***********************************************
void Cmd_Rotate(uint32_t a)
{
  Send_CMD(CMD_ROTATE);
  Send_CMD(a);
}

// *** Rotate Screen - BT81x Series Programmers Guide Section 5.53 ***********************************************
void Cmd_SetRotate(uint32_t rotation)
{
  Send_CMD(CMD_SETROTATE);
  Send_CMD(rotation);
}

// *** Scale Matrix - BT81x Series Programmers Guide Section 5.49 ************************************************
void Cmd_Scale(uint32_t sx, uint32_t sy)
{
  Send_CMD(CMD_SCALE);
  Send_CMD(sx);
  Send_CMD(sy);
}

void Cmd_Flash_Fast(void)
{
  Send_CMD(CMD_FLASHFAST);
  Send_CMD(0);
}

// *** Calibrate Touch Digitizer - BT81x Series Programmers Guide Section 5.52 ***********************************
// * This business about "result" in the manual really seems to be simply leftover cruft of no purpose - send zero
void Cmd_Calibrate(uint32_t result)
{
  Send_CMD(CMD_CALIBRATE);
  Send_CMD(result);
}

// An interactive calibration screen is created and executed.  
// New calibration values are written to the touch matrix registers of Eve.
void Calibrate_Manual(uint16_t Width, uint16_t Height, uint16_t V_Offset, uint16_t H_Offset)
{
  uint32_t displayX[3], displayY[3];
  uint32_t touchX[3], touchY[3]; 
  uint32_t touchValue = 0;
  //uint32_t touchValue = 0, storedValue = 0;  
  int32_t tmp, k;
  int32_t TransMatrix[6];
  uint8_t count = 0;
  uint8_t pressed = 0;
  char num[2];

  // These values determine where your calibration points will be drawn on your display
  displayX[0] = (uint32_t) (Width * 0.15) + H_Offset;
  displayY[0] = (uint32_t) (Height * 0.15) + V_Offset;
  
  displayX[1] = (uint32_t) (Width * 0.85) + H_Offset;
  displayY[1] = (uint32_t) (Height / 2) + V_Offset;
  
  displayX[2] = (uint32_t) (Width / 2) + H_Offset;
  displayY[2] = (uint32_t) (Height * 0.85) + V_Offset;

  while (count < 3) 
  {
    Send_CMD(CMD_DLSTART);
    Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));	
    Send_CMD(CLEAR(1,1,1));

    // Draw Calibration Point on screen
    Send_CMD(COLOR_RGB(255, 0, 0));
    Send_CMD(POINT_SIZE(20 * 16));
    Send_CMD(BEGIN(POINTS));
    Send_CMD(VERTEX2F((uint32_t)(displayX[count]) * 16, (uint32_t)((displayY[count])) * 16)); 
    Send_CMD(END());
    Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text((Width / 2) + H_Offset, (Height / 3) + V_Offset, 27, OPT_CENTER, "Calibrating");
    Cmd_Text((Width / 2) + H_Offset, (Height / 2) + V_Offset, 27, OPT_CENTER, "Please tap the dots");
    num[0] = count + 0x31; num[1] = 0;                                            // null terminated string of one character
    Cmd_Text(displayX[count], displayY[count], 27, OPT_CENTER, num);

    Send_CMD(DISPLAY());
    Send_CMD(CMD_SWAP);
    UpdateFIFO();                                                                 // Trigger the CoProcessor to start processing commands out of the FIFO
    Wait4CoProFIFOEmpty();                                                        // wait here until the coprocessor has read and executed every pending command.
    HAL_Delay(300);

	while (pressed == count)
	{
		touchValue = rd32(REG_TOUCH_DIRECT_XY + RAM_REG);                             // Read for any new touch tag inputs
		if (!(touchValue & 0x80000000))
		{
			touchX[count] = (touchValue >> 16) & 0x03FF;                                  // Raw Touchscreen Y coordinate
			touchY[count] = touchValue & 0x03FF;                                        // Raw Touchscreen Y coordinate

																						//Log("\ndisplay x[%d]: %ld display y[%d]: %ld\n", count, displayX[count], count, displayY[count]);
																						//Log("touch x[%d]: %ld touch y[%d]: %ld\n", count, touchX[count], count, touchY[count]);

			count++;
		}
	}
	pressed = count;

  }

  k = ((touchX[0] - touchX[2])*(touchY[1] - touchY[2])) - ((touchX[1] - touchX[2])*(touchY[0] - touchY[2]));

  tmp = (((displayX[0] - displayX[2]) * (touchY[1] - touchY[2])) - ((displayX[1] - displayX[2])*(touchY[0] - touchY[2])));
  TransMatrix[0] = ((int64_t)tmp << 16) / k;

  tmp = (((touchX[0] - touchX[2]) * (displayX[1] - displayX[2])) - ((displayX[0] - displayX[2])*(touchX[1] - touchX[2])));
  TransMatrix[1] = ((int64_t)tmp << 16) / k;

  tmp = ((touchY[0] * (((touchX[2] * displayX[1]) - (touchX[1] * displayX[2])))) + (touchY[1] * (((touchX[0] * displayX[2]) - (touchX[2] * displayX[0])))) + (touchY[2] * (((touchX[1] * displayX[0]) - (touchX[0] * displayX[1])))));
  TransMatrix[2] = ((int64_t)tmp << 16) / k;

  tmp = (((displayY[0] - displayY[2]) * (touchY[1] - touchY[2])) - ((displayY[1] - displayY[2])*(touchY[0] - touchY[2])));
  TransMatrix[3] = ((int64_t)tmp << 16) / k;

  tmp = (((touchX[0] - touchX[2]) * (displayY[1] - displayY[2])) - ((displayY[0] - displayY[2])*(touchX[1] - touchX[2])));
  TransMatrix[4] = ((int64_t)tmp << 16) / k;

  tmp = ((touchY[0] * (((touchX[2] * displayY[1]) - (touchX[1] * displayY[2])))) + (touchY[1] * (((touchX[0] * displayY[2]) - (touchX[2] * displayY[0])))) + (touchY[2] * (((touchX[1] * displayY[0]) - (touchX[0] * displayY[1])))));
  TransMatrix[5] = ((int64_t)tmp << 16) / k;


  uint32_t d0 = rd32(REG_DATESTAMP + RAM_REG);
  uint32_t d1 = rd32(REG_DATESTAMP + 4 + RAM_REG);
  uint32_t d2 = rd32(REG_DATESTAMP + 8 + RAM_REG);
  uint32_t d3 = rd32(REG_DATESTAMP + 12 + RAM_REG);

  printf("Cal: stamp %08x_%08x_%08x_%08x TransMatrix[A-F] are 0x%08x %08x %08x %08x %08x %08x\n", 
      d3, d2, d1, d0,
      TransMatrix[0], TransMatrix[1], TransMatrix[2],
      TransMatrix[3], TransMatrix[4], TransMatrix[5]);
  
  count = 0;
  do
  {
    wr32(REG_TOUCH_TRANSFORM_A + RAM_REG + (count * 4), TransMatrix[count]);  // Write to Eve config registers

//    uint16_t ValH = TransMatrix[count] >> 16;
//    uint16_t ValL = TransMatrix[count] & 0xFFFF;
//    Log("TM%d: 0x%04x %04x\n", count, ValH, ValL);
    
    count++;
  }while(count < 6);
}
// ***************************************************************************************************************
// *** Animation functions ***************************************************************************************
// ***************************************************************************************************************

void Cmd_AnimStart(int32_t ch, uint32_t aoptr, uint32_t loop)
{
	Send_CMD(CMD_ANIMSTART);
	Send_CMD(ch);
	Send_CMD(aoptr);
	Send_CMD(loop);
}

void Cmd_AnimStop(int32_t ch)
{
	Send_CMD(CMD_ANIMSTOP);
	Send_CMD(ch);
}

void Cmd_AnimXY(int32_t ch, int16_t x, int16_t y)
{
	Send_CMD(CMD_ANIMXY);
	Send_CMD(ch);
	Send_CMD(((uint32_t)y << 16) | x);
}

void Cmd_AnimDraw(int32_t ch)
{
	Send_CMD(CMD_ANIMDRAW);
	Send_CMD(ch);
}

void Cmd_AnimDrawFrame(int16_t x, int16_t y, uint32_t aoptr, uint32_t frame)
{
	Send_CMD(CMD_ANIMFRAME);
	Send_CMD(((uint32_t)y << 16) | x);
	Send_CMD(aoptr);
	Send_CMD(frame);
}

// ***************************************************************************************************************
// *** Utility and helper functions ******************************************************************************
// ***************************************************************************************************************

// Find the space available in the GPU AKA CoProcessor AKA command buffer AKA FIFO
uint16_t CoProFIFO_FreeSpace(void)
{
#ifdef OLDSTUFF
  uint16_t cmdBufferDiff, cmdBufferRd, cmdBufferWr, retval;
  
  cmdBufferRd = rd16(REG_CMD_READ + RAM_REG);
  cmdBufferWr = rd16(REG_CMD_WRITE + RAM_REG);

  // BT81x Programmers Guide 5.1.1
  if (cmdBufferWr >= cmdBufferRd)
    cmdBufferDiff = (cmdBufferWr-cmdBufferRd);
  else
    cmdBufferDiff = (cmdBufferWr + FT_CMD_FIFO_SIZE) - cmdBufferRd;
  //retval = (FT_CMD_FIFO_SIZE - 4) - cmdBufferDiff;
  retval = FT_CMD_FIFO_SIZE - cmdBufferDiff;

  return (retval);
#else

  return(rd16(REG_CMDB_SPACE + RAM_REG));

#endif
}

// Sit and wait until there are the specified number of bytes free in the <GPU/CoProcessor> incoming FIFO
void Wait4CoProFIFO(uint32_t room)
{
   uint16_t getfreespace;
   uint16_t start_space;
   int first = 1;
   int loops = 0;
   
   do {
     if (!first)
       mo_usleep(100);
     getfreespace = CoProFIFO_FreeSpace();
     loops++;
     if (first) {
       start_space = getfreespace;
       UpdateFIFO();    // Make sure coprocessor has everything
       }
     if (loops > 20000) {
       fprintf(stderr, "Wait4CoProFIFO: start %d now %d, loops %d\n", start_space, getfreespace, loops);
       mo_usleep(500000);
       }
   } while (getfreespace < room);

  //printf("start %d now %d, loops %d\n", start_space, getfreespace, loops);
}

// Sit and wait until the CoPro FIFO is empty
// Detect operational errors and print the error and stop.
void Wait4CoProFIFOEmpty(void)
{
  uint16_t ReadReg, WriteReg;
  uint8_t ErrChar;
  char buffy[128];

  do
  {
    ReadReg = rd16(REG_CMD_READ + RAM_REG);
    if (ReadReg == 0xFFF) {
      // this is a error which would require sophistication to fix and continue but we fake it somewhat unsuccessfully
      uint8_t Offset = 0;
      do
      {
        // Get the error character and display it
        ErrChar = rd8(RAM_ERR_REPORT + Offset);
        buffy[Offset++] = ErrChar;
      } while ( (ErrChar != 0) && (Offset < 128) ); // when the last stuffed character was null, we are done
      if (Offset == 128)
        buffy[127] = '\0';

      fprintf(stderr, "EVE: %s\n", buffy);
      //exit(1);

      // Eve is unhappy - needs a paddling.
      uint32_t Patch_Add = rd32(REG_COPRO_PATCH_PTR + RAM_REG);
      wr8(REG_CPU_RESET + RAM_REG, 1);
      wr8(REG_CMD_READ + RAM_REG, 0);
      wr8(REG_CMD_WRITE + RAM_REG, 0);
      wr8(REG_CMD_DL + RAM_REG, 0);
      wr8(REG_CPU_RESET + RAM_REG, 0);
      wr32(REG_COPRO_PATCH_PTR + RAM_REG, Patch_Add);
      HAL_Delay(250);  // we already saw one error message and we don't need to see then 1000 times a second
    }
  WriteReg = rd16(REG_CMD_WRITE + RAM_REG);
  if (ReadReg == WriteReg)
    break;
  mo_usleep(500);
  } while (1);
}

// Every CoPro transaction starts with enabling the SPI and sending an address
void StartCoProTransfer(uint32_t address, uint8_t reading)
{
  HAL_SPI_Enable();
  if (reading){
    HAL_SPI_Write(address >> 16);
    HAL_SPI_Write(address >> 8);
    HAL_SPI_Write(address);
    HAL_SPI_Write(0);
  }else{
    HAL_SPI_Write((address >> 16) | 0x80); 
    HAL_SPI_Write(address >> 8);           
    HAL_SPI_Write(address);                
  }
}

// *** CoProWrCmdBuf() - Transfer a buffer into the CoPro FIFO as part of an ongoing command operation ***********
void CoProWrCmdBuf(const uint8_t *buff, uint32_t count)
{
  uint32_t TransferSize = 0;
  int32_t Remaining = count; // signed

  do {                
    // Here is the situation:  You have up to about a megabyte of data to transfer into the FIFO
    // Your buffer is LogBuf - limited to 64 bytes (or some other value, but always limited).
    // You need to go around in loops taking 64 bytes at a time until all the data is gone.
    //
    // Most interactions with the FIFO are started and finished in one operation in an obvious fashion, but 
    // here it is important to understand the difference between Eve RAM registers and Eve FIFO.  Even though 
    // you are in the middle of a FIFO operation and filling the FIFO is an ongoing task, you are still free 
    // to write and read non-FIFO registers on the Eve chip.
    //
    // Since the FIFO is 4K in size, but the RAM_G space is 1M in size, you can not, obviously, send all
    // the possible RAM_G data through the FIFO in one step.  Also, since the Eve is not capable of updating
    // it's own FIFO pointer as data is written, you will need to intermittently tell Eve to go process some
    // FIFO in order to make room in the FIFO for more RAM_G data.    
    Wait4CoProFIFO(WorkBuffSz);                            // It is reasonable to wait for a small space instead of firing data piecemeal

    if (Remaining > WorkBuffSz)                            // Remaining data exceeds the size of our buffer
      TransferSize = WorkBuffSz;                           // So set the transfer size to that of our buffer
    else
    {
      TransferSize = Remaining;                            // Set size to this last dribble of data
      TransferSize = (TransferSize + 3) & 0xFFC;           // 4 byte alignment
    }
    
    StartCoProTransfer(FifoWriteLocation + RAM_CMD, false);// Base address of the Command Buffer plus our offset into it - Start SPI transaction
    
    if (TransferSize == 0)
      printf("ERROR: In CoProWrCmdBuf, zero call to HAL_SPI_WriteBuffer\n");
    HAL_SPI_WriteBuffer((uint8_t*)buff, TransferSize);         // write the little bit for which we found space
    buff += TransferSize;                                  // move the working data read pointer to the next fresh data

    FifoWriteLocation  = (FifoWriteLocation + TransferSize) % FT_CMD_FIFO_SIZE;  
    HAL_SPI_Disable();                                         // End SPI transaction with the FIFO
    
    wr16(REG_CMD_WRITE + RAM_REG, FifoWriteLocation);      // Manually update the write position pointer to initiate processing of the FIFO
    Remaining -= TransferSize;                             // reduce what we want by what we sent
    
  }while (Remaining > 0);                                  // keep going as long as we still want more
}

// Write a block of data into Eve RAM space a byte at a time.
// Return the last written address + 1 (The next available RAM address)
uint32_t WriteBlockRAM(uint32_t Add, const uint8_t *buff, uint32_t count)
{
  uint32_t index;
  uint32_t WriteAddress = Add;  // I want to return the value instead of modifying the variable in place
  
  for (index = 0; index < count; index++)
  {
    wr8(WriteAddress++, buff[index]);
  }
  return (WriteAddress);
}

// CalcCoef - Support function for manual screen calibration function
int32_t CalcCoef(int32_t Q, int32_t K)
{
  int8_t sn = 0;

  if (Q < 0)                                       // We need to work with positive values
  {
    Q *= -1;                                       // So here we make them positive
    sn++;                                          // and remember that fact
  }

  if (K < 0)                                       
  {
    K *= -1;
    sn++;                                          // 1 + 1 = 2 = 0b00000010
  }  
  
  uint32_t I = ((uint32_t)Q / (uint32_t)K) << 16;  // get the integer part and shift it by 16
  uint32_t R = Q % K;                              // get the remainder of a/k;
  R = R << 14;                                     // shift by 14 
  R = R / K;                                       // divide
  R = R << 2;                                      // Make up for the missing bits  
  int32_t returnValue = I + R;                     // combine them

  if (sn & 0x01)                                   // If the result is supposed to be negative
    returnValue *= -1;                             // then return it to that state.
      
  return (returnValue);
}

bool FlashAttach(void)
{
	Send_CMD(CMD_FLASHATTACH);
	UpdateFIFO();                                                       // Trigger the CoProcessor to start processing commands out of the FIFO
	Wait4CoProFIFOEmpty();                                              // wait here until the coprocessor has read and executed every pending command.

	uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
	if (FlashStatus != FLASH_STATUS_BASIC)
	{
		return false;
	}
	return true;
}

bool FlashDetach(void)
{
	Send_CMD(CMD_FLASHDETACH);
	UpdateFIFO();                                                       // Trigger the CoProcessor to start processing commands out of the FIFO
	Wait4CoProFIFOEmpty();                                              // wait here until the coprocessor has read and executed every pending command.

	uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
	if (FlashStatus != FLASH_STATUS_DETACHED)
	{
		return false;
	}
	return true;
}

bool FlashFast(void)
{
	Cmd_Flash_Fast();
	UpdateFIFO();                                                       // Trigger the CoProcessor to start processing commands out of the FIFO
	Wait4CoProFIFOEmpty();                                              // wait here until the coprocessor has read and executed every pending command.

	uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
	if (FlashStatus != FLASH_STATUS_FULL)
	{
		return false;
	}
	return true;
}

bool FlashErase(void)
{
	Send_CMD(CMD_FLASHERASE);
	UpdateFIFO();                                                       // Trigger the CoProcessor to start processing commands out of the FIFO
	Wait4CoProFIFOEmpty();                                              // wait here until the coprocessor has read and executed every pending command.
	return true;
}

