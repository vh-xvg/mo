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


#include <linux/types.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include "lidar.h"
#include "efi_protocol.h"

int lidar_i2c;

/*------------------------------------------------------------------------------
  I2C Init
  Initialize the I2C peripheral in the processor core
------------------------------------------------------------------------------*/
int lidar_i2c_init (void)
{
  const char *filename = (const char *) "/dev/i2c-6";
  static int broken = 0;

  if ((lidar_i2c = open(filename, O_RDWR)) < 0) {
    if (!broken) {
      dbprintf("Lidar: Failed to open the i2c bus\n");
      broken = 1;
      }
    return -1;
  } else {
    //dbprintf("Opened Lidar i2c interface OK\n");
    broken = 0;
    return 0;
  }
}

/*------------------------------------------------------------------------------
  I2C Connect
  Connect to the I2C device with the specified device address

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
int lidar_i2c_connect (uint8_t lidarliteAddress)
{
  static int broken = 0;

  if (ioctl(lidar_i2c, I2C_SLAVE, lidarliteAddress) < 0)
    {
    if (!broken) {
      dbprintf("Lidar: Failed to acquire bus access and/or talk to slave.\n");
      broken = 1;
      }
    return -1;
  } else {
    //dbprintf("Connected to Lidar OK\n");
    broken = 0;
    return 0;
  }
}

/*------------------------------------------------------------------------------
  Configure
  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void lidar_configure(uint8_t configuration, uint8_t lidarliteAddress)
{
    uint8_t sigCountMax;
    uint8_t acqConfigReg;
    uint8_t refCountMax;
    uint8_t thresholdBypass;

    switch (configuration)
    {
        case 0: // Default mode, balanced performance
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 1: // Short range, high speed
            sigCountMax     = 0x1d;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 2: // Default range, higher speed short range
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x00;
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 3: // Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 4: // High sensitivity detection, high erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x80;
            break;

        case 5: // Low sensitivity detection, low erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0xb0;
            break;

        case 6: // Short range, high speed, higher error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x01; // turn off short_sig, mode pin = status output mode
            refCountMax     = 0x03;
            thresholdBypass = 0x00;
            break;

        default: // Default mode, balanced performance - same as configure(0)
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;
    }

    lidar_i2cWrite(LLv3_SIG_CNT_VAL,   &sigCountMax    , 1, lidarliteAddress);
    lidar_i2cWrite(LLv3_ACQ_CONFIG,    &acqConfigReg   , 1, lidarliteAddress);
    lidar_i2cWrite(LLv3_REF_CNT_VAL,   &refCountMax    , 1, lidarliteAddress);
    lidar_i2cWrite(LLv3_THRESH_BYPASS, &thresholdBypass, 1, lidarliteAddress);
} /* configure */

/*------------------------------------------------------------------------------
  Set I2C Address
  Set Alternate I2C Device Address. See Operation Manual for additional info.
  
  Parameters
  ------------------------------------------------------------------------------
  newAddress: desired secondary I2C device address
  disableDefault: a non-zero value here means the default 0x62 I2C device
    address will be disabled.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void lidar_setI2Caddr(uint8_t newAddress, uint8_t disableDefault, uint8_t lidarliteAddress)
{
    uint8_t dataBytes[2];

    // Read UNIT_ID serial number bytes and write them into I2C_ID byte locations
    lidar_i2cRead ((LLv3_UNIT_ID_HIGH | 0x80), dataBytes, 2, lidarliteAddress);
    lidar_i2cWrite(LLv3_I2C_ID_HIGH,           dataBytes, 2, lidarliteAddress);

    // Write the new I2C device address to registers
    dataBytes[0] = newAddress;
    lidar_i2cWrite(LLv3_I2C_SEC_ADR,           dataBytes, 1, lidarliteAddress);

    // Enable the new I2C device address using the default I2C device address
    dataBytes[0] = 0;
    lidar_i2cWrite(LLv3_I2C_CONFIG,            dataBytes, 1, lidarliteAddress);

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefault)
    {
        dataBytes[0] = (1 << 3); // set bit to disable default address
        lidar_i2cWrite(LLv3_I2C_CONFIG, dataBytes, 1, newAddress);
    }
} /* setI2Caddr */

/*------------------------------------------------------------------------------
  Take Range
  Initiate a distance measurement by writing to register 0x00.
  
  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void lidar_takeRange(uint8_t lidarliteAddress)
{
    uint8_t commandByte = 0x04;

    lidar_i2cWrite(LLv3_ACQ_CMD, &commandByte, 1, lidarliteAddress);
} /* takeRange */

void lidar_set_threshold_bypass(uint8_t lidarliteAddress, uint8_t threshold_bypass)
{
    lidar_i2cWrite(LLv3_THRESH_BYPASS, &threshold_bypass, 1, lidarliteAddress);
}

/*------------------------------------------------------------------------------
  Wait for Busy Flag
  Blocking function to wait until the Lidar Lite's internal busy flag goes low
  
  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void lidar_waitForBusy(uint8_t lidarliteAddress)
{
  uint8_t  busyFlag;

  do
    {
    busyFlag = lidar_getBusyFlag(lidarliteAddress);
    } while (busyFlag && !mo_usleep(1000));
}

/*------------------------------------------------------------------------------
  Get Busy Flag
  Read BUSY flag from device registers. Function will return 0x00 if not busy.
  
  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint8_t lidar_getBusyFlag(uint8_t lidarliteAddress)
{
    uint8_t  statusByte = 0;
    uint8_t  busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    lidar_i2cRead(LLv3_STATUS, &statusByte, 1, lidarliteAddress);

    // STATUS bit 0 is busyFlag
    busyFlag = statusByte & 0x01;

    return busyFlag;
} 

uint8_t lidar_getStatus(uint8_t lidarliteAddress)
{
    uint8_t  statusByte = 0;
    int sts;

    // Read status register to check busy flag
    sts = lidar_i2cRead(LLv3_STATUS, &statusByte, 1, lidarliteAddress);

    dbprintf("lidar_getStatus: returned %d\n", sts);

    return statusByte;
}

/*------------------------------------------------------------------------------
  Read Distance
  Read and return result of distance measurement.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint16_t lidar_readDistance(uint8_t lidarliteAddress)
{
    uint8_t  distBytes[2] = {0};

    // Read two bytes from register 0x0f and 0x10 (autoincrement)
    lidar_i2cRead((LLv3_DISTANCE | 0x80), distBytes, 2, lidarliteAddress);

    // Shift high byte and OR in low byte
    return ((distBytes[0] << 8) | distBytes[1]);
} /* readDistance */

/*------------------------------------------------------------------------------
  Write
  Perform I2C write to device.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to data bytes to write
  numBytes:  number of bytes to write
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
uint32_t lidar_i2cWrite(uint8_t regAddr,  uint8_t * dataBytes,
                             uint8_t numBytes, uint8_t lidarliteAddress)
{
    uint8_t buffer[2];
    uint8_t i;
    uint32_t result;

    lidar_i2c_connect(lidarliteAddress);

    for (i=0 ; i<numBytes ; i++)
    {
        buffer[0] = regAddr + i;
        buffer[1] = dataBytes[i];
        result   |= write(lidar_i2c, buffer, 2);
    }

    return result;
} /* i2cWrite */

/*------------------------------------------------------------------------------
  Read
  Perform I2C read from device.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array to place read bytes
  numBytes:  number of bytes in 'dataBytes' array to read (32 bytes max)
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
  operating manual for instructions.
------------------------------------------------------------------------------*/
int lidar_i2cRead(uint8_t regAddr,  uint8_t * dataBytes,
                            uint8_t numBytes, uint8_t lidarliteAddress)
{
    uint8_t buffer;

    lidar_i2c_connect(lidarliteAddress);

    buffer = regAddr;

    write(lidar_i2c, &buffer, 1);
    return read(lidar_i2c, dataBytes, numBytes);
} /* i2cRead */

/*------------------------------------------------------------------------------
  Correlation Record Read
  The correlation record used to calculate distance can be read from the device.
  It has a bipolar wave shape, transitioning from a positive going portion to a
  roughly symmetrical negative going pulse. The point where the signal crosses
  zero represents the effective delay for the reference and return signals.
  
  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no correlation record without at least
      one distance reading being taken)
  2.  Set test mode select by writing 0x07 to register 0x40
  3.  For as many points as you want to read from the record (max is 1024) ...
      1.  Read two bytes from 0x52
      2.  The Low byte is the value from the record
      3.  The high byte is the sign from the record
  Parameters
  ------------------------------------------------------------------------------
  numberOfReadings: Default = 256. Maximum = 1024
  corrValues:       pointer to memory location to store the correlation record
                    ** Two bytes for every correlation value must be
                       allocated by calling function
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void lidar_correlationRecordRead(uint16_t * correlationArray,
                                         uint16_t numberOfReadings,
                                         uint8_t  lidarliteAddress)
{
    uint16_t  i = 0;
    uint8_t   dataBytes[2];
    uint16_t  correlationValue;
    uint8_t * correlationValuePtr = (uint8_t *) &correlationValue;

    //  Select memory bank
    dataBytes[0] = 0xc0;
    lidar_i2cWrite(LLv3_ACQ_SETTINGS, dataBytes, 1, lidarliteAddress);

    // Test mode enable
    dataBytes[0] = 0x07;
    lidar_i2cWrite(LLv3_COMMAND, dataBytes, 1, lidarliteAddress);

    for (i=0 ; i<numberOfReadings ; i++)
    {
        lidar_i2cRead((LLv3_CORR_DATA | 0x80), dataBytes, 2, lidarliteAddress);

        // First byte read is the magnitude of the data point
        correlationValuePtr[0] = dataBytes[0];

        // Second byte is the sign byte
        if (dataBytes[1])
            correlationValuePtr[1] = 0xff; // Artificially sign extend
        else
            correlationValuePtr[1] = 0x00;

        correlationArray[i] = correlationValue;
    }

    // Test mode disable
    dataBytes[0] = 0;
    lidar_i2cWrite(LLv3_COMMAND, dataBytes, 1, lidarliteAddress);
} /* correlationRecordRead */
