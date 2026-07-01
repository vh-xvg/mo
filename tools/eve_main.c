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
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

#include "eve_lib.h"
#include "hw_api.h"
#include "eve_config.h"
#include "eve_xvg.h"

static void eve_test3(void);

#define DIMENSION(x) (sizeof(x)/sizeof(x[0]))

// Calibration data:
// "Good" MO39:
//      Cal: stamp 382e352e_31203931_2d37302d_37313032 TransMatrix[A-F] are 0x000102fb ffffef1d fff74f53 000001ff 0000f698 000010be

/****************************************************************************
*                             IMPORTANT NOTICE                              *
* Please make sure to configure the display timing values and define what   *
* touch variant you are using in the MatrixEve2Conf.h file.                 *
*                                                                           *
****************************************************************************/


//MakeScreen_MatrixOrbital draws a blue dot in the center screen, along
//with the text "MATRIX ORBITAL"
void MakeScreen_MatrixOrbital(uint8_t DotSize)
{
	Send_CMD(CMD_DLSTART);                   //Start a new display list
	Send_CMD(VERTEXFORMAT(0));               //setup VERTEX2F to take pixel coordinates
	Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      //Determine the clear screen color
	Send_CMD(CLEAR(1, 1, 1));	             //Clear the screen and the curren display list
	Send_CMD(COLOR_RGB(26, 26, 192));        // change colour to blue
	Send_CMD(POINT_SIZE(DotSize * 16));      // set point size to DotSize pixels. Points = (pixels x 16)
	Send_CMD(BEGIN(POINTS));                 // start drawing point
	Send_CMD(TAG(1));                        // Tag the blue dot with a touch ID
	Send_CMD(VERTEX2F(DWIDTH / 2, DHEIGHT / 2));     // place blue point
	Send_CMD(END());                         // end drawing point
	Send_CMD(COLOR_RGB(255, 255, 255));      //Change color to white for text
	Cmd_Text(DWIDTH / 2, DHEIGHT / 2, 30, OPT_CENTER, " MATRIX         ORBITAL"); //Write text in the center of the screen
	Send_CMD(DISPLAY());                     //End the display list
	Send_CMD(CMD_SWAP);                      //Swap commands into RAM
	UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO
}


// A calibration screen for the touch digitizer 
void Calibrate(void)
{
	Calibrate_Manual(DWIDTH, DHEIGHT, PIXVOFFSET, PIXHOFFSET);
}

// A Clear screen function 
void ClearScreen(void)
{
	Send_CMD(CMD_DLSTART);
	Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
	Send_CMD(CLEAR(1, 1, 1));
	Send_CMD(DISPLAY());
	Send_CMD(CMD_SWAP);
	UpdateFIFO();                            // Trigger the CoProcessor to start processing commands out of the FIFO
	Wait4CoProFIFOEmpty();                   // wait here until the coprocessor has read and executed every pending command.		
	HAL_Delay(10);
}


void eve_standard_test()
{

  printf("eve_main: entered\n");

	ClearScreen();	                       //Clear any remnants in the RAM

	//If you are using a touch screen, make sure to define what 
	//variant you are using in the MatrixEveConf.h file
#ifdef TOUCH_RESISTIVE
	Calibrate();
#endif

	MakeScreen_MatrixOrbital(30);		  //Draw the Matrix Orbital Screen
	uint8_t pressed = 0;

        mo_usleep(20000);

	while (1)
	{
		uint8_t Tag = rd8(REG_TOUCH_TAG + RAM_REG);                    // Check for touches
		switch (Tag)
		{
			case 1:
				if (!pressed)
				{
					MakeScreen_MatrixOrbital(120); //Blue dot is 120 when not touched
					pressed = 1;
                                        mo_usleep(20000);
				}
				break;
			default:
				if (pressed)
				{
					pressed = 0;
					MakeScreen_MatrixOrbital(30); //Blue dot size is 30 when not touched
                                        mo_usleep(20000);
				}
				break;
		}		
        mo_usleep(20000);
	}
	HAL_Close();
}



void eve_test1()
{
  ClearScreen();
  eve_security_page();
}


void eve_test2()
{
  //Wait4CoProFIFOEmpty();
  //Cmd_SetRotate(5);
  //Wait4CoProFIFOEmpty();

  eve_initial_screen();

  while (1) {
    uint8_t Tag = rd8(REG_TOUCH_TAG + RAM_REG);                    // Check for touches
    if (Tag > 0) {
      eve_security_page();
      break;
    } else {
      mo_usleep(10000);
    }
  }

  eve_doors_page(0x7);
  mo_usleep(2000000);
  eve_test3();

}

static void eve_test3()
{
  const char *cylstr[] = {"1", "2", "3", "4", "5", "6"};
  const char *fpstr[] = {"Coilpack-L", "Coilpack-R", "Fuelpump-L", "Fuelpump-R"};
  int sts;
  
#ifdef OLD_STUBOUT
  while (1) {
    sts = eve_even_live(6, cylstr, DWIDTH);
    sts = eve_even_live(4, fpstr, DWIDTH);
  }
#endif
}

void eve_test4()
{
  uint8_t stuff[] = {0x0, 0x1, 0x3, 0x7, 0x2, 0x5, 0x4};
  uint8_t i;

  while (1) {
    for (i = 0; i < DIMENSION(stuff); i++) {
      eve_doors_page(stuff[i]);
      mo_usleep(2000000);
      }
    }
}


void eve_main(int test)
{

  BT81x_Init();

  switch(test) {
    case 1:
      eve_test1();
      break;

    case 2:
      eve_test2();
      break;

    case 3:
      eve_test3();
      break;

    case 4:
      eve_test4();
      break;

    default:
      eve_standard_test();
      break;
  }
}
