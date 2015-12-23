/*
 * wiringPi:
 *	Arduino compatable (ish) Wiring library for the Raspberry Pi
 *	Copyright (c) 2012 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose


#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(1==2)
#endif

// Environment Variables

#define	ENV_DEBUG	"WIRINGPI_DEBUG"
#define	ENV_CODES	"WIRINGPI_CODES"


// Mask for the bottom 64 pins which belong to the Banana Pro
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

struct wiringPiNodeStruct *wiringPiNodes = NULL ;


// Access from ARM Running Linux
//	Taken from Gert/Doms code. Some of this is not in the manual
//	that I can find )-:

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

// Locals to hold pointers to the hardware

static volatile uint32_t *gpio ;
static volatile uint32_t *pwm ;
static volatile uint32_t *clk ;
static volatile uint32_t *pads ;

#ifdef	USE_TIMER
static volatile uint32_t *timer ;
static volatile uint32_t *timerIrqRaw ;
#endif

/*add for BananaPro by LeMaker team*/
// for mmap BananaPro
#define	MAX_PIN_NUM		(0x40)  //64
#define SUNXI_GPIO_BASE (0x01C20800)
#define MAP_SIZE	(4096*2)
#define MAP_MASK	(MAP_SIZE - 1)
//sunxi_pwm
#define SUNXI_PWM_BASE (0x01c20e00)
#define SUNXI_PWM_CTRL_REG  (SUNXI_PWM_BASE)
#define SUNXI_PWM_CH0_PERIOD  (SUNXI_PWM_BASE + 0x4)
#define SUNXI_PWM_CH1_PERIOD  (SUNXI_PWM_BASE + 0x8)

#define SUNXI_PWM_CH0_EN			(1 << 4)
#define SUNXI_PWM_CH0_ACT_STA		(1 << 5)
#define SUNXI_PWM_SCLK_CH0_GATING	(1 << 6)
#define SUNXI_PWM_CH0_MS_MODE		(1 << 7) //pulse mode
#define SUNXI_PWM_CH0_PUL_START		(1 << 8)

#define SUNXI_PWM_CH1_EN			(1 << 19)
#define SUNXI_PWM_CH1_ACT_STA		(1 << 20)
#define SUNXI_PWM_SCLK_CH1_GATING	(1 << 21)
#define SUNXI_PWM_CH1_MS_MODE		(1 << 22) //pulse mode
#define SUNXI_PWM_CH1_PUL_START		(1 << 23)


#define PWM_CLK_DIV_120 	        0
#define PWM_CLK_DIV_180		1
#define PWM_CLK_DIV_240		2
#define PWM_CLK_DIV_360		3
#define PWM_CLK_DIV_480		4
#define PWM_CLK_DIV_12K		8
#define PWM_CLK_DIV_24K		9
#define PWM_CLK_DIV_36K		10
#define PWM_CLK_DIV_48K		11
#define PWM_CLK_DIV_72K		12

#define GPIO_PADS_BP		(0x00100000)
#define CLOCK_BASE_BP		(0x00101000)
//	addr should 4K*n
#define GPIO_BASE_BP		(0x01C20000)
#define GPIO_TIMER_BP		(0x0000B000)
#define GPIO_PWM_BP		(0x01c20000)  //need 4k*n

static int wiringPinMode = WPI_MODE_UNINITIALISED ;
int wiringPiCodes = FALSE ;
/*end 2014.09.18*/



//add for S500
#define	S500_MAX_PIN_NUM	(0x40)  //64
#define S500_GPIO_PADS	(0x00100000)
#define S500_CLOCK_BASE	(0xB0160000)
#define S500_GPIO_BASE	(0xB01B0000)
#define S500_GPIO_TIMER	(0xB0168000)
#define S500_GPIO_PWM	(0xB01B0050)


static int s500_physToGpio [64] =
{
    -1,           //0
    -1,     -1,   //1        2
    131,    -1,   //3(SDA2), 4
    130,    -1,   //5(SCK2), 6
    50,     91,   //7(B18),  8(UART0_TX)
    -1,     90,   //9,       10(UART0_RX)
    64,     40,   //11(C0),  12(B8-PWM)
    65,    -1,    //13(C1),  14
    68,     25,   //15(C4),  16(A25)
    -1,     70,   //17,      18(C6)
    89,     -1,   //19(MOSI),20
    88,     69,   //21(MISO),22(C5)
    86,     87,   //23(SCLK),24(SS)
    -1,     51,   //25,      26(GPIOB19)
    48,     46,   //27(B16), 28(B14)
    47,     -1,   //29(B15), 30
    42,     45,   //31(B10), 32(B13)
    32,     -1,   //33(B0),  34
    33,     28,   //35(B1),  36(A28)
    34,     31,   //37(B2),  38(A31)
    -1,     27,   //39,      40(A27)
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
    -1, -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;

static int s500_pinToGpio [64] =
{
    64, 40, 65, 68, 25, 70, 69, 50,    //WPI MAP: 0-7
    131, 130,   			     // I2C  - SDA0, SCL0 wpi  8 -  9
    87, 51,    // SPI  - CE1, CE0	 wpi 10 - 11
    89, 88, 86, // SPI  - MOSI, MISO, SCLK	wpi 12 - 14
    91, 90,    // UART - Tx, Rx	wpi 15 - 16
    -1, -1, -1, -1, // Rev 2: New GPIOs 8 though 11 wpi 17 - 20
    47, 42, 32, 33, 34,	// B+ wpi 21, 22, 23, 24, 25
    45, 28, 31, 27,	// B+ wpi 26, 27, 28, 29
    48, 46,	        // B+ wpi 30, 31

    // Padding:

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,// ... 47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,// ... 63
} ;

static int s500_pinTobcm [64] =
{
    48, 46,  //map to BCM GPIO0,1
    131, 130, //map to BCM GPIO2,3
    50, 47,  //map to BCM GPIO4,5
    42, 51,  //map to BCM GPIO6,7
    87, 88,  //map to BCM GPIO8,9
    89, 86,  //map to BCM GPIO10,11
    45, 32,  //map to BCM GPIO12,13
    91, 90,  //map to BCM GPIO14,15
    28, 64,  //map to BCM GPIO16,17
    40, 33,  //map to BCM GPIO18,19
    31, 27,  //map to BCM GPIO20,21
    68, 25,  //map to BCM GPIO22,23
    70, 69,  //map to BCM GPIO24,25
    34, 65,  //map to BCM GPIO26,27
    -1, -1,  //map to BCM GPIO28,29
    -1, -1,  //map to BCM GPIO30,31
    -1, -1,  //map to BCM GPIO32,33
    -1, -1,  //map to BCM GPIO34,35
    -1, -1,  //map to BCM GPIO36,37
    -1, -1,  //map to BCM GPIO38,39
    -1, -1,  //map to BCM GPIO40
    -1, -1,  //28... 43
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //44... 59
    -1, -1, -1, -1 // ...63
} ;



static int s500ValidGpio [132] =
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //0-> 15
    -1, -1, -1, -1, -1, -1, -1, -1, -1, 25, -1, 27, 28, -1, -1,  31, //16-> 31
    32, 33, 34, -1, -1, -1, -1, -1, 40, 41, 42, -1, -1, 45, 46, 47, //32-> 47
    48, -1, 50, 51, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //48-> 63
    64, 65, -1, -1, 68, 69, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //64-> 79
    -1, -1, -1, -1, -1, -1, 86, 87, 88, 89, 90, 91, -1, -1, -1, -1, //80-> 95
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //96-> 111
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //112-> 127
    -1, -1, 130, 131   //128->131
} ;




// Data for use with the boardId functions.
//	The order of entries here to correspond with the PI_MODEL_X
//	and PI_VERSION_X defines in wiringPi.h
//	Only intended for the gpio command - use at your own risk!

const char *piModelNames [6] =
{
    "Unknown",
    "Model A",
    "Model B",
    "Model B+",
    "Compute Module",
    "Banana Pro",    //add for BananaPro by LeMaker team
    "Guitar"
} ;

const char *piRevisionNames [5] =
{
    "Unknown",
    "1",
    "1.1",
    "1.2",
    "2",
} ;

const char *piMakerNames [5] =
{
    "Unknown",
    "Egoman",
    "Sony",
    "Qusda",
    "LeMaker", //add for BananaPro by LeMaker team
} ;


// Time for easy calculations

static uint64_t epochMilli, epochMicro ;

// Misc

static int wiringPiMode = WPI_MODE_UNINITIALISED ;
static volatile int    pinPass = -1 ;
static pthread_mutex_t pinMutex ;

// Debugging & Return codes

int wiringPiDebug       = FALSE ;
int wiringPiReturnCodes = FALSE ;

// sysFds:
//	Map a file descriptor from the /sys/class/gpio/gpioX/value

static int sysFds [64] =
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;


static int s500_sysFds [132] =
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1
} ;



// ISR Data

static void (*isrFunctions [64])(void) ;


// Doing it the Arduino way with lookup tables...
//	Yes, it's probably more innefficient than all the bit-twidling, but it
//	does tend to make it all a bit clearer. At least to me!

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//	Cope for 3 different board revisions here.

static int *pinToGpio ;



// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

static int *physToGpio ;


/*add for BananaPro by LeMaker team*/
//map tableb for BP

static int *physToPin ;

static int upDnConvert[3] = {0, 2, 1};

static int pinToGpio_BP [64] =
{
    275, 259,
    274, 273,
    244, 245,
    272, 226,
    53, 52,
    266, 270,
    268, 269,
    267, 228,
    229, -1,
    -1, -1,
    -1,  35,
    277, 45,
    39, 37,
    276, 38,
    44, 40,
    257, 256,     // ...31

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,// ... 63
} ;

static int pinTobcm_BP [64] =
{
    257, 256,  //map to BCM GPIO0,1
    53, 52,      //map to BCM GPIO2,3
    226, 35,    //map to BCM GPIO4,5
    277, 270,  //map to BCM GPIO6,7
    266, 269,  //map to BCM GPIO8,9
    268, 267,  //map to BCM GPIO10,11
    276, 45,     //map to BCM GPIO12,13
    228, 229,  //map to BCM GPIO14,15
    38, 275,   //map to BCM GPIO16,17
    259, 39,   //map to BCM GPIO18,19
    44, 40,     //map to BCM GPIO20,21
    273, 244,   //map to BCM GPIO22,23
    245, 272,   //map to BCM GPIO24,25
    37, 274,     //map to BCM GPIO26,27

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 29... 44
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //45... 60
    -1, -1, -1, -1                                                                   // ...63
} ;
static int physToGpio_BP [64] =
{
    -1,          // 0
    -1,     -1,     //1, 2
    53,    -1,     //3, 4
    52,    -1,     //5, 6
    226,  228,   //7, 8
    -1,     229,   //9, 10
    275,   259,   //11, 12
    274,   -1,     //13, 14
    273,   244,   //15, 16
    -1,     245,   //17, 18
    268,   -1,     //19, 20
    269,  272,   //21, 22
    267,   266,   //23, 24
    -1,    270,   //25, 26
    257,   256,  //27, 28
    35,    -1,    //29, 30
    277,  276,   //31, 32
    45,   -1,      //33, 34
    39,   38,    //35, 36
    37,   44,     //37, 38
    -1,   40,    //39, 40
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
    -1, -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;


static int syspin [64] =
{
    -1, -1, 2, 3, 4, 5, 6, 7,   //GPIO0,1 used to I2C
    8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

static int edge [64] =
{
    -1, -1, -1, -1, 4, -1, -1, 7,   //support the INT
    8, 9, 10, 11, -1, -1, 14, 15,
    -1, 17, -1, -1, -1, -1, 22, 23,
    24, 25, -1, 27, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

static int pinToGpioR [64] =
{
    17, 18, 27, 22, 23, 24, 25, 4,	// From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
    2,  3,				// I2C  - SDA0, SCL0				wpi  8 -  9
    8,  7,				// SPI  - CE1, CE0				wpi 10 - 11
    10,  9, 11, 				// SPI  - MOSI, MISO, SCLK			wpi 12 - 14
    14, 15,				// UART - Tx, Rx				wpi 15 - 16
    -1, -1, -1, -1,			// Rev 2: New GPIOs 8 though 11			wpi 17 - 20
    5,  6, 13, 19, 26,			// B+						wpi 21, 22, 23, 24, 25
    12, 16, 20, 21,			// B+						wpi 26, 27, 28, 29
    0,  1,				// B+						wpi 30, 31

    // Padding:

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;

static int physToGpioR [64] =//head num map to BCMpin
{
    -1,		// 0
    -1, -1,	// 1, 2
    2, -1,
    3, -1,
    4, 14,
    -1, 15,
    17, 18,
    27, -1,
    22, 23,
    -1, 24,
    10, -1,
    9, 25,
    11,  8,
    -1,  7,	// 25, 26

    0,   1,   //27, 28
    5,  -1,  //29, 30
    6,  12,  //31, 32
    13, -1, //33, 34
    19, 16, //35, 36
    26, 20, //37, 38
    -1, 21, //39, 40
    // Padding:

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 56
    -1, -1, -1, -1, -1, -1, -1, 	// ... 63
} ;

static int physToPinR3 [64] = //return wiringPI pin
{
    -1,		// 0
    -1, -1,	// 1, 2
    8, -1,  //3, 4
    9, -1,  //5, 6
    7, 15,  //7, 8
    -1, 16, //9,10
    0, 1, //11,12
    2, -1, //13,14
    3, 4, //15,16
    -1, 5, //17,18
    12, -1, //19,20
    13, 6, //21,22
    14, 10, //23, 24
    -1,  11,	// 25, 26

    30,   31,   //27, 28
    21,  -1,  //29, 30
    22,  26,  //31, 32
    23, -1, //33, 34
    24, 27, //35, 36
    25, 28, //37, 38
    -1, 29, //39, 40
    // Padding:

    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 56
    -1, -1, -1, -1, -1, -1, -1, 	// ... 63
} ;

static int BP_PIN_MASK[9][32] =  //[BANK]  [INDEX]
{
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PA
    { -1, -1, -1, 3, -1,  5,  6, 7,  8, -1, -1, -1, 12, 13, -1, -1, -1, -1, -1, -1, 20, 21, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PB
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PC
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PD
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PE
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PF
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PG
    { -1, -1,  2, -1, 4,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 20, 21, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PH
    {0, 1, -1, 3, -1, -1, -1, -1, -1, -1, 10, 11, 12, 13, 14, -1, 16, 17, 18, 19, 20, 21, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PI
};
static int version = 0;
static int pwmmode = 0;

/*end 20140918*/

/*
 * Functions
 *********************************************************************************
 */

/*add for BananaPro by LeMaker team*/
uint32_t sunxi_readl(uint32_t addr)
{
    uint32_t val = 0;
    uint32_t mmap_base = (addr & ~MAP_MASK);
    uint32_t mmap_seek = ((addr - mmap_base) >> 2);
    val = *(gpio + mmap_seek);
    return val;

}
void sunxi_writel(uint32_t val, uint32_t addr)
{
    uint32_t mmap_base = (addr & ~MAP_MASK);
    uint32_t mmap_seek = ((addr - mmap_base) >> 2);
    *(gpio + mmap_seek) = val;
}

static uint32_t s500_readl(uint32_t *addr)
{
    uint32_t val = 0;

    val = *addr;
    return val;

}
static void s500_writel(uint32_t val, uint32_t *addr)
{
    *addr = val;
}


//pwm for BananaPro only for pwm1
void sunxi_pwm_set_enable(int en)
{
    int val = 0;
    val = sunxi_readl(SUNXI_PWM_CTRL_REG);
    if(en)
    {
        val |= (SUNXI_PWM_CH1_EN | SUNXI_PWM_SCLK_CH1_GATING);
    }
    else
    {
        val &= ~(SUNXI_PWM_CH1_EN | SUNXI_PWM_SCLK_CH1_GATING);
    }
    if (wiringPiDebug)
        printf(">>function%s,no:%d,enable? :0x%x\n", __func__, __LINE__, val);
    sunxi_writel(val, SUNXI_PWM_CTRL_REG);
    delay (1) ;
}

void s500_pwm_set_enable(int en)
{
    uint32_t val = 0;
    uint32_t *phyaddr = NULL;

    //Disable output
    phyaddr = gpio + (0x000C >> 2);
    val = s500_readl(phyaddr);
    val &= ~(0x100);
    s500_writel(val, phyaddr);

    //Disable input
    phyaddr = gpio + (0x0010 >> 2);
    val = s500_readl(phyaddr);
    val &= ~(0x100);
    s500_writel(val, phyaddr);

    //MFP_CTL1
    phyaddr = gpio + (0x0044 >> 2);
    val = s500_readl(phyaddr);
    val &= 0xFC7FFFFF;
    val |= 0x01800000;
    s500_writel(val, phyaddr);

}

void sunxi_pwm_set_mode(int mode)
{
    int val = 0;
    val = sunxi_readl(SUNXI_PWM_CTRL_REG);
    mode &= 1; //cover the mode to 0 or 1
    if(mode)
    {
        //pulse mode
        val |= ( SUNXI_PWM_CH1_MS_MODE | SUNXI_PWM_CH1_PUL_START);
        pwmmode = 1;
    }
    else
    {
        //cycle mode
        val &= ~( SUNXI_PWM_CH1_MS_MODE);
        pwmmode = 0;
    }
    val |= ( SUNXI_PWM_CH1_ACT_STA);
    if (wiringPiDebug)
        printf(">>function%s,no:%d,mode? :0x%x\n", __func__, __LINE__, val);
    sunxi_writel(val, SUNXI_PWM_CTRL_REG);
    delay (1) ;
}

void s500_pwm_set_mode(int mode)
{
    //Nothing
}


void sunxi_pwm_set_clk(int clk)
{
    int val = 0;

    // sunxi_pwm_set_enable(0);
    val = sunxi_readl(SUNXI_PWM_CTRL_REG);
    //clear clk to 0
    val &= 0xf801f0;
    val |= ((clk & 0xf) << 15);  //todo check wether clk is invalid or not
    sunxi_writel(val, SUNXI_PWM_CTRL_REG);
    sunxi_pwm_set_enable(1);
    if (wiringPiDebug)
        printf(">>function%s,no:%d,clk? :0x%x\n", __func__, __LINE__, val);
    delay (1) ;
}

void s500_pwm_set_clk_source(int select)
{
    uint32_t regval = 0;
    uint32_t *phyaddr = clk + (0x007C >> 2);

    regval = s500_readl(phyaddr);
    if(select == 0)//IC_32K
    {
        regval &= ~(1 << 12 );
    }
    else//HOSC 24M
    {
        regval |= (1 << 12);
    }

    s500_writel(regval, phyaddr);

    if(wiringPiDebug)
    {
        printf(">>function%s,no:%d,clk sel :0x%x\n", __func__, __LINE__, regval);
    }

    delay (1);
}

/*
 clk_div 范围: 0~1023
*/
void s500_pwm_set_clk(int clk_div)
{
    uint32_t regval = 0;
    int temp;
    uint32_t *phyaddr = clk + (0x007C >> 2);

    regval = s500_readl(phyaddr);
    regval &= (1 << 12);

    temp = clk_div;
    temp |=  regval;
    s500_writel(temp, phyaddr);

    if(wiringPiDebug)
    {
        printf(">>function%s,no:%d,clk sel :0x%x\n", __func__, __LINE__, temp);
    }

    delay (1);
}



/**
 * ch0 and ch1 set the same,16 bit period and 16 bit act
 */
uint32_t sunxi_pwm_get_period(void)
{
    uint32_t period_cys = 0;
    period_cys = sunxi_readl(SUNXI_PWM_CH1_PERIOD);//get ch1 period_cys
    period_cys &= 0xffff0000;//get period_cys
    period_cys = period_cys >> 16;
    if (wiringPiDebug)
        printf(">>func:%s,no:%d,period/range:%d", __func__, __LINE__, period_cys);
    delay (1) ;
    return period_cys;
}


uint32_t s500_pwm_get_period(void)
{
    uint32_t period = 0;
    uint32_t *phyaddr = gpio + ((0x0050 + 4 * 3) >> 2);
    period = s500_readl(phyaddr);
    period &= 0x3FF;

    if (wiringPiDebug)
    {
        printf(">>func:%s,no:%d,period/range:%d", __func__, __LINE__, period);
    }

    return period;
}


uint32_t sunxi_pwm_get_act(void)
{
    uint32_t period_act = 0;
    period_act = sunxi_readl(SUNXI_PWM_CH1_PERIOD);//get ch1 period_cys
    period_act &= 0xffff;//get period_act
    if (wiringPiDebug)
        printf(">>func:%s,no:%d,period/range:%d", __func__, __LINE__, period_act);
    delay (1) ;
    return period_act;
}


uint32_t s500_pwm_get_act(void)
{
    uint32_t act = 0;
    uint32_t *phyaddr = gpio + ((0x0050 + 4 * 3) >> 2);
    act = s500_readl(phyaddr);
    act &= 0xFFC00;
    act >>= 10;

    if (wiringPiDebug)
    {
        printf(">>func:%s,no:%d,act:%d", __func__, __LINE__, act);
    }

    return act;
}




void sunxi_pwm_set_period(int period_cys)
{
    uint32_t val = 0;
    //all clear to 0
    if (wiringPiDebug)
        printf(">>func:%s no:%d\n", __func__, __LINE__);
    period_cys &= 0xffff; //set max period to 2^16
    period_cys = period_cys << 16;
    val = sunxi_readl(SUNXI_PWM_CH1_PERIOD);
    val &= 0x0000ffff;
    period_cys |= val;
    sunxi_writel(period_cys, SUNXI_PWM_CH1_PERIOD);
    delay (1) ;

}


void s500_pwm_set_period(int period)
{
    uint32_t val = 0;
    uint32_t *phyaddr = gpio + ((0x0050 + 4 * 3) >> 2);

    period &= 0x3FF; //set max period to 2^10
    val = s500_readl(phyaddr);
    val &= 0x1FFC00;
    period |= val;
    s500_writel(period, phyaddr);

    if (wiringPiDebug)
    {
        printf(">>func:%s,no:%d,period/range:%d\n", __func__, __LINE__, period);
    }

    delay (1);
}



void sunxi_pwm_set_act(int act_cys)
{
    uint32_t per0 = 0;
    //keep period the same, clear act_cys to 0 first
    if (wiringPiDebug)
        printf(">>func:%s no:%d\n", __func__, __LINE__);
    per0 = sunxi_readl(SUNXI_PWM_CH1_PERIOD);
    per0 &= 0xffff0000;
    act_cys &= 0xffff;
    act_cys |= per0;
    sunxi_writel(act_cys, SUNXI_PWM_CH1_PERIOD);
    delay (1) ;
}


void s500_pwm_set_act(int act)
{
    uint32_t val = 0;
    uint32_t *phyaddr = gpio + ((0x0050 + 4 * 3) >> 2);

    act &= 0x3FF; //set max period to 2^10
    val = s500_readl(phyaddr);
    val &= 0x1003FF;
    act <<= 10;
    act |= val;
    s500_writel(act, phyaddr);

    if (wiringPiDebug)
    {
        printf(">>func:%s,no:%d,act:%d\n", __func__, __LINE__, act);
    }

    delay (1);
}


uint32_t s500_pwm_get_polarity(void)
{
    uint32_t val = 0;
    uint32_t *phyaddr = gpio + ((0x0050 + 4 * 3) >> 2);

    val = s500_readl(phyaddr);
    val &= 0x100000;
    val >>= 20;

    if (wiringPiDebug)
    {
        printf(">>func:%s,no:%d,act:%d\n", __func__, __LINE__, val);
    }

    delay (1);

    return val;
}



/*
**  Polarity select
**  0:PWM low voltage level active
**  1:PWM high voltage level active
*/
uint32_t s500_pwm_set_polarity(int act)
{
    uint32_t val = 0;
    uint32_t *phyaddr = gpio + ((0x0050 + 4 * 3) >> 2);

    act &= 0x01; //range :0~1
    val = s500_readl(phyaddr);
    val &= 0x0FFFFF;
    act <<= 20;
    act |= val;
    s500_writel(act, phyaddr);

    if (wiringPiDebug)
    {
        printf(">>func:%s,no:%d,act:%d\n", __func__, __LINE__, act);
    }

    delay (1);
}




int sunxi_get_gpio_mode(int pin)
{
    uint32_t regval = 0;
    int bank = pin >> 5;
    int index = pin - (bank << 5);
    int offset = ((index - ((index >> 3) << 3)) << 2);
    uint32_t reval = 0;
    uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);
    if (wiringPiDebug)
        printf("func:%s pin:%d,  bank:%d index:%d phyaddr:0x%x\n", __func__, pin , bank, index, phyaddr);
    if(BP_PIN_MASK[bank][index] != -1)
    {
        regval = sunxi_readl(phyaddr);
        if (wiringPiDebug)
            printf("read reg val: 0x%x offset:%d  return: %d\n", regval, offset, reval);
        //reval=regval &(reval+(7 << offset));
        reval = (regval >> offset) & 7;
        if (wiringPiDebug)
            printf("read reg val: 0x%x offset:%d  return: %d\n", regval, offset, reval);
        return reval;
    }
    else
    {
        printf("line:%dpin number error\n", __LINE__);
        return reval;
    }
}


void sunxi_set_gpio_mode(int pin, int mode)
{
    uint32_t regval = 0;
    int bank = pin >> 5;
    int index = pin - (bank << 5);
    int offset = ((index - ((index >> 3) << 3)) << 2);
    uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);
    if (wiringPiDebug)
        printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x\n", __func__, pin , mode, bank, index, phyaddr);
    if(BP_PIN_MASK[bank][index] != -1)
    {
        regval = sunxi_readl(phyaddr);
        if (wiringPiDebug)
            printf("read reg val: 0x%x offset:%d\n", regval, offset);
        if(INPUT == mode)
        {
            regval &= ~(7 << offset);
            sunxi_writel(regval, phyaddr);
            regval = sunxi_readl(phyaddr);
            if (wiringPiDebug)
                printf("Input mode set over reg val: 0x%x\n", regval);
        }
        else if(OUTPUT == mode)
        {
            regval &= ~(7 << offset);
            regval |=  (1 << offset);
            if (wiringPiDebug)
                printf("Out mode ready set val: 0x%x\n", regval);
            sunxi_writel(regval, phyaddr);
            regval = sunxi_readl(phyaddr);
            if (wiringPiDebug)
                printf("Out mode set over reg val: 0x%x\n", regval);
        }
        else if(PWM_OUTPUT == mode)
        {
            // set pin PWMx to pwm mode
            regval &= ~(7 << offset);
            regval |=  (0x2 << offset);
            if (wiringPiDebug)
                printf(">>>>>line:%d PWM mode ready to set val: 0x%x\n", __LINE__, regval);
            sunxi_writel(regval, phyaddr);
            delayMicroseconds (200);
            regval = sunxi_readl(phyaddr);
            if (wiringPiDebug)
                printf("<<<<<PWM mode set over reg val: 0x%x\n", regval);
            //clear all reg
            sunxi_writel(0, SUNXI_PWM_CTRL_REG);
            sunxi_writel(0, SUNXI_PWM_CH0_PERIOD);
            sunxi_writel(0, SUNXI_PWM_CH1_PERIOD);

            //set default M:S to 1/2
            sunxi_pwm_set_period(1024);
            sunxi_pwm_set_act(512);
            pwmSetMode(PWM_MODE_MS);
            sunxi_pwm_set_clk(PWM_CLK_DIV_120);//default clk:24M/120
            delayMicroseconds (200);
        }
    }
    else
    {
        printf("line:%dpin number error\n", __LINE__);
    }

    return ;
}
void sunxi_digitalWrite(int pin, int value)
{
    uint32_t regval = 0;
    int bank = pin >> 5;
    int index = pin - (bank << 5);
    uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
    if (wiringPiDebug)
        printf("func:%s pin:%d, value:%d bank:%d index:%d phyaddr:0x%x\n", __func__, pin , value, bank, index, phyaddr);
    if(BP_PIN_MASK[bank][index] != -1)
    {
        regval = sunxi_readl(phyaddr);
        if (wiringPiDebug)
            printf("befor write reg val: 0x%x,index:%d\n", regval, index);
        if(0 == value)
        {
            regval &= ~(1 << index);
            sunxi_writel(regval, phyaddr);
            regval = sunxi_readl(phyaddr);
            if (wiringPiDebug)
                printf("LOW val set over reg val: 0x%x\n", regval);
        }
        else
        {
            regval |= (1 << index);
            sunxi_writel(regval, phyaddr);
            regval = sunxi_readl(phyaddr);
            if (wiringPiDebug)
                printf("HIGH val set over reg val: 0x%x\n", regval);
        }
    }
    else
    {
        printf("pin number error\n");
    }

    return ;
}
int sunxi_digitalRead(int pin)
{
    uint32_t regval = 0;
    int bank = pin >> 5;
    int index = pin - (bank << 5);
    uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
    if (wiringPiDebug)
        printf("func:%s pin:%d,bank:%d index:%d phyaddr:0x%x\n", __func__, pin, bank, index, phyaddr);
    if(BP_PIN_MASK[bank][index] != -1)
    {
        regval = sunxi_readl(phyaddr);
        regval = regval >> index;
        regval &= 1;
        if (wiringPiDebug)
            printf("***** read reg val: 0x%x,bank:%d,index:%d,line:%d\n", regval, bank, index, __LINE__);
        return regval;
    }
    else
    {
        printf("pin number error\n");
        return regval;
    }
}
void sunxi_pullUpDnControl (int pin, int pud)
{
    uint32_t regval = 0;
    int bank = pin >> 5;
    int index = pin - (bank << 5);
    int sub = index >> 4;
    int sub_index = index - 16 * sub;
    uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x1c + sub * 4; // +0x10 -> pullUpDn reg
    if (wiringPiDebug)
        printf("func:%s pin:%d,bank:%d index:%d sub:%d phyaddr:0x%x\n", __func__, pin, bank, index, sub, phyaddr);
    if(BP_PIN_MASK[bank][index] != -1)
    {
        //PI13~PI21 need check again
        regval = sunxi_readl(phyaddr);
        if (wiringPiDebug)
            printf("pullUpDn reg:0x%x, pud:0x%x sub_index:%d\n", regval, pud, sub_index);
        regval &= ~(3 << (sub_index << 1));
        regval |= (pud << (sub_index << 1));
        if (wiringPiDebug)
            printf("pullUpDn val ready to set:0x%x\n", regval);
        sunxi_writel(regval, phyaddr);
        regval = sunxi_readl(phyaddr);
        if (wiringPiDebug)
            printf("pullUpDn reg after set:0x%x  addr:0x%x\n", regval, phyaddr);
    }
    else
    {
        printf("pin number error\n");
    }
    delay (1) ;
    return ;
}
/*end 2014.09.18*/

/*
 * wiringPiFailure:
 *	Fail. Or not.
 *********************************************************************************
 */

int wiringPiFailure (int fatal, const char *message, ...)
{
    va_list argp ;
    char buffer [1024] ;

    if (!fatal && wiringPiReturnCodes)
        return -1 ;

    va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
    va_end (argp) ;

    fprintf (stderr, "%s", buffer) ;
    exit (EXIT_FAILURE) ;

    return 0 ;
}


/*
 * piBoardRev:
 *	Return a number representing the hardware revision of the board.
 *********************************************************************************
 *    3 --- Banana Pro
 *    4 --- S500
 *
 *********************************************************************************
 */

static void piBoardRevOops (const char *why)
{
    fprintf (stderr, "piBoardRev: Unable to determine board revision from /proc/cpuinfo\n") ;
    fprintf (stderr, " -> %s\n", why) ;
    fprintf (stderr, " ->  You may want to check:\n") ;
    fprintf (stderr, " ->  http://www.lemaker.org/\n") ;  /*modify for BananaPro by LeMmaker team*/
    exit (EXIT_FAILURE) ;
}

/*add for BananaPro by LeMaker team*/
int isA20(void)
{
    FILE *cpuFd ;
    char line [120] ;
    char *d;
    if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
        piBoardRevOops ("Unable to open /proc/cpuinfo") ;
    while (fgets (line, 120, cpuFd) != NULL)
    {
        if (strncmp (line, "Hardware", 8) == 0)
            break ;
    }

    fclose (cpuFd) ;
    if (strncmp (line, "Hardware", 8) != 0)
        piBoardRevOops ("No \"Hardware\" line") ;

    for (d = &line [strlen (line) - 1] ; (*d == '\n') || (*d == '\r') ; --d)
        *d = 0 ;
    if (wiringPiDebug)
        printf ("piboardRev: Hardware string: %s\n", line) ;

    if (strstr(line, "sun7i") != NULL)
    {
        if (wiringPiDebug)
            printf ("Hardware:%s\n", line) ;
        return 1 ;
    }
    else
    {
        if (wiringPiDebug)
            printf ("Hardware:%s\n", line) ;
        return 0 ;
    }
}
/*end 2014.09.18*/

/*add for S500*/
int isS500(void)
{
    FILE *cpuFd ;
    char line [120] ;
    char *d;
    if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
    {
        piBoardRevOops ("Unable to open /proc/cpuinfo") ;
    }

    while (fgets (line, 120, cpuFd) != NULL)
    {
        if (strncmp (line, "Hardware", 8) == 0)
            break ;
    }

    fclose (cpuFd) ;

    if (strncmp (line, "Hardware", 8) != 0)
    {
        piBoardRevOops ("No \"Hardware\" line") ;
    }

    for (d = &line [strlen (line) - 1] ; (*d == '\n') || (*d == '\r') ; --d)
    {
        *d = 0 ;
    }

    if (wiringPiDebug)
    {
        printf ("piboardRev: Hardware string: %s\n", line) ;
    }

    if (strstr(line, "gs705a") != NULL)
    {
        if (wiringPiDebug)
            printf ("Hardware:%s\n", line) ;
        return 1 ;
    }
    else
    {
        if (wiringPiDebug)
            printf ("Hardware:%s\n", line) ;
        return 0 ;
    }
}


int piBoardRev (void)
{
    //add for S500
    if(isS500())
    {
        version = S500_REV;

        if (wiringPiDebug)
        {
            printf ("piboardRev:  %d\n", version) ;
        }

        return S500_REV ;
    }
    /*add for BananaPro by LeMaker team*/
    else if(isA20())
    {
        version = BP_REV;
        if (wiringPiDebug)
            printf ("piboardRev:  %d\n", version) ;
        return BP_REV ;
    }
    /*end 2014.09.18*/
    else
    {
        printf("piboardRev is error!!!\n");
    }

}


/*
 * piBoardId:
 *	Do more digging into the board revision string as above, but return
 *	as much details as we can.
 *	This is undocumented and really only intended for the GPIO command.
 *	Use at your own risk!
 *********************************************************************************
 */

void piBoardId (int *model, int *rev, int *mem, int *maker, int *overVolted)
{
    FILE *cpuFd ;
    char line [120] ;
    char *c ;

    (void)piBoardRev () ;	// Call this first to make sure all's OK. Don't care about the result.

    //add for s500
    if(version == S500_REV)
    {
        *model = PI_MODEL_BPR;
        *rev = PI_VERSION_1_2;
        *mem = 1024;
        *maker = PI_MAKER_LEMAKER;

        return;
    }


    if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
        piBoardRevOops ("Unable to open /proc/cpuinfo") ;

    while (fgets (line, 120, cpuFd) != NULL)
        if (strncmp (line, "Revision", 8) == 0)
            break ;

    fclose (cpuFd) ;

    if (strncmp (line, "Revision", 8) != 0)
        piBoardRevOops ("No \"Revision\" line") ;

    // Chomp trailing CR/NL

    for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
        *c = 0 ;

    if (wiringPiDebug)
        printf ("piboardId: Revision string: %s\n", line) ;

    // Scan to first digit

    for (c = line ; *c ; ++c)
        if (isdigit (*c))
            break ;

    // Make sure its long enough

    if (strlen (c) < 4)
        piBoardRevOops ("Bogus \"Revision\" line") ;

    // If longer than 4, we'll assume it's been overvolted

    *overVolted = strlen (c) > 4 ;

    // Extract last 4 characters:

    c = c + strlen (c) - 4 ;

    // Fill out the replys as appropriate

    //add for BananaPro by LeMaker team
    if (strcmp (c, "0000") == 0)
    {
        *model = PI_MODEL_BPR;
        *rev = PI_VERSION_1_2;
        *mem = 1024;
        *maker = PI_MAKER_LEMAKER;
    }
    //end 2014.09.30
    else
    {
        *model = 0           ;
        *rev = 0              ;
        *mem =   0 ;
        *maker = 0 ;
    }
}



/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int wpiPinToGpio (int wpiPin)
{
    return pinToGpio [wpiPin & 63] ;
}


/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int physPinToGpio (int physPin)
{
    return physToGpio [physPin & 63] ;
}

/*
 * physPinToGpio:
 *	Translate a physical Pin number to wiringPi  pin number. add by lemaker team for BananaPi
 *	Provided for external support.
 *********************************************************************************
 */
int physPinToPin(int physPin)
{
    return physToPin [physPin & 63] ;
}

/*
 * setPadDrive:
 *	Set the PAD driver value
 *********************************************************************************
 */

void setPadDrive (int group, int value)
{

    //add for s500
    if(version == S500_REV)
    {
        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if(BP_REV == version)
    {
        return;
    }
    /*end 2014.08.19*/
    else
    {

    }

}


/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 *********************************************************************************
 */

int getAlt (int pin)
{
    int alt ;

    pin &= 63 ;

    if(version  == S500_REV)
    {
        alt = s500_get_gpio_mode(pin);

        return alt ;
    }
    /*add for BananaPro by LeMaker team*/
    else if(BP_REV == version)
    {
        if (wiringPiMode == WPI_MODE_PINS)
            pin = pinToGpio_BP [pin] ;
        else if (wiringPiMode == WPI_MODE_PHYS)
            pin = physToGpio_BP[pin] ;
        else if (wiringPiMode == WPI_MODE_GPIO)
            pin = pinTobcm_BP[pin]; //need map A20 to bcm
        else return 0 ;

        if(-1 == pin)
        {
            printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
            return -1;
        }
        alt = sunxi_get_gpio_mode(pin);
        return alt ;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error!!!");
    }

}


/*
 * pwmSetMode:
 *	Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */

void pwmSetMode (int mode)
{
    /* add for S500 */
    if(version == S500_REV)
    {
        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if (BP_REV == version)
    {
        sunxi_pwm_set_mode(mode);
        return;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error!!!");
    }

}


/*
 * pwmSetRange:
 *	Set the PWM range register. We set both range registers to the same
 *	value. If you want different in your own code, then write your own.
 *********************************************************************************
 */

void pwmSetRange (unsigned int range)
{
    /* add for S500 */
    if(version == S500_REV)
    {
        s500_pwm_set_period(range);
        delayMicroseconds (10) ;
        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if (BP_REV == version)
    {
        sunxi_pwm_set_period(range);
        return;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error!!!");
    }
}


/*
 * pwmSetClock:
 *	Set/Change the PWM clock. Originally my code, but changed
 *	(for the better!) by Chris Hall, <chris@kchall.plus.com>
 *	after further study of the manual and testing with a 'scope
 *********************************************************************************
 */

void pwmSetClock (int divisor)
{
    uint32_t pwm_control ;

    /* add for S500 */
    if(version == S500_REV)
    {
        s500_pwm_set_clk(divisor);
        delayMicroseconds (10) ;
        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if (BP_REV == version)
    {
        sunxi_pwm_set_clk(divisor);
        sunxi_pwm_set_enable(1);
        return;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error!!!");
    }

}


/*
 * gpioClockSet:
 *	Set the freuency on a GPIO clock pin
 *********************************************************************************
 */

void gpioClockSet (int pin, int freq)
{

    /* add for S500 */
    if(version == S500_REV)
    {
        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if (BP_REV == version)
    {
        return;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error!!!");
    }

}


/*
 * wiringPiFindNode:
 *      Locate our device node
 *********************************************************************************
 */

struct wiringPiNodeStruct *wiringPiFindNode (int pin)
{
    struct wiringPiNodeStruct *node = wiringPiNodes ;

    while (node != NULL)
        if ((pin >= node->pinBase) && (pin <= node->pinMax))
            return node ;
        else
            node = node->next ;

    return NULL ;
}


/*
 * wiringPiNewNode:
 *	Create a new GPIO node into the wiringPi handling system
 *********************************************************************************
 */

static void pinModeDummy             (struct wiringPiNodeStruct *node, int pin, int mode)
{
    return ;
}
static void pullUpDnControlDummy     (struct wiringPiNodeStruct *node, int pin, int pud)
{
    return ;
}
static int  digitalReadDummy         (struct wiringPiNodeStruct *node, int pin)
{
    return LOW ;
}
static void digitalWriteDummy        (struct wiringPiNodeStruct *node, int pin, int value)
{
    return ;
}
static void pwmWriteDummy            (struct wiringPiNodeStruct *node, int pin, int value)
{
    return ;
}
static int  analogReadDummy          (struct wiringPiNodeStruct *node, int pin)
{
    return 0 ;
}
static void analogWriteDummy         (struct wiringPiNodeStruct *node, int pin, int value)
{
    return ;
}

struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins)
{
    int    pin ;
    struct wiringPiNodeStruct *node ;

    // Minimum pin base is 64

    if (pinBase < 64)
        (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

    // Check all pins in-case there is overlap:

    for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
        if (wiringPiFindNode (pin) != NULL)
            (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

    node = (struct wiringPiNodeStruct *)calloc (sizeof (struct wiringPiNodeStruct), 1) ;	// calloc zeros
    if (node == NULL)
        (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

    node->pinBase         = pinBase ;
    node->pinMax          = pinBase + numPins - 1 ;
    node->pinMode         = pinModeDummy ;
    node->pullUpDnControl = pullUpDnControlDummy ;
    node->digitalRead     = digitalReadDummy ;
    node->digitalWrite    = digitalWriteDummy ;
    node->pwmWrite        = pwmWriteDummy ;
    node->analogRead      = analogReadDummy ;
    node->analogWrite     = analogWriteDummy ;
    node->next            = wiringPiNodes ;
    wiringPiNodes         = node ;

    return node ;
}


#ifdef notYetReady
/*
 * pinED01:
 * pinED10:
 *	Enables edge-detect mode on a pin - from a 0 to a 1 or 1 to 0
 *	Pin must already be in input mode with appropriate pull up/downs set.
 *********************************************************************************
 */

void pinEnableED01Pi (int pin)
{
    pin = pinToGpio [pin & 63] ;
}
#endif


/*
 *********************************************************************************
 * Core Functions
 *********************************************************************************
 */

/*
 * pinModeAlt:
 *	This is an un-documented special to let you set any pin to any mode
 *********************************************************************************
 */

void pinModeAlt (int pin, int mode)
{

    /*add for BananaPro by LeMaker team*/
    if (BP_REV == version || S500_REV == version)
    {
        return;
    }
    /*end 2014.08.19*/

}


int s500_get_gpio_mode(int pin)
{
    uint32_t regval = 0;
    int bank = pin >> 5;
    uint32_t *phyaddr = NULL;

    if ((pin & PI_GPIO_MASK) == 0)	// On-board pin
    {
        if (wiringPiMode == WPI_MODE_PHYS)
        {
            pin = s500_physToGpio[pin] ;
        }
        else if(wiringPiMode == WPI_MODE_PINS)
        {
            pin = s500_pinToGpio[pin];
        }
        else if(wiringPiMode == WPI_MODE_GPIO)
        {
            pin = s500_pinTobcm[pin];
        }
        else
        {
            return 0;
        }

        if(pin != -1)
        {
            //Input
            phyaddr = gpio + (bank * 3) + 0x01; // +0x04(Byte) -> Input Enable Register
            regval = s500_readl(phyaddr);
            if(regval == 1)
            {
                if(wiringPiDebug)
                {
                    printf("func:%s pin:%d, input reval:0x%x\n", __func__, pin , regval);
                }

                return 0;
            }

            //Output
            phyaddr = gpio + (bank * 3) + 0x00; // +0x00 -> Output Enable Register
            regval = s500_readl(phyaddr);
            if(regval == 1)
            {
                if(wiringPiDebug)
                {
                    printf("func:%s pin:%d, output reval:0x%x\n", __func__, pin , regval);
                }

                return 1;
            }

            //other function
            return 4;
        }

    }
    else
    {
        printf("line:%dpin number error\n", __LINE__);
        return -1;
    }

}




static void s500_set_gpio_mode(int pin, int mode)
{
    uint32_t regval = 0;


    if ((pin & PI_GPIO_MASK) == 0)	// On-board pin
    {
        if (wiringPiMode == WPI_MODE_PHYS)
        {
            pin = s500_physToGpio[pin] ;
        }
        else if(wiringPiMode == WPI_MODE_PINS)
        {
            pin = s500_pinToGpio[pin];
        }
        else if(wiringPiMode == WPI_MODE_GPIO)
        {
            pin = s500_pinTobcm[pin];
        }
        else
        {
            return;
        }


        if(pin != -1)
        {
            int bank = pin >> 5;
            int index = pin - (bank << 5);
            uint32_t *phyaddr = NULL;

            //LVDS信号需要先转为数字信号才能使用

            if(gpio == 42 || gpio == 45 || gpio == 46 || gpio == 47 || gpio == 48 || gpio == 50 || gpio == 51)
            {
                //lvds port must be set digital function.The default function is LVDS ODD PAD.
                phyaddr = gpio + (0x0044 >> 2);
                regval = s500_readl(phyaddr);
                regval |= (1 << 22);
                regval &= ~(1 << 21);
                s500_writel(regval, phyaddr);
            }
            else if(gpio == 64 || gpio == 65)
            {
                phyaddr = gpio + (0x0044 >> 2);
                regval = s500_readl(phyaddr);
                if(gpio == 64)
                {
                    regval |= (1 << 13);
                    regval |= (1 << 12);
                }
                else
                {
                    regval |= (1 << 11);
                    regval |= (1 << 10);
                }
                s500_writel(regval, phyaddr);
            }
            else if(gpio == 68 || gpio == 69)
            {
                phyaddr = gpio + (0x0048 >> 2);
                regval = s500_readl(phyaddr);
                regval |= (1 << 30);
                regval &= ~(1 << 29);
                s500_writel(regval, phyaddr);
            }

            if(INPUT == mode || OUTPUT == mode)
            {
                // Disable input/output function
                if(INPUT == mode)
                {
                    phyaddr = gpio + (bank * 3) + 0x00; // +0x00 -> Output Enable Register
                }
                else //OUTPUT
                {
                    phyaddr = gpio + (bank * 3) + 0x01; // +0x04(Byte) -> Input Enable Register
                }
                regval = s500_readl(phyaddr);
                if (wiringPiDebug)
                {
                    printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x read reg val: 0x%x \n", __func__, pin , mode, bank, index, phyaddr, regval);
                }
                regval &= ~(1 << index);
                s500_writel(regval, phyaddr);
                if (wiringPiDebug)
                {
                    regval = s500_readl(phyaddr);
                    printf("set over reg val: 0x%x\n", regval);
                }

                //Enable  input/output function
                if(INPUT == mode)
                {
                    phyaddr = gpio + (bank * 3) + 0x01; // +0x04(Byte) -> Input Enable Register
                }
                else //OUTPUT
                {
                    phyaddr = gpio + (bank * 3) + 0x00; // +0x00 -> Output Enable Register
                }
                regval = s500_readl(phyaddr);
                if (wiringPiDebug)
                {
                    printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x read reg val: 0x%x \n", __func__, pin , mode, bank, index, phyaddr, regval);
                }
                regval |= (1 << index);
                s500_writel(regval, phyaddr);
                if (wiringPiDebug)
                {
                    regval = s500_readl(phyaddr);
                    printf("set over reg val: 0x%x\n", regval);
                }

            }
            else if(PWM_OUTPUT == mode)
            {
                if(pin != 40)
                {
                    printf("the pin you choose is not surport hardware PWM\n");
                    printf("you can select KS_OUT1/GPIOB8 for PWM pin\n");
                    printf("or you can use it in softPwm mode\n");
                    return ;
                }


                s500_pwm_set_enable(1);
                //set default M:S to 1/2
                s500_pwm_set_clk_source(1); //default clk:24M
                s500_pwm_set_clk(120); // 24M/120
                s500_pwm_set_period(1023);
                s500_pwm_set_act(512);
                delayMicroseconds (200);
                printf("PWM_OUTPUT...............\n");
            }
            else
            {
                return;
            }
        }
        else
        {
            printf("func:%s line:%dpin number error\n", __func__, __LINE__);
        }
    }
    else
    {
        struct wiringPiNodeStruct *node = wiringPiNodes ;

        if ((node = wiringPiFindNode (pin)) != NULL)
        {
            node->pinMode (node, pin, mode) ;
        }
    }

}


static void sunxi_set_gpio_mode_wrap(int pin, int mode)
{
    if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
    {
        if (wiringPiMode == WPI_MODE_PINS)
            pin = pinToGpio_BP [pin] ;
        else if (wiringPiMode == WPI_MODE_PHYS)
            pin = physToGpio_BP[pin] ;
        else if (wiringPiMode == WPI_MODE_GPIO)
            pin = pinTobcm_BP[pin]; //need map A20 to bcm
        else return ;

        if (-1 == pin)	/*VCC or GND return directly*/
        {
            //printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
            return;
        }

        if (mode == INPUT)
        {
            sunxi_set_gpio_mode(pin, INPUT);
            wiringPinMode = INPUT;
            return ;
        }
        else if (mode == OUTPUT)
        {
            sunxi_set_gpio_mode(pin, OUTPUT); //gootoomoon_set_mode
            wiringPinMode = OUTPUT;
            return ;
        }
        else if (mode == PWM_OUTPUT)
        {
            if(pin != 259)
            {
                printf("the pin you choose is not surport hardware PWM\n");
                printf("you can select PI3 for PWM pin\n");
                printf("or you can use it in softPwm mode\n");
                return ;
            }
            //printf("you choose the hardware PWM:%d\n", 1);
            sunxi_set_gpio_mode(pin, PWM_OUTPUT);
            wiringPinMode = PWM_OUTPUT;
            return ;
        }
        else
            return ;
    }
    else
    {
        struct wiringPiNodeStruct *node = wiringPiNodes ;

        if ((node = wiringPiFindNode (pin)) != NULL)
            node->pinMode (node, pin, mode) ;
        return ;
    }

}



/*
 * pinMode:
 *	Sets the mode of a pin to be input, output or PWM output
 *********************************************************************************
 */

void pinMode (int pin, int mode)
{

    //add for S500
    if(S500_REV == version )
    {
        if (wiringPiDebug)
        {
            printf ("%s,%d,pin:%d,mode:%d\n", __func__, __LINE__, pin, mode) ;
        }

        s500_set_gpio_mode(pin, mode);

        return ;
    }
    /*add for BananaPro by LeMaker team*/
    else if(BP_REV == version )
    {
        if (wiringPiDebug)
            printf ("%s,%d,pin:%d,mode:%d\n", __func__, __LINE__, pin, mode) ;

        sunxi_set_gpio_mode_wrap(pin, mode);

        return ;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error !!!\n");
    }

}


void s500_pullUpDnControl (int pin, int pud)
{
    uint32_t regval = 0;
    uint32_t *phyaddr = NULL;

    if(wiringPiMode == WPI_MODE_PINS)
    {
        pin = s500_pinToGpio[pin];
    }
    else if (wiringPiMode == WPI_MODE_PHYS)
    {
        pin = s500_physToGpio[pin] ;
    }
    else if (wiringPiMode == WPI_MODE_GPIO)
    {
        pin  = s500_pinTobcm[pin];
    }
    else
    {
        return ;
    }

    if (-1 == pin)
    {
        printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
        return;
    }

    pud &= 0x01;

    switch(pin)
    {
    case 40: /* KS_OUT1/GPIOB8 */
    {
        phyaddr = gpio + (0x0060 >> 2);
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 1); // bit 1
        }
        else//Disable
        {
            regval &= ~(1 << 1);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 41: /* KS_OUT2/GPIOB9 */
    {
        phyaddr = gpio + (0x0060 >> 2);
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 28); // bit 28
        }
        else//Disable
        {
            regval &= ~(1 << 28);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 65: /* DSI_DN3/GPIOC1 */
    {
        phyaddr = gpio + (0x0060 >> 2); //0x0060/4 is bit to 32bit format
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 26); // bit 26
        }
        else//Disable
        {
            regval &= ~(1 << 26);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 68: /* DSI_CP/GPIOC4 */
    {
        phyaddr = gpio + (0x0064 >> 2); //0x0064/4 is bit to 32bit format
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 31); // bit 31
        }
        else//Disable
        {
            regval &= ~(1 << 31);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 69: /* DSI_CN/GPIOC5 */
    {
        phyaddr = gpio + (0x0064 >> 2); //0x0064/4 is bit to 32bit format
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 30); // bit 30
        }
        else//Disable
        {
            regval &= ~(1 << 30);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 90: /* UART0_RX/GPIOC26 */
    {
        phyaddr = gpio + (0x0064 >> 2);
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 2); // bit 2
        }
        else//Disable
        {
            regval &= ~(1 << 2);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 91: /* UART0_TX/GPIOC27 */
    {
        phyaddr = gpio + (0x0064 >> 2);
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 1); // bit 1
        }
        else//Disable
        {
            regval &= ~(1 << 1);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 130: /* TWI2_SCK/GPIOE2 */
    {
        phyaddr = gpio + (0x0068 >> 2);
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 7); // bit 7
        }
        else//Disable
        {
            regval &= ~(1 << 7);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    case 131: /* TWI2_SDA/GPIOE3  */
    {
        phyaddr = gpio + (0x0068 >> 2);
        regval = s500_readl(phyaddr);
        if(pud)//Enable
        {
            regval |= (1 << 8); // bit 8
        }
        else//Disable
        {
            regval &= ~(1 << 8);
        }
        s500_writel(regval, phyaddr);
    }
    break;

    default:
        return;
    }

    if(wiringPiDebug)
    {
        printf ("%s,%d,pin:%d, set over reg val: 0x%x\n", __func__, __LINE__, pin, regval) ;
    }

    delay (10) ;

    return ;
}


/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin
 *	The Arduino only has pull-ups and these are enabled by writing 1
 *	to a port when in input mode - this paradigm doesn't quite apply
 *	here though.
 *********************************************************************************
 */

void pullUpDnControl (int pin, int pud)
{
    struct wiringPiNodeStruct *node = wiringPiNodes ;


    //add for s500
    if(version == S500_REV)
    {
        if((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
        {
            s500_pullUpDnControl(pin, pud);
        }
        else
        {
            if((node = wiringPiFindNode (pin)) != NULL)
            {
                node->pullUpDnControl (node, pin, pud) ;
            }
        }

        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if(version == BP_REV)
    {
        if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
        {
            if (wiringPiMode == WPI_MODE_PINS)
                pin = pinToGpio_BP [pin] ;
            else if (wiringPiMode == WPI_MODE_PHYS)
                pin = physToGpio_BP[pin] ;
            else if (wiringPiMode == WPI_MODE_GPIO)
                pin = pinTobcm_BP[pin]; //need map A20 to bcm
            else return ;
            if (wiringPiDebug)
                printf ("%s,%d,pin:%d\n", __func__, __LINE__, pin) ;

            if (-1 == pin)
            {
                printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
                return;
            }

            pud = upDnConvert[pud];
            sunxi_pullUpDnControl(pin, pud);
            return;
        }
        else	// Extension module
        {
            if ((node = wiringPiFindNode (pin)) != NULL)
                node->pullUpDnControl (node, pin, pud) ;
            return ;
        }
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error !!!\n");
    }


}

static int s500_digitalRead(int pin)
{
    uint32_t regval = 0;
    char c ;

    if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    {
        if (wiringPiMode == WPI_MODE_PHYS)
        {
            pin = s500_physToGpio[pin] ;
        }
        else if(wiringPiMode == WPI_MODE_PINS)
        {
            pin = s500_pinToGpio[pin];
        }
        else if(wiringPiMode == WPI_MODE_GPIO)
        {
            pin = s500_pinTobcm[pin];
        }
        else if(wiringPiMode == WPI_MODE_GPIO_SYS)
        {
            pin = s500_pinTobcm[pin];

            if(pin == -1)
            {
                printf("%d %s,%d invalid pin,please check it over.\n", pin, __func__, __LINE__);
                return 0;
            }

            if (s500_sysFds [pin] == -1)
            {
                if (wiringPiDebug)
                {
                    printf ("pin %d sysFds -1.%s,%d\n", pin , __func__, __LINE__) ;
                }
                return LOW ;
            }

            if (wiringPiDebug)
            {
                printf ("pin %d :%d.%s,%d\n", pin , s500_sysFds [pin], __func__, __LINE__) ;
            }

            lseek  (s500_sysFds [pin], 0L, SEEK_SET) ;
            read   (s500_sysFds [pin], &c, 1) ;

            return (c == '0') ? LOW : HIGH ;

        }
        else
        {
            return LOW;
        }

        if(pin != -1)
        {
            int bank = pin >> 5;
            int index = pin - (bank << 5);
            uint32_t *phyaddr = gpio + (bank * 3) + 0x02; // +0x08 -> data reg

            regval = s500_readl(phyaddr);
            regval = regval >> index;
            regval &= 1;

            if (wiringPiDebug)
            {
                printf("***** read reg val: 0x%x,bank:%d,index:%d,line:%d\n", regval, bank, index, __LINE__);
            }

            return regval;
        }
        else
        {
            printf("pin number error\n");
            return regval;
        }
    }
    else
    {
        struct wiringPiNodeStruct *node = wiringPiNodes ;

        if ((node = wiringPiFindNode (pin)) == NULL)
        {
            return LOW ;
        }

        return node->digitalRead (node, pin) ;
    }
}


static int sunxi_digitalRead_wrap(int pin)
{
    char c ;
    if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    {
        if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
        {

            if(pin == 0)
            {
                //printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
                return 0;
            }
            if(syspin[pin] == -1)
            {
                //printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
                return 0;
            }
            if (sysFds [pin] == -1)
            {
                if (wiringPiDebug)
                    printf ("pin %d sysFds -1.%s,%d\n", pin , __func__, __LINE__) ;
                return LOW ;
            }
            if (wiringPiDebug)
                printf ("pin %d :%d.%s,%d\n", pin , sysFds [pin], __func__, __LINE__) ;
            lseek  (sysFds [pin], 0L, SEEK_SET) ;
            read   (sysFds [pin], &c, 1) ;
            return (c == '0') ? LOW : HIGH ;
        }
        else if (wiringPiMode == WPI_MODE_PINS)
            pin = pinToGpio_BP [pin] ;
        else if (wiringPiMode == WPI_MODE_PHYS)
            pin = physToGpio_BP[pin] ;
        else if (wiringPiMode == WPI_MODE_GPIO)
            pin = pinTobcm_BP[pin]; //need map A20 to bcm
        else
            return LOW ;
        if(-1 == pin)
        {
            printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
            return LOW;
        }
        return sunxi_digitalRead(pin);
    }
    else
    {
        struct wiringPiNodeStruct *node = wiringPiNodes ;

        if ((node = wiringPiFindNode (pin)) == NULL)
            return LOW ;
        return node->digitalRead (node, pin) ;
    }
}


/*
 * digitalRead:
 *	Read the value of a given Pin, returning HIGH or LOW
 *********************************************************************************
 */

int digitalRead (int pin)
{

    //add for S500
    if(S500_REV == version )
    {
        return  s500_digitalRead(pin);
    }
    /*add for BananaPro by LeMaker team*/
    else if(BP_REV == version)
    {
        return sunxi_digitalRead_wrap(pin);
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revision is error\n");
    }

    return -1;
}


static void s500_digitalWrite(int pin, int value)
{
    uint32_t regval = 0;


    if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    {
        if (wiringPiMode == WPI_MODE_PHYS)
        {
            pin = s500_physToGpio[pin] ;
        }
        else if(wiringPiMode == WPI_MODE_PINS)
        {
            pin = s500_pinToGpio[pin];
        }
        else if(wiringPiMode == WPI_MODE_GPIO)
        {
            pin = s500_pinTobcm[pin];
        }
        else if(wiringPiMode == WPI_MODE_GPIO_SYS)
        {
            pin = s500_pinTobcm[pin];

            if (s500_sysFds [pin] != -1)
            {
                if (value == LOW)
                {
                    write (s500_sysFds [pin], "0\n", 2) ;
                }
                else
                {
                    write (s500_sysFds [pin], "1\n", 2) ;
                }
            }
            return;
        }
        else
        {
            return;
        }

        if(pin != -1)
        {
            int bank = pin >> 5;
            int index = pin - (bank << 5);
            uint32_t *phyaddr = gpio + (bank * 3) + 0x02; // +0x08 -> data reg

            regval = s500_readl(phyaddr);

            if (wiringPiDebug)
            {
                printf("befor write reg val: 0x%x,index:%d\n", regval, index);
            }

            if(0 == value) //Low
            {
                regval &= ~(1 << index);
                s500_writel(regval, phyaddr);
            }
            else //High
            {
                regval |= (1 << index);
                s500_writel(regval, phyaddr);
            }

            if (wiringPiDebug)
            {
                regval = s500_readl(phyaddr);
                printf("after write reg val: 0x%x\n", regval);
            }

        }
        else
        {
            printf("pin number error\n");
            return regval;
        }
    }
    else
    {
        struct wiringPiNodeStruct *node = wiringPiNodes ;

        if ((node = wiringPiFindNode (pin)) != NULL)
        {
            node->digitalWrite (node, pin, value) ;
        }
    }

}


static void sxunxi_digitalWrite_wrap(int pin, int value)
{
    if (wiringPiDebug)
        printf ("%s,%d\n", __func__, __LINE__) ;

    if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    {
        if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
        {
            if (wiringPiDebug)
            {
                printf("%d %s,%d invalid pin,please check it over.\n", pin, __func__, __LINE__);
            }

            if(pin == 0)
            {
                //printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
                return;
            }

            if(syspin[pin] == -1)
            {
                //printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
                return;
            }

            if (sysFds [pin] == -1)
            {
                if (wiringPiDebug)
                    printf ("pin %d sysFds -1.%s,%d\n", pin , __func__, __LINE__) ;
            }

            if (sysFds [pin] != -1)
            {
                if (wiringPiDebug)
                    printf ("pin %d :%d.%s,%d\n", pin , sysFds [pin], __func__, __LINE__) ;
                if (value == LOW)
                    write (sysFds [pin], "0\n", 2) ;
                else
                    write (sysFds [pin], "1\n", 2) ;
            }
            return ;
        }
        else if (wiringPiMode == WPI_MODE_PINS)
            pin = pinToGpio_BP [pin] ;
        else if (wiringPiMode == WPI_MODE_PHYS)
            pin = physToGpio_BP[pin] ;
        else if (wiringPiMode == WPI_MODE_GPIO)
            pin = pinTobcm_BP[pin]; //need map A20 to bcm
        else
            return ;

        if(-1 == pin)
        {
            //printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
            return ;
        }
        sunxi_digitalWrite(pin, value);
    }
    else
    {
        struct wiringPiNodeStruct *node = wiringPiNodes ;
        if ((node = wiringPiFindNode (pin)) != NULL)
            node->digitalWrite (node, pin, value) ;
    }
}



/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */

void digitalWrite (int pin, int value)
{
    //add for S500
    if(S500_REV == version )
    {
        s500_digitalWrite(pin, value);
        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if(BP_REV == version)
    {
        sxunxi_digitalWrite_wrap(pin, value);
        return;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revison is error !!!\n");
    }

}


void s500_pwmWrite (int pin, int value)
{
    struct wiringPiNodeStruct *node = wiringPiNodes ;

    if (pin < MAX_PIN_NUM)
    {
        if (wiringPiMode == WPI_MODE_PHYS)
        {
            pin = s500_physToGpio[pin] ;
        }
        else if(wiringPiMode == WPI_MODE_PINS)
        {
            pin = s500_pinToGpio[pin];
        }
        else if(wiringPiMode == WPI_MODE_GPIO)
        {
            pin = s500_pinTobcm[pin];
        }
        else
        {
            return;
        }

        if(pin != -1)
        {
            if(pin != 40)
            {
                printf("please use soft pwmmode or choose PWM pin\n");
                return ;
            }

            uint32_t a_val = 0;

            a_val = s500_pwm_get_period();

            if(wiringPiDebug)
            {
                printf("==> no:%d period now is :%d,act_val to be set:%d\n", __LINE__, a_val, value);
            }

            if(value > 1023 || value < 0)
            {
                printf("val pwmWrite 0 <= X <= 1023\n");;
                return;
            }

            s500_pwm_set_act(value);

        }
    }
    else
    {
        printf ("not on board :%s,%d\n", __func__, __LINE__) ;
        if((node = wiringPiFindNode (pin)) != NULL)
        {
            if (wiringPiDebug)
            {
                printf ("Jim find node%s,%d\n", __func__, __LINE__) ;
            }
            node->digitalWrite (node, pin, value) ;
        }
    }

    if (wiringPiDebug)
    {
        printf ("this fun is ok now %s,%d\n", __func__, __LINE__) ;
    }
}


void sunxi_pwmWrite (int pin, int value)
{
    uint32_t a_val = 0;

    if(pwmmode == 1) //sycle
    {
        sunxi_pwm_set_mode(1);
    }
    else
    {
        //sunxi_pwm_set_mode(0);
    }

    if (pin < MAX_PIN_NUM)  // On-Board Pin needto fix me Jim
    {
        if (wiringPiMode == WPI_MODE_PINS)
            pin = pinToGpio_BP [pin] ;
        else if (wiringPiMode == WPI_MODE_PHYS)
        {
            pin = physToGpio_BP[pin] ;
        }
        else if (wiringPiMode == WPI_MODE_GPIO)
            pin = pinTobcm_BP[pin]; //need map A20 to bcm
        else
            return ;
        if(-1 == pin)
        {
            printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
            return ;
        }
        if(pin != 259)
        {
            printf("please use soft pwmmode or choose PWM pin\n");
            return ;
        }
        a_val = sunxi_pwm_get_period();
        if (wiringPiDebug)
            printf("==> no:%d period now is :%d,act_val to be set:%d\n", __LINE__, a_val, value);
        if(value > a_val)
        {
            printf("val pwmWrite 0 <= X <= 1024\n");
            printf("Or you can set new range by yourself by pwmSetRange(range\n");
            return;
        }
        //if value changed chang it
        sunxi_pwm_set_enable(0);
        sunxi_pwm_set_act(value);
        sunxi_pwm_set_enable(1);
    }
    else
    {
        struct wiringPiNodeStruct *node = wiringPiNodes ;
        printf ("not on board :%s,%d\n", __func__, __LINE__) ;
        if ((node = wiringPiFindNode (pin)) != NULL)
        {
            if (wiringPiDebug)
                printf ("Jim find node%s,%d\n", __func__, __LINE__) ;
            node->digitalWrite (node, pin, value) ;
        }
    }

    if (wiringPiDebug)
        printf ("this fun is ok now %s,%d\n", __func__, __LINE__) ;

}



/*
 * pwmWrite:
 *	Set an output PWM value
 *********************************************************************************
 */

void pwmWrite (int pin, int value)
{
    if(version == S500_REV)
    {
        s500_pwmWrite(pin, value);

        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if(BP_REV == version)
    {
        sunxi_pwmWrite(pin, value);

        return;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revison is error !!!\n");
    }
}


/*
 * analogRead:
 *	Read the analog value of a given Pin.
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

int analogRead (int pin)
{
    struct wiringPiNodeStruct *node = wiringPiNodes ;

    if ((node = wiringPiFindNode (pin)) == NULL)
        return 0 ;
    else
        return node->analogRead (node, pin) ;
}


/*
 * analogWrite:
 *	Write the analog value to the given Pin.
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

void analogWrite (int pin, int value)
{
    struct wiringPiNodeStruct *node = wiringPiNodes ;

    if ((node = wiringPiFindNode (pin)) == NULL)
        return ;

    node->analogWrite (node, pin, value) ;
}


/*
 * pwmToneWrite:
 *	Pi Specific.
 *      Output the given frequency on the Pi's PWM pin
 *********************************************************************************
 */

void pwmToneWrite (int pin, int freq)
{
    int range ;

    if (freq == 0)
        pwmWrite (pin, 0) ;             // Off
    else
    {
        range = 600000 / freq ;
        pwmSetRange (range) ;
        pwmWrite    (pin, freq / 2) ;
    }
}

static void s500_digitalWriteByte (int value)
{
    int mask = 1 ;
    int pin ;
    int phy2gpio[8] = {11, 12, 13, 15, 16, 18, 22, 7};
    int bcm2gpio[8] = {17, 18, 27, 22, 23, 24, 26, 4};

    if(wiringPiMode == WPI_MODE_PINS)
    {
        for (pin = 0 ; pin < 8 ; ++pin)
        {
            pinMode(pin, OUTPUT);
            delay(1);
            digitalWrite (pin, value & mask) ;
            mask <<= 1 ;
        }
    }
    else if(wiringPiMode == WPI_MODE_PHYS)
    {
        for (pin = 0 ; pin < 8 ; ++pin)
        {
            pinMode(phy2gpio[pin], OUTPUT);
            delay(1);
            digitalWrite (phy2gpio[pin], value & mask) ;
            mask <<= 1 ;
        }
    }
    else if(wiringPiMode == WPI_MODE_GPIO || wiringPiMode == WPI_MODE_GPIO_SYS)
    {
        for (pin = 0 ; pin < 8 ; ++pin)
        {
            pinMode(bcm2gpio[pin], OUTPUT);
            delay(1);
            digitalWrite (bcm2gpio[pin], value & mask) ;
            mask <<= 1 ;
        }
    }
    else
    {
        printf("wiringPiMode is error\n");
        return;
    }

}

static int head2win[8] = {11, 12, 13, 15, 16, 18, 22, 7}; /*add for BananaPro by lemaker team*/

static void sunxi_digitalWriteByte (int value)
{
    int mask = 1 ;
    int pin ;

    if (wiringPiMode == WPI_MODE_GPIO_SYS || wiringPiMode == WPI_MODE_GPIO)
    {

        for (pin = 0 ; pin < 8 ; ++pin)
        {
            pinMode(pin, OUTPUT);
            delay(1);
            digitalWrite (pinToGpio [pin], value & mask) ;
            mask <<= 1 ;
        }

    }
    else if(wiringPiMode == WPI_MODE_PINS)
    {

        for (pin = 0 ; pin < 8 ; ++pin)
        {

            pinMode(pin, OUTPUT);
            delay(1);
            digitalWrite (pin, value & mask) ;
            mask <<= 1 ;
        }
    }
    else
    {
        for (pin = 0 ; pin < 8 ; ++pin)
        {
            pinMode(head2win[pin], OUTPUT);
            delay(1);
            digitalWrite (head2win[pin], value & mask) ;
            mask <<= 1 ;
        }
    }

}



/*
 * digitalWriteByte:
 *	Pi Specific
 *	Write an 8-bit byte to the first 8 GPIO pins - try to do it as
 *	fast as possible.
 *	However it still needs 2 operations to set the bits, so any external
 *	hardware must not rely on seeing a change as there will be a change
 *	to set the outputs bits to zero, then another change to set the 1's
 *********************************************************************************
 */

void digitalWriteByte (int value)
{
    //add for s500
    if(version == S500_REV)
    {
        s500_digitalWriteByte(value);
        return;
    }
    /*add for BananaPro by LeMaker team*/
    else if(BP_REV == version)
    {
        sunxi_digitalWriteByte(value);
        return ;
    }
    /*end 2014.08.19*/
    else
    {
        printf("Hardware revison is error !!!\n");
    }

}


/*
 * waitForInterrupt:
 *	Pi Specific.
 *	Wait for Interrupt on a GPIO pin.
 *	This is actually done via the /sys/class/gpio interface regardless of
 *	the wiringPi access mode in-use. Maybe sometime it might get a better
 *	way for a bit more efficiency.
 *********************************************************************************
 */

int waitForInterrupt (int pin, int mS)
{
    int fd, x ;
    uint8_t c ;
    struct pollfd polls ;

    if(version == S500_REV)//add for S500
    {
        if (wiringPiMode == WPI_MODE_PINS)
        {
            pin = s500_pinToGpio[pin];
        }
        else if (wiringPiMode == WPI_MODE_PHYS)
        {
            pin = s500_physToGpio [pin] ;
        }

        if ((fd = s500_sysFds [pin]) == -1)
        {
            return -2 ;
        }
    }
    else
    {
        if (wiringPiMode == WPI_MODE_PINS)
            pin = pinToGpio [pin] ;
        else if (wiringPiMode == WPI_MODE_PHYS)
            pin = physToGpio [pin] ;

        if ((fd = sysFds [pin]) == -1)
            return -2 ;
    }

    // Setup poll structure

    polls.fd     = fd ;
    polls.events = POLLPRI ;	// Urgent data!

    // Wait for it ...

    x = poll (&polls, 1, mS) ;

    // Do a dummy read to clear the interrupt
    //	A one character read appars to be enough.

    (void)read (fd, &c, 1) ;

    return x ;
}


/*
 * interruptHandler:
 *	This is a thread and gets started to wait for the interrupt we're
 *	hoping to catch. It will call the user-function when the interrupt
 *	fires.
 *********************************************************************************
 */

static void *interruptHandler (void *arg)
{
    int myPin ;

    (void)piHiPri (55) ;	// Only effective if we run as root

    myPin   = pinPass ;
    pinPass = -1 ;

    for (;;)
        if (waitForInterrupt (myPin, -1) > 0)
            isrFunctions [myPin] () ;

    return NULL ;
}


/*
 * wiringPiISR:
 *	Pi Specific.
 *	Take the details and create an interrupt handler that will do a call-
 *	back to the user supplied function.
 *********************************************************************************
 */

int wiringPiISR (int pin, int mode, void (*function)(void))
{
    pthread_t threadId ;
    const char *modeS ;
    char fName   [64] ;
    char  pinS [8] ;
    pid_t pid ;
    int   count, i ;
    char  c ;
    int   bcmGpioPin ;

    if ((pin < 0) || (pin > 63))
        return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin must be 0-63 (%d)\n", pin) ;

    if(version == S500_REV)//add for S500
    {
        if (wiringPiMode == WPI_MODE_UNINITIALISED)
        {
            return wiringPiFailure (WPI_FATAL, "wiringPiISR: wiringPi has not been initialised. Unable to continue.\n") ;
        }
        else if (wiringPiMode == WPI_MODE_PINS)
        {
            bcmGpioPin = s500_pinToGpio[pin];
        }
        else if (wiringPiMode == WPI_MODE_PHYS)
        {
            bcmGpioPin = s500_physToGpio [pin] ;
        }
        else
        {
            bcmGpioPin = s500_pinTobcm [pin] ;
        }
    }
    else
    {
        if (wiringPiMode == WPI_MODE_UNINITIALISED)
            return wiringPiFailure (WPI_FATAL, "wiringPiISR: wiringPi has not been initialised. Unable to continue.\n") ;
        else if (wiringPiMode == WPI_MODE_PINS)
            bcmGpioPin = pinToGpio [pin] ;
        else if (wiringPiMode == WPI_MODE_PHYS)
            bcmGpioPin = physToGpio [pin] ;
        else
            bcmGpioPin = pin ;
    }

    /*add for BananaPro by LeMaker team*/

    if(BP_REV == version)
    {
        if(-1 == bcmGpioPin)  /**/
        {
            printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
            return -1;
        }

        if(edge[bcmGpioPin] == -1)
            return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin not sunpprt on bananaPi (%d,%d)\n", pin, bcmGpioPin) ;
    }
    /*end 2014.08.19*/

    //  	Now export the pin and set the right edge
    //	We're going to use the gpio program to do this, so it assumes
    //	a full installation of wiringPi. It's a bit 'clunky', but it
    //	is a way that will work when we're running in "Sys" mode, as
    //	a non-root user. (without sudo)

    if (mode != INT_EDGE_SETUP)
    {
        if (mode == INT_EDGE_FALLING)
            modeS = "falling" ;
        else if (mode == INT_EDGE_RISING)
            modeS = "rising" ;
        else
            modeS = "both" ;

        sprintf (pinS, "%d", bcmGpioPin) ;

        if ((pid = fork ()) < 0)	// Fail
            return wiringPiFailure (WPI_FATAL, "wiringPiISR: fork failed: %s\n", strerror (errno)) ;

        if (pid == 0)	// Child, exec
        {
            if (access ("/usr/local/bin/gpio", X_OK) == 0)
            {
                execl ("/usr/local/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
                return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
            }
            else if (access ("/usr/bin/gpio", X_OK) == 0)
            {
                execl ("/usr/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
                return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
            }
            else
                return wiringPiFailure (WPI_FATAL, "wiringPiISR: Can't find gpio program\n") ;
        }
        else		// Parent, wait
            wait (NULL) ;
    }

    // Now pre-open the /sys/class node - but it may already be open if
    //	we are in Sys mode...
    if(version == S500_REV)//add for S500
    {
        if (s500_sysFds [bcmGpioPin] == -1)
        {
            sprintf (fName, "/sys/class/gpio/gpio%d/value", bcmGpioPin) ;
            if ((s500_sysFds [bcmGpioPin] = open (fName, O_RDWR)) < 0)
                return wiringPiFailure (WPI_FATAL, "wiringPiISR: unable to open %s: %s\n", fName, strerror (errno)) ;
        }


        // Clear any initial pending interrupt

        ioctl (s500_sysFds [bcmGpioPin], FIONREAD, &count) ;
        for (i = 0 ; i < count ; ++i)
            read (s500_sysFds [bcmGpioPin], &c, 1) ;
    }
    else
    {
        if (sysFds [bcmGpioPin] == -1)
        {
            sprintf (fName, "/sys/class/gpio/gpio%d/value", bcmGpioPin) ;
            if ((sysFds [bcmGpioPin] = open (fName, O_RDWR)) < 0)
                return wiringPiFailure (WPI_FATAL, "wiringPiISR: unable to open %s: %s\n", fName, strerror (errno)) ;
        }


        // Clear any initial pending interrupt

        ioctl (sysFds [bcmGpioPin], FIONREAD, &count) ;
        for (i = 0 ; i < count ; ++i)
            read (sysFds [bcmGpioPin], &c, 1) ;
    }

    isrFunctions [pin] = function ;

    pthread_mutex_lock (&pinMutex) ;
    pinPass = pin ;
    pthread_create (&threadId, NULL, interruptHandler, NULL) ;
    while (pinPass != -1)
        delay (1) ;
    pthread_mutex_unlock (&pinMutex) ;

    return 0 ;
}


/*
 * initialiseEpoch:
 *	Initialise our start-of-time variable to be the current unix
 *	time in milliseconds and microseconds.
 *********************************************************************************
 */

static void initialiseEpoch (void)
{
    struct timeval tv ;

    gettimeofday (&tv, NULL) ;
    epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
    epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
}


/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
    struct timespec sleeper, dummy ;

    sleeper.tv_sec  = (time_t)(howLong / 1000) ;
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

    nanosleep (&sleeper, &dummy) ;
}


/*
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicrosecondsHard (unsigned int howLong)
{
    struct timeval tNow, tLong, tEnd ;

    gettimeofday (&tNow, NULL) ;
    tLong.tv_sec  = howLong / 1000000 ;
    tLong.tv_usec = howLong % 1000000 ;
    timeradd (&tNow, &tLong, &tEnd) ;

    while (timercmp (&tNow, &tEnd, < ))
        gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int howLong)
{
    struct timespec sleeper ;
    unsigned int uSecs = howLong % 1000000 ;
    unsigned int wSecs = howLong / 1000000 ;

    if (howLong ==   0)
        return ;
    else if (howLong  < 100)
        delayMicrosecondsHard (howLong) ;
    else
    {
        sleeper.tv_sec  = wSecs ;
        sleeper.tv_nsec = (long)(uSecs * 1000L) ;
        nanosleep (&sleeper, NULL) ;
    }
}


/*
 * millis:
 *	Return a number of milliseconds as an unsigned int.
 *********************************************************************************
 */

unsigned int millis (void)
{
    struct timeval tv ;
    uint64_t now ;

    gettimeofday (&tv, NULL) ;
    now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

    return (uint32_t)(now - epochMilli) ;
}


/*
 * micros:
 *	Return a number of microseconds as an unsigned int.
 *********************************************************************************
 */

unsigned int micros (void)
{
    struct timeval tv ;
    uint64_t now ;

    gettimeofday (&tv, NULL) ;
    now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;

    return (uint32_t)(now - epochMicro) ;
}


/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 *
 * Changed now to revert to "gpio" mode if we're running on a Compute Module.
 *********************************************************************************
 */

int wiringPiSetup (void)
{
    int   fd ;
    int   boardRev ;

    if (getenv (ENV_DEBUG) != NULL)
        wiringPiDebug = TRUE ;

    if (getenv (ENV_CODES) != NULL)
        wiringPiReturnCodes = TRUE ;

    if (geteuid () != 0)
        (void)wiringPiFailure (WPI_FATAL, "wiringPiSetup: Must be root. (Did you forget sudo?)\n") ;

    if (wiringPiDebug)
        printf ("wiringPi: wiringPiSetup called\n") ;

    boardRev = piBoardRev () ;

    if(version == S500_REV) //add for s500
    {
        pinToGpio =  pinToGpioR;
        physToGpio = physToGpioR;
    }
    else if (BP_REV == boardRev)  /*modify for BananaPro by LeMaker team*/
    {
        pinToGpio =  pinToGpioR ;
        physToGpio = physToGpioR ;
        physToPin = physToPinR3;
    }
    else
    {

    }

    // Open the master /dev/memory device

    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;


    if (S500_REV == boardRev)
    {

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

        // GPIO:
        gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, S500_GPIO_BASE) ;
        if ((int32_t)gpio == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

        // PWM
        pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, S500_GPIO_PWM & ~MAP_MASK) ;
        if ((int32_t)pwm == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PWM) failed: %s\n", strerror (errno)) ;

        // Clock control (needed for PWM)
        clk = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, S500_CLOCK_BASE) ;
        if ((int32_t)clk == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (CLOCK) failed: %s\n", strerror (errno)) ;

        // The drive pads
        pads = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, S500_GPIO_PADS) ;
        if ((int32_t)pads == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PADS) failed: %s\n", strerror (errno)) ;

#ifdef	USE_TIMER
        // The system timer
        timer = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, S500_GPIO_TIMER) ;
        if ((int32_t)timer == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (TIMER) failed: %s\n", strerror (errno)) ;

        // Set the timer to free-running, 1MHz.
        //	0xF9 is 249, the timer divide is base clock / (divide+1)
        //	so base clock is 250MHz / 250 = 1MHz.

        *(timer + TIMER_CONTROL) = 0x0000280 ;
        *(timer + TIMER_PRE_DIV) = 0x00000F9 ;
        timerIrqRaw = timer + TIMER_IRQ_RAW ;
#endif

    }
    else if (BP_REV == boardRev)  /*modify for BananaPro by LeMaker team*/
    {
        // GPIO:
        gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_BP);

        if ((int32_t)gpio == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

        // PWM

        pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_PWM_BP) ;
        if ((int32_t)pwm == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PWM) failed: %s\n", strerror (errno)) ;

        // Clock control (needed for PWM)

        clk = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, CLOCK_BASE_BP) ;
        if ((int32_t)clk == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (CLOCK) failed: %s\n", strerror (errno)) ;

        // The drive pads

        pads = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_PADS_BP) ;
        if ((int32_t)pads == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PADS) failed: %s\n", strerror (errno)) ;

#ifdef	USE_TIMER
        // The system timer

        timer = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_TIMER_BP) ;
        if ((int32_t)timer == -1)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (TIMER) failed: %s\n", strerror (errno)) ;

        // Set the timer to free-running, 1MHz.
        //	0xF9 is 249, the timer divide is base clock / (divide+1)
        //	so base clock is 250MHz / 250 = 1MHz.

        *(timer + TIMER_CONTROL) = 0x0000280 ;
        *(timer + TIMER_PRE_DIV) = 0x00000F9 ;
        timerIrqRaw = timer + TIMER_IRQ_RAW ;
#endif
    }
    else
    {

    }

    initialiseEpoch () ;

    // If we're running on a compute module, then wiringPi pin numbers don't really many anything...

    wiringPiMode = WPI_MODE_PINS ;

    return 0 ;
}


/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupGpio (void)
{
    (void)wiringPiSetup () ;

    if (wiringPiDebug)
        printf ("wiringPi: wiringPiSetupGpio called\n") ;

    wiringPiMode = WPI_MODE_GPIO ;

    return 0 ;
}


/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupPhys (void)
{
    (void)wiringPiSetup () ;

    if (wiringPiDebug)
        printf ("wiringPi: wiringPiSetupPhys called\n") ;

    wiringPiMode = WPI_MODE_PHYS ;

    return 0 ;
}


/*
 * wiringPiSetupSys:
 *	Must be called once at the start of your program execution.
 *
 * Initialisation (again), however this time we are using the /sys/class/gpio
 *	interface to the GPIO systems - slightly slower, but always usable as
 *	a non-root user, assuming the devices are already exported and setup correctly.
 */

int wiringPiSetupSys (void)
{
    int boardRev ;
    int pin ;
    char fName [128] ;

    if (getenv (ENV_DEBUG) != NULL)
        wiringPiDebug = TRUE ;

    if (getenv (ENV_CODES) != NULL)
        wiringPiReturnCodes = TRUE ;

    if (wiringPiDebug)
        printf ("wiringPi: wiringPiSetupSys called\n") ;

    boardRev = piBoardRev () ;

    if(version == S500_REV) //add for s500
    {
        pinToGpio =  pinToGpioR;
        physToGpio = physToGpioR;
    }
    else if (BP_REV == boardRev)   /*modify for BananaPro by LeMaker team*/
    {
        pinToGpio =  pinToGpioR ;
        physToGpio = physToGpioR ;
        physToPin = physToPinR3;
    }
    else
    {

    }

    // Open and scan the directory, looking for exported GPIOs, and pre-open
    //	the 'value' interface to speed things up for later
    if(boardRev == S500_REV)
    {
        char cmdstr[80] = {'\0'};

        for (pin = 0 ; pin < 132 ; ++pin)
        {
            if(s500ValidGpio[pin] != -1)
            {
                snprintf(cmdstr, 80, "/usr/local/bin/gpio export %d out", pin);
                system(cmdstr);
                sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
                s500_sysFds [pin] = open (fName, O_RDWR) ;
            }
        }
    }
    else if(BP_REV == boardRev)    /*modify for BananaPro by LeMaker team*/
    {
        for (pin = 1 ; pin < 32 ; ++pin)
        {
            sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
            sysFds [pin] = open (fName, O_RDWR) ;
        }
    }
    else
    {

    }

    initialiseEpoch () ;

    wiringPiMode = WPI_MODE_GPIO_SYS ;

    return 0 ;
}
