/*
 * softPwm_s500.c:
 *	Test of the software PWM driver.
 *	For S500
 *
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <softPwm.h>

#define RANGE		100
#define	NUM_LEDS	  8

int ledMap [NUM_LEDS] = { 0, 1, 2, 3, 4, 5, 6, 7 } ;

int values [NUM_LEDS] = { 0, 25, 50, 75, 100, 75, 50, 25 } ;

int main ()
{
    int i, j ;
    char buf [80] ;

    wiringPiSetup ()  ;

    softPwmCreate (3, 0, RANGE) ;//phy
    printf ("GPIOE3, %3d, %3d\n", 0, RANGE) ;

    softPwmWrite (3, 50) ;

    while(1)
    {

    }
}
