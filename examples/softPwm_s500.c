/*
 * softPwm_s500.c:
 *	Test of the software PWM driver.
 *	For S500
 *
 * Copyright (c) Lemaker
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
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
