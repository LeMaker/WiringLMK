/*
 * pwm.c:
 *	This tests the hardware PWM channel.
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
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

#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main (void)
{
    int bright ;
    int pol = -1;

    printf ("Raspberry Pi wiringPi PWM test program\n") ;

    if (wiringPiSetup () == -1)
        exit (1) ;

    pinMode (16, PWM_OUTPUT) ;
    printf("PWM Test...............\n");

    delay (10 * 1000) ;
    pwmWrite (16, 128) ;
    printf("PWM Test.........duty: 128......\n");
    delay (10 * 1000) ;
    pwmWrite (16, 256) ;
    printf("PWM Test.........duty: 256......\n");
    delay (10 * 1000) ;
    pwmWrite (16, 768) ;
    printf("PWM Test.........duty: 768......\n");

    pol = s500_pwm_get_polarity();
    printf("PWM Test.........pol: %d\n", pol);
    delay (10 * 1000) ;

    s500_pwm_set_polarity(1);
    pol = s500_pwm_get_polarity();
    printf("PWM Test.........pol: %d\n", pol);
    delay (10 * 1000) ;


    s500_pwm_set_polarity(0);
    pol = s500_pwm_get_polarity();
    printf("PWM Test.........pol: %d\n", pol);
    delay (10 * 1000) ;


    while(1)
    {
        delay (1) ;
    }

    /*for (;;)
    {
      for (bright = 0 ; bright < 1024 ; ++bright)
      {
        pwmWrite (1, bright) ;
        delay (1) ;
      }

      for (bright = 1023 ; bright >= 0 ; --bright)
      {
        pwmWrite (1, bright) ;
        delay (1) ;
      }
    }*/

    return 0 ;
}
