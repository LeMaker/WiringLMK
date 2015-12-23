/*
 * pwm_s500.c:
 *	This tests the hardware PWM channel for s500.
 *
 * Copyright (c) 2015 LeMaker
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
