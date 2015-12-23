/* Copyright (c) 2015 LeMaker */
#include <stdio.h>
#include <wiringPi.h>


int main(int argc, char **argv)
{

# if 0
    wiringPiSetupSys();
    s500_sys_gpio_type(1);
    while(1)
    {
        digitalWrite(3, 1);
        printf(".....pin 3.........HIGH\n");
        delay (6000) ;		// mS

        digitalWrite(3, 0);
        printf(".....pin 3.........LOW\n");
        delay (6000) ;		// mS
    }
#else
    wiringPiSetupSys();

    while(1)
    {
        //注意: 以下的pin需要改为bcm 对应的pin
        digitalWrite(3, 1);
        printf(".....pin 3.........HIGH\n");
        delay (6000) ;		// mS

        digitalWrite(3, 0);
        printf(".....pin 3.........LOW\n");
        delay (6000) ;		// mS
    }


#endif
}
