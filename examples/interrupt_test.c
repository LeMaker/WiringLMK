
#include <stdio.h>
#include <wiringPi.h>



void myInterrupt1 (void)
{
    printf("The Phy pin 16 is detected..............\n");
}
void myInterrupt2 (void)
{
    printf("The Phy pin 18 is detected..............\n");
}
void myInterrupt3 (void)
{
    printf("The Phy pin 8 is detected..............\n");
}
void myInterrupt4 (void)
{
    printf("The Phy pin 10 is detected..............\n");
}
void myInterrupt5 (void)
{
    printf("The Phy pin 3 is detected..............\n");
}
void myInterrupt6 (void)
{
    printf("The Phy pin 5 is detected..............\n");
}







int main(int argc, char **argv)
{
    wiringPiSetupPhys();

    pullUpDnControl(16, 1);
    wiringPiISR (16, INT_EDGE_FALLING, &myInterrupt1) ;

    pullUpDnControl(18, 1);
    wiringPiISR (18, INT_EDGE_FALLING, &myInterrupt2) ;

    pullUpDnControl(8, 1);
    wiringPiISR (8, INT_EDGE_FALLING, &myInterrupt3) ;

    pullUpDnControl(10, 1);
    wiringPiISR (10, INT_EDGE_FALLING, &myInterrupt4) ;

    /*pullUpDnControl(3, 1);
    wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt5) ;

    pullUpDnControl(5, 1);
    wiringPiISR (5, INT_EDGE_FALLING, &myInterrupt6) ;*/


    while(1)
    {
        delay (100) ;		// mS
    }
}
