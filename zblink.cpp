#include "mbed.h"
#include "DevSPI.h"
#include "XNucleoIHM02A1.h"

///DigitalIn User_Button(USER_BUTTON);

#define MPR_1 4

XNucleoIHM02A1 *x_nucleo_ihm02a1;

DigitalOut myled(LED1);
DigitalOut red(LED1);


int main()
{
#ifdef TARGET_STM32F429
    DevSPI dev_spi(D11, D12, D13);
#else
    DevSPI dev_spi(D11, D12, D13);
#endif

    /* Initializing Motor Control Expansion Board. */
    x_nucleo_ihm02a1 = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, &dev_spi);

    /* Building a list of motor control components. */
    L6470 **motors = x_nucleo_ihm02a1->get_components();
    
    while(1) {
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
    }
    
}
