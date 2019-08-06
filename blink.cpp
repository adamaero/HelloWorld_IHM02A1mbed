/* Includes ------------------------------------------------------------------*/

/* mbed specific header files. */
#include "mbed.h"

/* Helper header files. */
#include "DevSPI.h"

/* Expansion Board specific header files. */
#include "XNucleoIHM02A1.h"

///DigitalIn User_Button(USER_BUTTON);

/* Definitions ---------------------------------------------------------------*/

/* Number of movements per revolution. */
#define MPR_1 4

/* Variables -----------------------------------------------------------------*/

/* Motor Control Expansion Board. */
XNucleoIHM02A1 *x_nucleo_ihm02a1;

* Main ----------------------------------------------------------------------*/

int main()
{
    /*----- Initialization. -----*/

    /* Initializing SPI bus. */
#ifdef TARGET_STM32F429
    DevSPI dev_spi(D11, D12, D13);
#else
    DevSPI dev_spi(D11, D12, D13);
#endif

    /* Initializing Motor Control Expansion Board. */
    x_nucleo_ihm02a1 = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, &dev_spi);

    /* Building a list of motor control components. */
    L6470 **motors = x_nucleo_ihm02a1->get_components();
    
    
}
