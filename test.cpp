
#include "mbed.h"
#include "DevSPI.h"
#include "XNucleoIHM02A1.h"

#define STEPS_2 (200 * 128 * 2)

XNucleoIHM02A1 *x_nucleo_ihm02a1;
XNucleoIHM02A1 *x_nucleo_ihm02a1two;

L6470_init_t init[L6470DAISYCHAINSIZE] = {
    {
                24.0,
				200,
				StepperMotor::STEP_MODE_1_128,
				0xFF,
				0x2E88
    },
    {
                24.0,
				200,
				StepperMotor::STEP_MODE_1_128,
				0xFF,
				0x2E88
    }
};
int main()
{
#ifdef TARGET_STM32F429
    DevSPI dev_spi(D11, D12, D13);
#else
    DevSPI dev_spi(D11, D12, D13);
#endif

    x_nucleo_ihm02a1 =    new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, D2, &dev_spi);
    x_nucleo_ihm02a1two = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, &dev_spi);    

    L6470 **motors = x_nucleo_ihm02a1->get_components();
    L6470 **motorstwo = x_nucleo_ihm02a1two->get_components();

    /* Setting the home position. */
    //motorstwo[1]->set_home();
    int position = motorstwo[1]->get_position();
           
//      motors[1]->move(StepperMotor::FWD, STEPS_2);
//    wait_ms(DELAY_2);
      motorstwo[1]->move(StepperMotor::FWD, STEPS_2);
      
    position = motorstwo[1]->get_position();
}
