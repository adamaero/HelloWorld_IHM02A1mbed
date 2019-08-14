#include "mbed.h"
#include "DevSPI.h"
#include "XNucleoIHM02A1.h"

#define MPR_1 4

#define STEPS_1 (200 * 128)
#define STEPS_2 (STEPS_1 * 2)

#define DELAY_1 1000
#define DELAY_2 2000
#define DELAY_3 5000

L6470 **motors; // double pointer, to L6470 (chip) class

XNucleoIHM02A1 *x_nucleo_ihm02a1; // pointer to function

int position = -1;
int positiontwo = -1;

L6470_init_t init[L6470DAISYCHAINSIZE] = {
    /* First Motor. */
    {
        24.0,                          /* Motor supply voltage in V. */
        200,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */        
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    },

    /* Second Motor. */
    {
        24.0,                           /* Motor supply voltage in V. */
        200,                           /* Min number of steps per revolution for the motor. */
        1.7,                           /* Max motor phase voltage in A. */
        3.06,                          /* Max motor phase voltage in V. */
        300.0,                         /* Motor initial speed [step/s]. */
        500.0,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        992.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        3.06,                          /* Holding kval [V]. */
        3.06,                          /* Constant speed kval [V]. */
        3.06,                          /* Acceleration starting kval [V]. */
        3.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        3.06 * 1000 * 1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        3.06 * 1000 * 1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */        
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    }
};

void startup();
void forward();
void allTogether();

int main()
{      
    /* Initializing SPI bus. */
#ifdef TARGET_STM32F429
    DevSPI dev_spi(D11, D12, D13);
#else
    DevSPI dev_spi(D11, D12, D13);
#endif

    /* Initializing Motor Control Expansion Board. */
    x_nucleo_ihm02a1 = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, &dev_spi);

    /* Building a list of motor control components. */
    motors = x_nucleo_ihm02a1->get_components();

    startup();

    forward();
//    allTogether();
        

}  
    /*----- Functions -----*/ /*----- -----*/ 

void startup()
{
    /*----- Initialization. -----*/
    
    motors[0]->set_home();
    //motors[1]->set_home();                   // @@

    wait_ms(DELAY_1);

    position = motors[0]->get_position(); 
    positiontwo = motors[1]->get_position(); 
    
    wait_ms(DELAY_1);    
}

void forward()
{
        motors[0]->move(StepperMotor::FWD, STEPS_1);
        motors[1]->move(StepperMotor::FWD, STEPS_1);
        
    wait_ms(DELAY_2);
}
    
void allTogether()
{
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) 
        motors[m]->prepare_run(StepperMotor::BWD, 400);
        
    x_nucleo_ihm02a1->perform_prepared_actions();

    wait_ms(DELAY_3);
}
