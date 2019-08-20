#include "mbed.h"
#include "DevSPI.h"
#include "XNucleoIHM02A1.h"

#define L6470_L1M1 (0u) // Index of shield1 motor1
#define L6470_L1M2 (1u) // Index of shield1 motor2

// #define L6470_L2M1 (0u) // Index of shield2 motor1
#define L6470_L2M2 (1u) // Index of shield2 motor2

#define MPR_1 4

#define STEPS_1 (200 * 128)
#define STEPS_2 (STEPS_1 * 2)

#define DELAY_1 1000
#define DELAY_2 2000
#define DELAY_3 5000

L6470 **motors1stLevel; // Double pointer, to L6470 (chip) class
L6470 **motors2ndLevel;

XNucleoIHM02A1 *x_nucleo_ihm02a1one; // Pointer to function
XNucleoIHM02A1 *x_nucleo_ihm02a1two;

/*  First motor   */
DigitalIn encoder1A(D14);
DigitalIn encoder1B(D15);

/*  Second motor   */
DigitalIn encoder2A(D8);
DigitalIn encoder2B(D9);

/*  Third, fine increments, motor */
DigitalIn encoder3A(D6);
DigitalIn encoder3B(D7);

int positionL1M1, positionL1M2, positionL2M1, positionL2M2 = -1;

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
// void allTogether();

int main()
{      
    /* Initializing SPI bus. */
#ifdef TARGET_STM32F429
    DevSPI dev_spi(D11, D12, D13);
#else
    DevSPI dev_spi(D11, D12, D13);
#endif

    /* Initializing Motor Control Expansion Board. */
//    x_nucleo_ihm02a1one = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, &dev_spi);
    x_nucleo_ihm02a1one = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, D10, &dev_spi);
    x_nucleo_ihm02a1two = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, D2, &dev_spi);    
    
    // x_nucleo_ihm02a1two = new XNucleoIHM02A1(&init[0], &init[1], A0, A1, D1, D2, &dev_spi);  

  //                                                                                First shield works, motors move.
  //                                                                                Second shield (3rd motor) does nothing...
/**
 ******************************************************************************
  *
 * 
 * 
 * 
 *
 ******************************************************************************
 */
    /* Building a list of motor control components. */
    motors1stLevel = x_nucleo_ihm02a1one->get_components();
    motors2ndLevel = x_nucleo_ihm02a1two->get_components();

    // startup();

    forward();
  
//    allTogether();
        

}  
    /*----- Functions -----*/ /*----- -----*/ 

void startup()
{
    /*----- Initialization. -----*/
    motors1stLevel[L6470_L1M1]->set_home();
    motors1stLevel[L6470_L1M2]->set_home();
    // motors2ndLevel[L6470_L2M1]->set_home(); 
    motors2ndLevel[L6470_L2M2]->set_home();

    wait_ms(DELAY_1);

    positionL1M1 = motors1stLevel[L6470_L1M1]->get_position();
    positionL1M2 = motors1stLevel[L6470_L1M2]->get_position();
    // positionL2M1 = motors2ndLevel[L6470_L2M1]->get_position();
    positionL2M2 = motors2ndLevel[L6470_L2M2]->get_position();
      
    wait_ms(DELAY_1);    
}

void forward()
{
        motors1stLevel[L6470_L1M1]->move(StepperMotor::FWD, STEPS_1);
        motors1stLevel[L6470_L1M2]->move(StepperMotor::FWD, STEPS_1);
      wait_ms(DELAY_3);
//        motors2ndLevel[L6470_L2M2]->move(StepperMotor::FWD, STEPS_1);   // There is no L2M1
        motors2ndLevel[L6470_L2M2]->move(StepperMotor::FWD, STEPS_1);
}
    
void allTogether()// right now. Over me. bum bum, batta bumm...He bad production He got walrus gumboot He got Ono sideboard He one spinal cracker He got feet down below his kneeieeieesss
{
//    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) 
//        motors[m]->prepare_run(StepperMotor::BWD, 400);
//        
//    x_nucleo_ihm02a1->perform_prepared_actions();
//
//    wait_ms(DELAY_3);
}
