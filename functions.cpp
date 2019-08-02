/**
 ******************************************************************************
 * @file    main.cpp
 * @author  Davide Aliprandi, STMicroelectronics
 * @version V1.0.0g
 * @date    November 4th, 2015
 * @brief   mbed test application for the STMicroelectronics X-NUCLEO-IHM02A1
 *          Motor Control Expansion Board: control of 2 motors.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


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

/* Number of steps. */
#define STEPS_1 (200 * 128)   /* 1 revolution given a 200 steps motor configured at 1/128 microstep mode. */
#define STEPS_2 (STEPS_1 * 2)

/* Delay in milliseconds. */
#define DELAY_1 1000
#define DELAY_2 2000
#define DELAY_3 5000

/* Declarations --------------------------------------------------------------*/

   void forward();
   void backDouble();
   void goToMark();
   void goHome();    
   void halfMicrosteps();
   void allTogether();


/* Variables -----------------------------------------------------------------*/

    int position = -1;
    int positiontwo = -1;
    
/* Motor Control Expansion Board. */
XNucleoIHM02A1 *x_nucleo_ihm02a1;

/* Initialization parameters of the motors connected to the expansion board. */
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

//DigitalIn enable(p5);
/* Main ----------------------------------------------------------------------*/

int main()
{
    startup();  //  Must use.
    
    
    
    
    
    forward();
    backDouble();
    goToMark();
    goHome();    
    halfMicrosteps();
    allTogether();
        
 

}  


    /*----- Functions -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/
     /*----- -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/ /*----- -----*/ 

void startup()
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

    /* Printing to the console. */
    printf("Motor Control Application Example for 2 Motors\r\n\n");

    /*----- Setting home and mark positions, getting positions, and going to positions. 0-----*/

    /* Printing to the console. */
    printf("--> Setting home position.\r\n");

    /* Setting the home position. */
    motors[0]->set_home();
    //motors[1]->set_home();                   // @@

    /* Waiting. */
    wait_ms(DELAY_1);

    /* Getting the current position. */   
    position = motors[0]->get_position();       // Comment out this line to disable motor 1
    positiontwo = motors[1]->get_position(); // Comment out this line to disable motors 2 & 3

    /* Printing to the console. */
    if (position != -1)
    printf("--> Getting the current position1: %d\r\n", position);
    if (positiontwo != -1)
    printf("--> Getting the current position2: %d\r\n", positiontwo);
    
    /* Waiting. */
    wait_ms(DELAY_1);    
}

void forward()
{
    /*----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- 1: Forward-----*/
    /* Printing to the console. */
    printf("--> Moving forward %d steps.\r\n", STEPS_1);

    /* Moving. */
    if (position != -1)
        motors[0]->move(StepperMotor::FWD, STEPS_1);
    if (positiontwo != -1)
        motors[1]->move(StepperMotor::FWD, STEPS_1);
        
    /* Waiting while active. */
    if (position != -1)    
        motors[0]->wait_while_active();
    if (positiontwo != -1)    
        motors[1]->wait_while_active();

    /* Getting the current position. */
    if (position != -1)        
        position = motors[0]->get_position();
    if (positiontwo != -1)    
        position = motors[1]->get_position();
    
    /* Printing to the console. */
    
    printf("--> Getting the current position1: %d\r\n", position);
    printf("--> Getting the current position2: %d\r\n", positiontwo);

    /* Printing to the console. */
    printf("--> Marking the current position.\r\n");

    /* Marking the current position. */
    motors[0]->set_mark();

    /* Waiting. */
    wait_ms(DELAY_1);
}
    
    /*----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- 2: Double back-----*/    
    
void backDouble()
{
    /* Printing to the console. */
    printf("--> Moving backward %d steps.\r\n", STEPS_2);

    /* Moving. */
    motors[0]->move(StepperMotor::BWD, STEPS_2);

    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Waiting. */
    wait_ms(DELAY_1);
   
    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    printf("--> Getting the current position: %d\r\n", position);

    /* Waiting. */
    wait_ms(DELAY_1);    
}
    
    /*----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- 3: Go to mark-----*/
void goToMark()
{  
    /* Printing to the console. */
    printf("--> Going to marked position.\r\n");

    /* Going to marked position. */
    motors[0]->go_mark();
    
    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Waiting. */
    wait_ms(DELAY_1);

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    printf("--> Getting the current position: %d\r\n", position);

    /* Waiting. */
    wait_ms(DELAY_1);
}
    
    /*----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- 4: Go to home-----*/
void goHome()    
{    
    
    /* Printing to the console. */
    printf("--> Going to home position.\r\n");

    /* Going to home position. */
    motors[0]->go_home();
    
    /* Waiting while active. */
    motors[0]->wait_while_active();

    /* Waiting. */
    wait_ms(DELAY_1);

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    printf("--> Getting the current position: %d\r\n", position);

    /* Waiting. */
    wait_ms(DELAY_1);
}
    
    /*----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- 5-----*/    
    
void halfMicrosteps()
{    
    /* Printing to the console. */
    printf("--> Halving the microsteps.\r\n");

    /* Halving the microsteps. */
    init[0].step_sel = (init[0].step_sel > 0 ? init[0].step_sel -  1 : init[0].step_sel);
    
    if (!motors[0]->set_step_mode((StepperMotor::step_mode_t) init[0].step_sel)) {
        printf("    Step Mode not allowed.\r\n");
    }

    /* Waiting. */
    wait_ms(DELAY_1);

    /* Printing to the console. */
    printf("--> Setting home position.\r\n");

    /* Setting the home position. */
    motors[0]->set_home();

    /* Waiting. */
    wait_ms(DELAY_1);

    /* Getting the current position. */
    position = motors[0]->get_position();
    
    /* Printing to the console. */
    printf("--> Getting the current position: %d\r\n", position);

    /* Waiting. */
    wait_ms(DELAY_1);
}
    
    /*----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- 6: Together-----*/
    
void allTogether()
{
    /*----- Running together for a certain amount of time. -----*/

    /* Printing to the console. */
    printf("--> Running together for %d seconds.\r\n", DELAY_3 / 1000);

    /* Preparing each motor to perform a run at a specified speed. */
    for (int m = 0; m < L6470DAISYCHAINSIZE; m++) {
        motors[m]->prepare_run(StepperMotor::BWD, 400);
    }

    /* Performing the action on each motor at the same time. */
    x_nucleo_ihm02a1->perform_prepared_actions();

    /* Waiting. */
    wait_ms(DELAY_3);
}
