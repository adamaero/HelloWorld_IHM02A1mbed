//github.com/search?l=C%2B%2B&q=incremental+encoder&type=Repositories
    //github.com/borlum/Encoder/tree/master/src
    //github.com/IbanAino/RotaryIncrementalEncoder

//Mbed
    //os.mbed.com/users/vsluiter/code/Encoder/file/18b000b443af/encoder.cpp/
    //   ACTUALLY USING:
        //os.mbed.com/users/JonFreeman/code/RotaryEncoder/file/545775417e0d/main.cpp/
#include "mbed.h"
#include "DevSPI.h"
#include "XNucleoIHM02A1.h"
#include "BufferedSerial.h"

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

/*  ----- Encoders -----  */

/*  Second motor sensors  */
InterruptIn rcA     (D8);     //  Coder provides A and B quadrature outputs and a once-per-rev index position pulse (signal Z)
InterruptIn rcB     (D9);     //  These three signals wired to inputs configured to cause CPU interrupt on every level transition
InterruptIn rcZ     (D11);
DigitalOut  led_grn     (LED1); //  green led on ST Nucleo board set to flash to prove system alive
BufferedSerial pc(USBTX, USBRX);    //  serial port via usb used to display output on pc using TTY terminal e.g. PuTTY

volatile    bool    trigger_200ms = false;

Ticker  toggleTick;         //  turns led on / off @ 1Hz, provides visual check of cpu running
 
int     seconds = 0;



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
/*  ----- Encoders -----  */
void ledToggler(void) { //  Interrupt handler - led on half sec, off half sec, repeat
    led_grn = !led_grn;
    if  (led_grn)
        seconds++;  //  Counts up once per second
}
 
void    twohundredms_handler    ()  {
    trigger_200ms = true;
}
 
signed long angle = 0, turns = 0;
 
void    rcArise_handler ()  {   //  Handler of 'A' rising edge interrupts
    if  (rcB)   angle++;
    else        angle--;
}
 
void    rcAfall_handler ()  {   //  Handler of 'A' falling edge interrupts
    if  (rcB)   angle--;
    else        angle++;
}
 
void    rcBrise_handler ()  {   //  Handler of 'B' rising edge interrupts   
    if  (rcA)   angle--;
    else        angle++;
}
 
void    rcBfall_handler ()  {   //  Handler of 'B' falling edge interrupts
    if  (rcA)   angle++;
    else        angle--;
}
 
void    rcZrise_handler ()  {   //  Index pulse interrupt handler,
    if   (rcA)   {              //  Keeps count of whole turns of the shaft
        turns--;
    }
    else    {
        turns++;
    }
    angle = 0;
}
void    rcZfall_handler ()  {   }

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

    /* Building a list of motor control components. */
    motors1stLevel = x_nucleo_ihm02a1one->get_components();
    motors2ndLevel = x_nucleo_ihm02a1two->get_components();

    // startup();

    forward();
//    allTogether();
/**
 ******************************************************************************
  **/
    int     c_5 = 0;
    toggleTick.attach(&ledToggler, 0.5f);   //  Set interrupt handler to toggle led
    Ticker  tick200ms;
    tick200ms.attach_us(&twohundredms_handler, 200000); //  Cause timer interrupts at 5Hz
 
    rcA.mode  (PullUp);     //  Attach pullup resistors to the 3 coder outputs
    rcB.mode  (PullUp);
    rcZ.mode  (PullUp);
    rcA.rise  (&rcArise_handler);   //  Identify interrupt handlers for each of rise and fall
    rcA.fall  (&rcAfall_handler);   //  events on all three input lines
    rcB.rise  (&rcBrise_handler);
    rcB.fall  (&rcBfall_handler);
    rcZ.rise  (&rcZrise_handler);
    rcZ.fall  (&rcZfall_handler);
 
    pc.baud (115200);
    pc.printf   ("Jon's rotary encoder test sytem starting up\r\n");
 
    //  Main loop
    while(1) 
    {
        pc.printf   ("Turns %+d\t%+.1f degree\r\n", turns, (double)angle * 360.0 / 2048.0);
 
        c_5++;
        if(c_5 > 4)
        {               //  Do things once per second here
            c_5 = 0;
        }
        while   (!trigger_200ms)    ;//  NEARLY ALL CPU TIME WASTED HERE waiting for interrupt handler to set trigger_200 true
        trigger_200ms = false;      //  Hence main loop is cycled every 200ms
    }

/**
 ******************************************************************************
  **/        

}  
