/*!
    \file main.cpp
    \brief 2 wheel self balancing robot

    \author Michael Pillon
    \date Created: November 21, 2020
 
    Main program for self balancing robot.
    Using mbed LPC1768 microcontorller,
        MPU6050 IMU,
        2x A4988 stepper motor driver boards,
        2x NEMA 17 stepper motors,
        2x external pushbuttons 
    
 */

#include "mbed.h"
#include "Helper.hpp"

#include "A4988.hpp"
#include "BalanceBot.hpp"
#include "MPU6050.hpp"


/*!
    Hardware Initialization
 */
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

Thread ledThread; //!< Handling onboard LED routine

/*!
    Defines
 */

#define GO_TIME             true    //!< It's always go time!
#define BLINKING_RATE_MS    200     //!< Blinking rate in microseconds
#define BUTTON_DELAY        500
#define BUTTON_DEBOUNCE     20  
#define WAIT                1000



/*!
    Function Declarations
 */
void threadHeartbeat(void);

/*!
    \fn int std::main()
    \brief Main

    \return Exit success/failure
 */


int main()
{
    
    //!< Subroutine Calls
    ledThread.start(threadHeartbeat);

    printf("\n\rStarting MBED...\n\r");
    I2C i2c(p28, p27); //!< SDA, SCL
    //I2C i2c(I2C_SDA1, I2C_SCL1);
    i2c.frequency(400000); //<! Maxspeed for MPU6050

    BalanceBot bot(&i2c);

    //!< Main thread
    while (GO_TIME) {
        //bot.handlePBs();

        ThisThread::sleep_for(100);
        //wait_us(100000);

    }
    return EXIT_SUCCESS;

}

/*!
    \fn void threadHeartbeat(void)

    Subroutine for control over 4 onboard LEDs
    Handles what function to call and rate for blinking

    Use as a heartbeat indicator for major faults.
    During a major fault, LED1 blinks repeatedly.
 */
void threadHeartbeat(void)
{
    while(true) {
        Helper::knightRider(&led1, &led2, &led3, &led4);
        ThisThread::sleep_for(BLINKING_RATE_MS);
    }
}
