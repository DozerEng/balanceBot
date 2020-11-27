/*!
 * \file main.cpp
 * \brief Main page for controlling robot arm
 * 
 * \author Michael Pillon
 * \date Created: October 21, 2020
 * \date Last Modified: October 21, 2020
 * 
 * Uses RobotArm class to control EEZYbotARM MK2 robot arm from thingiverse.com
 * https://www.thingiverse.com/thing:1454048
 *              
 * Using MBED OS6
 *
 * Outstanding Tasks: Do everything!
 *      - I2C test for PCA9685
 * 
 * 
 */

#include "mbed.h"
#include "Helper.hpp"

#include "A4988.hpp"
#include "BalanceBot.hpp"
#include "MPU6050.hpp"



/*!
 *  Hardware Initialization
 */
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

DigitalIn topPB(p5);
DigitalIn midPB(p6);
DigitalIn botPB(p7);

//Left Wheel
DigitalOut l_step(p19);
DigitalOut l_dir(p20);
DigitalOut l_ms1(p14);
DigitalOut l_ms2(p15);
DigitalOut l_ms3(p16);
//Right Wheel
DigitalOut r_step(p17);
DigitalOut r_dir(p18);
DigitalOut r_ms1(p11);
DigitalOut r_ms2(p12);
DigitalOut r_ms3(p13);


Thread ledThread; //!< Handling onboard LED routine
Thread buttonThread; //!< Mnitoring push buttons

/*!
 *  Defines
 */

#define GO_TIME             true   //!< It's always go time!
#define BLINKING_RATE_MS    150 //!< Blinking rate in milliseconds
#define BUTTON_DELAY        500
#define BUTTON_DEBOUNCE     20  
#define WAIT                1000



/*!
 *  Function Declarations
 */
void threadLedRoutine(void);
void threadButtonRoutine(void);
void checkPBs(BalanceBot* bot);

/*!
 *  \fn int std::main()
 *  \brief Main
 *  
 *  \return Exit success/failure
 */


int main()
{
    //!< Subroutine Calls
    ledThread.start(threadLedRoutine);

    I2C i2c(p28, p27); //!< SDA, SCL
    //I2C i2c(I2C_SDA1, I2C_SCL1);
    i2c.frequency(400000); //<! Maxspeed for MPU6050
    A4988 leftWheel(l_step, l_dir, l_ms1, l_ms2, l_ms3);
    A4988 rightWheel(r_step, r_dir, r_ms1, r_ms2, r_ms3);

    BalanceBot bot(&leftWheel, &rightWheel, &i2c);

    //!< Main thread
    while (GO_TIME) {

        bot.propBalance();
        //checkPBs(&bot);
        //bot.step(50);
        //ThisThread::sleep_for(500);
    }
    return EXIT_SUCCESS;

}

/*!
 *  Function Definitions
 */


/*!
 * \fn void checkPBs()
 *
 * Checks if any push buttons are pressed and calls handlers 
 *
 */
void checkPBs(BalanceBot* bot)
{
    if(topPB == 0) {
        while(topPB == 0){
            //!< Wait for release of button
        }
        bot->decStepMode();         
    }
    if(midPB == 0 ) {
        while(midPB == 0){
            //!< Wait for release of button
        }
        bot->incStepMode();
    }
}

/*!
 * \fn void ledRoutine(void)
 *
 * Subroutine for control over onboard LEDs
 * Handles what function to call and rate for blinking
 */
void threadLedRoutine(void)
{
    while(true) {
        Helper::knightRider(&led1, &led2, &led3, &led4);
        ThisThread::sleep_for(BLINKING_RATE_MS);
    }
}
