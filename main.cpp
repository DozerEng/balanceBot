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
#include "A4988.hpp"
#include "BalanceBot.hpp"

/*!
 *  Hardware Initialization
 */
Thread ledThread; //!< Thread for handling onboard LED routine
Thread buttonThread; //!< Thread for monitoring push buttons

Serial pc(USBTX, USBRX);

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
void knightRider(void);
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

    
    A4988 leftWheel(l_step, l_dir, l_ms1, l_ms2, l_ms3);
    A4988 rightWheel(r_step, r_dir, r_ms1, r_ms2, r_ms3);
    BalanceBot bot(&leftWheel, &rightWheel);
    

    I2C i2c(p28, p27); //!< SDA, SCL
    //I2C i2c(I2C_SDA1, I2C_SCL1);
    i2c.frequency(400000); //<! Maxspeed for MPU6050

    //!< Main thread
    while (GO_TIME) {




        checkPBs(&bot);
        bot.step();
        
        //ThisThread::sleep_for(100);
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
        bot->setDirection();         
    } else if(midPB == 0 ) {
        /*!
            Decrease Microstep mode
        */
       while(midPB == 0){
            //!< Wait for release of button
        }
       bot->incStepMode();
//    } else if(botPB == 0) {
//        /*!
//             Increase Microstep mode
//         */
//        while(botPB == 0){
//             //!< Wait for release of button
//         }
//         bot->decStepMode();

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
        knightRider();
        ThisThread::sleep_for(BLINKING_RATE_MS);
    }
}

/*!
 * \fn void knightRider(void)
 * \sa ledRoutine
 * 
 * Knight Rider style LED routine.
 * LEDs illiminate back and forth from left to right
 *
 */
void knightRider(void)
{
    static int position = 0; //! position + 1 = next active LED
    static bool direction = 0; //!left to right (0) or right to left (1)
    
    //LED Position logic
    if (direction == 0) { //! Forward Direction
        switch (position) {
            case 0:
                led1 = 1;
                led2 = 0;
                led3 = 0;
                led4 = 0;
                position++;
                break;
            case 1:
                led1 = 0;
                led2 = 1;
                led3 = 0;
                led4 = 0;
                position++;
                break;
            case 2:
                led1 = 0;
                led2 = 0;
                led3 = 1;
                led4 = 0;
                position++;
                break;
            case 3:
                led1 = 0;
                led2 = 0;
                led3 = 0;
                led4 = 1;
                direction = 1; //reverse direction
                position--;
                break;
            default:
                position = 0; //Should NEVER get here
        }
    } else { //!Reverse direction == 1
        switch (position) {
            case 0:
                led1 = 1;
                led2 = 0;
                led3 = 0;
                led4 = 0;
                direction = 0;
                position++;
                break;
            case 1:
                led1 = 0;
                led2 = 1;
                led3 = 0;
                led4 = 0;
                position--;
                break;
            case 2:
                led1 = 0;
                led2 = 0;
                led3 = 1;
                led4 = 0;
                position--;
                break;
            case 3:
                led1 = 0;
                led2 = 0;
                led3 = 0;
                led4 = 1;
                position--; //reverse direction
                break;
            default:
                position = 1; //Should NEVER get here
        }
    }
}
