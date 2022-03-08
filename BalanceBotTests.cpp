/*!
 * \file BalanceBotTests.cpp
 * \author Michael Pillon
 * 
 * Functions to run to test parts of the robot
 *      - MPU6050 runs debug messages in console when initialized
 */

#include "BalanceBot.hpp"


/*!
    \fn runAllTests()

    Runs all hardware tests

    ~~~ MAKE SURE TO ADD ALL TESTS TO THIS FUNCTION ~~~

*/
void BalanceBot::runAllTests() {
    // testWheels();
    testWheelsFast();
}
/*!
    \fn testWheels()

    Runs both wheels in the same direction and opposite directions at all speeds.
    - Based on A4988 stepper motor drivers
    - It is recommended that tape or an identifier be put on each wheel to ensure full rotation at all microstepping modes.
    
    Procedure: 
        1) Both wheels should do a full rotation in the forward direction for each of the 5 microstepping modes of the A4988 as well as with a 1 second delay between each rotation. (From fastest to slowest)
        2) Both wheels should do a 6th rotation at the slowest speed.
        3) Steps 1 and 2 then repeat in the reverse direction but going from slowest to fastest with the 6th rotation being at the fastest speed.
        4) steps 1 and 2 are repeated with wheels going in opposite directions in both orientations

    Troubleshooting:
        - If wheels move in the same direction but do not move at 5 different speeds or do not make full rotations then the MSx pins are incorrectly set.
        - Something else went wrong, figure it out and document here.

 */
void BalanceBot::testWheels() {
    //!< Step 1 and 2
    setDirection(FORWARD);
    setStepMode(FULL_STEP);
    for(int i = 0; i < 6; i++) {
        printf("Step 1 part %i: Steps: %i\n\r", i, STEPS_PER_REVOLUTION*stepMode);
        steps(STEPS_PER_REVOLUTION*stepMode);
        incStepMode();
        ThisThread::sleep_for(500);
    }
    //!< Step 3
    setDirection(REVERSE);
    for(int i = 0; i < 6; i++) {
        printf("Step 3 part %i: Steps: %i\n\r", i, STEPS_PER_REVOLUTION*stepMode);
        steps(STEPS_PER_REVOLUTION*stepMode);
        decStepMode();
        ThisThread::sleep_for(500);
    }
    //!< Step 4 part 1
    setDirection(LEFT_TURN);
    setStepMode(FULL_STEP);
    for(int i = 0; i < 6; i++) {
        printf("Step 4a part %i: Steps: %i\n\r", i, STEPS_PER_REVOLUTION*stepMode);
        steps(STEPS_PER_REVOLUTION*stepMode);
        incStepMode();
        ThisThread::sleep_for(500);
    }    
    //!< Step 4 part 2
    setDirection(RIGHT_TURN);
    setStepMode(SIXTEENTH_STEP);
    for(int i = 0; i < 5; i++) {
        printf("Step 4b part %i: Steps: %i\n\r", i, STEPS_PER_REVOLUTION*stepMode);
        steps(STEPS_PER_REVOLUTION*stepMode);
        decStepMode();
        ThisThread::sleep_for(500);
    }
}

/*!
    \fn testWheelsFast()

    Implementation of testWheels() which has 200 steps per step mode while testWheels() has 200*stepMode steps per step mode.

 */
void BalanceBot::testWheelsFast() {
    //!< Step 1 and 2
    setDirection(FORWARD);
    setStepMode(FULL_STEP);
    for(int i = 0; i < 6; i++) {
        steps(STEPS_PER_REVOLUTION);
        incStepMode();
        ThisThread::sleep_for(100);
    }
    //!< Step 3
    setDirection(REVERSE);
    for(int i = 0; i < 6; i++) {
        steps(STEPS_PER_REVOLUTION);
        decStepMode();
        ThisThread::sleep_for(100);
    }
    //!< Step 4 part 1
    setDirection(LEFT_TURN);
    setStepMode(FULL_STEP);
    for(int i = 0; i < 6; i++) {
        steps(STEPS_PER_REVOLUTION);
        incStepMode();
        ThisThread::sleep_for(100);
    }    
    //!< Step 4 part 2
    setDirection(RIGHT_TURN);
    setStepMode(SIXTEENTH_STEP);
    for(int i = 0; i < 5; i++) {
        steps(STEPS_PER_REVOLUTION);
        decStepMode();
        ThisThread::sleep_for(100);
    }
}