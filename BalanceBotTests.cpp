/*!
 * \file BalanceBotTests.cpp
 * \author Michael Pillon
 * 
 * Functions to run to test parts of the robot
 *
 */

#include "BalanceBot.hpp"


/*!
    \fn runAllTests()

    Runs all hardware tests

    ~~~ MAKE SURE TO ADD ALL TESTS TO THIS FUNCTION ~~~

*/
void BalanceBot::runAllTests() {
    testWheels();
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
        step(STEPS_PER_REVOLUTION*stepMode);
        incStepMode();
        ThisThread::sleep_for(500);
    }
    //!< Step 3
    setDirection(REVERSE);
    for(int i = 0; i < 6; i++) {
        printf("Step 3 part %i: Steps: %i\n\r", i, STEPS_PER_REVOLUTION*stepMode);
        step(STEPS_PER_REVOLUTION*stepMode);
        decStepMode();
        ThisThread::sleep_for(500);
    }
    //!< Step 4 part 1
    leftWheel.setDirMode(FORWARD);
    rightWheel.setDirMode(REVERSE);
    setStepMode(SIXTEENTH_STEP);
    for(int i = 0; i < 5; i++) {
        step(STEPS_PER_REVOLUTION*stepMode);
        decStepMode();
        ThisThread::sleep_for(500);
    }    
    //!< Step 4 part 2
    leftWheel.setDirMode(REVERSE);
    rightWheel.setDirMode(FORWARD);
    setStepMode(SIXTEENTH_STEP);
    for(int i = 0; i < 5; i++) {
        step(STEPS_PER_REVOLUTION*stepMode);
        decStepMode();
        ThisThread::sleep_for(500);
    }
}