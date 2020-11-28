/*!
 * \class BalanceBot
 * \file BalanceBot.cpp
 * \author Michael Pillon
 * 
 */

#include "BalanceBot.hpp"

BalanceBot::BalanceBot(A4988* lw, A4988* rw, I2C* i2c) : 
    mpu(i2c),
    leftWheel(lw),
    rightWheel(rw)
    {
    

    printf("\n\rBalanceBot is online...\n\n\r");
}

/*!
    Incrememnt both wheels forward
    \param count Number of steps to rotate wheels
*/
void BalanceBot::step(const uint8_t count) {
    for(int i = 0; i < count; i++) {
        leftWheel->setStep(HIGH);
        rightWheel->setStep(HIGH);
        ThisThread::sleep_for(STEP_DELAY);
        leftWheel->setStep(LOW);
        rightWheel->setStep(LOW);
        ThisThread::sleep_for(STEP_DELAY);
    }
}
/*!
    Set direction of both wheels
    \param dir Direction to set wheels. No arg toggles state
 */
void BalanceBot::setDirection() {
    leftWheel->setDirMode();
    rightWheel->setDirMode();
}

void BalanceBot::setDirection(const uint8_t dir) {
    leftWheel->setDirMode(dir);
    rightWheel->setDirMode(dir);
}
/*!
    Set wheel motor microstepping mode
    Calls corresponding A4988 stepMode function
    \param uint8_t integer for xxxx_STEP mode
*/
void BalanceBot::setStepMode(const uint8_t mode) {
    leftWheel->setStepMode(mode);
    rightWheel->setStepMode(mode);

}
void BalanceBot::incStepMode() {
    leftWheel->incStepMode();
    rightWheel->incStepMode();
}
void BalanceBot::decStepMode() {
    leftWheel->decStepMode();
    rightWheel->decStepMode();
}
/*!
    Calculate approximate tilt of robot based on accelerometer data.
        tilt = actualAzimuth - liltError
        Arguments for atan2() may need adjusting depending on orientation of MPU6050.
    \return azimuth angle of robot. Radians by default.
 */
double BalanceBot::getTilt() {
    int16_t xAcceleration, yAcceleration, zAcceleration;

    mpu.getAccel(&xAcceleration, &yAcceleration, &zAcceleration);

    return atan2( xAcceleration, zAcceleration );
}

double BalanceBot::getTiltDegrees() {
    return getTilt() * 180.0 / M_PI;
}
/*!
    Balances the robot using proportional controller
        1) Get Measurement
        2) Set step mode based on lilt
        3) Set Direction
        4) Step motors as required by tilt angle
 */
void BalanceBot::propBalance() {
    double tiltAvg = 0.0;
    for(int i = 0; i < NUM_TILT_SAMPLES; i++) {
        tiltAvg += getTiltDegrees();
    }
    tiltAvg /= NUM_TILT_SAMPLES;
    printf("Tilt: %0.2f\n\r", tiltAvg);
    if(tiltAvg >=0) {
        setDirection(REVERSE);
    } else {
        setDirection(FORWARD);
    }
    //!< Handle Tilt
    double offset = abs(tiltAvg) - BALANCE_POINT;
    if((offset <= 1.0)) {
        return;
    } else if((offset <= 10.0)) {
        if ( stepMode != SIXTEENTH_STEP){
            setStepMode(SIXTEENTH_STEP);
            stepMode = SIXTEENTH_STEP;
        }
    } else {
        if ( stepMode != QUARTER_STEP){
            setStepMode(QUARTER_STEP);
            stepMode = QUARTER_STEP;
        }
    }
    uint8_t steps = uint8_t(offset*DEGREES_PER_STEP*stepMode);
    printf("Offset: %0.2f Steps: %i\n\r", offset, steps);
    step(steps);
}
