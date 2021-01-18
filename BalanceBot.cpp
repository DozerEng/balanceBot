/*!
 * \class BalanceBot
 * \file BalanceBot.cpp
 * \author Michael Pillon
 * 
 */

#include "BalanceBot.hpp"

BalanceBot::BalanceBot(I2C* i2c) : 
    //!< Hardware
    mpu(i2c),
    bothWheels(Port1, STEP_MASK_ON),
    leftWheel(L_STEP, L_DIR, L_MS1, L_MS2, L_MS3),
    rightWheel(R_STEP, R_DIR, R_MS1, R_MS2, R_MS3),
    topPB(TOP_PB),
    botPB(BOT_PB),
    //!< Registers
    setPoint(BALANCE_POINT),
    stepMode(FULL_STEP),
    //!< Other
    pid(0.7, 1, 1) // Needs tuning and #define statements
    {
    setStepMode(stepMode);
    mpu.setDLPF(DLPF_CFG_6); //!< Digital Low Pass Filter
    //!< Start robot/threads or run hardware tests
    
    runAllTests();
    //balanceThread.start(callback(this, &BalanceBot::balanceThreadRoutine));

    printf("\n\r~~~ BalanceBot online ~~~\n\n\r");
}

/*!
    \fn step()
    \param count Number of steps to rotate wheels, default to 1 step
*/
void BalanceBot::step(const uint16_t count) {
    for(int i = 0; i < count; i++) {
        bothWheels = STEP_MASK_ON;
        wait_us(STEP_DELAY);
        bothWheels = STEP_MASK_OFF;
        wait_us(STEP_DELAY);
    }
}
/*!
    \fn setDirection()
    Set direction of both wheels
    \param dir Direction to set wheels. No arg toggles state
 */
void BalanceBot::setDirection() {
    leftWheel.setDirMode();
    rightWheel.setDirMode();
}

void BalanceBot::setDirection(const uint8_t dir) {
    leftWheel.setDirMode(dir);
    rightWheel.setDirMode(dir);
}
/*!
    \fn setStepMode()
    Set wheel motor microstepping mode
    Calls corresponding A4988 stepMode function
    \param uint8_t integer for xxxx_STEP mode
*/
void BalanceBot::setStepMode(const uint8_t mode) {
    leftWheel.setStepMode(mode);
    rightWheel.setStepMode(mode);
    stepMode = mode;
}
/*!
    \fn incStepMode()
 */
void BalanceBot::incStepMode() {
    leftWheel.incStepMode();
    rightWheel.incStepMode();
    stepMode = leftWheel.getStepMode();
}
/*!
    \fn decStepMode()
 */
void BalanceBot::decStepMode() {
    leftWheel.decStepMode();
    rightWheel.decStepMode();
    stepMode = leftWheel.getStepMode();
}
/*!
    \fn getTilt()
    Calculate approximate tilt of robot based on accelerometer data.
        tilt = actualAzimuth - tiltError
        Arguments for atan2() may need adjusting depending on orientation of MPU6050.
    \return azimuth angle of robot. Radians by default.
 */
double BalanceBot::getTilt() {
    int16_t xAcceleration, yAcceleration, zAcceleration;
    mpu.getAccel(&xAcceleration, &yAcceleration, &zAcceleration);

    int16_t gyroX, gyroY, gyroZ;
    mpu.getGyro(&gyroX, &gyroY, &gyroZ); 
    int16_t netAccel = xAcceleration - gyroY * 0.2; //!< Compensating for moment about y axis (d = 0.2 meters)
    
    double azimuth = atan2( netAccel, zAcceleration );

    return atan2( xAcceleration, zAcceleration );
}
/*!
    \fn getTiltDegrees()
    \sa getTilt()
 */
double BalanceBot::getTiltDegrees() {
    return getTilt() * 180.0 / M_PI;
}
/*!
    \fn handlePBs()
    Handle the 2 external push buttons
 */
void BalanceBot::handlePBs() {
    if(topPB == 0) {
        while(topPB == 0){
            //!< Wait for release of button
        }
        //double temperature = mpu.getTemp();
        printf("Current Temperature %0.1f degrees celcius\n\r", mpu.getTemp());         
    }
    if(botPB == 0 ) {
        while(botPB == 0){
            //!< Wait for release of button
        }
        /*
            do stuff for bottom PB
         */
    }  
}

/*!
    \fn runMotors()
    Thread for handing movement of the robots wheels
    \param azimuth tilt angle of robot in degrees
 */
void BalanceBot::runMotors(double azimuth) {
    //!< Set motor direction
    azimuth -= setPoint;
    if(azimuth < 0.0) {
        setDirection(FORWARD);
        azimuth *= -1.0;
    } else {
        setDirection(REVERSE);
    }
    //!< Set motor resolution based on tilt
    if(azimuth < 2.0) {
        wait_us(100000);
    } else if(azimuth < 10.0 && stepMode != SIXTEENTH_STEP) {
        setStepMode(SIXTEENTH_STEP);
        stepMode = SIXTEENTH_STEP;
    } else if(azimuth < 25.0 && stepMode != QUARTER_STEP) {
        setStepMode(QUARTER_STEP);
        stepMode = QUARTER_STEP;
    } else if (stepMode != FULL_STEP) {
        setStepMode(FULL_STEP);
        stepMode = FULL_STEP;
    }

    uint8_t steps = uint8_t((azimuth*0.5/DEGREES_PER_STEP)*stepMode);
    
    //printf("Steps: %i\n\r", steps);
    step(steps);
}


/*!
    \fn balanceThreadRoutine
    Thread for finding the azimuth tilt of robot and saving it in private variable "azimuth"
        - Hardware: MPU6050
        - Controller: PID_Controller
 */
void BalanceBot::balanceThreadRoutine() {
    while(true) {
        double mpuReading = getTiltDegrees();
        //printf("Raw azimuth: %0.3f\n\r", mpuReading);
        //printf("%f\n\r", mpuReading);
        runMotors(mpuReading);
        //ThisThread::sleep_for(1000);
    }

    // azimuth = pid.controlStep(azimuth, BALANCE_POINT);

}
