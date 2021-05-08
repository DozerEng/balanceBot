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
    bothWheels(MOTOR_PORT, STEP_MASK),
    leftWheel(L_STEP, L_DIR, L_MS1, L_MS2, L_MS3),
    rightWheel(R_STEP, R_DIR, R_MS1, R_MS2, R_MS3),
    topPB(TOP_PB),
    botPB(BOT_PB),
    pid(KC, TI, TD), // Needs tuning and #define statements
    //!< Registers
    stepMode(FULL_STEP)
    {
    /*!
        Initalize modes of the robot
     */
    setStepMode(stepMode);
    setDirection(FORWARD); 
    mpu.setDLPF(DLPF_CFG_5); //!< Digital Low Pass Filter

    /*!
        Start robot/threads or run hardware tests
     */
    //runAllTests();
    controlSystemThread.start(callback(this, &BalanceBot::controlSystem));

    fprintf(bbOut, "\n\r~~~ BalanceBot online ~~~\n\n\r");
}

/*!
    \fn step()
    \param count Number of steps to rotate wheels, default to 1 step
*/
void BalanceBot::step(const uint16_t count) {
    for(int i = 0; i < count; i++) {
        bothWheels = bothWheels | STEP_MASK;
        wait_us(STEP_DELAY);
        bothWheels = bothWheels & ~STEP_MASK;
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
    switch (dir) {
    // Configuration when right wheel is forward by default and left wheel is moving in reverse
        case FORWARD: 
            leftWheel.setDirMode(REVERSE);
            rightWheel.setDirMode(FORWARD);
            break;
        case REVERSE: 
            leftWheel.setDirMode(FORWARD);
            rightWheel.setDirMode(REVERSE);
            break;
        case RIGHT_TURN: 
            leftWheel.setDirMode(REVERSE);
            rightWheel.setDirMode(REVERSE);
            break;
        case LEFT_TURN: 
            leftWheel.setDirMode(FORWARD);
            rightWheel.setDirMode(FORWARD);
            break;
        default:
            fprintf(bbErr, "BalanceBot::setDirection -> Invalid direction value passed");
            break;
    }    
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
    stepMode = rightWheel.getStepMode();
}
/*!
    \fn decStepMode()
 */
void BalanceBot::decStepMode() {
    leftWheel.decStepMode();
    rightWheel.decStepMode();
    stepMode = rightWheel.getStepMode();
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
    //fprintf(bbOut, "X acceleration: %i  Y acceleration: %i Z acceleration: %i\n\r", xAcceleration, yAcceleration,zAcceleration);

    // int16_t gyroX, gyroY, gyroZ;
    // mpu.getGyro(&gyroX, &gyroY, &gyroZ); 
   
    //sdouble azimuth = atan2( -zAcceleration, xAcceleration );
    
    return atan2( -zAcceleration, xAcceleration );//
}
/*!
    \fn getTiltDegrees()
    \sa getTilt()
 */
double BalanceBot::getTiltDegrees() {
    return getTilt() * RADIANS_TO_DEGREES;
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
    \fn plant()
    Thread for handing movement of the robots wheels
    \param azimuth tilt angle of robot in degrees
 */
void BalanceBot::plant(double controlVariable) {

    //!< Set motor resolution based on tilt
    /*
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
    */

    uint8_t steps = uint8_t((controlVariable*0.5/DEGREES_PER_STEP)*stepMode);
    
    //printf("Steps: %i\n\r", steps);
    step(steps);
}


/*!
    \fn controlSystem
    Thread for finding the azimuth tilt of robot and saving it in private variable "azimuth"
        - Hardware: MPU6050
        - Controller: PID_Controller
 */
void BalanceBot::controlSystem() {
    pid.start();
    while(true) {
        // 1) Calculate tilt/error from MPU6050 (Process variable)
        double mpuReading = getTiltDegrees();
        double error = (setPoint - mpuReading);
        fprintf(bbOut, "Error in degrees: %0.2f\n\r", error );
        fprintf(bbOut, "Azimuth in degrees: %0.2f\n\r", mpuReading );

        // 2) Setpoint and plant output get passed to PID controller
        double controlVariable = pid.controlStep(mpuReading, setPoint);
        error = controlVariable - setPoint;
        fprintf(bbOut, "Control Variable: %0.2f\n\r", error );
        ThisThread::sleep_for(500);  

        // 3) Control variable passed to plant which moves the motors
        //plant(controlVariable);

    }
    pid.stop();
    fprintf(bbOut, "Control system shutting down....\n\r");
    ThisThread::sleep_for(1500);
    fprintf(bbOut, "Control system successfully shut down\n\r");
}
