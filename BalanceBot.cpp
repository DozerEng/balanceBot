/*!
 * \class BalanceBot
 * \file BalanceBot.cpp
 * \author Michael Pillon
 * 
 */

#include "BalanceBot.hpp"

//!< Constructor
BalanceBot::BalanceBot(I2C* i2c) : 
    mpu(i2c),
    pid(KP, KI, KD, DT, INTEGRAL_WINDUP_LIMIT, bbOut),
    bothWheels(MOTOR_PORT, STEP_MASK),
    leftWheel(L_STEP, L_DIR, L_MS1, L_MS2, L_MS3),
    rightWheel(R_STEP, R_DIR, R_MS1, R_MS2, R_MS3),
    stepMode(QUARTER_STEP),
    // Inputs
    topPushButton(TOP_PB),
    bottomPushButton(BOTTOM_PB),
    limitSwitch(LIMIT_SWITCH),
    //pbs(PB_PORT, PB_MASK),
    // Outputs
    rgbLED(RGB_TYPE, RGB_R, RGB_G, RGB_B),
    bbOut(stdout), // stdout, "./fileName.txt", "a"
    bbErr(stderr) // stderr, 
    {


    /*!
        Initalize modes of the robot
     */
    
    setStepMode(stepMode);
    setDirection(FORWARD); 
    mpu.setDLPF(DLPF_CFG_6); //!< Digital Low Pass Filter

    /*!
        Start robot/threads or run hardware tests
     */
    //runAllTests();
    
    imuTicker.attach(callback(this, &BalanceBot::imuISR), DT);

    controlThread.start(callback(&controlQueue, &EventQueue::dispatch_forever));
    controlThread.set_priority(osPriorityHigh1);
    
    buttonThread.start(callback(&buttonQueue, &EventQueue::dispatch_forever));
    buttonThread.set_priority(osPriorityRealtime1);
    handlePBs(true); // Initialize PBs by running method once
    buttonQueue.call_every(BUTTON_CHECK_INTERVAL, callback(this, &BalanceBot::handlePBs), false);

    fprintf(bbOut, "\n\r~~~ BalanceBot online ~~~\n\n\r");
}

/*!
    \fn steps()
    \param count Number of steps to rotate wheels, default to 1 step
*/
void BalanceBot::steps(const uint16_t count) {
    for(int i = 0; i < count; i++) {
        bothWheels = bothWheels | STEP_MASK;
        wait_us(STEP_DELAY);
        bothWheels = bothWheels & ~STEP_MASK;
        wait_us(STEP_DELAY);
    }
}
/*!
    \fn step()
    \param count Number of steps to rotate wheels, default to 1 step
*/
void BalanceBot::step() {
    bothWheels = bothWheels | STEP_MASK;
    wait_us(STEP_DELAY);
    bothWheels = bothWheels & ~STEP_MASK;
    wait_us(STEP_DELAY);
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
            directionMode = FORWARD;
            break;
        case REVERSE: 
            leftWheel.setDirMode(FORWARD);
            rightWheel.setDirMode(REVERSE);
            directionMode = REVERSE;
            break;
        case RIGHT_TURN: 
            leftWheel.setDirMode(REVERSE);
            rightWheel.setDirMode(REVERSE);
            directionMode = RIGHT_TURN;
            break;
        case LEFT_TURN: 
            leftWheel.setDirMode(FORWARD);
            rightWheel.setDirMode(FORWARD);
            directionMode = LEFT_TURN;
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
void BalanceBot::handlePBs(bool initialize) {
    static double *activeConstant;
    static double constantIncrement;
    if (initialize == true ) {
        activeConstant = &kp; 
        constantIncrement = KP_INCREMENT;
        rgbLED.setRed();
    }

    /*!
        Limit Switch changes which constant is being modified
    */
    if(limitSwitch == 0 ) {
        ThisThread::sleep_for(BUTTON_DEBOUNCE); 
        fprintf(bbOut, "Changed active constant to ");
        while(limitSwitch == 0 );
        if(activeConstant == &kp) {
            activeConstant = &ki;
            constantIncrement = KI_INCREMENT;
            rgbLED.setGreen();
            fprintf(bbOut, "ki = %0.5f\n\r", *activeConstant);
        } else if(activeConstant == &ki) {
            activeConstant = &kd;
            constantIncrement = KD_INCREMENT;
            rgbLED.setBlue();
            fprintf(bbOut, "kd = %0.5f\n\r", *activeConstant);
        } else {
            activeConstant = &kp;
            constantIncrement = KP_INCREMENT;
            rgbLED.setRed();
            fprintf(bbOut, "kp = %0.5f\n\r", *activeConstant);
        } 
    }
    /*!
        Left Push Button increments active constant
    */
    if(topPushButton == 0) { // Test board wiring got fucked up when replacing a switch. Normally 0 == PRESSED
        ThisThread::sleep_for(BUTTON_DEBOUNCE); 
        while(topPushButton == 1);
        *activeConstant = *activeConstant + constantIncrement;
        pid.setGain(kp, ki, kd);
        fprintf(bbOut, "Increased active constant %0.5f\n\r", *activeConstant);
    }
    /*!
        Right Push Button decrements active constant
    */
    if(bottomPushButton == 0 ) {
        ThisThread::sleep_for(BUTTON_DEBOUNCE);            
        while(bottomPushButton == 0);
        *activeConstant = *activeConstant - constantIncrement;
        pid.setGain(kp, ki, kd);
        fprintf(bbOut, "Deacreased active constant to: %0.5f\n\r", *activeConstant);
    }
}

/*!
    \fn imuISR
    Get readings from IMU and calculate needed angles.
    Send results to an EventQueue for controller adjustments and motor control
 */
void BalanceBot::imuISR() {  
    static int controlQueueID = 0;
    // Clear control queue
    if(controlQueueID) controlQueue.cancel(controlQueueID);
    
    // Call control queue with new value
    controlQueueID = controlQueue.call(callback(this, &BalanceBot::controlSystem));
}

/*!
    \fn controlSystem
    Thread for finding the azimuth tilt of robot and saving it in private variable "azimuth"
        - Hardware: MPU6050
        - Controller: PID_Controller
 */
void BalanceBot::controlSystem() {
      
    // 1) Calculate tilt/error from MPU6050 (Process variable)
    double mpuReading = getTiltDegrees();

    // 2) Apply complimentary filter

    

    if(logControlData) {
        // printQueue.call(fprintf, bbOut, "%.2f\n\r", mpuReading);
        fprintf(bbOut, "%.2f\n\r", mpuReading);
    }

    // Apply PID
    double error = pid.controlStep(mpuReading, setPoint);

    // Do motor stuff Balancebot::motorSystem()
    motorSystem(error);
}


/*!
    \fn plant()
    Thread for handing movement of the robots wheels
 */
void BalanceBot::motorSystem(double error) {

    if(error < 0.0) {
        setDirection(FORWARD);
    } else {
        setDirection(REVERSE);
    }
    
    double absError = abs(error);

    if(absError > FULL_STEP_MINIMUM_ANGLE) {
        setStepMode(FULL_STEP);
    } else if(absError > HALF_STEP_MINIMUM_ANGLE) {
        setStepMode(HALF_STEP);
    } else if(absError > QUARTER_STEP_MINIMUM_ANGLE) {
        setStepMode(QUARTER_STEP);
    } else if(absError > EIGHTH_STEP_MINIMUM_ANGLE) {
        setStepMode(EIGHTH_STEP);
    } else {
        setStepMode(SIXTEENTH_STEP);
    }
    uint16_t motorStepCount = uint16_t(absError/DEGREES_PER_STEP)*stepMode;
    steps(motorStepCount);
}

