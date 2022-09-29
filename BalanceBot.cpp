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
    pid(KP, KI, KD, DT, CONTROLLER_LIMIT_MAX, CONTROLLER_LIMIT_MIN, TAU),
    bothWheels(MOTOR_PORT, STEP_MASK),
    leftWheel(L_STEP, L_DIR, L_MS1, L_MS2, L_MS3),
    rightWheel(R_STEP, R_DIR, R_MS1, R_MS2, R_MS3),
    stepMode(FULL_STEP),
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
    mpu.setDLPF(DLPF_CFG_0); //!< Digital Low Pass Filter

    /*!
        Start robot/threads or run hardware tests
     */
    if(limitSwitch == PRESSED){
        //runAllTests();
        testWheels(2);
    }
    
    /*! 
        IMU interrupt thread
    */
    imuTicker.attach(callback(this, &BalanceBot::imuISR), IMU_SAMPLING_RATE);
    imuThread.start(callback(&imuQueue,  &EventQueue::dispatch_forever));
    imuThread.set_priority(osPriorityHigh);

    /*!
        Control loop thread 
    */
    controlThread.start(callback(&controlQueue, &EventQueue::dispatch_forever));
    controlThread.set_priority(osPriorityAboveNormal);
    controlQueue.call_every(DT_MS, callback(this, &BalanceBot::controlSystem));
    
    /*
        Motor thread
    */
    motorThread.start(callback(&motorQueue, &EventQueue::dispatch_forever));
    motorThread.set_priority(osPriorityNormal);

    /*
        Button thread
    */
    buttonThread.start(callback(&buttonQueue, &EventQueue::dispatch_forever));
    buttonThread.set_priority(osPriorityRealtime); // Buttons should interrupt everything else
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
    if(limitSwitch == PRESSED ) {
        ThisThread::sleep_for(BUTTON_DEBOUNCE); 
        while(limitSwitch == PRESSED ) {
            // Change tilt angle while limit switch is held
            if(topPushButton == PRESSED ){ 
                setPoint += SET_POINT_INCREMENT;
                while(topPushButton == PRESSED);
            } else if ( bottomPushButton == PRESSED ){
                setPoint -= SET_POINT_INCREMENT;
                while(bottomPushButton == PRESSED);
            } else {
                continue;
            } 
            fprintf(bbOut, "Changed set point to: %0.2f degrees\n\r", setPoint);
        }
        if(activeConstant == &kp) {
            activeConstant = &ki;
            constantIncrement = KI_INCREMENT;
            rgbLED.setGreen();
            fprintf(bbOut, "Changed active constant to ki = %0.5f\n\r", *activeConstant);
        } else if(activeConstant == &ki) {
            activeConstant = &kd;
            constantIncrement = KD_INCREMENT;
            rgbLED.setBlue();
            fprintf(bbOut, "Changed active constant to kd = %0.5f\n\r", *activeConstant);
        } else {
            activeConstant = &kp;
            constantIncrement = KP_INCREMENT;
            rgbLED.setRed();
            fprintf(bbOut, "Changed active constant to kp = %0.5f\n\r", *activeConstant);
        } 
    }
    /*!
        Left Push Button increments active constant
    */
    if(topPushButton == PRESSED) { // Test board wiring got fucked up when replacing a switch. Normally 0 == PRESSED
        ThisThread::sleep_for(BUTTON_DEBOUNCE); 
        while(topPushButton == PRESSED);
        *activeConstant = *activeConstant + constantIncrement;
        pid.setGain(kp, ki, kd);
        fprintf(bbOut, "Increased active constant %0.5f\n\r", *activeConstant);
    }
    /*!
        Right Push Button decrements active constant
    */
    if(bottomPushButton == PRESSED) {
        ThisThread::sleep_for(BUTTON_DEBOUNCE);            
        while(bottomPushButton == PRESSED);
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
    imuQueue.call(callback(this, &BalanceBot::imuHandler));
}

void BalanceBot::imuHandler() {
    getTilt();

    if(logControlData) {
        // printQueue.call(fprintf, bbOut, "%.2f\n\r", mpuReading);
        fprintf(bbOut, "%.2f \n\r", tiltAngle);
    }
}

/*!
    \fn getTilt()
    Calculate approximate tilt of robot based on accelerometer data.
        tilt = actualAzimuth - tiltError
        Arguments for atan2() may need adjusting depending on orientation of MPU6050.
    \return azimuth angle of robot. Radians by default.
 */
void BalanceBot::getTilt() {
    /* This is the old getTilt() function
    int16_t xAcceleration, yAcceleration, zAcceleration;
    mpu.getAccel(&xAcceleration, &yAcceleration, &zAcceleration);

    int16_t gyroX, gyroY, gyroZ;
    mpu.getGyro(&gyroX, &gyroY, &gyroZ); 
   
    double azimuth = atan2( -zAcceleration, xAcceleration );
    */
    
    int16_t xAcceleration, yAcceleration, zAcceleration, gyroX, gyroY, gyroZ;
    mpu.getAccel(&xAcceleration, &yAcceleration, &zAcceleration);
    mpu.getGyro(&gyroX, &gyroY, &gyroZ); 
    
    // FOR ACC: USE Z
    // fprintf(bbOut, "xAcc: %i  \tyAcc: %i  \tzAcc: %i  \txGyro: %i  \tyGyro: %i  \tzGyro: %i\n\r", xAcceleration, yAcceleration, zAcceleration, gyroX, gyroY, gyroZ);

    // accelerometer scale set to 2G: 16384 LSB/g
    double tiltAccelerometer = atan2( -zAcceleration, xAcceleration ) * RADIANS_TO_DEGREES;
    // Gyro scale set to 500 degrees per second: 65.5 LSB/°/s 130.07
    double tiltGyro = IMU_SAMPLING_RATE * ( - gyroY / ACCEL_FACTOR_2G);
    tiltAngle = ALPHA * tiltAccelerometer + (1.0 - ALPHA) * ( tiltAngle + tiltGyro);
    // fprintf(bbOut, "Accelerometer Angle: %0.3f \t\tGyro Change: %0.3f\t\tTilt: %0.3f\n\r", tiltAccelerometer, tiltGyro, tiltAngle);



    /* Phils lab complimetary filter:
    static double phiHat_rad = 0.0;
    static double thetaHat_rad = 0.0;

    double phiHat_acc_rad = atan2(yAcceleration, zAcceleration);
    double thetaHat_acc_rad = atan2(xAcceleration, 9.80665); // Gravitational acceleration = 9.80665

    // x = r (phi), y = q (theta), z = p
    // Transform body rates to Euler rates
    double phiDot_rps = gyroZ + tan(thetaHat_rad) * (sin(phiHat_rad) * gyroY + cos(phiHat_rad) * gyroX);
    double thetaDot_rps = cos(phiHat_rad) * gyroY - sin(phiHat_rad) * gyroX;
    
    phiHat_rad = alpha * phiHat_acc_rad + (1.0f - alpha) * (phiHat_rad + DT * phiDot_rps);
    thetaHat_rad = alpha * thetaHat_acc_rad + (1.0f - alpha) * (thetaHat_rad + DT * thetaDot_rps);

    fprintf(bbOut, "PhiHat: %0.3f\tthetaHat: %0.3f\n\r", phiHat_rad*RADIANS_TO_DEGREES, thetaHat_rad*RADIANS_TO_DEGREES);
    */

    //return tiltAngle;
}

/*!
    \fn controlSystem
    Thread for finding the azimuth tilt of robot and saving it in private variable "azimuth"
        - Hardware: MPU6050
        - Controller: PID_Controller
 */
void BalanceBot::controlSystem() {
    // Sample currentTitle  
    double currentTilt = tiltAngle;
    
    // Set motor direction
    double error = setPoint - currentTilt;
    setMotorDirection(error);
    
    // Apply PID
    error = pid.controlStep(currentTilt, setPoint);

    // Stop motors from whatever they're doing and them make em do stuff
    static int motorQueueID = 0;
    // Clear control queue
    if(motorQueueID) motorQueue.cancel(motorQueueID);
    
    // Call control queue with new value
    motorQueueID = motorQueue.call(callback(this, &BalanceBot::motorSystem), error);
}


/*!
    \fn motorSystem()
    Handlerfor movement of the robots wheels
 */
void BalanceBot::motorSystem(double error) {
    // fprintf(bbOut, "%0.2f\n\r", error);
    double absError = abs(error);
    uint16_t motorStepCount = 0;
    if(absError > FULL_STEP_MINIMUM_ANGLE) {
        setStepMode(FULL_STEP);
        motorStepCount = uint16_t(absError/DEGREES_PER_STEP)*stepMode;
        steps(motorStepCount);
    } else if(absError > HALF_STEP_MINIMUM_ANGLE) {
        setStepMode(HALF_STEP);
        motorStepCount = uint16_t(absError/DEGREES_PER_STEP)*stepMode;
        steps(motorStepCount);
    } else if(absError > QUARTER_STEP_MINIMUM_ANGLE) {
        setStepMode(QUARTER_STEP);
        motorStepCount = uint16_t(absError/DEGREES_PER_STEP)*stepMode;
        steps(motorStepCount);
    }else if(absError > EIGHTH_STEP_MINIMUM_ANGLE) {
        setStepMode(EIGHTH_STEP);
        motorStepCount = uint16_t(absError/DEGREES_PER_STEP)*stepMode;
        steps(motorStepCount);
    }else if(absError > SIXTEENTH_STEP_MINIMUM_ANGLE) {
        setStepMode(SIXTEENTH_STEP);
        motorStepCount = uint16_t(absError/DEGREES_PER_STEP)*stepMode;
        steps(motorStepCount);
    }
    
}



/*!
    \fn setMotorDirection()
    Set direction and es
    \param tilt tilt error
 */
void BalanceBot::setMotorDirection(double error) {
    if(error < 0.0) {
        setDirection(FORWARD);
    } else {
        setDirection(REVERSE);
    }

    // double absError = abs(error);

    // if(absError > FULL_STEP_MINIMUM_ANGLE) {
    //     setStepMode(FULL_STEP);
    // } else if(absError > HALF_STEP_MINIMUM_ANGLE) {
    //     setStepMode(HALF_STEP);
    // } else if(absError > QUARTER_STEP_MINIMUM_ANGLE) {
    //     setStepMode(QUARTER_STEP);
    // } else if(absError > EIGHTH_STEP_MINIMUM_ANGLE) {
    //     setStepMode(EIGHTH_STEP);
    // } else {
    //     setStepMode(SIXTEENTH_STEP);
    // }
}

