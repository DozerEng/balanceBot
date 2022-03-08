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
    pid(KP, KI, KD, DT, bbOut),
    bothWheels(MOTOR_PORT, STEP_MASK),
    leftWheel(L_STEP, L_DIR, L_MS1, L_MS2, L_MS3),
    rightWheel(R_STEP, R_DIR, R_MS1, R_MS2, R_MS3),
    stepMode(FULL_STEP),
    // Inputs
    topPB(TOP_PB),
    botPB(BOT_PB),
    limSW(LIMIT_SWITCH),
    pbs(PB_PORT, PB_MASK),
    // Outputs
    rgb_r(RGB_R),
    rgb_g(RGB_G),
    rgb_b(RGB_B),
    rgb(RGB_PORT, RGB_MASK),
    bbOut(stdout), // stdout, "./fileName.txt", "a"
    bbErr(stderr) // stderr, 
    {


    /*!
        Initalize modes of the robot
     */
    
    setStepMode(stepMode);
    setDirection(FORWARD); 
    mpu.setDLPF(DLPF_CFG_4); //!< Digital Low Pass Filter

    /*!
        Start robot/threads or run hardware tests
     */
    runAllTests();
    
    buttonThread.start(callback(this, &BalanceBot::handlePBs));
    buttonThread.set_priority(osPriorityHigh1);
    /*
    controllerThread.start(callback(this, &BalanceBot::controlSystem));
    controllerThread.set_priority(osPriorityRealtime1);
    
    motorThread.start(callback(this, &BalanceBot::motorSystem));
    motorThread.set_priority(osPriorityAboveNormal1);
    */
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
void BalanceBot::handlePBs() {
    while(true) {
        if(topPB == 0) {
            while(topPB == 0){
                //!< Wait for release of button
            }
            fprintf(bbOut, "Top PB pressed\n\r");
            //double temperature = mpu.getTemp();
            fprintf(bbOut, "Current Temperature %0.1f degrees celcius\n\r", mpu.getTemp());         
        }
        if(limSW == 0 ) {
            while(limSW == 0){
                //!< Wait for release of button
            }
            /*
                do stuff for bottom PB
            */
            fprintf(bbOut, "Limit switch pressed\n\r");
        }
        if(botPB == 0 ) {
            while(botPB == 0){
                //!< Wait for release of button
            }
            /*
                do stuff for bottom PB
            */
            fprintf(bbOut, "Bottom PB pressed\n\r");
        }  
    
    ThisThread::sleep_for(100);
    }
}

/*!
    \fn plant()
    Thread for handing movement of the robots wheels
 */
void BalanceBot::motorSystem() {
    while(true) {
        //!< Check for change in error value from controller
        static double previousError;
        double absError = abs(error);

        //!< Set motor resolution based on tilt
        if(absError < 1.0) {
            ThisThread::sleep_for(10); 
        } else if((absError < 15.0) && (stepMode != SIXTEENTH_STEP)){
            setStepMode(SIXTEENTH_STEP);
        } else if((absError < 25.0) && (stepMode != QUARTER_STEP)) {
            setStepMode(QUARTER_STEP);
        } else if (stepMode != FULL_STEP) {
            setStepMode(FULL_STEP);
            
        }
        // Handle direction and step count
        if(error != previousError) {
            previousError = error;

            if(error < 0) {
                setDirection(FORWARD);
            } else {
                setDirection(REVERSE);
            }

             motorStepCount = uint16_t(absError*0.5/DEGREES_PER_STEP)*stepMode;
        }


        
        //fprintf(bbOut, "Steps: %i error: %0.3f dirMode: %i\n\r", motorStepCount, error, directionMode);
        steps(motorStepCount);
        motorStepCount = 0;
        // ThisThread::sleep_for(10);  
    }

}


/*!
    \fn controlSystem
    Thread for finding the azimuth tilt of robot and saving it in private variable "azimuth"
        - Hardware: MPU6050
        - Controller: PID_Controller
 */
void BalanceBot::controlSystem() {
    fprintf(bbOut, "Tilt \terror \tcontrolVariable\n\r" );

    while(true) {
        // Read push buttons
        //uint32_t inputReading = pbs;
        // uint32_t outputValue = inputReading << 3;
        //rgb = ~outputValue;

        // 1) Calculate tilt/error from MPU6050 (Process variable)
        double mpuReading = getTiltDegrees();

        // 2) Setpoint and plant output get passed to PID controller
        error = pid.controlStep(mpuReading, setPoint);

        ThisThread::sleep_for(DT_MS);  

    }
    
    fprintf(bbOut, "Control system shutting down....\n\r");
    ThisThread::sleep_for(1500); 
    fprintf(bbOut, "Control system successfully shut down\n\r");
}
