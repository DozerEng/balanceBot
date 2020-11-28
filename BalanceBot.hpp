/*!
 * \class BalanceBot
 * \file BalanceBot.hpp
 * \brief Class for a 2 wheel self balancing robot
 * \author Michael Pillon
 *
 * 
 * Features:
 *          2 Stepper motors for wheel using A4988 drivers
 *          MPU6050 IMU sensor for orienation measurements
 *          PID Controller for stabilizing system
 *          
 *          More to come...
 * 
 */

#include "mbed.h"
#include "A4988.hpp"
#include "MPU6050.hpp"
// #include "PID_Controller.hpp"

#include <cmath>
#ifndef M_PI
    #define M_PI           3.14159265358979323846
#endif

 #ifndef BALANCE_BOT_H 
 #define BALANCE_BOT_H 

/*! 
    Constants
 */
#define BALANCE_POINT           0.0 //!< Tilt angle to balance at

#define DEGREES_PER_STEP        1.8 //<! NEMA 17 Stepper Motor
#define STEPS_PER_REVOLUTION    200 //!< # of FULL_STEP 
#define NUM_TILT_SAMPLES        10
/*!
    Data Types
 */


/*!
    BalanceBot 

    2 Wheel self balancing robot
 */
class BalanceBot {
private: 

    A4988* leftWheel;
    A4988* rightWheel;

    uint8_t stepMode;
    double setPoint = BALANCE_POINT;
    
public:
    MPU6050 mpu;
    BalanceBot (A4988 *lw, A4988* rw, I2C* i2c);

    void step(const uint8_t count = 1);

    void setStepMode(const uint8_t mode);
    void incStepMode();
    void decStepMode();

    void setDirection(void);
    void setDirection(const uint8_t dir);

    double getTilt(void);
    double getTiltDegrees(void);
    
    void propBalance(void);
};

#endif

