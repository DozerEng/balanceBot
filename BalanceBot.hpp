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
#include "PID_Controller.hpp"

#include <cmath>
#ifndef M_PI
    //M_PI unavailable by default on mbed
    #define M_PI           3.14159265358979323846
#endif

#ifndef BALANCE_BOT_H 
#define BALANCE_BOT_H 
/*!
    Hardware Defines
 */
 /*!
    \def STEP_MASK
    Match L_STEP and R_STEP pins. Both step pins MUST be on the same port for synchronous wheel movement.
    p19 => P1.30 (Port1.30)
    p20 => P1.31 (Port1.31)
    Mask: 0xC0000000 == 0b11000000000000000000000000000000
    
 */
#define STEP_MASK_ON    0xC0000000
#define STEP_MASK_OFF   0
//!< Left Wheel Pins
#define L_STEP  p19
#define L_DIR   p17
#define L_MS1   p14
#define L_MS2   p15
#define L_MS3   p16
//!< Right Wheel Pins
#define R_STEP  p20
#define R_DIR   p18
#define R_MS1   p11
#define R_MS2   p12
#define R_MS3   p13
//!< Pushbuttons
#define TOP_PB  p5
#define BOT_PB  p6

/*! 
    Constants
 */
#define ROBOT_ON        true
#define ROBOT_OFF       false
#define BALANCE_POINT   -6.0 //!< Tilt angle for balancing with no movement

/*!
    Data Types
 */


/*!
    BalanceBot 

    2 Wheel self balancing robot
 */
class BalanceBot {
private: 
    //!< IMU
    MPU6050 mpu;
    PID_Controller pid;
    
    //!< Motors
    PortOut bothWheels; //!< Step pins for both wheels for synchronized stepping
    A4988 leftWheel;
    A4988 rightWheel;
    //!< Onboard Pushbuttons
    DigitalIn topPB;
    DigitalIn botPB;

    /*!
        Measures azimuth tilt angle of robot using MPU6050 and PID_Controller
        \sa balanceThreadRoutine
     */
    Thread balanceThread;

    //!< Registers
    uint8_t stepMode;
    double setPoint;
    
    /*!
        Private methods
    */
    void balanceThreadRoutine(void);
    void runMotors(double azimuth);
         
public:
    BalanceBot (I2C* i2c);

    void step(const uint16_t count = 1);

    void setStepMode(const uint8_t mode);
    void incStepMode();
    void decStepMode();

    void setDirection(void);
    void setDirection(const uint8_t dir);

    double getTilt(void);
    double getTiltDegrees(void);
    
    void handlePBs(void);

    /*!
        Test functions
        These functions are all in BalanceBotTests.cpp and test general functionality of the robot and relevant methods.
     */
    void runAllTests(void);
    void testWheels(void);

};

#endif

