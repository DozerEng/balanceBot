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
    p15 => P0.23 
    p10 => P0.1 
    Mask: 0x00800002 or 0b00000000100000000000000000000010
    
 */
#define STEP_MASK   0x00800002
#define MOTOR_PORT  Port0
//#define STEP_MASK_OFF   0
//!< Left Wheel Pins
#define L_STEP  p15
#define L_DIR   p14
#define L_MS1   p18
#define L_MS2   p17
#define L_MS3   p16
//!< Right Wheel Pins
#define R_STEP  p10
#define R_DIR   p9
#define R_MS1   p13
#define R_MS2   p12
#define R_MS3   p11
//!< Pushbuttons
#define PB_MASK   0x00800007
#define PB_PORT  Port2
#define TOP_PB  p25
#define BOT_PB  p26
#define LIMIT_SWITCH  p24
//!< RGB LED
#define RGB_MASK 0x00000038
#define RGB_PORT  Port2
#define RGB_R p23
#define RGB_G p22
#define RGB_B p21


/*! 
    Constants
 */
#define ROBOT_ON        true
#define ROBOT_OFF       false
#define BALANCE_POINT   3 //!< Default balancing angle  in degrees

#define RADIANS_TO_DEGREES  57.2957795130823208767981548

#define KC 1.0
#define TI 0.0
#define TD 0.0

/*!
    BalanceBot  - 2 Wheel self balancing robot
 */
class BalanceBot {
private: 
    FILE* bbOut;// = stdout; //For logging data
    FILE* bbErr;// = stderr; //For logging errors

    //!< IMU
    MPU6050 mpu;
    
    //!< PID Controller
    PID_Controller pid;
    double setPoint = BALANCE_POINT; // Approximate balance point in degrees, should be set by a calibration function
    double kc = KC; 
    double ti = TI; 
    double td = TD; 
    
    //!< Motors
    PortOut bothWheels; //!< Step pins for both wheels for synchronized stepping
    A4988 leftWheel;
    A4988 rightWheel;
    uint8_t stepMode;

    //!< Onboard Pushbuttons
    DigitalIn topPB;
    DigitalIn botPB;
    DigitalIn limSW;
    PortIn pbs;
    //!< RGB LED
    DigitalOut rgb_r;
    DigitalOut rgb_g;
    DigitalOut rgb_b;
    PortOut rgb;

    /*! 
        Thread for operating control system
        \sa controlSystem()
     */
    Thread controlSystemThread;

    /*!
        Private methods
    */
    void controlSystem(void);
    void plant(const double controlVariable);
         
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
        -   These functions are all in BalanceBotTests.cpp and test general functionality of the robot and relevant methods.
     */
    void runAllTests(void);
    void testWheels(void);
    void testWheelsFast(void);

};

#endif

