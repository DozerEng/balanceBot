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
#define LEFT_PB  p25
#define RIGHT_PB  p26
#define LIMIT_SWITCH  p24
//!< RGB LED
#define RGB_MASK 0x00000038
#define RGB_PORT  Port2
#define RGB_R p23
#define RGB_G p22
#define RGB_B p21
#define LED_ON 1
#define LED_OFF 0


/*! 
    Constants
 */
#define ROBOT_ON        true
#define ROBOT_OFF       false
#define BALANCE_POINT   1.285 //!< Default balancing angle in degrees

#define RADIANS_TO_DEGREES  57.2957795130823208767981548

#define KP 2.0
#define KI 0.00
#define KD 0.00
#define DT 0.040 // Sensor sample rate in seconds
#define DT_MS 40 // Sensor sample rate in milliseconds
#define INTEGRAL_WINDUP_LIMIT 25.0

#define KP_INCREMENT 0.2
#define KI_INCREMENT 0.2
#define KD_INCREMENT 0.1

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
    Mutex outputLock;
    double error = 0.0;
    uint16_t motorStepCount = 0;
    double setPoint = BALANCE_POINT; // Approximate balance point in degrees, should be set by a calibration function
    double kp = KP; 
    double ki = KI; 
    double kd = KD; 
    double dt = DT;
    
    //!< Motors
    PortOut bothWheels; //!< Step pins for both wheels for synchronized stepping
    A4988 leftWheel;
    A4988 rightWheel;
    uint8_t stepMode;
    uint8_t directionMode;

    //!< Onboard Pushbuttons
    #define BUTTON_CHECK_INTERVAL 100 // in ms
    #define BUTTON_DEBOUNCE 10 // in ms
    DigitalIn leftPushButton;
    DigitalIn rightPushButton;
    DigitalIn limitSwitch;
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
    Thread motorThread;
    Thread controllerThread;
    Thread buttonThread;

    /*!
        Private methods
    */
    void controlSystem(void);
    void motorSystem(void);
         
public:
    BalanceBot (I2C* i2c);

    void step();
    void steps(const uint16_t count = 1);

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

