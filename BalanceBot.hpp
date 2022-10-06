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
#include <string>
#include "RGB_LED.hpp"
#include <cstdint>

#ifndef M_PI
    //M_PI unavailable by default on mbed
    #define M_PI           3.14159265358979323846
#endif

#ifndef BALANCE_BOT_H 
#define BALANCE_BOT_H 
/*!
    BalanceBot  - 2 Wheel self balancing robot
 */
class BalanceBot {

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
public:
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
// #define PB_MASK   0x00800007
// #define PB_PORT  Port2
#define TOP_PB  p6
#define BOTTOM_PB  p7
#define LIMIT_SWITCH  p5
#define PRESSED     0
#define RELEASED    1
//!< RGB LED
// #define RGB_MASK 0x00000038
// #define RGB_PORT  Port2
#define RGB_TYPE RGB_LED::COMMON_CATHODE 
#define RGB_R p20
#define RGB_G p19
#define RGB_B p8
#define LED_ON 1
#define LED_OFF 0


/*! 
    Constants
 */
#define ROBOT_ON        true
#define ROBOT_OFF       false
#define BALANCE_POINT   0.00 //!< Default balancing angle in degrees

#define RADIANS_TO_DEGREES  57.2957795130823208767981548

//!< PID constants
#define KP 1.0
#define KI 0.00
#define KD 0.00
#define CONTROLLER_LIMIT_MAX 125 //!< Maximum error allowed per PID loop
#define CONTROLLER_LIMIT_MIN -125
 //!< Derivative LPF time constant in seconds: fc =  1/(2*pi*tau)
#define TAU 0.001
#define DT 0.001 // PID rate in seconds
#define DT_MS 1 // integer: PID rate in milliseconds 
#define IMU_SAMPLING_RATE 0.001 //Sensor sample rate in seconds
#define IMU_SAMPLING_RATE_MS 1.0
#define INTEGRAL_WINDUP_LIMIT 15

//!< PID tuning increments
#define KP_INCREMENT 0.1
#define KI_INCREMENT 0.25
#define KD_INCREMENT 0.01
//!< Set point tuning increment
#define SET_POINT_INCREMENT 0.1
//!< Complimentary filter constant
#define ALPHA   0.005

//!< Motor system transistion points
#define FULL_STEP_MINIMUM_ANGLE         4.0
#define HALF_STEP_MINIMUM_ANGLE         2.0
#define QUARTER_STEP_MINIMUM_ANGLE      1.0
#define EIGHTH_STEP_MINIMUM_ANGLE       0.5
#define SIXTEENTH_STEP_MINIMUM_ANGLE    0.5  // Limit for motion


private: 
    FILE* bbOut;// = stdout; //For logging data
    FILE* bbErr;// = stderr; //For logging errors
    bool logControlData = false;

    //!< IMU
    MPU6050 mpu;
    
    //!< PID Controller
    PID_Controller pid;    
    double setPoint = BALANCE_POINT; // Approximate balance point in degrees, should be set by a calibration function
    double currentTilt = 0.0;
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
    #define BUTTON_DEBOUNCE 5 // in ms
    DigitalIn topPushButton;
    DigitalIn bottomPushButton;
    DigitalIn limitSwitch;
    // PortIn pbs;

    RGB_LED rgbLED;

    /*! 
        Application flow
     */
    Ticker imuTicker;
    void imuISR(void);
    void imuHandler(void);
    Thread imuThread;
    EventQueue imuQueue;
    
    Thread controlThread;
    EventQueue controlQueue;
    void controlSystem();
    Thread motorThread;
    EventQueue motorQueue;
    void motorSystem(double error);
    void setMotorDirection(double error);

    Thread buttonThread;
    EventQueue buttonQueue;
    void handlePBs(const bool initialize = false);
             
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
    void updateTilt(void);

    /*!
        Test functions
        -   These functions are all in BalanceBotTests.cpp and test general functionality of the robot and relevant methods.
     */
    void runAllTests(void);
    void testWheels(uint8_t scaleFactor = 1);

};

#endif

