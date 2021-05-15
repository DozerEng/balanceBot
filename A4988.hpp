/*!
 * \class A4988
 * \file A4988.hpp
 * \brief Class for A4988 servo motor control board
 * 
 * \author Michael Pillon
 * 
 */

#include "mbed.h"

#ifndef A4988_hpp
#define A4988_hpp

/*! 
    Constants
 */
#define STEP_DELAY 1000     //!< DigitalOut delay in microseconds
#define DEGREES_PER_STEP        1.8 //<! For NEMA 17 Stepper Motor
#define STEPS_PER_REVOLUTION    200 //!< # of FULL_STEP per full revolution

//!< Setting step direction based on the MSx outputs
#define FULL_STEP       1 //!< Low Low Low
#define HALF_STEP       2 //!< High Low Low
#define QUARTER_STEP    4 //!< Low High Low
#define EIGHTH_STEP     8 //!< High High Low
#define SIXTEENTH_STEP  16 //!< High High High

#define MS1_MASK    0x01
#define MS2_MASK    0x02    //<! Binary shift output >> 1
#define MS3_MASK    0x04    //<! Binary shift output >> 2

#define FORWARD     0
#define REVERSE     1
#define LEFT_TURN   2
#define RIGHT_TURN  3

#define HIGH        1
#define LOW         0

/*!
    Data Types
 */


/*!
    A4988 

    Servo motor control program
 */

class A4988 {
private: 
   DigitalOut step;
   DigitalOut dir;
   DigitalOut ms1;
   DigitalOut ms2;
   DigitalOut ms3;

   int microStepMode;
   char currentDirection;

public: 
    /*!
        Constructor
        \param step Pin for step signal
        \param dir Pin for direction signal
        \param ms1 Pin for MS1 microstepping mode register
        \param ms2 Pin for MS2 microstepping mode register
        \param ms3 Pin for MS3 microstepping mode register
    */
    A4988(PinName step, PinName dir, PinName ms1, PinName ms2, PinName ms3);
    
    /*!
        Sets MSx outputs for requested step mode
        
        FULL_STEP => Low Low Low
        HALF_STEP => High Low Low
        QUARTER_STEP => Low High Low
        EIGHTH_STEP => High High Low
        SIXTEENTH_STEP => High High High

        \param newMode set mode for Microstepping
    */
    void setStepMode();
    void setStepMode(const uint8_t newMode);
    void incStepMode();
    void decStepMode();
    /*!
        Directly sets state of step pin
        \param state HIGH/LOW
    */
    void setStep(const uint8_t state);

    /*!
        Sets direction mode
        When no argument is passed, direction is toggled
        \param char mode enum for direction mode
    */
    void setDirMode();
    void setDirMode(const uint8_t mode) ;
    
    /*!
        Increments stepper motor desired number of steps
        \param int stepCount number of steps to progress
    */
    void increment(const uint16_t stepCount = 1) ;

    /*!
        Get Functions
     */
    int getStepMode() ;
    char getDirMode() ;
 

};

#endif 
