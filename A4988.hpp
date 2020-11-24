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
#define STEP_OSC  100.0     //!< 1us minimum, measured in ms 
#define STEP_DELAY 2        //!< DigitalOut delay

//!< Setting step direction based on the MSx outputs
#define FULL_STEP       0 //!< Low Low Low
#define HALF_STEP       1 //!< High Low Low
#define QUARTER_STEP    2 //!< Low High Low
#define EIGHTH_STEP     3 //!< High High Low
#define SIXTEENTH_STEP  4 //!< High High High

#define MS1_MASK    0x01
#define MS2_MASK    0x02    //<! Binary shift output >> 1
#define MS3_MASK    0x04    //<! Binary shift output >> 2

#define NUM_STEP_MODES 5
const char STEP_MODES[NUM_STEP_MODES] = { 
    FULL_STEP, 
    HALF_STEP, 
    QUARTER_STEP, 
    EIGHTH_STEP, 
    SIXTEENTH_STEP
    };

#define FORWARD     0
#define REVERSE     1
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
    A4988(DigitalOut step, DigitalOut dir, DigitalOut ms1, DigitalOut ms2, DigitalOut ms3);
    
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
    void setStepMode(uint8_t);
    void incStepMode();
    void decStepMode();
    /*!
        Directly sets state of step pin
        \param state HIGH/LOW
    */
    void setStep(uint8_t state);

    /*!
        Sets direction mode
        When no argument is passed, direction is toggled
        \param char mode enum for direction mode
    */
    void setDirMode();
    void setDirMode(uint8_t mode) ;
    
    /*!
        Increments stepper motor desired number of steps
        \param int stepCount number of steps to progress
    */
    void increment(uint8_t stepCount = 1) ;

    /*!
        Get Functions
     */
    int getStepMode() ;
    char getDirMode() ;
 

};

#endif 
