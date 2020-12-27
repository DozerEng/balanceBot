/*!
 * \class A4988
 * \file A4988.cpp
 * 
 * \author Michael Pillon
 * 
 */

#include "mbed.h"
#include "A4988.hpp"


    /*!
        Constructor
        \param step Pin for step signal
        \param dir Pin for direction signal
        \param ms1 Pin for MS1 microstepping mode register
        \param ms2 Pin for MS2 microstepping mode register
        \param ms3 Pin for MS3 microstepping mode register
    */
    A4988::A4988(PinName step, PinName dir, PinName ms1, PinName ms2, PinName ms3) : 
        step(step),
        dir(dir),
        ms1(ms1),
        ms2(ms2),
        ms3(ms3) 
        {
            printf("Initializing A4988...\n\r");
            setStepMode(); //!< Default is full step mode
            setDirMode(FORWARD);
            printf("Successfuly initialized A4988\n\r");
         };
    
    /*!
        Sets MSx outputs for requested step mode
        
        FULL_STEP => Low Low Low
        HALF_STEP => High Low Low
        QUARTER_STEP => Low High Low
        EIGHTH_STEP => High High Low
        SIXTEENTH_STEP => High High High

        \param newMode set mode for Microstepping
    */
    void A4988::setStepMode() {
        this->setStepMode(FULL_STEP);
    }
    void A4988::setStepMode(const uint8_t newMode) {
        switch (newMode) {
            case FULL_STEP:
                ms1 = 0;
                ms2 = 0;
                ms3 = 0;
                break;
            case HALF_STEP:
                ms1 = 1;
                ms2 = 0;
                ms3 = 0;
                break;
            case QUARTER_STEP:
                ms1 = 0;
                ms2 = 1;
                ms3 = 0;
                break;
            case EIGHTH_STEP:
                ms1 = 1;
                ms2 = 1;
                ms3 = 0;
                break;
            case SIXTEENTH_STEP:
                ms1 = 1;
                ms2 = 1;
                ms3 = 1;
                break;
            default:
                printf("Invalid step mode Requested\n\r");
                return;
        }
        microStepMode = newMode;
        //printf("Step Mode set to: 1\\%i\n\r", newMode);
    }
    /*!
        Increments step mode
    */
    void A4988::incStepMode() {
        microStepMode *= 2;
        if (microStepMode >= SIXTEENTH_STEP) {
            microStepMode = SIXTEENTH_STEP;
       }
       setStepMode(microStepMode);
    }
    /*!
        Decrements step mode
    */
    void A4988::decStepMode() {
        microStepMode /= 2;
        if (microStepMode <= 1) {
            microStepMode = FULL_STEP;
        } 
        setStepMode(microStepMode);
    }
    /*!
        Directly sets state of step pin
        \param state
    */
    void A4988::setStep(const uint8_t state) {
        if ( state == LOW ) {
            step = LOW;
        } else {
            step = HIGH;
        }
    }
    /*!
        Sets direction mode
        When no argument is passed, direction is toggled
        \param char mode enum for direction mode
    */
    void A4988::setDirMode() {
        dir = !dir;
        //printf("Direction toggled\n\r");
    }

    void A4988::setDirMode(const uint8_t newDir) {
        if((newDir == FORWARD) || (newDir == REVERSE)) {
            dir = newDir;
            //printf("Direction set to %i\n\r", dir);
        } else {
            fprintf(stderr, "A4988::setDirMode Invalid direction argument: %i\n\r", newDir);
        }
    }
    
    /*!
        Increments stepper motor desired number of steps
        \param int stepCount number of steps to progress
    */
    void A4988::increment(const uint16_t stepCount) {
        //printf("Taking %i steps\n\r", stepCount);
        for(int i = 0; i < stepCount; i++) {
            step = HIGH;
            wait_us(STEP_DELAY);
            step = LOW;
            wait_us(STEP_DELAY);
        }
    }
    /*!
        Increments stepper motor desired number of steps
        \return int current step mode
    */
    int A4988::getStepMode() {
        //printf("Current Step Mode: %i\n\r", microStepMode);
        return microStepMode;
    }
    char A4988::getDirMode() {
        //printf("Current Direction Mode: %i\n\r", currentDirection);
        return currentDirection;
    };