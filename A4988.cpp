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
    A4988::A4988(DigitalOut step, DigitalOut dir, DigitalOut ms1, DigitalOut ms2, DigitalOut ms3) : 
        step(step), dir(dir), ms1(ms1), ms2(ms2), ms3(ms3) {
            printf("Initializing A4988...\n\r");
            setStepMode(); //!< Default to full step mode
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
    void A4988::setStepMode(uint8_t newMode) {
        switch (newMode) {
            case FULL_STEP:
                ms1 = 0;
                ms2 = 0;
                ms3 = 0;
                microStepMode = FULL_STEP;
                printf("Step Mode set to: Full Step\n\r");
                break;
            case HALF_STEP:
                ms1 = 1;
                ms2 = 0;
                ms3 = 0;
                microStepMode = HALF_STEP;
                printf("Step Mode set to: Half Step\n\r");
                break;
            case QUARTER_STEP:
                ms1 = 0;
                ms2 = 1;
                ms3 = 0;
                microStepMode = QUARTER_STEP;
                printf("Step Mode set to: Quarter Step\n\r");
                break;
            case EIGHTH_STEP:
                ms1 = 1;
                ms2 = 1;
                ms3 = 0;
                microStepMode = EIGHTH_STEP;
                printf("Step Mode set to: Eighth Step\n\r");
                break;
            case SIXTEENTH_STEP:
                ms1 = 1;
                ms2 = 1;
                ms3 = 1;
                microStepMode = SIXTEENTH_STEP;
                printf("Step Mode set to: Sixteenth Step\n\r");
                break;
            default:
                printf("Invalid step mode Requested\n\r");
                break;
        }
    }
    /*!
        Increments step mode
    */
    void A4988::incStepMode() {
        if (microStepMode >= (NUM_STEP_MODES - 1)) {
            //!< Rollover from SIXTEENTH_STEP to FULL_STEP
            setStepMode(FULL_STEP);
       } else {
           microStepMode ++;
           setStepMode(microStepMode);
       }
    }
    /*!
        Decrements step mode
    */
    void A4988::decStepMode() {
        if (microStepMode <= 0) {
           //!< Rollover from FULL_STEP to SIXTEENTH_STEP
            setStepMode(SIXTEENTH_STEP);
       } else {
           microStepMode --;
           setStepMode(microStepMode);
       }
    }
    /*!
        Directly sets state of step pin
        \param state
    */
    void A4988::setStep(uint8_t state) {
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
        printf("Direction toggled\n\r");
    }

    void A4988::setDirMode(uint8_t mode) {
        switch (mode) {
            case FORWARD: 
                dir = FORWARD;
                printf("Direction set to forward\n\r");
                break;
            case REVERSE:
                dir = REVERSE;
                printf("Direction set to reverse\n\r");
                break;
            default:
                fprintf(stderr, "Invalid Direction\n\r");
        }
    }
    
    /*!
        Increments stepper motor desired number of steps
        \param int stepCount number of steps to progress
    */
    void A4988::increment(uint8_t stepCount) {
        //printf("Taking %i steps\n\r", stepCount);
        for(int i = 0; i < stepCount; i++) {
            step = HIGH;
            ThisThread::sleep_for(STEP_DELAY);
            step = LOW;
            ThisThread::sleep_for(STEP_DELAY);
        }
    }
    /*!
        Increments stepper motor desired number of steps
        \return int current step mode
    */
    int A4988::getStepMode() {
        printf("Current Step Mode: %i\n\r", microStepMode);
        return microStepMode;
    }
    char A4988::getDirMode() {
        printf("Current Direction Mode: %i\n\r", currentDirection);
        return currentDirection;
    };