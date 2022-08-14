/*!
 * \file PID_Controller.hpp
 * 
 * \author Michael Pillon
 *
 *  PID Controller using a linear approximation for the integeral
 *      - Timer is specifically for the MBED LPC1768 and may need modification
 *          - ToDo: Write TimerDev library
 *  
 */


#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "mbed.h"
#include "stdio.h"

class PID_Controller {
private:
    double kp, ki, kd; 
    double previousError = 0.0;
    double previousOutput = 0.0; 
    double integralSum = 0.0;
    double integralWindupLimit = 0.0;

    double dt = 0;

    FILE *pidOut;
    
public:
    /*!
     * Constructor just copies the gain.
     *
     * \param propGain The proportional gain
     * \param integralGain The integral gain
     * \param derivativeGain The derivative gain
     * \param deltaT Constant period in ms
     */
    PID_Controller (const double kp, const double ki, const double kd, const double dt, const double integralWindupLimit, FILE *printPID = stdout);
    
    double controlStep (double input, double setpoint);
    void setGain(const double newKp, const double newKi, const double newKd);
};


#endif

