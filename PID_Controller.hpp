/*!
 * \file PID_Controller.hpp
 * 
 * \author Michael Pillon
 *
 *  PID Controller
 *
 *  References:
 *          - Philes Lab PID implmentation code: https://github.com/pms67/PID/blob/master/PID.c
 *
 *  
 */


#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "mbed.h"
#include "stdio.h"

class PID_Controller {
private:
    double kp, ki, kd; 

    double proportional = 0.0;
    double integrator = 0.0;
    double differentiator = 0.0;

    double previousError = 0.0;
    double previousOutput = 0.0; 
    double previousMeasurement = 0.0;
    
    double controllerLimitMax, controllerLimitMin;
    double integratorLimitMax, integratorLimitMin;

    /*!
     * Derivative low-pass filter time constant
     * The time constant of the filter (-3dB frequency in Hz, fc = 1 / (2*pi*tau)). 
     * A larger value of tau means the signal is filtered more heavily. 
     * As tau approaches zero, the differentiator approaches a 'pure differentiator' with no filtering.
     */
    double tau; 
    
    //!< Sample time
    double dt; 
    
public:
    /*!
     * Constructor just copies the gain.
     *
     * \param kp The proportional gain
     * \param ki The integral gain
     * \param kd The derivative gain
     * \param dt Constant period in ms
     * \param controllerLimitMax Absolute physical limits of controller
     * \param controllerLimitMin Absolute physical limits of controller
     * \param tau Derivative low-pass filter time constant
     */
    PID_Controller (const double kp, const double ki, const double kd, const double dt, const double controllerLimitMax, const double controllerLimitMin, const double tau);
    
    double controlStep (double measurement, double setpoint);
    void setGain(const double newKp, const double newKi, const double newKd);
    void resetPID(void);
};


#endif

