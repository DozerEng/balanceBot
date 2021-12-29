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
    double previousOutput = 0.0;  
    double previousInput = 0.0;
    double previousIntegral = 0.0;

    Timer timer;
    double deltaT = 0;

    FILE *pidOut;
    
public:
    /*!
     * Constructor just copies the gain.
     *
     * \param propGain The proportional gain
     * \param integralGain The integral gain
     * \param derivativeGain The derivative gain
     */
    PID_Controller (const double propGain, const double integralGain, const double derivativeGain, FILE *printPID = stdout);
    
    // Start PID - Initializes starts the timer
    void start();
    // Restart PID - Restarts timer counting value back to 0 while running or not.
    void restart();
    // Stop PID - Stops and resets PID timer
    void stop();

    
    // Our promise to implement the function from the abstract base class
    double controlStep (double plantOutput, double setpoint);
};


#endif

