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



class PID_Controller {
private:
    #define KC 0.7
    #define TI 1
    #define TD 1

    double kc = KC;  // The proportional gain
    double ti = TI;    // Integral Term
    double td = TD;    // Derivative Term
    double previousOutput = 0.0;  //previous output from plant
    double previousQ = 0.0; //Previous q value for integral sum calculation

    Timer timer;
    double deltaT = 0;
    
    
    
public:
    /*!
     * Constructor just copies the gain.
     *
     * \param propGain The proportional gain
     * \param integralGain The integral gain
     * \param derivativeGain The derivative gain
     */
    PID_Controller (const double propGain, const double integralGain, const double derivativeGain);
    
    // Start PID - Initializes starts the timer
    void start();
    // Stop PID - Stops and resets PID timer
    void stop();

    
    // Our promise to implement the function from the abstract base class
    double controlStep (double plantOutput, double setpoint);
};


#endif

