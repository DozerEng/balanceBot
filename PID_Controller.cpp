/*!
 * \class PID_Controller
 * \file PID_Controller.cpp
 * \brief Class for a PID controller
 * 
 * \author Michael Pillon
 * 
 */

#include "PID_Controller.hpp"

// Constructor 
PID_Controller::PID_Controller (const double propGain, const double integralGain, const double derivativeGain, FILE *printPID) : 
    kp(propGain),
    ki(integralGain),
    kd(derivativeGain) {
        pidOut = printPID;
};

// Start PID - Initializes starts the timer
void PID_Controller::start() {
    timer.start();
}
// Restart PID - Initializes starts the timer
void PID_Controller::restart() {
    deltaT = 0.0;
    timer.reset();
    timer.start();
}
// Stop PID - Stops and resets PID timer
void PID_Controller::stop() {
    timer.stop();
    timer.reset();
}

// Implementation of controlStep()
double PID_Controller::controlStep (const double measurement, double setpoint) {
    deltaT = timer.read();
    timer.reset();
    
    double error = (setpoint - measurement);
    //!< Numerical approximation for integradl:
    double integral = 0.9 * previousIntegral + 0.1 * error;
    

    //!< Calculate deltaT

    //!< Calculate new input
    double controlVariable = setpoint + kp * error + deltaT * ki * integral - ( kd / deltaT ) * ( measurement - previousOutput );

    //!< Store required values for next function call
    previousIntegral = integral; 
    previousOutput = measurement;

    fprintf(stdout, "%f %f %f %f\n\r", deltaT, measurement, error, controlVariable);

    
    return controlVariable;
}