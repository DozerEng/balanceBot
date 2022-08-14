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
PID_Controller::PID_Controller (const double kp, const double ki, const double kd, const double dt, const double integralWindupLimit, FILE *printPID) : 
    kp(kp),
    ki(ki),
    kd(kd),
    dt(dt),
    integralWindupLimit(integralWindupLimit) {
        pidOut = printPID;
};


// Implementation of controlStep()
double PID_Controller::controlStep (const double input, double setpoint) {

    double error = (setpoint - input);
    integralSum += dt * error;
    if(integralSum >= integralWindupLimit) {
        integralSum = integralWindupLimit;
    } else if (integralSum <= (-1 * integralWindupLimit)) {
        integralSum = -1 * integralWindupLimit;
    }
    
    double derivative = ( input - previousError ) / dt;
    
    //!< Calculate new input
    double controlVariable =  kp * error + ki * integralSum - kd * derivative; 

    //!< Store required values for next function call
    previousOutput = controlVariable;
    previousError = input;
    //fprintf(stdout, "dt:%f \tIMU:%f \te:%f \tcv:%f\n\r", deltaT, measurement, error, controlVariable);
    
    // This print statement is for MATLAB data logging
    // fprintf(stdout, "%f %f %f %f\n\r", dt, input, error, controlVariable);

    return controlVariable;
}

void PID_Controller::setGain(const double newKp, const double newKi, const double newKd) {
    kp = newKp;
    ki = newKi; 
    kd = newKd;
    previousError = 0.0;
    previousOutput = 0.0; 
    integralSum = 0.0;
}