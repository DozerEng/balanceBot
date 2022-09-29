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
PID_Controller::PID_Controller (const double kp, const double ki, const double kd, const double dt, const double controllerLimitMax, const double controllerLimitMin, const double tau) : 
    kp(kp),
    ki(ki),
    kd(kd),
    dt(dt),
    controllerLimitMax(controllerLimitMax),
    controllerLimitMin(controllerLimitMin), 
    tau(tau) { 
    integratorLimitMax = controllerLimitMax;
    integratorLimitMin = controllerLimitMin;
    };


// Implementation of controlStep()
double PID_Controller::controlStep (const double measurement, double setpoint) {
    double error = (setpoint - measurement);
    proportional = kp * error; 

    integrator = integrator +  ki * 0.5* dt * (error + previousError);
    //!< Dynamic integrator anti-windup
    //double integratorLimitMax, integratorLimitMin;
    //!< Compute limits  
    /*if (controllerLimitMax > proportional) {
        integratorLimitMax = controllerLimitMax - proportional;
        // fprintf(stdout, "TP1\n\r");
    } else {
        integratorLimitMax = 0.0;
        fprintf(stdout, "TP2\n\r");
    }
    if (controllerLimitMin < proportional) {
        integratorLimitMin = controllerLimitMin - proportional;        
        // fprintf(stdout, "TP3\n\r");
    } else {
        integratorLimitMin = 0.0;
        fprintf(stdout, "TP4\n\r");
    }*/
    //!< Clamp integral
    // if (integrator > integratorLimitMax) {
    //     integrator = integratorLimitMax;
    //     // fprintf(stdout, "TP: Max integral limit hit\n\r");
    // } else if (integrator < integratorLimitMin) {
    //     integrator = integratorLimitMin;
    //     // fprintf(stdout, "TP: Min integral limit hit\n\r");
    // }

    //!< Derivative - Band-limited differentiator
    differentiator  = - 2.0 * kd * (measurement - previousMeasurement) 
                    + (2.0 * tau - dt) * differentiator / (2.0 * tau + dt);
    
    //!< Calculate new control variable
    double controlVariable =  proportional + integrator - kd * differentiator; 
    //!< Apply controller limits
    // if (controlVariable > controllerLimitMax) {
    //     controlVariable = controllerLimitMax;
    // } else if (controlVariable < controllerLimitMin) {
    //     controlVariable = controllerLimitMin;
    // }

    //!< Store required values for next function call
    previousOutput = controlVariable;
    previousMeasurement = measurement;
    previousError = error;
    // fprintf(stdout, "kd: %0.4f ki: %0.4f kd %0.4f\n\r", kp, ki, kd);
    // fprintf(stdout, "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n\r", measurement, error, proportional, integrator, differentiator, controlVariable);

    // This print statement is for MATLAB data logging
    // fprintf(stdout, "%f %f %f %f\n\r", dt, input, error, controlVariable);

    return controlVariable;
}

void PID_Controller::setGain(const double newKp, const double newKi, const double newKd) {
    kp = newKp;
    ki = newKi; 
    kd = newKd;
    resetPID();
}

void PID_Controller::resetPID(void){
    proportional = 0.0;
    integrator = 0.0;
    differentiator = 0.0;

    previousError = 0.0;
    previousOutput = 0.0; 
    previousMeasurement = 0.0;
}