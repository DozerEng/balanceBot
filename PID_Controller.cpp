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
PID_Controller::PID_Controller (const double propGain, const double integralGain, const double derivativeGain) : 
        kc(propGain),
        ti(integralGain),
        td(derivativeGain) { };

// Start PID - Initializes starts the timer
void PID_Controller::start() {
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
double PID_Controller::controlStep (const double plantOutput, double setpoint) {
    deltaT = timer.read();
    
    double currentError = (setpoint - plantOutput);
    //!< Numerical approximation for integradl:
    double currentQ = 0.9 * previousQ + 0.1 * currentError;
    

    //!< Calculate deltaT

    //!< Calculate new input
    double plantInput = setpoint + kc * (currentError + ( deltaT / ti) * currentQ - ( td / deltaT ) * ( plantOutput - previousOutput ));

    //!< Store required values for next function call
    previousQ = currentQ; 
    previousOutput = plantOutput;
    timer.reset();
    return plantInput;
}