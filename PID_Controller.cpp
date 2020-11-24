/*!
 * \class PID_Controller
 * \file PID_Controller.cpp
 * \brief Class for a PID controller
 * 
 * \author Michael Pillon
 * 
 */

#include "PID_Controller.hpp"

// Implementation of controlStep()
double PID_Controller::controlStep (const double plantOutput, double setpoint) {
    double currentError = (setpoint - plantOutput);
    double currentQ = 0.9 * previousQ + 0.1 * currentError;

    //Calculate new input
    double plantInput = setpoint + kc * (currentError + ( deltaT / ti) * currentQ - ( td / deltaT ) * ( plantOutput - previousOutput ));

    //Store required values for next function call.  
    previousQ = currentQ; //Could also be static function variables, not sure if that would have been better. Would love feedback on this.
    previousOutput = plantOutput;
    return plantInput;
}