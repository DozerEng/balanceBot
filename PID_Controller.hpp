/*!
 * \file PID_Controller.hpp
 * 
 * \author Michael Pillon
 *
 *  PID Controller using a linear approximation for the integeral
 *  
 */


#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


class PID_Controller {
private:
    double deltaT = 0.0;
    double kc;  // The proportional gain
    double ti;    // Integral Term
    double td;    // Derivative Term
    double previousOutput = 0.0;  //previous output from plant
    double previousQ = 0.0; //Previous q value for integral sum calculation


public:
    /*!
     * Constructor just copies the gain.
     *
     * \param propGain The proportional gain
     * \param integralGain The integral gain
     * \param derivativeGain The derivative gain
     */
    PID_Controller (const double propGain, const double integralGain, const double derivativeGain) 
        : kc(propGain)
        , ti(integralGain)
        , td(derivativeGain) {   };
    
    // Our promise to implement the function from the abstract base class
    double controlStep (double plantOutput, double setpoint);
};


#endif

