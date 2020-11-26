/*!
 * \class BalanceBot
 * \file BalanceBot.hpp
 * \brief Class for a 2 wheel self balancing robot
 * \author Michael Pillon
 *
 * 
 * Features:
 *          2 Stepper motors for wheel using A4988 drivers
 *          MPU6050 IMU sensor for orienation measurements
 *          PID Controller for stabilizing system
 *          
 *          More to come...
 * 
 */

#include "mbed.h"
#include "A4988.hpp"
#include "MPU6050.hpp"
// #include "PID_Controller.hpp"

 #ifndef BALANCE_BOT_H 
 #define BALANCE_BOT_H 

/*! 
    Constants
 */
/*!
    Data Types
 */


/*!
    BalanceBot 

    2 Wheel self balancing robot
 */
class BalanceBot {
private: 
    A4988* leftWheel;
    A4988* rightWheel;
    
public:
    BalanceBot (A4988 *lw, A4988* rw);

    /*!
        Incrememnt both wheels forward
        \param count Number of steps to rotate wheels
     */
    void step(const uint8_t count = 1);
    /*!
        Set direction of both wheels
        \param dir Direction to set wheels. No arg toggles state
     */
    void setDirection(void);
    void setDirection(const uint8_t dir);
    /*!
        Set wheel motor microstepping mode
        Calls corresponding A4988 stepMode function
    */
    void setStepMode(const uint8_t mode);
    void incStepMode();
    void decStepMode();
};

#endif