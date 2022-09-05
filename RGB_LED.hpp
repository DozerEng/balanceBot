/*!
 * \class A4988
 * \file A4988.hpp
 * \brief Class for A4988 servo motor control board
 * 
 * \author Michael Pillon
 * 
 */

#include "mbed.h"

#ifndef RGB_LED_hpp
#define RGB_LED_hpp

/*!
    RGB_LED 

    For control of RGB LEDs
 */

class RGB_LED {
public:
/*!
    Common cathode = 0
        LED_ON = 1, LED_OFF = 0
    Common anode = 1
        LED_ON = 0, LED_OFF = 1
*/
enum Type { COMMON_CATHODE = 0, COMMON_ANODE };
const Type type; 
const bool LED_ON;
const bool LED_OFF;

/*! 
    Magenta = red + blue
    Yellow = red + green
    Cyan = green + blue
    White = red + green + blue
*/
enum State { OFF, RED, GREEN, BLUE, MAGENTA, YELLOW, CYAN, WHITE };

private: 
   DigitalOut redLED;
   DigitalOut greenLED;
   DigitalOut blueLED;

   State currentState;

public: 
    /*!
        Constructor
        \param red Pin for red segment
        \param green Pin for green segment
        \param blue PIN for pin segment
    */
    RGB_LED(Type type, PinName redPin, PinName greenPin, PinName bluePin);
    
    void setRed(void);
    void setGreen(void);
    void setBlue(void);
    void setMagenta(void);
    void setYellow(void);
    void setCyan(void); 
    void setWhite(void);
    void setOff(void);

    State getCurrentSate(void);

};

#endif 
