/*!
 * \class RGB_LED
 * \file RGB_LED.cpp
 * 
 * \author Michael Pillon
 * 
 */

#include "mbed.h"
#include "RGB_LED.hpp"


    /*!
        Constructor
    */
    RGB_LED::RGB_LED(Type type, PinName redPin, PinName greenPin, PinName bluePin) : 
        type(type),
        LED_ON(!type),
        LED_OFF(type),
        redLED(redPin),
        greenLED(greenPin),
        blueLED(bluePin)
        {
            setOff();
         };

    /*!
        Set RGB LED to the colour red
    */
    void RGB_LED::setRed() {
        currentState = RED;
        redLED = LED_ON;
        greenLED = LED_OFF;
        blueLED = LED_OFF;
    }
    /*!
        Set RGB LED to the colour green
    */
    void RGB_LED::setGreen() {
        currentState = GREEN;
        redLED = LED_OFF;
        greenLED = LED_ON;
        blueLED = LED_OFF;
    }
    /*!
        Set RGB LED to the colour Blue
    */
    void RGB_LED::setBlue() {
        currentState = BLUE;
        redLED = LED_OFF;
        greenLED = LED_OFF;
        blueLED = LED_ON;
    }
    /*!
        Set RGB LED to the colour magenta
    */
    void RGB_LED::setMagenta() {
        currentState = MAGENTA;
        redLED = LED_ON;
        greenLED = LED_OFF;
        blueLED = LED_ON;
    }
    /*!
        Set RGB LED to the colour yellow
    */
    void RGB_LED::setYellow() {
        currentState = YELLOW;
        redLED = LED_ON;
        greenLED = LED_ON;
        blueLED = LED_OFF;
    }
    /*!
        Set RGB LED to the colour cyan
    */
    void RGB_LED::setCyan() {
        currentState = CYAN;
        redLED = LED_OFF;
        greenLED = LED_ON;
        blueLED = LED_ON;
    }
    /*!
        Set RGB LED to the colour white
    */
    void RGB_LED::setWhite() {
        currentState = WHITE;
        redLED = LED_ON;
        greenLED = LED_ON;
        blueLED = LED_ON;
    }
    /*!
        Set GRB LED to off
    */
    void RGB_LED::setOff() {
        currentState = OFF;
        redLED = LED_OFF;
        greenLED = LED_OFF;
        blueLED = LED_OFF;
    }
    /*!
        Get the current LED colour
    */
    RGB_LED::State RGB_LED::getCurrentSate() {
        return currentState;
    }
