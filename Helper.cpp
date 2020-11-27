/*!
 * \file I2CHelper.cpp
 *
 * \author Michael Pillon
 * 
 */

#include "Helper.hpp"



namespace Helper {

    
/*!
 * \fn void knightRider(void)
 * \sa ledRoutine
 * 
 * Knight Rider style LED routine.
 * LEDs illiminate back and forth from left to right
 *
 */
void knightRider(DigitalOut* led1, DigitalOut* led2, DigitalOut* led3, DigitalOut* led4)
{
    static int position = 0; //! position + 1 = next active LED
    static bool direction = 0; //!left to right (0) or right to left (1)
    
    //LED Position logic
    if (direction == 0) { //! Forward Direction
        switch (position) {
            case 0:
                *led1 = 1;
                *led2 = 0;
                *led3 = 0;
                *led4 = 0;
                position++;
                break;
            case 1:
                *led1 = 0;
                *led2 = 1;
                *led3 = 0;
                *led4 = 0;
                position++;
                break;
            case 2:
                *led1 = 0;
                *led2 = 0;
                *led3 = 1;
                *led4 = 0;
                position++;
                break;
            case 3:
                *led1 = 0;
                *led2 = 0;
                *led3 = 0;
                *led4 = 1;
                direction = 1; //reverse direction
                position--;
                break;
            default:
                position = 0; //Should NEVER get here
        }
    } else { //!Reverse direction == 1
        switch (position) {
            case 0:
                *led1 = 1;
                *led2 = 0;
                *led3 = 0;
                *led4 = 0;
                direction = 0;
                position++;
                break;
            case 1:
                *led1 = 0;
                *led2 = 1;
                *led3 = 0;
                *led4 = 0;
                position--;
                break;
            case 2:
                *led1 = 0;
                *led2 = 0;
                *led3 = 1;
                *led4 = 0;
                position--;
                break;
            case 3:
                *led1 = 0;
                *led2 = 0;
                *led3 = 0;
                *led4 = 1;
                position--; //reverse direction
                break;
            default:
                position = 1; //Should NEVER get here
        }
    }
}

}
