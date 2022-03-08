/*!
    \file Helper.hpp
    \brief Assorted helper functions
    \author Michael Pillon


 */

#include "mbed.h"

#ifndef HELPER_H
#define HELPER_H

#define LSB_MASK    0x01    //!< See printBinary()

/*!
    Helper Namespace

 */
namespace Helper {



void knightRider(DigitalOut* led1, DigitalOut* led2, DigitalOut* led3, DigitalOut* led4);



/*!
    \fn printBinary()
    Template function to print data in binary

    \param T pointer for data to be printed in binary
    \param numBytes number of elements to print
 */
template <typename T>
void printBinary(const T* data, const uint16_t numBytes = 1) {
     //!< Get number of bits in 1 element
    uint8_t elementSize = sizeof(data[0]) * 8;
    //!< Print each data point starting with 0b
    for(int i = 0; i < numBytes; i++) {
        printf("\t0b");
        //!< Print each element bit by bit
        for (int j = 0; j < elementSize; j++) {
            if( (j != 0) && (j % 4 == 0)) {
                printf(" ");
            }
            char temp = ((data[i]) >> (elementSize - 1 -j)) & LSB_MASK;
            printf("%i", temp);
        }
        printf("\n\r");
    }
    printf("\n\r");
}

}//EO namespace Helper 


#endif
