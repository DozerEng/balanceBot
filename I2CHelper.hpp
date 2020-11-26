/*!
 * \file I2CHelper.hpp
 * \brief Class for running I2C helper functions
 * \author Michael Pillon
 *
 * 
 * 
 */

#include "mbed.h"

#ifndef I2C_HELPER_H
#define I2C_HELPER_H


class I2CHelper {
    private:
        I2C* i2c;
    public:
        I2CHelper(I2C* i2c);
};



#endif
