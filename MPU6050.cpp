/*!
 * \class MPU6050
 * \file MPU6050.cpp
 *
 * \author Michael Pillon
 * 
 */

#include "MPU6050.hpp"

// Implementation of controlStep()
MPU6050::MPU6050 (I2C* i2c) 
    : i2c(i2c) {
    
    //Disable sleep mode
    //Set I2C frequency in I2C_MST_CTRL
    //Set GYRO_CONFIG register page 14
    //Set ACCEL_CONFIG register page 14
};