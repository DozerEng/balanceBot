/*!
 * \class MPU6050
 * \file MPU6050.cpp
 *
 * \author Michael Pillon
 * 
 */

#include "MPU6050.hpp"

/*!
    Constructor
    \param i2c Pointer to an I2C object
*/
MPU6050::MPU6050 (I2C* i2c) 
    : i2c(i2c) {
    //!< Make sure MPU is awake
    this->enableSleep(DISABLE);
    //Change clock source to a gyro axis
    //Set I2C frequency in I2C_MST_CTRL
    //Set GYRO_CONFIG register page 14
    //Set ACCEL_CONFIG register page 14
    //Run self tests on all 6 DOF
};
    /*!
        Enable Sleep mode
        \param state ENABLE / DISABLE
    */
void MPU6050::enableSleep(uint8_t state){ 
    char data[2];
    data[0] = PWR_MGMT_1; //!< PWR_MGMT_1
    //!< Get current device state then mask off sleep bit
    i2c->write(MPU6050_ADDRESS_8BIT, data, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, &data[1], 1);
    if(state == DISABLE) {
        data[1] = data[1] & 0xBF; //!< Sleep mode bit = 0
    } else if (state == ENABLE) {
        data[1] = data[1] | 0x40; //!< Sleep mode bit = 1
    }
    i2c->write(MPU6050_ADDRESS_8BIT, data, 2);
}

/*!
    Check axis value
    \param axis Gyro / Accel axis to check
*/
void MPU6050::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    char wData[1];
    char rData[6];
    wData[0] = ACCEL_XOUT_H; //!< First register for accelerometer data

    i2c->write(MPU6050_ADDRESS_8BIT, wData, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, rData, 6);
    *x = (int16_t(rData[0]) << 8) | rData[1];
    *y = (int16_t(rData[2]) << 8) | rData[3];
    *z = (int16_t(rData[4]) << 8) | rData[5];
}

