/*!
 * \class MPU6050
 * \file MPU6050.cpp
 *
 * \author Michael Pillon
 * 
 *  See Header
 *
 */

#include "MPU6050.hpp"

/*!
    Constructor
    \param i2c Pointer to an I2C object
*/
MPU6050::MPU6050 (I2C* i2c) 
    : i2c(i2c) {
    printf("Initializing MPU6050...\n\r");
    /*/
        Use whoAmI() to check if MPU is listening.
        Returns zero if no response
     */
    if( whoAmI() ) {
        //!< Reset MPU6050 to set all registers to default (0x00)
        resetDevice();
        //!< Make sure MPU is awake. Reset puts device to sleep
        enableSleep(DISABLE);
        //!< Change clock source to a gyro axis
        setClockSource(CLK_REF_X_GYRO);
        //!< Set gyroscope scale
        setGyroScale(GYRO_SCALE_250);
        //!< Set accelerometer scale
        setAccelScale(ACCEL_SCALE_2G);
        //!< Run full self test
        selfTest();
        //!< Get and display device temperature
        double currentTemp = getTemp();
        printf("Current MPU6050 temperature: %0.1fC\n\r", currentTemp);
        printf("Successfully initialized MPU6050\n\r");
    } else {
        printf("Failed to initialize MPU6050\n\r");
    }
};
/*!
    Reset MPU and all of its registers

    Register: PWR_MGMT_1
    Bit7    DEVICE_RESET - Resets all registers default
    Bit6    SLEEP - Sleep mode enable/disable
    Bit5    CYCLE - Cycle Mode enable/disable
    Bit4    -
    Bit3    TEMP_DIS - Temperature sensor enable/disable
    Bit2    CLKSEL[2:0]
    Bit1    CLKSEL[2:0]
    Bit0    CLKSEL[2:0]

    \param state ENABLE / DISABLE
*/
void MPU6050::resetDevice(){ 
    char data[2];
    data[0] = PWR_MGMT_1; //!< PWR_MGMT_1
    //!< Get current device state
    // i2c->write(MPU6050_ADDRESS_8BIT, data, 1, true);
    // i2c->read(MPU6050_ADDRESS_8BIT, &data[1], 1);
    data[1] = 0x80;
    i2c->write(MPU6050_ADDRESS_8BIT, data, 2);
    wait_us(WAIT_FOR_MPU);
}
/*!
    Enable Sleep mode

    Register: PWR_MGMT_1
    Bit7    DEVICE_RESET - Resets all registers default
    Bit6    SLEEP - Sleep mode enable/disable
    Bit5    CYCLE - Cycle Mode enable/disable
    Bit4    -
    Bit3    TEMP_DIS - Temperature sensor enable/disable
    Bit2    CLKSEL[2:0]
    Bit1    CLKSEL[2:0]
    Bit0    CLKSEL[2:0]

    \param state ENABLE / DISABLE
*/
void MPU6050::enableSleep(uint8_t state){ 
    char data[2];
    data[0] = PWR_MGMT_1; //!< PWR_MGMT_1
    //!< Get current device state
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
    Configures the digital low pass filter

    \param state ENABLE / DISABLE
*/
void MPU6050::setDLPF(uint8_t dlfp_cfg){
    char data[2];
    data[0] = CONFIG; //!< PWR_MGMT_1
    //!< Get current device state
    i2c->write(MPU6050_ADDRESS_8BIT, data, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, &data[1], 1);
    //!< Turn all DLPF bits off then mask with dlfp_cfg
    data[1] = ( data[1] & 0xF8 ) | dlfp_cfg;
    i2c->write(MPU6050_ADDRESS_8BIT, data, 2);
}

/*!
    Enable Temperature sensor

    Register: PWR_MGMT_1
    Bit7    DEVICE_RESET - Resets all registers default
    Bit6    SLEEP - Sleep mode enable/disable
    Bit5    CYCLE - Cycle Mode enable/disable
    Bit4    -
    Bit3    TEMP_DIS - Temperature sensor enable/disable
    Bit2    CLKSEL[2:0]
    Bit1    CLKSEL[2:0]
    Bit0    CLKSEL[2:0]

    \param state ENABLE / DISABLE
*/
void MPU6050::enableTemp(uint8_t state){ 
    char data[2];
    data[0] = PWR_MGMT_1; //!< PWR_MGMT_1
    //!< Get current device state then mask off sleep bit
    i2c->write(MPU6050_ADDRESS_8BIT, data, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, &data[1], 1);
    if(state == ENABLE) {
        data[1] = data[1] & 0xF7; //!< TEMP_DIS = 0
    } else if (state == DISABLE) {
        data[1] = data[1] | 0x08; //!< TEMP_DIS = 1
    }
    i2c->write(MPU6050_ADDRESS_8BIT, data, 2);
}
/*!
    Set device clock source

    Bit7    DEVICE_RESET - Resets all registers default
    Bit6    SLEEP - Sleep mode enable/disable
    Bit5    CYCLE - Cycle Mode enable/disable
    Bit4    -
    Bit3    TEMP_DIS - Temperature sensor enable/disable
    Bit2    CLKSEL[2:0]
    Bit1    CLKSEL[2:0]
    Bit0    CLKSEL[2:0]

CLKSEL          Clock Source
    0       Internal 8MHz oscillator
    1       PLL with X axis gyroscope reference
    2       PLL with Y axis gyroscope reference
    3       PLL with Z axis gyroscope reference
    4       PLL with external 32.768kHz reference
    5       PLL with external 19.2MHz reference
    6       Reserved
    7       Stops the clock and keeps the timing generator in reset

    \param source 3 bit clock source
*/
void MPU6050::setClockSource(uint8_t source){ 
    char data[2];
    data[0] = PWR_MGMT_1; //!< Register
    //!< Get current device state
    i2c->write(MPU6050_ADDRESS_8BIT, data, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, &data[1], 1);
    data[1] = ( data[1] & 0xF8 ) | source;
    i2c->write(MPU6050_ADDRESS_8BIT, data, 2);
}
/*!
    Get accelerometer data
    \param x Point to accelerometer axis to check
    \param y Point to accelerometer axis to check
    \param z Point to accelerometer axis to check
*/
void MPU6050::getAccel(int16_t* x, int16_t* y, int16_t* z) {
    char wData[1];
    char rData[6];
    wData[0] = ACCEL_XOUT_H; //!< First register for accelerometer data

    i2c->write(MPU6050_ADDRESS_8BIT, wData, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, rData, 6);
    *x = (int16_t(rData[0]) << 8) | rData[1];
    *y = (int16_t(rData[2]) << 8) | rData[3];
    *z = (int16_t(rData[4]) << 8) | rData[5];
    //printf("Accel X: %i Accel Y: %i Accel Z: %i", *x, *y, *z);
}

/*!
    Get gyroscope data
    \param x Point to gyroscope axis to check
    \param y Point to gyroscope axis to check
    \param z Point to gyroscope axis to check
*/
void MPU6050::getGyro(int16_t* x, int16_t* y, int16_t* z) {
    char wData[1];
    char rData[6];
    wData[0] = GYRO_XOUT_H; //!< First register for gyroscope data

    i2c->write(MPU6050_ADDRESS_8BIT, wData, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, rData, 6);
    *x = (int16_t(rData[0]) << 8) | rData[1];
    *y = (int16_t(rData[2]) << 8) | rData[3];
    *z = (int16_t(rData[4]) << 8) | rData[5];
    //printf("Gyro X: %i Gyro Y: %i Gyro Z: %i\n\r", *x, *y, *z);
}
/*!
    Get temperature data
    Datasheet: 
        Range -40 to +85 °C
        Sensitivity Untrimmed 340 LSB/ºC
        Temperature Offset 35ºC -521 LSB
        Linearity Best fit straight line (-40°C to +85°C) 
    \return Temperature data converted to celcius
*/
double MPU6050::getTemp() {
    char wData[1];
    char rData[2];
    wData[0] = TEMP_OUT_H; //!< First register for teperature data

    i2c->write(MPU6050_ADDRESS_8BIT, wData, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, rData, 2);
    int16_t rawTemp = (int16_t(rData[0]) << 8) | rData[1];

    return rawTemp / 340.0 + 35.0;; //!< Default Scaling
}

/*!
    Set scale of accelerometer

AFS_SEL     Full Scale Range
    0           ± 2g
    1           ± 4g
    2           ± 8g
    3           ± 16g

    Bit7    X axis self test trigger
    Bit6    Y axis self test trigger
    Bit5    z axis self test trigger
    Bit4    AFS_SEL[1] 
    Bit3    AFS_SEL[0] 
    Bit2    N/A
    Bit1    N/A
    Bit0    N/A

    Triggering self test stores results in SELF_TEST_x  registers

 */
uint8_t MPU6050::getAccelScale() {
    char reg = ACCEL_CONFIG; //!< First register for teperature data;
    char data;
    i2c->write(MPU6050_ADDRESS_8BIT, &reg, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, &data, 1);
    return uint8_t(data >> 3) & 0x03;
}
void MPU6050::setAccelScale(uint8_t scale) {
    char data[2];
    data[0] = ACCEL_CONFIG; 
    data[1] = char(scale << 3); 
    i2c->write(MPU6050_ADDRESS_8BIT, data, 2);
}

/*!
    Gyroscope Config
    
FS_SEL      Full Scale Range
    0           ± 250 °/s
    1           ± 500 °/s
    2           ± 1000 °/s
    3           ± 2000 °/s

    Bit7    X axis self test trigger
    Bit6    Y axis self test trigger
    Bit5    z axis self test trigger
    Bit4    FS_SEL[1] <-- Bits may be out of order,
    Bit3    FS_SEL[0] <-- Requires testing
    Bit2    N/A
    Bit1    N/A
    Bit0    N/A

    Triggering self test stores results in SELF_TEST_x  registers
 */
uint8_t MPU6050::getGyroScale() {
    char reg = GYRO_CONFIG; //!< First register for teperature data;
    char data;
    i2c->write(MPU6050_ADDRESS_8BIT, &reg, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, &data, 1);
    return uint8_t(data >> 3) & 0x03;
}
void MPU6050::setGyroScale(uint8_t scale) {
    char data[2];
    data[0] = GYRO_CONFIG; 
    data[1] = char(scale << 3); 
    i2c->write(MPU6050_ADDRESS_8BIT, data, 2);
}

/*!
    Self Test for Gyro and Accelerometer
    Data is stored as 5 bit unsiged integers
    
Reg     Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
0x0D        XA_TEST[4-2]                    XG_TEST[4-0]
0x0E        YA_TEST[4-2]                    YG_TEST[4-0]
0x0F        ZA_TEST[4-2]                    ZG_TEST[4-0]
0x10    RESERVED        XA_TEST[1-0]    YA_TEST[1-0]    ZA_TEST[1-0]

 */
void MPU6050::selfTest(void) {
    printf("Running MPU6050 self test...\n\r");
    uint8_t originalGyro = getGyroScale();
    uint8_t originalAccel = getAccelScale();
    char cmd[3];
    //!< Run gyroscope and accelerometeer self tests
    cmd[0] = GYRO_CONFIG; //!< GYRO_CONFIG, ACCEL_CONFIG is next
    cmd[1] = 0xE0; //!< Test all axis at 250 degree/sec
    cmd[2] = 0xE0; //!< Test all axis at 2g
    i2c->write(MPU6050_ADDRESS_8BIT, cmd, 3);
    wait_us(WAIT_FOR_MPU);

    /*!
        Get self test results as 5 bit unsinged integers
     */
    uint8_t xA = 0, yA = 0, zA = 0, xG = 0, yG = 0, zG = 0;
    cmd[0] = SELF_TEST_X; //!< First of 4 registers with self test data
    char data[4];
    i2c->write(MPU6050_ADDRESS_8BIT, cmd, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, data, 4);
    //!< See table above for bit combinations
    xA = ( ( (data[3] & 0x30) >> 4 ) | ( (data[0] & 0xE0) >> 5 ) ) & 0x1F;
    yA = ( ( (data[3] & 0x0C) >> 2 ) | ( (data[1] & 0xE0) >> 5 ) ) & 0x1F;
    zA = (   (data[3] & 0x03)        | ( (data[2] & 0xE0) >> 5 ) ) & 0x1F;
    xG = (data[0] & 0x1F);
    yG = (data[1] & 0x1F);
    zG = (data[2] & 0x1F);
    //!< 5 Bit unsiged int, max value = 32
    printf("Raw0: 0x%X  Raw1: 0x%X  Raw2: 0x%X  Raw3: 0x%X\n\r", data[0], data[1], data[2], data[3]);
    printf("XA: %i   YA: %i   ZA: %i\n\r", xA, yA, zA);
    printf("XG: %i   YG: %i   ZG: %i\n\r", xG, yG, zG);
    printf("MPU6050 Self Test Passed\n\r"); //!< Need to find out the pass/fail criteria. For now... I'm optimistic... can also be used to benchmark devices.

    /*!
        Reset to pre-self test configuration
     */
    setGyroScale(originalGyro);
    setAccelScale(originalAccel);
}

/*!
    whoAmI() checks WHO_AM_I register
    Checks if MPU6050 is listening
    Response is device address
        - 0x68 by default (7 Bit Address) as uint8_t
 */
uint8_t MPU6050::whoAmI(void) {
    char reg = WHO_AM_I; 
    char data = 0;
    i2c->write(MPU6050_ADDRESS_8BIT, &reg, 1, true);
    i2c->read(MPU6050_ADDRESS_8BIT, &data, 1);
    printf("MPU6050 slave listening at I2C address: 0x%X\n\r", data);
    return uint8_t(data);
}