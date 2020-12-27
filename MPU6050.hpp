/*!
 * \class MPU6050
 * \file MPU6050.hpp
 * \brief Class for using an MPU6050 IMU
 * \author Michael Pillon
 *
 *  The MCU6050 uses I2C communication.
 * 
 *  TODO: Set I2C frequency in I2C_MST_CTRL
 *  TODO: Configure Digital Low-Pass Filter
 *
 *  Register map and usage:
 *  https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
 * 
 *  Datasheet: 
 *  https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *
 */

#include "mbed.h"
//#include "I2CHelper.hpp"

#ifndef MPU6050_H
#define MPU6050_H

/*!
    Registers
    See register map linked above
 */
#define MPU6050_ADDRESS_7BIT     0x68 //!< 7-bit address
#define MPU6050_ADDRESS_8BIT     0xD0 //!< 8-bit address
/*!
    Self Tests for accel/gyro

Reg     Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
0x0D        XA_TEST[4-2]                    XG_TEST[4-0]
0x0E        YA_TEST[4-2]                    YG_TEST[4-0]
0x0F        ZA_TEST[4-2]                    ZG_TEST[4-0]
0x10    RESERVED        XA_TEST[1-0]    YA_TEST[1-0]    ZA_TEST[1-0]

 */
#define SELF_TEST_X         0x0D
#define SELF_TEST_Y         0x0E    
#define SELF_TEST_Z         0x0F
#define SELF_TEST_A         0x10

#define PASSED              true
#define FAILED              false
/*!
    Returns sample rate as:
    sample rate = gyroscope output rate / ( 1 + SMPLRT_DIV )
    Register returns uint8_t
 */
#define SMPLRT_DIV          0x19    
/*!
    General Config.
    Settings for both accel and gyro.
    Digital low pass filter will filter noise out at frequencies 
        FSYNC - External Frame Synchronization
        DLPF - Digital Low Pass Filter

        Bit7    N/A
        Bit6    N/A
        Bit5    EXT_SYNC_SET[2:0]
        Bit4    EXT_SYNC_SET[2:0]
        Bit3    EXT_SYNC_SET[2:0]
        Bit2    DLPF_CFG[2:0]
        Bit1    DLPF_CFG[2:0]
        Bit0    DLPF_CFG[2:0]

    See Page 13 of register map for details
 */
#define CONFIG              0x1A    //!< General Config
#define DLPF_CFG_0          0x00    //!< Accel: 260Hz - Gyro: 256Hz
#define DLPF_CFG_1          0x01    //!< Accel: 184Hz - Gyro: 188Hz
#define DLPF_CFG_2          0x02    //!< Accel: 94Hz - Gyro: 98Hz
#define DLPF_CFG_3          0x03    //!< Accel: 44Hz - Gyro: 42Hz
#define DLPF_CFG_4          0x04    //!< Accel: 21Hz - Gyro: 20Hz
#define DLPF_CFG_5          0x05    //!< Accel: 10Hz - Gyro: 10Hz
#define DLPF_CFG_6          0x06    //!< Accel: 5Hz - Gyro: 5Hz

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
#define GYRO_CONFIG         0x1B
#define GYRO_SCALE_250      0
#define GYRO_SCALE_500      1
#define GYRO_SCALE_1000     2
#define GYRO_SCALE_2000     3

/*!
    Accelerometer Config

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
#define ACCEL_CONFIG        0x1C
#define ACCEL_SCALE_2G      0
#define ACCEL_SCALE_4G      1
#define ACCEL_SCALE_8G      2
#define ACCEL_SCALE_16G     3

#define MOT_THR             0x1F //!< Threshold for motion detect interupt (Accelerometer)
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24 //!< I2C Settings - Includes bus frequency
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38

#define INT_STATUS          0x3A
/*!
    Accelerometer data registers
    Data is int16_t
 */
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
/*!
    Temperature data registers
    Data is int16_t
 */
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
/*!
    Gyroscope data registers
    Data is int16_t
 */
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
/*!
    External sensor data registers
 */
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60

#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
/*!
    Power Management 1
    - Reset value 0x40

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
 */
#define PWR_MGMT_1          0x6B
#define CLK_REF_INTERNAL    0x00
#define CLK_REF_X_GYRO      0x01
#define CLK_REF_Y_GYRO      0x02
#define CLK_REF_Z_GYRO      0x03

#define PWR_MGMT_2          0x6C
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75    //!< Reset Value: 0x68

/*!
    Constants
 */
#define WAIT_FOR_MPU 1000 //!< Wait for MPU6050. Minimum value for reset, useful elsewhere. In microseconds.

#define ENABLE      1
#define DISABLE     0


/*!
    MPU6050 class

 */

class MPU6050 {
private:
    I2C* i2c;
public:
    MPU6050 (I2C* i2c);

    void resetDevice(void);
    void configure(uint8_t config);
    void setClockSource(uint8_t source);
    void selfTest(void);
    uint8_t whoAmI(void);
    void setDLPF(uint8_t dlfp_cfg);

    void enableSleep(uint8_t state);

    void enableTemp(uint8_t state);
    double getTemp(void);

    void getAccel(int16_t* x, int16_t* y, int16_t* z);
    void getGyro(int16_t* x, int16_t* y, int16_t* z);
    
    void setAccelScale(uint8_t scale);
    uint8_t getAccelScale(void);
    void setGyroScale(uint8_t scale);
    uint8_t getGyroScale(void);
     
    

};

#endif

