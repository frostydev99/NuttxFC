#pragma once

#include <nuttx/config.h>

#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <sched.h>
#include <cerrno>
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <cstring>


enum MagBandwidth {
    bw100 = 0x00, // 8ms measurement time
    bw200 = 0x01, // 4ms measurement time
    bw400 = 0x02, // 2ms measurement time
    bw800 = 0x03, // 0.5ms measurement time
};

class MMC5983MA {

public:

    MMC5983MA();

    /**
     * Initialization Function
     * @param i2cBus Return from opened i2c bus
     * fd = open(i2cPath, O_WRONLY)
     * @return T/F if sensor initialized
     */
    bool init(int i2cBus);


    bool getMagnetometerXYZ(float *x, float *y, float *z);



    bool setMagBandwidth(MagBandwidth bw);

private:
    int fd = 0;

    int16_t busWrite(uint8_t reg, uint8_t  val);

    int16_t busRead(uint8_t reg, uint8_t* val, int8_t len);

    bool getRawMagXYZ(uint32_t *x, uint32_t *y, uint32_t *z);

    constexpr static uint8_t X_OUT_0_REG    = 0x00;
    constexpr static uint8_t X_OUT_1_REG    = 0X01;
    constexpr static uint8_t Y_OUT_0_REG    = 0x02;
    constexpr static uint8_t Y_OUT_1_REG    = 0x03;
    constexpr static uint8_t Z_OUT_0_REG    = 0x04;
    constexpr static uint8_t Z_OUT_1_REG    = 0x05;
    constexpr static uint8_t XYZ_OUT_2_REG  = 0x06;
    constexpr static uint8_t T_OUT_REG      = 0x07;
    constexpr static uint8_t STATUS_REG     = 0x08;
    constexpr static uint8_t INT_CTRL_0_REG = 0x09;
    constexpr static uint8_t INT_CTRL_1_REG = 0x0a;
    constexpr static uint8_t INT_CTRL_2_REG = 0x0b;
    constexpr static uint8_t INT_CTRL_3_REG = 0x0c;
    constexpr static uint8_t PROD_ID_REG    = 0x2f;
    constexpr static uint8_t DUMMY          = 0x0;

    constexpr static uint8_t SW_RST = 1 << 7;

    constexpr static uint8_t I2C_ADDR       = 0x30;
    constexpr static uint8_t PROD_ID        = 0x30;

    uint32_t _rawMagX = 0;
    uint32_t _rawMagY = 0;
    uint32_t _rawMagZ = 0;

};