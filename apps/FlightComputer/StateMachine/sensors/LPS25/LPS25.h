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


class LPS25 {


public:

    LPS25();

    /**
     * Initialization Function
     * @param i2cBus Return from opened i2c bus
     * fd = open(i2cPath, O_WRONLY)
     * @return T/F if sensor initialized
     */
    bool init(int i2cBus);

    /**
     * Read Pressure
     * @return Pressure in hPa (mbar)
     */
    float getPressure();

    int16_t getTemperature();

private:
    int16_t busWrite(uint8_t reg, uint8_t  val);

    int16_t busRead(uint8_t reg, uint8_t* val, int8_t len);

    constexpr static uint8_t LPS25_ADDR = 0x5c;
    constexpr static uint8_t LPS25_WHOAMI = 0x0f;
    constexpr static uint8_t LPS25_WHO_ID = 0xBD;

    constexpr static uint8_t LPS25_CTRL1 = 0x20;
    constexpr static uint8_t LPS25_CTRL2 = 0x21;
    constexpr static uint8_t LPS25_CTRL3 = 0x22;
    constexpr static uint8_t LPS25_CTRL4 = 0x23;

    constexpr static uint8_t LPS25_PWRUP = 0x80;

    // ODR
    constexpr static uint8_t ODR_1 = 0x01;
    constexpr static uint8_t ODR_7 = 0x02;
    constexpr static uint8_t ODR_12_5 = 0x03;
    constexpr static uint8_t ODR_25 = 0x4;


    constexpr static uint8_t LPS25_STATUS_REG = 0x27;
    constexpr static uint8_t LPS25_TEMP_READY = 0x1;
    constexpr static uint8_t LPS25_PRESSURE_READY = 0x2;

    constexpr static uint8_t LPS25_PRESSURE_OUT_XL = 0x28;
    constexpr static uint8_t LPS25_PRESSURE_L_REG = 0x29;
    constexpr static uint8_t LPS25_PRESSURE_H_REG = 0x2A;

    constexpr static uint8_t LPS25_TEMP_L_REG = 0x2B;
    constexpr static uint8_t LPS25_TEMP_H_REG = 0x2C;

    float _pressure = 0.0;
    int16_t _temperature = 0.0;

    int fd = 0;
};