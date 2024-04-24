#pragma once

#include <nuttx/config.h>

#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>


class LPS25 {


public:

    LPS25();

    bool init();
    int fd;

    float getPresure();

    float getTemperature();

private:


    int16_t write(uint8_t reg, uint8_t  val);

    int16_t read(uint8_t reg, uint8_t* val, int16_t len);

    constexpr static uint8_t LPS25_ADDR = 0x5D;
    constexpr static uint8_t LPS25_WHOAMI = 0x0F;

    constexpr static uint8_t LPS25_CTRL1 = 0x20;
    constexpr static uint8_t LPS25_CTRL2 = 0x21;
    constexpr static uint8_t LPS25_CTRL3 = 0x22;
    constexpr static uint8_t LPS25_CTRL4 = 0x23;

    constexpr static uint8_t LP25_PRESSURE_OUT = (0x28 | 0x80);

    typedef enum {
        LPS25_RATE_ONE_SHOT,
        LPS25_RATE_1_HZ,
        LPS25_RATE_7_HZ,
        LPS25_RATE_12_5_HZ,
        LPS25_RATE_25_HZ
    } lps25_rate_t;

};