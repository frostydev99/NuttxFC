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

#include "../../FlightLib/SensorData.h"
#include "../../FlightLib/FlightConfig.h"

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
} accelData;

typedef struct {
    float gyroX;
    float gyroY;
    float gyroZ;
} gyroData;

class ICM42688 {
public:
    ICM42688();

    bool init(int i2cBus);

    float getAccel(accelData &accelOut);
private:

    int16_t busWrite(uint8_t reg, uint8_t val);

    int16_t busRead(uint8_t reg, uint8_t *val, int8_t len);

    float getAccelX();
    float getAccelY();
    float getAccelZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();

    float _accelX = 0.0;
    float _accelY = 0.0;
    float _accelZ = 0.0;
    float _gyroX = 0.0;
    float _gyroY = 0.0;
    float _gyroZ = 0.0;

    int fd = 0;

    constexpr static uint8_t ACCEL_ODR_VAL = 0x06; //1kHz
    constexpr static uint8_t ACCEL_FSR_VAL = 0x00; // +-16g

    constexpr static uint8_t GYRO_ODR_VAL = 0x06; // 1kHz
    constexpr static uint8_t GYRO_FSR_VAL = 0x02; // 500dps
};