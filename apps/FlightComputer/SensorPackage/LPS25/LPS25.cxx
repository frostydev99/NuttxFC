#include "LPS25.h"

LPS25::LPS25() = default;

bool LPS25::init(int i2cBus) {
    printf("[LPS25] Starting sensor...\n");

    this->fd = i2cBus;

    uint8_t dev_id = 0x00;

    busRead(LPS25_WHOAMI, &dev_id, 1);

    if(dev_id == LPS25_WHO_ID) {
        printf("[LPS25] Succesfully Initialized!\n");
        printf("[LPS25] %d", dev_id);
    } else {
        printf("[LPS25] Error: Failed to initialize...\n");

        return false;
    }

    uint8_t powerupData;

    busRead(LPS25_CTRL1, &powerupData, 1);

    busWrite(LPS25_CTRL1, 0xC0); // Power on 25Hz output

    return true;
}

int16_t LPS25::busWrite(uint8_t reg, uint8_t val) {
    int16_t ex;

    uint8_t txBuffer[2];

    txBuffer[0] = reg;
    txBuffer[1] = val;

    struct i2c_msg_s i2cMsg {
        .frequency = 400000,
        .addr = LPS25_ADDR,
        .flags = 0,
        .buffer = txBuffer,
        .length = 2
    };

    struct i2c_transfer_s i2cTransfer {
        .msgv = &i2cMsg,
        .msgc = 1
    };

    ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t) &i2cTransfer);

    if(ex < 0) {
        printf("[LPS25] Error: Failed to set register -> %d\n", ex);
    }

    return ex;
}

int16_t LPS25::busRead(uint8_t reg, uint8_t *val, int8_t len) {
    int16_t ex;

    struct i2c_msg_s i2cMsg[2] = {
            {
                .frequency = 400000,
                .addr = LPS25_ADDR,
                .flags = 0,
                .buffer = &reg,
                .length = 1
            },
            {
                .frequency = 400000,
                .addr = LPS25_ADDR,
                .flags = I2C_M_READ,
                .buffer = val,
                .length = len
            }
    };

    struct i2c_transfer_s i2cTransfer {
        .msgv = i2cMsg,
        .msgc = 2
    };

    ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t) &i2cTransfer);

    if(ex < 0) {
        printf("[LPS25] Read Error: %d\n", ex);
    }

    return ex;
}

float LPS25::getPressure() {

    uint8_t dataRead = 0x00;

    int32_t pData = 0;

    // Check status register to see if pressure data is available
    busRead(LPS25_PRESSURE_H_REG, &dataRead, 1);
    pData = dataRead << 16; // MSB

    busRead(LPS25_PRESSURE_L_REG, &dataRead, 1);
    pData |= dataRead << 8; //LSB

    busRead(LPS25_PRESSURE_OUT_XL, &dataRead, 1);
    pData |= dataRead;

    _pressure = (float) pData / 4096.0f;

    return _pressure;
}

uint16_t LPS25::getTemperature() {

    uint8_t dataRead = 0x00;

    uint16_t tData = 0;

    busRead(LPS25_TEMP_H_REG, &dataRead, 1);

    tData = dataRead << 16; // MSB

    busRead(LPS25_TEMP_L_REG, &dataRead, 1);

    tData |= dataRead << 8; // LSB

    _temperature = tData / 480;

    return _temperature;
}