#include "ICM42688.h"

ICM42688::ICM42688() = default;

bool ICM42688::init(int i2cBus) {
    printf("[ICM42688] Starting sensor...\n");

    this->fd = i2cBus;

    uint8_t dev_id = 0x00;

    return true;
}

int16_t ICM42688::busWrite(uint8_t reg, uint8_t val) {
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

int16_t ICM42688::busRead(uint8_t reg, uint8_t *val, int8_t len) {
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