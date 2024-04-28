#include "MMC5983MA.h"

MMC5983MA::MMC5983MA() = default;

bool MMC5983MA::init(int i2cBus) {
    this->fd = i2cBus;

    uint8_t dev_id = 0x00;

    busRead(PROD_ID_REG, &dev_id, 1);

    if(dev_id == PROD_ID) {
        printf("[MMC5983] Successfully Initialized!\n");
    } else {
        printf("[MMC5983] Failed to initialize...\n");
    }

    return true;
}

int16_t MMC5983MA::busWrite(uint8_t reg, uint8_t val) {
    int16_t ex;

    uint8_t txbuffer[2];

    txbuffer[0] = reg;
    txbuffer[1] = val;

    struct i2c_msg_s i2c_msg {
            .frequency = 400000,
            .addr = I2C_ADDR,
            .flags = 0,
            .buffer = txbuffer,
            .length = 2,
    };

    struct i2c_transfer_s i2c_transfer {
            .msgv = &i2c_msg,
            .msgc = 1
    };

    ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t)&i2c_transfer);

    if(ex < 0) {
        printf("[LPS25] I2C Write Error: %d\n", ex);
    }

    return ex;

}

int16_t MMC5983MA::busRead(uint8_t reg, uint8_t *val, int8_t len) {

    int16_t ex;

    struct i2c_msg_s i2c_msg[2] = {
            {
                    .frequency = 400000,
                    .addr = I2C_ADDR,
                    .flags = 0,
                    .buffer = &reg,
                    .length = 1
            },
            {
                    .frequency = 400000,
                    .addr = I2C_ADDR,
                    .flags = I2C_M_READ,
                    .buffer = val,
                    .length = len
            }
    };

    struct i2c_transfer_s i2c_transfer {
            .msgv = i2c_msg,
            .msgc = 2
    };

    ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t) &i2c_transfer);

    if(ex < 0) {
        printf("[LPS25] Read Error: %d\n", ex);
    }

    return ex;

}

bool MMC5983MA::getRawMagXYZ(uint32_t *x, uint32_t *y, uint32_t *z) {

    return true;
}

bool MMC5983MA::getMagnetometerXYZ(float *x, float *y, float *z) {

    return true;
}

bool MMC5983MA::setMagBandwidth(MagBandwidth bw) {

    bool success = false;

    uint8_t ctrl1 = 0x00;

    // Read control register
    busRead(INT_CTRL_1_REG, &ctrl1, 1);

    // Bit shift mag bandwidth into configuration
    ctrl1 |= bw << 2;

    success = busWrite(INT_CTRL_1_REG, ctrl1);

    return success;
}