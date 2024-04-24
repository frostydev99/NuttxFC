#include "LPS25.h"

#include "nuttx/i2c/i2c_master.h"

LPS25::LPS25() = default;

bool LPS25::init() {

    printf("[LPS25] Starting Sensor...\n");

    int16_t ex = -1;
    uint8_t dev_id;

    read(LPS25_WHOAMI, &dev_id, 1); // read whois

    printf("[LPS25] Address: %04x\n", dev_id);

    return true;
}

int16_t LPS25::write(uint8_t reg, uint8_t val) {
    int16_t ex;
    struct i2c_msg_s i2c_msg;
    struct i2c_transfer_s i2c_transfer;
    uint8_t txbuffer[2];

    txbuffer[0] = reg;
    txbuffer[1] = val;

    i2c_msg.addr = LPS25_ADDR;
    i2c_msg.flags = 0;
    i2c_msg.buffer = txbuffer;
    i2c_msg.length = 2;
    i2c_msg.frequency = 400000;

    i2c_transfer.msgv = &i2c_msg;
    i2c_transfer.msgc = 1;

    ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t)&i2c_transfer);

    if(ex < 0) {
        printf("[LPS25] I2C Write Error: %d\n", ex);
    }

    return ex;

}

int16_t LPS25::read(uint8_t reg, uint8_t *val, int16_t len) {

    int16_t ex;
    struct i2c_msg_s i2c_msg[2];
    struct i2c_transfer_s i2c_transfer;

    i2c_msg[0].addr = LPS25_ADDR;
    i2c_msg[0].flags = 0;
    i2c_msg[0].buffer = &reg;
    i2c_msg[0].length = 1;
    i2c_msg[0].frequency = 400000;

    i2c_msg[1].addr = LPS25_ADDR;
    i2c_msg[1].flags = I2C_M_READ;
    i2c_msg[1].buffer = val;
    i2c_msg[1].length = len;
    i2c_msg[1].frequency = 400000;

    i2c_transfer.msgv = i2c_msg;
    i2c_transfer.msgc = 2;

    ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t) &i2c_transfer);

    if(ex < 0) {
        printf("[LPS25] Read Error: %d\n", ex);
    }

    return ex;

}