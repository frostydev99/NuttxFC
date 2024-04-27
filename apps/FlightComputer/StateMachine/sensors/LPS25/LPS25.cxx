#include "LPS25.h"

#include "nuttx/i2c/i2c_master.h"


LPS25::LPS25() = default;

bool LPS25::init(int i2cBus) {

    printf("[LPS25] Starting Sensor...\n");

    this->fd = i2cBus;

//    int16_t ex = -1;
    uint8_t dev_id = 0x00;

    busRead(LPS25_WHOAMI, &dev_id, 1); // Read WHOIS register

    if(dev_id == LPS25_WHO_ID) {
        printf("[LPS25] Successfully Initialized!\n");
    } else {
        printf("[LPS25] Failed to Initialize...\n");
        return false;
    }

    uint8_t powerupData;

    busRead(LPS25_CTRL1, &powerupData, 1);

//    powerupData |= 0x80; // Power Up
//    powerupData |= ODR_25; // 25Hz Data Read

    busWrite(LPS25_CTRL1, 0xB0);

    return true;
}

int16_t LPS25::busWrite(uint8_t reg, uint8_t val) {
    int16_t ex;

    uint8_t txbuffer[2];

    txbuffer[0] = reg;
    txbuffer[1] = val;

    struct i2c_msg_s i2c_msg {
        .frequency = 400000,
        .addr = LPS25_ADDR,
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

int16_t LPS25::busRead(uint8_t reg, uint8_t *val, int8_t len) {

    int16_t ex;

    struct i2c_msg_s i2c_msg[2] = {
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

float LPS25::getPressure() {

    uint8_t dataRead = 0x00;

    uint32_t pData = 0;

    // Check status register to see if pressure data is available
    busRead(LPS25_PRESSURE_H_REG, &dataRead, 1);
    pData = dataRead << 16; // MSB

    busRead(LPS25_PRESSURE_L_REG, &dataRead, 1);
    pData |= dataRead << 8; //LSB

    busRead(LPS25_PRESSURE_OUT_XL, &dataRead, 1);
    pData |= dataRead;

    _pressure = (float) pData / 4096.0f;

    return _pressure;

//
//    busRead(LPS25_PRESSURE_OUT_XL, &pxl, 1);
//    busRead(LPS25_PRESSURE_L_REG, &pl, 1);
//    busRead(LPS25_PRESSURE_H_REG, &ph, 1);

    return 0;
}
//
//float LPS25::getPressure() {
//
//    int32_t rawPressure = readPressureRaw();
//
//    printf("[LPS25] Raw Pressure: %li\n", rawPressure);
//
//    float pressure = (float) rawPressure / 4096.0f;
//
//    auto * pressureBytes = (uint8_t *) &pressure;
//    auto * rawPressureBytes = (uint8_t *) &rawPressure;
//
//    printf("Raw Bytes: \n");
//    for(int i=0; i < sizeof(rawPressureBytes); i++) {
//
//        if(i == sizeof(pressureBytes) - 1) {
//            printf("%#X\n", rawPressureBytes[i]);
//        } else {
//            printf("%#X | ", rawPressureBytes[i]);
//        }
//
//    }
//
//    printf("Float Bytes: \n");
//    for(int i=0; i < sizeof(pressureBytes); i++) {
//
//        if(i == sizeof(pressureBytes) - 1) {
//            printf("%#X\n", pressureBytes[i]);
//        } else {
//            printf("%#X | ", pressureBytes[i]);
//        }
//
//    }
//
//    printf("[LPS25] Pressure: %f", pressure);
//
//
//    return static_cast<float>(rawPressure) / 4096.0f;
//}

double LPS25::getTemperature() {
    return 0.0f;
}

/**
* void Adafruit_LPS25::setDataRate(lps25_rate_t new_data_rate) {
  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LPS25_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 3, 4);

  data_rate.write((uint8_t)new_data_rate);

  isOneShot = (new_data_rate == LPS25_RATE_ONE_SHOT) ? true : false;
}
*/