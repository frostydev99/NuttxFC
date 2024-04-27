#include "MMC5983MA.h"

MMC5983MA::MMC5983MA() = default;

bool MMC5983MA::init(int i2cBus) {
    this->fd = i2cBus;

    return true;
}