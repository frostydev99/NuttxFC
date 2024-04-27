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

private:
    int fd = 0;
};