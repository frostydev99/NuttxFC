#include <stdio.h>

#include "sensors/LPS25/LPS25.h"

LPS25 barometer = LPS25();

static int i2cBus = 0;

static int initFlightComputer() {
    int ret = 0;

    i2cBus = open("/dev/i2c0", O_WRONLY);

    if(i2cBus < 0) {
        printf("[Flight Computer] I2C0 Failed to initialize...\n");

        ret = -1;
    } else {
        printf("[Flight Computer] I2C0 Successfully Initialized!\n");
        ret = 1;
    }

    return ret;
}

static int stateMachineTask(int argc, char *argv[]) {

    int initSuccess = initFlightComputer();

    initSuccess = barometer.init(i2cBus);

    float pressure = barometer.getPressure();

    printf("[Flight Computer] Pressure: %4f", pressure);

//    double temp = barometer.getTemperature();

//    while(initSuccess >= 0) {
//        initSuccess = 1;
//
//        float pressure = barometer.getPressure();
//
//
//        printf("[Flight Computer] Looping...\n");
//
//        sleep(1);
//
////        usleep(10000); // 100Hz
//    }

    close(i2cBus);
    printf("[Flight Computer] Task Terminated...\n");

    return EXIT_FAILURE;
}

extern "C" int FC_StateMachine_main(int argc, char *argv[]) {
	printf("Starting Flight Computer...\n");

    int ret = task_create("StateMachineTask",
                          CONFIG_FLIGHTCOMPUTER_STATEMACHINE_PRIORITY,
                          CONFIG_FLIGHTCOMPUTER_STATEMACHINE_STACKSIZE,
                          stateMachineTask,
                          nullptr);

    if(ret < 0) {
        int errcode = errno;

        printf("[State Machine] ERROR: Failed to start State Machine task: %d\n", errcode);

        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}