#include <nuttx/config.h>
#include <nuttx/init.h>
#include <cstdio>
#include <mqueue.h>
#include <sys/types.h>
#include <fcntl.h>
#include <cstdlib>
#include <unistd.h>
// #include "../FlightLib/FlightConfig.h"
// #include "../FlightLib/SensorData.h"
#include "../FlightLib/SensorData.h"
#include "../FlightLib/FlightConfig.h"



static int stateMachineTask(int argc, char *argv[]) {

    mqd_t   imuQueue, magQueue, gpsQueue, baroQueue;

    imuQueue = mq_open("/imuQueue", O_RDONLY);
    if(imuQueue == (mqd_t)-1) {
        printf("[State Machine] Error: Unable to open IMU queue...\n");
        return EXIT_FAILURE;
    }

    magQueue = mq_open("/magQueue", O_RDONLY);
    if(magQueue == (mqd_t)-1) {
        printf("[State Machine] Error: Unable to open MAG queue...\n");
        return EXIT_FAILURE;
    }

    gpsQueue = mq_open("/gpsQueue", O_RDONLY);
    if(gpsQueue == (mqd_t)-1) {
        printf("[State Machine] Error: Unable to open GPS queue...\n");
        return EXIT_FAILURE;
    }

    baroQueue = mq_open("/baroQueue", O_RDONLY);
    if(baroQueue == (mqd_t)-1) {
        printf("[State Machine] Error: Unable to open BARO queue...\n");
        return EXIT_FAILURE;
    }

    while(1) {

        imu_data_t imuData;
        mag_data_t magData;
        gps_data_t gpsData;
        baro_data_t baroData;

        if(mq_receive(imuQueue, (char *)&imuData, sizeof(imu_data_t), nullptr) == -1) {
            printf("[State Machine] Error: Not receiving IMU data...\n");
        }

        if(mq_receive(magQueue, (char *)&magData, sizeof(mag_data_t), nullptr) == -1) {
            printf("[State Machine] Error: Not receiving IMU data...\n");
        }

        if(mq_receive(gpsQueue, (char *)&gpsData, sizeof(gps_data_t), nullptr) == -1) {
            printf("[State Machine] Error: Not receiving IMU data...\n");
        }

        if(mq_receive(baroQueue, (char *)&baroData, sizeof(baro_data_t), nullptr) == -1) {
            printf("[State Machine] Error: Not receiving IMU data...\n");
        }

//        printf("Accel X: %f\n", imuData.accel_x);
//        printf("Gyro Y: %f\n", imuData.gyro_y);
//        printf("Mag Z: %f\n", magData.mag_z);
//        printf("GPS Satellites: %i\n", gpsData.gps_numSats);
        printf("Baro Pressure: %f\n", baroData.pressure);
        printf("Baro Temp: %i\n", baroData.temperature);


        // Convert Hz to microseconds
        usleep(1e6 / stateMachine_loopRate);
    }

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