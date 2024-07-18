#pragma once

typedef struct {
    // IMU Readings
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;

    // GPS Readings
    float gps_e_x;
    float gps_e_y;
    float gps_e_z;
    float gps_numSats;
} sensor_data_t;