#pragma once

typedef struct {
    // IMU Readings
    float accel_x; // [g]
    float accel_y; // [g]
    float accel_z; // [g]
    float gyro_x;  // [rad/s]
    float gyro_y;  // [rad/s]
    float gyro_z;  // [rad/s]
} imu_data_t;

typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
} mag_data_t;

typedef struct {
    // GPS Readings
    float gps_e_x;
    float gps_e_y;
    float gps_e_z;
    uint8_t gps_numSats;
} gps_data_t;

typedef struct {
    float pressure; // [hPa]
    uint16_t temperature; // [C] ?
} baro_data_t;