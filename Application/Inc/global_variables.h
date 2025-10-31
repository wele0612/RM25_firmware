#ifndef GLOBAL_H
#define GLOBAL_H

#ifdef GENERATE_DECLARE_GLOBAL_VARS
#define __GLOBAL_VAR 
#else
#define __GLOBAL_VAR extern
#endif

#include <robot_arch.h>
#include <application.h>
#include <receiver.h>
#include <icm42688.h>


typedef struct robot_VOFA_report_t{
    float val[10];
    unsigned char tail[4];
}robot_VOFA_report_t;
__GLOBAL_VAR robot_VOFA_report_t vofa;

#define REMOTE_TIMEOUT (50U)
typedef struct timeout_monitor_t{
    uint64_t last_remote_tick;
}timeout_monitor_t;

typedef struct robot_config_t{
    float imu_gyro_offset[3];
    int test_val;
}Robot_config_t;
__GLOBAL_VAR Robot_config_t robot_config;

__GLOBAL_VAR robot_motors_t motors;
__GLOBAL_VAR robot_ctrl_t robot_geo;

#define DR16_UART &huart5
__GLOBAL_VAR receiver_DBUS_t dr16;

__GLOBAL_VAR imu_data_t imu_data;

// -----------------------------------------------------
// | State machines.                                   |
// |                                                   |
// -----------------------------------------------------

typedef enum IMU_state{
    IMU_RESET,
    IMU_RUNNING,
    IMU_CALIBRATE
}IMU_state_t;
__GLOBAL_VAR IMU_state_t imu_state;

#endif 
