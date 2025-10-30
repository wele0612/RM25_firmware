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

__GLOBAL_VAR robot_motors_t motors;
__GLOBAL_VAR robot_ctrl_t robot_geo;
__GLOBAL_VAR robot_VOFA_report_t vofa;

#define DR16_UART &huart5
__GLOBAL_VAR receiver_DBUS_t dr16;

__GLOBAL_VAR imu_data_t imu_data;

// -------- state machines --------
__GLOBAL_VAR IMU_state_t imu_state;

#endif 
