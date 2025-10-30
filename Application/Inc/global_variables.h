#ifndef GLOBAL_H
#define GLOBAL_H

#include <robot_arch.h>
#include <application.h>
#include <receiver.h>
#include <icm42688.h>

extern robot_motors_t motors;
extern robot_ctrl_t robot_geo;
extern robot_VOFA_report_t vofa;
extern receiver_DBUS_t dr16;
extern imu_data_t imu_data;

// state machines
extern IMU_state_t imu_state;

#endif 
