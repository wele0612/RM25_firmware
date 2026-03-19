#ifndef __BTB_H
#define __BTB_H

/* All board-to-board message type */

// Gimbal to Base Message types:
#include<stdint.h>

#ifdef GENERATE_DECLARE_GLOBAL_VARS
#define __BTB_VAR 
#else
#define __BTB_VAR extern
#endif

// typedef struct __attribute__((packed)) {
//     float gimbal_yaw_pos_imu;
//     int16_t gimbal_pitch; // 1e-4 RAD/LSB
//     int16_t gimbal_fold; // 1e-4 RAD/LSB
// }g2b_A_t;
// __BTB_VAR g2b_A_t g2b_A;
// #define G2B_MSG_A_ID 0x10

// typedef struct __attribute__((packed)) {
//     float gimbal_yaw_vel_imu;
//     uint8_t feeder_in_place : 1;
//     uint8_t rsvd : 7;
//     uint8_t PAD[3];
// }g2b_B_t;
// __BTB_VAR g2b_B_t g2b_B;
// #define G2B_MSG_B_ID 0x11

// typedef struct __attribute__((packed)) {
//     int16_t aim_yaw_pos; // 1e-4 RAD/LSB
    
//     int16_t aim_yaw_vel; // 1e-4 RAD/LSB

//     uint8_t vision_online : 1;
//     uint8_t vision_tracked : 1;
//     uint8_t vision_locked : 1;
//     uint8_t rsvd: 5;

//     int16_t aim_distence; // 1e-3 M/LSB
// }g2b_C_t;
// __BTB_VAR g2b_C_t g2b_C;
// #define G2B_MSG_C_ID 0x12

// Base to Gimbal Message types:

typedef struct __attribute__((packed)) {
    float target_mtr_fold_vel;
    float target_mtr_pitch_vel;
}b2g_A_t;
__BTB_VAR b2g_A_t b2g_A;
#define B2G_MSG_A_ID 0x20

// typedef struct __attribute__((packed)) {
//     float sni_target_pitch_vel;
//     int16_t target_pitch_vel; // 3E-4rad/s per LSB
//     uint8_t flywheel_enabled : 1; // For hero
//     uint8_t feeder_push : 1; // For hero
//     uint8_t aim_enabled : 1; // Enable auto aim
//     uint8_t aim_robot_centre_not_armor : 1; // Aim robot centre, instead of armor
//     uint8_t rsvd:4;
//     uint8_t gimbal_mode;
// } b2g_B_t;
// __BTB_VAR b2g_B_t b2g_B;
// _Static_assert(sizeof(b2g_B_t) == 8, "b2g_B_t size must be 8 bytes");
// #define B2G_MSG_B_ID 0x21

__BTB_VAR uint32_t btb_last_received;
#define BTB_MSG_INTERVAL (HAL_GetTick()-btb_last_received)
#define BTB_ONLINE (BTB_MSG_INTERVAL<=20)
#define BTB_UPDATE_CNTDOWN() do{btb_last_received = HAL_GetTick();}while(0)

#endif
