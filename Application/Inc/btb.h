#ifndef __BTB_H
#define __BTB_H

/* All board-to-board message type */

// Gimbal to Base Message types:

#ifdef GENERATE_DECLARE_GLOBAL_VARS
#define __BTB_VAR 
#else
#define __BTB_VAR extern
#endif

typedef struct __attribute__((packed)) {
    float gimbal_yaw_vel_imu;
    int16_t gimbal_pitch; // 1e-4 RAD/LSB
    int16_t gimbal_fold; // 1e-4 RAD/LSB
}g2b_A_t;
__BTB_VAR g2b_A_t g2b_A;
#define G2B_MSG_A_ID 0x10

typedef struct __attribute__((packed)) {
    uint8_t feeder_in_place : 1;
    uint8_t rsvd : 7;
    uint8_t PAD[7];
}g2b_B_t;
__BTB_VAR g2b_B_t g2b_B;
#define G2B_MSG_B_ID 0x11

// Base to Gimbal Message types:

// typedef struct __attribute__((packed)) {
//     float base_yaw;
//     float base_pitch;
// }b2g_A_t;
// __BTB_VAR b2g_A_t b2g_A;
// #define B2G_MSG_A_ID 0x20

typedef struct __attribute__((packed)) {
    float base_roll;
    int16_t target_pitch_vel; // 3E-4rad/s per LSB
    uint8_t flywheel_enabled:1;
    uint8_t feeder_push:1;
    uint8_t rsvd:6;
    uint8_t gimbal_mode;
} b2g_B_t;
__BTB_VAR b2g_B_t b2g_B;
_Static_assert(sizeof(b2g_B_t) == 8, "b2g_B_t size must be 8 bytes");
#define B2G_MSG_B_ID 0x21

__BTB_VAR uint32_t btb_last_received;
#define BTB_ONLINE (HAL_GetTick()-btb_last_received <= 20)
#define BTB_UPDATE_CNTDOWN() do{btb_last_received = HAL_GetTick();}while(0)

#endif
