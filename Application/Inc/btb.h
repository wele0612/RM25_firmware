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

#include <ctrl_input.h>

typedef struct __attribute__((packed)) {
    int16_t gimbal_request_T_yaw; // 1e-3 Nm/LSB

    uint8_t autoaim_can_fire : 1; // When auto aim says it's good time to shoot
    uint8_t RSVD : 7;
    
    uint8_t PAD[5];
}g2b_A_t;
__BTB_VAR g2b_A_t g2b_A;
#define G2B_MSG_A_ID 0x10

typedef struct __attribute__((packed)) {
    chasis_ctrl_input_t chasis_ctrl;
}g2b_B_t;
__BTB_VAR g2b_B_t g2b_B; // Chasis control input from gimbal
#define G2B_MSG_B_ID 0x11

// Base to Gimbal Message types:

typedef struct __attribute__((packed)) {
    gimbal_ctrl_input_t gimbal_ctrl; // Gimbal control input from chasis
}b2g_A_t;
__BTB_VAR b2g_A_t b2g_A;
#define B2G_MSG_A_ID 0x20

typedef struct __attribute__((packed)) {
    int16_t gimbal_mtr_yaw_pos; // 1e-4 Rad/LSB
    // pos used for vision calibration. Does not drift.
    
    uint8_t PAD[6];
}b2g_B_t;
__BTB_VAR b2g_B_t b2g_B;
#define B2G_MSG_B_ID 0x21

__BTB_VAR uint32_t btb_last_received;
#define BTB_MSG_INTERVAL (HAL_GetTick()-btb_last_received)
#define BTB_ONLINE (BTB_MSG_INTERVAL<=20)
#define BTB_UPDATE_CNTDOWN() do{btb_last_received = HAL_GetTick();}while(0)

#endif
