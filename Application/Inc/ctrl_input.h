#ifndef __CTRL_INPUT_H
#define __CTRL_INPUT_H

#include <stdint.h>
#include <assert.h>

// Abstract for robot controls. All input from DR16, VTM or sentry are mapped into
// this structure, than used by code to control. 

typedef struct __attribute__((packed)){
    int16_t robot_forward_v; // 1e-3 m/(s*LSB)
    int16_t robot_leftward_v; // 1e-3 m/(s*LSB)
    int16_t robot_yaw_omega; // 1e-3 RAD/LSB

    uint8_t supercap_discharge : 1; // Accelerate beyond power limit

    // 0: None. 1: Yaw follow 2: Spintop 3: Variable Spintop
    uint8_t spin_mode : 2; // Spintop level.
    uint8_t fire_pressed : 1; // If the fire button is pressed

    uint8_t minipc_online : 1; // If mini PC is online
    uint8_t vision_allow_fire : 1; // If vision allow shoot
    uint8_t custom_UI_drawcall : 1; // Request to redraw custom UI
    uint8_t bypass_shoot_heat_control : 1; // Ignore heat control and shoot. For emergency only.
    
    uint8_t RSVD1;

}chasis_ctrl_input_t;
static_assert(sizeof(chasis_ctrl_input_t) == 8U, "Must be 8 bytes");

typedef struct __attribute__((packed)){
    int16_t gimbal_yaw_omega; // 1e-3 RAD/LSB
    int16_t gimbal_pitch_omega; // 1e-3 RAD/LSB

    // 0: Disable  1: Mouse  2: Auto-aim  3: Sentry
    uint8_t gimbal_control_mode : 2; 
    
    uint8_t swap_head_tail : 1; // Turn 180 deg to escape
    uint8_t flywheel_enabled : 1; // Flywheel

    uint8_t RSVD0 : 4;

    uint8_t RSVD1[3];

}gimbal_ctrl_input_t;
static_assert(sizeof(gimbal_ctrl_input_t) == 8U, "Must be 8 bytes");


#endif
