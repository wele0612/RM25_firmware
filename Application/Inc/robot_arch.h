#ifndef __ROBOT_ARCH_H
#define __ROBOT_ARCH_H

#include<config.h>

#include <motors.h>

// Gimbal Platform Combinations
#ifdef CONFIG_PLATFORM_GIMBAL
    #ifdef CONFIG_ROBOT_INFANTRY_BALANCE
        // GIMBAL + INFANTRY_BALANCE configuration
    #endif
    
    #ifdef CONFIG_ROBOT_INFANTRY_OMNI
        // GIMBAL + INFANTRY_OMNI configuration
    #endif
    
    #ifdef CONFIG_ROBOT_HERO
        // GIMBAL + HERO configuration
    #endif
    
    #ifdef CONFIG_ROBOT_AIMING_BENCH
        // GIMBAL + AIMING_BENCH configuration
        typedef struct robot_motors_t{
            report_DM4310_t pitch;

        }robot_motors_t;

        typedef struct robot_ctrl_t{
            float target_pitch_omega;
            float target_pitch_pos;
        }robot_ctrl_t;

    #endif
#endif

// Base Platform Combinations
#ifdef CONFIG_PLATFORM_BASE
    #ifdef CONFIG_ROBOT_INFANTRY_BALANCE
        // BASE + INFANTRY_BALANCE configuration
    #endif
    
    #ifdef CONFIG_ROBOT_INFANTRY_OMNI
        // BASE + INFANTRY_OMNI configuration
    #endif
    
    #ifdef CONFIG_ROBOT_HERO
        // BASE + HERO configuration
    #endif
    
    #ifdef CONFIG_ROBOT_AIMING_BENCH
        // BASE + AIMING_BENCH configuration
        typedef struct robot_motors_t{
            report_M3508_t wheel_LF;
            report_M3508_t wheel_LB;
            report_M3508_t wheel_RF;
            report_M3508_t wheel_RB;

        }robot_motors_t;

        typedef struct robot_ctrl_t{
            float target_speed_x;
            float target_speed_y;
            float target_omega_yaw;
        }robot_ctrl_t;
    #endif
#endif

#endif