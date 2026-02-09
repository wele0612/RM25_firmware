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

        }robot_ctrl_t;

    #endif
#endif

// Base Platform Combinations
#ifdef CONFIG_PLATFORM_BASE
    #ifdef CONFIG_ROBOT_INFANTRY_BALANCE
        // BASE + INFANTRY_BALANCE configuration
        #define JOINT_LF_CTRLID 0x08
        #define JOINT_RF_CTRLID 0x09
        #define JOINT_LB_CTRLID 0x0A
        #define JOINT_RB_CTRLID 0x0B
        #define JOINT_YAW_CTRLID 0x0C

        #define JOINT_LF_FEEDBACKID 0x00
        #define JOINT_RF_FEEDBACKID 0x01
        #define JOINT_LB_FEEDBACKID 0x02
        #define JOINT_RB_FEEDBACKID 0x03
        #define JOINT_YAW_FEEDBACKID 0x04

        typedef struct robot_motors_t{
            // CAN2 motors:
            report_M3508_t wheel_L; // ID = 1
            report_M3508_t wheel_R; // ID = 2
            report_M2006_t agi; // ID = 3

            // CAN3 motors:
            report_DM8009P_t joint_LF;
            report_DM8009P_t joint_RF;
            report_DM8009P_t joint_LB;
            report_DM8009P_t joint_RB;
            report_DM4310_t joint_yaw;
            
        }robot_motors_t;

        typedef struct robot_ctrl_t{

        }robot_ctrl_t;
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