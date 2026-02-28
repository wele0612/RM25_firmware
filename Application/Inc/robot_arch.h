#ifndef __ROBOT_ARCH_H
#define __ROBOT_ARCH_H

#include<config.h>

#include <motors.h>

// Gimbal Platform Combinations
#ifdef CONFIG_PLATFORM_GIMBAL
    #ifdef CONFIG_ROBOT_INFANTRY_BALANCE
        // GIMBAL + INFANTRY_BALANCE configuration
        typedef struct robot_motors_t{
            report_M3508_t wheel_left;
            report_M3508_t wheel_right;

        }robot_motors_t;

        typedef struct robot_ctrl_t{
            float target_rpm_left;
            float target_rpm_right;
        }robot_ctrl_t;
        
    #endif
    
    #ifdef CONFIG_ROBOT_INFANTRY_OMNI
        // GIMBAL + INFANTRY_OMNI configuration
    #endif
    
    #ifdef CONFIG_ROBOT_HERO
        // GIMBAL + HERO configuration
        typedef struct robot_motors_t{
            report_M3508_t flywheel_1; // ID = 1
            report_M3508_t flywheel_2; // ID = 2
            report_M3508_t flywheel_3; // ID = 3

            report_M2006_t feeder_top; // ID = 4

        }robot_motors_t;

        typedef struct robot_ctrl_t{ // All units SI, unless specified.
            float feeder_position;
            float feeder_vel;

            float target_flywheel_rpm; // RPM
            float target_feeder_vel;
            float target_feeder_position;
        }robot_ctrl_t;
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

        typedef struct robot_ctrl_t{ // All Units in SI
            float yaw_f; // Draft-removed yaw with wheel motion
            float yaw_m1; // Yaw of last cycle

            float L_l;
            float L_r;
            float dL_l;
            float dL_r;

            float s; // increase if moving forward
            float ds; // positive moving forward
            float phi; // right-hand rule, pointing upward
            float dphi; // right-hand rule, pointing upward
            float th_ll; // left leg - ground angle, positive when wheel is behind the body
            float dth_ll; // derivative of th_ll
            float th_lr; // right leg - ground angle, positive when wheel is behind the body
            float dth_lr; // derivative of th_lr
            float th_b; // body - ground angle, positive when nose-down, tail-up
            float dth_b; // derivative of th_b

            float lqr_err[10];

            float J_l[4];
            float J_r[4];

            // Same, but relative to robot body coordinate system: Does not consider th_b
            float th_ll_nb; // left leg - ground angle, positive when wheel is behind the body
            float dth_ll_nb; // derivative of th_ll
            float th_lr_nb; // right leg - ground angle, positive when wheel is behind the body
            float dth_lr_nb; // derivative of th_lr

            float s_max;
 
            // float 
            float Fnl; // leg noraml force (left), pointing outward
            float Fnr; // leg noraml force (right), pointing outward
            float Tbll; // leg torque (left)
            float Tblr; // leg torque (right)
            float Twl; // wheel torque (left)
            float Twr; // wheel torque (right)

            float T_LF;
            float T_LB;
            float T_RF;
            float T_RB;

            float F_wheel_support; // total supporting force from the joint

            float target_ds;
            float target_phi;
            float target_dphi;
            float target_th_ll;
            float target_th_lr;
            float target_b_height;

            float target_L_length;
            float target_R_length;

            float target_L_leg_omega;
            float target_R_leg_omega;

        }robot_ctrl_t;
    #endif
    
    #ifdef CONFIG_ROBOT_INFANTRY_OMNI
        // BASE + INFANTRY_OMNI configuration
        
    #endif
    
    #ifdef CONFIG_ROBOT_HERO
        // BASE + HERO configuration
        typedef struct robot_motors_t{

        }robot_motors_t;

        typedef struct robot_ctrl_t{
 
        }robot_ctrl_t;
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