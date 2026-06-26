#ifndef __CONFIG_H
#define __CONFIG_H

//========= Choose Platform ===========
#define CONFIG_PLATFORM_GIMBAL
// #define CONFIG_PLATFORM_BASE

//========= Choose Robot ===========
// #define CONFIG_ROBOT_INFANTRY_BALANCE
// #define CONFIG_ROBOT_SENTRY_OMNI
#define CONFIG_ROBOT_INFANTRY_OMNI
// #define CONFIG_ROBOT_HERO
// #define CONFIG_ROBOT_AIMING_BENCH

//========= Choose Vision System ===========
#define CONFIG_AIM_SP_VISION_25
// #define CONFIG_AIM_RM_VISION

//========= Choose Auto Sentry ===========
// #define CONFIG_AUTO_SENTRY

//========= Choose Control Method ===========
// #define CONFIG_KEYBOARD_CONTROL
#define CONFIG_JOYSTICK_CONTROL

//-------------------------------------------
// DO NOT modify code below this line
// UNLESS you clearly know what you are doing
//--------------------------------------------

#define CONFIG_ENABLE_VISION

//========= Dependent Options ===========
#if defined(CONFIG_ROBOT_INFANTRY_BALANCE) && defined(CONFIG_PLATFORM_GIMBAL)
    #define CONFIG_ENABLE_IMU_FLYWHEEL_FILTER
#endif

#if defined(CONFIG_ROBOT_HERO) && defined(CONFIG_PLATFORM_GIMBAL)
    #define CONFIG_ENABLE_IMU_FLYWHEEL_FILTER
    #define REVERSE_PITCH
#endif

#if defined(CONFIG_ROBOT_SENTRY_OMNI) && defined(CONFIG_PLATFORM_GIMBAL)
    #define CONFIG_ENABLE_IMU_FLYWHEEL_FILTER
    #define REVERSE_PITCH
#endif

#if defined(CONFIG_ROBOT_INFANTRY_OMNI) && defined(CONFIG_PLATFORM_GIMBAL)
    #define CONFIG_ENABLE_IMU_FLYWHEEL_FILTER
    #define REVERSE_PITCH
#endif

#if defined(CONFIG_INPUT_DR16)
    #define CONFIG_BASE_HAS_CONTROL
#endif

#endif
