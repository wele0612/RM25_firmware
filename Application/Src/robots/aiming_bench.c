#include<config.h>
#ifdef CONFIG_ROBOT_AIMING_BENCH

#include<role.h>

#ifdef CONFIG_PLATFORM_BASE

void role_controller_init(){

}

void role_controller_step(const float CTRL_DELTA_T){

}

#else

void role_controller_init(){

}

void role_controller_step(const float CTRL_DELTA_T){

}

#endif

#endif
