#include<config.h>
#include<application.h>

#include <motors.h>
#ifdef CONFIG_PLATFORM_BASE

void controller_init(){
    role_controller_init();
    ESTOP_reset();
}

void controller_cycle(const float CTRL_DELTA_T){
    role_controller_step(CTRL_DELTA_T);

    // ESTOP if remote is offline
    if(remote_online()){
        ESTOP_reset();
    }else{
        ESTOP();
    }
}

__weak void dr16_on_change(){
    
}

#endif

