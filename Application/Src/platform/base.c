#include<config.h>
#include<application.h>

#include <motors.h>

#include <h7can.h>
#include <btb.h>
#include <global_variables.h>

#ifdef CONFIG_PLATFORM_BASE

void controller_init(){
    role_controller_init();
    ESTOP_reset();
}

void DR16_on_change(){

    // add this in referee update
    // gimbal_ctrl.feedback_shoot_speed = (int16_t)(referee.shoot_data_0x0207.initial_speed*1e3f);

    return;
}

void controller_cycle(const float CTRL_DELTA_T){
    role_controller_step(CTRL_DELTA_T);

    // ESTOP if remote is offline
    if(control_online()){
        ESTOP_reset();
    }else{
        ESTOP();
    }

}

#endif

