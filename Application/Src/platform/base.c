#include<config.h>
#include<application.h>
#ifdef CONFIG_PLATFORM_BASE

void controller_init(){
    role_controller_init();
}

void controller_cycle(const float CTRL_DELTA_T){
    role_controller_step(CTRL_DELTA_T);
}



void dr16_on_change(){
    
}

#endif

