#include<config.h>
#ifdef CONFIG_ROBOT_AIMING_BENCH

#include<application.h>

#include<h7can.h>


#ifdef CONFIG_PLATFORM_BASE

void role_controller_init(){

}

uint8_t rxtest_buffer[8]={2,0,1,2,0,7,1,2};
void role_controller_step(const float CTRL_DELTA_T){
    
    fdcanx_send_data(&hfdcan2, 0x88, rxtest_buffer, 8);
}

void robot_CAN_msgcallback(int ID, uint8_t *msg){
    volatile int id = ID;

    return;
}



#else

void role_controller_init(){

}

void role_controller_step(const float CTRL_DELTA_T){

}

#endif

#endif
