#include<application.h>

void robot_init(){

    controller_init();
}

void robot_step(const float CTRL_DELTA_T){

    controller_cycle(CTRL_DELTA_T);
}

void robot_loop(){

}

void referee_uart_transmit_once(const uint8_t *send_buf, uint16_t size){

}

