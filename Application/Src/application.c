#include<application.h>

void robot_init(){
    HAL_TIM_Base_Start_IT(&htim7);

    controller_init();
}

void robot_step(const float CTRL_DELTA_T){
    HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, SET);

    controller_cycle(CTRL_DELTA_T);
    HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, RESET);
}

void robot_loop(){
    HAL_Delay(200);
    
}

void referee_uart_transmit_once(const uint8_t *send_buf, uint16_t size){

}

