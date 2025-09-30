#include<application.h>

#include <buzzer.h>
#include <h7can.h>

#include <robot_arch.h>
robot_motors_t motors;

void robot_init(){
    can_bsp_init();
    buzzer_DJI_startup();

    controller_init();
    HAL_TIM_Base_Start_IT(&htim7);
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

