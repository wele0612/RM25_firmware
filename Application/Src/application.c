#include<application.h>

#include <buzzer.h>
#include <h7can.h>
#include <icm42688.h>
#include <receiver.h>

#include <robot_arch.h>
robot_motors_t motors;

timeout_monitor_t timeout;

#define DR16_UART &huart5
receiver_DBUS_t dr16;

RAM_D2_SECTION robot_VOFA_report_t vofa = {.tail  = VOFA_TAIL};
imu_data_t imu_data;

// ---- state machines ----
IMU_state_t imu_state=IMU_RESET;

void robot_init(){
    // Must give IMU clock first, so that IMU does not go into sleep mode.
    HAL_TIM_Base_Start_IT(&htim6);// BUG: INT1 does not work. Use TIM6 interruption for now...
    can_bsp_init();

    while(icm_init() != 0);

    buzzer_DJI_startup();

    // HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, (uint8_t*)dr16.msg, 32);
    // __HAL_UART_ENABLE_IT(DR16_UART, UART_IT_IDLE);
    // __HAL_DMA_DISABLE_IT(&huart5,DMA_IT_HT);

    timeout.last_remote_tick=HAL_GetTick();
    
    controller_init();
    HAL_TIM_Base_Start_IT(&htim7);

    //vofa message buffer is in NOLOAD section. Manually initialize after power up.
    memset(&vofa,0,sizeof(vofa));
    vofa.tail[0]=0x00;
    vofa.tail[1]=0x00;
    vofa.tail[2]=0x80;
    vofa.tail[3]=0x7f;

}


void robot_step(const float CTRL_DELTA_T){
    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, SET);

    imu_obtain_data(&imu_data);

    controller_cycle(CTRL_DELTA_T);

    HAL_UART_Transmit_DMA(&huart7, (void *)&vofa, sizeof(vofa));

    //HAL_UART_Transmit_IT(&huart7, (void *)&(vofa), sizeof(vofa));
    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, RESET);
}

void robot_loop(){
    HAL_Delay(200);
    
}

void referee_uart_transmit_once(const uint8_t *send_buf, uint16_t size){

}

void robot_UART_msgcallback(UART_HandleTypeDef *huart){
    
    if(huart == DR16_UART){
        if(HAL_UARTEx_GetRxEventType(DR16_UART) == HAL_UART_RXEVENT_HT){
            return;
        }

        set_DR16_previous_state(&dr16);
        parse_DR16_receiver_msg(&dr16);
        timeout.last_remote_tick=HAL_GetTick();
        //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dr16.msg, 18);
        dr16_on_change();

    // }else if(huart == AIMING_UART){
    //     if(HAL_UARTEx_GetRxEventType(AIMING_UART) == HAL_UART_RXEVENT_HT){
    //         return;
    //     }
    //     parse_aiming_receiver_msg(&aim);

    // }else if(huart == REFEREE_UART){
    //     referee_recv_byte(referee_buf[0]);
    //     HAL_UART_Receive_IT(REFEREE_UART, referee_buf, 1);
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    robot_UART_msgcallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    robot_UART_msgcallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart == DR16_UART)
	{
		__HAL_UNLOCK(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, (uint8_t*)dr16.msg, 32);
    }
}


