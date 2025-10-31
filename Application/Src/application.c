#include<application.h>

#include <buzzer.h>
#include <h7can.h>
#include <icm42688.h>
#include <receiver.h>

#include <robot_arch.h>

#define GENERATE_DECLARE_GLOBAL_VARS
#include <global_variables.h>
#undef GENERATE_DECLARE_GLOBAL_VARS

timeout_monitor_t timeout;

// DMA buffer outside
RAM_D2_SECTION uint8_t dr16_buffer_recv[32]; // 18 bytes total

RAM_D2_SECTION robot_VOFA_report_t vofa = {.tail  = VOFA_TAIL};

static int err=0;
static void self_test(){
    err = 0;
    // Check IMU calibration data
    const float imu_gyro_offset_max = 3.0f;
    for(int i=0;i<3;i++){
        if(isnan(robot_config.imu_gyro_offset[i]))  err=1;
        if(fabs(robot_config.imu_gyro_offset[i])>imu_gyro_offset_max)   err=2;
    }

    while(err){
        buzzer_calibration_startup();
    }
}

void robot_init(){
    imu_state=IMU_RESET;

    // Must give IMU clock first, so that IMU does not go into sleep mode.
    HAL_TIM_Base_Start_IT(&htim6);// BUG: IMU_INT1 does not work. Use TIM6 interruption for now...
    can_bsp_init();
    HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, dr16_buffer_recv, 32);

    while(icm_init() != 0); //Init IMU
    robot_readconfig(&robot_config); //Read from flash

    // Hold User key during start up to enter IMU callibration mode.
#define IMU_CALIBRATION_MODE_ENABLE
#ifdef IMU_CALIBRATION_MODE_ENABLE
    if(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == RESET){
        HAL_Delay(10);
        if(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == RESET){
            buzzer_calibration_startup();
            HAL_Delay(5000);

            imu_state = IMU_CALIBRATE;

            for(int i=0;i<3;i++){
                robot_config.imu_gyro_offset[i]=0.0f;
            }

            HAL_Delay(10);

            float gyro_offset[3]={0.0f,0.0f,0.0f};
            const float calib_sample_nums=2000;
            for(int i=0;i<calib_sample_nums;i++){
                imu_obtain_data(&imu_data);
                gyro_offset[0] += imu_data.gyro[0];
                gyro_offset[1] += imu_data.gyro[1];
                gyro_offset[2] += imu_data.gyro[2];
            }
            robot_config.test_val += 1;
            robot_config.imu_gyro_offset[0] = gyro_offset[0]/calib_sample_nums;
            robot_config.imu_gyro_offset[1] = gyro_offset[1]/calib_sample_nums;
            robot_config.imu_gyro_offset[2] = gyro_offset[2]/calib_sample_nums;
            robot_saveconfig(&robot_config);

            imu_state = IMU_RUNNING;
        }
    }
#endif

    self_test();

    buzzer_DJI_startup();

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

        HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, dr16_buffer_recv, 32); // 接收完毕后重启

        set_DR16_previous_state(&dr16);

        // parse_DR16_receiver_msg(&dr16);
        parse_DR16_receiver_msg(&dr16, dr16_buffer_recv);

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

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
// 	if(huart == DR16_UART)
// 	{
// 		__HAL_UNLOCK(huart);
// 		HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, (uint8_t*), 32);
//     }
// }


