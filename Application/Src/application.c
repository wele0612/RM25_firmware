#include<application.h>

#include <buzzer.h>
#include <h7can.h>
#include <icm42688.h>

#include <robot_arch.h>
robot_motors_t motors;

robot_VOFA_report_t vofa = {.tail  = VOFA_TAIL};
imu_data_t imu_data;

void robot_init(){
    can_bsp_init();

    while(icm_init() != 0);

    buzzer_DJI_startup();
    
    controller_init();
    HAL_TIM_Base_Start_IT(&htim7);
}

void robot_step(const float CTRL_DELTA_T){
    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, SET);

    imu_obtain_data(&imu_data);

    vofa.val[0]=imu_data.gyro[0];
    vofa.val[1]=imu_data.gyro[1];
    vofa.val[2]=imu_data.gyro[2];
    vofa.val[3]=imu_data.gyro[0];
    vofa.val[4]=imu_data.gyro[1];
    vofa.val[5]=imu_data.gyro[2];
    // vofa.val[3]=imu.acc[0];
    // vofa.val[4]=imu.acc[1];
    // vofa.val[5]=imu.acc[2];
    vofa.val[7]=imu_data.yaw;
    vofa.val[8]=imu_data.pitch;
    vofa.val[9]=imu_data.roll;

    controller_cycle(CTRL_DELTA_T);

    HAL_UART_Transmit_IT(&huart7, (void *)&(vofa), sizeof(vofa));
    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, RESET);
}

void robot_loop(){
    HAL_Delay(200);
    
}

void referee_uart_transmit_once(const uint8_t *send_buf, uint16_t size){

}

