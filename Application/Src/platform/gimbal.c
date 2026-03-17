#include<config.h>
#include<application.h>

#include<vision.h>
#include<global_variables.h>

#include<buzzer.h>
#ifdef CONFIG_PLATFORM_GIMBAL

RAM_D2_SECTION uint8_t vision_ToRos_buf[VISION_TO_ROS_SIZE];

void controller_init(){
    role_controller_init();
}

void controller_cycle(const float CTRL_DELTA_T){
    role_controller_step(CTRL_DELTA_T);

    if(vision_FromRos.packet.tracking && (HAL_GetTick()%200) > 100){
        buzzer_set_freq(TUNE_A6);
        buzzer_on();
    }else{
        buzzer_off();
    }
    
    if(HAL_GetTick()%2 == 0){ // 500Hz
        McuToRosPacket_t* toRos = &(vision_ToRos.packet);
        toRos->detect_color = 1;
        toRos->roll = imu_data.roll; 
        #ifdef REVERSE_PITCH
        toRos->pitch = imu_data.pitch;
        #else
        toRos->pitch = -imu_data.pitch; // Note: ROS and ICM42688 has opposite defination for Pitch
        #endif
        toRos->yaw = imu_data.yaw;
        toRos->reset_tracker = 0;
        HAL_UART_StateTypeDef state = HAL_UART_GetState(AIMING_UART);
        if (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX){
            memcpy(vision_ToRos_buf, vision_send_pack(&vision_ToRos), VISION_TO_ROS_SIZE);
            HAL_UART_Transmit_DMA(AIMING_UART, vision_ToRos_buf, VISION_TO_ROS_SIZE);
        }
    }
    
}

__weak void dr16_on_change(){}

#endif
