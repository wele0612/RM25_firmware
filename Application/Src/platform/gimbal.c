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

    if(vision_FromRos.packet.tracking){
        if((HAL_GetTick()%480) > 240){
            buzzer_set_freq(TUNE_D6_SHARP_E6_FLAT);
        }else{
            buzzer_set_freq(TUNE_B6);
        }
        buzzer_on();
    }else{
        buzzer_off();
    }
    
    // construct the packet from MCU to mini PC
    if(HAL_GetTick()%2 == 0){ // 500Hz
        McuToRosPacket_t* toRos = &(vision_ToRos.packet);
        // 260527 new added: constructing packet
        toRos->mode = 0;
        #ifdef REVERSE_PITCH
        toRos->pitch = imu_data.pitch;
        #else
        toRos->pitch_vel = -imu_data.gyro[1]; // Note: ROS and ICM42688 has opposite defination for Pitch
        #endif
        toRos->yaw_vel = imu_data.gyro[2];
        toRos->yaw = imu_data.yaw;
        toRos->pitch = imu_data.pitch;
        
        // get the quaternion data from MahonyAHRS:
        toRos->q[0] = imu_data.q[0];
        toRos->q[1] = imu_data.q[1];
        toRos->q[2] = imu_data.q[2];
        toRos->q[3] = imu_data.q[3];

        // NOT PROVIDED AT 260527
        toRos->bullet_speed = 25.0f;
        toRos->bullet_count = 0; // 累积发弹量
        HAL_UART_StateTypeDef state = HAL_UART_GetState(AIMING_UART);
        if (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX){
            memcpy(vision_ToRos_buf, vision_send_pack(&vision_ToRos), VISION_TO_ROS_SIZE);
            HAL_UART_Transmit_DMA(AIMING_UART, vision_ToRos_buf, VISION_TO_ROS_SIZE);
            // HAL_UART_Transmit_DMA(&huart10, vision_ToRos_buf, VISION_TO_ROS_SIZE);
        }
    }
    
}

__weak void dr16_on_change(){}

#endif
