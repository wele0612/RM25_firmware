#include<config.h>
#include<application.h>

#include<vision.h>
#include<global_variables.h>
#include<btb.h>
#include<h7can.h>

#include<buzzer.h>
#ifdef CONFIG_PLATFORM_GIMBAL

RAM_D2_SECTION uint8_t vision_ToRos_buf[VISION_TO_ROS_SIZE];

void controller_init(){
    role_controller_init();
    ESTOP_reset();
}

void controller_cycle(const float CTRL_DELTA_T){
    role_controller_step(CTRL_DELTA_T);
    
    if(HAL_GetTick()%2 == 0){ // 500Hz
        McuToRosPacket_t* toRos = &(vision_ToRos.packet);
        toRos->aim_color = 0;

        float bullet_speed_referee = b2g_A.gimbal_ctrl.feedback_shoot_speed*1e-3f;
        if(bullet_speed_referee > 10.0f){
            toRos->bullet_speed = bullet_speed_referee;
        }else{
            toRos->bullet_speed = 15.0f;
        }
        
        toRos->mode = SP25_AUTO_AIM;

        #ifdef REVERSE_PITCH
        toRos->pitch = imu_data.pitch;
        toRos->pitch_vel = imu_data.gyro[1]*DEGtoRAD;
        #else
        // Note: ROS and ICM42688 has opposite defination for Pitch
        toRos->pitch = -imu_data.pitch; 
        toRos->pitch_vel = -imu_data.gyro[1]*DEGtoRAD;
        #endif

        float yaw_diff = imu_data.yaw - toRos->yaw;
        // // float yaw_diff = b2g_B.gimbal_mtr_yaw_pos*1e-4f - toRos->yaw;
        // if(yaw_diff > PI){
        //     toRos->yaw += yaw_diff - (2.0f*PI);
        // }else if(yaw_diff < -PI){
        //     toRos->yaw += yaw_diff + (2.0f*PI);
        // }else{
        //     toRos->yaw += yaw_diff;
        // }
        toRos->yaw = imu_data.yaw;
        toRos->yaw_vel = imu_data.gyro[2]*DEGtoRAD;

        ypr_to_spvision_q(toRos->yaw, toRos->pitch, imu_data.roll, toRos->q);

        HAL_UART_StateTypeDef state = HAL_UART_GetState(AIMING_UART);
        if (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX){
            memcpy(vision_ToRos_buf, vision_send_pack(&vision_ToRos), VISION_TO_ROS_SIZE);
            HAL_UART_Transmit_DMA(AIMING_UART, vision_ToRos_buf, VISION_TO_ROS_SIZE);
        }
    }

    HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, gimbal_ctrl.fired ? SET : RESET);
    
    // Upload chasis control commands
    memcpy(&g2b_B.chasis_ctrl, &chasis_ctrl, sizeof(chasis_ctrl_input_t));
    fdcanx_send_data(&hfdcan1, G2B_MSG_B_ID, (uint8_t *)&g2b_B, 8);
}

__weak void dr16_on_change(){}

#endif
