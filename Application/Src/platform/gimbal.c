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

    if(control_online()){
        ESTOP_reset();
    }else{
        ESTOP();
    }
    
    if(HAL_GetTick()%2 == 0){ // 500Hz
        McuToRosPacket_t* toRos = &(vision_ToRos.packet);
        toRos->aim_color = 0;

        float bullet_speed_referee = b2g_B.feedback_shoot_speed*1e-3f;
        if(bullet_speed_referee > 10.0f){
            toRos->bullet_speed = bullet_speed_referee;
        }else{
            toRos->bullet_speed = 14.7f;
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

        toRos->self_HP = b2g_B.self_HP;
        toRos->match_started = b2g_B.match_started;

        toRos->yaw = imu_data.yaw;
        // toRos->yaw = b2g_B.gimbal_mtr_yaw_pos*1e-4f;
        toRos->yaw_vel = imu_data.gyro[2]*DEGtoRAD;
        toRos->aim_color = b2g_B.is_enemy_red ? 1:0;

        ypr_to_spvision_q(toRos->yaw, toRos->pitch, imu_data.roll, toRos->q);

        HAL_UART_StateTypeDef state = HAL_UART_GetState(AIMING_UART);
        if (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX){
            memcpy(vision_ToRos_buf, vision_send_pack(&vision_ToRos), VISION_TO_ROS_SIZE);
            HAL_UART_Transmit_DMA(AIMING_UART, vision_ToRos_buf, VISION_TO_ROS_SIZE);
        }
    }

    HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, chasis_ctrl.fire_pressed ? SET : RESET);
    
    // Upload chasis control commands
    if(control_online()){
        chasis_ctrl.minipc_online = vision_online() ? 1:0;
        if(gimbal_ctrl.gimbal_control_mode == 1){
            chasis_ctrl.vision_allow_fire = 1;
        }else{
            chasis_ctrl.vision_allow_fire = (vision_FromRos.packet.mode == 2)? 1:0;
        }

        if(chasis_ctrl.L5_auto_drive){
            if(vision_online()){
                const float speed_limit = 1.0f;
                chasis_ctrl.robot_forward_v = limit_val(vision_FromRos.packet.forward_vel, speed_limit)*1e3f;
                chasis_ctrl.robot_leftward_v = limit_val(vision_FromRos.packet.leftward_vel, speed_limit)*1e3f;
            }else{
                chasis_ctrl.robot_forward_v = 0.0f;
                chasis_ctrl.robot_leftward_v = 0.0f;
            }
        }

        memcpy(&g2b_B.chasis_ctrl, &chasis_ctrl, sizeof(chasis_ctrl_input_t));
        fdcanx_send_data(&hfdcan1, G2B_MSG_B_ID, (uint8_t *)&g2b_B, 8);
    }

}

void process_keyboard(uint16_t key, uint16_t* key_event, mouse_state_t* mouse){
    const int16_t press_move_vel = (int16_t)(2.0f*1e3);
        const int16_t press_rotate_vel = (int16_t)(PI*1e3);
        int16_t accelerate_factor;
        if(key & KEYBOARD_SHIFT_BIT){
            accelerate_factor = 3;
            chasis_ctrl.supercap_discharge = 1;
        }else{
            accelerate_factor = 1;
            chasis_ctrl.supercap_discharge = 0;
        }

        if(key & KEYBOARD_W_BIT){
            chasis_ctrl.robot_forward_v = accelerate_factor*press_move_vel;
        }else if(key & KEYBOARD_S_BIT){
            chasis_ctrl.robot_forward_v = accelerate_factor*-press_move_vel;
        }else{
            chasis_ctrl.robot_forward_v = 0;
        }

        if(key & KEYBOARD_A_BIT){
            chasis_ctrl.robot_leftward_v = accelerate_factor*press_move_vel;
        }else if(key & KEYBOARD_D_BIT){
            chasis_ctrl.robot_leftward_v = accelerate_factor*-press_move_vel;
        }else{
            chasis_ctrl.robot_leftward_v = 0;
        }

        // if(key & KEYBOARD_Q_BIT){
        //     chasis_ctrl.robot_yaw_omega = accelerate_factor*press_rotate_vel;
        // }else if(key & KEYBOARD_E_BIT){
        //     chasis_ctrl.robot_yaw_omega = accelerate_factor*-press_rotate_vel;
        // }else{
        //     chasis_ctrl.robot_yaw_omega = 0;
        // }

        // int swap_head_tail = gimbal_ctrl.swap_head_tail;

        // if((key & KEYBOARD_X_BIT) && (*key_event & KEYBOARD_X_BIT)){ // posedge
        //     swap_head_tail = swap_head_tail ? 0:1;
        //     *key_event &= (~KEYBOARD_X_BIT);
        // }
        int swap_head_tail = (key & KEYBOARD_X_BIT) ? 1:0;

        if((key & KEYBOARD_Q_BIT) && (*key_event & KEYBOARD_Q_BIT)){
            if(chasis_ctrl.spin_mode != 3){
                chasis_ctrl.spin_mode = 3;
            }else{
                chasis_ctrl.spin_mode = 1;
            }
            *key_event &= (~KEYBOARD_Q_BIT);
        }

        chasis_ctrl.custom_UI_drawcall = (key & KEYBOARD_R_BIT) ? 1:0;
        chasis_ctrl.fire_pressed = mouse->press_l ? 1:0;
        chasis_ctrl.bypass_shoot_heat_control = (key & KEYBOARD_B_BIT) ? 1:0;

        const int16_t mouse_gimbal_control_sensitivity = 20;

        gimbal_ctrl.gimbal_pitch_omega = mouse->y*mouse_gimbal_control_sensitivity;
        gimbal_ctrl.gimbal_yaw_omega = mouse->x*-mouse_gimbal_control_sensitivity;

        gimbal_ctrl.swap_head_tail = swap_head_tail;
        gimbal_ctrl.gimbal_control_mode = mouse->press_r ? 2:1;
}

void autodrive_mode(){
    chasis_ctrl.L5_auto_drive = 1;

    gimbal_ctrl.gimbal_yaw_omega = 0;
    gimbal_ctrl.gimbal_pitch_omega = 0;
    gimbal_ctrl.gimbal_control_mode = 2;
    gimbal_ctrl.swap_head_tail = 0;

    chasis_ctrl.auto_respawn_enabled = 1;
    chasis_ctrl.bypass_shoot_heat_control = 0;
    chasis_ctrl.custom_UI_drawcall = 0;
    chasis_ctrl.fire_pressed = 1;
    chasis_ctrl.spin_mode = 3;
    chasis_ctrl.supercap_discharge = 1;
}

void remote_on_change(){
    // Priority: DR16 > Auto Sentry UART (Only when enabled) > VTM Link
    if(DR16_online()){
        int auto_sentry_mode = (dr16.s1 == DR16_SWITCH_UP);
        
        if(auto_sentry_mode){
            autodrive_mode();
        }else{
            process_keyboard(dr16.key.v, &dr16.key.v_edge_event, &dr16.mouse);
            int use_joystick = (dr16.s1 == DR16_SWITCH_DOWN);
            
            if(use_joystick){
                chasis_ctrl.robot_forward_v = (int16_t)(dr16.channel[1]*2.0f*1e3f);
                chasis_ctrl.robot_leftward_v = (int16_t)(-dr16.channel[0]*2.0f*1e3f);
                chasis_ctrl.robot_yaw_omega = 0;

                chasis_ctrl.fire_pressed = 0;
                chasis_ctrl.spin_mode = 0;
                chasis_ctrl.supercap_discharge = 0;

                gimbal_ctrl.gimbal_pitch_omega = (int16_t)(-dr16.channel[3]*2*PI*1e3f);
                gimbal_ctrl.gimbal_yaw_omega = (int16_t)(-dr16.channel[2]*2*PI*1e3f);
            }
            if(chasis_ctrl.L5_auto_drive){
                chasis_ctrl.spin_mode = 1;
            }
            chasis_ctrl.L5_auto_drive = 0;

            chasis_ctrl.auto_respawn_enabled = 0;
        }

        gimbal_ctrl.flywheel_enabled = (dr16.s1 == DR16_SWITCH_UP || dr16.s1 == DR16_SWITCH_MID);

        control_timeout_update();
    }else if(VTM_online()){
        int auto_sentry_mode = (vtm.mode_sw == VTM_SW_SPORT);

        if(auto_sentry_mode){
            autodrive_mode();
        }else{
            process_keyboard(vtm.key.v, &vtm.key.v_edge_event, &vtm.mouse);

            int use_joystick = (vtm.mode_sw == VTM_SW_CINE);
            
            if(use_joystick){
                chasis_ctrl.robot_forward_v = (int16_t)(vtm.channel[1]*2.0f*1e3f);
                chasis_ctrl.robot_leftward_v = (int16_t)(-vtm.channel[0]*2.0f*1e3f);
                chasis_ctrl.robot_yaw_omega = 0;

                chasis_ctrl.fire_pressed = vtm.buttons.trigger;
                chasis_ctrl.spin_mode = vtm.buttons.s2 ? 3 : 0;
                chasis_ctrl.supercap_discharge = 0;

                gimbal_ctrl.gimbal_pitch_omega = (int16_t)(-vtm.channel[2]*2*PI*1e3f);
                gimbal_ctrl.gimbal_yaw_omega = (int16_t)(-vtm.channel[3]*2*PI*1e3f);
            }
            if(chasis_ctrl.L5_auto_drive){
                chasis_ctrl.spin_mode = 1;
            }
            chasis_ctrl.L5_auto_drive = 0;

            chasis_ctrl.auto_respawn_enabled = 0;
        }

        gimbal_ctrl.flywheel_enabled = 1;
        // gimbal_ctrl.flywheel_enabled = (vtm.mode_sw == VTM_SW_SPORT || vtm.mode_sw == VTM_SW_NORMAL);

        control_timeout_update();
    }

}

void DR16_on_change(){
    remote_on_change();
}

void VTM_on_change(){
    remote_on_change();
}

#endif
