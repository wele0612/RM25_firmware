#include<config.h>
#include<application.h>

#include <motors.h>

#include <h7can.h>
#include <btb.h>
#include <global_variables.h>

#ifdef CONFIG_PLATFORM_BASE

void controller_init(){
    role_controller_init();
    ESTOP_reset();
}

void dr16_on_change(){
    // robot_ctrl_t *geo = &robot_geo;

    // if(gim_state == GIM_IMU && (dr16.mouse.press_l && !dr16.previous.mouse.press_l) && BTB_ONLINE){
    //     if(wrap_to_pi(geo->target_agi_pos - geo->agi_pos) < 20.0f*DEGtoRAD){
    //         geo->target_agi_pos = get_nearest_agi_reset_pos(geo->target_agi_pos + AGI_PER_POS_INCRE);
    //     }
    // }
    #ifdef CONFIG_INPUT_DR16
    #ifdef CONFIG_KEYBOARD_CONTROL
        const int16_t press_move_vel = (int16_t)(1.0f*1e3);
        const int16_t press_rotate_vel = (int16_t)(PI*1e3);
        int16_t accelerate_factor;
        if(dr16.key.v & DR16_KEY_SHIFT_BIT){
            accelerate_factor = 3;
        }else{
            accelerate_factor = 1;
        }

        if(dr16.key.v & DR16_KEY_W_BIT){
            chasis_ctrl.robot_forward_v = accelerate_factor*press_move_vel;
        }else if(dr16.key.v & DR16_KEY_S_BIT){
            chasis_ctrl.robot_forward_v = accelerate_factor*-press_move_vel;
        }else{
            chasis_ctrl.robot_forward_v = 0;
        }

        if(dr16.key.v & DR16_KEY_A_BIT){
            chasis_ctrl.robot_leftward_v = accelerate_factor*press_move_vel;
        }else if(dr16.key.v & DR16_KEY_D_BIT){
            chasis_ctrl.robot_leftward_v = accelerate_factor*-press_move_vel;
        }else{
            chasis_ctrl.robot_leftward_v = 0;
        }

        if(dr16.key.v & DR16_KEY_Q_BIT){
            chasis_ctrl.robot_yaw_omega = accelerate_factor*press_rotate_vel;
        }else if(dr16.key.v & DR16_KEY_E_BIT){
            chasis_ctrl.robot_yaw_omega = accelerate_factor*-press_rotate_vel;
        }else{
            chasis_ctrl.robot_yaw_omega = 0;
        }
        int swap_head_tail = 0;

        chasis_ctrl.fire_pressed = dr16.mouse.press_l;
        chasis_ctrl.spintop_level = 0;
        chasis_ctrl.supercap_discharge = 0;
        chasis_ctrl.swap_head_tail = swap_head_tail;

        const int16_t mouse_gimbal_control_sensitivity = 20;

        gimbal_ctrl.gimbal_mouse_pitch_omega = dr16.mouse.y*mouse_gimbal_control_sensitivity;
        gimbal_ctrl.gimbal_mouse_yaw_omega = dr16.mouse.x*-mouse_gimbal_control_sensitivity;

        gimbal_ctrl.swap_head_tail = swap_head_tail;
        gimbal_ctrl.gimbal_control_mode = dr16.mouse.press_r ? 2:1;
        gimbal_ctrl.flywheel_enabled = 1;
        
    #endif
    #ifdef CONFIG_JOYSTICK_CONTROL
        chasis_ctrl.robot_forward_v = (int16_t)(dr16.channel[0]*2.0f*1e3f);
        chasis_ctrl.robot_leftward_v = (int16_t)(-dr16.channel[1]*2.0f*1e3f);
        chasis_ctrl.robot_yaw_omega = (int16_t)(-dr16.channel[2]*PI*1e3f);

        int swap_head_tail = 0;

        chasis_ctrl.fire_pressed = 0;
        chasis_ctrl.spintop_level = 0;
        chasis_ctrl.supercap_discharge = 0;
        chasis_ctrl.swap_head_tail = swap_head_tail;

        gimbal_ctrl.gimbal_mouse_pitch_omega = 0;
        gimbal_ctrl.gimbal_mouse_yaw_omega = 0;
        gimbal_ctrl.swap_head_tail = swap_head_tail;
        gimbal_ctrl.gimbal_control_mode = 1;
        gimbal_ctrl.flywheel_enabled = (dr16.s1 == DR16_SWITCH_UP);
    #endif

    #endif

    // add this in referee update
    // gimbal_ctrl.feedback_shoot_speed = (int16_t)(referee.shoot_data_0x0207.initial_speed*1e3f);

    return;
}

void controller_cycle(const float CTRL_DELTA_T){
    role_controller_step(CTRL_DELTA_T);

    // ESTOP if remote is offline
    if(remote_online()){
        ESTOP_reset();
    }else{
        ESTOP();
    }

    #ifdef CONFIG_INPUT_VTM_LINK
        gimbal_ctrl.gimbal_use_VTM_not_dr16 = 1;
    #else
        gimbal_ctrl.gimbal_use_VTM_not_dr16 = 0;
    #endif

    // Upload gimbal control commands
    memcpy(&b2g_A.gimbal_ctrl, &gimbal_ctrl, sizeof(gimbal_ctrl_input_t));
    fdcanx_send_data(&hfdcan1, G2B_MSG_A_ID, (uint8_t *)&b2g_A, 8);

}

#endif

