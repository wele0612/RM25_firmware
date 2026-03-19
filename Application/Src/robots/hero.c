#include<config.h>

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<vision.h>

#include<motors.h>
#include<servo_pwm.h>
#include<btb.h>
#include<h7can.h>

#ifdef CONFIG_ROBOT_HERO

enum {
    GIM_STANDBY = 0,
    GIM_IMU_PREP_FOLDRISE = 1,
    GIM_IMU_PREP_GIMDOWN = 2,
    GIM_IMU = 3,
    GIM_FOLD_PREP = 4,
    GIM_FOLD = 5,
    GIM_SNIPER = 6, // 吊射模式
}gim_state;

enum {
    LAUNCH_INIT = 0, // 初始模式
    LAUNCH_IMU = 1, // 近战模式，积累位置环
    LAUNCH_SNP_FEEDTURN = 2, // 吊射模式阶段一，预置转
    LAUNCH_SNP_AGITURN = 3, // 吊射模式阶段二，拨弹转
}launch_state;

const float FOLD_MODE_PITCH_MOTOR_RAD = 129.0f*DEGtoRAD;

#ifdef CONFIG_PLATFORM_BASE

const float AGI_PER_POS_INCRE = 2*PI/6.0f;
uint32_t base_startup_time = 0;

// 求最近的拨盘起始位置
static inline float get_nearest_agi_reset_pos(float current_pos){
    const float INVERSE_THRE = 0.3f;
    float pos = wrap_to_pi(floorf(current_pos * (1.0f/AGI_PER_POS_INCRE))*AGI_PER_POS_INCRE);
    float pos_diff = wrap_to_pi(current_pos - pos);
    if(pos_diff > INVERSE_THRE){
        pos += AGI_PER_POS_INCRE;
    }
    return pos;
}

static void inline init_agi_damiao(){
    fdcanx_send_data(&hfdcan3, AGI_CTRLID, enable_DM_Joint(motors.agi.tranmitbuf), 8);
}

void role_controller_init(){
    gim_state = GIM_STANDBY;

    HAL_Delay(1500);
    fdcanx_send_data(&hfdcan3, YAW_CTRLID, enable_DM_Joint(motors.yaw.tranmitbuf), 8);
    HAL_Delay(2);
    init_agi_damiao();

    base_startup_time = HAL_GetTick();

}

void dr16_on_change(){
    robot_ctrl_t *geo = &robot_geo;

    if(gim_state == GIM_IMU && (dr16.mouse.press_l && !dr16.previous.mouse.press_l) && BTB_ONLINE){
        if(wrap_to_pi(geo->target_agi_pos - geo->agi_pos) < 20.0f*DEGtoRAD){
            geo->target_agi_pos = get_nearest_agi_reset_pos(geo->target_agi_pos + AGI_PER_POS_INCRE);
        }
    }

    float pitch_incr = 0.0f, yaw_incr=0.0f;

    if(!(dr16.key.v & DR16_KEY_B_BIT)){

        if((dr16.key.v & DR16_KEY_W_BIT) && !(dr16.previous.key.v & DR16_KEY_W_BIT)){
            pitch_incr = 0.3f*DEGtoRAD;
        }else if((dr16.key.v & DR16_KEY_S_BIT) && !(dr16.previous.key.v & DR16_KEY_S_BIT)){
            pitch_incr = -0.3f*DEGtoRAD;
        }else{
            pitch_incr = 0.0f;
        }

        if((dr16.key.v & DR16_KEY_D_BIT) && !(dr16.previous.key.v & DR16_KEY_D_BIT)){
            yaw_incr = -0.3f*DEGtoRAD;
        }else if((dr16.key.v & DR16_KEY_A_BIT) && !(dr16.previous.key.v & DR16_KEY_A_BIT)){
            yaw_incr = 0.3f*DEGtoRAD;
        }else{
            yaw_incr = 0.0f*DEGtoRAD;
        }

        if((dr16.key.v & DR16_KEY_SHIFT_BIT)){
            yaw_incr *= 10.0f;
            pitch_incr *= 10.0f;
        }else if((dr16.key.v & DR16_KEY_CTRL_BIT)){
            yaw_incr *= 0.2f;
            pitch_incr *= 0.2f;
        }
    }

    geo->target_yaw_pos = wrap_to_pi(geo->target_yaw_pos + yaw_incr);
    geo->target_sni_pitch_pos += pitch_incr;

    if(geo->target_sni_pitch_pos > 80.0f*DEGtoRAD){
        geo->target_sni_pitch_pos = 80.0f*DEGtoRAD;
    }else if(geo->target_sni_pitch_pos < 20.0f*DEGtoRAD){
        geo->target_sni_pitch_pos = 20.0f*DEGtoRAD;
    }
    return;
}

PID_t body_x_vel_pid={
    .P = 12.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t body_y_vel_pid={
    .P = 12.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t body_yaw_vel_pid={
    .P = 35.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t body_yaw_pos_pid={
    .P = 1.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t g_gimbal_yaw_vel_pid={
    .P = 3.5f,
    .I = 18.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t g_gimbal_yaw_pos_pid={
    .P = 12.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.005f
};

PID_t f_gimbal_yaw_vel_pid={
    .P = 1.5f,
    .I = 20.0f,
    .D = 0.0f,
    .integral_max = 0.15f
};

PID_t f_gimbal_yaw_pos_pid={
    .P = 3.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.005f
};


PID_t agi_vel_pid={
    .P = 5.0f,
    .I = 60.0f,
    .D = 0.0f,
    .integral_max = 0.05f
};

PID_t agi_pos_pid={
    .P = 16.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

const float gimbal_standby_position = 68.4f*DEGtoRAD;

/* Usually, during folding stage, Yaw should turn to a specific position relative to body.
   However, gimbal could enter a folding stage just after power-up. 
   In this case, Yaw should not turn (even in a folding stage!) until gimbal enter a stable mode.
*/
int enable_folding_fixed_gimbal = 0;

void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();

    // float accelerate_factor = 1.0f;
    // // if(dr16.key.v & DR16_KEY_SHIFT_BIT){
    // //     accelerate_factor = 2.5f;
    // // }

    // float pitch_incr;
    // float yaw_incr;

    // if(dr16.key.v & DR16_KEY_B_BIT){
    //     if(dr16.key.v & DR16_KEY_W_BIT){
    //         geo->target_vy = accelerate_factor*1.0f;
    //     }else if(dr16.key.v & DR16_KEY_S_BIT){
    //         geo->target_vy = accelerate_factor*-1.0f;
    //     }else{
    //         geo->target_vy = 0.0f;
    //     }

    //     if(dr16.key.v & DR16_KEY_D_BIT){
    //         geo->target_vx = accelerate_factor*1.0f;
    //     }else if(dr16.key.v & DR16_KEY_A_BIT){
    //         geo->target_vx = accelerate_factor*-1.0f;
    //     }else{
    //         geo->target_vx = 0.0f;
    //     }

    //     if(dr16.key.v & DR16_KEY_Q_BIT){
    //         geo->target_vyaw = accelerate_factor*0.5f;
    //     }else if(dr16.key.v & DR16_KEY_E_BIT){
    //         geo->target_vyaw = accelerate_factor*-0.5f;
    //     }else{
    //         geo->target_vyaw = 0.0f;
    //     }
    // }else{
    //     geo->target_vy = 0.0f;
    //     geo->target_vx = 0.0f;
    //     geo->target_vyaw = 0.0f;
    // }

    

    // // DR16_KEY_SHIFT_BIT

    // int enable_auto_aim = dr16.mouse.press_r;

    // // geo->target_vx = dr16.channel[0]*5.0f;
    // // geo->target_vy = dr16.channel[1]*5.0f;
    // // geo->target_vyaw = -dr16.channel[2]*8.0f;

    // // geo->input_yaw_vel = dr16.channel[2]*1.0f;
    // // geo->input_pitch_vel = dr16.channel[3]*1.0f;
    // const float input_mouse_alpha = 0.02f;

    // float control_sensitivity;
    // if(gim_state == GIM_SNIPER){
    //     control_sensitivity = 0.005f;
    // }else{
    //     control_sensitivity = 0.02f;
    // }
    // // geo->input_pitch_vel = geo->input_pitch_vel*(1.0f-input_mouse_alpha) + dr16.mouse.y*control_sensitivity*input_mouse_alpha;
    // // geo->input_yaw_vel = geo->input_yaw_vel*(1.0f-input_mouse_alpha) + -dr16.mouse.x*control_sensitivity*input_mouse_alpha;

    // geo->input_pitch_vel*=0.5f;

    // b2g_B.flywheel_enabled = (dr16.s2 == DR16_SWITCH_UP);

    // // geo->target_yaw_vel = dr16.channel[3]*1.5f;
    
    // // if(dr16.channel[3] > 0.3f){
    // //     geo->target_yaw_vel = 1.5f;
    // //     // geo->target_yaw_pos = 1.0f;
    // // }else if(dr16.channel[3] < -0.3f){
    // //     // geo->target_yaw_vel = -1.5f;
    // //     geo->target_yaw_pos = -1.0f;
    // // }else{
    // //     // geo->target_yaw_vel = 0.0f;
    // //     geo->target_yaw_pos = 0.0f;
    // // }

    // /* Agitator control */

    // const int feeder_in_pos = g2b_B.feeder_in_place;

    // switch (launch_state){
    // case LAUNCH_INIT:
    //     geo->target_agi_pos = get_nearest_agi_reset_pos(geo->agi_pos);
    //     if(gim_state == GIM_IMU && BTB_ONLINE){
    //         launch_state = LAUNCH_IMU;
    //     }else if(dr16.mouse.press_l && BTB_ONLINE){
    //         launch_state = LAUNCH_SNP_FEEDTURN;
    //     }
    //     break;
    // case LAUNCH_IMU:
    //     if(gim_state == GIM_SNIPER){
    //         launch_state = LAUNCH_INIT;
    //     }

        
    //     break;
    // case LAUNCH_SNP_FEEDTURN:
    //     if(!BTB_ONLINE){
    //         launch_state = LAUNCH_INIT;
    //     }else if(feeder_in_pos){ // 主动预置到位，拨盘开始转
    //         geo->target_agi_pos = get_nearest_agi_reset_pos(geo->agi_pos + AGI_PER_POS_INCRE);
    //         launch_state = LAUNCH_SNP_AGITURN;
    //     }
    //     break;
    // case LAUNCH_SNP_AGITURN:
    //     if(fabs(wrap_to_pi(geo->target_agi_pos - geo->agi_pos)) < 5.0f*DEGtoRAD){
    //         launch_state = LAUNCH_INIT; // 拨盘转到位了，发射时序结束。
    //     }
    //     break;
    // default:
    //     break;
    // }
    // // launch_state == LAUNCH_IMU ||
    // if(launch_state == LAUNCH_SNP_FEEDTURN){
    //     b2g_B.feeder_push = 1;
    // }else if(wrap_to_pi(geo->target_agi_pos - geo->agi_pos) > 3.0f*DEGtoRAD && gim_state == GIM_IMU){
    //     b2g_B.feeder_push = 1;
    // }else{
    //     b2g_B.feeder_push = 0;
    // }

    // geo->agi_pos = wrap_to_pi(fmotor.agi.position);
    // geo->target_agi_vel = pid_cycle(&agi_pos_pid, wrap_to_pi(geo->target_agi_pos - geo->agi_pos), CTRL_DELTA_T);
    // geo->target_agi_vel = limit_val(geo->target_agi_vel, 10.0f);
    // geo->agi_vel = fmotor.agi.speed * 0.25f;
    // geo->T_agi = pid_cycle(&agi_vel_pid, geo->target_agi_vel-geo->agi_vel, CTRL_DELTA_T);

    // /* Gimbal control */

    // const float gimbal_folding_position_error = wrap_to_pi(gimbal_standby_position - geo->gimbal_mtr_yaw_pos);

    // geo->gimbal_mtr_pitch_pos = (float)(g2b_A.gimbal_pitch)*1E-4f;
    // geo->gimbal_mtr_fold_pos = (float)(g2b_A.gimbal_fold)*1E-4f;
    // const float dm_motor_alpha = 0.1f;
    // geo->gimbal_mtr_yaw_vel = dm_motor_alpha*fmotor.yaw.speed + (1.0f-dm_motor_alpha)*geo->gimbal_mtr_yaw_vel;
    // geo->gimbal_mtr_yaw_pos = wrap_to_pi(fmotor.yaw.position);

    // geo->gimbal_abs_yaw_vel = g2b_B.gimbal_yaw_vel_imu;
    // const float yaw_drift_threshold = 0.01f;
    // if(gim_state == GIM_SNIPER && BTB_ONLINE){
        
    //     if(fabsf(geo->gimbal_abs_yaw_vel) < yaw_drift_threshold){
    //         geo->gimbal_abs_yaw_vel = 0.0f;
    //     }

    //     geo->gimbal_abs_yaw_pos += geo->gimbal_abs_yaw_vel*CTRL_DELTA_T;
    // }else{
    //     geo->gimbal_abs_yaw_pos = g2b_A.gimbal_yaw_pos_imu;
    // }

    // // State-transition functions

    // const int is_pitch_up_maximum = 
    //     (fabsf(FOLD_MODE_PITCH_MOTOR_RAD - geo->gimbal_mtr_pitch_pos) < 3.0f*DEGtoRAD);

    // const int is_pitch_down = 
    //     (fabsf(geo->gimbal_mtr_pitch_pos) < 5.0f*DEGtoRAD);

    // const int is_fold_up = 
    //     (fabsf(geo->gimbal_mtr_fold_pos) < 5.0f*DEGtoRAD);

    // b2g_B.gimbal_mode = gim_state;
    // switch(gim_state){
    // case GIM_IMU:
    //     if(dr16.s1 == DR16_SWITCH_UP){ 
    //         gim_state = GIM_FOLD_PREP; // 折叠指令被触发， 准备折叠
    //     }else if(dr16.s1 == DR16_SWITCH_DOWN){
    //         geo->target_yaw_pos = 0.0f;
    //         gim_state = GIM_SNIPER;    // 进吊射模式
    //     }
    //     enable_folding_fixed_gimbal = 1;

    //     break;
    // case GIM_FOLD_PREP:
    //     if(dr16.s1 != DR16_SWITCH_UP){
    //         gim_state = GIM_IMU_PREP_FOLDRISE; // 折叠指令被取消
    //     }
    //     const float yaw_ready_err_max = 2.0f*DEGtoRAD;
    //     if(fabsf(gimbal_folding_position_error) < yaw_ready_err_max && is_pitch_up_maximum){
    //         gim_state = GIM_FOLD; // Yaw轴对准且云台上升到位，开始折叠
    //     }
    //     break;
    // case GIM_FOLD:
    //     if(dr16.s1 != DR16_SWITCH_UP){
    //         gim_state = GIM_IMU_PREP_FOLDRISE; // 折叠指令被取消
    //     }
    //     break;
    // case GIM_IMU_PREP_FOLDRISE:
    //     if(is_fold_up){
    //         gim_state = GIM_IMU_PREP_GIMDOWN; // 云台上升到位，切换回IMU模式
    //     }
    //     break;
    // case GIM_IMU_PREP_GIMDOWN:
    //     if(is_pitch_down){
    //         geo->target_yaw_pos = 0.0f;
    //         gim_state = GIM_IMU; // 云台上升到位，切换回IMU模式
    //     }else if(dr16.s1 == DR16_SWITCH_DOWN){
    //         gim_state = GIM_SNIPER;    // 进吊射模式
    //     }
    //     break;
    // case GIM_SNIPER:
    //     enable_folding_fixed_gimbal = 1;
    //     if(dr16.s1 != DR16_SWITCH_DOWN){ // 退出吊射模式
    //         gim_state = GIM_IMU_PREP_GIMDOWN;
    //     }
    //     break;
    // case GIM_STANDBY:
    //     if(HAL_GetTick()-base_startup_time < 100){
    //         break;
    //     }
    //     if(BTB_ONLINE && is_fold_up && is_pitch_down){
    //         geo->target_yaw_pos = 0.0f;
    //         gim_state = GIM_IMU;
    //     }else if(BTB_ONLINE && is_fold_up){
    //         gim_state = GIM_IMU_PREP_GIMDOWN;
    //     }else if(BTB_ONLINE){
    //         gim_state = GIM_IMU_PREP_FOLDRISE;
    //     }
    //     break;
    // default:
    //     gim_state=GIM_STANDBY;
    //     break;
    // }

    // if(!enable_folding_fixed_gimbal){
    //     geo->T_yaw = 0.0f;
    // }else if(BTB_ONLINE && (gim_state == GIM_IMU || gim_state == GIM_SNIPER)){
    //     if(enable_auto_aim && g2b_C.vision_tracked){
    //         float aim_target_yaw_pos = g2b_C.aim_yaw_pos*1E-4;
    //         float aim_target_yaw_vel = g2b_C.aim_yaw_vel*1E-4;

    //         geo->target_yaw_vel = aim_target_yaw_vel + pid_cycle(&g_gimbal_yaw_pos_pid, wrap_to_pi(aim_target_yaw_pos - geo->gimbal_abs_yaw_pos), CTRL_DELTA_T);

    //         float gimbal_yaw_pid_coe = 0.2f + fabsf(cosf(geo->gimbal_mtr_pitch_pos))*(0.8f);
    //         geo->T_yaw = gimbal_yaw_pid_coe * pid_cycle(&g_gimbal_yaw_vel_pid, geo->target_yaw_vel - geo->gimbal_abs_yaw_vel, CTRL_DELTA_T);

    //         geo->target_yaw_pos = geo->gimbal_abs_yaw_pos;
    //     }else{
    //         // geo->target_yaw_pos = wrap_to_pi(geo->target_yaw_pos + geo->input_yaw_vel*CTRL_DELTA_T);
            
    //         // geo->target_yaw_vel = geo->input_yaw_vel + limit_val(pid_cycle(&g_gimbal_yaw_pos_pid, wrap_to_pi(geo->target_yaw_pos - geo->gimbal_abs_yaw_pos), CTRL_DELTA_T), 5.0f);

    //         geo->target_yaw_vel = geo->input_yaw_vel + limit_val(pid_cycle(&f_gimbal_yaw_pos_pid, wrap_to_pi(geo->target_yaw_pos - geo->gimbal_mtr_yaw_pos), CTRL_DELTA_T), 5.0f);


    //         if(gim_state == GIM_SNIPER){
    //             geo->target_yaw_vel = limit_val(geo->target_yaw_vel, 1.0f);
    //             if(fabsf(geo->target_yaw_vel) < yaw_drift_threshold){
    //                 geo->target_yaw_vel = 0.0f;
    //             }
    //         }

    //         float gimbal_yaw_pid_coe = 0.2f + fabsf(cosf(geo->gimbal_mtr_pitch_pos))*(0.8f);
    //         // geo->T_yaw = gimbal_yaw_pid_coe * pid_cycle(&g_gimbal_yaw_vel_pid, geo->target_yaw_vel - geo->gimbal_abs_yaw_vel, CTRL_DELTA_T);
    //         geo->T_yaw = gimbal_yaw_pid_coe * pid_cycle(&f_gimbal_yaw_vel_pid, geo->target_yaw_vel - geo->gimbal_mtr_yaw_vel, CTRL_DELTA_T);
        
        
    //     }
        
    //     }else if(BTB_ONLINE && (gim_state != GIM_STANDBY)){

    //     geo->target_yaw_vel = pid_cycle(&f_gimbal_yaw_pos_pid, gimbal_folding_position_error, CTRL_DELTA_T);
    //     geo->target_yaw_vel = limit_val(geo->target_yaw_vel, 2.0f);
    //     geo->T_yaw = pid_cycle(&f_gimbal_yaw_vel_pid, geo->target_yaw_vel - geo->gimbal_mtr_yaw_vel, CTRL_DELTA_T);
    // }else{
    //     geo->T_yaw = 0.0f;
    // }

    // /* Base control */

    // const float WHEEL_RADIUS = 0.153f/2.0f;
    // const float BODY_RADIUS = 0.6f/2.0f;

    // const float VEL_RPM_COE = RPMtoRADS*M3508_CUSTOM_GB_RATIO*WHEEL_RADIUS;
    // float vel_LF = fmotor.wheel_LF.speed*VEL_RPM_COE;
    // float vel_LB = fmotor.wheel_LB.speed*VEL_RPM_COE;
    // float vel_RF = fmotor.wheel_RF.speed*VEL_RPM_COE;
    // float vel_RB = fmotor.wheel_RB.speed*VEL_RPM_COE;

    // geo->vyaw_gyro = -imu_data.gyro[2]*DEGtoRAD;
    
    // /* Forward kinetics of the base */
    // float vx_b = -(vel_LF + vel_RF - vel_LB - vel_RB)*(1.4142136f/4.0f);
    // float vy_b = -(vel_LF - vel_RF + vel_LB - vel_RB)*(1.4142136f/4.0f);
    // float vyaw_wheel = (vel_LF + vel_LB + vel_RF + vel_RB)*(0.25f/BODY_RADIUS);

    // const float wheel_v_alpha = 0.2f;
    // geo->vx_b = vx_b*wheel_v_alpha + geo->vx_b*(1.0f-wheel_v_alpha);
    // geo->vy_b = vy_b*wheel_v_alpha + geo->vy_b*(1.0f-wheel_v_alpha);
    // geo->vyaw_wheel = vyaw_wheel*wheel_v_alpha + geo->vyaw_wheel*(1.0f-wheel_v_alpha);

    // float body_yaw_offset;
    // if(gim_state == GIM_IMU || gim_state == GIM_STANDBY){
    //     body_yaw_offset = -geo->gimbal_mtr_yaw_pos;
    // }else{
    //     body_yaw_offset = 0.0f;
    // }

    // const float cosby = cosf(body_yaw_offset);
    // const float sinby = sinf(body_yaw_offset);

    // // Convert body coordinate system to gimbal coordinate system.
    // geo->vx = geo->vx_b * cosby + geo->vy_b * -sinby;
    // geo->vy = geo->vx_b * sinby + geo->vy_b * cosby;

    // geo->F_x = pid_cycle(&body_x_vel_pid, geo->target_vx - geo->vx, CTRL_DELTA_T);
    // geo->F_y = pid_cycle(&body_y_vel_pid, geo->target_vy - geo->vy, CTRL_DELTA_T);
    // geo->T_base_yaw = pid_cycle(&body_yaw_vel_pid, geo->target_vyaw - geo->vyaw_wheel, CTRL_DELTA_T);
    
    // geo->F_x = limit_val(geo->F_x, 9.0f);
    // geo->F_y = limit_val(geo->F_y, 9.0f);
    // geo->T_base_yaw = limit_val(geo->T_base_yaw, 17.0f);
    
    // // geo->F_x = geo->target_vx*0.2f;
    // // geo->F_y = geo->target_vy*0.2f;
    // // geo->T_base_yaw = geo->target_vyaw*0.5f;

    // geo->F_x_b = geo->F_x * cosby + geo->F_y * sinby;
    // geo->F_y_b = geo->F_x * -sinby + geo->F_y * cosby;

    // float F_single_wheel_turn = -geo->T_base_yaw*(0.25f/BODY_RADIUS);
    // float T_single_wheel_turn = F_single_wheel_turn*WHEEL_RADIUS;
    // float force_vector_coe = 1.4142136f*0.25f;
    // geo->T_LF =  (geo->F_y_b + geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    // geo->T_LB =  (geo->F_y_b - geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    // geo->T_RF = (-geo->F_y_b + geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    // geo->T_RB = (-geo->F_y_b - geo->F_x_b)*force_vector_coe + T_single_wheel_turn;

    // uint8_t tx_buf[8];
    // // fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(tx_buf,
    // //     geo->T_LF*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
    // //     geo->T_LB*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
    // //     geo->T_RF*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
    // //     geo->T_RB*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB)), 8);

    // geo->T_yaw=0.0f;
    // fdcanx_send_data(&hfdcan3, YAW_CTRLID, set_torque_DM4310(motors.yaw.tranmitbuf, geo->T_yaw), 8);

    // geo->T_agi=0.0f;
    // fdcanx_send_data(&hfdcan3, AGI_CTRLID, set_torque_DM4310(motors.agi.tranmitbuf, geo->T_agi), 8);

    b2g_A.target_mtr_fold_vel = dr16.channel[1];
    b2g_A.target_mtr_pitch_vel = dr16.channel[3];
    fdcanx_send_data(&hfdcan1, B2G_MSG_A_ID, (uint8_t *)&b2g_A, 8);

    // geo->input_pitch_vel = limit_val(geo->input_pitch_vel, 6.0f);
    // b2g_B.target_pitch_vel = (int16_t)(geo->input_pitch_vel*(1/3E-4f));
    // b2g_B.sni_target_pitch = geo->target_sni_pitch_pos;
    // b2g_B.aim_enabled = enable_auto_aim;
    // fdcanx_send_data(&hfdcan1, B2G_MSG_B_ID, (uint8_t *)&b2g_B, 8);

    vofa.val[0]=geo->F_x;
    vofa.val[1]=geo->F_y;
    vofa.val[2]=geo->T_base_yaw;

    vofa.val[3]=geo->gimbal_abs_yaw_vel;
    vofa.val[4]=geo->gimbal_abs_yaw_pos;
    vofa.val[5]=fmotor.yaw.position;
    vofa.val[6]=gim_state;

    // vofa.val[7]=geo->target_agi_pos;
    // vofa.val[8]=geo->agi_vel;
    // vofa.val[9]=fmotor.agi.position;

    vofa.val[7]=geo->target_yaw_pos;
    vofa.val[8]=geo->target_sni_pitch_pos;
    vofa.val[9]=fabsf(geo->target_agi_pos - geo->agi_pos);
}


void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x201:
        parse_feedback_M3508(msg, &motors.wheel_LF);
        break;
    case 0x202:
        parse_feedback_M3508(msg, &motors.wheel_LB);
        break;
    case 0x203:
        parse_feedback_M3508(msg, &motors.wheel_RF);
        break;
    case 0x204:
        parse_feedback_M3508(msg, &motors.wheel_RB);
        break;
    case YAW_FEEDBACKID:
        parse_feedback_DM4310(msg, &motors.yaw, YAW_CTRLID);
        break;
    case AGI_FEEDBACKID:
        parse_feedback_DM4310(msg, &motors.agi, AGI_CTRLID);
        break;
    // case G2B_MSG_A_ID:
    //     memcpy(&g2b_A, msg, 8);
    //     BTB_UPDATE_CNTDOWN();
    //     break;
    // case G2B_MSG_B_ID:
    //     memcpy(&g2b_B, msg, 8);
    //     BTB_UPDATE_CNTDOWN();
    //     break;
    // case G2B_MSG_C_ID:
    //     memcpy(&g2b_C, msg, 8);
    //     BTB_UPDATE_CNTDOWN();
    //     break;
    default:
        break;
    }
    return;
}

#endif 
#ifdef CONFIG_PLATFORM_GIMBAL

PID_t flywheel_1_pid={
    .P=0.007f,
    .I=0.05f,
    .D=0.000005f,
    .integral_max=20.0f
};

PID_t flywheel_2_pid={
    .P=0.007f,
    .I=0.05f,
    .D=0.000005f,
    .integral_max=20.0f
};

PID_t flywheel_3_pid={
    .P=0.007f,
    .I=0.05f,
    .D=0.000005f,
    .integral_max=20.0f
};

PID_t feeder_1_vel_pid={
    .P=0.1f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
};

// ------------ For gimbal mode ------------
// PID_t g_pitch_vel_pid={
//     .P=1.0f,
//     .I=70.0f,
//     .D=0.0f,
//     .integral_max=0.2f
// }; 

PID_t g_pitch_pos_pid={
    .P=12.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

// ------------ For fold mode ------------
// PID_t f_pitch_vel_pid={
//     .P=0.1f,
//     .I=0.0f,
//     .D=0.0f,
//     .integral_max=0.01f
// };

PID_t f_pitch_pos_pid={
    .P=3.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
}; 

// PID_t fold_vel_pid={
//     .P=0.1f,
//     .I=0.0f,
//     .D=0.0f,
//     .integral_max=0.01f
// };

// PID_t fold_pos_pid={
//     .P=0.1f,
//     .I=0.0f,
//     .D=0.0f,
//     .integral_max=0.01f
// };

// uint8_t reset_zeropoint[8] = {0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void role_controller_init(){

}

float fold_mtrpos = -5.0f, pitch_mtrpos = -30.0f;

void dr16_on_change(){
    // robot_ctrl_t *geo = &robot_geo;

    // if(dr16.s2 == DR16_SWITCH_MID && dr16.previous.s2 == DR16_SWITCH_DOWN){
    //     geo->feeder_position = 0.0f;
    //     geo->target_feeder_position = 1.7f;
    // }
    
}

#define FOLDING_LOCK()    do{servo_SetAngle(0.0f);}while(0)
#define FOLDING_UNLOCK()  do{servo_SetAngle(60.0f);}while(0)

/* For X4-36, need to first set VEL_P to 0.015, VEL_I to 0.00005 */
/* Also need to increase X4-36 max acceleration */
void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();
    
    // if(BTB_ONLINE){
    //     if(b2g_B.feeder_push){
    //         if(gim_state == GIM_IMU){
    //             geo->target_feeder_vel = 24.0f;
    //         }else{
    //             geo->target_feeder_vel = 3.14f;
    //         }
            
    //     }else{
    //         geo->target_feeder_vel = 2.0f;
    //     }
    // }else{
    //     geo->target_feeder_vel = 0.0f;
    // }

    // if(BTB_ONLINE){
    //     if(b2g_B.flywheel_enabled){
    //         // if(gim_state == GIM_SNIPER){
    //         //     // geo->target_flywheel_rpm = 3750.0f;
    //         //     geo->target_flywheel_rpm = 3000.0f;
    //         // }else{
    //         //     geo->target_flywheel_rpm = 4000.0f;
    //         // }
    //         geo->target_flywheel_rpm = 4025.0f;
            
    //     }else{
    //         geo->target_flywheel_rpm = 1000.0f;
    //     }
    // }else{
    //     geo->target_flywheel_rpm = 60.0f;
    // }

    // geo->yaw_vel_imu = wrap_to_pi(imu_data.yaw - geo->yaw_m1)*(1.0f/CTRL_DELTA_T);
    // geo->yaw_m1 = imu_data.yaw;

    // /* Note on Pitch Motor Zero point: 
    // * Motor zero for both motor => gimbal locked + minimum pitch
    // */
    // const float PITCH_MOTOR_LOCKED_PITCH_MINIMUM = -9.5f*DEGtoRAD;
    // const float FOLD_MOTOR_LOCKED_BENDING_BACKWARD = -20.0f*DEGtoRAD;
    
    // geo->abs_pitch_vel = -imu_data.gyro[1]*DEGtoRAD;
    // geo->abs_pitch_pos = -imu_data.pitch;

    // geo->mtr_fold_pos = fmotor.fold.precise_position;

    // const float myact_motor_alpha = 1.0f;

    // geo->mtr_pitch_pos = -fmotor.pitch.precise_position + PITCH_MOTOR_LOCKED_PITCH_MINIMUM;
    // geo->mtr_pitch_vel = myact_motor_alpha*(-fmotor.pitch.speed)
    //                     + geo->mtr_pitch_vel*(1.0f-myact_motor_alpha);

    // if(BTB_ONLINE && b2g_B.target_pitch_vel > 0.3f){
    //     geo->target_pitch_pos = 0.7f;
    //     // geo->target_pitch_vel = 0.5f;
    // }else if(BTB_ONLINE && b2g_B.target_pitch_vel < -0.3f){
    //     geo->target_pitch_pos = 0.0f;
    //     // geo->target_pitch_vel = -0.5f;
    // }else{
    //     geo->target_pitch_pos = 0.3f;
    //     // geo->target_pitch_vel = 0.0f;
    // }

    // geo->gimbal_abs_yaw_pos = g2b_A.gimbal_yaw_pos_imu;
    // geo->gimbal_abs_yaw_vel = g2b_B.gimbal_yaw_vel_imu;

    // geo->input_pitch_vel = b2g_B.target_pitch_vel*3E-4;

    // float predict_yaw, predict_pitch, predict_vyaw, predict_vpitch, predict_distence;
    // int vision_tracked = vision_get_armorplate(&predict_yaw, &predict_pitch, 
    //     &predict_vyaw, &predict_vpitch, &predict_distence, 10.0f);

    // uint8_t gimbal_mode = b2g_B.gimbal_mode;

    // switch (gimbal_mode){
    //     case GIM_IMU:
    //         geo->target_pitch_pos += geo->input_pitch_vel*CTRL_DELTA_T;

    //         const float MAX_PITCH_ANGLE=50.0f*DEGtoRAD;
    //         const float MIN_PITCH_ANGLE=PITCH_MOTOR_LOCKED_PITCH_MINIMUM+1.5f*DEGtoRAD;

    //         float target_pitch_pos, feedforward_pitch_vel;
    //         if(BTB_ONLINE && b2g_B.aim_enabled && vision_tracked){
                
    //             // 公式: y = -0.002085x³ + 0.029508x² - 0.111906x + 0.223759
    //             const float coefficients[] = {
    //                 -0.002085f,   // x³ 系数 a
    //                 0.029508f,   // x² 系数 b  
    //                 -0.111906f,   // x  系数 c
    //                 0.223759f    // 常数项 d
    //             };
    //             const float pitch_diff = third_order_fit(coefficients, predict_distence);
                
    //             target_pitch_pos = predict_pitch + pitch_diff;
    //             feedforward_pitch_vel = 0.0f;

    //             geo->target_pitch_pos = geo->abs_pitch_pos;

    //             // 禁用target_pitch_pos = geo->target_pitch_pos;
    //             // feedforward_pitch_vel = geo->input_pitch_vel;
    //         }else{
    //             target_pitch_pos = geo->target_pitch_pos;
    //             feedforward_pitch_vel = geo->input_pitch_vel;
    //         }
            
    //         if(target_pitch_pos > MAX_PITCH_ANGLE){
    //             target_pitch_pos = MAX_PITCH_ANGLE;
    //             geo->target_pitch_vel = pid_cycle(&g_pitch_pos_pid, target_pitch_pos - geo->abs_pitch_pos, CTRL_DELTA_T);
    //         }else if(target_pitch_pos < MIN_PITCH_ANGLE){
    //             target_pitch_pos = MIN_PITCH_ANGLE;
    //             geo->target_pitch_vel = pid_cycle(&g_pitch_pos_pid, target_pitch_pos - geo->abs_pitch_pos, CTRL_DELTA_T);
    //         }else{
    //             geo->target_pitch_vel = feedforward_pitch_vel + 
    //             pid_cycle(&g_pitch_pos_pid, target_pitch_pos - geo->abs_pitch_pos, CTRL_DELTA_T);
    //         }

    //         // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
    //         //     set_speed_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_vel*RADtoDEG, 0), 8);

    //         geo->target_pitch_pos = 20.0f*DEGtoRAD;
    //         fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5,
    //             set_position_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_pos*RADtoDEG, 20.0f), 8);
            
    //         break;

    //     case GIM_SNIPER:
    //         if(BTB_ONLINE){
    //             geo->target_pitch_pos = b2g_B.sni_target_pitch;
    //         }else{
    //             geo->target_pitch_pos = 50.0f*DEGtoRAD;
    //         }
            
    //         fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5,
    //             set_position_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_pos*RADtoDEG, 20.0f), 8);
            
    //         break;

    //     case GIM_IMU_PREP_GIMDOWN:
    //         geo->target_pitch_pos = geo->abs_pitch_pos;

    //         geo->target_pitch_vel = pid_cycle(&f_pitch_pos_pid, 
    //             2.0f*DEGtoRAD - geo->mtr_pitch_pos, CTRL_DELTA_T);
            
    //         limit_val(geo->target_pitch_vel, 1.5f);
    //         // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
    //         //     set_speed_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_vel*RADtoDEG, 0), 8);
    //         break;
    //     case GIM_FOLD:
    //     case GIM_IMU_PREP_FOLDRISE:
    //     case GIM_FOLD_PREP:

    //         geo->target_pitch_vel = pid_cycle(&f_pitch_pos_pid, 
    //             FOLD_MODE_PITCH_MOTOR_RAD - geo->mtr_pitch_pos, CTRL_DELTA_T);
            
    //         limit_val(geo->target_pitch_vel, 1.5f);

    //         // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
    //         //     set_speed_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_vel*RADtoDEG, 0), 8);
    //         break;
        
    //     default:
    //         // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
    //         //     set_speed_MyAct(motors.pitch.tranmitbuf, 0.0f, 0), 8);
    //         break;
    // }

    // const float GIMBAL_WEIGHT = 4.0f;
    // const float GIMBAL_CENTRE_OF_MASS_DISTENCE = 0.12f;
    // float gravity_feedforward = cosf(geo->abs_pitch_pos)*(9.81f*GIMBAL_WEIGHT*GIMBAL_CENTRE_OF_MASS_DISTENCE);
    // geo->T_pitch += gravity_feedforward;
    
    // // float Tfly_1 = pid_cycle(&flywheel_1_pid, -geo->target_flywheel_rpm - geo->f1vel_filtered, CTRL_DELTA_T);
    // float Tfly_1 = pid_cycle(&flywheel_1_pid, -geo->target_flywheel_rpm - fmotor.flywheel_1.speed, CTRL_DELTA_T);
    // float Tfly_2 = pid_cycle(&flywheel_2_pid, -geo->target_flywheel_rpm - fmotor.flywheel_2.speed, CTRL_DELTA_T);
    // float Tfly_3 = pid_cycle(&flywheel_3_pid, geo->target_flywheel_rpm - fmotor.flywheel_3.speed, CTRL_DELTA_T);

    // float feeder_vel = fmotor.feeder_top.speed*(RPMtoRADS*M2006_GEAR_RATIO);
    // const float feeder_vel_alpha = 0.2f;
    // geo->feeder_vel = geo->feeder_vel * (1.0f-feeder_vel_alpha) + feeder_vel*feeder_vel_alpha;
    // float Tfeeder_1 = pid_cycle(&feeder_1_vel_pid, geo->target_feeder_vel - geo->feeder_vel, CTRL_DELTA_T);

    // const int feeder_push = b2g_B.feeder_push;
    // if(!feeder_push){
    //     geo->feeder_position = 0.0f;
    //     Tfeeder_1 = limit_val(Tfeeder_1, 0.09f);
    // }else{
    //     geo->feeder_position += feeder_vel * CTRL_DELTA_T;
    // }

    // geo->feeder_position += feeder_vel * CTRL_DELTA_T;

    // if(geo->feeder_position > geo->target_feeder_position){
    //     Tfeeder_1 = limit_val(Tfeeder_1, 0.115f);
    // }

    // uint8_t tx_buffer[8];
    // fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(
    //     tx_buffer,
    //     Tfly_1*(1/M3508_TORQUE_CONSTANT),
    //     Tfly_2*(1/M3508_TORQUE_CONSTANT),
    //     Tfly_3*(1/M3508_TORQUE_CONSTANT),
    //     Tfeeder_1*(1/M2006_TORQUE_CONSTANT)
    // ), 8);

    // if(BTB_ONLINE){
    //     // g2b_A.gimbal_yaw_vel_imu = geo->yaw_vel_imu;
    //     g2b_A.gimbal_yaw_pos_imu =  imu_data.yaw;
    //     g2b_A.gimbal_pitch = (int16_t)(wrap_to_pi(geo->mtr_pitch_pos)*1E4f);
    //     g2b_A.gimbal_fold = (int16_t)(wrap_to_pi(geo->mtr_fold_pos)*1E4f);
    //     fdcanx_send_data(&hfdcan1, G2B_MSG_A_ID, (uint8_t *)&g2b_A, 8);

    //     const float FEEDER_POSITION_PER_SHOT = 1.0f;
    //     g2b_B.gimbal_yaw_vel_imu = geo->yaw_vel_imu;
    //     g2b_B.feeder_in_place = (geo->feeder_position > FEEDER_POSITION_PER_SHOT);
    //     fdcanx_send_data(&hfdcan1, G2B_MSG_B_ID, (uint8_t *)&g2b_B, 8);


    //     predict_yaw -= 1.2f*DEGtoRAD;

    //     g2b_C.aim_yaw_pos = (int16_t)(predict_yaw*1E4);
    //     g2b_C.aim_yaw_vel = (int16_t)(predict_vyaw*1E4*0.3f);
    //     // g2b_C.aim_yaw_vel = 0;
    //     g2b_C.vision_tracked = vision_tracked;
    //     g2b_C.vision_locked = 0;
    //     g2b_C.aim_distence = (int16_t)(predict_distence*1E3);
    //     fdcanx_send_data(&hfdcan1, G2B_MSG_C_ID, (uint8_t *)&g2b_C, 8);
    // }

    // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
    //     set_torque_X4_36(motors.pitch.tranmitbuf, -geo->T_pitch), 8);

    const float FOLDING_MAX_SPEED_DPS = 60.0f;
    const float NORMAL_POSITION_DEG = 1.2f;
    const float DOWN_POSITION_DEG = -129.5f; //(19.5+110)
    // const float DOWN_POSITION_DEG = -90.0f;
    // if(gimbal_mode == GIM_FOLD){
    //     fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x6, 
    //         set_position_MyAct(motors.fold.tranmitbuf, DOWN_POSITION_DEG, FOLDING_MAX_SPEED_DPS), 8);
    // }else{

    fold_mtrpos += b2g_A.target_mtr_fold_vel*60.0f*CTRL_DELTA_T;
    pitch_mtrpos += -b2g_A.target_mtr_pitch_vel*60.0f*CTRL_DELTA_T;

    // if(fold_mtrpos < -NORMAL_POSITION_DEG){
    //     fold_mtrpos = -NORMAL_POSITION_DEG;
    // }else if(fold_mtrpos > -DOWN_POSITION_DEG){
    //     fold_mtrpos = -DOWN_POSITION_DEG;
    // }
        fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x6, 
            set_position_MyAct(motors.fold.tranmitbuf, fold_mtrpos, 120.0f), 8);
        fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5,
            set_position_MyAct(motors.pitch.tranmitbuf, pitch_mtrpos, 120.0f), 8);
            
        //         fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x6, 
        //     set_speed_MyAct(motors.fold.tranmitbuf, b2g_A.target_mtr_fold_vel*60.0f, 200), 8);
        // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5,
        //     set_speed_MyAct(motors.pitch.tranmitbuf, -b2g_A.target_mtr_pitch_vel*60.0f, 200), 8);
           
    
    // }

    // const float is_fold_down = (fabsf(fmotor.fold.precise_position*RADtoDEG - DOWN_POSITION_DEG) < 10.0f);

    // if((gimbal_mode == GIM_FOLD_PREP || (gimbal_mode == GIM_FOLD && !is_fold_down)) 
    //     && BTB_ONLINE){
        FOLDING_UNLOCK();
    // }else{
    //     FOLDING_LOCK();
    // }

    // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_ALL, 
    //     acquire_motor_angle_MyAct(motors.pitch.tranmitbuf), 8);

    vofa.val[0]=-fmotor.flywheel_1.speed;
    // vofa.val[1]=(float)b2g_B.feeder_push;

    // vofa.val[2]=geo->mtr_fold_pos;
    // vofa.val[3]=geo->mtr_pitch_pos;

    // vofa.val[4]=vision_FromRos.packet.v_yaw*RADtoDEG;
    // vofa.val[5]=geo->mtr_pitch_vel;
    
    // vofa.val[6]=predict_distence;

    // vofa.val[7]=predict_pitch;
    // vofa.val[8]=geo->abs_pitch_pos;
    // vofa.val[9]=geo->abs_pitch_pos - predict_pitch;
}


void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x201:
        parse_feedback_M3508(msg, &motors.flywheel_1);
        break;
    case 0x202:
        parse_feedback_M3508(msg, &motors.flywheel_2);
        break;
    case 0x203:
        parse_feedback_M3508(msg, &motors.flywheel_3);
        break;
    case 0x204:
        parse_feedback_M3508(msg, &motors.feeder_top);
        break;
    case MYACT_RPTID+0x5: 
        parse_feedback_X4_36(msg, &motors.pitch);
        break;
    case MYACT_RPTID+0x6:
        parse_feedback_X4_36(msg, &motors.fold);
        break;
    case B2G_MSG_A_ID:
        memcpy(&b2g_A, msg, 8);
        BTB_UPDATE_CNTDOWN();
        break;
    // case B2G_MSG_B_ID:
    //     memcpy(&b2g_B, msg, 8);
    //     BTB_UPDATE_CNTDOWN();
    //     break;
    default:
        volatile int a=ID;
        volatile uint8_t temp[8];
        memcpy(temp, msg, 8);
        a++;
        break;
    }

    return;
}

#endif

#endif
