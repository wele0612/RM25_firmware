#include<config.h>

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<motors.h>
#include<btb.h>
#include<h7can.h>

#ifdef CONFIG_ROBOT_HERO

typedef enum {
    GIMBAL_IMU_MODE = 0, // Normal IMU-based pitch control
    GIMBAL_SNIPER_MODE = 1, // Sniper mode, use absolute encoder value
    GIMBAL_FOLD_MODE = 2 // Folding mode, keep motor at position (-199.5 deg)
}gimbal_mode_t;

#ifdef CONFIG_PLATFORM_BASE

enum {
    GIM_STANDBY = 0,
    GIM_IMU_PREP = 1,
    GIM_IMU = 2,
    GIM_FOLD_PREP = 3,
    GIM_FOLD = 4,
    GIM_SNIPER = 5, // 吊射模式
}gim_state;

const float AGI_INIT_SETPOINT = 2.266f;

static void inline init_agi_damiao(){
    fdcanx_send_data(&hfdcan3, AGI_CTRLID, enable_DM_Joint(motors.agi.tranmitbuf), 8);
}

void role_controller_init(){
    gim_state = GIM_IMU;

    HAL_Delay(1500);
    fdcanx_send_data(&hfdcan3, YAW_CTRLID, enable_DM_Joint(motors.yaw.tranmitbuf), 8);
    HAL_Delay(2);
    init_agi_damiao();

}

void dr16_on_change(){
    return;
}

PID_t body_x_vel_pid={
    .P = 5.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t body_y_vel_pid={
    .P = 5.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t body_yaw_vel_pid={
    .P = 5.0f,
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
    .P = 10.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.005f
};

PID_t f_gimbal_yaw_vel_pid={
    .P = 1.5f,
    .I = 30.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t f_gimbal_yaw_pos_pid={
    .P = 3.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.005f
};


PID_t agi_vel_pid={
    .P = 3.0f,
    .I = 20.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t agi_pos_pid={
    .P = 15.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();

    geo->target_vx = dr16.channel[0]*0.6f;
    geo->target_vy = dr16.channel[1]*0.6f;
    // geo->target_vyaw = dr16.channel[2]*1.5f;

    // geo->input_yaw_vel = dr16.channel[2]*1.0f;
    // geo->input_pitch_vel = dr16.channel[3]*1.0f;
    const float input_mouse_alpha = 0.02f;
    geo->input_pitch_vel = geo->input_pitch_vel*(1.0f-input_mouse_alpha) + dr16.mouse.y*0.02f*input_mouse_alpha;
    geo->input_yaw_vel = geo->input_yaw_vel*(1.0f-input_mouse_alpha) + -dr16.mouse.x*0.02f*input_mouse_alpha;

    b2g_B.flywheel_enabled = (dr16.s2 == DR16_SWITCH_UP);

    // geo->target_yaw_vel = dr16.channel[3]*1.5f;
    
    // if(dr16.channel[3] > 0.3f){
    //     geo->target_yaw_vel = 1.5f;
    //     // geo->target_yaw_pos = 1.0f;
    // }else if(dr16.channel[3] < -0.3f){
    //     // geo->target_yaw_vel = -1.5f;
    //     geo->target_yaw_pos = -1.0f;
    // }else{
    //     // geo->target_yaw_vel = 0.0f;
    //     geo->target_yaw_pos = 0.0f;
    // }

    geo->target_yaw_pos = wrap_to_pi(geo->target_yaw_pos + geo->input_yaw_vel*CTRL_DELTA_T);

    /* Agitator control */

    geo->agi_pos = fmotor.agi.position;
    geo->target_agi_vel = pid_cycle(&agi_pos_pid, wrap_to_pi(geo->target_agi_pos - geo->agi_pos), CTRL_DELTA_T);
    geo->target_agi_vel = limit_val(geo->target_agi_vel, 10.0f);
    geo->agi_vel = fmotor.agi.speed * 0.25f;
    geo->T_agi = pid_cycle(&agi_vel_pid, geo->target_agi_vel-geo->agi_vel, CTRL_DELTA_T);

    /* Gimbal control */

    geo->gimbal_mtr_pitch_pos = (float)(g2b_A.gimbal_pitch)*1E-4f;
    const float dm_motor_alpha = 0.2f;
    geo->gimbal_mtr_yaw_vel = dm_motor_alpha*fmotor.yaw.speed + (1.0f-dm_motor_alpha)*geo->gimbal_mtr_yaw_vel;
    geo->gimbal_mtr_yaw_pos = wrap_to_pi(fmotor.yaw.position);

    geo->gimbal_abs_yaw_vel = g2b_A.gimbal_yaw_vel_imu;
    geo->gimbal_abs_yaw_pos = wrap_to_pi(geo->gimbal_abs_yaw_pos + geo->gimbal_abs_yaw_vel*CTRL_DELTA_T);

    // State-transition functions
    switch(gim_state){
    case GIM_IMU:
        b2g_B.gimbal_mode = GIMBAL_IMU_MODE;
        if(dr16.s1 == DR16_SWITCH_UP){
            gim_state = GIM_FOLD_PREP;
        }
        break;
    case GIM_FOLD_PREP:
        b2g_B.gimbal_mode = GIMBAL_FOLD_MODE;
        if(dr16.s1 != DR16_SWITCH_UP){
            gim_state = GIM_IMU_PREP;
        }
        const float yaw_ready_err_max = 1.0f*DEGtoRAD;
        break;
    case GIM_IMU_PREP:

        break;
    default:
        gim_state=GIM_STANDBY;
        break;
    }

    if(BTB_ONLINE && gim_state == GIM_IMU){
        geo->target_yaw_vel = geo->input_yaw_vel + pid_cycle(&g_gimbal_yaw_pos_pid, wrap_to_pi(geo->target_yaw_pos - geo->gimbal_abs_yaw_pos), CTRL_DELTA_T);
        // geo->target_yaw_vel = limit_val(geo->target_yaw_vel, 5.0f);

        float gimbal_yaw_pid_coe = 0.2f + fabsf(cosf(geo->gimbal_mtr_pitch_pos))*(0.8f);
        geo->T_yaw = gimbal_yaw_pid_coe * pid_cycle(&g_gimbal_yaw_vel_pid, geo->target_yaw_vel - geo->gimbal_abs_yaw_vel, CTRL_DELTA_T);
    }else if(BTB_ONLINE){
        const float gimbal_standby_position = 68.4f*DEGtoRAD;
        geo->target_yaw_vel = pid_cycle(&f_gimbal_yaw_pos_pid, wrap_to_pi(gimbal_standby_position - geo->gimbal_mtr_yaw_pos), CTRL_DELTA_T);
        geo->target_yaw_vel = limit_val(geo->target_yaw_vel, 2.0f);
        geo->T_yaw = pid_cycle(&f_gimbal_yaw_vel_pid, geo->target_yaw_vel - geo->gimbal_mtr_yaw_vel, CTRL_DELTA_T);
    }else{
        geo->T_yaw = 0.0f;
    }

    /* Base control */

    const float WHEEL_RADIUS = 0.153f/2.0f;
    const float BODY_RADIUS = 0.6f/2.0f;

    const float VEL_RPM_COE = RPMtoRADS*M3508_CUSTOM_GB_RATIO*WHEEL_RADIUS;
    float vel_LF = fmotor.wheel_LF.speed*VEL_RPM_COE;
    float vel_LB = fmotor.wheel_LB.speed*VEL_RPM_COE;
    float vel_RF = fmotor.wheel_RF.speed*VEL_RPM_COE;
    float vel_RB = fmotor.wheel_RB.speed*VEL_RPM_COE;

    geo->vyaw_gyro = -imu_data.gyro[2]*DEGtoRAD;
    
    /* Forward kinetics of the base */
    float vx_b = -(vel_LF + vel_RF - vel_LB - vel_RB)*(0.7071068f/4.0f);
    float vy_b = -(vel_LF - vel_RF + vel_LB - vel_RB)*(0.7071068f/4.0f);
    float vyaw_wheel = (vel_LF + vel_LB + vel_RF + vel_RB)*(0.25f/BODY_RADIUS);

    const float wheel_v_alpha = 0.2f;
    geo->vx_b = vx_b*wheel_v_alpha + geo->vx_b*(1.0f-wheel_v_alpha);
    geo->vy_b = vy_b*wheel_v_alpha + geo->vy_b*(1.0f-wheel_v_alpha);
    geo->vyaw_wheel = vyaw_wheel*wheel_v_alpha + geo->vyaw_wheel*(1.0f-wheel_v_alpha);

    // geo->T_base_yaw = pid_cycle()
    geo->F_x_b = geo->F_x;
    geo->F_y_b = geo->F_y;

    float F_single_wheel_turn = -geo->T_base_yaw*(0.25f/BODY_RADIUS);
    float T_single_wheel_turn = F_single_wheel_turn*WHEEL_RADIUS;
    float force_vector_coe = 1.4142136f*0.25f;
    geo->T_LF =  -(geo->F_y_b + geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    geo->T_LB =  -(geo->F_y_b - geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    geo->T_RF = -(-geo->F_y_b + geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    geo->T_RB = -(-geo->F_y_b - geo->F_x_b)*force_vector_coe + T_single_wheel_turn;

    uint8_t tx_buf[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(tx_buf,
        geo->T_LF*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
        geo->T_LB*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
        geo->T_RF*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
        geo->T_RB*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB)), 8);

    fdcanx_send_data(&hfdcan3, YAW_CTRLID, set_torque_DM4310(motors.yaw.tranmitbuf, geo->T_yaw), 8);

    geo->T_agi=0.0f;
    fdcanx_send_data(&hfdcan3, AGI_CTRLID, set_torque_DM4310(motors.agi.tranmitbuf, geo->T_agi), 8);

    fdcanx_send_data(&hfdcan1, B2G_MSG_A_ID, (uint8_t *)&b2g_A, 8);

    geo->input_pitch_vel = limit_val(geo->input_pitch_vel, 6.0f);
    b2g_B.target_pitch_vel = (int16_t)(geo->input_pitch_vel*(1/3E-4f));
    b2g_B.base_roll = imu_data.roll;
    fdcanx_send_data(&hfdcan1, B2G_MSG_B_ID, (uint8_t *)&b2g_B, 8);

    vofa.val[0]=geo->vx_b;
    vofa.val[1]=geo->vy_b;
    vofa.val[2]=geo->gimbal_mtr_pitch_pos;
    vofa.val[3]=geo->vyaw_wheel;

    vofa.val[4]=geo->input_yaw_vel;
    vofa.val[5]=geo->gimbal_mtr_yaw_pos;
    vofa.val[6]=gim_state;

    // vofa.val[7]=geo->target_agi_pos;
    // vofa.val[8]=geo->agi_vel;
    // vofa.val[9]=fmotor.agi.position;

    vofa.val[7]=geo->target_yaw_vel;
    vofa.val[8]=geo->target_yaw_pos;
    vofa.val[9]=geo->gimbal_mtr_yaw_vel;
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
    case G2B_MSG_A_ID:
        memcpy(&g2b_A, msg, 8);
        BTB_UPDATE_CNTDOWN();
        break;
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
    .P=1.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
}; 

PID_t fold_vel_pid={
    .P=0.1f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t fold_pos_pid={
    .P=0.1f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

// uint8_t reset_zeropoint[8] = {0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void role_controller_init(){

}

void dr16_on_change(){
    robot_ctrl_t *geo = &robot_geo;

    if(dr16.s2 == DR16_SWITCH_MID && dr16.previous.s2 == DR16_SWITCH_DOWN){
        geo->feeder_position = 0.0f;
        geo->target_feeder_position = 1.7f;
    }
    
}

/* For X4-36, need to first set VEL_P to 0.015, VEL_I to 0.00005 */
/* Also need to increase X4-36 max acceleration */
void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();
    
    if(BTB_ONLINE){
        geo->target_feeder_vel = 3.14f;
    }else{
        geo->target_feeder_vel = 0.0f;
    }

    if(BTB_ONLINE){
        if(b2g_B.flywheel_enabled){
            geo->target_flywheel_rpm = 4075.0f;
        }else{
            geo->target_flywheel_rpm = 200.0f;
        }
    }else{
        geo->target_flywheel_rpm = 60.0f;
    }

    geo->yaw_vel_imu = wrap_to_pi(imu_data.yaw - geo->yaw_m1)*(1.0f/CTRL_DELTA_T);
    geo->yaw_m1 = imu_data.yaw;

    /* Note on Pitch Motor Zero point: 
    * Motor zero for both motor => gimbal locked + minimum pitch
    */
    const float PITCH_MOTOR_LOCKED_PITCH_MINIMUM = -9.5f*DEGtoRAD;
    const float FOLD_MOTOR_LOCKED_BENDING_BACKWARD = -20.0f*DEGtoRAD;
    
    geo->abs_pitch_vel = -imu_data.gyro[1]*DEGtoRAD;
    geo->abs_pitch_pos = -imu_data.pitch;

    const float myact_motor_alpha = 0.5f;

    geo->mtr_pitch_pos = -fmotor.pitch.precise_position + PITCH_MOTOR_LOCKED_PITCH_MINIMUM;
    geo->mtr_pitch_vel = myact_motor_alpha*(-fmotor.pitch.speed)
                        + geo->mtr_pitch_vel*(1.0f-myact_motor_alpha);

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

    geo->input_pitch_vel = b2g_B.target_pitch_vel*3E-4;

    uint8_t gimbal_mode = b2g_B.gimbal_mode;

    switch (gimbal_mode){
        case GIMBAL_IMU_MODE:
            geo->target_pitch_pos += geo->input_pitch_vel*CTRL_DELTA_T;

            const float MAX_PITCH_ANGLE=50.0f*DEGtoRAD;
            const float MIN_PITCH_ANGLE=PITCH_MOTOR_LOCKED_PITCH_MINIMUM+1.5f*DEGtoRAD;
            
            if(geo->target_pitch_pos > MAX_PITCH_ANGLE){
                geo->target_pitch_pos = MAX_PITCH_ANGLE;
                geo->target_pitch_vel = pid_cycle(&g_pitch_pos_pid, geo->target_pitch_pos - geo->abs_pitch_pos, CTRL_DELTA_T);
            }else if(geo->target_pitch_pos < MIN_PITCH_ANGLE){
                geo->target_pitch_pos = MIN_PITCH_ANGLE;
                geo->target_pitch_vel = pid_cycle(&g_pitch_pos_pid, geo->target_pitch_pos - geo->abs_pitch_pos, CTRL_DELTA_T);
            }else{
                geo->target_pitch_vel = geo->input_pitch_vel + 
                pid_cycle(&g_pitch_pos_pid, geo->target_pitch_pos - geo->abs_pitch_pos, CTRL_DELTA_T);
            }

            fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
                set_speed_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_vel*RADtoDEG, 0), 8);

            break;
        case GIMBAL_FOLD_MODE:

            const float FOLD_MODE_PITCH_MOTOR_RAD = 129.0f*DEGtoRAD;
            geo->target_pitch_vel = pid_cycle(&f_pitch_pos_pid, 
                FOLD_MODE_PITCH_MOTOR_RAD - geo->mtr_pitch_pos, CTRL_DELTA_T);
            
            limit_val(geo->target_pitch_vel, 1.0f);
            fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
                set_speed_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_vel*RADtoDEG, 0), 8);
            break;
        
        default:
            break;
    }

    const float GIMBAL_WEIGHT = 4.0f;
    const float GIMBAL_CENTRE_OF_MASS_DISTENCE = 0.12f;
    float gravity_feedforward = cosf(geo->abs_pitch_pos)*(9.81f*GIMBAL_WEIGHT*GIMBAL_CENTRE_OF_MASS_DISTENCE);
    geo->T_pitch += gravity_feedforward;
    
    // float Tfly_1 = pid_cycle(&flywheel_1_pid, -geo->target_flywheel_rpm - geo->f1vel_filtered, CTRL_DELTA_T);
    float Tfly_1 = pid_cycle(&flywheel_1_pid, -geo->target_flywheel_rpm - fmotor.flywheel_1.speed, CTRL_DELTA_T);
    float Tfly_2 = pid_cycle(&flywheel_2_pid, -geo->target_flywheel_rpm - fmotor.flywheel_2.speed, CTRL_DELTA_T);
    float Tfly_3 = pid_cycle(&flywheel_3_pid, geo->target_flywheel_rpm - fmotor.flywheel_3.speed, CTRL_DELTA_T);

    float feeder_vel = fmotor.feeder_top.speed*(RPMtoRADS*M2006_GEAR_RATIO);
    const float feeder_vel_alpha = 0.2f;
    geo->feeder_vel = geo->feeder_vel * (1.0f-feeder_vel_alpha) + feeder_vel*feeder_vel_alpha;
    float Tfeeder_1 = pid_cycle(&feeder_1_vel_pid, geo->target_feeder_vel - geo->feeder_vel, CTRL_DELTA_T);

    geo->feeder_position += feeder_vel * CTRL_DELTA_T;

    if(geo->feeder_position > geo->target_feeder_position){
        Tfeeder_1 = limit_val(Tfeeder_1, 0.115f);
    }

    uint8_t tx_buffer[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(
        tx_buffer,
        Tfly_1*(1/M3508_TORQUE_CONSTANT),
        Tfly_2*(1/M3508_TORQUE_CONSTANT),
        Tfly_3*(1/M3508_TORQUE_CONSTANT),
        Tfeeder_1*(1/M2006_TORQUE_CONSTANT)
    ), 8);

    if(!BTB_ONLINE){
        geo->T_pitch = 0.0f;
    }else{
        g2b_A.gimbal_yaw_vel_imu = geo->yaw_vel_imu;
        g2b_A.gimbal_pitch = (int16_t)(wrap_to_pi(geo->mtr_pitch_pos)*1E4f);
        fdcanx_send_data(&hfdcan1, G2B_MSG_A_ID, (uint8_t *)&g2b_A, 8);
    }

    // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
    //     set_torque_X4_36(motors.pitch.tranmitbuf, -geo->T_pitch), 8);
    
    fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x6, 
        set_torque_X4_36(motors.fold.tranmitbuf, 0.0f), 8);

    fdcanx_send_data(&hfdcan3, MYACT_CTRLID_ALL, 
        acquire_motor_angle_MyAct(motors.pitch.tranmitbuf), 8);

    vofa.val[0]=-fmotor.flywheel_1.speed;
    vofa.val[1]=gravity_feedforward;

    vofa.val[2]=geo->abs_pitch_pos;
    vofa.val[3]=geo->mtr_pitch_pos;

    vofa.val[4]=geo->abs_pitch_vel;
    vofa.val[5]=geo->mtr_pitch_vel;
    
    vofa.val[6]=geo->target_pitch_pos;
    vofa.val[7]=geo->target_pitch_vel;
    vofa.val[8]=(float)b2g_B.gimbal_mode;

    vofa.val[9]=fmotor.pitch.precise_position;
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
    case B2G_MSG_B_ID:
        memcpy(&b2g_B, msg, 8);
        BTB_UPDATE_CNTDOWN();
        break;
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
