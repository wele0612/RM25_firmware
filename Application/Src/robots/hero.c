#include<config.h>

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<motors.h>
#include<btb.h>
#include<h7can.h>

#ifdef CONFIG_ROBOT_HERO

#ifdef CONFIG_PLATFORM_BASE

static void inline init_agi_damiao(){
    fdcanx_send_data(&hfdcan3, AGI_CTRLID, enable_DM_Joint(motors.agi.tranmitbuf), 8);
}

void role_controller_init(){
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

PID_t gimbal_yaw_vel_pid={
    .P = 1.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t gimbal_yaw_pos_pid={
    .P = 1.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
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

    /* Agitator control */
    if(dr16.channel[3] > 0.3f){
        geo->target_agi_pos = PI/3.0f;
    }else{
        geo->target_agi_pos = 0.0f;
    }

    geo->agi_pos = fmotor.agi.position;
    geo->target_agi_vel = pid_cycle(&agi_pos_pid, wrap_to_pi(geo->target_agi_pos - geo->agi_pos), CTRL_DELTA_T);
    geo->target_agi_vel = limit_val(geo->target_agi_vel, 10.0f);
    geo->agi_vel = fmotor.agi.speed * 0.25f;
    geo->T_agi = pid_cycle(&agi_vel_pid, geo->target_agi_vel-geo->agi_vel, CTRL_DELTA_T);

    /* Base control */

    const float WHEEL_RADIUS = 0.153f/2.0f;
    const float BODY_RADIUS = 0.6f/2.0f;

    const float VEL_RPM_COE = RPMtoRADS*M3508_CUSTOM_GB_RATIO*WHEEL_RADIUS;
    float vel_LF = fmotor.wheel_LF.speed*VEL_RPM_COE;
    float vel_LB = fmotor.wheel_LB.speed*VEL_RPM_COE;
    float vel_RF = fmotor.wheel_RF.speed*VEL_RPM_COE;
    float vel_RB = fmotor.wheel_RB.speed*VEL_RPM_COE;

    geo->target_vx = dr16.channel[0]*0.6f;
    geo->target_vy = dr16.channel[1]*0.6f;
    geo->target_vyaw = dr16.channel[2]*1.5f;

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

    // fdcanx_send_data(&hfdcan1, B2G_MSG_A_ID, (uint8_t *)&b2g_A, 8);

    geo->input_pitch_vel = limit_val(geo->input_pitch_vel, 6.0f);
    b2g_B.target_pitch_vel = (int16_t)(geo->input_pitch_vel*(1/3E-4f));
    b2g_B.base_roll = imu_data.roll;
    fdcanx_send_data(&hfdcan1, B2G_MSG_B_ID, (uint8_t *)&b2g_B, 8);

    vofa.val[0]=geo->vx_b;
    vofa.val[1]=geo->vy_b;
    vofa.val[2]=geo->vyaw_gyro;
    vofa.val[3]=geo->vyaw_wheel;

    vofa.val[4]=geo->F_x_b;
    vofa.val[5]=geo->F_y_b;
    vofa.val[6]=geo->T_base_yaw;

    vofa.val[7]=geo->target_agi_pos;
    vofa.val[8]=geo->agi_vel;
    vofa.val[9]=fmotor.agi.position;
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

// For gimbal mode
PID_t gimbal_pitch_vel_pid={
    .P=0.5f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
}; 

// For gimbal mode
PID_t gimbal_pitch_pos_pid={
    .P=0.1f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
}; 

// For fold mode
PID_t fold_pitch_vel_pid={
    .P=0.1f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
};

// For fold mode
PID_t fold_fold_vel_pid={
    .P=0.1f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
};

const float AGI_INIT_SETPOINT = 2.266f;

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

    // const float flywheel_alpha = 0.2f;
    // geo->f1vel_filtered = geo->f1vel_filtered*(1.0f - flywheel_alpha) + flywheel_alpha*fmotor.flywheel_1.speed;

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

    Tfeeder_1 = 0.0f;

    uint8_t tx_buffer[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(
        tx_buffer,
        Tfly_1*(1/M3508_TORQUE_CONSTANT),
        Tfly_2*(1/M3508_TORQUE_CONSTANT),
        Tfly_3*(1/M3508_TORQUE_CONSTANT),
        Tfeeder_1*(1/M2006_TORQUE_CONSTANT)
    ), 8);

    fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
        set_torque_X4_36(motors.pitch.tranmitbuf, 0.0f), 8);

    fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x6, 
        set_torque_X4_36(motors.pitch.tranmitbuf, 0.0f), 8);

    // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_ALL, 
    //     set_torque_X4_36(motors.pitch.tranmitbuf, 0.0f), 8);

    vofa.val[0]=-fmotor.flywheel_1.speed;
    vofa.val[1]=-fmotor.flywheel_2.speed;
    vof
    vofa.val[5]=fmotor.pitch.position;
    vofa.val[6]=fmotor.fold.position;
    vofa.val[7]=fmotor.feeder_top.speed;
    vofa.val[8]=referee.shoot_data_0x0207.initial_speed;

    vofa.val[9]=fmotor.flywheel_1.tempreture;
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
