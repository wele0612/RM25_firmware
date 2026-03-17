#include<config.h>
#ifdef CONFIG_ROBOT_AIMING_BENCH

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<h7can.h>

#ifdef CONFIG_PLATFORM_BASE

PID_t vy_pid={
    .P = 6.0f,
    .I = 0.0f,
    .D = 0.0f,
};

PID_t vx_pid={
    .P = 6.0f,
    .I = 0.0f,
    .D = 0.0f,
};

PID_t vyaw_pid={
    .P = 1.5f,
    .I = 0.0f,
    .D = 0.0f,
};

void role_controller_init(){

}


void role_controller_step(const float CTRL_DELTA_T){
    uint8_t tx_buffer[8]; 

    robot_ctrl_t *geo = &robot_geo;

    geo->target_vy = dr16.channel[1]*1.2f;
    geo->target_vx = dr16.channel[0]*1.2f;
    // geo->target_vyaw = dr16.channel[2]*2.0f;

    if(dr16.s2 == DR16_SWITCH_DOWN){
        geo->target_vyaw = -dr16.channel[2]*1.0f;
        geo->yaw_offset = imu_data.yaw;
    }else if(dr16.s2 == DR16_SWITCH_MID){
        geo->target_vyaw = 0.5f*2.0f*PI;
    }

    const float WHEEL_RADIUS = 0.154f * 0.5f;
    const float RPM_TO_MS = RPMtoRADS*WHEEL_RADIUS*M3508_GEAR_RATIO;

    const float v_alpha = 0.3f;
    float vy = (RPM_TO_MS*0.25f)*(motors.wheel_LF.speed + motors.wheel_LB.speed - motors.wheel_RF.speed - motors.wheel_RB.speed);
    float vx = (RPM_TO_MS*0.25f)*(motors.wheel_LF.speed - motors.wheel_LB.speed + motors.wheel_RF.speed - motors.wheel_RB.speed);
    
    geo->vy_b = vy*v_alpha + geo->vy_b*(1.0f - v_alpha);
    geo->vx_b = vx*v_alpha + geo->vx_b*(1.0f - v_alpha);
    geo->vyaw = imu_data.gyro[2]*DEGtoRAD;

    const float body_yaw_offset = imu_data.yaw - geo->yaw_offset;
    const float cosby = cosf(body_yaw_offset);
    const float sinby = sinf(body_yaw_offset);

    // // Convert body coordinate system to gimbal coordinate system.
    geo->vx = geo->vx_b * cosby + geo->vy_b * -sinby;
    geo->vy = geo->vx_b * sinby + geo->vy_b * cosby;


    float F_y = pid_cycle(&vy_pid, geo->target_vy - geo->vy, CTRL_DELTA_T);
    float F_x = pid_cycle(&vx_pid, geo->target_vx - geo->vx, CTRL_DELTA_T);
    float T_yaw = pid_cycle(&vyaw_pid, geo->target_vyaw - geo->vyaw, CTRL_DELTA_T);

    float F_x_b = F_x * cosby + F_y * sinby;
    float F_y_b = F_x * -sinby + F_y * cosby;

    float T_LF=0.0f, T_LB=0.0f, T_RF=0.0f, T_RB=0.0f;

    T_LF = F_y_b + F_x_b - T_yaw;
    T_LB = F_y_b - F_x_b - T_yaw;
    T_RF = -F_y_b + F_x_b - T_yaw;
    T_RB = -F_y_b - F_x_b - T_yaw;

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(tx_buffer,
        T_LF*(1.0f/M3508_TORQUE_CONSTANT), 
        T_LB*(1.0f/M3508_TORQUE_CONSTANT), 
        T_RF*(1.0f/M3508_TORQUE_CONSTANT), 
        T_RB*(1.0f/M3508_TORQUE_CONSTANT)
    ), 8);

    vofa.val[0]=imu_data.yaw;
    vofa.val[1]=imu_data.pitch;
    vofa.val[2]=imu_data.gyro[2];
    vofa.val[3]=imu_data.gyro[1];

    vofa.val[4]=geo->vy;
    vofa.val[5]=geo->vx;
    vofa.val[6]=geo->vyaw;
    vofa.val[7]=dr16.channel[3];

    vofa.val[8]=(float)robot_config.test_val;
    vofa.val[9]=robot_config.imu_gyro_offset[2];

    // vofa.val[8]=(float)dr16.s1;
    // vofa.val[9]=(float)dr16.s2;
    

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

    default:
        break;
    }

    return;
}



#else

PID_t pitch_omega_pid={
    .P=3.0f,
    .I=100.0f,
    .D=0.02f,
    .integral_max=0.01f
};

PID_t pitch_pos_pid={
    .P=17.0f,
    .I=0.0f,
    .D=0.00f,
    .integral_max=0.01f
};

// 0x0F is CAN(Slave) and 0x00 is Master
void role_controller_init(){
    HAL_Delay(1000);
    fdcanx_send_data(&hfdcan3, 0x0F, enable_DM4310(motors.pitch.tranmitbuf), 8);
}

void role_controller_step(const float CTRL_DELTA_T){
    // float pitch_tau = dr16.channel[0] * 2.0f;
    // fdcanx_send_data(&hfdcan3, 0x0F, set_torque_DM4310(motors.pitch.tranmitbuf, pitch_tau), 8);
    // if (dr16.channel[0] >= 0.5f) {
    //     robot_geo.target_pitch_omega = 1.0f;
    // } else if (dr16.channel[0] <= -0.5f) {
    //     robot_geo.target_pitch_omega = -1.0f;
    // }else {
    //     robot_geo.target_pitch_omega = 0.0f;
    // }
    // -0.2 -> 0.5
    // if (dr16.channel[0] >= 0.5f) {
    //     robot_geo.target_pitch_pos = 0.50f;
    // } else if (dr16.channel[0] <= -0.5f) {
    //     robot_geo.target_pitch_pos = -0.10f;
    // }else {
    //     robot_geo.target_pitch_pos = 0.25f;
    // }
    robot_geo.target_pitch_pos += -0.007f * (float)dr16.mouse.y * CTRL_DELTA_T;
    if (robot_geo.target_pitch_pos >= 0.5f) {
        robot_geo.target_pitch_pos = 0.5f;
    }
    if (robot_geo.target_pitch_pos <= -0.2f) {
        robot_geo.target_pitch_pos = -0.2f;
    }


    // robot_geo.target_pitch_omega = dr16.channel[0]; 

    // const float target_pitch_omega=robot_geo.target_pitch_omega;
    const float target_pitch_pos=robot_geo.target_pitch_pos;
    //float pitch_omega_error = target_pitch_omega - motors.pitch.speed;
    float pitch_omega_error = 0.0f;
    float pitch_pos_error = 0.0f;

    pitch_pos_error = target_pitch_pos - imu_data.pitch; 
    float pitch_pid_result = pid_cycle(&pitch_pos_pid, pitch_pos_error, 0.001f);

    const float target_pitch_omega = pitch_pid_result; 

    if (fabsf(motors.pitch.speed) < 0.05f && fabsf(imu_data.gyro[1]*(PI/180.0f)) < 0.05f){
        pitch_omega_error = target_pitch_omega;
    } else {
        pitch_omega_error = target_pitch_omega - imu_data.gyro[1]*(PI/180.0f);
    }
    
    float pitch_torque = 0.0f;
    
    pitch_torque = pid_cycle(&pitch_omega_pid, pitch_omega_error, 0.001f);
    pitch_torque+=1.1689f*imu_data.pitch+0.3553f;

    // pitch_torque = 0.0f;
    fdcanx_send_data(&hfdcan3, 0x0F, set_torque_DM4310(motors.pitch.tranmitbuf, pitch_torque), 8);

    vofa.val[0]=motors.pitch.position; // range 91-144
    vofa.val[1]=motors.pitch.speed;
    vofa.val[2]=imu_data.gyro[1]*(PI/180.0f);
    vofa.val[3]=target_pitch_omega;
    vofa.val[4]=target_pitch_pos;
    vofa.val[5]=pitch_torque;
    vofa.val[6]=imu_data.pitch; // -18 deg -> 36 deg, choose 0 10 20 
    vofa.val[7]=(float)dr16.mouse.y;
    // vofa.val[6]=(float)gyro_raw[1];
}
//根据pitch计算前馈torque:y=1.1689x+0.7553;

void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x0:
        parse_feedback_DM4310(msg, &motors.pitch, 0x1F);
        break;


    default:
        break;
    }
    
    


    return;
}

#endif

#endif
