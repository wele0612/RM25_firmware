#include<config.h>
#ifdef CONFIG_ROBOT_AIMING_BENCH

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<chasis_power.h>

#include<h7can.h>

#ifdef CONFIG_PLATFORM_BASE

PID_t vy_pid={
    .P = 4.0f,
    .I = 0.0f,
    .D = 0.0f,
};

PID_t vx_pid={
    .P = 4.0f,
    .I = 0.0f,
    .D = 0.0f,
};

PID_t vyaw_pid={
    .P = 1.2f,
    .I = 0.0f,
    .D = 0.0f,
};

PID_t agi_omega_pid={
    .P = 1.1f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

// only used for aiming bench 
static float powermeter_voltage = 0.0f;
static float powermeter_current = 0.0f;

void role_controller_init(){

}


void role_controller_step(const float CTRL_DELTA_T){
    uint8_t tx_buffer[8]; 

    robot_ctrl_t *geo = &robot_geo;

    // geo->target_vy = dr16.channel[1]*3.0f;
    // geo->target_vx = dr16.channel[0]*3.0f;
    // geo->target_vyaw = dr16.channel[2]*2.0f;

    if(dr16.s1 == DR16_SWITCH_UP){
        const float nav_speedlimit = 0.2f;
        geo->target_vy = limit_val(vision_FromRos.packet.forward_vel, nav_speedlimit);
        geo->target_vx = limit_val(-vision_FromRos.packet.leftward_vel, nav_speedlimit);
    }else{
        geo->target_vy = chasis_ctrl.robot_forward_v*1e-3f;
        geo->target_vx = -chasis_ctrl.robot_leftward_v*1e-3f;
        geo->target_vyaw = (-dr16.channel[2]*6.5f);
    }


    // if(dr16.s2 == DR16_SWITCH_MID){
    //     geo->target_vyaw = 0.5f*2.0f*PI;
    // }else if(dr16.s2 == DR16_SWITCH_UP){
    //     geo->target_vyaw = 1.2f*2.0f*PI;
    // }else{
    //     geo->target_vyaw = -dr16.channel[2]*1.0f;
    //     geo->yaw_offset = imu_data.yaw;
    // }
    
    // if (dr16.s1 == DR16_SWITCH_MID) { // 16Hz 
    //     geo->target_agi_omega = - 0.33f*2.0f*PI;
    // } else if(dr16.s1 == DR16_SWITCH_UP){
    //     geo->target_agi_omega = 0.2f*PI;
    // } else {
    //     geo->target_agi_omega = 0.0f;
    // }

    // geo->target_agi_omega = 2.0f*PI;

    const float WHEEL_RADIUS = 0.154f * 0.5f;
    const float RPM_TO_MS = RPMtoRADS*WHEEL_RADIUS*M3508_GEAR_RATIO;

    const float v_alpha = 0.3f;
    float vy = (RPM_TO_MS*0.25f)*(motors.wheel_LF.speed + motors.wheel_LB.speed - motors.wheel_RF.speed - motors.wheel_RB.speed);
    float vx = (RPM_TO_MS*0.25f)*(motors.wheel_LF.speed - motors.wheel_LB.speed + motors.wheel_RF.speed - motors.wheel_RB.speed);

    geo->vy_b = vy*v_alpha + geo->vy_b*(1.0f - v_alpha);
    geo->vx_b = vx*v_alpha + geo->vx_b*(1.0f - v_alpha);
    geo->vyaw = imu_data.gyro[2]*DEGtoRAD;
    geo->agi_omega = motors.agi.speed * RPMtoRADS; // update agi omega

    // const float body_yaw_offset = imu_data.yaw - geo->yaw_offset;
    const float body_yaw_offset = 0.0f;
    const float cosby = cosf(body_yaw_offset);
    const float sinby = sinf(body_yaw_offset);

    // // Convert body coordinate system to gimbal coordinate system.
    geo->vx = geo->vx_b * cosby + geo->vy_b * -sinby;
    geo->vy = geo->vx_b * sinby + geo->vy_b * cosby;


    float F_y = pid_cycle(&vy_pid, geo->target_vy - geo->vy, CTRL_DELTA_T);
    float F_x = pid_cycle(&vx_pid, geo->target_vx - geo->vx, CTRL_DELTA_T);
    float T_yaw = pid_cycle(&vyaw_pid, geo->target_vyaw - geo->vyaw, CTRL_DELTA_T);
    float T_agi = pid_cycle(&agi_omega_pid, geo->target_agi_omega - geo->agi_omega * M2006_GEAR_RATIO, CTRL_DELTA_T);

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

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID5_8, set_current_M3508(tx_buffer,  
        T_agi*(1.0f/M2006_TORQUE_CONSTANT),
        0.0f,
        0.0f,
        0.0f
    ), 8);

    float estimated_total_power = 0.0f;
    estimated_total_power += m3508_estimate_power(motors.wheel_LF.current, motors.wheel_LF.speed*RPMtoRADS);
    estimated_total_power += m3508_estimate_power(motors.wheel_LB.current, motors.wheel_LB.speed*RPMtoRADS);
    estimated_total_power += m3508_estimate_power(motors.wheel_RF.current, motors.wheel_RF.speed*RPMtoRADS);
    estimated_total_power += m3508_estimate_power(motors.wheel_RB.current, motors.wheel_RB.speed*RPMtoRADS);
    
    vofa.val[0]=imu_data.yaw;
    vofa.val[1]=imu_data.pitch;
    vofa.val[2]=motors.wheel_RF.current;
    vofa.val[3]=motors.wheel_RF.speed*RPMtoRADS;

    // vofa.val[4]=geo->vy;
    // vofa.val[5]=geo->vx;
    // vofa.val[6]=geo->vyaw;
    vofa.val[4] = powermeter_voltage;
    vofa.val[5] = vision_FromRos.packet.forward_vel;
    vofa.val[6] = powermeter_current * powermeter_voltage - 1.2f; // power
    vofa.val[7] = motors.agi.current;

    vofa.val[8] = geo->agi_omega * M2006_GEAR_RATIO;
    vofa.val[9] = estimated_total_power;

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

    case 0x205:
        parse_feedback_M3508(msg, &motors.agi);
        break;

    // here used for powermeter
    case 0x391:
        // handle for voltage
        memcpy(&powermeter_voltage, msg + 0, sizeof(float));
        // handle for current
        memcpy(&powermeter_current, msg + 4, sizeof(float));
        break;

    default:
        break;
    }

    return;
}



#else

PID_t test_omega_pid={
    .P=0.5f,
    .I=5.0f,
    .D=0.0f,
    .integral_max=0.1f
};


// 0x0F is CAN(Slave) and 0x00 is Master
void role_controller_init(){
    HAL_Delay(500);
    fdcanx_send_data(&hfdcan3, 0x05, enable_DM_Joint(motors.test_mtr.tranmitbuf), 8);
}

void role_controller_step(const float CTRL_DELTA_T){

    // pitch_torque = 0.0f;
    // fdcanx_send_data(&hfdcan3, 0x0F, set_torque_DM4310(motors.pitch.tranmitbuf, pitch_torque), 8);

    float test_mtr_target_speed = dr16.channel[3] < -0.5f ? 12.0f : 0.0f;
    float test_err = test_mtr_target_speed - motors.test_mtr.speed;
    float T_test = pid_cycle(&test_omega_pid, test_err, CTRL_DELTA_T);

    fdcanx_send_data(&hfdcan3, 0x05, set_torque_DM4310(motors.test_mtr.tranmitbuf, T_test), 8);

    #include<btb.h>
    b2g_B.self_HP = 30;

    vofa.val[0]=motors.test_mtr.position; 
    vofa.val[1]=motors.test_mtr.speed;
    vofa.val[2]=imu_data.gyro[1]*(PI/180.0f);
    vofa.val[3]=test_mtr_target_speed;
    vofa.val[4]=vision_FromRos.packet.forward_vel;
    vofa.val[5]=vision_FromRos.packet.leftward_vel;
    vofa.val[6]=imu_data.pitch; // -18 deg -> 36 deg, choose 0 10 20 
    vofa.val[7]=(float)dr16.mouse.y;
    // vofa.val[6]=(float)gyro_raw[1];
}
//根据pitch计算前馈torque:y=1.1689x+0.7553;

void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x0D: //Ctrl 0x05
        parse_feedback_DM4310(msg, &motors.test_mtr, 0x05);
        break;
    default:
        break;
    }
    
    


    return;
}

#endif

#endif
