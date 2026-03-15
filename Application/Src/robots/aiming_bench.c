#include<config.h>
#ifdef CONFIG_ROBOT_AIMING_BENCH

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<h7can.h>

#ifdef CONFIG_PLATFORM_BASE

PID_t single_LF_wheel_velocity_pid={
    .P=0.007f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t single_LB_wheel_velocity_pid = {
    .P=0.007f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t single_RF_wheel_velocity_pid={
    .P=0.007f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t single_RB_wheel_velocity_pid={
    .P=0.007f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t yaw_pos_pid={
    .P=10.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

void role_controller_init(){
    robot_geo.vehicle_angle = 0.0f;
}

float get_body_omega(){
    //the length of wheel to centre is 25 cm
    //x and y are 17.6777
    float LF_speed=motors.wheel_LF.speed*1.41421f/0.25f;
    float LB_speed=motors.wheel_LB.speed*1.41421f/0.25f;
    float RF_speed=motors.wheel_RF.speed*1.41421f/0.25f;
    float RB_speed=motors.wheel_RB.speed*1.41421f/0.25f;
    
    return (LF_speed+LB_speed+RF_speed+RB_speed)/4.0f;  
}   

void role_controller_step(const float CTRL_DELTA_T){
    uint8_t tx_buffer[8]; 
    const float wheel_radius = 0.154f/2.0f;
    const float target_rpm_coe = 60.0f/(2*PI*wheel_radius)*(1/M3508_GEAR_RATIO);
    // const float robot_side = 0.3536f; 
    // const float one_over_sqrt2 = 0.70710678f;

    robot_geo.target_speed_x = target_rpm_coe*dr16.channel[0]*1.2f;
    robot_geo.target_speed_y = target_rpm_coe*dr16.channel[1]*1.2f;
    robot_geo.target_omega_yaw = 500.0f*dr16.channel[2];
    // robot_geo.target_omega_yaw = 0.0f;

    // expected yaw integration:
    // let target_yaw_pos rad
    // robot_geo.target_yaw_pos += robot_geo.target_omega_yaw / 60.0f * 2.0f * PI * CTRL_DELTA_T; // convert rpm to rad/s

    human_yaw_fix += dr16.channel[2]*0.01f*CTRL_DELTA_T; // 0.1 is the fix coefficient, which is experimentally determined to make the control more stable. It can be tuned or removed.
    float yaw_fix = wrap_to_pi(imu_data.yaw + human_yaw_fix); // 0.1 is the fix coefficient, which is experimentally determined to make the control more stable. It can be tuned or removed.

    if (dr16.s2 == DR16_SWITCH_MID) { // check if in the mode to max speed
        robot_geo.target_yaw_pos += 1.5f * CTRL_DELTA_T; // convert rpm to rad/s
        float target_rad_pos = wrap_to_pi(robot_geo.target_yaw_pos);
        const float pos_error = wrap_to_pi(target_rad_pos - imu_data.yaw);
        robot_geo.target_omega_yaw = 1500.0f + pid_cycle(&yaw_pos_pid, pos_error, CTRL_DELTA_T); // 0.4 rotation/s + pos control
        float vx = cosf(yaw_fix)*robot_geo.target_speed_x + sinf(yaw_fix)*robot_geo.target_speed_y;
        float vy = - sinf(yaw_fix)*robot_geo.target_speed_x + cosf(yaw_fix)*robot_geo.target_speed_y;
        robot_geo.target_speed_x = vx;
        robot_geo.target_speed_y = vy;
    }

    // if (dr16.s2 == DR16_SWITCH_MID) {
    //     //this is the angle of the vehicle, which has a w of 1.5 rad/s
    //     // robot_geo.vehicle_angle += (imu_data.gyro[2]*DEGtoRAD*2.0f*CTRL_DELTA_T);
    //     // // warp to pi:
    //     // robot_geo.vehicle_angle = wrap_to_pi(robot_geo.vehicle_angle);

    //     // float theta = robot_geo.vehicle_angle; 

    //     // find the angular velocity with IMU
    //     float omega = imu_data.gyro[2]*DEGtoRAD;
    //     // accumulate the angle
    //     robot_geo.vehicle_angle += omega * CTRL_DELTA_T;
    //     float theta = wrap_to_pi(robot_geo.vehicle_angle);
    //     // do matrix
    //     float vx = cosf(theta)*robot_geo.target_speed_x + sinf(theta)*robot_geo.target_speed_y;
    //     float vy = - sinf(theta)*robot_geo.target_speed_x + cosf(theta)*robot_geo.target_speed_y;
    //     robot_geo.target_speed_x = vx;
    //     robot_geo.target_speed_y = vy;
    // }

    // requirement: 0.4 rotation/s
    // if (dr16.s2 == DR16_SWITCH_UP) { // turn off everything but rotation
    //     robot_geo.target_speed_x = 0.0f;
    //     robot_geo.target_speed_y = 0.0f;
    //     robot_geo.target_omega_yaw = 0.0f;
    // }

    // 0.4 rotation/s means it is 0.4*2*pi rad/s *1000 => target 
    if (dr16.s2 ==  DR16_SWITCH_UP) {
        robot_geo.target_yaw_pos += 2.513274f * CTRL_DELTA_T; // convert rpm to rad/s
        float target_rad_pos = wrap_to_pi(robot_geo.target_yaw_pos);
        const float pos_error = wrap_to_pi(target_rad_pos - imu_data.yaw);
        robot_geo.target_omega_yaw = 2513.274f + pid_cycle(&yaw_pos_pid, pos_error, CTRL_DELTA_T); // 0.4 rotation/s + pos control
        float vx = cosf(yaw_fix)*robot_geo.target_speed_x + sinf(yaw_fix)*robot_geo.target_speed_y;
        float vy = - sinf(yaw_fix)*robot_geo.target_speed_x + cosf(yaw_fix)*robot_geo.target_speed_y;
        robot_geo.target_speed_x = vx;
        robot_geo.target_speed_y = vy;
    }

    // const float yaw_error = wrap_to_pi(robot_geo.target_yaw_pos - imu_data.yaw); // use imu
    // float yaw_control = pid_cycle(&yaw_pos_pid, yaw_error, CTRL_DELTA_T); // the pos output used as target speed input

    // robot_geo.target_omega_yaw = yaw_control;

    // const float body_radius = 0.25f;

    // const float yaw_omega_to_wheel_rpm = -body_radius * 0.707f / wheel_radius * (1.0f / M3508_GEAR_RATIO) * 60.0f / (2.0f * PI);

    // float target_rpm_lf=robot_geo.target_speed_y  + robot_geo.target_speed_x + robot_geo.target_omega_yaw * yaw_omega_to_wheel_rpm; 
    // float target_rpm_lb=robot_geo.target_speed_y  - robot_geo.target_speed_x + robot_geo.target_omega_yaw * yaw_omega_to_wheel_rpm;
    // float target_rpm_rf=-robot_geo.target_speed_y + robot_geo.target_speed_x + robot_geo.target_omega_yaw * yaw_omega_to_wheel_rpm;
    // float target_rpm_rb=-robot_geo.target_speed_y - robot_geo.target_speed_x + robot_geo.target_omega_yaw * yaw_omega_to_wheel_rpm;  

    float target_rpm_lf=robot_geo.target_speed_y  + robot_geo.target_speed_x + robot_geo.target_omega_yaw; 
    float target_rpm_lb=robot_geo.target_speed_y  - robot_geo.target_speed_x + robot_geo.target_omega_yaw;
    float target_rpm_rf=-robot_geo.target_speed_y + robot_geo.target_speed_x + robot_geo.target_omega_yaw;
    float target_rpm_rb=-robot_geo.target_speed_y - robot_geo.target_speed_x + robot_geo.target_omega_yaw;  

    float motor_lf_err=target_rpm_lf-motors.wheel_LF.speed;
    float motor_lb_err=target_rpm_lb-motors.wheel_LB.speed;
    float motor_rf_err=target_rpm_rf-motors.wheel_RF.speed;
    float motor_rb_err=target_rpm_rb-motors.wheel_RB.speed;

    float lf_current = pid_cycle(&single_LF_wheel_velocity_pid, motor_lf_err, CTRL_DELTA_T) * (1/M3508_TORQUE_CONSTANT);
    float lb_current = pid_cycle(&single_LB_wheel_velocity_pid, motor_lb_err, CTRL_DELTA_T) * (1/M3508_TORQUE_CONSTANT);
    float rf_current = pid_cycle(&single_RF_wheel_velocity_pid, motor_rf_err, CTRL_DELTA_T) * (1/M3508_TORQUE_CONSTANT);
    float rb_current = pid_cycle(&single_RB_wheel_velocity_pid, motor_rb_err, CTRL_DELTA_T) * (1/M3508_TORQUE_CONSTANT);

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(tx_buffer,\
        lf_current, lb_current, rf_current, rb_current), 8);
    
    const float vy_b = (motors.wheel_LF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO +
                        motors.wheel_LB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO -  
                        motors.wheel_RF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO - 
                        motors.wheel_RB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO ) / 4.0f;
    const float vx_b = (motors.wheel_LF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO -
                        motors.wheel_LB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO + 
                        motors.wheel_RF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO - 
                        motors.wheel_RB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO ) / 4.0f;

    const float yaw_angle = imu_data.yaw;
    const float vx_world = cosf(yaw_angle)* vx_b - sinf(yaw_angle)* vy_b;
    const float vy_world = sinf(yaw_angle)* vx_b + cosf(yaw_angle)* vy_b;

    vofa.val[0] = imu_data.yaw;
    vofa.val[1] = vy_b;

    vofa.val[2]=motors.wheel_LF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    vofa.val[3]=motors.wheel_LB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    vofa.val[4]=motors.wheel_RF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    vofa.val[5]=motors.wheel_RB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;

    vofa.val[6]=vx_world;
    vofa.val[7]=vy_world;

    vofa.val[8]=lf_current;
    vofa.val[9]=yaw_fix;
    
    // find single wheel speed:


    // find out the overall car speed:
    // const float overall_speed_x = (vx_lf + vx_lb + vx_rf + vx_rb) / 4.0f;
    // const float overall_speed_y = (vy_lf + vy_lb + vy_rf + vy_rb) / 4.0f;

    // const float world_frame_x = cosf(theta_calc)* overall_speed_x - sinf(theta_calc)* overall_speed_y;
    // const float world_frame_y = sinf(theta_calc)* overall_speed_x + cosf(theta_calc)* overall_speed_y;

    // rpm rot/min => rot*2*pi/60s => *2pi/60 => w 
    // vofa.val[0]=vx_lf / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO; // 轮子线速度 +
    // // vofa.val[1]=motors.wheel_LF.speed / 60.0f * 2.0f * PI * wheel_radius; 
    // // here print the target 
    // // vofa.val[1] = target_rpm_lf / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    // // vofa.val[2]=overall_speed;
    // vofa.val[1]=vy_lf / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO; // +
    // vofa.val[2]=motors.wheel_RF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    // vofa.val[3]=motors.wheel_RB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;

    // // vofa.val[4]=imu_data.gyro[2]*DEGtoRAD;  
    // // vofa.val[5]=robot_geo.vehicle_angle;
    // // vofa.val[6]=get_body_omega();
    // vofa.val[4]=world_frame_x / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    // vofa.val[5]=world_frame_y / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    // vofa.val[6]=0.0f;

    // // vofa.val[7]=motors.wheel_RF.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO; // -
    // // vofa.val[8]=motors.wheel_RB.speed / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    // vofa.val[7]=overall_speed_x / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    // vofa.val[8]=overall_speed_y / 60.0f * 2.0f * PI * wheel_radius * M3508_GEAR_RATIO;
    // vofa.val[9]=robot_config.imu_gyro_offset[2];

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
