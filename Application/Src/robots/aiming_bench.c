#include<config.h>
#ifdef CONFIG_ROBOT_AIMING_BENCH

#include<application.h>
#include<utils.h>
#include<global_variables.h>

#include<h7can.h>

#ifdef CONFIG_PLATFORM_BASE

PID_t single_LF_wheel_velocity_pid={
    .P=0.005f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t single_LB_wheel_velocity_pid = {
    .P=0.005f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t single_RF_wheel_velocity_pid={
    .P=0.005f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t single_RB_wheel_velocity_pid={
    .P=0.005f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

void role_controller_init(){

}


void role_controller_step(const float CTRL_DELTA_T){
    uint8_t tx_buffer[8]; 
    const float wheel_radius = 0.154f;

    const float target_rpm_coe = 60.0f/(2*PI*wheel_radius)*(1/M3508_GEAR_RATIO);

    robot_geo.target_speed_x = target_rpm_coe*dr16.channel[0]*1.0f;
    robot_geo.target_speed_y = target_rpm_coe*dr16.channel[1]*1.0f;
    robot_geo.target_omega_yaw = 500.0f*dr16.channel[2];

    const float target_rpm_lf=robot_geo.target_speed_y + robot_geo.target_speed_x + robot_geo.target_omega_yaw; 
    const float target_rpm_lb=robot_geo.target_speed_y - robot_geo.target_speed_x + robot_geo.target_omega_yaw;
    const float target_rpm_rf=-robot_geo.target_speed_y + robot_geo.target_speed_x + robot_geo.target_omega_yaw;
    const float target_rpm_rb=-robot_geo.target_speed_y - robot_geo.target_speed_x + robot_geo.target_omega_yaw;   

    float motor_lf_err=target_rpm_lf-motors.wheel_LF.speed;
    float motor_lb_err=target_rpm_lb-motors.wheel_LB.speed;
    float motor_rf_err=target_rpm_rf-motors.wheel_RF.speed;
    float motor_rb_err=target_rpm_rb-motors.wheel_RB.speed;
    const float lf_torque = pid_cycle(&single_LF_wheel_velocity_pid, motor_lf_err, CTRL_DELTA_T);
    const float lb_torque = pid_cycle(&single_LB_wheel_velocity_pid, motor_lb_err, CTRL_DELTA_T);
    const float rf_torque = pid_cycle(&single_RF_wheel_velocity_pid, motor_rf_err, CTRL_DELTA_T);
    const float rb_torque = pid_cycle(&single_RB_wheel_velocity_pid, motor_rb_err, CTRL_DELTA_T);

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_torque_M3508(tx_buffer,\
        lf_torque, lb_torque, rf_torque, rb_torque), 8);

    vofa.val[0]=imu_data.yaw / PI * 180;
    vofa.val[1]=imu_data.pitch / PI * 180;
    vofa.val[2]=imu_data.roll / PI * 180;
    vofa.val[3]=imu_data.gyro[1];

    vofa.val[4]=dr16.channel[0];
    vofa.val[5]=dr16.channel[1];
    vofa.val[6]=dr16.channel[2];
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

void role_controller_init(){

}

void role_controller_step(const float CTRL_DELTA_T){

}

#endif

#endif
