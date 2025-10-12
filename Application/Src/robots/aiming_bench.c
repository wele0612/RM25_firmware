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

    const float target_rpm=500.0f;
    float motor_lf_err=target_rpm-motors.wheel_LF.speed;
    float motor_lb_err=target_rpm-motors.wheel_LB.speed;
    float motor_rf_err=target_rpm-motors.wheel_RF.speed;
    float motor_rb_err=target_rpm-motors.wheel_RB.speed;
    const float lf_torque = pid_cycle(&single_LF_wheel_velocity_pid, motor_lf_err, CTRL_DELTA_T);
    const float lb_torque = pid_cycle(&single_LB_wheel_velocity_pid, motor_lb_err, CTRL_DELTA_T);
    const float rf_torque = pid_cycle(&single_RF_wheel_velocity_pid, motor_rf_err, CTRL_DELTA_T);
    const float rb_torque = pid_cycle(&single_RB_wheel_velocity_pid, motor_rb_err, CTRL_DELTA_T);

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_torque_M3508(tx_buffer,\
        lf_torque, lb_torque, rf_torque, rb_torque), 8);

    vofa.val[0]=motors.wheel_LF.speed;
    vofa.val[1]=target_rpm;
    vofa.val[2]=lf_torque;

    vofa.val[3]=dr16.channel[0];

    vofa.val[4]=imu_data.pitch;


    
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
