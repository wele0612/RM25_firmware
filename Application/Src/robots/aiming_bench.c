#include<config.h>
#ifdef CONFIG_ROBOT_AIMING_BENCH

#include<application.h>
#include<utils.h>

#include<h7can.h>
#include<motors.h>

#include<robot_arch.h>

extern robot_motors_t motors;

#ifdef CONFIG_PLATFORM_BASE

PID_t single_wheel_demo_pid={
    .P=0.005f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

void role_controller_init(){

}


void role_controller_step(const float CTRL_DELTA_T){
    uint8_t tx_buffer[8];

    const float target_rpm=200.0f;
    float motor_lf_err=target_rpm-motors.wheel_LF.speed;
    const float lf_torque = pid_cycle(&single_wheel_demo_pid, motor_lf_err, CTRL_DELTA_T);

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_torque_M3508(tx_buffer,\
        lf_torque, 0.0f, 0.0f, 0.0f), 8);
    
}

void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x201:
        parse_feedback_M3508(msg, &motors.wheel_LF);
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
