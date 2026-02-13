#include<config.h>

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<h7can.h>

#ifdef CONFIG_ROBOT_HERO

#ifdef CONFIG_PLATFORM_BASE

#endif 
#ifdef CONFIG_PLATFORM_GIMBAL

PID_t flywheel_1_pid={
    .P=0.01f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
};

PID_t flywheel_2_pid={
    .P=0.01f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
};

PID_t flywheel_3_pid={
    .P=0.01f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
};

PID_t feeder_1_vel_pid={
    .P=0.1f,
    .I=0.0f,
    .D=0.000005f,
    .integral_max=0.01f
};

void role_controller_init(){
    robot_geo.target_flywheel_rpm = 0.0f;
}

void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;

    if(dr16.s1 == DR16_SWITCH_UP){
        geo->target_flywheel_rpm = 1000.0f;
    }else{
        geo->target_flywheel_rpm = 0.0f;
    }

    if(dr16.s2 == DR16_SWITCH_UP){
        geo->target_feeder_vel = 3.14f;
    }else if(dr16.s2 == DR16_SWITCH_DOWN){
        geo->target_feeder_vel = -3.14f;
    }else{
        geo->target_feeder_vel = 0.0f;
    }

    float Tfly_1 = pid_cycle(&flywheel_1_pid, -geo->target_flywheel_rpm - motors.flywheel_1.speed, CTRL_DELTA_T);
    float Tfly_2 = pid_cycle(&flywheel_2_pid, -geo->target_flywheel_rpm - motors.flywheel_2.speed, CTRL_DELTA_T);
    float Tfly_3 = pid_cycle(&flywheel_3_pid, geo->target_flywheel_rpm - motors.flywheel_3.speed, CTRL_DELTA_T);

    float feeder_vel = motors.feeder_top.speed*(RPMtoRADS*M2006_GEAR_RATIO);
    const float feeder_vel_alpha = 0.2f;
    geo->feeder_vel = geo->feeder_vel * (1.0f-feeder_vel_alpha) + feeder_vel*feeder_vel_alpha;
    float Tfeeder_1 = pid_cycle(&feeder_1_vel_pid, geo->target_feeder_vel - geo->feeder_vel, CTRL_DELTA_T);

    geo->feeder_position += feeder_vel * CTRL_DELTA_T;

    Tfeeder_1 = limit_val(Tfeeder_1, 0.11f);

    uint8_t tx_buffer[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(
        tx_buffer,
        Tfly_1*(1/M3508_TORQUE_CONSTANT),
        Tfly_2*(1/M3508_TORQUE_CONSTANT),
        Tfly_3*(1/M3508_TORQUE_CONSTANT),
        Tfeeder_1*(1/M2006_TORQUE_CONSTANT)
    ), 8);

    vofa.val[0]=motors.flywheel_1.speed;
    vofa.val[1]=motors.flywheel_2.speed;
    vofa.val[2]=motors.flywheel_3.speed;
    vofa.val[3]=geo->feeder_vel;

    vofa.val[4]=geo->feeder_position;
    vofa.val[5]=Tfeeder_1;
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
    default:
        break;
    }
    
    


    return;
}

#endif

#endif
