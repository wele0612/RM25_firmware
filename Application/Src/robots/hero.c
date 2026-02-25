#include<config.h>

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<h7can.h>

#ifdef CONFIG_ROBOT_HERO

#ifdef CONFIG_PLATFORM_BASE

void role_controller_init(){

}

void dr16_on_change(){
    return;
}

void role_controller_step(const float CTRL_DELTA_T){
    vofa.val[0]=(float)HAL_GetTick();
}


void robot_CAN_msgcallback(int ID, uint8_t *msg){
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

void role_controller_init(){
    robot_geo.target_flywheel_rpm = 0.0f;
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

    if(dr16.s1 == DR16_SWITCH_UP){
        geo->target_flywheel_rpm = 4300.0f;
    }else if((dr16.s1 == DR16_SWITCH_MID)){
        geo->target_flywheel_rpm = 1000.0f;
    }else{
        geo->target_flywheel_rpm = 0.0f;
    }

    if(dr16.s2 == DR16_SWITCH_UP){
        geo->target_feeder_vel = 0.0f;
    }else{
        geo->target_feeder_vel = 3.14f;
    }

    const float flywheel_alpha = 0.2f;
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

    uint8_t tx_buffer[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(
        tx_buffer,
        Tfly_1*(1/M3508_TORQUE_CONSTANT),
        Tfly_2*(1/M3508_TORQUE_CONSTANT),
        Tfly_3*(1/M3508_TORQUE_CONSTANT),
        Tfeeder_1*(1/M2006_TORQUE_CONSTANT)
    ), 8);

    vofa.val[0]=-fmotor.flywheel_1.speed;
    vofa.val[1]=-fmotor.flywheel_2.speed;
    vofa.val[2]=fmotor.flywheel_3.speed;
    vofa.val[3]=geo->feeder_vel;

    vofa.val[4]=Tfeeder_1;

    vofa.val[5]=-fmotor.flywheel_1.current;
    vofa.val[6]=-fmotor.flywheel_2.current;
    vofa.val[7]=fmotor.flywheel_3.current;
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
    default:
        break;
    }

    return;
}

#endif

#endif
