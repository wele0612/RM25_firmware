#include<config.h>

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<vision.h>

#include<chasis_power.h>

#include<motors.h>
#include<servo_pwm.h>
#include<btb.h>
#include<h7can.h>

#ifdef CONFIG_ROBOT_HERO

#ifdef CONFIG_PLATFORM_BASE

const float AGI_PER_POS_INCRE = 2*PI/6.0f;
uint32_t base_startup_time = 0;

// 求最近的拨盘起始位置
static inline float get_nearest_agi_reset_pos(float current_pos){
    const float INVERSE_THRE = 0.3f;
    float pos = wrap_to_pi(floorf(current_pos * (1.0f/AGI_PER_POS_INCRE))*AGI_PER_POS_INCRE);
    float pos_diff = wrap_to_pi(current_pos - pos);
    if(pos_diff > INVERSE_THRE){
        pos += AGI_PER_POS_INCRE;
    }
    return pos;
}

static void inline init_agi_damiao(){
    fdcanx_send_data(&hfdcan3, AGI_CTRLID, enable_DM_Joint(motors.agi.tranmitbuf), 8);
}

void role_controller_init(){

    HAL_Delay(1500);
    fdcanx_send_data(&hfdcan3, YAW_CTRLID, enable_DM_Joint(motors.yaw.tranmitbuf), 8);
    HAL_Delay(2);
    init_agi_damiao();

    base_startup_time = HAL_GetTick();

}

PID_t body_x_vel_pid={
    .P = 12.0f,
    .I = 10.0f,
    .D = 0.0f,
    .integral_max = 0.05f
};

PID_t body_y_vel_pid={
    .P = 12.0f,
    .I = 10.0f,
    .D = 0.0f,
    .integral_max = 0.05f
};

PID_t body_yaw_vel_pid={
    .P = 35.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t body_yaw_pos_pid={
    .P = 3.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

PID_t agi_vel_pid={
    .P = 1.3f,
    .I = 50.0f,
    .D = 0.0f,
    .integral_max = 0.07f
};

PID_t agi_pos_pid={
    .P = 40.0f,
    .I = 0.0f,
    .D = 0.0f,
    .integral_max = 0.1f
};

const float gimbal_standby_position = 68.4f*DEGtoRAD;

/* Usually, during folding stage, Yaw should turn to a specific position relative to body.
   However, gimbal could enter a folding stage just after power-up. 
   In this case, Yaw should not turn (even in a folding stage!) until gimbal enter a stable mode.
*/
int enable_folding_fixed_gimbal = 0;

static enum{
    HERO_AGI_INIT,
    HERO_AGI_IDLE,
    HERO_AGI_PUSHING
}agi_state = HERO_AGI_INIT;

static int prev_shoot_clicked = 0;

void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();

    if(HAL_GetTick() % 1000){
        fdcanx_send_data(&hfdcan3, YAW_CTRLID, enable_DM_Joint(motors.yaw.tranmitbuf), 8);
        fdcanx_send_data(&hfdcan3, AGI_CTRLID, enable_DM_Joint(motors.agi.tranmitbuf), 8);
    }

    float m_power = geo->measured_current*geo->measured_voltage;
    const float power_alpha = 0.2f;
    geo->measured_power = power_alpha*m_power + (1.0f-power_alpha)*geo->measured_power;

    const float input_mouse_alpha = 0.02f;

    // const float reduced_pitch_sensitivity_ratio = 0.01f;
    // geo->input_pitch_vel = geo->input_pitch_vel*(1.0f-input_mouse_alpha) + dr16.mouse.y*control_sensitivity*input_mouse_alpha*reduced_pitch_sensitivity_ratio;
    // geo->input_yaw_vel = geo->input_yaw_vel*(1.0f-input_mouse_alpha) + -dr16.mouse.x*control_sensitivity*input_mouse_alpha;

    /* Agitator control */

    /* Base control */

    const float WHEEL_RADIUS = 0.153f/2.0f;
    const float BODY_RADIUS = 0.6f/2.0f;

    const float VEL_RPM_COE = RPMtoRADS*M3508_CUSTOM_GB_RATIO*WHEEL_RADIUS;
    float vel_LF = fmotor.wheel_LF.speed*VEL_RPM_COE;
    float vel_LB = fmotor.wheel_LB.speed*VEL_RPM_COE;
    float vel_RF = fmotor.wheel_RF.speed*VEL_RPM_COE;
    float vel_RB = fmotor.wheel_RB.speed*VEL_RPM_COE;

    geo->vyaw_gyro = -imu_data.gyro[2]*DEGtoRAD;

    /* 
    Note related to the coordinate system: 
    +Y -> Point to the front
    +X -> Point to the right

    This defination is kinda weird but... since the code already works,
    there's no plan to change it, I guess.
    */
    
    /* Forward kinetics of the base */
    float vx_b = -(vel_LF + vel_RF - vel_LB - vel_RB)*(1.4142136f/4.0f);
    float vy_b = -(vel_LF - vel_RF + vel_LB - vel_RB)*(1.4142136f/4.0f);
    float vyaw_wheel = (vel_LF + vel_LB + vel_RF + vel_RB)*(0.25f/BODY_RADIUS);

    const float wheel_v_alpha = 0.2f;
    geo->vx_b = vx_b*wheel_v_alpha + geo->vx_b*(1.0f-wheel_v_alpha);
    geo->vy_b = vy_b*wheel_v_alpha + geo->vy_b*(1.0f-wheel_v_alpha);
    geo->vyaw_wheel = vyaw_wheel*wheel_v_alpha + geo->vyaw_wheel*(1.0f-wheel_v_alpha);

    float body_yaw_offset = wrap_to_pi( -fmotor.yaw.position );

    const float cosby = cosf(body_yaw_offset);
    const float sinby = sinf(body_yaw_offset);

    // Convert body coordinate system to gimbal coordinate system.
    geo->vx = geo->vx_b * cosby + geo->vy_b * -sinby;
    geo->vy = geo->vx_b * sinby + geo->vy_b * cosby;

    geo->target_vx = -chasis_ctrl.robot_leftward_v*1e-3f;
    geo->target_vy = chasis_ctrl.robot_forward_v*1e-3f;
    // geo->target_vyaw = chasis_ctrl.robot_yaw_omega*1e-3f;
    if(BTB_ONLINE && remote_online()){
        if(chasis_ctrl.chasis_yaw_follow){
            float body_ori_err = -wrap_to_pi(body_yaw_offset);
            geo->target_vyaw = pid_cycle(&body_yaw_pos_pid, body_ori_err, CTRL_DELTA_T);
            geo->target_vyaw = limit_val(geo->target_vyaw, 2.0f);
        }else{
            geo->target_vyaw = chasis_ctrl.spintop_level*PI;
        }
    }else{
        geo->target_vyaw = 0.0f;
    }  

    geo->agi_pos = fmotor.agi.position;
    geo->agi_vel = fmotor.agi.speed;

    // geo->target_agi_pos = get_nearest_agi_reset_pos(geo->agi_pos);
    int agi_in_position = (fabsf(wrap_to_pi(geo->target_agi_pos - geo->agi_pos)) < (10.0f*DEGtoRAD));

    int shoot_is_posedge = (chasis_ctrl.fire_pressed && (!prev_shoot_clicked));
    prev_shoot_clicked = chasis_ctrl.fire_pressed;

    if(!referee.robot_status_0x0201.power_management_shooter_output){
        agi_state = HERO_AGI_INIT;
    }
    
    switch (agi_state) {
        case HERO_AGI_INIT:
            geo->target_agi_pos = geo->agi_pos;
            if(referee.robot_status_0x0201.power_management_shooter_output && shoot_is_posedge){
                agi_state = HERO_AGI_IDLE;
            }
            break;

        case HERO_AGI_IDLE:
            geo->target_agi_pos = get_nearest_agi_reset_pos(geo->agi_pos);
            if(shoot_is_posedge){
                agi_state = HERO_AGI_PUSHING;
                geo->target_agi_pos = get_nearest_agi_reset_pos(geo->agi_pos + PI/3.0f);
            }
            break;

        case HERO_AGI_PUSHING:
            if(agi_in_position){
                agi_state = HERO_AGI_IDLE;
            }
            break;

        default:
            break;
    }

    // geo->target_agi_pos = unit_step_generator((float)(HAL_GetTick()%2000), 1000.0f)*1.5f;
    geo->target_agi_vel = pid_cycle(&agi_pos_pid, wrap_to_pi(geo->target_agi_pos - geo->agi_pos), CTRL_DELTA_T);
    
    // geo->target_agi_vel = unit_step_generator((float)(HAL_GetTick()%2000), 1000.0f)*3.0f;
    geo->target_agi_vel = limit_val(geo->target_agi_vel, 30.0f);
    geo->T_agi = pid_cycle(&agi_vel_pid, geo->target_agi_vel - geo->agi_vel, CTRL_DELTA_T);


    geo->F_x = pid_cycle(&body_x_vel_pid, geo->target_vx - geo->vx, CTRL_DELTA_T);
    geo->F_y = pid_cycle(&body_y_vel_pid, geo->target_vy - geo->vy, CTRL_DELTA_T);
    geo->T_base_yaw = pid_cycle(&body_yaw_vel_pid, geo->target_vyaw - geo->vyaw_wheel, CTRL_DELTA_T);
    
    geo->F_x = limit_val(geo->F_x, 9.0f);
    geo->F_y = limit_val(geo->F_y, 9.0f);
    geo->T_base_yaw = limit_val(geo->T_base_yaw, 17.0f);
    
    // geo->F_x = geo->target_vx*0.2f;
    // geo->F_y = geo->target_vy*0.2f;
    // geo->T_base_yaw = geo->target_vyaw*0.5f;

    geo->F_x_b = geo->F_x * cosby + geo->F_y * sinby;
    geo->F_y_b = geo->F_x * -sinby + geo->F_y * cosby;

    float F_single_wheel_turn = -geo->T_base_yaw*(0.25f/BODY_RADIUS);
    float T_single_wheel_turn = F_single_wheel_turn*WHEEL_RADIUS;
    float force_vector_coe = 1.4142136f*0.25f;
    geo->T_LF =  (geo->F_y_b + geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    geo->T_LB =  (geo->F_y_b - geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    geo->T_RF = (-geo->F_y_b + geo->F_x_b)*force_vector_coe + T_single_wheel_turn;
    geo->T_RB = (-geo->F_y_b - geo->F_x_b)*force_vector_coe + T_single_wheel_turn;

    float chasis_currents[4] = {
        geo->T_LF*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
        geo->T_LB*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
        geo->T_RF*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
        geo->T_RB*(-1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB)
    };

    float chasis_omegas[4] = {
        motors.wheel_LF.speed*RPMtoRADS,
        motors.wheel_LB.speed*RPMtoRADS,
        motors.wheel_RF.speed*RPMtoRADS,
        motors.wheel_RB.speed*RPMtoRADS
    };

    float chasis_power_limit = (float)(referee.robot_status_0x0201.chassis_power_limit);
    if(chasis_power_limit > 200.0f ){ chasis_power_limit = 200.0f;}
    if(chasis_power_limit < 25.0f ){chasis_power_limit = 25.0f;}
    float chasis_current_scaling = m3508_quadwheel_get_scaling(
        chasis_currents, chasis_omegas, chasis_power_limit - 10.0f);

    uint8_t tx_buf[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(tx_buf,
        chasis_currents[0]*chasis_current_scaling,
        chasis_currents[1]*chasis_current_scaling,
        chasis_currents[2]*chasis_current_scaling,
        chasis_currents[3]*chasis_current_scaling), 8);

    if(BTB_ONLINE){
        geo->T_yaw=limit_val(g2b_A.gimbal_request_T_yaw*1e-3f, 10.0f);
    }else{
        geo->T_yaw=0.0f;
    }
    fdcanx_send_data(&hfdcan3, YAW_CTRLID, set_torque_DM4310(motors.yaw.tranmitbuf, geo->T_yaw), 8);

    // geo->T_agi=0.0f;
    fdcanx_send_data(&hfdcan3, AGI_CTRLID, set_torque_DM4310(motors.agi.tranmitbuf, geo->T_agi), 8);

    float estimated_total_power = 0.0f;
    estimated_total_power += m3508_estimate_power(chasis_currents[0], motors.wheel_LF.speed*RPMtoRADS);
    estimated_total_power += m3508_estimate_power(chasis_currents[1], motors.wheel_LB.speed*RPMtoRADS);
    estimated_total_power += m3508_estimate_power(chasis_currents[2], motors.wheel_RF.speed*RPMtoRADS);
    estimated_total_power += m3508_estimate_power(chasis_currents[3], motors.wheel_RB.speed*RPMtoRADS);

    static float ob_fx;
    static float ob_fy;
    const float ob_alpha=0.05f;
    ob_fx = ob_fx*(1.0f-ob_alpha) + geo->F_x*ob_alpha;
    ob_fy = ob_fy*(1.0f-ob_alpha) + geo->F_y*ob_alpha;
    vofa.val[0]=sqrtf(imu_data.acc[0]*imu_data.acc[0] +
        imu_data.acc[1]*imu_data.acc[1] + imu_data.acc[2]*imu_data.acc[2]);
    vofa.val[1]=geo->agi_pos;
    vofa.val[2]=geo->target_agi_pos;

    vofa.val[3]=chasis_ctrl.fire_pressed;
    // vofa.val[0] = fmotor.wheel_LF.current;
    // vofa.val[1] = fmotor.wheel_LB.current;
    // vofa.val[2] = fmotor.wheel_RF.current;
    // vofa.val[3] = fmotor.wheel_RB.current;

    vofa.val[4]=geo->vy;
    vofa.val[5]=geo->vyaw_gyro*(60.0f/(2.0f*PI));

    vofa.val[6]=(float)(referee.robot_status_0x0201.chassis_power_limit);
    vofa.val[7]=dr16.mouse.x;
    vofa.val[8]=dr16.key.v;
    vofa.val[9]=geo->measured_power;
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
    case G2B_MSG_B_ID:
        #ifndef CONFIG_BASE_HAS_CONTROL
        memcpy(&chasis_ctrl, msg, 8);
        BTB_UPDATE_CNTDOWN();
        #endif
        break;
    case 0x391:
        // handle for voltage
        memcpy(&robot_geo.measured_voltage, msg + 0, sizeof(float));
        // handle for current
        memcpy(&robot_geo.measured_current, msg + 4, sizeof(float));
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

PID_t g_yaw_vel_pid={
    .P=3.0f,
    .I=30.0f,
    .D=0.007f,
    .integral_max=0.05f
};

PID_t g_yaw_pos_pid={
    .P=22.0f,
    .I=0.0f,
    .D=0.7f,
    .integral_max=0.01f
};

// uint8_t reset_zeropoint[8] = {0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void role_controller_init(){

}

// void dr16_on_change(){
//     // robot_ctrl_t *geo = &robot_geo;

//     // if(dr16.s2 == DR16_SWITCH_MID && dr16.previous.s2 == DR16_SWITCH_DOWN){
//     //     geo->feeder_position = 0.0f;
//     //     geo->target_feeder_position = 1.7f;
//     // }
    
// }

/* For X4-36, need to first set VEL_P to 0.015, VEL_I to 0.00005 */
/* Also need to increase X4-36 max acceleration */
void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();

    const float PITCH_MTR_MINIMUM = -2.1f;
    const float PITCH_MTR_MAXIMUM = -1.2f;
    geo->mtr_pitch_pos = -fmotor.pitch.precise_position;
    geo->mtr_pitch_vel = fmotor.pitch.speed;

    float min_pitch_vel = linear_map(geo->mtr_pitch_pos,
        PITCH_MTR_MINIMUM, PITCH_MTR_MINIMUM + 0.2f, 0.05f, -3.0f);

    float max_pitch_vel = linear_map(geo->mtr_pitch_pos,
        PITCH_MTR_MAXIMUM - 0.2f, PITCH_MTR_MAXIMUM, 3.0f, -0.05f);

    float V_pitch = gimbal_ctrl.gimbal_pitch_omega*1e-3f;
    V_pitch = fmaxf(min_pitch_vel, V_pitch);
    V_pitch = fminf(max_pitch_vel, V_pitch);

    geo->abs_pitch_pos = imu_data.pitch;
    geo->abs_pitch_vel = imu_data.gyro[1]*DEGtoRAD;

    geo->abs_yaw_pos = imu_data.yaw;
    geo->abs_yaw_vel = imu_data.gyro[2]*DEGtoRAD;

    float input_yaw_vel = gimbal_ctrl.gimbal_yaw_omega*1e-3f;
    geo->target_yaw_pos += input_yaw_vel*CTRL_DELTA_T;
    // geo->target_yaw_pos = unit_step_generator(input_yaw_vel, 1.0f)*0.7f;
    if(!BTB_ONLINE){
        geo->target_yaw_pos = geo->abs_yaw_pos;
    }
    geo->target_yaw_vel = pid_cycle(&g_yaw_pos_pid, wrap_to_pi(geo->target_yaw_pos - geo->abs_yaw_pos), CTRL_DELTA_T);
    geo->target_yaw_vel = limit_val(geo->target_yaw_vel, 12.0f) + input_yaw_vel;

    // geo->target_yaw_vel = unit_step_generator(input_yaw_vel, 1.0f)*1.2f;

    float T_yaw = pid_cycle(&g_yaw_vel_pid, geo->target_yaw_vel - geo->abs_yaw_vel, CTRL_DELTA_T);


    if(BTB_ONLINE){
        if(gimbal_ctrl.flywheel_enabled){
            geo->target_flywheel_rpm = 3800.0f;            
        }else{
            geo->target_flywheel_rpm = 800.0f;
        }
    }else{
        geo->target_flywheel_rpm = 60.0f;
    }

            // geo->target_pitch_vel = pid_cycle(&f_pitch_pos_pid, 
            //     2.0f*DEGtoRAD - geo->mtr_pitch_pos, CTRL_DELTA_T);
            
            // limit_val(geo->target_pitch_vel, 1.5f);
            // fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
            //     set_speed_MyAct(motors.pitch.tranmitbuf, -geo->target_pitch_vel*RADtoDEG, 0), 8);


    const float GIMBAL_WEIGHT = 4.0f;
    const float GIMBAL_CENTRE_OF_MASS_DISTENCE = 0.12f;
    float gravity_feedforward = cosf(geo->abs_pitch_pos)*(9.81f*GIMBAL_WEIGHT*GIMBAL_CENTRE_OF_MASS_DISTENCE);
    geo->T_pitch += gravity_feedforward;
    
    // float Tfly_1 = pid_cycle(&flywheel_1_pid, -geo->target_flywheel_rpm - geo->f1vel_filtered, CTRL_DELTA_T);
    float Tfly_1 = pid_cycle(&flywheel_1_pid, -geo->target_flywheel_rpm - fmotor.flywheel_1.speed, CTRL_DELTA_T);
    float Tfly_2 = pid_cycle(&flywheel_2_pid, -geo->target_flywheel_rpm - fmotor.flywheel_2.speed, CTRL_DELTA_T);
    float Tfly_3 = pid_cycle(&flywheel_3_pid, geo->target_flywheel_rpm - fmotor.flywheel_3.speed, CTRL_DELTA_T);

    
    
    uint8_t tx_buffer[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(
        tx_buffer,
        Tfly_1*(1/M3508_TORQUE_CONSTANT),
        Tfly_2*(1/M3508_TORQUE_CONSTANT),
        Tfly_3*(1/M3508_TORQUE_CONSTANT),
        0.0f
    ), 8);

    if(BTB_ONLINE){
        g2b_A.gimbal_request_T_yaw = (int16_t)limit_val((1e3f * T_yaw), 32700.0f);
        fdcanx_send_data(&hfdcan1, G2B_MSG_A_ID, (uint8_t *)&g2b_A, 8);
    }

    fdcanx_send_data(&hfdcan3, MYACT_CTRLID_SINGLE+0x5, 
        set_speed_MyAct(motors.pitch.tranmitbuf, -V_pitch*RADtoDEG, 50), 8);

    fdcanx_send_data(&hfdcan3, MYACT_CTRLID_ALL, 
        acquire_motor_angle_MyAct(motors.pitch.tranmitbuf), 8);

    vofa.val[0]=-fmotor.flywheel_1.speed;

    vofa.val[1]=gimbal_ctrl.gimbal_yaw_omega*1e-3f;
    vofa.val[2]=gimbal_ctrl.gimbal_pitch_omega*1e-3f;

    vofa.val[3]=geo->mtr_pitch_pos;
    vofa.val[4]=geo->mtr_pitch_vel;
    
    vofa.val[5]=V_pitch;
    vofa.val[6]=geo->target_yaw_vel;
    // vofa.val[6]=predict_distence;

    vofa.val[7]=geo->abs_yaw_vel;
    vofa.val[8]=geo->abs_yaw_pos;
    vofa.val[9]=geo->target_yaw_pos;
    // vofa.val[9]=geo->abs_pitch_pos - predict_pitch;
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
    case MYACT_RPTID+0x5: 
        parse_feedback_X4_36(msg, &motors.pitch);
        break;
    case B2G_MSG_A_ID:
        ;
        gimbal_ctrl_input_t* ctrl_msg = (void *)msg;
        if (ctrl_msg->gimbal_use_VTM_not_dr16){
            gimbal_ctrl.gimbal_use_VTM_not_dr16 = 1;
        }else{
            memcpy(&gimbal_ctrl, msg, 8);
        }
        BTB_UPDATE_CNTDOWN();
        break;
    default:
        ;
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
