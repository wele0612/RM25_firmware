#include<config.h>

#include<application.h>
#include<utils.h>
#include<global_variables.h>
#include<stdlib.h>

#include<vision.h>

#include<chasis_power.h>
#include<supercap.h>

#include<motors.h>
#include<btb.h>
#include<h7can.h>

#ifdef CONFIG_ROBOT_SENTRY_OMNI

const int firing_table[7][3]={ 
        //  x   y   length   
            50,-90,180,
            50,-66, 120,
            50,-58,70,
            50,-63,45,
            50,  -80,27,
            50,  -100,20,
            50,  -140,13
        };


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

PID_t body_x_vel_pid={
    .P = 9.0f,
    .I = 7.0f,
    .D = 0.0f,
    .integral_max = 0.05f
};

PID_t body_y_vel_pid={
    .P = 9.0f,
    .I = 7.0f,
    .D = 0.0f,
    .integral_max = 0.05f
};

PID_t body_yaw_vel_pid={
    .P = 30.0f,
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
    .P=0.5f,
    .I=5.0f,
    .D=0.0f,
    .integral_max=0.5f
};

enum{
    AGI_NORMAL,
    AGI_STALLING,
    AGI_RECOVERING
} agi_state;
uint32_t agi_state_last_enter_time=0;

static int prev_shoot_clicked = 0;

void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();

    if(HAL_GetTick() % 500){
        fdcanx_send_data(&hfdcan3, YAW_CTRLID, enable_DM_Joint(motors.yaw.tranmitbuf), 8);
        fdcanx_send_data(&hfdcan3, AGI_CTRLID, enable_DM_Joint(motors.agi.tranmitbuf), 8);
    }

    float m_power = geo->measured_current*geo->measured_voltage;
    const float power_alpha = 0.2f;
    geo->measured_power = power_alpha*m_power + (1.0f-power_alpha)*geo->measured_power;

    /* Base control */

    const float WHEEL_RADIUS = 0.153f/2.0f;
    const float BODY_RADIUS = 0.475f/2.0f; // TODO: Change this later

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

    int sentry_but_match_not_started = chasis_ctrl.L5_auto_drive &&
        referee.game_status_0x0001.game_progress != 4;

    if(sentry_but_match_not_started){
        geo->target_vx = 0.0f;
        geo->target_vy = 0.0f;
    }else{
        geo->target_vx = -chasis_ctrl.robot_leftward_v*1e-3f;
        geo->target_vy = chasis_ctrl.robot_forward_v*1e-3f;
    }
    
    // geo->target_vyaw = chasis_ctrl.robot_yaw_omega*1e-3f;
    if(BTB_ONLINE && control_online()){
        if(chasis_ctrl.spin_mode == 1){
            float body_ori_err = -wrap_to_pi(body_yaw_offset);
            geo->target_vyaw = pid_cycle(&body_yaw_pos_pid, body_ori_err, CTRL_DELTA_T);
            geo->target_vyaw = limit_val(geo->target_vyaw, 2.0f);
        }else if(chasis_ctrl.spin_mode == 2){
            geo->target_vyaw = 2.0f*PI;
        }else if(chasis_ctrl.spin_mode == 3){
            if(sentry_but_match_not_started){
                geo->target_vyaw = 0.5f*PI;
            }else if( fabsf(geo->target_vx)<= 0.2f && fabsf(geo->target_vy)<=0.2f ){
                geo->target_vyaw = 4.0f*PI 
                    + cosf(1e-3f*HAL_GetTick()*PI*1.5f)*(PI);
            }else{
                    geo->target_vyaw = 2.0f*PI;
            }
        }else{
            geo->target_vyaw = 0.0f;
        }
    }else{
        geo->target_vyaw = 0.0f;
    }  

    geo->agi_pos = fmotor.agi.position;
    // geo->agi_vel = (1.0f - agi_vel_alpha)*geo->agi_vel + fmotor.agi.speed*agi_vel_alpha;
    geo->agi_vel = fmotor.agi.speed;

    const float agi_anti_blocking_alpha = 0.05f;
    geo->filtered_agi_vel = (1.0f-agi_anti_blocking_alpha)*geo->filtered_agi_vel 
        + agi_anti_blocking_alpha*geo->agi_vel;
    geo->filtered_T_agi = (1.0f-agi_anti_blocking_alpha)*geo->filtered_T_agi 
        + agi_anti_blocking_alpha*fmotor.agi.torque_actual;

    int agi_probably_block = fabsf(geo->filtered_agi_vel)<0.1f && fabsf(geo->filtered_T_agi)>2.0f;
    switch (agi_state){
        case AGI_NORMAL:
            if(agi_probably_block){
                agi_state = AGI_STALLING;
                agi_state_last_enter_time = HAL_GetTick();
            }
            break;
        case AGI_STALLING:
            if(!agi_probably_block){
                agi_state = AGI_NORMAL;
                agi_state_last_enter_time = HAL_GetTick();
            }else if(HAL_GetTick() - agi_state_last_enter_time > 500){
                agi_state = AGI_RECOVERING;
                agi_state_last_enter_time = HAL_GetTick();
            }
            break;
        case AGI_RECOVERING:
            if(HAL_GetTick() - agi_state_last_enter_time > 500){
                agi_state = AGI_NORMAL;
                agi_state_last_enter_time = HAL_GetTick();
            }
            break;
        default:
            agi_state = AGI_NORMAL;
            break;
    }

    // geo->target_agi_pos = get_nearest_agi_reset_pos(geo->agi_pos);
    // int agi_in_position = (fabsf(wrap_to_pi(geo->target_agi_pos - geo->agi_pos)) < (10.0f*DEGtoRAD));

    // int shoot_is_posedge = 0;
    // if(chasis_ctrl.vision_allow_fire){
    //     shoot_is_posedge = (chasis_ctrl.fire_pressed && (!prev_shoot_clicked));
    //     prev_shoot_clicked = chasis_ctrl.fire_pressed;
    // }

    int remain_heat = referee.robot_status_0x0201.shooter_barrel_heat_limit
        - referee.power_heat_data_0x0202.shooter_17mm_barrel_heat;
    int heat_control_allow_shoot =
        remain_heat >= 29
        || chasis_ctrl.bypass_shoot_heat_control;

    int heat_control_reduce_speed = remain_heat < 80;

    if(heat_control_allow_shoot 
        && chasis_ctrl.fire_pressed 
        && chasis_ctrl.vision_allow_fire
        && !sentry_but_match_not_started){

        if(heat_control_reduce_speed){
            geo->target_agi_vel = 1.0f*PI;
        }else{
            geo->target_agi_vel = 4.0f*PI;
        }
    }else{
        geo->target_agi_vel = 0.0f;
    }

    if(chasis_ctrl.agi_anti_blocking || agi_state == AGI_RECOVERING){
        geo->T_agi = -0.5f;
    }else{
        geo->T_agi = pid_cycle(&agi_vel_pid, geo->target_agi_vel - geo->agi_vel, CTRL_DELTA_T);
        if(geo->target_agi_vel > 2.0f){
            geo->T_agi += 3.0f;
        }
    }
    
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

    float boost_power = 0.0f;
    if(chasis_ctrl.supercap_discharge && supercap_online() && supercap.cap_state==CAP_ON){
        boost_power = fminf(100.0f, supercap.max_discharge_power*1e-2f);
    }

    float chasis_current_scaling = m3508_quadwheel_get_scaling(
        chasis_currents, chasis_omegas, chasis_power_limit - 10.0f + boost_power);

    uint8_t tx_buf[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, set_current_M3508(tx_buf,
        chasis_currents[0]*chasis_current_scaling,
        chasis_currents[1]*chasis_current_scaling,
        chasis_currents[2]*chasis_current_scaling,
        chasis_currents[3]*chasis_current_scaling), 8);

    if(BTB_ONLINE){
        geo->T_yaw=limit_val(g2b_A.gimbal_request_T_yaw*1e-3f, 10.0f);
        // geo->T_yaw += friction_compensation(fmotor.yaw.speed, 0.1f, 0.5f);
    }else{
        geo->T_yaw=0.0f;
    }
    fdcanx_send_data(&hfdcan3, YAW_CTRLID, set_torque_DM4310(motors.yaw.tranmitbuf, geo->T_yaw), 8);

    // geo->T_agi=0.0f;
    fdcanx_send_data(&hfdcan3, AGI_CTRLID, set_torque_DM4310(motors.agi.tranmitbuf, geo->T_agi), 8);

    b2g_B.gimbal_mtr_yaw_pos = (int16_t)(wrap_to_pi(fmotor.yaw.position)*1e4f);
    b2g_B.feedback_shoot_speed = (int16_t)(referee.shoot_data_0x0207.initial_speed*1e3f);
    b2g_B.is_enemy_red = (referee.robot_status_0x0201.robot_id >= 100)? 1:0;
    b2g_B.self_HP = referee.robot_status_0x0201.current_HP;
    b2g_B.match_started = (referee.game_status_0x0001.game_progress == 4);
    fdcanx_send_data(&hfdcan1, B2G_MSG_B_ID, (uint8_t *)&b2g_B, 8);

    capcan_toCap_t cap_msg;
    cap_msg.power_target = (uint16_t)((chasis_power_limit - 3.0f)*100.0f);
    cap_msg.referee_power = (uint16_t)chasis_power_limit;
    cap_msg.rsvd1 = 0x2012;
    cap_msg.rsvd2 = 0x0712;
    fdcanx_send_data(&hfdcan1, CAPCAN_TOCAP_MSG_ID, (uint8_t *)&cap_msg, 8);

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
    vofa.val[0]=remain_heat;
    vofa.val[1]=geo->filtered_agi_vel;
    vofa.val[2]=geo->target_agi_vel;
    vofa.val[3]=geo->filtered_T_agi;
    vofa.val[4]=agi_probably_block;
    vofa.val[5]=agi_state;

    vofa.val[6]=(float)(referee.robot_status_0x0201.chassis_power_limit);
    vofa.val[7]=supercap.cap_energy_percentage;
    vofa.val[8]=supercap.base_power*1e-2f;
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
    case CAPCAN_FROMCAP_MSG_ID:
        memcpy(&supercap, msg, 8);
        supercap_update_timeout();
        break;
    case G2B_MSG_A_ID:
        memcpy(&g2b_A, msg, 8);
        BTB_UPDATE_CNTDOWN();
        break;
    case G2B_MSG_B_ID:
        memcpy(&chasis_ctrl, msg, 8);
        control_timeout_update();
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
    .P=0.005f,
    .I=0.15f,
    .D=0.000005f,
    .integral_max=10.0f
};

PID_t flywheel_2_pid={
    .P=0.005f,
    .I=0.15f,
    .D=0.000005f,
    .integral_max=10.0f
};

// ------------ For gimbal mode ------------
PID_t g_pitch_vel_pid={
    .P=2.5f,
    .I=50.0f,
    .D=0.0f,
    .integral_max=0.03f
}; 

PID_t g_pitch_pos_pid={
    .P=16.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t g_yaw_vel_pid={
    .P=2.2f,
    .I=65.0f,
    .D=0.0f,
    .integral_max=0.01f
};

PID_t g_yaw_pos_pid={
    .P=16.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.01f
};

// uint8_t reset_zeropoint[8] = {0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void role_controller_init(){
    HAL_Delay(500);
    fdcanx_send_data(&hfdcan3, PITCH_CTRLID, enable_DM_Joint(motors.pitch.tranmitbuf), 8);
}

static int turning_back=0;
static uint32_t turnback_start_tick=0;
static float turnback_end_pos;

static uint32_t last_recognized_target_tick=0;

void role_controller_step(const float CTRL_DELTA_T){
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;
    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();

    if(HAL_GetTick() % 1000){
        fdcanx_send_data(&hfdcan3, PITCH_CTRLID, enable_DM_Joint(motors.pitch.tranmitbuf), 8);
    }

    const float PITCH_MTR_MINIMUM = 0.1f;
    const float PITCH_MTR_MAXIMUM = 0.9f;
    geo->mtr_pitch_pos = -fmotor.pitch.position;
    geo->mtr_pitch_vel = -fmotor.pitch.speed;

    geo->abs_pitch_pos = -imu_data.pitch;
    geo->abs_pitch_vel = -imu_data.gyro[1]*DEGtoRAD;

    geo->abs_yaw_pos = imu_data.yaw;
    geo->abs_yaw_vel = imu_data.gyro[2]*DEGtoRAD;

    int gimbal_controlled_by_vision = 
        (gimbal_ctrl.gimbal_control_mode == 2) 
        && vision_online() 
        && vision_FromRos.packet.mode != 0;

    float input_pitch_vel = gimbal_ctrl.gimbal_pitch_omega*1e-3f;
    float input_yaw_vel = gimbal_ctrl.gimbal_yaw_omega*1e-3f;
    
    const float input_gim_alpha = 0.05f;
    geo->input_pitch_vel = (1.0f-input_gim_alpha)*geo->input_pitch_vel + input_gim_alpha*input_pitch_vel;
    geo->input_yaw_vel = (1.0f-input_gim_alpha)*geo->input_yaw_vel + input_gim_alpha*input_yaw_vel;

    float yaw_vel_feedforward;
    float yaw_acc_feedforward;
    float pitch_vel_feedforward;
    float pitch_acc_feedforward;
    
    if(gimbal_controlled_by_vision){
        geo->target_pitch_pos = -vision_FromRos.packet.pitch;
        pitch_vel_feedforward = -vision_FromRos.packet.pitch_vel;
        pitch_acc_feedforward = -vision_FromRos.packet.pitch_acc;

        geo->target_yaw_pos = vision_FromRos.packet.yaw;
        yaw_vel_feedforward = vision_FromRos.packet.yaw_vel;
        yaw_acc_feedforward = vision_FromRos.packet.yaw_acc;

        turning_back = 0;
    }else{
        // geo->target_pitch_pos = unit_step_generator(input_pitch_vel, 1.0f)*0.3f;
        // geo->target_pitch_pos += geo->input_pitch_vel*CTRL_DELTA_T;
        geo->target_pitch_pos = geo->abs_pitch_pos;
        // pitch_vel_feedforward = unit_step_generator(input_pitch_vel, 1.0f)*1.0f;
        pitch_vel_feedforward = geo->input_pitch_vel;
        // pitch_vel_feedforward = 0.0f;
        pitch_acc_feedforward = 0.0f;

        // geo->target_yaw_pos += geo->input_yaw_vel*CTRL_DELTA_T;
        // geo->target_yaw_pos = unit_step_generator(input_yaw_vel, 0.2f)*1.0f;
        // yaw_vel_feedforward = 0.0f;
        geo->target_yaw_pos = geo->abs_yaw_pos;
        yaw_vel_feedforward = geo->input_yaw_vel;
        yaw_acc_feedforward = 0.0f;

        if(gimbal_ctrl.swap_head_tail && !turning_back){
            turning_back = 1;
            turnback_end_pos = geo->abs_yaw_pos + PI;
            turnback_start_tick = HAL_GetTick();
        }else if(turning_back){
            yaw_vel_feedforward = 6.0f;
            
            int turnback_completed = fabsf(wrap_to_pi(turnback_end_pos - geo->abs_yaw_pos)) < 0.2f;
            int turnback_timeout = (HAL_GetTick() - turnback_start_tick) > 2000;
            if(turnback_completed || turnback_timeout){
                turning_back = 0;
            }
        }

        if(chasis_ctrl.L5_auto_drive){
            // Sentry scanning mode

            if(vision_FromRos.packet.mode != 0 && vision_online()){
                last_recognized_target_tick = HAL_GetTick();
            }

            geo->target_pitch_pos = cosf(1e-3f*3.5f*PI*HAL_GetTick())*0.15f - 0.25f;
            // if(HAL_GetTick() - last_recognized_target_tick < 2000){
            //     yaw_vel_feedforward = 0.0f;
            // }else{
            yaw_vel_feedforward = -PI*1.0f;
            // }
            
        }

    }

    const float down_range = geo->mtr_pitch_pos - PITCH_MTR_MINIMUM;
    const float up_range = PITCH_MTR_MAXIMUM - geo->mtr_pitch_pos;

    if(geo->target_pitch_pos < geo->abs_pitch_pos - down_range){
        geo->target_pitch_pos = geo->abs_pitch_pos - down_range;
    }else if(geo->target_pitch_pos > geo->abs_pitch_pos + up_range){
        geo->target_pitch_pos = geo->abs_pitch_pos + up_range;
    }

    float min_pitch_vel = linear_map(geo->mtr_pitch_pos,
        PITCH_MTR_MINIMUM, PITCH_MTR_MINIMUM + 0.1f, 0.05f, -9.0f);

    float max_pitch_vel = linear_map(geo->mtr_pitch_pos,
        PITCH_MTR_MAXIMUM - 0.1f, PITCH_MTR_MAXIMUM, 9.0f, -0.05f);

    // For safety. Gimbal will not suddently move after reconnected to base.
    if(!BTB_ONLINE || !control_online()){
        geo->target_yaw_pos = geo->abs_yaw_pos;
        geo->target_pitch_pos = geo->abs_pitch_pos;
    }

    float yaw_pos_err =geo->target_yaw_pos - geo->abs_yaw_pos;
    // if(!gimbal_controlled_by_vision && gimbal_ctrl.swap_head_tail){
    //     yaw_pos_err += PI;
    // }

    geo->target_yaw_vel = pid_cycle(&g_yaw_pos_pid, wrap_to_pi(yaw_pos_err), CTRL_DELTA_T);
    geo->target_yaw_vel = limit_val(geo->target_yaw_vel + yaw_vel_feedforward, 15.0f);

    geo->target_pitch_vel = pid_cycle(&g_pitch_pos_pid, wrap_to_pi(geo->target_pitch_pos - geo->abs_pitch_pos), CTRL_DELTA_T);
    geo->target_pitch_vel += pitch_vel_feedforward;
    
    geo->target_pitch_vel = fmaxf(min_pitch_vel, geo->target_pitch_vel);
    geo->target_pitch_vel = fminf(max_pitch_vel, geo->target_pitch_vel);

    // For safety. If vision send weird positions, stop rotating after switch to regular mode
    if(gimbal_controlled_by_vision){ geo->target_yaw_pos = geo->abs_yaw_pos; } 
    
    // geo->target_yaw_vel = unit_step_generator(input_yaw_vel, 0.2f)*2.0f;

    geo->T_yaw = pid_cycle(&g_yaw_vel_pid, geo->target_yaw_vel - geo->abs_yaw_vel, CTRL_DELTA_T);
    
    // geo->T_yaw = 0.0f;
    // Gimbal inertia characterizaion
    // if(input_yaw_vel > 1.0f){
    //     geo->T_yaw = cosf(HAL_GetTick()*1e-3f*3.0f*PI)*2.0f;
    // }else{
    //     geo->T_yaw = 0.0f;
    // }
    const float GIMBAL_YAW_INERTIA = 0.065f; // kg*m^2
    geo->T_yaw += yaw_acc_feedforward * (GIMBAL_YAW_INERTIA * 1.0f);

    if(BTB_ONLINE && control_online()){
        if(gimbal_ctrl.flywheel_enabled){
            geo->target_flywheel_rpm = 6000.0f;        
        }else{
            geo->target_flywheel_rpm = 1500.0f;
        }
    }else{
        geo->target_flywheel_rpm = 60.0f;
    }

    // geo->target_flywheel_rpm = 60.0f;
    
    geo->T_pitch = pid_cycle(&g_pitch_vel_pid, geo->target_pitch_vel - geo->abs_pitch_vel, CTRL_DELTA_T);

    const float pitch_G_polyfit[3] = {-0.358f, -0.690f, 0.543f};
    float gravity_feedforward = 
        (geo->abs_pitch_pos * geo->abs_pitch_pos)*pitch_G_polyfit[0] +
        geo->abs_pitch_pos*pitch_G_polyfit[1] + 
        pitch_G_polyfit[2];

    geo->T_pitch += limit_val(gravity_feedforward, 1.5f);
    
    float Tfly_1 = pid_cycle(&flywheel_1_pid, geo->target_flywheel_rpm - fmotor.flywheel_1.speed, CTRL_DELTA_T);
    float Tfly_2 = pid_cycle(&flywheel_2_pid, -geo->target_flywheel_rpm - fmotor.flywheel_2.speed, CTRL_DELTA_T);
    
    uint8_t tx_buffer[8];
    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID5_8, set_current_M3508(
        tx_buffer,
        Tfly_1*(1/M3508_TORQUE_CONSTANT),
        Tfly_2*(1/M3508_TORQUE_CONSTANT),
        0.0f,
        0.0f
    ), 8);

    if(BTB_ONLINE){
        g2b_A.gimbal_request_T_yaw = (int16_t)limit_val((1e3f * geo->T_yaw), 32700.0f);
        fdcanx_send_data(&hfdcan1, G2B_MSG_A_ID, (uint8_t *)&g2b_A, 8);
    }

    // geo->T_pitch = 0.5f;
    fdcanx_send_data(&hfdcan3, PITCH_CTRLID, set_torque_DM4310(motors.pitch.tranmitbuf, -geo->T_pitch), 8);

    vofa.val[0]=fmotor.flywheel_1.speed;
    vofa.val[1]=-fmotor.flywheel_2.speed;

    // vofa.val[1]=vision_FromRos.packet.yaw;
    // vofa.val[2]=vision_online();
    vofa.val[2]=geo->abs_pitch_pos;
    vofa.val[3]=geo->mtr_pitch_pos;
    vofa.val[4]=geo->abs_pitch_vel;
    
    vofa.val[5]=geo->target_pitch_vel;
    vofa.val[6]=geo->target_pitch_pos;
    // vofa.val[6]=predict_distence;

    vofa.val[7]=geo->abs_yaw_vel;
    vofa.val[8]=geo->abs_yaw_pos;
    vofa.val[9]=geo->target_yaw_pos;
    // vofa.val[9]=geo->abs_pitch_pos - predict_pitch;
}


void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x205:
        parse_feedback_M3508(msg, &motors.flywheel_1);
        break;
    case 0x206:
        parse_feedback_M3508(msg, &motors.flywheel_2);
        break;
    case PITCH_FEEDBACKID: 
        parse_feedback_DM4310(msg, &motors.pitch, PITCH_CTRLID);
        break;
    case B2G_MSG_B_ID:
        memcpy(&b2g_B, msg, 8);
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

