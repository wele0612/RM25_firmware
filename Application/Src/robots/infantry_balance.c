#include<config.h>

#ifdef CONFIG_ROBOT_INFANTRY_BALANCE

#include<application.h>
#include<utils.h>
#include<global_variables.h>

#include<h7can.h>

#ifdef CONFIG_PLATFORM_BASE

static void enable_all_Damiao(){
    HAL_Delay(2);
    fdcanx_send_data(&hfdcan3, JOINT_LF_CTRLID, enable_DM_Joint(motors.joint_LF.tranmitbuf), 8);
    HAL_Delay(2);
    fdcanx_send_data(&hfdcan3, JOINT_RF_CTRLID, enable_DM_Joint(motors.joint_RF.tranmitbuf), 8);
    HAL_Delay(2);
    fdcanx_send_data(&hfdcan3, JOINT_LB_CTRLID, enable_DM_Joint(motors.joint_LB.tranmitbuf), 8);
    HAL_Delay(2);
    fdcanx_send_data(&hfdcan3, JOINT_RB_CTRLID, enable_DM_Joint(motors.joint_RB.tranmitbuf), 8);
    HAL_Delay(2);
    fdcanx_send_data(&hfdcan3, JOINT_YAW_CTRLID, enable_DM_Joint(motors.joint_LF.tranmitbuf), 8);
}

static const float L1 = 250.0e-3f;
static const float L2 = 94.5e-3f + 115.5e-3f;
static const float d1 = 94.5e-3f;
static const float d2 = 112.5e-3f;
static const float d3 = 112.5e-3f;
static const float d4 = 94.5e-3f;

static inline void forward_kinetics_jacobian(
                            float theta1, float theta2, 
                            float * L, 
                            float * theta, 
                            float * J){

    float alpha = (theta1 + theta2) * 0.5f;
    float sin_a = sinf(alpha);
    float cos_a = cosf(alpha);
    
    float a = L2;
    float b = L1;
    
    float a_sin_a = a * sin_a;
    float disc_sq = b * b - a_sin_a * a_sin_a;
    
    if (disc_sq < 0.0f) {
        disc_sq = 0.0f;
    }
    float sqrt_disc = sqrtf(disc_sq);
    
    *L = a * cos_a + sqrt_disc;
    *theta = wrap_to_pi((theta1 - theta2) * 0.5f);
    
    float dL_dalpha;
    if (sqrt_disc > 1e-6f) {
        dL_dalpha = -a * sin_a - (a * a * sin_a * cos_a) / sqrt_disc;
    } else {
        dL_dalpha = -a * sin_a;
    }
    
    J[0] = 0.5f * dL_dalpha;  // dL/dTheta1
    J[1] = 0.5f;              // dTheta/dTheta1
    J[2] = 0.5f * dL_dalpha;  // dL/dTheta2  
    J[3] = -0.5f;             // dTheta/dTheta2
}

PID_t body_roll_pid={
    .P=0.2f,
    .D=0.0f,
    .I=0.7f,
    .integral_max=0.1f
};

PID_t wheel_l_vel_calib_pit={
    .P=0.001f,
    .I=0.02f,
    .D=0.0f,
    .integral_max=10.0f
};

PID_t wheel_r_vel_calib_pit={
    .P=0.001f,
    .I=0.02f,
    .D=0.0f,
    .integral_max=10.0f
};

// Legal leg length: 140mm to 330mm
PID_t leftleg_length_pid={ // left side leg length
    .P=1200.0f,
    .I=0.0f,
    .D=30.0f,
    .integral_max=10.0f
};

PID_t leftleg_dtheta_pid={ // left side leg angular velocity
    .P=12.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=1.0f
};

PID_t rightleg_length_pid={ // right side leg length
    .P=1200.0f,
    .I=0.0f,
    .D=30.0f,
    .integral_max=10.0f
};

PID_t rightleg_dtheta_pid={ // right side leg angular velocity
    .P=12.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=1.0f
};

enum {
    WBR_STANDBY=0, // All joints disabled. Default after power-up
    WBR_CONST_VEL, // Constant leg rotation velocity
    WBR_LQR_PREP,  // Set leg, get ready to stand up
    WBR_LQR,       // Normal LQR driving mode
    WBR_FLOAT      // Floating mode, leg not touching ground
}wbr_state;

void role_controller_init(){
    enable_all_Damiao();

    robot_geo.s_max = 0.5f;
    robot_geo.target_L_length = 0.21f;
    robot_geo.target_R_length = 0.21f;
}

void role_controller_step(const float CTRL_DELTA_T){
    uint8_t wheel_tx_buf[8];
    robot_ctrl_t *geo = &robot_geo;
    robot_motors_t fmotor;

    __disable_irq(); // Important Note: create a snapshot of all motor states.
    fmotor = motors; 
    __enable_irq();

    if(dr16.s1 == DR16_SWITCH_UP){
        wbr_state=WBR_CONST_VEL;
    }else if(dr16.s1 == DR16_SWITCH_MID){
        wbr_state=WBR_LQR_PREP;
    }else if(dr16.s1 == DR16_SWITCH_DOWN){
        wbr_state=WBR_LQR;
    }
    // wbr_state = WBR_STANDBY;

    // geo->target_R_length = 0.2f;

    // const float target_wheel_vel = 3000.0f*dr16.channel[1];
    // float wheel_l_T = pid_cycle(&wheel_l_vel_calib_pit, target_wheel_vel - fmotor.wheel_L.speed, CTRL_DELTA_T);
    // float wheel_r_T = pid_cycle(&wheel_r_vel_calib_pit, target_wheel_vel - fmotor.wheel_R.speed, CTRL_DELTA_T); 

    // static float J_l[4], J_r[4];
    const float left_th1 = fmotor.joint_LF.position + (PI/2); 
    const float left_th2 = -fmotor.joint_LB.position + (PI/2);
    const float right_th1 = -fmotor.joint_RF.position + (PI/2); 
    const float right_th2 = fmotor.joint_RB.position + (PI/2);

    const float left_dth1 = -fmotor.joint_LF.speed;
    const float left_dth2 = fmotor.joint_LB.speed;
    const float right_dth1 = fmotor.joint_RF.speed;
    const float right_dth2 = -fmotor.joint_RB.speed;

    float yaw_diff = imu_data.yaw - geo->yaw_m1;
    geo->yaw_m1 = imu_data.yaw;
    // Wheel distance: 430mm (?)
    float wheel_dphi = (fmotor.wheel_R.speed + fmotor.wheel_L.speed)
        *(RPMtoRADS*0.12f*0.5f*M3508_CUSTOM_GB_RATIO*0.5f/0.215f);
    if(fabsf(wheel_dphi) >= 0.005f || fabs(yaw_diff)>= 0.00002f){
        geo->yaw_f = wrap_to_pi(geo->yaw_f + yaw_diff);
    }    

    // Wheel size: 120mm
    float ds = (fmotor.wheel_R.speed - fmotor.wheel_L.speed)*(RPMtoRADS*0.12f*0.5f*M3508_CUSTOM_GB_RATIO*0.5f);
    const float ds_alpha = 0.2f;
    if(fabsf(geo->target_ds) < 0.1f){
        geo->s = limit_val(geo->s + CTRL_DELTA_T*(ds-geo->target_ds), geo->s_max);
    }

    geo->ds = ds*ds_alpha + geo->ds*(1.0f - ds_alpha);
    
    geo->phi = imu_data.yaw;
    geo->dphi = -imu_data.gyro[2]*DEGtoRAD;

    if(HAL_GetTick()%2==0){
        forward_kinetics_jacobian(left_th1, left_th2, &geo->L_l, &geo->th_ll_nb, geo->J_l);
        geo->dL_l = geo->J_l[0]*left_dth1 + geo->J_l[2]*left_dth2;
        geo->dth_ll_nb = -(geo->J_l[1]*left_dth1 + geo->J_l[3]*left_dth2);
    }else{
        forward_kinetics_jacobian(right_th1, right_th2, &geo->L_r, &geo->th_lr_nb, geo->J_r);
        geo->dL_r = geo->J_r[0]*right_dth1 + geo->J_r[2]*right_dth2;
        geo->dth_lr_nb = -(geo->J_r[1]*right_dth1 + geo->J_r[3]*right_dth2);
    }

    // float thb_diff = (-imu_data.pitch - geo->th_b)*(1/CTRL_DELTA_T);
    geo->th_b = -imu_data.pitch;
    geo->dth_b = imu_data.gyro[1]*DEGtoRAD;

    geo->th_ll = wrap_to_pi(geo->th_ll_nb +  geo->th_b);
    geo->dth_ll = geo->dth_ll_nb + geo->dth_b;
    geo->th_lr = wrap_to_pi(geo->th_lr_nb +  geo->th_b);
    geo->dth_lr = geo->dth_lr_nb + geo->dth_b;

    geo->b_height = (geo->L_l + geo->L_r)*0.5f;
    geo->roll = wrap_to_pi(-imu_data.roll+PI);

    geo->F_wheel_support = (fmotor.joint_LF.torque_actual - fmotor.joint_LB.torque_actual) / (2.0f * geo->J_l[0]);

    // ================== Update target value =======================

    if(wbr_state == WBR_CONST_VEL){
        geo->target_L_length = 0.22f + 0.08f*dr16.channel[3];
        geo->target_L_leg_omega = 1.5f * dr16.channel[2];
        geo->target_R_length = 0.22f + 0.08f*dr16.channel[1];
        geo->target_R_leg_omega = 1.5f * dr16.channel[0];

    }else if(wbr_state == WBR_LQR_PREP){
        geo->target_L_length = 0.19f;
        geo->target_L_leg_omega = limit_val(3.0f*(0.2f - geo->th_ll_nb), 1.6f);
        geo->target_R_length = 0.19f;
        geo->target_R_leg_omega = limit_val(3.0f*(0.2f-geo->th_lr_nb), 1.6f);

        geo->target_phi = geo->phi;

    }else if(wbr_state == WBR_LQR){
        geo->target_b_height = 0.22f;

        geo->target_th_ll = 3.5f*DEGtoRAD;
        geo->target_th_lr = 3.5f*DEGtoRAD;

        geo->target_ds = dr16.channel[1]*1.1f;
        geo->target_dphi = -dr16.channel[0]*5.0f;
        geo->target_phi += geo->target_dphi*CTRL_DELTA_T;

        geo->target_roll = dr16.channel[2]*0.15f;
    }

    // ==================== Update Controllers =====================

    if(wbr_state == WBR_CONST_VEL || wbr_state == WBR_LQR_PREP){
        if(HAL_GetTick()%2==0){ // Left side
            geo->Fnl = pid_cycle(&leftleg_length_pid, geo->target_L_length - geo->L_l, CTRL_DELTA_T*2);
            geo->Tbll = pid_cycle(&leftleg_dtheta_pid, wrap_to_pi(geo->target_L_leg_omega - geo->dth_ll_nb), CTRL_DELTA_T*2);
        }else{ // Right side
            geo->Fnr = pid_cycle(&rightleg_length_pid, geo->target_R_length - geo->L_r, CTRL_DELTA_T*2);
            geo->Tblr = pid_cycle(&rightleg_dtheta_pid, wrap_to_pi(geo->target_R_leg_omega - geo->dth_lr_nb), CTRL_DELTA_T*2);
        }
        geo->Twl = pid_cycle(&wheel_l_vel_calib_pit, - fmotor.wheel_L.speed, CTRL_DELTA_T);
        geo->Twr = pid_cycle(&wheel_r_vel_calib_pit, - fmotor.wheel_R.speed, CTRL_DELTA_T);
    }else if(wbr_state == WBR_LQR){
        const float K_mat[4][10] = {
            {-0.43076f, -2.88243f, -0.71386f, -1.60790f, -7.73232f, -0.84936f, -6.17080f, -0.69328f, -4.91836f, -1.10595f},
            {-0.43076f, -2.88243f, 0.71386f, 1.60790f, -6.17080f, -0.69328f, -7.73232f, -0.84936f, -4.91836f, -1.10595f},
            {1.48885f, 9.83748f, -1.53494f, -3.51626f, 23.74244f, 2.60984f, 0.47002f, 1.10241f, -35.78619f, -4.07231f},
            {1.48885f, 9.83748f, 1.53494f, 3.51626f, 0.47002f, 1.10241f, 23.74244f, 2.60984f, -35.78619f, -4.07231f}
        };

        geo->lqr_err[0]= 0.0f - geo->s;
        geo->lqr_err[1]= geo->target_ds - geo->ds;
        geo->lqr_err[2]= wrap_to_pi(geo->target_phi - geo->phi);
        geo->lqr_err[3]= geo->target_dphi - geo->dphi;
        geo->lqr_err[4]= wrap_to_pi(geo->target_th_ll - geo->th_ll);
        geo->lqr_err[5]= 0.0f - geo->dth_ll;
        geo->lqr_err[6]= wrap_to_pi(geo->target_th_lr - geo->th_lr);
        geo->lqr_err[7]= 0.0f - geo->dth_lr;
        geo->lqr_err[8]= 0.0f - geo->th_b;
        geo->lqr_err[9]= 0.0f - geo->dth_b;

        // Direction: Tbl causes thb to decrease
        // (pushes leg to the back)

        // Direction: Twl causes thl to decrease
        // Twl cause ds(body velocity) to increase

        // [Twll Twlr Tbll Tblr]
        geo->Twl = -0.0f; // CAUTION: Twl here is OPPOSITE DIRECTION!!!!!!
        geo->Twr = 0.0f;
        geo->Tbll = 0.0f;
        geo->Tblr = 0.0f;
        for(int i=0;i<10;i++){
            geo->Twl -= K_mat[0][i]*geo->lqr_err[i];
            geo->Twr += K_mat[1][i]*geo->lqr_err[i];
            geo->Tbll += K_mat[2][i]*geo->lqr_err[i];
            geo->Tblr += K_mat[3][i]*geo->lqr_err[i];
        }

        float L_diff = pid_cycle(&body_roll_pid, geo->target_roll - geo->roll, CTRL_DELTA_T);
        L_diff = limit_val(L_diff, 0.05f);
        geo->target_L_length = geo->target_b_height + L_diff;
        geo->target_R_length = geo->target_b_height - L_diff;
        
        if(HAL_GetTick()%2==0){ // Left side
            geo->Fnl = pid_cycle(&leftleg_length_pid, geo->target_L_length - geo->L_l, CTRL_DELTA_T*2);
            geo->Fnl -= 50.0f; // There's a spring on left side.
            geo->Fnl += 50.0f;
        }else{ // Right side
            geo->Fnr = pid_cycle(&rightleg_length_pid, geo->target_R_length - geo->L_r, CTRL_DELTA_T*2);
            geo->Fnr += 50.0f;
        }

    }else if(wbr_state == WBR_STANDBY){
        if(HAL_GetTick()%2==0){ // Left side
            geo->Fnl = 0.0f;
            geo->Tbll = 0.0f;
        }else{ // Right side
            geo->Fnr = 0.0f;
            geo->Tblr = 0.0f;
        }
        geo->Twl = 0.0f;
        geo->Twr = 0.0f;
    }
    // geo->Twl += friction_compensation(fmotor.wheel_L.speed, 0.05f, (1/100.0f));
    // geo->Twr += friction_compensation(fmotor.wheel_R.speed, 0.15f, (1/100.0f));

    // ==================== Send to fmotor =====================

    if(HAL_GetTick()%2==0){
        geo->T_LF = geo->J_l[0]*geo->Fnl + geo->J_l[1]*geo->Tbll;
        geo->T_LB = geo->J_l[2]*geo->Fnl + geo->J_l[3]*geo->Tbll;
        fdcanx_send_data(&hfdcan3, JOINT_LF_CTRLID, set_torque_DM8009P(fmotor.joint_LF.tranmitbuf, geo->T_LF), 8);
        fdcanx_send_data(&hfdcan3, JOINT_LB_CTRLID, set_torque_DM8009P(fmotor.joint_LB.tranmitbuf, -geo->T_LB), 8);
    }else{
        geo->T_RF = -geo->J_r[0]*geo->Fnr - geo->J_r[1]*geo->Tblr;
        geo->T_RB = -geo->J_r[2]*geo->Fnr - geo->J_r[3]*geo->Tblr;
        fdcanx_send_data(&hfdcan3, JOINT_RF_CTRLID, set_torque_DM8009P(fmotor.joint_RF.tranmitbuf, geo->T_RF), 8);
        fdcanx_send_data(&hfdcan3, JOINT_RB_CTRLID, set_torque_DM8009P(fmotor.joint_RB.tranmitbuf, -geo->T_RB), 8);
    }

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, \
        set_current_M3508(wheel_tx_buf, 
            geo->Twl*(1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB), 
            geo->Twr*(1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
            0.0f, 
            0.0f),
        8);

    vofa.val[0]=geo->s;
    vofa.val[1]=geo->ds;

    vofa.val[2]=geo->phi;
    vofa.val[3]=geo->dphi;

    vofa.val[4]=geo->th_ll;
    vofa.val[5]=geo->dth_ll;

    vofa.val[6]=fmotor.joint_LF.torque_actual;
    vofa.val[7]=fmotor.joint_LB.torque_actual;

    vofa.val[8]=geo->roll;
    vofa.val[9]=geo->b_height;

    // vofa.val[8]=geo->th_b;
    // vofa.val[9]=geo->dth_b;

    // if(CTRL_DELTA_T != 0.001f){
    //     vofa.val[9]+=0.001f;
    // }

    // vofa.val[0]=geo->L_l;
    // vofa.val[1]=geo->Fnl;

    // vofa.val[2]=geo->L_r;
    // vofa.val[3]=geo->Fnr;

    // vofa.val[4]=geo->T_LF;
    
}

void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x201:
        parse_feedback_M3508(msg, &motors.wheel_L);
        break;
    case 0x202:
        parse_feedback_M3508(msg, &motors.wheel_R);
        break;
    case JOINT_LF_FEEDBACKID:
        parse_feedback_DM8009P(msg, &motors.joint_LF, JOINT_LF_CTRLID);
        break;
    case JOINT_RF_FEEDBACKID:
        parse_feedback_DM8009P(msg, &motors.joint_RF, JOINT_RF_CTRLID);
        break;
    case JOINT_LB_FEEDBACKID:
        parse_feedback_DM8009P(msg, &motors.joint_LB, JOINT_LB_CTRLID);
        break;
    case JOINT_RB_FEEDBACKID:
        parse_feedback_DM8009P(msg, &motors.joint_RB, JOINT_RB_CTRLID);
        break;

    default:
        break;
    }

    return;
}



#else


// 0x0F is CAN(Slave) and 0x00 is Master
void role_controller_init(){
}

void role_controller_step(const float CTRL_DELTA_T){

    vofa.val[0]=motors.pitch.position;
    vofa.val[1]=motors.pitch.speed;
    vofa.val[2]=motors.pitch.torque_actual;
    vofa.val[3]=target_pitch_omega;
    vofa.val[4]=pitch_omega_error;
    vofa.val[5]=pitch_torque;
}

void robot_CAN_msgcallback(int ID, uint8_t *msg){
    switch (ID){
    case 0x0:

    default:
        break;
    }
    
    return;
}

#endif

#endif
