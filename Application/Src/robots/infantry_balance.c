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
                            float * restrict L, 
                            float * restrict theta, 
                            float * restrict J){

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
    *theta = (theta1 - theta2) * 0.5f;
    
    // dL/dalpha = -a*sin(alpha) * (1 + a*cos(alpha)/sqrt_disc)
    float dL_dalpha;
    const float eps = 1e-6f;
    
    if (sqrt_disc < eps) {
        dL_dalpha = -a * sin_a;  
    } else {
        dL_dalpha = -a * sin_a * (1.0f + (a * cos_a) / sqrt_disc);
    }
    
    float half_dL = 0.5f * dL_dalpha;
    
    J[0] = half_dL;
    J[1] = 0.5f;
    J[2] = -half_dL;
    J[3] = 0.5f;

}

PID_t wheel_l_vel_calib_pit={
    .P=0.001f,
    .I=0.03f,
    .D=0.0f,
    .integral_max=10.0f
};

PID_t wheel_r_vel_calib_pit={
    .P=0.001f,
    .I=0.03f,
    .D=0.0f,
    .integral_max=10.0f
};

void role_controller_init(){
    enable_all_Damiao();
}

void role_controller_step(const float CTRL_DELTA_T){
    uint8_t wheel_tx_buf[8];
    // Legal leg length: 140mm to 330mm
    robot_ctrl_t *geo = &robot_geo;

    const float left_th1=motors.joint_LF.position + (PI/2); 
    const float left_th2= -motors.joint_LB.position + (PI/2);
    float J_l[4];
    forward_kinetics_jacobian(left_th1, left_th2, &geo->L_l, &geo->th_ll, J_l);

    const float right_th1 = -motors.joint_RF.position + (PI/2); 
    const float right_th2 = motors.joint_RB.position + (PI/2);
    float J_r[4];
    forward_kinetics_jacobian(right_th1, right_th2, &geo->L_r, &geo->th_lr, J_r);

    geo->Fnl = 20.0f;
    geo->Tbll = 2.0f;

    geo->Fnr = 20.0f;
    geo->Tblr = 2.0f;

    const float target_wheel_vel = 3000.0f*dr16.channel[1];
    // float wheel_l_T = pid_cycle(&wheel_l_vel_calib_pit, target_wheel_vel - motors.wheel_L.speed, CTRL_DELTA_T);
    // float wheel_r_T = pid_cycle(&wheel_r_vel_calib_pit, target_wheel_vel - motors.wheel_R.speed, CTRL_DELTA_T);
    // geo->Twl = friction_compensation(motors.wheel_L.speed, 0.075f, (1/200.0f));
    // geo->Twr = friction_compensation(motors.wheel_R.speed, 0.138f, (1/200.0f));

    float T_LF = J_l[0]*geo->Fnl + J_l[1]*geo->Tbll;
    float T_LB = J_l[2]*geo->Fnl + J_l[3]*geo->Tbll;

    float T_RF = - J_r[0]*geo->Fnr + J_r[1]*geo->Tblr;
    float T_RB = - J_r[2]*geo->Fnr + J_r[3]*geo->Tblr;

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, \
        set_current_M3508(wheel_tx_buf, 
            geo->Twl*(1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB), 
            geo->Twr*(1.0f/M3508_TORQUE_CONSTANT_CUSTOM_GB),
            0.0f, 
            0.0f),
        8);

    if(HAL_GetTick()%2==0){
        fdcanx_send_data(&hfdcan3, JOINT_LF_CTRLID, set_torque_DM8009P(motors.joint_LF.tranmitbuf, T_LF), 8);
        fdcanx_send_data(&hfdcan3, JOINT_LB_CTRLID, set_torque_DM8009P(motors.joint_LB.tranmitbuf, T_LB), 8);
    }else{
        fdcanx_send_data(&hfdcan3, JOINT_RF_CTRLID, set_torque_DM8009P(motors.joint_RF.tranmitbuf, T_RF), 8);
        fdcanx_send_data(&hfdcan3, JOINT_RB_CTRLID, set_torque_DM8009P(motors.joint_RB.tranmitbuf, T_RB), 8);
    }

    vofa.val[0]=imu_data.yaw;
    vofa.val[1]=imu_data.pitch;

    vofa.val[2]=J_l[0];
    vofa.val[3]=J_l[1];
    vofa.val[4]=T_RF;
    vofa.val[5]=T_RB;
    vofa.val[6]=robot_geo.L_r;
    vofa.val[7]=robot_geo.L_l;

    vofa.val[8]=T_LF;
    vofa.val[9]=T_LB;

    // vofa.val[8]=(float)dr16.s1;
    // vofa.val[9]=(float)dr16.s2;
    
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
