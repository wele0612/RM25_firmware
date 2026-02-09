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

static inline void forward_kinetics(float theta1, float theta2, float *L, float *theta){
    float theta12 = theta1 + theta2;
    float cos_theta12 = cosf(theta12);
    float d5_sq = d1 * d1 + d4 * d4 - 2.0f * d1 * d4 * cos_theta12;
    float d5 = sqrtf(d5_sq);
    
    float A = (d4 * d4 + d5 * d5 - d1 * d1) / (2.0f * d4 * d5);
    float B = (d3 * d3 + d5 * d5 - d2 * d2) / (2.0f * d3 * d5);
    
    float theta3_p1 = acosf(A);
    float theta3_p2 = acosf(B);
    float theta3 = theta3_p1 + theta3_p2;
    
    // calculate leg length (L)
    float cos_theta3 = cosf(theta3);
    float L_sq = L1 * L1 + L2 * L2 - 2.0f * L1 * L2 * cos_theta3;
    float L_val = sqrtf(L_sq);
    
    // calculate leg angle (Theta)
    float sin_theta3 = sinf(theta3);
    float theta4 = asinf((L1 / L_val) * sin_theta3);
    
    *L = L_val;
    *theta = wrap_to_pi(theta1 - theta4);
}

PID_t wheel_l_vel_calib_pit={
    .P=0.0f,
    .I=0.0f,
    .D=0.0f,
    .integral_max=0.1f
};

void role_controller_init(){
    enable_all_Damiao();
}

void role_controller_step(const float CTRL_DELTA_T){
    uint8_t wheel_tx_buf[8];

    fdcanx_send_data(&hfdcan2, M3508_CTRLID_ID1_4, \
        set_current_M3508(wheel_tx_buf, 0.0f, 0.0f, 0.0f, 0.0f), 8);

    if(HAL_GetTick()%2==0){
        fdcanx_send_data(&hfdcan3, JOINT_LF_CTRLID, set_torque_DM8009P(motors.joint_LF.tranmitbuf, 0.0f), 8);
        fdcanx_send_data(&hfdcan3, JOINT_LB_CTRLID, set_torque_DM8009P(motors.joint_LB.tranmitbuf, 0.0f), 8);
    }else{
        fdcanx_send_data(&hfdcan3, JOINT_RF_CTRLID, set_torque_DM8009P(motors.joint_RF.tranmitbuf, 0.0f), 8);
        fdcanx_send_data(&hfdcan3, JOINT_RB_CTRLID, set_torque_DM8009P(motors.joint_RB.tranmitbuf, 0.0f), 8);
    }

    const float left_th1=motors.joint_LF.position + (PI/2); 
    const float left_th2= -motors.joint_LB.position + (PI/2);
    forward_kinetics(left_th1, left_th2, &robot_geo.L_l, &robot_geo.th_ll);

    const float right_th1 = -motors.joint_RF.position + (PI/2); 
    const float right_th2 = motors.joint_RB.position + (PI/2);
    forward_kinetics(right_th1, right_th2, &robot_geo.L_r, &robot_geo.th_lr);

    
    
    vofa.val[0]=imu_data.yaw;
    vofa.val[1]=imu_data.pitch;
    vofa.val[2]=imu_data.gyro[1];
    vofa.val[3]=dr16.channel[0];

    vofa.val[4]=left_th1*RADtoDEG;
    vofa.val[5]=right_th1*RADtoDEG;
    vofa.val[6]=robot_geo.L_r;
    vofa.val[7]=robot_geo.th_lr*RADtoDEG;

    vofa.val[8]=robot_geo.L_l;
    vofa.val[9]=robot_geo.th_ll*RADtoDEG;

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
