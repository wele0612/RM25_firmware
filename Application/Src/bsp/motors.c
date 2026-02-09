#include <motors.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#include <stddef.h>
#include <stdint.h>

#include <string.h>

// CRC 算法：CRC-8/MAXIM
// 多项式：x8 + x5 + x4 + 1
uint8_t calc_crc8(void *data, size_t size) {
    
    uint8_t *bytes = (uint8_t *)data;
    uint8_t crc = 0x00; // 初始值
    uint8_t poly = 0x31; // 多项式 x⁸ + x⁵ + x⁴ + 1

    for (size_t i = 0; i < size; i++) {
        // 对输入数据进行位反转
        uint8_t b = bytes[i];
        b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
        b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
        b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
        
        crc ^= b;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }

    crc = ((crc & 0xF0) >> 4) | ((crc & 0x0F) << 4);
    crc = ((crc & 0xCC) >> 2) | ((crc & 0x33) << 2);
    crc = ((crc & 0xAA) >> 1) | ((crc & 0x55) << 1);
    
    return crc;
}
static inline float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float dm_uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int dm_float_to_uint(float x, float x_min, float x_max, int bits){
    // Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/* =============== M3508 =============== */
uint8_t* set_torque_M3508(uint8_t *buf, \
    float m1_torque, float m2_torque, float m3_torque, float m4_torque){

    const float m3508_current_to_int = (16384.0f/20.0f);

    const float MAX_TORQUE = 19.5f * M3508_TORQUE_CONSTANT;

    m1_torque = fmaxf(-MAX_TORQUE, fminf(m1_torque, MAX_TORQUE));
    m2_torque = fmaxf(-MAX_TORQUE, fminf(m2_torque, MAX_TORQUE));
    m3_torque = fmaxf(-MAX_TORQUE, fminf(m3_torque, MAX_TORQUE));
    m4_torque = fmaxf(-MAX_TORQUE, fminf(m4_torque, MAX_TORQUE));
      
    uint16_t m1_current = (int16_t)(m1_torque/M3508_TORQUE_CONSTANT*m3508_current_to_int);
    buf[0] = m1_current >> 8;
    buf[1] = m1_current & 0x00ff;

    uint16_t m2_current = (int16_t)(m2_torque/M3508_TORQUE_CONSTANT*m3508_current_to_int);
    buf[2] = m2_current >> 8;
    buf[3] = m2_current & 0x00ff;

    uint16_t m3_current = (int16_t)(m3_torque/M3508_TORQUE_CONSTANT*m3508_current_to_int);
    buf[4] = m3_current >> 8;
    buf[5] = m3_current & 0x00ff;

    uint16_t m4_current = (int16_t)(m4_torque/M3508_TORQUE_CONSTANT*m3508_current_to_int);
    buf[6] = m4_current >> 8;
    buf[7] = m4_current & 0x00ff;

    return buf;
}

void parse_feedback_M3508(uint8_t *msg, report_M3508_t *rpt){
    int16_t angle = ((int16_t)msg[0] << 8) | msg[1];
    float anglef = (float)(angle*(360.0f/8191.0f));

    rpt->angle = anglef;

    int16_t speed = ((int16_t)msg[2] << 8) | msg[3];
    rpt->speed = (float)speed;

    int16_t current = ((int16_t)msg[4] << 8) | msg[5];
    rpt->current = (float)(current*(20.0f/16384.0f));

    int16_t tempreture = (int16_t)msg[6];
    rpt->tempreture = (float)tempreture;

}


/* =============== DaMiao series =============== */

uint8_t *enable_DM_Joint(uint8_t *buf) { 
    for(int i=0;i<7;i++){
        buf[i] = 0xff;
    }
    buf[7] = 0xfc;
    return buf; 
}

uint8_t *disable_DM_Joint(uint8_t *buf) { 
    for(int i=0;i<7;i++){
        buf[i] = 0xff;
    }
    buf[7] = 0xfd;
    return buf; 
}

static const float P_MIN = -12.56637f, P_MAX = 12.56637f;
static const float V_MIN = -45.0f, V_MAX = 45.0f;
static const float KP_MIN = 0.0f, KP_MAX = 500.0f;
static const float KD_MIN = 0.0f, KD_MAX = 5.0f;

static const float T_MAX_8009P = 54.0f;
static const float T_MAX_4310 = 18.0f;

static uint8_t *set_MIT_DM_joint(uint8_t *buf, float position,
                        float velocity, float torque, const float kp,
                        const float kd, const float T_MAX) {

    *(uint64_t*)buf = 0x0;

    position = constrain(position, P_MIN, P_MAX);
    velocity = constrain(velocity, V_MIN, V_MAX);
    torque = constrain(torque, -T_MAX, T_MAX);

    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = dm_float_to_uint(position, P_MIN, P_MAX, 16);
    vel_tmp = dm_float_to_uint(velocity, V_MIN, V_MAX, 12);
    kp_tmp = dm_float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = dm_float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = dm_float_to_uint(torque, -T_MAX, T_MAX, 12);

    buf[0] = (pos_tmp >> 8);
    buf[1] = pos_tmp;
    buf[2] = (vel_tmp >> 4);
    buf[3] = ((vel_tmp&0xF)<<4) | (kp_tmp>>8);
    buf[4] = kp_tmp;
    buf[5] = (kd_tmp >> 4);
    buf[6] = ((kd_tmp & 0xF)<<4) | (tor_tmp>>8);
    buf[7] = tor_tmp;
    
    return buf;
}


uint8_t* set_torque_DM4310(uint8_t *buf, float torque){
    return set_MIT_DM_joint(buf, 0.0f, 0.0f, torque, 0.0f, 0.0f, T_MAX_4310);
}

uint8_t* set_torque_DM8009P(uint8_t *buf, float torque){
    return set_MIT_DM_joint(buf, 0.0f, 0.0f, torque, 0.0f, 0.0f, T_MAX_8009P);
}

static void parse_feedback_DM_Joint(uint8_t *msg, report_DM_Joint_t *rpt, uint8_t head_id, const float T_MAX){
    uint16_t p_int, v_int, t_int;

    if((msg[0] & 0x0f) != head_id){// Double check to eliminate bugs.
        return;
    }

    p_int = (msg[1] << 8) | msg[2];
    v_int = (msg[3] << 4) | (msg[4] >> 4);
    t_int = ((msg[4] & 0xF) << 8) | msg[5];
    
    rpt->position = dm_uint_to_float(p_int, P_MIN, P_MAX, 16);

    rpt->speed = dm_uint_to_float(v_int, V_MIN, V_MAX, 12);

    rpt->torque_actual = dm_uint_to_float(t_int, -T_MAX, T_MAX, 12);

    return;
}

void parse_feedback_DM4310(uint8_t *msg, report_DM4310_t *rpt, uint8_t head_id){
    parse_feedback_DM_Joint(msg, (report_DM_Joint_t *)rpt, head_id, T_MAX_4310);
}

void parse_feedback_DM8009P(uint8_t *msg, report_DM8009P_t *rpt, uint8_t head_id){
    parse_feedback_DM_Joint(msg, (report_DM_Joint_t *)rpt, head_id, T_MAX_8009P);
}


/* ============= M0603A-111 ============= */
uint8_t* set_mode_M0603A(uint8_t *buf, uint8_t id, uint8_t mode){
    memset(buf, 0, 9);
    buf[0] = id;
    buf[1] = M0603A_CTRLID_SETMODE;
    buf[2] = mode;

    buf[9] = calc_crc8(buf, 9);
    return buf;
}

uint8_t* set_current_M0603A(uint8_t* buf, uint8_t id, float current){
    if(current > 3.9f) current = 3.9f;
    if(current < -3.9f) current = -3.9f;
    const int16_t current_i16 = (int16_t)(current*(32767/4.0f));
    // const int16_t current_i16 = 0x0bb8;

    memset(buf, 0, 9);
    buf[0] = id;
    buf[1] = M0603A_CTRLID_TURN;
    buf[2] = current_i16 >> 8;
    buf[3] = current_i16 & 0x00ff;

    buf[9] = calc_crc8(buf, 9);
    return buf;
}

uint8_t* set_torque_M0603A(uint8_t* buf, uint8_t id, float torque){
    float current;
    /* From experiments: 
    when I <= 0.2A, torque constant = 0.2Nm/A 
    when I > 0.2A, marginal torque constant = 0.4Nm/A */
    // if( fabs(torque) <= 0.04f ){
    //     current = torque * (1/0.2f);
    // } else if ( torque > 0.04f ){
    //     torque -= 0.04f;
    //     current = torque * (1/0.4f) + 0.2f;
    // } else {
    //     torque += 0.04f;
    //     current = torque * (1/0.4f) - 0.2f;
    // }
    //current = torque * (1/0.4f);
    const float Tconstant_1 = 0.3;/* Nm/A */
    if ( fabs(torque) <= (0.2f * Tconstant_1) ) {
        current = torque / Tconstant_1;
    } else if ( torque > (0.2f * Tconstant_1) ) {
        torque -= 0.2f * Tconstant_1;
        current = torque / 0.4f + 0.2f;
    } else {
        torque += 0.2f * Tconstant_1;
        current = torque / 0.4f - 0.2f;
    }

    return set_current_M0603A(buf, id, current);
}

uint8_t* obtain_position_M0603A(uint8_t* buf, uint8_t id){
    memset(buf, 0, 9);
    buf[0] = id;
    buf[1] = M0603A_CTRLID_OBTAIN_VAL;

    buf[9] = calc_crc8(buf, 9);
    return buf;
}

void parse_feedback_M0603A(uint8_t *msg, report_M0603A_t *rpt){
    if( calc_crc8(msg, 9) == msg[9] ){
        switch(msg[1]){
            case M0603A_CTRLID_TURN:
            case M0603A_CTRLID_OBTAIN_VAL:
                break;

            case M0603A_CTRLID_TURN_FEEDBACK:
                int16_t speed = ((int16_t)msg[2] << 8) | msg[3];
                rpt->speed = speed*(0.1f*2*3.141592653f/60.0f);

                int16_t current = ((int16_t)msg[4] << 8) | msg[5];
                rpt->current = current*(4.0f/32767);

                rpt->tempreture = (float)msg[7];
                rpt->err_code = msg[8];
                break;
            case M0603A_CTRLID_OBTAIN_RET:
                rpt->total_turns = ((int32_t)msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | msg[5];
                rpt->position = (((uint16_t)msg[6] << 8) | msg[7])*(2*3.141592653f/32767.0f);
                
                break;
            default:
                break; 
        }
    }
    return;
}

/* ============= P1010B-111 ============= */

uint8_t* enable_P1010B(uint8_t *buf){
    *(uint64_t*)buf = 0x0202020202020202;
    return buf;
}
uint8_t* disable_P1010B(uint8_t *buf){
    *(uint64_t*)buf = 0x0101010101010101;
    return buf;
}
uint8_t* error_eliminate_P1010B(uint8_t *buf, uint8_t id, uint16_t errormask){
    *(uint64_t*)buf = 0x0;
    buf[0] = id;
    buf[1] = P1010B_PARAMID_ERROR_ELIMINATE;
    buf[2] = errormask & 0x00ff;
    buf[3] = errormask >> 8;
    return buf;
}
uint8_t* enable_heartbeat_P1010B(uint8_t *buf, uint8_t id, uint8_t enabled){
    *(uint64_t*)buf = 0x0;
    buf[0] = id;
    buf[1] = P1010B_PARAMID_HEARTBEAT_ENABLE;
    buf[2] = enabled;
    return buf;
}
uint8_t* set_heartbeat_timeout_P1010B(uint8_t *buf, uint8_t id, int16_t time_ms){
    *(uint64_t*)buf = 0x0;
    buf[0] = id;
    buf[1] = P1010B_PARAMID_HEARTBEAT_TIMEOUT;
    buf[3] = time_ms >> 8;
    buf[2] = time_ms & 0x00ff;
    return buf;
}
uint8_t* set_torque_P1010B(uint8_t *buf, \
    float m1_torque, float m2_torque, float m3_torque, float m4_torque){
      
    uint16_t m1_current = (int16_t)(m1_torque/P1010B_TORQUE_CONSTANT*100);
    buf[0] = m1_current >> 8;
    buf[1] = m1_current & 0x00ff;

    uint16_t m2_current = (int16_t)(m2_torque/P1010B_TORQUE_CONSTANT*100);
    buf[2] = m2_current >> 8;
    buf[3] = m2_current & 0x00ff;

    uint16_t m3_current = (int16_t)(m3_torque/P1010B_TORQUE_CONSTANT*100);
    buf[4] = m3_current >> 8;
    buf[5] = m3_current & 0x00ff;

    uint16_t m4_current = (int16_t)(m4_torque/P1010B_TORQUE_CONSTANT*100);
    buf[6] = m4_current >> 8;
    buf[7] = m4_current & 0x00ff;

    return buf;
}

void parse_feedback_P1010B(uint8_t *msg, report_P1010B_t *rpt){
    int16_t speed = ((int16_t)msg[0] << 8) | msg[1];
    rpt->speed = (float)speed * (0.1f/(60.0f*P1010B_GEAR_RATIO));

    int16_t current = ((int16_t)msg[2] << 8) | msg[3];
    rpt->current = (float)current * 0.01f;

    int16_t angle = ((int16_t)msg[4] << 8) | msg[5];
    float anglef = (float)(angle*(360.0f/32768.0f));
    // if (anglef - rpt->angle > 3.0f){
    //     rpt->angle += 3.0f;
    // } else if (anglef - rpt->angle < -3.0f){
    //     rpt->angle -= 3.0f;
    // } else {
        rpt->angle = anglef;
    // }    

    int16_t voltage = ((int16_t)msg[6] << 8) | msg[7];
    rpt->voltage = (float)voltage * 0.1f;

}

/* ============= M1505B-111 ============= */

uint8_t* set_torque_M1505B(uint8_t *buf, \
    float m1_torque, float m2_torque, float m3_torque, float m4_torque){

    const float conversion = (32767.0f/55.0f)/M1505B_TORQUE_CONSTANT;

    uint16_t m1_current = (int16_t)(m1_torque*conversion);
    buf[0] = m1_current >> 8;
    buf[1] = m1_current & 0x00ff;

    uint16_t m2_current = (int16_t)(m2_torque*conversion);
    buf[2] = m2_current >> 8;
    buf[3] = m2_current & 0x00ff;

    uint16_t m3_current = (int16_t)(m3_torque*conversion);
    buf[4] = m3_current >> 8;
    buf[5] = m3_current & 0x00ff;

    uint16_t m4_current = (int16_t)(m4_torque*conversion);
    buf[6] = m4_current >> 8;
    buf[7] = m4_current & 0x00ff;

    return buf;
}

uint8_t* set_feedback_mode_M1505B(uint8_t* buf, uint8_t is_active, uint8_t period_ms){
    *(uint64_t*)buf = 0x0;

    uint8_t feedback = 0;
    if (is_active){
        feedback |= period_ms;
    } else {
        feedback |= 0x80;
    }
    for(int i=0;i<8;i++){
        buf[i] = feedback;
    }

    return buf;
}


void parse_feedback_M1505B(uint8_t *msg, report_M1505B_t *rpt){
    int16_t speed = ((int16_t)msg[0] << 8) | msg[1];
    rpt->speed = (float)speed * (0.1f*2.0f*3.1415926f/(60.0f));

    const float conversion = (55.0f/32767.0f);
    int16_t current = ((int16_t)msg[2] << 8) | msg[3];
    rpt->current = (float)current*conversion;

    int16_t angle = ((int16_t)msg[4] << 8) | msg[5];
    rpt->angle = (float)(angle*(360.0f/32768.0f));
}

