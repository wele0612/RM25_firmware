#ifndef __MOTORS_H
#define __MOTORS_H

#include<stdint.h>

/* =============== M3508/M2006 ================ */
#define M3508_CTRLID_ID1_4     0x200
#define M3508_CTRLID_ID5_8     0x1FF

#define M3508_TORQUE_CONSTANT 0.3f // Nm/A

#define M3508_GEAR_RATIO	(187.0f/3591.0f)
#define M2006_GEAR_RATIO	(1.0f/36.0f)
// M2006 also use this driver.

uint8_t *set_torque_M3508(uint8_t *buf, \
	float m1_torque, float m2_torque, float m3_torque, float m4_torque);

typedef struct report_M3508_t
{
	float speed;
	float current;
	float angle;
	float tempreture;

	uint32_t last_ontime;
}report_M3508_t;

// code for M2006 is identical to M3508
typedef struct report_M3508_t report_M2006_t; 

void parse_feedback_M3508(uint8_t *msg, report_M3508_t *rpt);

/* =============== DM motors =============== */
// ISSUE on ALL Damiao motors: The default position ranges from -12.5 ~ 12.5,
// SET this value to multiples of PI on new motors!
// typedef struct report_DM4310_t{
// 	float speed;
// 	float position;
// 	float torque_actual;

// 	uint8_t recvbuf[8];
// 	uint8_t tranmitbuf[8];
// }report_DM4310_t;

typedef struct report_DM_Joint_t{
	float speed;
	float position;
	float torque_actual;

	uint8_t recvbuf[8];
	uint8_t tranmitbuf[8];
}report_DM_Joint_t;

typedef struct report_DM_Joint_t report_DM4310_t;
typedef struct report_DM_Joint_t report_DM8009P_t;

uint8_t *enable_DM_Joint(uint8_t *buf);
uint8_t *disable_DM_Joint(uint8_t *buf);

uint8_t *set_torque_DM4310(uint8_t *buf, float torque);
uint8_t *set_torque_DM8009P(uint8_t *buf, float torque);

void parse_feedback_DM4310(uint8_t *msg, report_DM4310_t *rpt, uint8_t head_id);
void parse_feedback_DM8009P(uint8_t *msg, report_DM8009P_t *rpt, uint8_t head_id);

/* =============== M0603A =============== */
#define M0603A_CTRLID_SETMODE		0xA0
#define M0603A_CTRLID_TURN			0x64
#define M0603A_CTRLID_TURN_FEEDBACK	0x65
#define M0603A_CTRLID_OBTAIN_VAL	0x74
#define M0603A_CTRLID_OBTAIN_RET	0x75

#define M0603A_MODE_OPENLOOP		0x00
#define M0603A_MODE_CURRENT		0x01
#define M0603A_MODE_SPEED		0x02
#define M0603A_MODE_ENABLE		0x08
#define M0603A_MODE_DISABLE		0x09
#define M0603A_TURNBACK_150		0x0A

#define M0603A_MSG_LENGTH 		(10U)

uint8_t *set_mode_M0603A(uint8_t *buf, uint8_t id, uint8_t mode);

uint8_t* set_current_M0603A(uint8_t* buf, uint8_t id, float current);

uint8_t* set_torque_M0603A(uint8_t* buf, uint8_t id, float torque);

uint8_t* obtain_position_M0603A(uint8_t* buf, uint8_t id);

typedef struct report_M0603A_t{
	float speed; // rad/s
	float current;
	float tempreture; // degrees C
	uint8_t err_code;

	int32_t total_turns;
	float position; // rad

	uint8_t recv_ptr;
	uint8_t recvbuf[M0603A_MSG_LENGTH*2];
}report_M0603A_t;

void parse_feedback_M0603A(uint8_t *msg, report_M0603A_t *rpt);

/* =============== P1010B =============== */
#define P1010B_GEAR_RATIO 10

#define P1010B_TORQUE_CONSTANT 2.1f // N/m

#define P1010B_CTRLID_SETSTATE  0x38
#define P1010B_CTRLID_SETPARAM  0x36
#define P1010B_CTRLID_ID1_4     0x32
#define P1010B_CTRLID_ID5_8     0x33

#define P1010B_RPTID_MOTOR_1    0x51
#define P1010B_RPTID_MOTOR_2    0x52
#define P1010B_RPTID_MOTOR_3    0x53
#define P1010B_RPTID_MOTOR_4    0x54

#define P1010B_ERRORMASK_UVP			(1U)
#define P1010B_ERRORMASK_OVP			(1U << 1)
#define P1010B_ERRORMASK_BUS_TIMEOUT	(1U << 2)
#define P1010B_ERRORMASK_PWR_OCP		(1U << 3)
#define P1010B_ERRORMASK_ADC_OFFSET		(1U << 4)
#define P1010B_ERRORMASK_OVERLOAD		(1U << 5)

uint8_t* enable_P1010B(uint8_t *buf);

uint8_t* disable_P1010B(uint8_t *buf);

#define P1010B_PARAMID_ERROR_ELIMINATE 11
uint8_t* error_eliminate_P1010B(uint8_t *buf, uint8_t id, uint16_t errormask);

#define P1010B_PARAMID_HEARTBEAT_ENABLE 22
uint8_t* enable_heartbeat_P1010B(uint8_t *buf, uint8_t id, uint8_t enabled);

#define P1010B_PARAMID_HEARTBEAT_TIMEOUT 47
uint8_t* set_heartbeat_timeout_P1010B(uint8_t *buf, uint8_t id, int16_t time_ms);

uint8_t* set_torque_P1010B(uint8_t *buf, \
    float m1_torque, float m2_torque, float m3_torque, float m4_torque);

typedef struct report_P1010B_t
{
	float speed;
	float current;
	float angle;
	float voltage;

	uint32_t last_ontime;
}report_P1010B_t;

void parse_feedback_P1010B(uint8_t *msg, report_P1010B_t *rpt);

/* ============= M1505B-111 ============= */
#define M1505B_CTRLID_SETMODE		0x105
#define M1505B_CTRLID_SETFEEDBACK	0x106
#define M1505B_CTRLID_ID1_4     	0x32
#define M1505B_CTRLID_ID5_8     	0x33

#define M1505B_RPTID_MOTOR_1    0x97
#define M1505B_RPTID_MOTOR_2    0x98
#define M1505B_RPTID_MOTOR_3    0x99
#define M1505B_RPTID_MOTOR_4    0x9A
#define M1505B_RPTID_MOTOR_5    0x9B
#define M1505B_RPTID_MOTOR_6    0x9C
#define M1505B_RPTID_MOTOR_7    0x9D
#define M1505B_RPTID_MOTOR_8    0x9E

#define M1505B_TORQUE_CONSTANT 0.8f // N/m

uint8_t* set_torque_M1505B(uint8_t *buf, \
    float m1_torque, float m2_torque, float m3_torque, float m4_torque);

uint8_t* set_feedback_mode_M1505B(uint8_t* buf, uint8_t is_active, uint8_t period_ms);

typedef struct report_M1505B_t
{
	float speed; // rad/s
	float current;
	float angle; // degrees

	uint32_t last_ontime;
}report_M1505B_t;

void parse_feedback_M1505B(uint8_t *msg, report_M1505B_t *rpt);

#endif