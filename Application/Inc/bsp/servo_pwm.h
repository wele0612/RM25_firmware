#ifndef __SERVO_PWM_H
#define __SERVO_PWM_H

#include <main.h>

#include<tim.h>

void servo_init();
void servo_SetAngle(float angle);

#endif
