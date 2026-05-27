#ifndef __CHASIS_POWER_H
#define __CHASIS_POWER_H

#include<stdint.h>

#endif

float m3508_estimate_power(float current, float omega);

float m3508_quadwheel_get_scaling(float currents[], const float omegas[], const float target_power);
