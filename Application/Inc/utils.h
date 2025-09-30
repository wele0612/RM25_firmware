#ifndef __UTILS_H
#define __UTILS_H

typedef struct PID_t{
    const float P;
    const float I;
    const float D;
    const float integral_max;
    float integral;
    float err_m1;
}PID_t;

float pid_cycle(PID_t *sys, float err, const float delta_t);

float wrap_to_pi(float rad);

float wrap_to_2pi(float rad);

float limit_val(float val, const float limit);

#endif
