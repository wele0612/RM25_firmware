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

// Filters
// IIR filter, up to 3-order
typedef struct iir_filter_t{
    float x[4]; //x[t] ~ x[t-3]
    float y[4]; //y[t] ~ y[t-3]
}iir_filter_t;

float filter_iir_eval(iir_filter_t* iir, const float x, const int order, const float a[4], const float b[4]);


#endif
