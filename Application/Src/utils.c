#include <utils.h>

inline float pid_cycle(PID_t *sys, float err, const float delta_t){
    sys->integral += err*delta_t;
    if(sys->integral > sys->integral_max){
        sys->integral = sys->integral_max;
    }else if(sys->integral < -sys->integral_max){
        sys->integral = -sys->integral_max;
    }
    
    float derivative = (err - sys->err_m1)/delta_t;
    float output = sys->P * err + sys->I * sys->integral + sys->D * derivative;

    sys->err_m1 = err;
    return output;
}

inline float wrap_to_pi(float rad) {
    while (rad >  3.14159265f) rad -= 6.28318530718f;
    while (rad < -3.14159265f) rad += 6.28318530718f;
    return rad;
}

inline float wrap_to_2pi(float rad) {
    const float two_pi = 6.28318530718f;
    while (rad >= two_pi) rad -= two_pi;
    while (rad < 0.0f) rad += two_pi;
    return rad;
}

inline float limit_val(float val, const float limit){
    if( val > limit){
        return limit;
    } else if (val < -limit){
        return -limit;
    }
    return val;
}

inline float filter_iir_eval(iir_filter_t *iir, const float x, const int order, const float a[4], const float b[4]){
    for (int i = order; i > 0; i--) {
        iir->x[i] = iir->x[i-1];
        iir->y[i] = iir->y[i-1];
    }
    iir->x[0] = x;

    float y = 0.0f;
    // feedforward
    for (int i = 0; i <= order; i++) {
        y += b[i] * iir->x[i];
    }
    // feedback (skip a[0])
    for (int i = 1; i <= order; i++) {
        y -= a[i] * iir->y[i];
    }

    // a[0] != 1，normalize
    if (a[0] != 1.0f) {
        y /= a[0];
    }

    iir->y[0] = y;
    return y;
}
