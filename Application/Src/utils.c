#include <utils.h>

float pid_cycle(PID_t *sys, float err, const float delta_t){
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

float wrap_to_pi(float rad) {
    while (rad >  3.14159265f) rad -= 6.28318530718f;
    while (rad < -3.14159265f) rad += 6.28318530718f;
    return rad;
}

float wrap_to_2pi(float rad) {
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

