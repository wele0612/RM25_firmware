#include <utils.h>

#include <math.h>

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

float friction_compensation(float omega, const float coulomb_torque, const float omega_threshold_inv){
    const float inv_threshold = omega_threshold_inv; // 1/omega
    
    // 归一化到 [-1, 1]
    float x = omega * inv_threshold;
    float abs_x = fabsf(x);
    
    if (abs_x >= 1.0f) {
        // High speed saturation
        return coulomb_torque * copysignf(1.0f, omega);
    } else {
        // 低速过渡区：3次多项式 f(x) = 1.5x - 0.5x³
        // 满足 f(0)=0, f(±1)=±1, f'(±1)=0 (C1连续)
        float x2 = x * x;
        float smooth_factor = x * (1.5f - 0.5f * x2);
        return coulomb_torque * smooth_factor;
    }
}

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

#ifndef TWO_PI_F  
#define TWO_PI_F 6.28318530717958647692f
#endif

/**
 * 将弧度限制在 (-π, π] 区间
 */
inline float wrap_to_pi(float rad) {
    if (!isfinite(rad)) {
        return 0.0f; 
    }
    
    rad = fmodf(rad + PI_F, TWO_PI_F);
    
    if (rad <= 0.0f) {
        rad += TWO_PI_F;
    }
    
    return rad - PI_F;  // 映射回 (-π, π]
}

/**
 * 将弧度限制在 [0, 2π) 区间  
 */
inline float wrap_to_2pi(float rad) {
    if (!isfinite(rad)) {
        return 0.0f;
    }
    
    rad = fmodf(rad, TWO_PI_F);
    
    if (rad < 0.0f) {
        rad += TWO_PI_F;
    }
    
    if (rad >= TWO_PI_F) {
        rad -= TWO_PI_F;
    }
    
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
