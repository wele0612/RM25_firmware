#include <chasis_power.h>
#include <math.h>

// In SI unit; torque current=A, speed=rad/s (rotor)

// W = K0
//   + K1 * current
//   + K2 * speed
//   + K3 * current * speed
//   + K4 * current * current
//   + K5 * speed * speed

const float M3508_POLYMODEL[]={
    2.1599749,   // K0
    0.0015595104,   // K1
    -0.00017931848,   // K2
    0.016782476,   // K3
    // 0.13301039,   // K4
    0.11301039,   // K4
    1.1531789e-05    // K5
};

/**
 * When rotor speed (omega) is known, 
 * total power consumption and current is linked with a quadratic polynomial.
 * 
 * P = C0 + C1*current + C2*current*current
 */
inline static void m3508_obtain_current_relationship(float omega, float *coes){
    coes[0] = M3508_POLYMODEL[0] + M3508_POLYMODEL[2]*omega + M3508_POLYMODEL[5]*omega*omega;
    coes[1] = M3508_POLYMODEL[1] + M3508_POLYMODEL[3]*omega;
    coes[2] = M3508_POLYMODEL[4];
}

float m3508_estimate_power(float current, float omega){
    float coes[3];
    m3508_obtain_current_relationship(omega, coes);
    return coes[0]+coes[1]*current+coes[2]*current*current;
}

float m3508_quadwheel_get_scaling(float currents[], const float omegas[], const float target_power){
    float coes[3];
    float a = 0.0f, b = 0.0f, c = 0.0f;
    
    for (int i = 0; i < 4; i++) {
        m3508_obtain_current_relationship(omegas[i], coes);
        float I = currents[i];
        c += coes[0];
        b += coes[1] * I;
        a += coes[2] * I * I;
    }
    
    // Already lower than power limit.
    float unscaled_power = a + b + c;
    if (unscaled_power <= target_power) {
        return 1.0f;
    }
    
    // Solve quadratic equation a*lambda^2 + b*lambda + c = target_power
    if (a < 1e-6f) {
        // Degenerates to linear equation b*lambda + c = target_power
        if (b > 1e-6f || b < -1e-6f) {
            float lambda = (target_power - c) / b;
            if (lambda < 0.0f) return 0.0f;
            if (lambda > 1.0f) return 1.0f;
            return lambda;
        }
        // Both a and b are near zero; power is independent of lambda but already exceeds target, return 0
        return 0.0f;
    }
    
    float disc = b * b - 4.0f * a * (c - target_power);
    if (disc < 0.0f) {
        // No real root; power still exceeds target_power even at lambda=0
        return 0.0f;
    }
    
    float sqrt_disc = sqrtf(disc);
    float lambda1 = (-b - sqrt_disc) / (2.0f * a);
    float lambda2 = (-b + sqrt_disc) / (2.0f * a);
    
    // Choose the root within [0, 1]
    if (lambda1 >= 0.0f && lambda1 <= 1.0f) {
        return lambda1;
    }
    if (lambda2 >= 0.0f && lambda2 <= 1.0f) {
        return lambda2;
    }

    return 0.0f;
}