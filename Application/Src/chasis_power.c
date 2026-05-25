#include <chasis_power.h>

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
    0.13301039,   // K4
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