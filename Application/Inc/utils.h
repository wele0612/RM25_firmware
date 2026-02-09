#ifndef __UTILS_H
#define __UTILS_H

#define PI (3.1415926f)

#define RADtoDEG (180.0f/PI)
#define DEGtoRAD (PI/180.0f)

// -----------------------------------------------------------
/** What the below macros are used for:
 * In STM32, there are memory areas called DTCM and ITCM. DTCM nice, as it provides same-cycle
 * data memory access, make your program fast...
 * 
 * The problem is, DTCM is not reachable for DMAs (it can only be access by CPU). So we need to
 * put all DMA-related variables and buffers in other area, such as RMA_D2.
 * 
 * Read the linker script FIRST!
 * Note: RAM_D2 sections may be NOLOAD, which is not initialized after power-on.
*/
#define RAM_D2_SECTION   __attribute__((section(".RAM_D2")))
#define DTCM_SECTION     __attribute__((section(".DTCMRAM")))
#define RAM_D1_SECTION   __attribute__((section(".RAM")))
// -----------------------------------------------------------

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
