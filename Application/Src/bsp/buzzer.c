#include <buzzer.h>

#define BUZZER_TIM TIM12

#define BUZZER_TICK_LENGTH 1.0E-6f

void buzzer_on(){
    LL_TIM_OC_SetMode(BUZZER_TIM, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_EnableAllOutputs(BUZZER_TIM);
    LL_TIM_EnableCounter(BUZZER_TIM);
    LL_TIM_CC_EnableChannel(BUZZER_TIM, LL_TIM_CHANNEL_CH2);
}

void buzzer_off(){
    LL_TIM_OC_SetMode(BUZZER_TIM, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FORCED_INACTIVE);
}
void buzzer_set_freq(float freq){
    int reload=(int)(0.5f/(freq*BUZZER_TICK_LENGTH))-1;
    if(reload<=1) reload=1;
    LL_TIM_SetAutoReload(BUZZER_TIM, reload);
}

void buzzer_DJI_startup(){
    const int delay_ms = 255;
    buzzer_on();
    buzzer_set_freq(TUNE_C6);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_D6);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_G6);
    HAL_Delay(delay_ms+100);
    buzzer_off();
}

void buzzer_calibration_startup(){
    const int delay_ms = 100;
    buzzer_on();
    buzzer_set_freq(TUNE_D6);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_C6_SHARP_D6_FLAT);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_E6);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_F6_SHARP_G6_FLAT);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_G6);
    HAL_Delay(delay_ms);
    buzzer_off();
}

void buzzer_calibration_done(){
    const int delay_ms = 100;
    buzzer_set_freq(TUNE_G5);
    for(int i=0;i<3;i++){
        buzzer_on();
        HAL_Delay(delay_ms);
        buzzer_off();
        HAL_Delay(delay_ms);
    }
}

void buzzer_motor_notconnected(int id){
    const int delay_ms=200;
    buzzer_on();
    buzzer_set_freq(TUNE_B6);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_D6);
    HAL_Delay(delay_ms);
    buzzer_set_freq(TUNE_F6);
    HAL_Delay(delay_ms);
    buzzer_off();
    HAL_Delay(delay_ms*2);

    for(int i=0;i<id;i++){
        buzzer_set_freq(TUNE_D6_SHARP_E6_FLAT);
        buzzer_on();
        HAL_Delay(delay_ms*2);
        buzzer_off();
        HAL_Delay(delay_ms*2);
    }
    
}
