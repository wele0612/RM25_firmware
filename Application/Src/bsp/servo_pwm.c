#include<servo_pwm.h>

volatile uint16_t g_servo_pulse_us = 1500;

void servo_init(){
    HAL_TIM_Base_Start_IT(&htim2);      // 启动时基中断（20ms周期）
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
}

// 设置舵机角度的API（0-180度映射到500-2500μs）
void servo_SetAngle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 360) angle = 360;
    // 线性映射：0度=500μs, 180度=2500μs
    g_servo_pulse_us = (uint16_t)(500.0f + (angle / 180.0f) * 2000.0f);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // 20ms周期开始：开启GPIO
        HAL_GPIO_WritePin(SERVO_PWM1_GPIO_Port, SERVO_PWM1_Pin, GPIO_PIN_SET);
        
        // 动态设置比较值（脉宽时间）
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_servo_pulse_us);
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        // 脉宽时间到达：关闭GPIO
        HAL_GPIO_WritePin(SERVO_PWM1_GPIO_Port, SERVO_PWM1_Pin, GPIO_PIN_RESET);
    }
}
