#include <icm42688.h>

#include <utils.h>
#include <global_variables.h>

//#include <Fusion.h>
#include <math.h>
#include <string.h>

#define ICM_CLK_GEN_TIM (&htim1)
#define ICM_USE_SPI     (&hspi2)

#ifndef DEG2RAD
#define DEG2RAD (3.14159265358979323846f / 180.0f)
#endif

static inline float clamp_f(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


// ---------- MahonyAHRS struct (merged & smoothed) ----------
typedef struct {
    float q[4];
    float integralFB[3];

    float Kp;
    float Ki;

    float roll;
    float pitch;
    float yaw;
} MahonyAHRS;

static MahonyAHRS ahrs;

// ---------- Mahony init (call once in icm_init) ----------
static inline void Mahony_Init(MahonyAHRS *mahony) {

    mahony->q[0]=1.0f; mahony->q[1]=mahony->q[2]=mahony->q[3]=0.0f;
    mahony->integralFB[0]=mahony->integralFB[1]=mahony->integralFB[2]=0.0f;

    mahony->Kp = 1.0f;       
    mahony->Ki = 0.01f;

    mahony->roll = mahony->pitch = mahony->yaw = 0.0f;
}

static inline void Mahony_Update(MahonyAHRS *mahony, const float acc[3], const float gyro[3], float dt){
    float normalizedValue;
    float copy_acc[3], copy_gyro[3];
    copy_acc[0] = acc[0]; copy_acc[1] = acc[1]; copy_acc[2] = acc[2];
    copy_gyro[0] = gyro[0]; copy_gyro[1] = gyro[1]; copy_gyro[2] = gyro[2];
    float theory_gx, theory_gy, theory_gz;
    float q0 = mahony->q[0], q1 = mahony->q[1], q2 = mahony->q[2], q3 = mahony->q[3];
    float qa, qb, qc;
    float error_x, error_y, error_z;
    float acc_norm_sq;
    float kp, ki;  // effective gains

    acc_norm_sq = acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2];
    
    // Adaptive gain: boost PI when static (near 1g & low rotation) for faster bias convergence
    float gain_mul = 1.0f;
    if(acc_norm_sq > 0.0001f){
        float acc_norm = sqrtf(acc_norm_sq);
        float gyro_norm = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
        if(fabsf(acc_norm - 1.0f) < 0.15f && gyro_norm < 0.1f) gain_mul = 5.0f; // 3x gain when steady
    }
    kp = mahony->Kp * gain_mul;
    ki = mahony->Ki * gain_mul;

    if(acc_norm_sq > 0.0001f){
        normalizedValue = 1.0f/sqrtf(acc_norm_sq);
        copy_acc[0] *= normalizedValue; 
        copy_acc[1] *= normalizedValue;
        copy_acc[2] *= normalizedValue;

        theory_gx = 2.0f*(q1*q3 - q0*q2);
        theory_gy = 2.0f*(q2*q3 + q0*q1);
        theory_gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        error_x = (copy_acc[1]*theory_gz - copy_acc[2]*theory_gy);
        error_y = (copy_acc[2]*theory_gx - copy_acc[0]*theory_gz);
        error_z = (copy_acc[0]*theory_gy - copy_acc[1]*theory_gx);

        if (mahony->Ki > 0.0f){
            // Use adaptive ki
            mahony->integralFB[0] += ki * error_x * dt;
            mahony->integralFB[1] += ki * error_y * dt;
            mahony->integralFB[2] += ki * error_z * dt;
            
            #define INTEGRAL_LIMIT 0.02f
            if(mahony->integralFB[0] > INTEGRAL_LIMIT) mahony->integralFB[0] = INTEGRAL_LIMIT;
            else if(mahony->integralFB[0] < -INTEGRAL_LIMIT) mahony->integralFB[0] = -INTEGRAL_LIMIT;
            if(mahony->integralFB[1] > INTEGRAL_LIMIT) mahony->integralFB[1] = INTEGRAL_LIMIT;
            else if(mahony->integralFB[1] < -INTEGRAL_LIMIT) mahony->integralFB[1] = -INTEGRAL_LIMIT;
            if(mahony->integralFB[2] > INTEGRAL_LIMIT) mahony->integralFB[2] = INTEGRAL_LIMIT;
            else if(mahony->integralFB[2] < -INTEGRAL_LIMIT) mahony->integralFB[2] = -INTEGRAL_LIMIT;
            
            copy_gyro[0] += mahony->integralFB[0];
            copy_gyro[1] += mahony->integralFB[1];
            copy_gyro[2] += mahony->integralFB[2];
        } else {
            mahony->integralFB[0] = mahony->integralFB[1] = mahony->integralFB[2] = 0.0f;
        }

        // Use adaptive kp
        copy_gyro[0] += kp * error_x;
        copy_gyro[1] += kp * error_y;
        copy_gyro[2] += kp * error_z;
    }

    copy_gyro[0] *= (0.5f * dt);
    copy_gyro[1] *= (0.5f * dt);
    copy_gyro[2] *= (0.5f * dt);

    qa = q0; qb = q1; qc = q2;

    q0 += (-qb * copy_gyro[0] - qc * copy_gyro[1] - q3 * copy_gyro[2]); 
    q1 += (qa * copy_gyro[0] + qc * copy_gyro[2] - q3 * copy_gyro[1]);
    q2 += (qa * copy_gyro[1] - qb * copy_gyro[2] + q3 * copy_gyro[0]);
    q3 += (qa * copy_gyro[2] + qb * copy_gyro[1] - qc * copy_gyro[0]); 

    normalizedValue = 1.0f/sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= normalizedValue; q1 *= normalizedValue; q2 *= normalizedValue; q3 *= normalizedValue;

    mahony->q[0] = q0; mahony->q[1] = q1; mahony->q[2] = q2; mahony->q[3] = q3;

    #define PI 3.14159265359f
    mahony->roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    mahony->pitch = asinf(fminf(fmaxf(-2.0f*(q1*q3 - q0*q2), -1.0f), 1.0f));
    mahony->yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    
    if(mahony->roll > PI) mahony->roll -= 2.0f*PI; else if(mahony->roll < -PI) mahony->roll += 2.0f*PI;
    if(mahony->pitch > PI) mahony->pitch -= 2.0f*PI; else if(mahony->pitch < -PI) mahony->pitch += 2.0f*PI;
    if(mahony->yaw > PI) mahony->yaw -= 2.0f*PI; else if(mahony->yaw < -PI) mahony->yaw += 2.0f*PI;
}

/**
 * @brief 读取ICM-42688-P单个寄存器
 * @param addr: 寄存器地址
 * @param data: 读取的数据指针
 * @return 0成功，-1失败
 */
int icm_read_byte(uint8_t addr, uint8_t *data){
    uint8_t tx[2];
    uint8_t rx[2];

    // 设置读位（MSB = 1）
    tx[0] = addr | 0x80u;
    tx[1] = 0x00u; // 用于时钟输出数据的虚拟字节

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, RESET);
    int err = HAL_SPI_TransmitReceive(ICM_USE_SPI, tx, rx, 2u, 50);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, SET);

    if(err == HAL_OK){
        *data = rx[1]; // 第二个字节是寄存器值
        return 0;
    }
    return -1;
}
/**
 * @brief 写入ICM-42688-P单个寄存器
 * @param addr: 寄存器地址
 * @param data: 要写入的数据
 * @return 0成功，-1失败
 */
int icm_write_byte(uint8_t addr, uint8_t data){
    uint8_t tx[2];

    // 设置写位（MSB = 0）
    tx[0] = addr & 0x7Fu;
    tx[1] = data;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, RESET);
    int err = HAL_SPI_Transmit(ICM_USE_SPI, tx, 2u, 50);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, SET);

    return (err == HAL_OK) ? 0 : -1;
}

/**
 * @brief 设置寄存器bank
 * @param bank: bank号 (0-4)
 * @return 0成功，-1失败
 */
int icm_set_bank(uint8_t bank){
    if(bank > 4) return -1;
    return icm_write_byte(ICM42688_REG_BANK_SEL, bank & 0x07);
}

/**
 * @brief 尝试唤醒 ICM42688
 * @return 0=成功, -1=失败
 */
int icm_wakeup(void)
{
    // 拉低CS片选，确保SPI处于活动状态
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // 先尝试写入 PWR_MGMT0，唤醒IMU
    uint8_t wake_cmd = 0x0F;  // Gyro LN + Accel LN
    if (icm_write_byte(ICM42688_PWR_MGMT0, wake_cmd) != 0)
        return -1;

    // 延时几毫秒让内部时钟稳定
    HAL_Delay(5);

    return 0;
}


// 电源管理配置 - PWR_MGMT0 寄存器
typedef enum {
    ICM42688_PWR_MGMT0_ACCEL_OFF        = 0x00,  // 加速度计关闭
    ICM42688_PWR_MGMT0_ACCEL_LP_MODE    = 0x02,  // 加速度计低功耗模式
    ICM42688_PWR_MGMT0_ACCEL_LN_MODE    = 0x03,  // 加速度计低噪声模式
    
    ICM42688_PWR_MGMT0_GYRO_OFF         = 0x00,  // 陀螺仪关闭
    ICM42688_PWR_MGMT0_GYRO_STANDBY     = 0x01,  // 陀螺仪待机模式
    ICM42688_PWR_MGMT0_GYRO_LN_MODE     = 0x03,  // 陀螺仪低噪声模式
} icm42688_pwr_mgmt0_mode_t;

// 陀螺仪量程配置 - GYRO_CONFIG0 寄存器
typedef enum {
    ICM42688_GYRO_FS_SEL_2000DPS        = 0x00,  // ±2000dps
    ICM42688_GYRO_FS_SEL_1000DPS        = 0x01,  // ±1000dps
    ICM42688_GYRO_FS_SEL_500DPS         = 0x02,  // ±500dps
    ICM42688_GYRO_FS_SEL_250DPS         = 0x03,  // ±250dps
    ICM42688_GYRO_FS_SEL_125DPS         = 0x04,  // ±125dps
    ICM42688_GYRO_FS_SEL_62_5DPS        = 0x05,  // ±62.5dps
    ICM42688_GYRO_FS_SEL_31_25DPS       = 0x06,  // ±31.25dps
    ICM42688_GYRO_FS_SEL_15_625DPS      = 0x07,  // ±15.625dps
} icm42688_gyro_fs_sel_t;

// 加速度计量程配置 - ACCEL_CONFIG0 寄存器
typedef enum {
    ICM42688_ACCEL_FS_SEL_16G           = 0x00,  // ±16g
    ICM42688_ACCEL_FS_SEL_8G            = 0x01,  // ±8g
    ICM42688_ACCEL_FS_SEL_4G            = 0x02,  // ±4g
    ICM42688_ACCEL_FS_SEL_2G            = 0x03,  // ±2g
} icm42688_accel_fs_sel_t;

// 陀螺仪输出数据率配置 - GYRO_CONFIG0 寄存器
typedef enum {
    ICM42688_GYRO_ODR_32KHZ             = 0x01,  // 32kHz
    ICM42688_GYRO_ODR_16KHZ             = 0x02,  // 16kHz
    ICM42688_GYRO_ODR_8KHZ              = 0x03,  // 8kHz
    ICM42688_GYRO_ODR_4KHZ              = 0x04,  // 4kHz
    ICM42688_GYRO_ODR_2KHZ              = 0x05,  // 2kHz
    ICM42688_GYRO_ODR_1KHZ              = 0x06,  // 1kHz (默认)
    ICM42688_GYRO_ODR_200HZ             = 0x07,  // 200Hz
    ICM42688_GYRO_ODR_100HZ             = 0x08,  // 100Hz
    ICM42688_GYRO_ODR_50HZ              = 0x09,  // 50Hz
    ICM42688_GYRO_ODR_25HZ              = 0x0A,  // 25Hz
    ICM42688_GYRO_ODR_12_5HZ            = 0x0B,  // 12.5Hz
    ICM42688_GYRO_ODR_500HZ             = 0x0F,  // 500Hz
} icm42688_gyro_odr_t;

// 加速度计输出数据率配置 - ACCEL_CONFIG0 寄存器
typedef enum {
    ICM42688_ACCEL_ODR_32KHZ            = 0x01,  // 32kHz (LN模式)
    ICM42688_ACCEL_ODR_16KHZ            = 0x02,  // 16kHz (LN模式)
    ICM42688_ACCEL_ODR_8KHZ             = 0x03,  // 8kHz (LN模式)
    ICM42688_ACCEL_ODR_4KHZ             = 0x04,  // 4kHz (LN模式)
    ICM42688_ACCEL_ODR_2KHZ             = 0x05,  // 2kHz (LN模式)
    ICM42688_ACCEL_ODR_1KHZ             = 0x06,  // 1kHz (LN模式，默认)
    ICM42688_ACCEL_ODR_200HZ            = 0x07,  // 200Hz (LP或LN模式)
    ICM42688_ACCEL_ODR_100HZ            = 0x08,  // 100Hz (LP或LN模式)
    ICM42688_ACCEL_ODR_50HZ             = 0x09,  // 50Hz (LP或LN模式)
    ICM42688_ACCEL_ODR_25HZ             = 0x0A,  // 25Hz (LP或LN模式)
    ICM42688_ACCEL_ODR_12_5HZ           = 0x0B,  // 12.5Hz (LP或LN模式)
    ICM42688_ACCEL_ODR_6_25HZ           = 0x0C,  // 6.25Hz (LP模式)
    ICM42688_ACCEL_ODR_3_125HZ          = 0x0D,  // 3.125Hz (LP模式)
    ICM42688_ACCEL_ODR_1_5625HZ         = 0x0E,  // 1.5625Hz (LP模式)
    ICM42688_ACCEL_ODR_500HZ            = 0x0F,  // 500Hz (LP或LN模式)
} icm42688_accel_odr_t;


/**
 * @brief ICM-42688-P初始化函数
 * @return 0成功，-1失败
 */
int icm_init(){
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_Delay(5); // 等待电源稳定

    // 拉低CS片选，确保SPI处于活动状态
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // 先尝试写入 PWR_MGMT0，唤醒IMU
    uint8_t wake_cmd = 0x0F;  // Gyro LN + Accel LN
    if (icm_write_byte(ICM42688_PWR_MGMT0, wake_cmd) != 0)
        return -1;

    // init mahony
    Mahony_Init(&ahrs);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, SET);
    
    // 1. 检查设备ID
    uint8_t who_am_i;
    if(icm_read_byte(ICM42688_WHO_AM_I, &who_am_i) != 0){
        return -1;
    }
    
    if(who_am_i != ICM42688_WHO_AM_I_DEFAULT){
        return -1;
    }

    // 2. 软件复位（可选，确保设备处于已知状态）
    if(icm_write_byte(ICM42688_DEVICE_CONFIG, 0x01) != 0) { // DEVICE_CONFIG寄存器，SOFT_RESET_CONFIG位
        return -1;
    }
    HAL_Delay(5); // 等待复位完成

    // 3. 配置外部时钟输入 (40kHz)
    // 切换到Bank 1配置CLKIN引脚功能
    if(icm_set_bank(ICM42688_BANK_1) != 0) {
        return -1;
    }
    
    // 配置PIN9为CLKIN功能 (INT2/FSYNC/CLKIN引脚)
    uint8_t intf_config5;
    if(icm_read_byte(ICM42688_INTF_CONFIG5, &intf_config5) != 0) {
        return -1;
    }
    intf_config5 &= ~ICM42688_INTF_CONFIG5_PIN9_FUNCTION_MASK; // 清除PIN9_FUNCTION位
    intf_config5 |= (0x02 << 1);  // ICM42688_PIN9_FUNCTION_CLKIN 是 0x02, ICM42688_INTF_CONFIG5_PIN9_FUNCTION_POS 是 1
    if(icm_write_byte(ICM42688_INTF_CONFIG5, intf_config5) != 0) {
        return -1;
    }

    // 切换回Bank 0
    if(icm_set_bank(ICM42688_BANK_0) != 0) {
        return -1;
    }

    // 启用RTC模式使用外部时钟
    uint8_t intf_config1;
    if(icm_read_byte(ICM42688_INTF_CONFIG1, &intf_config1) != 0) {
        return -1;
    }
    intf_config1 |= 0x04; // 设置RTC_MODE位为1
    intf_config1 &= ~0x03; // 清除bits 1:0
    intf_config1 |= 0x01;  // CLKSEL = 01 (PLL/RC自动选择 - 使用外部时钟)
    if(icm_write_byte(ICM42688_INTF_CONFIG1, intf_config1) != 0) {
        return -1;
    }

    // 4. 配置加速度计 - 16kHz ODR, ±16g (实际ODR会按比例缩放为20kHz)
    uint8_t accel_config = (0x00 << 5) | 0x02; // FSR=16g, ODR=16kHz
    if(icm_write_byte(ICM42688_ACCEL_CONFIG0, accel_config) != 0){
        return -1;
    }

    // 5. 配置陀螺仪 - 16kHz ODR, ±2000dps (实际ODR会按比例缩放为20kHz)
    uint8_t gyro_config = (0x00 << 5) | 0x02; // FSR=2000dps, ODR=16kHz
    if(icm_write_byte(ICM42688_GYRO_CONFIG0, gyro_config) != 0){
        return -1;
    }

    // 6. 启用抗混叠滤波器
    // 切换到Bank 1配置陀螺仪抗混叠滤波器
    if(icm_set_bank(ICM42688_BANK_1) != 0) {
        return -1;
    }
    
    // 启用陀螺仪抗混叠滤波器
    uint8_t gyro_config_static2;
    if(icm_read_byte(ICM42688_BANK1_GYRO_CONFIG_STATIC2, &gyro_config_static2) != 0) {
        return -1;
    }
    gyro_config_static2 &= ~0x02; // 清除GYRO_AAF_DIS位 (0=启用抗混叠滤波器)
    if(icm_write_byte(ICM42688_BANK1_GYRO_CONFIG_STATIC2, gyro_config_static2) != 0) {
        return -1;
    }

    // 切换到Bank 2配置加速度计抗混叠滤波器
    if(icm_set_bank(ICM42688_BANK_2) != 0) {
        return -1;
    }
    
    // 启用加速度计抗混叠滤波器
    uint8_t accel_config_static2;
    if(icm_read_byte(ICM42688_BANK2_ACCEL_CONFIG_STATIC2, &accel_config_static2) != 0) {
        return -1;
    }
    accel_config_static2 &= ~0x01; // 清除ACCEL_AAF_DIS位 (0=启用抗混叠滤波器)
    if(icm_write_byte(ICM42688_BANK2_ACCEL_CONFIG_STATIC2, accel_config_static2) != 0) {
        return -1;
    }

    // 切换回Bank 0
    if(icm_set_bank(ICM42688_BANK_0) != 0) {
        return -1;
    }
    
    // --- Step 7: Enable accel + gyro in LN mode ---
    uint8_t pwr_mgmt0 = (0x03 << 2) | 0x03; // Gyro LN + Accel LN
    if(icm_write_byte(ICM42688_PWR_MGMT0, pwr_mgmt0) != 0){
        return -1;
    }
    HAL_Delay(10); // wait sensors startup

    // --- Step 8: Configure INT1 pin (active high, push-pull, pulse) ---
    uint8_t int_config = 0;
    int_config |= (1 << 0); // INT1_POLARITY = 1 (active high)
    int_config |= (1 << 1); // INT1_DRIVE_CIRCUIT = 1 (push-pull)
    int_config |= (0 << 2); // INT1_MODE = 0 (pulse, not latched)
    if(icm_write_byte(ICM42688_INT_CONFIG, int_config) != 0) {
        return -1;
    }

    // --- 关键：配置INT_CONFIG1寄存器 ---
    // 清除INT_ASYNC_RESET位以确保INT1正常工作
    uint8_t int_config1;
    if(icm_read_byte(ICM42688_INT_CONFIG1, &int_config1) != 0) {
        return -1;
    }
    int_config1 &= ~0x10; // 清除bit 4 (INT_ASYNC_RESET)
    // 设置适当的中断脉冲持续时间
    int_config1 |= (0 << 6); // INT_TPULSE_DURATION = 0 (100µs脉冲)
    int_config1 |= (0 << 5); // INT_TDEASSERT_DISABLE = 0 (启用去断言)
    if(icm_write_byte(ICM42688_INT_CONFIG1, int_config1) != 0) {
        return -1;
    }

    // --- Step 9: Enable Data Ready interrupt on INT1 ---
    // write exact value instead of OR to avoid junk bits
    //uint8_t int_source0 = 0x08; // UI_DRDY_INT1_EN = bit3
    uint8_t int_source0 = 0x18;
    if(icm_write_byte(ICM42688_INT_SOURCE0, int_source0) != 0) {
        return -1;
    }

    // --- Step 10: Clear any pending interrupts by reading INT_STATUS ---
    uint8_t dummy=0;
    icm_read_byte(ICM42688_INT_STATUS, &dummy);

    HAL_Delay(10);

    imu_state = IMU_RUNNING;

    return 0;
}


/**
 * @brief 批量读取所有传感器数据并转换为物理单位（高效方式）
 * @param accel_data: 加速度数据数组 [x, y, z]，单位：g
 * @param gyro_data: 陀螺仪数据数组 [x, y, z]，单位：deg/s
 * @param temperature: 温度数据指针，单位：°C
 * @return 0成功，-1失败
 */
int icm_read_all_data(float accel_data[3], float gyro_data[3]){
    uint8_t buffer[12]; // 加速度6字节 + 陀螺仪6字节
    
    // 从加速度计X数据寄存器开始连续读取
    for(int i = 0; i < 12; i++){
        if(icm_read_byte(0x1F + i, &buffer[i]) != 0){
            return -1;
        }
    }
    
    // 解析原始数据
    int16_t accel_raw[3], gyro_raw[3];
    
    // 加速度原始数据
    accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X轴
    accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]); // Y轴
    accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]); // Z轴
    
    // 陀螺仪原始数据
    gyro_raw[0] = (int16_t)((buffer[6] << 8) | buffer[7]);   // X轴
    gyro_raw[1] = (int16_t)((buffer[8] << 8) | buffer[9]);   // Y轴
    gyro_raw[2] = (int16_t)((buffer[10] << 8) | buffer[11]); // Z轴
    
    // // 温度原始数据
    // temp_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // 转换为物理单位
    
    // 加速度转换 (±16g, 灵敏度 2048 LSB/g)
    const float accel_sensitivity = 2048.0f; // LSB/g for ±16g
    accel_data[0] = accel_raw[0] / accel_sensitivity; // X轴 (g)
    accel_data[1] = accel_raw[1] / accel_sensitivity; // Y轴 (g)
    accel_data[2] = accel_raw[2] / accel_sensitivity; // Z轴 (g)
    
    // 陀螺仪转换 (±2000dps, 灵敏度 16.4 LSB/dps)
    const float gyro_sensitivity = 16.4f; // LSB/dps for ±2000dps
    gyro_data[0] = gyro_raw[0] / gyro_sensitivity; // X轴 (deg/s)
    gyro_data[1] = gyro_raw[1] / gyro_sensitivity; // Y轴 (deg/s)
    gyro_data[2] = gyro_raw[2] / gyro_sensitivity; // Z轴 (deg/s)
    
    // // 温度转换
    // *temperature = (temp_raw / 132.48f) + 25.0f; // °C
    
    // --- 重要：清除数据就绪中断标志 ---
    // 读取INT_STATUS寄存器会自动清除DATA_RDY_INT标志位
    uint8_t int_status;
    icm_read_byte(ICM42688_INT_STATUS, &int_status);
    
    return 0;
}

#define ICM_REG_ACCEL_XOUT_H   0x1F
#define ICM_SPI_TIMEOUT_MS     50
#define ICM_READ_LEN           12

#define ICM_DMA_TX_LEN         (1 + ICM_READ_LEN)
#define ICM_DMA_RX_LEN         (1 + ICM_READ_LEN)

RAM_D2_SECTION uint8_t icm_tx_buffer[20];
RAM_D2_SECTION uint8_t icm_rx_buffer[20];
volatile uint8_t icm_dma_done;

/**
 * @brief SPI DMA完成回调
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
        icm_dma_done = 1;
    }
}

/**
 * @brief 启动一次ICM DMA读取（非阻塞）
 */
static int icm_read_all_data_dma_begin(void)
{
    icm_tx_buffer[0] = ICM_REG_ACCEL_XOUT_H | 0x80u; // 读位=1
    memset(&icm_tx_buffer[1], 0x00, ICM_READ_LEN);   // dummy bytes

    icm_dma_done = 0; // 清除完成标志

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(
        ICM_USE_SPI,
        icm_tx_buffer,
        icm_rx_buffer,
        ICM_DMA_TX_LEN
    );

    if (ret != HAL_OK) {
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        return -1;
    }

    return 0;
}

/**
 * @brief 同步方式读取ICM42688加速度与陀螺仪数据（DMA + 等待完成）
 * @param accel_data: 加速度 [3], 单位 g
 * @param gyro_data:  陀螺仪 [3], 单位 deg/s
 * @return 0 成功, -1 失败或超时
 */
int icm_read_all_data_dma(float accel_data[3], float gyro_data[3])
{

    if (icm_dma_done == 0) {
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        // HAL_SPI_DMAStop(ICM_USE_SPI);
        icm_read_all_data_dma_begin();
        return -1; // unable to fetch data (due to not enabled/other bugs)
    } else {
        // DMA传输完成后释放CS
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    }

    // 拷贝寄存器数据（跳过第一个dummy字节）
    uint8_t *buffer = &icm_rx_buffer[1];

    // --- 解析原始数据 ---
    int16_t accel_raw[3];
    int16_t gyro_raw[3];

    accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

    gyro_raw[0] = (int16_t)((buffer[6] << 8) | buffer[7]);
    gyro_raw[1] = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyro_raw[2] = (int16_t)((buffer[10] << 8) | buffer[11]);

    // --- 转换为物理单位 ---
    // const float accel_sensitivity = 2048.0f; // ±16g
    const float accel_sensitivity = 1024.0f; // ±16g
    const float gyro_sensitivity  = 16.4f;   // ±2000 dps

    accel_data[0] = accel_raw[0] / accel_sensitivity;
    accel_data[1] = accel_raw[1] / accel_sensitivity;
    accel_data[2] = accel_raw[2] / accel_sensitivity;

    gyro_data[0] = gyro_raw[0] / gyro_sensitivity;
    gyro_data[1] = gyro_raw[1] / gyro_sensitivity;
    gyro_data[2] = gyro_raw[2] / gyro_sensitivity;

    // Call dma read at the end start reading at background
    if (icm_read_all_data_dma_begin() != 0)
        return -1;

    return 0;
}


//============================ AHRS ===================================
iir_filter_t iir[sizeof(imu_data_t)/4];

// --- AHRS update ---
int imu_update_ahrs(imu_data_t* imu, imu_data_t* imu_clean, float SAMPLE_PERIOD){
    if(imu_state == IMU_RESET){
        return -1;
    }

    //if(icm_read_all_data(imu->acc, imu->gyro)!=0) return -1;
    if(icm_read_all_data_dma(imu->acc, imu->gyro)!=0) return -1;

    // apply calibration
    for(int i=0;i<3;i++){
        imu->gyro[i] -= robot_config.imu_gyro_offset[i];
    }
    
    // 二阶Butterworth滤波器 (fs=20kHz, fc=600Hz) 
    const float a[4] = {1.0f, -1.734725768809275f, 0.7660066009432638f, 0.0f}; 
    const float b[4] = {0.007820208033497193f, 0.015640416066994386f, 0.007820208033497193f, 0.0f};

    for(int i=0;i<3;i++){ // DO NOT filter Pitch, Yaw and Roll due to Warp-to-PI
        imu_clean->acc[i]=filter_iir_eval(&iir[i], imu->acc[i], 2, a, b);
        imu_clean->gyro[i]=filter_iir_eval(&iir[i], imu->gyro[i], 2, a, b);
    }

    float gyro_rad[3];
    gyro_rad[0]=imu->gyro[0]*DEG2RAD;
    gyro_rad[1]=imu->gyro[1]*DEG2RAD;
    gyro_rad[2]=imu->gyro[2]*DEG2RAD;

    // 13% interrupt
    Mahony_Update(&ahrs, imu->acc, gyro_rad, SAMPLE_PERIOD);

    imu->yaw = ahrs.yaw;
    imu->pitch = ahrs.pitch;
    imu->roll = ahrs.roll;

    imu_clean->pitch = imu->pitch;
    imu_clean->yaw = imu->yaw;
    imu_clean->roll = imu->roll;

    return 0;
}

static imu_data_t imu, imu_clean;

void imu_update(){
    // HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, SET);
    imu_update_ahrs(&imu, &imu_clean, 1.0f/20e3f);
    // HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, RESET);
    // 目前是读取和传输同时开启，这个过程需要改变，改成先在传输时候dma在读取，这样dma数据到了下一个传输可以直接传输
    // 这两个操作得在20khz同一个cycle下执行，读取的dma数据会在下一个cycle出现
    // 实现：先传输在继续开启读写
}

void imu_obtain_data(imu_data_t *data){
    __disable_irq();
    memcpy(data, &imu_clean, sizeof(imu_data_t));
    __enable_irq();
    return;
}
