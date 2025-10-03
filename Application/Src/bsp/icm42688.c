#include <icm42688.h>

//#include <Fusion.h>
#include <math.h>
#include <string.h>

#define ICM_CLK_GEN_TIM (&htim1)
#define ICM_USE_SPI     (&hspi2)

#ifndef DEG2RAD
#define DEG2RAD (3.14159265358979323846f / 180.0f)
#endif
typedef struct {
    float q[4];
    float integralFB[3];
    float Kp;
    float Kp_max;
    float Ki;
    float integral_limit;
    float accel_reject_thresh;    // g
    float gyro_boost_thresh;      // rad/s
    float boost_duration;         // s
    float boost_timer;
    float last_gyro_norm;
    float gyro_drop_trigger;      // rad/s
    float gyro_dot_trigger;       // rad/s^2
    float centripetal_omega_thresh; // rad/s
    float centripetal_trust;      // scale for accel trust
    // smoothing + fast settle
    float accel_norm_lpf;
    float accel_lpf_alpha;        // LPF alpha for accel norm
    float fast_settle_timer;
    float fast_settle_duration;   // s
    // outputs
    float roll;
    float pitch;
    float yaw;
} MahonyAHRS;
MahonyAHRS ahrs;

static inline float clamp_f(float v, float lo, float hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static inline void Mahony_Init(MahonyAHRS *mahony, float Kp, float Ki) {
    (void)Kp; (void)Ki;
    mahony->q[0]=1.0f; mahony->q[1]=0.0f; mahony->q[2]=0.0f; mahony->q[3]=0.0f;
    mahony->integralFB[0]=mahony->integralFB[1]=mahony->integralFB[2]=0.0f;

    // 激进参数（用于快速收敛），可根据实测再微调
    mahony->Kp = 3.0f;            // 基础 Kp（比之前大）
    mahony->Kp_max = 30.0f;       // 极限 boost（短时使用）
    mahony->Ki = 0.06f;           // 积分用于慢漂移校正
    mahony->integral_limit = 0.25f;
    mahony->accel_reject_thresh = 0.22f; // g
    mahony->gyro_boost_thresh = 0.8f;    // rad/s (≈46°/s)
    mahony->boost_duration = 0.02f;      // 20 ms 短时 boost
    mahony->boost_timer = 0.0f;
    mahony->last_gyro_norm = 0.0f;

    mahony->gyro_drop_trigger = 0.4f;    // 更敏感的减速触发
    mahony->gyro_dot_trigger = 18.0f;    // 更敏感的陀螺突变触发
    mahony->centripetal_omega_thresh = 3.0f; // 降低阈值，更早检测向心影响
    mahony->centripetal_trust = 0.08f;   // 强力压低加速度可信度

    mahony->accel_norm_lpf = 1.0f;
    mahony->accel_lpf_alpha = 0.08f;     // 低通系数（短时平滑）
    mahony->fast_settle_duration = 0.015f; // 15 ms very-fast settle window
    mahony->fast_settle_timer = 0.0f;

    mahony->roll = mahony->pitch = mahony->yaw = 0.0f;
}

// Mahony update with accel-lpf, aggressive boost and fast-settle
static inline void Mahony_Update(MahonyAHRS *mahony, const float acc[3], const float gyro[3], float dt){
    float ax = acc[0], ay = acc[1], az = acc[2];
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float q0 = mahony->q[0], q1 = mahony->q[1], q2 = mahony->q[2], q3 = mahony->q[3];

    float accel_norm = sqrtf(ax*ax + ay*ay + az*az);
    if(accel_norm < 1e-6f) return;

    // LPF accel norm to avoid instant spikes messing trust calculation
    mahony->accel_norm_lpf = mahony->accel_norm_lpf * (1.0f - mahony->accel_lpf_alpha)
                             + accel_norm * mahony->accel_lpf_alpha;
    float anorm = mahony->accel_norm_lpf;

    // normalize accel vector using raw values (not lpf) for direction, but trust uses lpf
    ax /= accel_norm; ay /= accel_norm; az /= accel_norm;

    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    float accel_err = fabsf(anorm - 1.0f); // use LPF'ed norm
    float accel_trust = clamp_f(1.0f - accel_err / mahony->accel_reject_thresh, 0.0f, 1.0f);

    float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);
    float gyro_dot = (gyro_norm - mahony->last_gyro_norm) / dt;

    // sensitive triggers for boost (both drop and rapid change)
    float gyro_drop = mahony->last_gyro_norm - gyro_norm;
    if(gyro_drop > mahony->gyro_drop_trigger || fabsf(gyro_dot) > mahony->gyro_dot_trigger) {
        mahony->boost_timer = mahony->boost_duration;
    }
    mahony->last_gyro_norm = gyro_norm;

    if(mahony->boost_timer > 0.0f) {
        mahony->boost_timer -= dt;
        if(mahony->boost_timer < 0.0f) mahony->boost_timer = 0.0f;
    }

    // centripetal detection: high omega + significant accel error -> reduce trust strongly
    if(gyro_norm > mahony->centripetal_omega_thresh && accel_err > (mahony->accel_reject_thresh * 0.5f)) {
        accel_trust *= mahony->centripetal_trust;
    }

    // compute Kp_eff (aggressive)
    float Kp_eff;
    if(accel_trust < 0.12f) {
        Kp_eff = mahony->Kp * 0.08f;
        mahony->integralFB[0]=mahony->integralFB[1]=mahony->integralFB[2]=0.0f;
    } else {
        if(mahony->boost_timer > 0.0f || gyro_norm <= mahony->gyro_boost_thresh) {
            Kp_eff = mahony->Kp_max * accel_trust;
        } else {
            float t = 1.0f - clamp_f(gyro_norm / (mahony->gyro_boost_thresh * 3.0f), 0.0f, 1.0f);
            float base = mahony->Kp + (mahony->Kp_max - mahony->Kp) * t;
            Kp_eff = base * accel_trust;
        }
    }

    // fast settle: if very low angular rate and accel trusted, apply an extra short multiplier
    if(gyro_norm < 0.20f && accel_trust > 0.7f) {
        mahony->fast_settle_timer = mahony->fast_settle_duration;
    }
    if(mahony->fast_settle_timer > 0.0f) {
        mahony->fast_settle_timer -= dt;
        // temporary very strong Kp multiplier
        Kp_eff *= 2.2f;
    }

    // dynamic Ki: disable in high dynamic or low trust
    float Ki_eff = mahony->Ki;
    if(accel_trust < 0.30f || gyro_norm > (mahony->centripetal_omega_thresh * 1.2f)) Ki_eff = 0.0f;

    if(Ki_eff > 0.0f) {
        mahony->integralFB[0] = clamp_f(mahony->integralFB[0] + Ki_eff * ex * dt, -mahony->integral_limit, mahony->integral_limit);
        mahony->integralFB[1] = clamp_f(mahony->integralFB[1] + Ki_eff * ey * dt, -mahony->integral_limit, mahony->integral_limit);
        mahony->integralFB[2] = clamp_f(mahony->integralFB[2] + Ki_eff * ez * dt, -mahony->integral_limit, mahony->integral_limit);
        gx += mahony->integralFB[0];
        gy += mahony->integralFB[1];
        gz += mahony->integralFB[2];
    }

    gx += Kp_eff * ex;
    gy += Kp_eff * ey;
    gz += Kp_eff * ez;

    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * (q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * (q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * (q0*gz + q1*gy - q2*gx);

    q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
    float qnorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(qnorm > 1e-9f) { q0/=qnorm; q1/=qnorm; q2/=qnorm; q3/=qnorm; }

    mahony->q[0]=q0; mahony->q[1]=q1; mahony->q[2]=q2; mahony->q[3]=q3;

    mahony->roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    mahony->pitch = asinf(clamp_f(-2.0f*(q1*q3 - q0*q2), -1.0f, 1.0f));
    mahony->yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
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

    // init mahony
    Mahony_Init(&ahrs, 2.0f, 0.1f);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, SET);
    //FusionAhrsInitialise(&ahrs);
    HAL_Delay(10); // 等待电源稳定

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
    intf_config1 &= ~0x03; // CLKSEL设置为00 (使用内部RC振荡器，但RTC模式会使用外部时钟)
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

    HAL_TIM_Base_Start_IT(&htim6);// BUG: INT1 does not work. Use TIM6 interruption for now...

    HAL_Delay(10);

    return 0;
}


/**
 * @brief 批量读取所有传感器数据并转换为物理单位（高效方式）
 * @param accel_data: 加速度数据数组 [x, y, z]，单位：g
 * @param gyro_data: 陀螺仪数据数组 [x, y, z]，单位：deg/s
 * @param temperature: 温度数据指针，单位：°C
 * @return 0成功，-1失败
 */
int icm_read_all_data(float accel_data[3], float gyro_data[3], float *temperature){
    uint8_t buffer[14]; // 加速度6字节 + 陀螺仪6字节 + 温度2字节
    
    // 从加速度计X数据寄存器开始连续读取
    for(int i = 0; i < 14; i++){
        if(icm_read_byte(0x1F + i, &buffer[i]) != 0){
            return -1;
        }
    }
    
    // 解析原始数据
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    
    // 加速度原始数据
    accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X轴
    accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]); // Y轴
    accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]); // Z轴
    
    // 陀螺仪原始数据
    gyro_raw[0] = (int16_t)((buffer[6] << 8) | buffer[7]);   // X轴
    gyro_raw[1] = (int16_t)((buffer[8] << 8) | buffer[9]);   // Y轴
    gyro_raw[2] = (int16_t)((buffer[10] << 8) | buffer[11]); // Z轴
    
    // 温度原始数据
    temp_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
    
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
    
    // 温度转换
    *temperature = (temp_raw / 132.48f) + 25.0f; // °C
    
    // --- 重要：清除数据就绪中断标志 ---
    // 读取INT_STATUS寄存器会自动清除DATA_RDY_INT标志位
    uint8_t int_status;
    icm_read_byte(ICM42688_INT_STATUS, &int_status);
    
    return 0;
}

// --- AHRS update ---
int imu_update_ahrs(imu_data_t* imu, float SAMPLE_PERIOD){
    imu_data_t imu_old = *imu;

    if(icm_read_all_data(imu->acc, imu->gyro, &imu->tempreture)!=0) return -1;

    HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, SET);

    const float alpha=1.0f;
    for(int i=0;i<3;i++){
        imu->acc[i] = imu->acc[i]*alpha + imu_old.acc[i]*(1.0f-alpha);
        imu->gyro[i] = imu->gyro[i]*alpha + imu_old.gyro[i]*(1.0f-alpha);
    }

    float gyro_rad[3];
    gyro_rad[0]=imu->gyro[0]*DEG2RAD;
    gyro_rad[1]=imu->gyro[1]*DEG2RAD;
    gyro_rad[2]=imu->gyro[2]*DEG2RAD;

    Mahony_Update(&ahrs, imu->acc, gyro_rad, SAMPLE_PERIOD);

    imu->yaw = ahrs.yaw;
    imu->pitch = ahrs.pitch;
    imu->roll = ahrs.roll;

    HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, RESET);

    return 0;
}