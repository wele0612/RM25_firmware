#include <icm42688.h>

#include <Fusion.h>

#define ICM_CLK_GEN_TIM (&htim1)
#define ICM_USE_SPI     (&hspi2)

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


FusionAhrs ahrs;

/**
 * @brief ICM-42688-P初始化函数
 * @return 0成功，-1失败
 */

int icm_init(){
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, SET);
    FusionAhrsInitialise(&ahrs);
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
    if(icm_write_byte(0x11, 0x01) != 0) { // DEVICE_CONFIG寄存器，SOFT_RESET_CONFIG位
        return -1;
    }
    HAL_Delay(5); // 等待复位完成

    // 3. 配置加速度计 - 16kHz ODR, ±16g
    uint8_t accel_config = (ICM42688_ACCEL_FS_SEL_16G << 5) | ICM42688_ACCEL_ODR_16KHZ;
    if(icm_write_byte(ICM42688_ACCEL_CONFIG0, accel_config) != 0){
        return -1;
    }

    // 4. 配置陀螺仪 - 16kHz ODR, ±2000dps
    uint8_t gyro_config = (ICM42688_GYRO_FS_SEL_2000DPS << 5) | ICM42688_GYRO_ODR_16KHZ;
    if(icm_write_byte(ICM42688_GYRO_CONFIG0, gyro_config) != 0){
        return -1;
    }

    // 5. 配置电源模式 - 启用加速度计和陀螺仪（低噪声模式）
    uint8_t pwr_mgmt0 = (ICM42688_PWR_MGMT0_GYRO_LN_MODE << 2) | ICM42688_PWR_MGMT0_ACCEL_LN_MODE;
    if(icm_write_byte(ICM42688_PWR_MGMT0, pwr_mgmt0) != 0){
        return -1;
    }

    // 6. 配置滤波器（可选）- 根据IMU解算需求调整
    // 设置UI滤波器带宽以获得更好的性能
    // Bank 0, Register 0x52 - GYRO_ACCEL_CONFIG0
    uint8_t filter_config = 0x11; // 默认值，可根据需要调整
    if(icm_write_byte(0x52, filter_config) != 0){
        return -1;
    }

    // 7. 等待传感器稳定
    HAL_Delay(50);

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
    
    return 0;
}



int imu_update_ahrs(imu_data_t* imu, const float SAMPLE_PERIOD){

    icm_read_all_data(imu->acc,imu->gyro,&imu->tempreture);

    const FusionVector gyroscope = {imu->gyro[0], imu->gyro[1], imu->gyro[2]};
    const FusionVector accelerometer = {imu->acc[0], imu->acc[1], imu->acc[2],};

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD/10);

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    imu->yaw=euler.angle.yaw;
    imu->pitch=euler.angle.pitch;
    imu->roll=euler.angle.roll;

}