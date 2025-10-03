#ifndef __ICM42688_H
#define __ICM42688_H

#include<spi.h>
#include<tim.h>

#include<stdint.h>

typedef struct imu_data_t{
    float gyro[3];
    float acc[3];
    float tempreture;
    float yaw;
    float pitch;
    float roll;
}imu_data_t;


int icm_init();

int icm_read_all_data(float accel_data[3], float gyro_data[3], float *temperature);
int imu_update_ahrs(imu_data_t *imu, const float SAMPLE_PERIOD);

/* ---------------------------
 * Common / bank selection
 * ---------------------------
 */

/* REG_BANK_SEL (accessible from all banks) */
#define ICM42688_REG_BANK_SEL            0x76u   /* REG_BANK_SEL (address 0x76, dec 118) */
#define ICM42688_REG_BANK_SEL_RESET      0x00u

/* Bank values for writing into REG_BANK_SEL */
#define ICM42688_BANK_0                  0x00u
#define ICM42688_BANK_1                  0x01u
#define ICM42688_BANK_2                  0x02u
#define ICM42688_BANK_3                  0x03u
#define ICM42688_BANK_4                  0x04u

/* WHO_AM_I register (Bank 0) */
#define ICM42688_WHO_AM_I                0x75u   /* 0x75 (117) */
#define ICM42688_WHO_AM_I_DEFAULT        0x47u   /* default value */

/* ---------------------------
 * Bank 0 - User Bank 0 register addresses & bitfields
 * (Bank 0 is default after reset)
 * ---------------------------
 *
 * Addresses shown in hex (and are AP register addresses when REG_BANK_SEL=0)
 */

/* Bank 0 register addresses */
#define ICM42688_DEVICE_CONFIG           0x11u /* 0x11 (17) */
#define ICM42688_DRIVE_CONFIG            0x13u /* 0x13 (19) */
#define ICM42688_INT_CONFIG              0x14u /* 0x14 (20) */
#define ICM42688_FIFO_CONFIG             0x16u /* 0x16 (22) */

#define ICM42688_TEMP_DATA1              0x1Du /* 0x1D (29) */
#define ICM42688_TEMP_DATA0              0x1Eu /* 0x1E (30) */

#define ICM42688_ACCEL_DATA_X1           0x1Fu /* 0x1F (31) */
#define ICM42688_ACCEL_DATA_X0           0x20u /* 0x20 (32) */
#define ICM42688_ACCEL_DATA_Y1           0x21u /* 0x21 (33) */
#define ICM42688_ACCEL_DATA_Y0           0x22u /* 0x22 (34) */
#define ICM42688_ACCEL_DATA_Z1           0x23u /* 0x23 (35) */
#define ICM42688_ACCEL_DATA_Z0           0x24u /* 0x24 (36) */

#define ICM42688_GYRO_DATA_X1            0x25u /* 0x25 (37) */
#define ICM42688_GYRO_DATA_X0            0x26u /* 0x26 (38) */
#define ICM42688_GYRO_DATA_Y1            0x27u /* 0x27 (39) */
#define ICM42688_GYRO_DATA_Y0            0x28u /* 0x28 (40) */
#define ICM42688_GYRO_DATA_Z1            0x29u /* 0x29 (41) */
#define ICM42688_GYRO_DATA_Z0            0x2Au /* 0x2A (42) */

#define ICM42688_TMST_FSYNCH             0x2Bu /* 0x2B (43) */
#define ICM42688_TMST_FSYNCL             0x2Cu /* 0x2C (44) */

#define ICM42688_INT_STATUS              0x2Du /* 0x2D (45) */
#define ICM42688_FIFO_COUNTH             0x2Eu /* 0x2E (46) */
#define ICM42688_FIFO_COUNTL             0x2Fu /* 0x2F (47) */
#define ICM42688_FIFO_DATA               0x30u /* 0x30 (48) */

#define ICM42688_APEX_DATA0              0x31u /* 0x31 (49) */
#define ICM42688_APEX_DATA1              0x32u /* 0x32 (50) */
#define ICM42688_APEX_DATA2              0x33u /* 0x33 (51) */
#define ICM42688_APEX_DATA3              0x34u /* 0x34 (52) */
#define ICM42688_APEX_DATA4              0x35u /* 0x35 (53) */
#define ICM42688_APEX_DATA5              0x36u /* 0x36 (54) */

#define ICM42688_INT_STATUS2             0x40u /* 0x40 (64) */
#define ICM42688_INT_STATUS3             0x41u /* 0x41 (65) */

#define ICM42688_SIGNAL_PATH_RESET       0x46u /* 0x46 (70) */

#define ICM42688_INTF_CONFIG0            0x4Bu /* 0x4B (75) - sometimes called INTF_CONFIG0 */
#define ICM42688_INTF_CONFIG1            0x4Du /* 0x4D (77) */
#define ICM42688_PWR_MGMT0               0x4Eu /* 0x4E (78) */

#define ICM42688_GYRO_CONFIG0            0x4Fu /* 0x4F (79) */
#define ICM42688_ACCEL_CONFIG0           0x50u /* 0x50 (80) */
#define ICM42688_GYRO_CONFIG1            0x51u /* 0x51 (81) */
#define ICM42688_GYRO_ACCEL_CONFIG0      0x52u /* 0x52 (82) */
#define ICM42688_ACCEL_CONFIG1           0x53u /* 0x53 (83) */

#define ICM42688_TMST_CONFIG             0x54u /* 0x54 (84) */
#define ICM42688_APEX_CONFIG0            0x56u /* 0x56 (86) */
#define ICM42688_SMD_CONFIG              0x57u /* 0x57 (87) */

/* FIFO_CONFIG1..3, FSYNC, INT_CONFIG0..1, INT_SOURCE0..10, FIFO_LOST_PKT0..1 */
#define ICM42688_FIFO_CONFIG1            0x58u /* 0x58 (88) */
#define ICM42688_FIFO_CONFIG2            0x59u /* 0x59 (89) */
#define ICM42688_FIFO_CONFIG3            0x5Au /* 0x5A (90) */

#define ICM42688_FSYNC_CONFIG            0x5Bu /* 0x5B (91) */

#define ICM42688_INT_CONFIG0             0x5Cu /* 0x5C (92) */
#define ICM42688_INT_CONFIG1             0x5Du /* 0x5D (93) */

#define ICM42688_INT_SOURCE0             0x5Eu /* 0x5E (94) */
#define ICM42688_XG_ST_DATA              0x5Fu /* 0x5F (95) */
#define ICM42688_YG_ST_DATA              0x60u /* 0x60 (96) */
#define ICM42688_ZG_ST_DATA              0x61u /* 0x61 (97) */

#define ICM42688_TMSTVAL0                0x62u /* 0x62 (98) */
#define ICM42688_TMSTVAL1                0x63u /* 0x63 (99) */
#define ICM42688_TMSTVAL2                0x64u /* 0x64 (100) */

#define ICM42688_INTF_CONFIG4            0x7Au /* 0x7A (122) */
#define ICM42688_INTF_CONFIG5            0x7Bu /* 0x7B (123) */
#define ICM42688_INTF_CONFIG6            0x7Cu /* 0x7C (124) */

#define ICM42688_OFFSET_USER0            0x77u /* 0x77 (119) */
#define ICM42688_OFFSET_USER1            0x78u /* 0x78 (120) */
#define ICM42688_OFFSET_USER2            0x79u /* 0x79 (121) */
#define ICM42688_OFFSET_USER3            0x7Au /* 0x7A (122) - note: overlap name; see datasheet */
#define ICM42688_OFFSET_USER4            0x7Bu /* 0x7B (123) */
#define ICM42688_OFFSET_USER5            0x7Cu /* 0x7C (124) */
#define ICM42688_OFFSET_USER6            0x7Du /* 0x7D (125) */
#define ICM42688_OFFSET_USER7            0x7Eu /* 0x7E (126) */
#define ICM42688_OFFSET_USER8            0x7Fu /* 0x7F (127) */

#define ICM42688_FIFO_LOST_PKT0          0x6Cu /* 0x6C (108) */
#define ICM42688_FIFO_LOST_PKT1          0x6Du /* 0x6D (109) */

#define ICM42688_SELF_TEST_CONFIG        0x70u /* 0x70 (112) */

/* ---------------------------
 * Bank 0 - bitfields (common / important)
 * --------------------------- */

/* DEVICE_CONFIG (0x11) */
#define ICM42688_DEVICE_CONFIG_SPI_MODE_SHIFT   4
#define ICM42688_DEVICE_CONFIG_SPI_MODE_MASK    (1U << ICM42688_DEVICE_CONFIG_SPI_MODE_SHIFT)
/* 0: SPI mode 0/3 (default) ; 1: SPI mode 1/2 */

#define ICM42688_DEVICE_CONFIG_SOFT_RESET_SHIFT 0
#define ICM42688_DEVICE_CONFIG_SOFT_RESET_MASK  (1U << ICM42688_DEVICE_CONFIG_SOFT_RESET_SHIFT)
/* Write 1 to trigger software reset; wait ~1 ms after write. */

/* DRIVE_CONFIG (0x13) */
#define ICM42688_DRIVE_CONFIG_I2C_SLEW_SHIFT   3
#define ICM42688_DRIVE_CONFIG_I2C_SLEW_MASK    (0x7U << ICM42688_DRIVE_CONFIG_I2C_SLEW_SHIFT)

#define ICM42688_DRIVE_CONFIG_SPI_SLEW_SHIFT   0
#define ICM42688_DRIVE_CONFIG_SPI_SLEW_MASK    (0x7U << ICM42688_DRIVE_CONFIG_SPI_SLEW_SHIFT)
/* Values 000..101 define slew rate ranges; 110..111 reserved */

/* INT_CONFIG (0x14) */
#define ICM42688_INT_CONFIG_INT2_MODE_SHIFT       5
#define ICM42688_INT_CONFIG_INT2_MODE_MASK        (1U << ICM42688_INT_CONFIG_INT2_MODE_SHIFT)
/* 0: pulsed ; 1: latched */

#define ICM42688_INT_CONFIG_INT2_DRIVE_SHIFT      4
#define ICM42688_INT_CONFIG_INT2_DRIVE_MASK       (1U << ICM42688_INT_CONFIG_INT2_DRIVE_SHIFT)
/* 0: open-drain ; 1: push-pull */

#define ICM42688_INT_CONFIG_INT2_POLARITY_SHIFT   3
#define ICM42688_INT_CONFIG_INT2_POLARITY_MASK    (1U << ICM42688_INT_CONFIG_INT2_POLARITY_SHIFT)

#define ICM42688_INT_CONFIG_INT1_MODE_SHIFT       2
#define ICM42688_INT_CONFIG_INT1_MODE_MASK        (1U << ICM42688_INT_CONFIG_INT1_MODE_SHIFT)

#define ICM42688_INT_CONFIG_INT1_DRIVE_SHIFT      1
#define ICM42688_INT_CONFIG_INT1_DRIVE_MASK       (1U << ICM42688_INT_CONFIG_INT1_DRIVE_SHIFT)

#define ICM42688_INT_CONFIG_INT1_POLARITY_SHIFT   0
#define ICM42688_INT_CONFIG_INT1_POLARITY_MASK    (1U << ICM42688_INT_CONFIG_INT1_POLARITY_SHIFT)

/* FIFO_CONFIG (0x16) */
#define ICM42688_FIFO_CONFIG_FIFO_MODE_SHIFT    6
#define ICM42688_FIFO_CONFIG_FIFO_MODE_MASK     (0x3U << ICM42688_FIFO_CONFIG_FIFO_MODE_SHIFT)
/* 00: Bypass (default); 01: Stream-to-FIFO; 10/11: STOP-on-FULL */

/* INT_STATUS (0x2D) - read clears */
#define ICM42688_INT_STATUS_UI_FSYNC_INT_SHIFT  6
#define ICM42688_INT_STATUS_UI_FSYNC_INT_MASK   (1U << ICM42688_INT_STATUS_UI_FSYNC_INT_SHIFT)

#define ICM42688_INT_STATUS_PLL_RDY_INT_SHIFT   5
#define ICM42688_INT_STATUS_PLL_RDY_INT_MASK    (1U << ICM42688_INT_STATUS_PLL_RDY_INT_SHIFT)

#define ICM42688_INT_STATUS_RESET_DONE_INT_SHIFT 4
#define ICM42688_INT_STATUS_RESET_DONE_INT_MASK  (1U << ICM42688_INT_STATUS_RESET_DONE_INT_SHIFT)

#define ICM42688_INT_STATUS_DATA_RDY_INT_SHIFT  3
#define ICM42688_INT_STATUS_DATA_RDY_INT_MASK   (1U << ICM42688_INT_STATUS_DATA_RDY_INT_SHIFT)

#define ICM42688_INT_STATUS_FIFO_THS_INT_SHIFT  2
#define ICM42688_INT_STATUS_FIFO_THS_INT_MASK   (1U << ICM42688_INT_STATUS_FIFO_THS_INT_SHIFT)

#define ICM42688_INT_STATUS_FIFO_FULL_INT_SHIFT 1
#define ICM42688_INT_STATUS_FIFO_FULL_INT_MASK  (1U << ICM42688_INT_STATUS_FIFO_FULL_INT_SHIFT)

#define ICM42688_INT_STATUS_AGC_RDY_INT_SHIFT   0
#define ICM42688_INT_STATUS_AGC_RDY_INT_MASK    (1U << ICM42688_INT_STATUS_AGC_RDY_INT_SHIFT)

/* FIFO_COUNTH / FIFO_COUNTL - reading FIFO_COUNTH latches both bytes */

/* FIFO_DATA (0x30) - read data from FIFO port */

/* TMST / TMSTVAL handling (0x54 TMST_CONFIG & 0x62..0x64 values) */
#define ICM42688_TMST_CONFIG_TMST_TO_REGS_EN_SHIFT 4
#define ICM42688_TMST_CONFIG_TMST_TO_REGS_EN_MASK  (1U << ICM42688_TMST_CONFIG_TMST_TO_REGS_EN_SHIFT)

#define ICM42688_TMST_CONFIG_TMST_RES_SHIFT 3
#define ICM42688_TMST_CONFIG_TMST_RES_MASK  (1U << ICM42688_TMST_CONFIG_TMST_RES_SHIFT)

#define ICM42688_TMST_CONFIG_TMST_DELTA_EN_SHIFT 2
#define ICM42688_TMST_CONFIG_TMST_DELTA_EN_MASK  (1U << ICM42688_TMST_CONFIG_TMST_DELTA_EN_SHIFT)

#define ICM42688_TMST_CONFIG_TMST_FSYNC_EN_SHIFT 1
#define ICM42688_TMST_CONFIG_TMST_FSYNC_EN_MASK  (1U << ICM42688_TMST_CONFIG_TMST_FSYNC_EN_SHIFT)

#define ICM42688_TMST_CONFIG_TMST_EN_SHIFT 0
#define ICM42688_TMST_CONFIG_TMST_EN_MASK  (1U << ICM42688_TMST_CONFIG_TMST_EN_SHIFT)

/* PWR_MGMT0 (0x4E) */
#define ICM42688_PWR_MGMT0_TEMP_DIS_SHIFT    5
#define ICM42688_PWR_MGMT0_TEMP_DIS_MASK     (1U << ICM42688_PWR_MGMT0_TEMP_DIS_SHIFT) /* 0: enabled (default) */

#define ICM42688_PWR_MGMT0_IDLE_SHIFT        4
#define ICM42688_PWR_MGMT0_IDLE_MASK         (1U << ICM42688_PWR_MGMT0_IDLE_SHIFT)

#define ICM42688_PWR_MGMT0_GYRO_MODE_SHIFT   2
#define ICM42688_PWR_MGMT0_GYRO_MODE_MASK    (0x3U << ICM42688_PWR_MGMT0_GYRO_MODE_SHIFT)
/* 00: OFF (default) ; 01: Standby ; 11: Low Noise (LN) ; 10: reserved */

#define ICM42688_PWR_MGMT0_ACCEL_MODE_SHIFT  0
#define ICM42688_PWR_MGMT0_ACCEL_MODE_MASK   (0x3U << ICM42688_PWR_MGMT0_ACCEL_MODE_SHIFT)
/* 00/01: off ; 10: Low Power (LP) ; 11: Low Noise (LN) */

/* GYRO_CONFIG0 (0x4F) - includes GYRO_ODR, GYRO_FS_SEL etc. (detailed bitfields available in datasheet) */
/* ACCEL_CONFIG0 (0x50) - includes ACCEL_ODR, ACCEL_FS_SEL etc. (see datasheet) */

/* SELF_TEST_CONFIG (0x70) */
#define ICM42688_SELF_TEST_CONFIG_ACCEL_ST_POWER_SHIFT 6
#define ICM42688_SELF_TEST_CONFIG_ACCEL_ST_POWER_MASK  (1U << ICM42688_SELF_TEST_CONFIG_ACCEL_ST_POWER_SHIFT)

#define ICM42688_SELF_TEST_CONFIG_EN_AZ_ST_SHIFT 5
#define ICM42688_SELF_TEST_CONFIG_EN_AZ_ST_MASK  (1U << ICM42688_SELF_TEST_CONFIG_EN_AZ_ST_SHIFT)

#define ICM42688_SELF_TEST_CONFIG_EN_AY_ST_SHIFT 4
#define ICM42688_SELF_TEST_CONFIG_EN_AY_ST_MASK  (1U << ICM42688_SELF_TEST_CONFIG_EN_AY_ST_SHIFT)

#define ICM42688_SELF_TEST_CONFIG_EN_AX_ST_SHIFT 3
#define ICM42688_SELF_TEST_CONFIG_EN_AX_ST_MASK  (1U << ICM42688_SELF_TEST_CONFIG_EN_AX_ST_SHIFT)

#define ICM42688_SELF_TEST_CONFIG_EN_GZ_ST_SHIFT 2
#define ICM42688_SELF_TEST_CONFIG_EN_GZ_ST_MASK  (1U << ICM42688_SELF_TEST_CONFIG_EN_GZ_ST_SHIFT)

#define ICM42688_SELF_TEST_CONFIG_EN_GY_ST_SHIFT 1
#define ICM42688_SELF_TEST_CONFIG_EN_GY_ST_MASK  (1U << ICM42688_SELF_TEST_CONFIG_EN_GY_ST_SHIFT)

#define ICM42688_SELF_TEST_CONFIG_EN_GX_ST_SHIFT 0
#define ICM42688_SELF_TEST_CONFIG_EN_GX_ST_MASK  (1U << ICM42688_SELF_TEST_CONFIG_EN_GX_ST_SHIFT)

/* ---------------------------
 * Bank 1 - Register addresses & bitfields
 * --------------------------- */

/* Bank 1 register addresses (these are used when REG_BANK_SEL = 1) */
#define ICM42688_BANK1_SENSOR_CONFIG0            0x03u /* 0x03 (3) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC2      0x0Bu /* 0x0B (11) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC3      0x0Cu /* 0x0C (12) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC4      0x0Du /* 0x0D (13) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC5      0x0Eu /* 0x0E (14) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC6      0x0Fu /* 0x0F (15) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC7      0x10u /* 0x10 (16) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC8      0x11u /* 0x11 (17) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC9      0x12u /* 0x12 (18) */
#define ICM42688_BANK1_GYRO_CONFIG_STATIC10     0x13u /* 0x13 (19) */

#define ICM42688_BANK1_XG_ST_DATA               0x5Fu /* 0x5F (95) */
#define ICM42688_BANK1_YG_ST_DATA               0x60u /* 0x60 (96) */
#define ICM42688_BANK1_ZG_ST_DATA               0x61u /* 0x61 (97) */

#define ICM42688_BANK1_TMSTVAL0                 0x62u /* 0x62 (98) */
#define ICM42688_BANK1_TMSTVAL1                 0x63u /* 0x63 (99) */
#define ICM42688_BANK1_TMSTVAL2                 0x64u /* 0x64 (100) */

#define ICM42688_BANK1_INTF_CONFIG4             0x7Au /* 0x7A (122) */
#define ICM42688_BANK1_INTF_CONFIG5             0x7Bu /* 0x7B (123) */
#define ICM42688_BANK1_INTF_CONFIG6             0x7Cu /* 0x7C (124) */

/* Bank 1 bitfields (selected / fully defined) */

/* SENSOR_CONFIG0 (0x03 in bank1) */
#define ICM42688_SENSOR_CONFIG0_ZG_DISABLE_SHIFT 5
#define ICM42688_SENSOR_CONFIG0_ZG_DISABLE_MASK  (1U << ICM42688_SENSOR_CONFIG0_ZG_DISABLE_SHIFT)
#define ICM42688_SENSOR_CONFIG0_YG_DISABLE_SHIFT 4
#define ICM42688_SENSOR_CONFIG0_YG_DISABLE_MASK  (1U << ICM42688_SENSOR_CONFIG0_YG_DISABLE_SHIFT)
#define ICM42688_SENSOR_CONFIG0_XG_DISABLE_SHIFT 3
#define ICM42688_SENSOR_CONFIG0_XG_DISABLE_MASK  (1U << ICM42688_SENSOR_CONFIG0_XG_DISABLE_SHIFT)
#define ICM42688_SENSOR_CONFIG0_ZA_DISABLE_SHIFT 2
#define ICM42688_SENSOR_CONFIG0_ZA_DISABLE_MASK  (1U << ICM42688_SENSOR_CONFIG0_ZA_DISABLE_SHIFT)
#define ICM42688_SENSOR_CONFIG0_YA_DISABLE_SHIFT 1
#define ICM42688_SENSOR_CONFIG0_YA_DISABLE_MASK  (1U << ICM42688_SENSOR_CONFIG0_YA_DISABLE_SHIFT)
#define ICM42688_SENSOR_CONFIG0_XA_DISABLE_SHIFT 0
#define ICM42688_SENSOR_CONFIG0_XA_DISABLE_MASK  (1U << ICM42688_SENSOR_CONFIG0_XA_DISABLE_SHIFT)
/* Note: 0 = sensor on, 1 = sensor disabled for corresponding axis */

/* GYRO_CONFIG_STATIC2 (0x0B) */
#define ICM42688_GYRO_CONFIG_STATIC2_GYRO_AAF_DIS_SHIFT 1
#define ICM42688_GYRO_CONFIG_STATIC2_GYRO_AAF_DIS_MASK  (1U << ICM42688_GYRO_CONFIG_STATIC2_GYRO_AAF_DIS_SHIFT)
#define ICM42688_GYRO_CONFIG_STATIC2_GYRO_NF_DIS_SHIFT 0
#define ICM42688_GYRO_CONFIG_STATIC2_GYRO_NF_DIS_MASK  (1U << ICM42688_GYRO_CONFIG_STATIC2_GYRO_NF_DIS_SHIFT)

/* GYRO_CONFIG_STATIC3 (0x0C) - GYRO_AAF_DELT (bits 5:0) */
#define ICM42688_GYRO_CONFIG_STATIC3_GYRO_AAF_DELT_SHIFT 0
#define ICM42688_GYRO_CONFIG_STATIC3_GYRO_AAF_DELT_MASK  (0x3Fu << ICM42688_GYRO_CONFIG_STATIC3_GYRO_AAF_DELT_SHIFT)

/* GYRO_CONFIG_STATIC10 (0x13) */
#define ICM42688_GYRO_CONFIG_STATIC10_GYRO_NF_BW_SEL_SHIFT 4
#define ICM42688_GYRO_CONFIG_STATIC10_GYRO_NF_BW_SEL_MASK  (0x7U << ICM42688_GYRO_CONFIG_STATIC10_GYRO_NF_BW_SEL_SHIFT)

/* INTF_CONFIG4 (0x7A in bank1) */
#define ICM42688_INTF_CONFIG4_I3C_BUS_MODE_SHIFT 6
#define ICM42688_INTF_CONFIG4_I3C_BUS_MODE_MASK (1U << ICM42688_INTF_CONFIG4_I3C_BUS_MODE_SHIFT)
#define ICM42688_INTF_CONFIG4_SPI_AP_4WIRE_SHIFT 1
#define ICM42688_INTF_CONFIG4_SPI_AP_4WIRE_MASK  (1U << ICM42688_INTF_CONFIG4_SPI_AP_4WIRE_SHIFT)

/* INTF_CONFIG5 (0x7B) PIN9_FUNCTION (bits 2:1) */
#define ICM42688_INTF_CONFIG5_PIN9_FUNCTION_SHIFT 1
#define ICM42688_INTF_CONFIG5_PIN9_FUNCTION_MASK  (0x3U << ICM42688_INTF_CONFIG5_PIN9_FUNCTION_SHIFT)
/* 00: INT2 ; 01: FSYNC ; 10: CLKIN ; 11: Reserved */

/* INTF_CONFIG6 (0x7C) */
#define ICM42688_INTF_CONFIG6_ASYNCTIME0_DIS_SHIFT 7
#define ICM42688_INTF_CONFIG6_ASYNCTIME0_DIS_MASK  (1U << ICM42688_INTF_CONFIG6_ASYNCTIME0_DIS_SHIFT)
#define ICM42688_INTF_CONFIG6_I3C_EN_SHIFT 4
#define ICM42688_INTF_CONFIG6_I3C_EN_MASK   (1U << ICM42688_INTF_CONFIG6_I3C_EN_SHIFT)
#define ICM42688_INTF_CONFIG6_I3C_IBI_BYTE_EN_SHIFT 3
#define ICM42688_INTF_CONFIG6_I3C_IBI_BYTE_EN_MASK  (1U << ICM42688_INTF_CONFIG6_I3C_IBI_BYTE_EN_SHIFT)
#define ICM42688_INTF_CONFIG6_I3C_IBI_EN_SHIFT 2
#define ICM42688_INTF_CONFIG6_I3C_IBI_EN_MASK   (1U << ICM42688_INTF_CONFIG6_I3C_IBI_EN_SHIFT)
#define ICM42688_INTF_CONFIG6_I3C_DDR_EN_SHIFT 1
#define ICM42688_INTF_CONFIG6_I3C_DDR_EN_MASK  (1U << ICM42688_INTF_CONFIG6_I3C_DDR_EN_SHIFT)
#define ICM42688_INTF_CONFIG6_I3C_SDR_EN_SHIFT 0
#define ICM42688_INTF_CONFIG6_I3C_SDR_EN_MASK  (1U << ICM42688_INTF_CONFIG6_I3C_SDR_EN_SHIFT)

/* ---------------------------
 * Bank 2 - Register addresses & bitfields
 * --------------------------- */

/* Bank 2 register addresses (use REG_BANK_SEL = 2) */
#define ICM42688_BANK2_ACCEL_CONFIG_STATIC2   0x03u /* 0x03 (3) */
#define ICM42688_BANK2_ACCEL_CONFIG_STATIC3   0x04u /* 0x04 (4) */
#define ICM42688_BANK2_ACCEL_CONFIG_STATIC4   0x05u /* 0x05 (5) */

#define ICM42688_BANK2_XA_ST_DATA             0x3Bu /* 0x3B (59) */
#define ICM42688_BANK2_YA_ST_DATA             0x3Cu /* 0x3C (60) */
#define ICM42688_BANK2_ZA_ST_DATA             0x3Du /* 0x3D (61) */

/* ACCEL_CONFIG_STATIC2 (0x03 in bank2) */
#define ICM42688_ACCEL_CONFIG_STATIC2_ACCEL_AAF_DELT_SHIFT 1
#define ICM42688_ACCEL_CONFIG_STATIC2_ACCEL_AAF_DELT_MASK  (0x3FU << ICM42688_ACCEL_CONFIG_STATIC2_ACCEL_AAF_DELT_SHIFT)
#define ICM42688_ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_SHIFT  0
#define ICM42688_ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_MASK   (1U << ICM42688_ACCEL_CONFIG_STATIC2_ACCEL_AAF_DIS_SHIFT)

/* ---------------------------
 * Bank 3 - Register addresses & bitfields
 * --------------------------- */

/* Bank 3 register addresses (use REG_BANK_SEL = 3) */
#define ICM42688_BANK3_CLKDIV                  0x2Au /* 0x2A (42) */

/* CLKDIV (0x2A) */
#define ICM42688_CLKDIV_VALUE_MASK             (0x7FU) /* bits 6:0 */

/* ---------------------------
 * Bank 4 - Register addresses & bitfields
 * --------------------------- */

/* Bank 4 register addresses (use REG_BANK_SEL = 4) */
#define ICM42688_BANK4_APEX_CONFIG1            0x40u /* 0x40 (64) */
#define ICM42688_BANK4_APEX_CONFIG2            0x41u /* 0x41 (65) */
#define ICM42688_BANK4_APEX_CONFIG3            0x42u /* 0x42 (66) */
#define ICM42688_BANK4_APEX_CONFIG4            0x43u /* 0x43 (67) */
#define ICM42688_BANK4_APEX_CONFIG5            0x44u /* 0x44 (68) */
#define ICM42688_BANK4_APEX_CONFIG6            0x45u /* 0x45 (69) */
#define ICM42688_BANK4_APEX_CONFIG7            0x46u /* 0x46 (70) */
#define ICM42688_BANK4_APEX_CONFIG8            0x47u /* 0x47 (71) */
#define ICM42688_BANK4_APEX_CONFIG9            0x48u /* 0x48 (72) */

#define ICM42688_BANK4_ACCEL_WOM_X_THR         0x4Au /* 0x4A (74) */
#define ICM42688_BANK4_ACCEL_WOM_Y_THR         0x4Bu /* 0x4B (75) */
#define ICM42688_BANK4_ACCEL_WOM_Z_THR         0x4Cu /* 0x4C (76) */

#define ICM42688_BANK4_INT_SOURCE6             0x4Du /* 0x4D (77) */
#define ICM42688_BANK4_INT_SOURCE7             0x4Eu /* 0x4E (78) */
#define ICM42688_BANK4_INT_SOURCE8             0x4Fu /* 0x4F (79) */
#define ICM42688_BANK4_INT_SOURCE9             0x50u /* 0x50 (80) */
#define ICM42688_BANK4_INT_SOURCE10            0x51u /* 0x51 (81) */

/* INT_SOURCE6..10 bitfields (examples) */
/* INT_SOURCE6 - bits control which APEX interrupts are routed (see datasheet for mapping) */

/* OFFSET_USER registers (0x77..0x7F are accessible in bank0 as well; bank mapping noted in datasheet) */
#define ICM42688_BANK4_OFFSET_USER0            0x77u
#define ICM42688_BANK4_OFFSET_USER1            0x78u
#define ICM42688_BANK4_OFFSET_USER2            0x79u
#define ICM42688_BANK4_OFFSET_USER3            0x7Au
#define ICM42688_BANK4_OFFSET_USER4            0x7Bu
#define ICM42688_BANK4_OFFSET_USER5            0x7Cu
#define ICM42688_BANK4_OFFSET_USER6            0x7Du
#define ICM42688_BANK4_OFFSET_USER7            0x7Eu
#define ICM42688_BANK4_OFFSET_USER8            0x7Fu

/* ---------------------------
 * Misc helper macros
 * --------------------------- */

/* combine bank and reg: to select a register the software must write REG_BANK_SEL then access the
 * register address at AP. Typically:
 *   write REG_BANK_SEL <- BANK_n
 *   read/write reg_addr (0x00..0x7F)
 */

/* Macro to extract bit(s) */
#define ICM42688_MASK(width, shift)    (((1U << (width)) - 1U) << (shift))

#endif
