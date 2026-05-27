#ifndef _VISION_H
#define _VISION_H

#include <stdint.h>

// RM-Vision style UART definitions

/**
 * @brief 电控发送给ROS的数据包 (对应ROS端的 ReceivePacket)
 * @note  总大小: 28字节 (1+1+24+2)
 * @note  字节序: 小端序 (Little Endian，ARM默认)
 * @note  校验: CRC16-CCITT，初始值0xFFFF，范围[0, 25]字节
 */
typedef struct __attribute__((packed)) {
    uint8_t header;              // 帧头: 固定 0x5A

    uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];    // wxyz顺序 四元数

    float yaw;
    float yaw_vel;

    float pitch;
    float pitch_vel;
    
    float bullet_speed;
    uint16_t bullet_count;  // 子弹累计发送次数

    uint16_t crc16;

} McuToRosPacket_t;

#define VISION_TO_ROS_SIZE (sizeof(McuToRosPacket_t))
typedef union {
    uint8_t buffer[VISION_TO_ROS_SIZE];
    McuToRosPacket_t packet;
} McuToRosData_t;

typedef struct __attribute__((packed)) {
    uint8_t header;              // 帧头: 固定 0xA5
    
    // 状态字节 (位域定义)
    uint8_t tracking : 1;        // 0:未追踪/丢失, 1:正在追踪(含临时丢失预测)
    uint8_t id : 3;              // 目标ID: 
                                 //   0: outpost(前哨站)/unknown
                                 //   1-5: 英雄/工程/步兵3/4/5号
                                 //   6: guard(哨兵)
                                 //   7: base(基地)
    uint8_t armors_num : 3;      // 装甲板数量: 2-平衡步兵, 3-前哨站, 4-普通机器人
    uint8_t reserved : 1;        // 保留位，置0
    
    float x;                     // 机器人中心X坐标 [m] (世界坐标系/odom)
    float y;                     // 机器人中心Y坐标 [m]
    float z;                     // 装甲板高度Z [m] (平均高度或当前装甲板高度)
    float yaw;                   // 机器人朝向 [rad] (-π ~ +π，已解算为连续角度)
    float vx;                    // X方向速度 [m/s] (机器人中心速度)
    float vy;                    // Y方向速度 [m/s]
    float vz;                    // Z方向速度 [m/s] (装甲板升降速度，通常为0)
    float v_yaw;                 // 旋转角速度 [rad/s] (正值为逆时针)
    float r1;                    // 当前追踪装甲板半径 [m] (中心到装甲板距离)
    float r2;                    // 另一组装甲板半径 [m] (仅4板机器人有效，否则=r1)
    float dz;                    // 两组装甲板高度差 [m] (仅4板机器人有效，否则=0)
    
    uint16_t checksum;           // CRC16校验码 (校验范围: 第0字节~第45字节)
} RosToMcuPacket_t;

#define VISION_FROM_ROS_SIZE (sizeof(RosToMcuPacket_t))
typedef union {
    uint8_t buffer[VISION_FROM_ROS_SIZE];
    RosToMcuPacket_t packet;
} RosToMcuData_t;

#endif

void vision_recv_byte(uint8_t data);
uint8_t *vision_send_pack(McuToRosData_t *toRos);

int vision_get_armorplate(float *yaw, float *pitch, float *vyaw, float *vpitch,
    float *distence, float predict_time);

int vision_get_body(float *yaw, float *pitch, float *vyaw, float *vpitch,
    float *distence, float predict_time);

float third_order_fit(const float coes[], float x);
