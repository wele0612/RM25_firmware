#include <vision.h>
#include <global_variables.h>

#include <config.h>

#ifdef CONFIG_PLATFORM_GIMBAL

static enum vision_recv_states_list_t{
    RECV_IDLE,
    RECV_FRAME_DATA
}vision_recv_fsm = RECV_IDLE;

size_t vision_ptr=0;
RosToMcuData_t vision_buf;

static void vision_on_recv(){
    return;
}

/**
 * Called every time UART receive data from vision ROS system
 */
void vision_recv_byte(uint8_t data){
    if(vision_ptr >= VISION_FROM_ROS_SIZE){
        vision_recv_fsm = RECV_IDLE;
        vision_ptr = 0;
    }
    vision_buf.buffer[vision_ptr] = data;
    vision_ptr++;

    if(vision_recv_fsm == RECV_IDLE){
        if(data == 0xA5){
            vision_recv_fsm = RECV_FRAME_DATA;
        }else{
            vision_ptr = 0;
        }
    }else if(vision_ptr == VISION_FROM_ROS_SIZE){
        uint16_t crc16 = Get_CRC16_Check_Sum(vision_buf.buffer, VISION_FROM_ROS_SIZE-2, CRC16_INIT);
        if(vision_buf.packet.checksum == crc16){
            memcpy(&vision_FromRos, &vision_buf, VISION_FROM_ROS_SIZE);
            vision_on_recv();
        }
        vision_recv_fsm = RECV_IDLE;
        vision_ptr = 0;
    }
}

uint8_t* vision_send_pack(McuToRosData_t* toRos){
    toRos->packet.header = 0x5A; // Be careful! MCU to ROS use different header.
    uint16_t crc16 = Get_CRC16_Check_Sum(toRos->buffer, VISION_TO_ROS_SIZE-2, CRC16_INIT);
    toRos->packet.checksum = crc16;
    return toRos->buffer;
}

// ------ Coordinate representation conversion
/* 装甲板切换阈值：45度（可修改） */
#define ARMOR_YAW_THRESHOLD     (PI / 4.0f)   // 0.785398 rad

/* 时间转换：ms 转 s */
#define MS_TO_S                 0.001f

/* 最小有效距离，防止除零 */
#define MIN_VALID_DISTANCE      0.01f

/**
 * @brief 获取预测后的装甲板瞄准角度
 * @param[out] yaw       目标方位角（弧度，odom坐标系，相对于x轴）
 * @param[out] pitch     目标俯仰角（弧度，odom坐标系，相对于水平面）
 * @param[out] vyaw      目标方位角速度（弧度/秒，用于前馈）
 * @param[out] vpitch    目标俯仰角速度（弧度/秒，用于前馈）
 * @param[out] distence  目标距离（米）
 * @param[in]  predict_time  预测时间（毫秒）
 * @return     0: 无有效目标，1: 成功计算
 */
int vision_get_armorplate(float *yaw, float *pitch, float *vyaw, float *vpitch,
    float *distence, float predict_time)
{
    // 检查是否有有效追踪目标
    if (!vision_FromRos.packet.tracking) {
        return 0;
    }

    /* 1. 提取当前状态数据 */
    float x = vision_FromRos.packet.x;
    float y = vision_FromRos.packet.y;
    float z = vision_FromRos.packet.z;
    float vx = vision_FromRos.packet.vx;
    float vy = vision_FromRos.packet.vy;
    float vz = vision_FromRos.packet.vz;
    float robot_yaw = vision_FromRos.packet.yaw;
    float v_yaw = vision_FromRos.packet.v_yaw;
    float r1 = vision_FromRos.packet.r1;
    float r2 = vision_FromRos.packet.r2;
    float dz = vision_FromRos.packet.dz;
    uint8_t armors_num = vision_FromRos.packet.armors_num;

    /* 2. 恒速度模型预测 predict_time 后的机器人中心状态 */
    float dt = predict_time * MS_TO_S;
    
    float x_pred = x + vx * dt;
    float y_pred = y + vy * dt;
    float z_pred = z + vz * dt;
    float yaw_pred = robot_yaw + v_yaw * dt;

    /* 3. 角度归一化到 [-PI, PI] */
    yaw_pred = wrap_to_pi(yaw_pred);

    /* 4. 确定要瞄准的装甲板参数（默认使用当前追踪的装甲板，即r1组） */
    float armor_yaw = yaw_pred;      // 当前追踪装甲板的方向
    float r = r1;                    // 当前装甲板半径
    float z_armor = z_pred;          // 当前装甲板高度

    /* 5. 4装甲板机器人切换逻辑 */
    if (armors_num == 4) {
        // 计算当前假设装甲板（使用r1）的位置
        float armor_x_temp = x_pred - r * cosf(armor_yaw);
        float armor_y_temp = y_pred - r * sinf(armor_yaw);
        
        // 计算从云台（假设在odom原点）到该装甲板的视线方位角
        float line_of_sight_yaw = atan2f(armor_y_temp, armor_x_temp);
        
        // 计算装甲板法向与视线的夹角（绝对值）
        float angle_diff = armor_yaw - line_of_sight_yaw;
        angle_diff = wrap_to_pi(angle_diff);  // 归一化差值到 [-PI, PI]
        float abs_angle_diff = fabsf(angle_diff);

        // 如果夹角超过45度，切换到下一个装甲板（+90度，提早出现）
        if (abs_angle_diff > ARMOR_YAW_THRESHOLD) {
            armor_yaw += PI / 2.0f;   // 旋转90度到下一个装甲板
            
            // 重新归一化
            armor_yaw = wrap_to_pi(armor_yaw);
            
            // 切换为另一组装甲板的几何参数（r2和高度差dz）

            // 对于非对称结构的机器人，取消注释这里！
            // r = r2;
            z_armor += dz;  // 注意：如果实际结构是另一组更低，这里应改为 -= dz，需根据机械结构调整
        }
    }
    /* 对于2板（平衡步兵）或3板（前哨站），不切换，始终使用r1和当前z */

    /* 6. 计算最终装甲板在odom坐标系中的绝对位置 */
    float armor_x = x_pred - r * cosf(armor_yaw);
    float armor_y = y_pred - r * sinf(armor_yaw);
    float armor_z = z_armor;

    /* 7. 计算云台（假设在odom原点）指向装甲板的瞄准角度 */
    float dx = armor_x;
    float dy = armor_y;
    float dz_pos = armor_z;
    
    float horizontal_dist = sqrtf(dx*dx + dy*dy);
    float total_dist = sqrtf(dx*dx + dy*dy + dz_pos*dz_pos);

    // 距离有效性检查
    if (total_dist < MIN_VALID_DISTANCE) {
        return 0;
    }

    float yaw_target = atan2f(dy, dx);           // 方位角
    float pitch_target = atan2f(dz_pos, horizontal_dist);  // 俯仰角

    /* 8. 计算前馈速度（vyaw, vpitch） */
    float yaw_rate_translation = (vy * cosf(yaw_pred) - vx * sinf(yaw_pred)) / horizontal_dist;
    *vyaw = v_yaw + yaw_rate_translation;
    *vpitch = vz / horizontal_dist;

    /* 9. 输出结果 */
    *yaw = yaw_target;
    *pitch = pitch_target;
    *distence = total_dist;

    return 1;
}
#endif