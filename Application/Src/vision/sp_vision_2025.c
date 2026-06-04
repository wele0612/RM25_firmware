#include <vision.h>
#include <global_variables.h>
#include <math.h>

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
        if(data == 'S'){
            vision_recv_fsm = RECV_FRAME_DATA;
        }else{
            vision_ptr = 0;
        }
    }else if(vision_ptr == VISION_FROM_ROS_SIZE){
        uint16_t crc16 = Get_CRC16_Check_Sum(vision_buf.buffer, VISION_FROM_ROS_SIZE-2, CRC16_INIT);
        if(vision_buf.packet.crc16 == crc16){
            memcpy(&vision_FromRos, &vision_buf, VISION_FROM_ROS_SIZE);
            vision_on_recv();
        }
        vision_recv_fsm = RECV_IDLE;
        vision_ptr = 0;
    }
}

uint8_t* vision_send_pack(McuToRosData_t* toRos){
    toRos->packet.head[0] = 'S';
    toRos->packet.head[1] = 'P';

    uint16_t crc16 = Get_CRC16_Check_Sum(toRos->buffer, VISION_TO_ROS_SIZE-2, CRC16_INIT);
    toRos->packet.crc16 = crc16;
    return toRos->buffer;
}

// sp vision坐标系:
// y指向左，z指向上，x指向前。
// 旋转方向为右手定则。yaw轴为z，pitch轴为y，roll轴为x。 
// q为wxyz顺序
void ypr_to_spvision_q(float yaw, float pitch, float roll, float *q){
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = cy * sp * cr + sy * cp * sr;
    q[3] = sy * cp * cr - cy * sp * sr;
}