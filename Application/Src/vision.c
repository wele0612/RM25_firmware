#include <vision.h>
#include <global_variables.h>

#include <config.h>

#ifdef CONFIG_PLATFORM_GIMBAL

static enum vision_recv_states_list_t{
    RECV_IDLE,
    RECV_FRAME_DATA
}vision_recv_fsm = RECV_IDLE;

/**
 * Called every time UART receive data from vision ROS system
 */

size_t vision_ptr=0;
RosToMcuData_t vision_buf;
void vision_recv_byte(uint8_t data){
    if(vision_ptr >= VISION_FROM_ROS_SIZE){
        vision_recv_fsm = RECV_IDLE;
        vision_ptr = 0;
    }
    vision_buf.buffer[vision_ptr] = data;
    vision_ptr++;

    if(vision_recv_fsm == RECV_IDLE){
        if(data == 0x5A){
            vision_recv_fsm = RECV_FRAME_DATA;
        }else{
            vision_ptr = 0;
        }
    }else if(vision_ptr == VISION_FROM_ROS_SIZE){
        uint16_t crc16 = Get_CRC16_Check_Sum(vision_buf.buffer, VISION_FROM_ROS_SIZE-2, CRC16_INIT);
        if(vision_buf.packet.checksum == crc16){
            memcpy(&vision_FromRos, &vision_buf, VISION_FROM_ROS_SIZE);
        }
        vision_recv_fsm = RECV_IDLE;
        vision_ptr = 0;
    }
}

uint8_t* vision_send_pack(){
    vision_ToRos.packet.header = 0x5A; // Be careful! MCU to ROS use different header.
    uint16_t crc16 = Get_CRC16_Check_Sum(vision_ToRos.buffer, VISION_TO_ROS_SIZE-2, CRC16_INIT);
    vision_ToRos.packet.checksum = crc16;
    return vision_ToRos.buffer;
}

#endif

