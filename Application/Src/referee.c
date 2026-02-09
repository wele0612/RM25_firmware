#include <referee.h>
#include <stddef.h>

#include<global_variables.h>

#include<application.h>

__weak void referee_on_receive(){
    return;
}


// CRC8 generator polynomial: G(x) = x8 + x5 + x4 + 1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

static unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return ucCRC8;
}

// CRC16
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check, Stream length, initialized checksum
** Output: CRC checksum
*/
static uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
    uint8_t chData;
    if (pchMessage == NULL) {
        return 0xFFFF;
    }
    while (dwLength--) {
        chData = *pchMessage++;
        wCRC = (uint16_t)(wCRC >> 8) ^ wCRC_Table[(uint16_t)(wCRC ^ (uint16_t)chData) & 0x00ff];
    }
    return wCRC;
}


uint16_t get_client_id(uint16_t robot_id) {
    // 红方机器人ID映射
    if (robot_id >= 1 && robot_id <= 11) {
        switch (robot_id) {
            case 1:  return 0x0101; // 红方英雄
            case 2:  return 0x0102; // 红方工程
            case 3:  return 0x0103; // 红方步兵3
            case 4:  return 0x0104; // 红方步兵4
            case 5:  return 0x0105; // 红方步兵5
            case 6:  return 0x0106; // 红方空中
            case 7:  return 0x8080; // 红方哨兵（使用服务器ID）
            case 8:  return 0x8080; // 红方飞镖（使用服务器ID）
            case 9:  return 0x8080; // 红方雷达（使用服务器ID）
            case 10: return 0x8080; // 红方前哨站（使用服务器ID）
            case 11: return 0x8080; // 红方基地（使用服务器ID）
            default: return 0xFFFF; // 无效ID
        }
    }
    // 蓝方机器人ID映射
    else if (robot_id >= 101 && robot_id <= 111) {
        switch (robot_id) {
            case 101: return 0x0165; // 蓝方英雄
            case 102: return 0x0166; // 蓝方工程
            case 103: return 0x0167; // 蓝方步兵3
            case 104: return 0x0168; // 蓝方步兵4
            case 105: return 0x0169; // 蓝方步兵5
            case 106: return 0x016A; // 蓝方空中
            case 107: return 0x8080; // 蓝方哨兵（使用服务器ID）
            case 108: return 0x8080; // 蓝方飞镖（使用服务器ID）
            case 109: return 0x8080; // 蓝方雷达（使用服务器ID）
            case 110: return 0x8080; // 蓝方前哨站（使用服务器ID）
            case 111: return 0x8080; // 蓝方基地（使用服务器ID）
            default: return 0xFFFF; // 无效ID
        }
    }
    // 哨兵/雷达自主决策指令
    else if (robot_id == 0x8080) {
        return 0x8080;
    }
    // 无效ID
    else {
        return 0xFFFF;
    }
}


/// @brief Test if this is a legal frame header.
/// @param frame_p pointer to the beginning of the frame
/// @return 0x0 if invalid header, data_length if valid.
int parse_frame_header(uint8_t* frame_p){
    if(frame_p[0] != 0xa5){
        return 0;
    }
    uint8_t crc8 = Get_CRC8_Check_Sum(frame_p, 4, CRC8_INIT);
    if(crc8 == frame_p[4]){
        return (frame_p[2] << 8) | frame_p[1];
    }else{
        return 0;
    }

}

void parse_referee_msg(int full_frame_size, const uint8_t* frame_begin_ptr){
    uint16_t crc16 = Get_CRC16_Check_Sum(frame_begin_ptr, full_frame_size - 2, CRC_INIT);

    if(((crc16 & 0xff) != frame_begin_ptr[full_frame_size - 2]) 
        || ((crc16 >> 8 & 0xff) != frame_begin_ptr[full_frame_size - 1])){
            return;
    }

    memcpy(&referee_prev, &referee, sizeof(Referee_info_t));
    
    uint16_t cmd_id = (frame_begin_ptr[6] << 8) | frame_begin_ptr[5];
    const void* content_ptr = frame_begin_ptr + 7;
    int content_size = full_frame_size - (5+4);

    void* copy_dest_ptr = NULL;
    int cmd_msg_size = 0;

    switch(cmd_id){
        case 0x201:
            copy_dest_ptr = &(referee.robot_status_0x0201);
            cmd_msg_size = 13;
            break;
        case 0x202:
            copy_dest_ptr = &(referee.power_heat_data_0x0202);
            cmd_msg_size = 16;
            break;
        case 0x207:
            copy_dest_ptr = &(referee.shoot_data_0x0207);
            cmd_msg_size = 7;
            break;
        case 0x208:
            copy_dest_ptr = &(referee.projectile_allowance_0x0208);
            cmd_msg_size = 6;
            break;
    }

    if(cmd_msg_size == content_size && copy_dest_ptr != NULL){
        memcpy(copy_dest_ptr, content_ptr, content_size);
    }

    referee_on_receive();
    
    return;
}

#define REFEREE_BUF_SIZE (512)
uint8_t r_buf[REFEREE_BUF_SIZE];
int referee_ptr;
int data_begin_ptr;
int msg_size;

enum referee_recv_states_list_t{
    RECV_IDLE,
    RECV_FRAME_HEADER,
    RECV_FRAME_DATA
}referee_recv_fsm = RECV_IDLE;

/**
 * Called every time UART receive data from referee system
 */
void referee_recv_byte(const uint8_t data){
    if(referee_ptr >= REFEREE_BUF_SIZE){
        referee_recv_fsm = RECV_IDLE;
        referee_ptr = 0;
    }
    r_buf[referee_ptr] = data;
    referee_ptr++;

    if(referee_recv_fsm == RECV_IDLE){
        if(data == 0xa5){
            referee_recv_fsm = RECV_FRAME_HEADER;
        }else{
            referee_ptr = 0;
        }
    }else if(referee_recv_fsm == RECV_FRAME_HEADER && referee_ptr >= 5){
        int data_size = parse_frame_header(&(r_buf[referee_ptr - 5]));
        if (data_size != 0){
            msg_size = data_size;
            referee_recv_fsm = RECV_FRAME_DATA;
            data_begin_ptr = referee_ptr;
        }
    }else if(referee_recv_fsm == RECV_FRAME_DATA && referee_ptr - (msg_size + 4) == data_begin_ptr){
        parse_referee_msg(msg_size + 4 + 5, r_buf + data_begin_ptr - 5);
        referee_recv_fsm = RECV_IDLE;
        referee_ptr = 0;
    }
    
}

/* Initialize common graphic properties */
void init_graphic(interaction_figure_t* fig, 
                 const char* name,
                 figure_operation_t op,
                 figure_type_t type,
                 uint8_t layer,
                 figure_color_t color,
                 uint16_t width) {
    memcpy(fig->figure_name, name, 3);
    fig->operate_type = op;
    fig->figure_type = type;
    fig->layer = layer;
    fig->color = color;
    fig->width = width;
}

/* Draw a line */
void draw_line(interaction_figure_t* fig,
              const char* name,
              figure_operation_t op,
              uint8_t layer,
              figure_color_t color,
              uint16_t width,
              uint16_t start_x, uint16_t start_y,
              uint16_t end_x, uint16_t end_y) {
    init_graphic(fig, name, op, FIGURE_TYPE_LINE, layer, color, width);
    fig->start_x = start_x;
    fig->start_y = start_y;
    fig->details_d = end_x;
    fig->details_e = end_y;
}

/* Draw a rectangle */
void draw_rectangle(interaction_figure_t* fig,
                   const char* name,
                   figure_operation_t op,
                   uint8_t layer,
                   figure_color_t color,
                   uint16_t width,
                   uint16_t start_x, uint16_t start_y,
                   uint16_t opposite_x, uint16_t opposite_y) {
    init_graphic(fig, name, op, FIGURE_TYPE_RECTANGLE, layer, color, width);
    fig->start_x = start_x;
    fig->start_y = start_y;
    fig->details_d = opposite_x;
    fig->details_e = opposite_y;
}

/* Draw a circle */
void draw_circle(interaction_figure_t* fig,
                const char* name,
                figure_operation_t op,
                uint8_t layer,
                figure_color_t color,
                uint16_t width,
                uint16_t center_x, uint16_t center_y,
                uint16_t radius) {
    init_graphic(fig, name, op, FIGURE_TYPE_CIRCLE, layer, color, width);
    fig->start_x = center_x;
    fig->start_y = center_y;
    fig->details_c = radius;
}

/* Draw an ellipse */
void draw_ellipse(interaction_figure_t* fig,
                 const char* name,
                 figure_operation_t op,
                 uint8_t layer,
                 figure_color_t color,
                 uint16_t width,
                 uint16_t center_x, uint16_t center_y,
                 uint16_t semi_axis_x, uint16_t semi_axis_y) {
    init_graphic(fig, name, op, FIGURE_TYPE_ELLIPSE, layer, color, width);
    fig->start_x = center_x;
    fig->start_y = center_y;
    fig->details_d = semi_axis_x;
    fig->details_e = semi_axis_y;
}

/* Draw an arc */
void draw_arc(interaction_figure_t* fig,
              const char* name,
              figure_operation_t op,
              uint8_t layer,
              figure_color_t color,
              uint16_t width,
              uint16_t center_x, uint16_t center_y,
              uint16_t start_angle, uint16_t end_angle,
              uint16_t semi_axis_x, uint16_t semi_axis_y) {
    init_graphic(fig, name, op, FIGURE_TYPE_ARC, layer, color, width);
    fig->start_x = center_x;
    fig->start_y = center_y;
    fig->details_a = start_angle;
    fig->details_b = end_angle;
    fig->details_d = semi_axis_x;
    fig->details_e = semi_axis_y;
}

/* Draw a floating point number */
void draw_float(interaction_figure_t* fig,
               const char* name,
               figure_operation_t op,
               uint8_t layer,
               figure_color_t color,
               uint16_t width,
               uint16_t pos_x, uint16_t pos_y,
               uint16_t font_size,
               float value) {
    init_graphic(fig, name, op, FIGURE_TYPE_FLOAT, layer, color, width);
    fig->start_x = pos_x;
    fig->start_y = pos_y;
    fig->details_a = font_size;
    
    // Combine all 32 bits for the float value (details_c + details_d + details_e)
    uint32_t combined_value = (uint32_t)(value * 1000);
    fig->details_c = (combined_value >> 22) & 0x3FF;  // Upper 10 bits
    fig->details_d = (combined_value >> 11) & 0x7FF;  // Middle 11 bits
    fig->details_e = combined_value & 0x7FF;          // Lower 11 bits
}

/* Draw an integer */
void draw_integer(interaction_figure_t* fig,
                 const char* name,
                 figure_operation_t op,
                 uint8_t layer,
                 figure_color_t color,
                 uint16_t width,
                 uint16_t pos_x, uint16_t pos_y,
                 uint16_t font_size,
                 int32_t value) {
    init_graphic(fig, name, op, FIGURE_TYPE_INTEGER, layer, color, width);
    fig->start_x = pos_x;
    fig->start_y = pos_y;
    fig->details_a = font_size;
    
    // Combine all 32 bits for the integer value (details_c + details_d + details_e)
    uint32_t combined_value = (uint32_t)value;
    fig->details_c =  combined_value        & 0x3FF;          // bits [0..9]
    fig->details_d = (combined_value >> 10) & 0x7FF;          // bits [10..20]
    fig->details_e = (combined_value >> 21) & 0x7FF;          // bits [21..31]
}

/* Draw a character */
void draw_char(interaction_figure_t* fig,
              const char* name,
              figure_operation_t op,
              uint8_t layer,
              figure_color_t color,
              uint16_t width,
              uint16_t pos_x, uint16_t pos_y,
              uint16_t font_size,
              uint8_t char_length) {
    init_graphic(fig, name, op, FIGURE_TYPE_CHAR, layer, color, width);
    fig->start_x = pos_x;
    fig->start_y = pos_y;
    fig->details_a = font_size;
    fig->details_b = char_length;
}

uint8_t r_send_buf[256];
uint8_t r_send_seq = 0;

uint8_t* referee_send_data(uint16_t data_size, robot_interaction_data_t* data){
    memset(r_send_buf, 0, sizeof(r_send_buf));

    r_send_buf[0] = 0xa5; // SOF
    r_send_buf[1] = data_size & 0xff;
    r_send_buf[2] = (data_size >> 8) & 0xff;
    r_send_buf[3] = ++ r_send_seq;
    r_send_buf[4] = Get_CRC8_Check_Sum(r_send_buf, 4, CRC8_INIT);

    uint16_t cmd_id = 0x301;
    r_send_buf[5] = cmd_id & 0xff;
    r_send_buf[6] = (cmd_id >> 8) & 0xff;

    memcpy(r_send_buf + 7, data, data_size);

    uint16_t crc16 = Get_CRC16_Check_Sum(r_send_buf, data_size + 7, CRC_INIT);
    r_send_buf[data_size + 7] = (crc16 & 0xff);
    r_send_buf[data_size + 8] = (crc16 >> 8 & 0xff);

    return r_send_buf;
}

