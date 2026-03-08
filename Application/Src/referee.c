#include <referee.h>
#include <stddef.h>

#include<global_variables.h>
#include<utils.h>

#include<application.h>

__weak void referee_on_receive(){
    return;
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
    uint16_t crc16 = Get_CRC16_Check_Sum(frame_begin_ptr, full_frame_size - 2, CRC16_INIT);

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

    uint16_t crc16 = Get_CRC16_Check_Sum(r_send_buf, data_size + 7, CRC16_INIT);
    r_send_buf[data_size + 7] = (crc16 & 0xff);
    r_send_buf[data_size + 8] = (crc16 >> 8 & 0xff);

    return r_send_buf;
}

