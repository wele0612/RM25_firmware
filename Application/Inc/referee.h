#ifndef __REFEREE_H
#define __REFEREE_H

#include <stdint.h>
#include <string.h>
#include <main.h>

#include <receiver.h>

// Referee system communication. Protocal Version V1.7.

// CMD 0x0201
typedef struct __attribute__((packed)) {
    uint8_t robot_id;                              // 本机器人ID
    uint8_t robot_level;                           // 机器人等级
    uint16_t current_HP;                           // 机器人当前血量
    uint16_t maximum_HP;                           // 机器人血量上限
    uint16_t shooter_barrel_cooling_value;         // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;            // 机器人射击热量上限
    uint16_t chassis_power_limit;                  // 机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1;    // bit0: gimbal口输出(0-无输出,1-24V输出)
    uint8_t power_management_chassis_output : 1;    // bit1: chassis口输出(0-无输出,1-24V输出)
    uint8_t power_management_shooter_output : 1;    // bit2: shooter口输出(0-无输出,1-24V输出)
} robot_status_t;

// CMD 0x0202
typedef struct __attribute__((packed)) {
    uint16_t reserved1;                  // 保留位
    uint16_t reserved2;                  // 保留位
    float reserved3;                     // 保留位
    uint16_t buffer_energy;              // 缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat; // 第1个17mm发射机构的射击热量
    uint16_t shooter_17mm_2_barrel_heat; // 第2个17mm发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;   // 42mm发射机构的射击热量
} power_heat_data_t;

// CMD 0x0207
typedef struct __attribute__((packed)) {
    uint8_t bullet_type;         // 弹丸类型：1-17mm弹丸，2-42mm弹丸
    uint8_t shooter_number;      // 发射机构ID：1-第1个17mm，2-第2个17mm，3-42mm
    uint8_t launching_frequency; // 弹丸射速（单位：Hz）
    float initial_speed;         // 弹丸初速度（单位：m/s）
} shoot_data_t;

// CMD 0x0208
typedef struct __attribute__((packed)) {
    uint16_t projectile_allowance_17mm; // 17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm; // 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;       // 剩余金币数量
} projectile_allowance_t;

// CMD 0x0301 -----------------------
// typedef struct __attribute__((packed)) {
//     uint16_t data_cmd_id;    // 子内容ID (需为开放的子内容ID)
//     uint16_t sender_id;      // 发送者ID (需与自身ID匹配)
//     uint16_t receiver_id;    // 接收者ID (仅限己方通信，需为规则允许的接收者)
//     uint8_t user_data[112];     // 内容数据段 (最大为112字节)
// } robot_interaction_data_t;

/* UI Drawing Commands */
#define CMD_DELETE_GRAPHIC_LAYER    0x0100  // 选手端删除图层 (2 bytes)
#define CMD_DRAW_SINGLE_GRAPHIC     0x0101  // 选手端绘制一个图形 (15 bytes)
#define CMD_DRAW_DOUBLE_GRAPHICS    0x0102  // 选手端绘制两个图形 (30 bytes)
#define CMD_DRAW_FIVE_GRAPHICS      0x0103  // 选手端绘制五个图形 (75 bytes)
#define CMD_DRAW_SEVEN_GRAPHICS     0x0104  // 选手端绘制七个图形 (105 bytes)
#define CMD_DRAW_CHARACTER_GRAPHIC  0x0110  // 选手端绘制字符图形 (45 bytes)

/* Autonomous Decision Commands */
#define CMD_SENTRY_DECISION         0x0120  // 哨兵自主决策指令 (4 bytes)
#define CMD_RADAR_DECISION          0x0121  // 雷达自主决策指令 (1 byte)

/* Graphic Operation Types */
typedef enum {
    FIGURE_OPERATION_NONE = 0,   // 空操作
    FIGURE_OPERATION_ADD,        // 增加
    FIGURE_OPERATION_MODIFY,     // 修改
    FIGURE_OPERATION_DELETE      // 删除
} figure_operation_t;

/* Graphic Types */
typedef enum {
    FIGURE_TYPE_LINE = 0,        // 直线
    FIGURE_TYPE_RECTANGLE,       // 矩形
    FIGURE_TYPE_CIRCLE,          // 正圆
    FIGURE_TYPE_ELLIPSE,         // 椭圆
    FIGURE_TYPE_ARC,             // 圆弧
    FIGURE_TYPE_FLOAT,           // 浮点数
    FIGURE_TYPE_INTEGER,         // 整型数
    FIGURE_TYPE_CHAR             // 字符
} figure_type_t;

/* Color Types */
typedef enum {
    COLOR_TEAM = 0,             // 红/蓝（己方颜色）
    COLOR_YELLOW,               // 黄色
    COLOR_GREEN,                // 绿色
    COLOR_ORANGE,               // 橙色
    COLOR_PURPLE,               // 紫红色
    COLOR_PINK,                 // 粉色
    COLOR_CYAN,                 // 青色
    COLOR_BLACK,                // 黑色
    COLOR_WHITE                 // 白色
} figure_color_t;

// Sub ID 0x101
typedef struct __attribute__((packed)) {
    uint8_t figure_name[3];      // 图形名 (在图形删除、修改等操作中，作为索引)
    
    /* Graphic Configuration 1 */
    uint32_t operate_type : 3;   // 图形操作 (0-空操作,1-增加,2-修改,3-删除)
    uint32_t figure_type : 3;    // 图形类型 (0-直线,1-矩形,2-正圆,3-椭圆,4-圆弧,5-浮点数,6-整型数,7-字符)
    uint32_t layer : 4;          // 图层数 (0~9)
    uint32_t color : 4;          // 颜色 (0-己方颜色,1-黄色,...,8-白色)
    uint32_t details_a : 9;      // 根据图形类型不同而不同
    uint32_t details_b : 9;      // 根据图形类型不同而不同
    
    /* Graphic Configuration 2 */
    uint32_t width : 10;         // 线宽 (建议字体大小与线宽比例为10:1)
    uint32_t start_x : 11;       // 起点/圆心x坐标
    uint32_t start_y : 11;       // 起点/圆心y坐标
    
    /* Graphic Configuration 3 */
    uint32_t details_c : 10;     // 根据图形类型不同而不同
    uint32_t details_d : 11;     // 根据图形类型不同而不同
    uint32_t details_e : 11;     // 根据图形类型不同而不同
} interaction_figure_t;

// Sub ID 0x110 - Character graphic
typedef struct __attribute__((packed)) {
    interaction_figure_t graphic_data;
    uint8_t char_data[30];
} ext_client_custom_character_t;

// Main interaction data structure with union
typedef struct __attribute__((packed)) {
    uint16_t data_cmd_id;    // 子内容ID (需为开放的子内容ID)
    uint16_t sender_id;      // 发送者ID (需与自身ID匹配)
    uint16_t receiver_id;    // 接收者ID (仅限己方通信，需为规则允许的接收者)
    
    union {
        uint8_t raw_data[112];  // Raw data access

        // Graphics commands
        struct {
            uint8_t operation;  // For 0x0100 (delete layer)
        } delete_layer;
        
        interaction_figure_t single_graphic[1];    // For 0x0101
        interaction_figure_t double_graphics[2]; // For 0x0102
        interaction_figure_t five_graphics[5];   // For 0x0103
        interaction_figure_t seven_graphics[7];  // For 0x0104
        ext_client_custom_character_t char_graphic; // For 0x0110
        
    } user_data;
} robot_interaction_data_t;
// -----------------------


/* Line Drawing */
void draw_line(interaction_figure_t* fig,
              const char* name,
              figure_operation_t op,
              uint8_t layer,
              figure_color_t color,
              uint16_t width,
              uint16_t start_x, uint16_t start_y,
              uint16_t end_x, uint16_t end_y);

/* Rectangle Drawing */
void draw_rectangle(interaction_figure_t* fig,
                   const char* name,
                   figure_operation_t op,
                   uint8_t layer,
                   figure_color_t color,
                   uint16_t width,
                   uint16_t start_x, uint16_t start_y,
                   uint16_t opposite_x, uint16_t opposite_y);

/* Circle Drawing */
void draw_circle(interaction_figure_t* fig,
                const char* name,
                figure_operation_t op,
                uint8_t layer,
                figure_color_t color,
                uint16_t width,
                uint16_t center_x, uint16_t center_y,
                uint16_t radius);

/* Ellipse Drawing */
void draw_ellipse(interaction_figure_t* fig,
                 const char* name,
                 figure_operation_t op,
                 uint8_t layer,
                 figure_color_t color,
                 uint16_t width,
                 uint16_t center_x, uint16_t center_y,
                 uint16_t semi_axis_x, uint16_t semi_axis_y);

/* Arc Drawing */
void draw_arc(interaction_figure_t* fig,
              const char* name,
              figure_operation_t op,
              uint8_t layer,
              figure_color_t color,
              uint16_t width,
              uint16_t center_x, uint16_t center_y,
              uint16_t start_angle, uint16_t end_angle,
              uint16_t semi_axis_x, uint16_t semi_axis_y);

/* Floating Point Number Drawing */
void draw_float(interaction_figure_t* fig,
               const char* name,
               figure_operation_t op,
               uint8_t layer,
               figure_color_t color,
               uint16_t width,
               uint16_t pos_x, uint16_t pos_y,
               uint16_t font_size,
               float value);

/* Integer Number Drawing */
void draw_integer(interaction_figure_t* fig,
                 const char* name,
                 figure_operation_t op,
                 uint8_t layer,
                 figure_color_t color,
                 uint16_t width,
                 uint16_t pos_x, uint16_t pos_y,
                 uint16_t font_size,
                 int32_t value);

/* Character Drawing */
void draw_char(interaction_figure_t* fig,
              const char* name,
              figure_operation_t op,
              uint8_t layer,
              figure_color_t color,
              uint16_t width,
              uint16_t pos_x, uint16_t pos_y,
              uint16_t font_size,
              uint8_t char_length);

// ================ Robot Data Buffer ================

typedef struct referee_info_t{
    robot_status_t robot_status_0x0201;
    power_heat_data_t power_heat_data_0x0202;
    shoot_data_t shoot_data_0x0207;
    projectile_allowance_t projectile_allowance_0x0208;
}Referee_info_t;

uint16_t get_client_id(uint16_t robot_id);

void referee_recv_byte(const uint8_t data);

uint8_t *referee_send_data(uint16_t data_size, robot_interaction_data_t *data);

typedef void (*referee_transmit_fn_t)(const uint8_t *send_buf, uint16_t size);

void referee_init(referee_transmit_fn_t transmit_fn);
void referee_reset_ui();
void referee_ui_next_frame();

void referee_on_receive();


#endif

