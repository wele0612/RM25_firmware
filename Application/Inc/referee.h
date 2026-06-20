#ifndef __REFEREE_H
#define __REFEREE_H

#include <stdint.h>
#include <string.h>
#include <main.h>

#include <receiver.h>

// Referee system communication. Protocal Version V1.1.0 (20251217).

// CMD 0x0001
typedef struct __attribute__((packed)) {
    uint8_t game_type : 4;           // bit 0-3: 比赛类型（1-超级对抗赛, 2-高校单项赛, 3-AI挑战赛, 4-联盟赛3V3, 5-联盟赛步兵对抗）
    uint8_t game_progress : 4;       // bit 4-7: 当前比赛阶段（0-未开始, 1-准备阶段, 2-十五秒自检, 3-五秒倒计时, 4-比赛中, 5-比赛结算中）
    uint16_t stage_remain_time;      // 当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp;          // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
} game_status_t;

// CMD 0x0101
typedef struct __attribute__((packed)) {
    uint32_t supply_zone_non_overlap : 1;  // bit 0: 己方与资源区不重叠的补给区占领状态，1为已占领
    uint32_t supply_zone_overlap : 1;      // bit 1: 己方与资源区重叠的补给区占领状态，1为已占领
    uint32_t supply_zone_rmul : 1;         // bit 2: 己方补给区的占领状态，1为已占领（仅RMUL适用）
    uint32_t small_power_status : 2;       // bit 3-4: 己方小能量机关激活状态，0-未激活，1-已激活，2-正在激活
    uint32_t large_power_status : 2;       // bit 5-6: 己方大能量机关激活状态，0-未激活，1-已激活，2-正在激活
    uint32_t central_highland : 2;         // bit 7-8: 己方中央高地占领状态，1-被己方占领，2-被对方占领
    uint32_t trapezoidal_highland : 2;     // bit 9-10: 己方梯形高地占领状态，1-已占领
    uint32_t dart_hit_time : 9;            // bit 11-19: 对方飞镖最后一次击中己方前哨站或基地的时间(0-420，开局默认为0)
    uint32_t dart_hit_target : 3;          // bit 20-22: 对方飞镖最后一次击中目标（0-默认, 1-前哨站, 2-基地固定目标, 3-基地随机固定目标, 4-基地随机移动目标, 5-基地末端移动目标）
    uint32_t center_gain_point : 2;        // bit 23-24: 中心增益点占领状态，0-未被占领，1-被己方占领，2-被对方占领，3-被双方占领（仅RMUL适用）
    uint32_t fortress_gain_point : 2;      // bit 25-26: 己方堡垒增益点占领状态，0-未被占领，1-被己方占领，2-被对方占领，3-被双方占领
    uint32_t outpost_gain_point : 2;       // bit 27-28: 己方前哨站增益点占领状态，0-未被占领，1-被己方占领，2-被对方占领
    uint32_t base_gain_point : 1;          // bit 29: 己方基地增益点占领状态，1-已占领
    uint32_t reserved : 2;                 // bit 30-31: 保留位
} event_data_t;

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
    uint16_t shooter_17mm_barrel_heat;   // 17mm发射机构的射击热量
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
    uint16_t projectile_allowance_17mm;    // 17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm;    // 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;          // 剩余金币数量
    uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm弹丸允许发弹量；该值与机器人是否实际占领堡垒无关
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
            uint8_t delete_type;   // 删除操作: 0-空操作, 1-删除图层, 2-删除所有
            uint8_t layer;         // 图层数: 0~9
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
    game_status_t game_status_0x0001;
    event_data_t event_data_0x0101;
    robot_status_t robot_status_0x0201;
    power_heat_data_t power_heat_data_0x0202;
    shoot_data_t shoot_data_0x0207;
    projectile_allowance_t projectile_allowance_0x0208;
}Referee_info_t;

uint16_t get_client_id(uint16_t robot_id);

void referee_recv_byte(const uint8_t data);

uint8_t *referee_send_data(uint16_t data_size, robot_interaction_data_t *data);

void referee_reset_ui();
void referee_ui_next_frame();

void referee_on_receive();


#endif

