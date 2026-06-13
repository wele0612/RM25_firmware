#include <receiver.h>
#include <stdint.h>

#include <utils.h>
#include <main.h>

// ------------------ General -----------------
static int control_last_recv_tick=0;
void control_timeout_update(){
    control_last_recv_tick = HAL_GetTick();
}

const int CONTROL_TIMEOUT = 100;
int control_online(){
    return (HAL_GetTick() - control_last_recv_tick) < CONTROL_TIMEOUT;
}

// ------------------- DR16 -------------------
static int dr16_last_recv_tick=0;

__attribute__((weak)) void DR16_on_change(){
}

int DR16_acquire_key_edge(receiver_DBUS_t* dr16, uint16_t KEY){
    int has_event = DR16_EDGE_NONE;
    if(dr16->key.v_edge_event & KEY){
        if(dr16->key.v & KEY){
            has_event = DR16_EDGE_RISE;
        }else{
            has_event = DR16_EDGE_FALL;
        }
        dr16->key.v_edge_event &= ~KEY;
    }

    return has_event;
}

void parse_DR16_receiver_msg(receiver_DBUS_t* dr16, uint8_t* msg){
    const uint8_t* buff = msg;
    uint16_t ch0 = (buff[0] | buff[1] << 8) & 0x07FF;
    dr16->channel[0] = ((float)ch0-1024.0f)*(1/660.0f);

    uint16_t ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    dr16->channel[1] = ((float)ch1-1024.0f)*(1/660.0f);

    uint16_t ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    dr16->channel[2] = ((float)ch2-1024.0f)*(1/660.0f);

    uint16_t ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    dr16->channel[3] = ((float)ch3-1024.0f)*(1/660.0f);

    const float invalid_range = 0.01f;
    for(int i=0;i<4;i++){
        if(dr16->channel[i] <= invalid_range && dr16->channel[i] >= -invalid_range)
            dr16->channel[i] = 0.0f;
    }

    dr16->s1 = ((buff[5] >> 4) & 0x000C) >> 2;
    dr16->s2 = (buff[5] >> 4) & 0x0003;

    dr16->mouse.x = ((int16_t)buff[6]) | ((int16_t)buff[7] << 8);
    dr16->mouse.y = ((int16_t)buff[8]) | ((int16_t)buff[9] << 8);
    dr16->mouse.z = ((int16_t)buff[10]) | ((int16_t)buff[11] << 8);

    uint8_t press_l = buff[12];
    uint8_t press_r = buff[13];
    uint16_t key_v = ((uint16_t)buff[14]) | ((uint16_t)buff[15] << 8);

    dr16->mouse.press_l_edge_event = (press_l != dr16->mouse.press_l);
    dr16->mouse.press_r_edge_event = (press_r != dr16->mouse.press_r);

    dr16->mouse.press_l = press_l;
    dr16->mouse.press_r = press_r;

    dr16->key.v_edge_event |= dr16->key.v ^ key_v;

    dr16->key.v = key_v;

    dr16_last_recv_tick = HAL_GetTick();
    DR16_on_change();
}

const int DR16_TIMEOUT = 100;
int DR16_online(){
    return (HAL_GetTick() - dr16_last_recv_tick) < DR16_TIMEOUT;
}

// ------------------- VTM -------------------

static int vtm_last_recv_tick=0;

typedef struct __attribute__((packed)){
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}VTM_data_t;

static enum vtm_recv_states_list_t{
    RECV_IDLE,
    RECV_FRAME_DATA
}vtm_recv_fsm = RECV_IDLE;

#define VTM_PACKET_SIZE (sizeof(VTM_data_t))

static int vtm_ptr=0;
static uint8_t vtm_buf[VTM_PACKET_SIZE];

__attribute__((weak)) void VTM_on_change(){
}

static void VTM_parse_data(receiver_VTM_t* vtm, VTM_data_t* data){
    vtm->channel[0] = ((float)data->ch_0 - 1024.0f) * (1.0f / 660.0f);
    vtm->channel[1] = ((float)data->ch_1 - 1024.0f) * (1.0f / 660.0f);
    vtm->channel[2] = ((float)data->ch_2 - 1024.0f) * (1.0f / 660.0f);
    vtm->channel[3] = ((float)data->ch_3 - 1024.0f) * (1.0f / 660.0f);

    vtm->wheel = ((float)data->wheel - 1024.0f) * (1.0f / 660.0f);

    int8_t new_mode_sw = (int8_t)data->mode_sw;
    int8_t new_pause   = (int8_t)data->pause;
    int8_t new_s1      = (int8_t)data->fn_1;
    int8_t new_s2      = (int8_t)data->fn_2;
    int8_t new_trigger = (int8_t)data->trigger;

    vtm->buttons.pause_event   = (new_pause   != vtm->buttons.pause);
    vtm->buttons.s1_event      = (new_s1      != vtm->buttons.s1);
    vtm->buttons.s2_event      = (new_s2      != vtm->buttons.s2);
    vtm->buttons.trigger_event = (new_trigger != vtm->buttons.trigger);

    vtm->mode_sw         = new_mode_sw;
    vtm->buttons.pause   = new_pause;
    vtm->buttons.s1      = new_s1;
    vtm->buttons.s2      = new_s2;
    vtm->buttons.trigger = new_trigger;

    uint8_t press_l   = (uint8_t)data->mouse_left;
    uint8_t press_r   = (uint8_t)data->mouse_right;
    uint8_t press_mid = (uint8_t)data->mouse_middle;
    uint16_t key_v    = data->key;

    vtm->mouse.press_l_edge_event   = (press_l   != vtm->mouse.press_l);
    vtm->mouse.press_r_edge_event   = (press_r   != vtm->mouse.press_r);
    vtm->mouse.press_mid_edge_event = (press_mid != vtm->mouse.press_mid);

    vtm->mouse.x         = data->mouse_x;
    vtm->mouse.y         = data->mouse_y;
    vtm->mouse.z         = data->mouse_z;
    vtm->mouse.press_l   = press_l;
    vtm->mouse.press_r   = press_r;
    vtm->mouse.press_mid = press_mid;

    vtm->key.v_edge_event |= vtm->key.v ^ key_v;
    vtm->key.v = key_v;

    vtm_last_recv_tick = HAL_GetTick();
    VTM_on_change();
}

void VTM_recv_byte(receiver_VTM_t* vtm, uint8_t data){
    if(vtm_ptr >= VTM_PACKET_SIZE){
        vtm_recv_fsm = RECV_IDLE;
        vtm_ptr = 0;
    }
    vtm_buf[vtm_ptr] = data;
    vtm_ptr++;

    if(vtm_recv_fsm == RECV_IDLE){
        if(data == 0xA9){
            vtm_recv_fsm = RECV_FRAME_DATA;
        }else{
            vtm_ptr = 0;
        }
    }else if(vtm_ptr == VTM_PACKET_SIZE){
        uint16_t crc16 = Get_CRC16_Check_Sum(vtm_buf, VTM_PACKET_SIZE - 2, CRC16_INIT);
        uint16_t packet_crc16 = ((VTM_data_t *)vtm_buf)->crc16;
        if(packet_crc16 == crc16){
            VTM_parse_data(vtm, (VTM_data_t *)vtm_buf);
        }
        vtm_recv_fsm = RECV_IDLE;
        vtm_ptr = 0;
    }
}

const int VTM_TIMEOUT = 100;
int VTM_online(){
    return (HAL_GetTick() - vtm_last_recv_tick) < VTM_TIMEOUT;
}
