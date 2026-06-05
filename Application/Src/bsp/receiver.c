#include <receiver.h>
#include <stdint.h>

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

    dr16->key.v_edge_event = dr16->key.v ^ key_v;

    dr16->key.v = key_v;
}
