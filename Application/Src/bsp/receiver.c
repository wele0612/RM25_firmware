#include <receiver.h>
#include <stdint.h>



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
    dr16->mouse.press_l = buff[12];
    dr16->mouse.press_r = buff[13];
    dr16->key.v = ((int16_t)buff[14]);// | ((int16_t)buff[15] << 8);

}

void set_DR16_previous_state(receiver_DBUS_t *dr16){
    memcpy(&(dr16->previous), dr16, sizeof(dr16->previous));
}

void parse_aiming_receiver_msg(Aiming_message_t *aim) {
    const uint8_t* buff = aim->msg;

    const float* asfloat = (const float *)buff;

    aim->target_yaw_rad = asfloat[0];
    aim->target_pitch_rad = asfloat[1];
    aim->target_distance_m = asfloat[2];

    return;
}
