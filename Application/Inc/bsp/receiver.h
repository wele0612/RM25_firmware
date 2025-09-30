#ifndef __RECEIVER_H
#define __RECEIVER_H

#include <stdint.h>
#include <string.h>

enum receiver_type{
    RECEIVER_TYPE_DBUS,
    RECEIVER_TYPE_IBUS
};

/*
Bit0 -- W 键
Bit1 -- S 键
Bit2 -- A 键
Bit3 -- D 键
Bit4 -- Q 键
Bit5 -- E 键
Bit6 -- Shift 键
Bit7 -- Ctrl 键
*/
#define DR16_KEY_W_BIT  (1U << 0)
#define DR16_KEY_S_BIT  (1U << 1)
#define DR16_KEY_A_BIT  (1U << 2)
#define DR16_KEY_D_BIT  (1U << 3)
#define DR16_KEY_SHIFT_BIT  (1U << 4)
#define DR16_KEY_CTRL_BIT   (1U << 5)
#define DR16_KEY_Q_BIT  (1U << 6)
#define DR16_KEY_E_BIT  (1U << 7)

typedef struct receiver_DBUS_t{
    float channel[4];
    int8_t s1;
    int8_t s2;
    float thumbwheel;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;
    uint8_t msg[16]; // To prevent out-of-bound. Only 18 bytes used.
    struct{
            float channel[4];
        int8_t s1;
        int8_t s2;
        float thumbwheel;
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t press_l;
            uint8_t press_r;
        } mouse;
        struct
        {
            uint16_t v;
        } key;
    }previous;
}receiver_DBUS_t;

typedef struct Aiming_message_t{
    float target_yaw_rad;
    float target_pitch_rad;
    float target_distance_m;
    uint32_t state_code;

    uint8_t msg[64];
}Aiming_message_t;

void parse_DR16_receiver_msg(receiver_DBUS_t* dr16);
void set_DR16_previous_state(receiver_DBUS_t* dr16);

void parse_aiming_receiver_msg(Aiming_message_t* aim);

#endif
