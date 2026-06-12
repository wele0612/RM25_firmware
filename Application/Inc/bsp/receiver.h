#ifndef __RECEIVER_H
#define __RECEIVER_H

#include <stdint.h>
#include <string.h>

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

#define DR16_KEY_R_BIT (1U << 8)
#define DR16_KEY_F_BIT (1U << 9)
#define DR16_KEY_G_BIT (1U << 10)
#define DR16_KEY_Z_BIT (1U << 11)
#define DR16_KEY_X_BIT (1U << 12)
#define DR16_KEY_C_BIT (1U << 13)
#define DR16_KEY_V_BIT (1U << 14)
#define DR16_KEY_B_BIT (1U << 15)

#define DR16_SWITCH_UP (0x1)
#define DR16_SWITCH_MID (0x3)
#define DR16_SWITCH_DOWN (0x2)

#define VTM_SW_CINE     (0x0)
#define VTM_SW_NORMAL   (0x1)
#define VTM_SW_SPORT    (0x2)

#define DR16_EDGE_NONE (0U)
#define DR16_EDGE_RISE (1U)
#define DR16_EDGE_FALL (2U)

typedef struct receiver_DBUS_t{
    float channel[4];
    int8_t s1; // {Up, Mid, Down}={1,3,2}
    int8_t s2; // {Up, Mid, Down}={1,3,2}
    float thumbwheel;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;

        uint8_t press_l_edge_event;
        uint8_t press_r_edge_event;
    } mouse;
    struct
    {
        uint16_t v;
        uint16_t v_edge_event;
    } key;

}receiver_DBUS_t;

int DR16_acquire_key_edge(receiver_DBUS_t *dr16, uint16_t KEY);

void parse_DR16_receiver_msg(receiver_DBUS_t *dr16, uint8_t *msg);


typedef struct receiver_VTM_t{
    float channel[4];   
    float wheel;
    int8_t mode_sw;

    struct{
        int8_t pause;
        int8_t s1;
        int8_t s2;
        int8_t trigger;

        int8_t pause_event;
        int8_t s1_event;
        int8_t s2_event;
        int8_t trigger_event;
    } buttons;

    struct{
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
        uint8_t press_mid;

        uint8_t press_l_edge_event;
        uint8_t press_r_edge_event;
        uint8_t press_mid_edge_event;
    } mouse;

    struct
    {
        uint16_t v;
        uint16_t v_edge_event;
    } key;
}receiver_VTM_t;

void VTM_recv_byte(receiver_VTM_t* vtm, uint8_t data);
void VTM_on_change();

#endif
