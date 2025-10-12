#ifndef __APPLICATION_H
#define __APPLICATION_H

#include <stdint.h>

#include <main.h>
#include <gpio.h>
#include <spi.h>
#include <tim.h>
#include <usart.h>
#include <octospi.h>

#include <icm42688.h>

#include <utils.h>

#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7f}

typedef struct robot_VOFA_report_t{
    float val[10];
    unsigned char tail[4];
}robot_VOFA_report_t;

#define REMOTE_TIMEOUT (50U)
typedef struct timeout_monitor_t{
    uint64_t last_remote_tick;
}timeout_monitor_t;


void robot_init();
void robot_step(const float CTRL_DELTA_T); // bind to interruption with fixed frequency.
void robot_loop();

void referee_uart_transmit_once(const uint8_t *send_buf, uint16_t size);

// -----------------------------------------------------
// | Implementation of functions below depends on the  |
// |   platform.                                       |
// -----------------------------------------------------
void controller_init();
void controller_cycle(const float CTRL_DELTA_T);

// -----------------------------------------------------
// | Implementation of functions below depends on the  |
// |   robot.                                          |
// -----------------------------------------------------
void role_controller_init();
void role_controller_step(const float CTRL_DELTA_T);

void robot_CAN_msgcallback(int ID, uint8_t *msg);
void dr16_on_change();


// -----------------------------------------------------
// | State machines.                                   |
// |                                                   |
// -----------------------------------------------------

typedef enum IMU_state{
    IMU_RESET,
    IMU_RUNNING,
    IMU_CALIBRATE
}IMU_state_t;

#endif
