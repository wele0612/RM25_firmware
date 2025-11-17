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
#include <w25q.h>

#include <utils.h>

#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7f}

#define ROBOT_CONFIG_ROM_ADDR (0x0)

#define robot_saveconfig(config) do{\
    OSPI_W25Qxx_BlockErase_32K(ROBOT_CONFIG_ROM_ADDR);\
    OSPI_W25Qxx_WriteBuffer((uint8_t*)(config), ROBOT_CONFIG_ROM_ADDR,sizeof(Robot_config_t));\
}while(0)

#define robot_readconfig(config) do{\
    OSPI_W25Qxx_ReadBuffer((uint8_t*)(config),ROBOT_CONFIG_ROM_ADDR,sizeof(Robot_config_t));\
}while(0)


void robot_init();
void referee_ui_update(int updata_level);
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

#endif
