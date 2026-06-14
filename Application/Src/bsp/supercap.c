#include<main.h>
#include<supercap.h>

static uint32_t supercap_last_recv_tick=0;
void supercap_update_timeout(){
    supercap_last_recv_tick = HAL_GetTick();
}

const int supercap_timeout_threshold = 50;
int supercap_online(){
    return (HAL_GetTick() - supercap_last_recv_tick) < supercap_timeout_threshold;
}