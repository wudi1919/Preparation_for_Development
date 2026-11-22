//
// Created by Ken on 2026/11/22.
//

#include "bsp_delay.h"

uint8_t Delay_NonBlock(uint32_t *last_tick, uint32_t ms)
{
    uint32_t now = HAL_GetTick();

    if ((now - *last_tick) >= ms)
    {
        *last_tick = now;
        return 1;   // 到时间了
    }
    return 0;       // 还没到
}
