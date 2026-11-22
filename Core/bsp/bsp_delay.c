//
// Created by Ken on 2026/11/22.
//

#include "bsp_delay.h"

void Delay_ms(uint32_t ms)
{
    int a;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < ms)
    {
        a++;// 忙等，直到 ms 毫秒过去
    }
}
