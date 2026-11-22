//
// Created by Ken on 2026/11/22.
//

#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

#include "main.h"

/* 非阻塞延时：每 ms 毫秒触发一次（返回 1），其余返回 0。
 * last_tick 由调用者保存（static uint32_t），不占 CPU */
uint8_t Delay_NonBlock(uint32_t *last_tick, uint32_t ms);

#endif /* __BSP_DELAY_H__ */
