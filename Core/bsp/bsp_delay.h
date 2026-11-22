//
// Created by Ken on 2026/11/22.
//

#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

#include "main.h"

/* 阻塞延时（忙等），单位毫秒。循环在函数内部实现 */
void Delay_ms(uint32_t ms);

#endif /* __BSP_DELAY_H__ */
