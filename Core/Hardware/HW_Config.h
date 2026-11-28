//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_HW_CONFIG_H
#define H7_PREPARED_DEVELOPMENT_HW_CONFIG_H
#include "stm32h7xx_hal.h"

/* 【配置函数】电机初始化 + PID 参数配置，定义在 HW_Config.c，主函数只调一次 */

/* 【配置函数】电机初始化 + PID 参数配置，定义在 HW_Config.c，主函数只调一次 */
void Mo_Init_DJI(void);

/* 【配置函数】达妙电机初始化（协议/量程 + PID + 使能），定义在 HW_Config.c */
void Mo_Init_DM(void);

/* 【配置函数】电机初始化统一入口（DJI + 达妙 + 通道使能），主函数只调一次 */
void Mo_Init(void);
#endif //H7_PREPARED_DEVELOPMENT_HW_CONFIG_H
