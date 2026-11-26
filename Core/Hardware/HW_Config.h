//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_HW_CONFIG_H
#define H7_PREPARED_DEVELOPMENT_HW_CONFIG_H
#include "stm32h7xx_hal.h"

/* 电机数量 */
#define Mo_number 4

/* 底盘尺寸（麦轮运动学解算用，单位 m） */
#define L_chassis 0.19f
#define W_chassis 0.17f
#define K_chassis ((L_chassis+W_chassis)/2)
#define R_wheel 0.04f

/*舵轮解算所需的底盘参数*/

/*
*这里都使用电流控制如果需要使用电压控制自行更改标识符
*CALLBACK是基地址后续电机需要加上电机id进行偏移
*/

#define GM6020_IDE_CH1 0X1FE
#define GM6020_IDE_CH2 0X2FE
#define GM6020_CALLBACK 0x204

#define C610_IDE_CH1 0x200
#define C610_IDE_CH2 0x1FF
#define C610_CALLBACK 0x200

#define C620_IDE_CH1 0X200
#define C620_IDE_CH2 0X1FF
#define C620_CALLBACK 0x200

#define j4340_DM_SpeedAndDirection_BAR 0x100
#define j4340_DM_SpeedOnly_BAR 0x200
#define j4340_DM_SpeedOnly_BAR 0x200

/* 【配置函数】电机初始化 + PID 参数配置，定义在 HW_Config.c，主函数只调一次 */
void Mo_Init_DJI(void);

#endif //H7_PREPARED_DEVELOPMENT_HW_CONFIG_H
