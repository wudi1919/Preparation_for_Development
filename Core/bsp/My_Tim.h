//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_MY_TIM_H
#define H7_PREPARED_DEVELOPMENT_MY_TIM_H

#include "tim.h"
#include "imu_quaternion.h"

/* 所有定时器的启动都放这里（CubeMX 已经在 MX_TIMx_Init 里配好 PSC/ARR/NVIC）。
 *
 * 以后加新定时器：
 *   1. CubeMX 里配置新定时器（含 NVIC 中断）并重新生成；
 *   2. 在 My_Tim_Init() 里加一句 HAL_TIM_xxx_Start_IT(&htimX)；
 *   3. 在 HAL_TIM_PeriodElapsedCallback() 里加对应的 if 分支。
 */
void My_Tim_Init(void);

#endif //H7_PREPARED_DEVELOPMENT_MY_TIM_H
