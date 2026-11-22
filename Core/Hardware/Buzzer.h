//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_BUZZER_H
#define H7_PREPARED_DEVELOPMENT_BUZZER_H

#include "tim.h"

/* 有源蜂鸣器：TIM12_CH2（PB15）PWM 驱动
 * 占空比 > 0 响，= 0 静音 */

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

/* 非阻塞：开始响 ms 毫秒，需在主循环里周期调用 Buzzer_Process 才会自动停 */
void Buzzer_Beep(uint16_t ms);

/* 非阻塞：主循环里周期调用，到时间自动关 */
void Buzzer_Process(void);

#endif //H7_PREPARED_DEVELOPMENT_BUZZER_H
