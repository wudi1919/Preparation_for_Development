//
// Created by Ken on 2026/11/22.
//

#include "Buzzer.h"
#include "bsp_delay.h"

static uint32_t beep_tick = 0;
static uint16_t beep_ms   = 0;
static uint8_t  beeping   = 0;

void Buzzer_Init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    Buzzer_Off();   // 上电默认静音
}

void Buzzer_On(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 1000);   // 50% 占空比，发声
}

void Buzzer_Off(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);      // 0 占空比，静音
}

void Buzzer_Beep(uint16_t ms)
{
    Buzzer_On();
    Delay_ms(ms);
    Buzzer_Off();
    Delay_ms(ms);
}

void Buzzer_Beep_multiple(uint16_t Interval_time, uint16_t times) {
    for (uint16_t i = 0; i < times; i++) {
        Buzzer_Beep(Interval_time);
    }
    Delay_ms(Interval_time*10);
}

