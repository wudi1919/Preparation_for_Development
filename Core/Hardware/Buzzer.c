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
    beep_tick = HAL_GetTick();   // 记录开始时间
    beep_ms   = ms;
    beeping   = 1;
}

void Buzzer_Process(void)
{
    if (beeping && Delay_NonBlock(&beep_tick, beep_ms))
    {
        Buzzer_Off();
        beeping = 0;
    }
}
