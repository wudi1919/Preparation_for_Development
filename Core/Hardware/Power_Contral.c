//
// Created by Ken on 2026/11/22.
//

#include "Power_Contral.h"
#include "stm32h7xx.h"
void Power_Contral(Power_Channal Open_ch) {
    if (Open_ch == Ch_15V_1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else if (Open_ch == Ch_15V_2) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    }
    else if (Open_ch == Ch_5V) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }
    else if (Open_ch == ALL) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }
}