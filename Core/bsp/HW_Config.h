//
// Created by Ken on 2026/10/19.
//

#ifndef PWM_CHASSIS_CONTROL_HW_CONFIG_H
#define PWM_CHASSIS_CONTROL_HW_CONFIG_H
#include "main.h"

//下面的是给电机控制的pinA/B(AB向)和PWM(定时器通道)写定义的
#define PINA_Mo1 GPIOD, GPIO_PIN_11
#define PINB_Mo1 GPIOD, GPIO_PIN_12
#define PWM_Mo1 TIM_CHANNEL_1

#define PINA_Mo2 GPIOD, GPIO_PIN_13
#define PINB_Mo2 GPIOD, GPIO_PIN_14
#define PWM_Mo2 TIM_CHANNEL_2

#define PINA_Mo3 GPIOD, GPIO_PIN_6
#define PINB_Mo3 GPIOD, GPIO_PIN_7
#define PWM_Mo3 TIM_CHANNEL_3

#define PINA_Mo4 GPIOD, GPIO_PIN_0
#define PINB_Mo4 GPIOD, GPIO_PIN_1
#define PWM_Mo4 TIM_CHANNEL_4

#define PWM_study_MAX 70u
#define Mo_number 4
#define ARR_Register 200u

#define CMD_SCALE 1000.0f   // AA帧命令缩放：实际值(m/s 或 rad/s) = 原始int16值 / 1000

#define Encoder_TIM_1  &htim5
#define Encoder_TIM_2  &htim8
#define Encoder_TIM_3  &htim4
#define Encoder_TIM_4  &htim3

#define L_chassis 0.19f
#define W_chassis 0.17f
#define K_chassis ((L_chassis+W_chassis)/2)

#define R_wheel 0.04f
#endif //PWM_CHASSIS_CONTROL_HW_CONFIG_H
/*
 *这里说明uart传输的帧头对应的意思
 *帧头为BB的就是设置pid的参数,符号字节 rx_buffer[8]/[9]/[10]：0 为负，1 为正
 *帧头为AA的就是设置Vx,Vy,Vy
 *
 */
