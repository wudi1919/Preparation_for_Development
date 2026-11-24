//
// Created by Ken on 2026/11/22.
//

#include "Motor_Contral.h"
#include "HW_Config.h"
#include "bsp_fdcan.h"
#include "PID_Calculator.h"

static int8_t TX_Buffer_Demo[8] = {0};

/**
 * @brief  设置 1 号 GM6020 电机电流并发送（CAN1，控制帧 0x1FE，大端）
 * @param  current 电流原始值（16 位，按 int16 带符号解释；负值请传 (uint16_t)(-x)）
 * @note   0x1FE 一帧装 4 个电机，这里只控制 1 号电机（前 2 字节），其余电机保持 0。
 */
void Motor_Contral_current_Demo(uint16_t current) {
    TX_Buffer_Demo[0] = (uint8_t)(current >> 8);       /* 高字节在前 */
    TX_Buffer_Demo[1] = (uint8_t)(current & 0xFF);     /* 低字节在后 */

    fdcanx_send_data(&hfdcan1, GM6020_IDE_CH1, (uint8_t *)TX_Buffer_Demo, 8);
}

/**
 * @brief  周期调用：跑 1 号 GM6020 的速度环 PID，输出电流并发送
 * @note   反馈转速需由 can_data_processing_Demo 写入 Mo_Demo.Mo_Speed.Actual。
 */
void Motor_Callback_Demo(void) {
    PID_Calculator(&Mo_Demo.Mo_Speed);                                  /* 速度环 PID */
    Motor_Contral_current_Demo((uint16_t)(int16_t)Mo_Demo.Mo_Speed.Out); /* 输出电流并发送 */
}

void Motor_Contral_PID(float Speed) {
    Mo_Demo.Mo_Speed.Target = Speed;
}
