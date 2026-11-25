//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
#define H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
#include"Overall_Struct.h"

/* 设置某路 CAN 上 C610/C620 的类型（每个通道只会是 C610 或 C620 其中一种） */
void Mo_Select_DJI(uint8_t Can_Ch, uint8_t type);

/* 设置某个电机的电流指令，写入对应的 Tx 缓冲 */
void Mo_Set_current_DJI(uint8_t type, uint8_t id, int16_t current, uint8_t can_Ch);

/* 把指定 CAN 通道的 Tx 缓冲按标识符发送出去 */
void Mo_Callback_DJI(uint8_t Can_Ch, uint8_t GM6020_Ch, uint8_t C620_And_C610_Ch);

/* 速度环 PID：目标转速 → PID 计算 → 电流写入 Tx 缓冲 */
void Mo_PID_Speed_Control_DJI(uint8_t type, uint8_t id, int16_t Speed, uint8_t can_Ch);

/* 周期回调：跑速度环 PID 并发送 */
void Mo_PID_Callbakc_DJI(uint8_t Can_Ch, uint8_t GM6020_Ch, uint8_t C620_And_C610_Ch);

/* 反馈状态数组（由 Can_Data_Processing_DJI 写入转速/角度） */
extern Mo_PID_Data GM6020_stat[3][8];
extern Mo_PID_Data C620_stat[3][8];
extern Mo_PID_Data C610_stat[3][8];
extern uint8_t CAN3__C620_and_C610_Type_select[3];

#endif //H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
