//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
#define H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H

#include "Overall_Struct.h"
#include "bsp_fdcan.h"

enum {
    Ch1 = 1,
    Ch2 = 2
};

/* 设置某一路电机的目标电流/电压指令
 * @param can_CH     CAN 口：1/2/3
 * @param type       电机类型：GM6020 / C610 / C620
 * @param Mo_ID      电机编号：1~4 为通道 1，5~8 为通道 2
 * @param Mo_Current 16 位原始指令值（int16 带符号）
 */
void Mo_Set_Current(uint8_t can_CH, uint8_t type, uint8_t Mo_ID, int16_t Mo_Current);

/* 把缓冲区里所有已设置的电机指令按 DJI 协议打包，通过指定 CAN 口发送 */
void Mo_Sent_Data(hcan_t *hfdcan,uint8_t CH, uint8_t type);
void Mo_type_Setting(uint8_t Channal, uint8_t Mo_ID,uint8_t type);
void Mo_PID_Speed(uint8_t can_CH, uint8_t type, uint8_t Mo_ID, int16_t Speed);
void Mo_PID_Callback(void);

extern Mo_PID  Mo_GM6020[3][7];
extern Mo_PID  Mo_C610_and_C620[3][8];
extern uint8_t Mo_Select[3][8];
#endif //H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
