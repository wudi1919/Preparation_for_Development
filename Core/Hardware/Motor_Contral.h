//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
#define H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H

#include "Overall_Struct.h"
#include "bsp_fdcan.h"

/* 设置某一路电机的目标电流/电压指令
 * @param type      电机类型：GM6020（电压控制）/ C610、C620（电流控制）
 * @param Mo_ID     电机编号：1~4 为通道 1，5~8 为通道 2
 * @param Mo_Current 16 位原始指令值（按 int16 带符号解释；负值请传 (uint16_t)(-x)）
 */
void Mo_Set_Current(Mo_type type, uint8_t Mo_ID, uint16_t Mo_Current);

/* 把缓冲区里所有已设置的电机指令按 DJI 协议打包，通过指定 CAN 口发送 */
void Mo_Sent_Data(hcan_t *hfdcan);

#endif //H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
