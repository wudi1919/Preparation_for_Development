//
// Created by Ken on 2026/11/24.
//

#include "bsp_Can_Callback.h"
#include "Overall_Struct.h"
#include "HW_Config.h"
#include "Motor_Contral.h"

/* bsp_fdcan.c 里的接收缓冲与 ID（全局，此处 extern 引用，不拷贝） */
extern uint8_t  rx_data1[64];
extern uint16_t rec_id1;
extern uint8_t  rx_data2[64];
extern uint16_t rec_id2;
extern uint8_t  rx_data3[64];
extern uint16_t rec_id3;

/* Motor_Contral.c 里的电机 PID 数据（全局，直接指针写入，无拷贝） */
/**
 * @brief  CAN 反馈接收回调：解析反馈帧，直接把速度/角度写入 PID 数据（无拷贝）
 * @param  Can_Ch  CAN 口：1/2/3
 * @note   反馈 ID：C610/C620 = 0x200+Mo_ID(1~8)，GM6020 = 0x204+Mo_ID(1~7)，
 *         两者在 0x205~0x208 重叠，用 Mo_Select[Can_Ch-1][Mo_ID-1] 判定类型。
 *         反馈帧（大端，高字节在前）：DATA[0..1] 机械角 0~8191，DATA[2..3] 转速 rpm（带符号），
 *         DATA[4..5] 转矩电流，DATA[6] 温度，DATA[7] 空。
 *         数组下标统一 Mo_ID-1（与 Mo_Set_Current 一致）。
 */
void My_can_Read_Callback(uint8_t Can_Ch) {
    Mo_PID   *pid = NULL;
    uint8_t  *rx_data;
    uint16_t  rx_id;
    uint8_t   Mo_ID, t;
    int16_t   angle, speed;

    /* 按 CAN 口选对应的接收缓冲 */
    switch (Can_Ch) {
        case 1: rx_data = rx_data1; rx_id = rec_id1; break;
        case 2: rx_data = rx_data2; rx_id = rec_id2; break;
        case 3: rx_data = rx_data3; rx_id = rec_id3; break;
        default: return;
    }

    /* 先按 GM6020 判定（反馈基址 0x204，电机 1~7） */
    Mo_ID = (uint8_t)(rx_id - GM6020_CALLBACK);
    if (Mo_ID >= 1 && Mo_ID <= 7 && Mo_Select[Can_Ch - 1][Mo_ID - 1] == GM6020) {
        pid = &Mo_GM6020[Can_Ch - 1][Mo_ID - 1];
    } else {
        /* 再按 C610/C620 判定（反馈基址 0x200，电机 1~8） */
        Mo_ID = (uint8_t)(rx_id - C610_CALLBACK);
        if (Mo_ID >= 1 && Mo_ID <= 8) {
            t = Mo_Select[Can_Ch - 1][Mo_ID - 1];
            if (t == C610 || t == C620)
                pid = &Mo_C610_and_C620[Can_Ch - 1][Mo_ID - 1];
        }
    }

    if (pid == NULL) return;   /* 未匹配到任何已配置电机 */

    /* 大端解析（高字节在前） */
    angle = (int16_t)((rx_data[0] << 8) | rx_data[1]);   /* 机械角 0~8191 */
    speed = (int16_t)((rx_data[2] << 8) | rx_data[3]);   /* 转速 rpm（带符号） */

    /* 直接指针写入 PID 数据，无拷贝 */
    pid->Mo_Speed.Actual     = (float)speed;
    pid->Mo_Direction.Actual = (float)angle;
}
