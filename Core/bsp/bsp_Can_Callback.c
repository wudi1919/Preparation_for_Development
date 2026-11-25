//
// Created by Ken on 2026/11/25.
//

#include "bsp_Can_Callback.h"
#include "Overall_Struct.h"
#include "HW_Config.h"

#include <stdint.h>

/* 转速一阶低通滤波系数：越小越平滑（滞后越大），范围 0~1 */
#define SPEED_LPF_ALPHA 0.3f

/* bsp_fdcan.c 里的接收缓冲与 ID（全局，此处 extern 引用，不拷贝） */
extern uint8_t  rx_data1[64];
extern uint16_t rec_id1;
extern uint8_t  rx_data2[64];
extern uint16_t rec_id2;
extern uint8_t  rx_data3[64];
extern uint16_t rec_id3;

/* Motor_Contral.c 里的反馈状态数组 */
extern Mo_PID_Data GM6020_stat[3][8];
extern Mo_PID_Data C610_stat[3][8];
extern Mo_PID_Data C620_stat[3][8];

/* Motor_Contral.c 里的类型选择：每个 CAN 通道只会是 C610 或 C620 其中一种 */
extern uint8_t CAN3__C620_and_C610_Type_select[3];

/**
 * @brief  大疆电机反馈解析：把三种电机的转速/角度写入对应的 *_stat 结构体
 * @param  Can_Ch  CAN 口：1 / 2 / 3
 * @note   反馈帧（8 字节，大端，高位在前）：
 *           DATA[0..1] 机械角 0~8191
 *           DATA[2..3] 转速 rpm（int16 带符号）
 *         反馈 ID：
 *           GM6020     = 0x204 + ID（1~7）→ 0x205~0x20B
 *           C610 / C620 = 0x200 + ID（1~8）→ 0x201~0x208
 *         C610 与 C620 共享反馈 ID；由 Type_select[Can_Ch-1] 决定该通道是 C610 还是 C620。
 */
void Can_Data_Processing_DJI(uint8_t Can_Ch) {
    uint8_t  *rx_data;
    uint16_t  rec_id;

    switch (Can_Ch) {
        case 1: rx_data = rx_data1; rec_id = rec_id1; break;
        case 2: rx_data = rx_data2; rec_id = rec_id2; break;
        case 3: rx_data = rx_data3; rec_id = rec_id3; break;
        default: return;
    }

    /* 大端解析：机械角 0~8191，转速 rpm */
    uint16_t angle = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
    int16_t  speed = (int16_t)((rx_data[2] << 8) | rx_data[3]);

    uint8_t c  = Can_Ch - 1;
    uint8_t id;

    /* C610 / C620：反馈 ID = 0x200 + ID（1~8），按该通道类型选择是 C610 还是 C620 */
    if (rec_id > C610_CALLBACK && rec_id <= C610_CALLBACK + 8) {
        id = (uint8_t)(rec_id - C610_CALLBACK);   /* 1~8 */
        if (CAN3__C620_and_C610_Type_select[c] == C610) {
            C610_stat[c][id - 1].Speed.Actual += SPEED_LPF_ALPHA * (speed - C610_stat[c][id - 1].Speed.Actual);
            C610_stat[c][id - 1].Angel.Actual = angle;
            return;
        }
        if (CAN3__C620_and_C610_Type_select[c] == C620) {
            C620_stat[c][id - 1].Speed.Actual += SPEED_LPF_ALPHA * (speed - C620_stat[c][id - 1].Speed.Actual);
            C620_stat[c][id - 1].Angel.Actual = angle;
            return;
        }
    }

    /* GM6020：反馈 ID = 0x204 + ID（1~7） */
    if (rec_id > GM6020_CALLBACK && rec_id <= GM6020_CALLBACK + 7) {
        id = (uint8_t)(rec_id - GM6020_CALLBACK);  /* 1~7 */
        GM6020_stat[c][id - 1].Speed.Actual += SPEED_LPF_ALPHA * (speed - GM6020_stat[c][id - 1].Speed.Actual);
        GM6020_stat[c][id - 1].Angel.Actual = angle;
    }
}
