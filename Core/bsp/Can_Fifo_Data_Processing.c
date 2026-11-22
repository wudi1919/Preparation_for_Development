//
// Created by Ken on 2026/11/22.
//

#include "Can_Fifo_Data_Processing.h"
#include "Overall_Struct.h"
#include "HW_Config.h"

/* bsp_fdcan.c 里定义的各通道接收缓冲（保存最近一帧的 ID 和 8 字节反馈数据） */
extern uint16_t rec_id1, rec_id2, rec_id3;
extern uint8_t  rx_data1[64], rx_data2[64], rx_data3[64];

/* Motor_Contral.c 里定义的电机 PID 变量（定义见 Motor_Contral.h，[通道][电机号] 均 0 基） */
extern PID_Data Mo_GM6020[3][7];
extern PID_Data Mo_C610[3][7];
extern PID_Data Mo_C620[3][7];

/**
 * @brief  处理某一路 CAN 的电机反馈，把实际转速写入对应 PID 变量的 Actual
 * @param  can_ch 通道号（1 基）：1=FDCAN1，2=FDCAN2，3=FDCAN3
 * @note   反馈帧固定 8 字节（大端）：
 *         byte0~1 转子机械角度(0~8191)，byte2~3 转子转速(RPM，有符号)，
 *         byte4~5 实际转矩电流，byte6 温度，byte7 保留。
 *         这里只取转速（RPM）写入 PID 的 Actual 字段。
 *
 *         电机类型按通道区分：C610/C620 的反馈基址都是 0x200，ID 无法区分二者，
 *         只能靠所在通道判断。当前约定：1 通道=GM6020，2 通道=C610，3 通道=C620。
 *         若与实际接线不一致，改下面 switch 里各 case 对应的电机类型即可。
 */
void can_data_processing(uint8_t can_ch)
{
    uint16_t id;
    uint8_t *data;

    /* 取出对应通道最近一帧的反馈 ID 与数据 */
    switch (can_ch) {
        case 1: id = rec_id1; data = rx_data1; break;
        case 2: id = rec_id2; data = rx_data2; break;
        case 3: id = rec_id3; data = rx_data3; break;
        default: return;                       /* 无效通道，直接返回 */
    }

    uint8_t ch = can_ch - 1;                                 /* 0 基通道索引 */
    int16_t speed = (int16_t)((data[2] << 8) | data[3]);     /* 转子转速 RPM（有符号） */

    /* 按通道路由到对应的电机 PID 变量数组 */
    switch (can_ch) {
        case 1: /* GM6020：反馈 ID = GM6020_CALLBACK(0x204) + 电机号(1~7) */
            if (id >= GM6020_CALLBACK + 1 && id <= GM6020_CALLBACK + 7)
                Mo_GM6020[ch][id - GM6020_CALLBACK - 1].Actual = speed;
            break;
        case 2: /* C610：反馈 ID = C610_CALLBACK(0x200) + 电机号(1~7) */
            if (id >= C610_CALLBACK + 1 && id <= C610_CALLBACK + 7)
                Mo_C610[ch][id - C610_CALLBACK - 1].Actual = speed;
            break;
        case 3: /* C620：反馈 ID = C620_CALLBACK(0x200) + 电机号(1~7) */
            if (id >= C620_CALLBACK + 1 && id <= C620_CALLBACK + 7)
                Mo_C620[ch][id - C620_CALLBACK - 1].Actual = speed;
            break;
        default:
            break;
    }
}
