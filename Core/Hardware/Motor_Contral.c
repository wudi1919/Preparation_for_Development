//
// Created by Ken on 2026/11/22.
//

#include "Motor_Contral.h"
#include"HW_Config.h"
#include"Overall_Struct.h"
#include "bsp_fdcan.h"
static uint8_t CAN_GM6020[3][8];//考虑到can总线负载每一路只考虑使用最多8个大疆的电机所以id是1到4还是5到八也需要单独配置
static uint8_t CAN_C620_and_C610[3][8];



Mo_PID_Data GM6020_stat[3][8];
Mo_PID_Data C620_stat[3][8];
Mo_PID_Data C610_stat[3][8];
uint8_t CAN3__C620_and_C610_Type_select[3];

void Mo_Select_DJI(uint8_t Can_Ch,uint8_t type) {
    CAN3__C620_and_C610_Type_select[Can_Ch-1] = type;
}

void Mo_Set_current_DJI(uint8_t type,uint8_t id,int16_t current,uint8_t can_Ch) {
    /* 越界/非法保护 */
    if (can_Ch == 0 || can_Ch > 3 || id == 0 || id > 8) return;

    uint8_t *buf;
    if (type == GM6020) {
        buf = CAN_GM6020[can_Ch - 1];
    } else if (type == C610 || type == C620) {
        buf = CAN_C620_and_C610[can_Ch - 1];
    } else {
        return;
    }

    /* 每帧 8 字节 = 4 个电机（int16 × 4）。
     * id 1~4 走 CH1（槽 0~3），id 5~8 走 CH2（也映射到槽 0~3）。
     * 实际使用只会用到 CH1 或 CH2 其中一个，所以共用一个缓冲不冲突。 */
    uint8_t slot = (id - 1) & 0x03;                       /* 0~3 */
    buf[slot * 2]     = (uint8_t)((uint16_t)current >> 8);  /* 高字节在前（大端） */
    buf[slot * 2 + 1] = (uint8_t)current;                   /* 低字节在后 */
}

void Mo_Callback_DJI(uint8_t Can_Ch,uint8_t GM6020_Ch,uint8_t C620_And_C610_Ch) {
    if (Can_Ch < 1 || Can_Ch > 3) return;

    FDCAN_HandleTypeDef *hfdcan;
    switch (Can_Ch) {
        case 1: hfdcan = &hfdcan1; break;
        case 2: hfdcan = &hfdcan2; break;
        default: hfdcan = &hfdcan3; break;
    }

    uint8_t ch = Can_Ch - 1;

    /* GM6020：CH1 0x1FE / CH2 0x2FE */
    switch (GM6020_Ch) {
        case 1: fdcanx_send_data(hfdcan, GM6020_IDE_CH1, CAN_GM6020[ch], 8); break;
        case 2: fdcanx_send_data(hfdcan, GM6020_IDE_CH2, CAN_GM6020[ch], 8); break;
    }

    /* C610/C620：CH1 0x200 / CH2 0x1FF */
    switch (C620_And_C610_Ch) {
        case 1: fdcanx_send_data(hfdcan, C610_IDE_CH1, CAN_C620_and_C610[ch], 8); break;
        case 2: fdcanx_send_data(hfdcan, C610_IDE_CH2, CAN_C620_and_C610[ch], 8); break;
    }
}