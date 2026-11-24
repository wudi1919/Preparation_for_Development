//
// Created by Ken on 2026/11/22.
//

#include "Motor_Contral.h"
#include "HW_Config.h"
#include "Overall_Struct.h"
#include "bsp_fdcan.h"
#include "PID_Calculator.h"

/* 电机控制内部状态（仅本文件可见，不对外暴露） */
Mo_PID Mo_Dem = {0};
Mo_PID Mo_GM6020[3][7]= {0};
Mo_PID Mo_C610_and_C620[3][8]= {0};
uint8_t Mo_Select[3][8]= {0};
//虽然理论上每一条总线上可以挂载15个电机,但是7个电机比较好
/* 发送缓冲：每通道 8 字节（4×int16 大端，高字节在前），直接按 uint8_t* 发送，无需额外拷贝。
 * [通道][字节]：通道 0=CH1(电机1~4)，通道 1=CH2(电机5~8) */
static uint8_t Mo_GM6020_Tx_can1[2][8] = {0};
static uint8_t Mo_C610_and_C620_TX_can1[2][8] = {0};

static uint8_t Mo_GM6020_Tx_can2[2][8] = {0};
static uint8_t Mo_C610_and_C620_TX_can2[2][8] = {0};

static uint8_t Mo_GM6020_Tx_can3[2][8] = {0};
static uint8_t Mo_C610_and_C620_TX_can3[2][8] = {0};


/* 每个通道的电机数（与 Mo_number 一致），CH1/CH2 共两个通道 */

/* 取某个 CAN 口、某个类型、某个通道的 8 字节发送行指针（无拷贝，非法参数返回 NULL） */
static uint8_t *Mo_Tx_Row(uint8_t can, uint8_t type, uint8_t ch) {
    if (type == GM6020) {
        switch (can) {
            case 1: return Mo_GM6020_Tx_can1[ch];
            case 2: return Mo_GM6020_Tx_can2[ch];
            case 3: return Mo_GM6020_Tx_can3[ch];
            default: return NULL;
        }
    } else if (type == C610 || type == C620) {
        switch (can) {
            case 1: return Mo_C610_and_C620_TX_can1[ch];
            case 2: return Mo_C610_and_C620_TX_can2[ch];
            case 3: return Mo_C610_and_C620_TX_can3[ch];
            default: return NULL;
        }
    }
    return NULL;
}

void Mo_Set_Current(uint8_t can_CH, uint8_t type, uint8_t Mo_ID, int16_t Mo_Current) {
    uint8_t *dst;
    uint8_t ch, idx;
    uint16_t u;

    if (can_CH == 0 || can_CH > 3) return;   /* 越界防护 */
    if (Mo_ID  == 0 || Mo_ID  > 8) return;

    ch  = (Mo_ID - 1) / 4;   /* 0:CH1(电机1~4)，1:CH2(电机5~8) */
    idx = (Mo_ID - 1) % 4;   /* 0~3 */

    dst = Mo_Tx_Row(can_CH, type, ch);
    if (dst == NULL) return;

    u = (uint16_t)Mo_Current;
    dst[idx * 2]     = (uint8_t)(u >> 8);     /* 高字节在前 */
    dst[idx * 2 + 1] = (uint8_t)(u & 0xFF);   /* 低字节在后 */
}


void Mo_Sent_Data(hcan_t *hfdcan, uint8_t CH, uint8_t type) {
    uint16_t id;
    uint8_t *src;
    uint8_t ch, can;

    if (CH < Ch1 || CH > Ch2) return;   /* CH 只允许 1 或 2 */

    ch = CH - 1;   /* 转成缓冲下标 0/1 */

    /* 由 FDCAN 句柄反推出是哪个 CAN 口 */
    if      (hfdcan == &hfdcan1) can = 1;
    else if (hfdcan == &hfdcan2) can = 2;
    else if (hfdcan == &hfdcan3) can = 3;
    else return;

    src = Mo_Tx_Row(can, type, ch);
    if (src == NULL) return;

    switch (type) {
        case GM6020:
            id = (ch == 0) ? GM6020_IDE_CH1 : GM6020_IDE_CH2;
            break;
        case C610:id = (ch == 0) ? C610_IDE_CH1 : C610_IDE_CH2;
            break;
        case C620:                            /* 共用 ID，穿透 */
            id = (ch == 0) ? C620_IDE_CH1 : C620_IDE_CH2;
            break;
        default:
            return;
    }

    fdcanx_send_data(hfdcan, id, src, 8);   /* 直接传缓冲指针，无拷贝 */
}

/**
 * @brief  设置某一路电机的目标转速（写入速度环 Target，直接指针，无拷贝）
 * @param  can_CH  CAN 口：1/2/3
 * @param  type    电机类型：GM6020 / C610 / C620
 * @param  Mo_ID   电机编号：1~8（GM6020 最多 7）
 * @param  Speed   目标转速（rpm，带符号）
 */
void Mo_PID_Speed(uint8_t can_CH, uint8_t type, uint8_t Mo_ID, int16_t Speed) {
    PID_Data *pid;

    if (can_CH == 0 || can_CH > 3) return;   /* 越界防护 */
    if (Mo_ID == 0) return;

    if (type == GM6020) {
        if (Mo_ID > 7) return;   /* GM6020 最多 7 个电机 */
        pid = &Mo_GM6020[can_CH - 1][Mo_ID - 1].Mo_Speed;
    } else if (type == C610 || type == C620) {
        if (Mo_ID > 8) return;
        pid = &Mo_C610_and_C620[can_CH - 1][Mo_ID - 1].Mo_Speed;
    } else {
        return;
    }

    pid->Target = (float)Speed;   /* 直接指针写入目标转速 */
}

/**
 * @brief  周期调用（TIM17，10ms）：遍历所有已配置电机，跑速度环 PID，输出电流并发送
 */
void Mo_PID_Callback(void) {
    uint8_t can, Mo_ID, type;
    Mo_PID *pid;
    hcan_t *h;

    /* 1) 遍历所有已配置电机，跑速度环 PID，输出电流到发送缓冲 */
    for (can = 1; can <= 3; can++) {
        for (Mo_ID = 1; Mo_ID <= 8; Mo_ID++) {
            type = Mo_Select[can - 1][Mo_ID - 1];

            if (type == GM6020) {
                if (Mo_ID > 7) continue;   /* GM6020 最多 7 个电机 */
                pid = &Mo_GM6020[can - 1][Mo_ID - 1];
            } else if (type == C610 || type == C620) {
                pid = &Mo_C610_and_C620[can - 1][Mo_ID - 1];
            } else {
                continue;
            }

            PID_Calculator(&pid->Mo_Speed);   /* 速度环 PID，结果写入 pid->Mo_Speed.Out */
            Mo_Set_Current(can, type, Mo_ID, (int16_t)pid->Mo_Speed.Out);   /* 输出电流 */
        }
    }

    /* 2) 发送所有帧（C610/C620 共用缓冲和 ID，只发一次） */
    for (can = 1; can <= 3; can++) {
        if      (can == 1) h = &hfdcan1;
        else if (can == 2) h = &hfdcan2;
        else               h = &hfdcan3;

        Mo_Sent_Data(h, Ch1, GM6020);
        Mo_Sent_Data(h, Ch2, GM6020);
        Mo_Sent_Data(h, Ch1, C610);
        Mo_Sent_Data(h, Ch2, C610);
    }
}

void Mo_type_Setting(uint8_t Channal, uint8_t Mo_ID,uint8_t type) {
    Mo_Select[Channal-1][Mo_ID-1] = type;
}