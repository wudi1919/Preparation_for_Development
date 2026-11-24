//
// Created by Ken on 2026/11/22.
//

#include "Motor_Contral.h"
#include "HW_Config.h"
#include "Overall_Struct.h"
#include "bsp_fdcan.h"

/* 每个通道的电机数（与 Mo_number 一致），CH1/CH2 共两个通道 */
#define MO_CH_NUM    Mo_number
#define MO_SLOT_NUM  (2 * Mo_number)

/* 各电机目标指令缓冲（int16 带符号）
 * 下标 [type][Mo_ID-1]，Mo_ID 1~4 走 CH1，5~8 走 CH2
 * GM6020 存电压（±30000 ↔ ±24V），C610/C620 存电流（原始 int16） */
static int16_t Mo_Tx_Buf[3][MO_SLOT_NUM] = {0};
/* 记录哪些槽位被设置过；只有被设置过的槽位才参与发送，避免发送无意义的 0 帧 */
static uint8_t Mo_Tx_Active[3][MO_SLOT_NUM] = {0};

/**
 * @brief  设置某一路电机的目标电流/电压指令
 * @param  type      电机类型（GM6020 电压控制 / C610、C620 电流控制）
 * @param  Mo_ID     电机编号（1~4 通道1，5~8 通道2）
 * @param  Mo_Current 16 位原始指令值（按 int16 带符号解释，可传负值如 (uint16_t)(-5000)）
 */
void Mo_Set_Current(Mo_type type, uint8_t Mo_ID, uint16_t Mo_Current) {
    /* 越界防护：type 非法 / Mo_ID 非法直接丢弃 */
    if (type < GM6020 || type > C620 || Mo_ID == 0 || Mo_ID > MO_SLOT_NUM)
        return;

    Mo_Tx_Buf[type][Mo_ID - 1] = (int16_t)Mo_Current;   /* 16 位原始值按有符号存储 */
    Mo_Tx_Active[type][Mo_ID - 1] = 1;
}

/**
 * @brief  将缓冲区内的电机指令按 DJI 协议打包并发送
 * @param  hfdcan 目标 CAN 口
 * @note   C610/C620：电流控制，4 个电机共用一帧（8 字节 = 4 × int16 小端）
 *         GM6020：电压控制，每个电机单独一帧（前 2 字节 int16 电压，其余为 0）
 *         注意：C610 与 C620 共用控制 ID（0x200 / 0x1FF），同一系统二者选其一；
 *         GM6020 的 ID 与电流电机存在重叠，应分到不同 CAN 总线使用。
 */
void Mo_Sent_Data(hcan_t *hfdcan) {
    uint8_t tx[8];
    uint16_t id;
    uint16_t uv;
    int i, t;
    int send;

    /* ---- 电流电机 C610 / C620：每帧打包 4 个电机（int16 × 4，小端） ---- */
    for (t = C610; t <= C620; t++) {
        /* CH1：电机 1~4 */
        send = 0;
        for (i = 0; i < MO_CH_NUM; i++) {
            uv = (uint16_t)Mo_Tx_Buf[t][i];
            tx[2 * i]     = (uint8_t)(uv & 0xFF);      /* 低字节在前 */
            tx[2 * i + 1] = (uint8_t)(uv >> 8);        /* 高字节在后 */
            if (Mo_Tx_Active[t][i]) send = 1;
        }
        if (send)
            fdcanx_send_data(hfdcan, (t == C610) ? C610_IDE_CH1 : C620_IDE_CH1, tx, 8);

        /* CH2：电机 5~8 */
        send = 0;
        for (i = 0; i < MO_CH_NUM; i++) {
            uv = (uint16_t)Mo_Tx_Buf[t][MO_CH_NUM + i];
            tx[2 * i]     = (uint8_t)(uv & 0xFF);
            tx[2 * i + 1] = (uint8_t)(uv >> 8);
            if (Mo_Tx_Active[t][MO_CH_NUM + i]) send = 1;
        }
        if (send)
            fdcanx_send_data(hfdcan, (t == C610) ? C610_IDE_CH2 : C620_IDE_CH2, tx, 8);
    }

    /* ---- GM6020 电压电机：每个电机单独一帧（int16 电压占前 2 字节） ---- */
    for (i = 0; i < MO_SLOT_NUM; i++) {
        if (!Mo_Tx_Active[GM6020][i]) continue;

        uv = (uint16_t)Mo_Tx_Buf[GM6020][i];
        tx[0] = (uint8_t)(uv & 0xFF);
        tx[1] = (uint8_t)(uv >> 8);
        tx[2] = tx[3] = tx[4] = tx[5] = tx[6] = tx[7] = 0;

        /* 1~4 号走 CH1（0x1FE+1..4），5~8 号走 CH2（0x2FE+1..4） */
        id = (i < MO_CH_NUM)
             ? (uint16_t)(GM6020_IDE_CH1 + (i + 1))
             : (uint16_t)(GM6020_IDE_CH2 + (i - MO_CH_NUM + 1));

        fdcanx_send_data(hfdcan, id, tx, 8);
    }
}