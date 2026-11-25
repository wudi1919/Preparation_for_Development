//
// Created by Ken on 2026/11/25.
//

#include "bsp_Can_Callback.h"
#include "Overall_Struct.h"
#include "HW_Config.h"

#include <stdint.h>

/* 转速一阶低通滤波系数：越小越平滑（滞后越大），范围 0~1。
 * CAN 反馈 1kHz：ALPHA=0.15 → 时间常数约 6.7ms，
 * 既压住低速齿槽/换相噪声，又比 PID 周期(20ms)快得多，不拖慢闭环。
 * 仅用于 C620（仍走直报速度）。C610/GM6020 已改用角度差分测速。 */
#define SPEED_LPF_ALPHA 0.15f

/* ---------------- 多圈累计角度（角度制，度） ----------------
 * 电调直报速度在低速时噪声极大，不适合低速闭环。统一改用机械角 0~8191
 * 差分：20ms 窗口 120 RPM → 约 328 counts，量化误差仅 ±0.37 RPM。
 * 累计时处理 0~360° 回卷，得到连续多圈角度（可超 360°），
 * 同时供 Mo_Contral_DJI 差分测速和位置/增量角度环使用。 */
#define DJI_ANGLE_SCALE (360.0f / 8192.0f)   /* counts → 度 */

static float   dji_angle_multiturn[3][8];    /* 多圈累计角度（度，可超360） */
static float   dji_angle_prev_deg[3][8];     /* 上帧单圈角度（度，回卷检测） */
static uint8_t dji_angle_valid[3][8];        /* 首帧标志 */

/* CAN 回调里累积：单圈角度差分 + 回卷处理，累加成连续多圈角度 */
static void DJI_Angle_Accumulate(uint8_t c, uint8_t id, uint16_t angle_counts) {
    float deg = (float)angle_counts * DJI_ANGLE_SCALE;
    if (!dji_angle_valid[c][id]) {
        dji_angle_prev_deg[c][id] = deg;
        dji_angle_valid[c][id] = 1;
    } else {
        float d = deg - dji_angle_prev_deg[c][id];
        if (d >  180.0f) d -= 360.0f;
        if (d < -180.0f) d += 360.0f;
        dji_angle_multiturn[c][id] += d;
        dji_angle_prev_deg[c][id] = deg;
    }
}

/**
 * @brief  读取多圈累计角度（度，可超 360）
 * @param  can_ch  CAN 通道 1~3
 * @param  id      电机 ID 1~8
 * @return 累计角度（度，带符号，可超 360）
 */
float DJI_Angle_MultiTurn_Deg(uint8_t can_ch, uint8_t id) {
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return 0.0f;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float a = dji_angle_multiturn[can_ch - 1][id - 1];
    if (!primask) __enable_irq();
    return a;
}

/* bsp_fdcan.c 里的接收缓冲与 ID（全局，此处 extern 引用，不拷贝） */
extern uint8_t  rx_data1[64];
extern uint16_t rec_id1;
extern uint8_t  rx_data2[64];
extern uint16_t rec_id2;
extern uint8_t  rx_data3[64];
extern uint16_t rec_id3;

/* Mo_Contral_DJI.c 里的反馈状态数组 */
extern Mo_PID_Data GM6020_stat[3][8];
extern Mo_PID_Data C610_stat[3][8];
extern Mo_PID_Data C620_stat[3][8];

/* Mo_Contral_DJI.c 里的类型选择：每个 CAN 通道只会是 C610 或 C620 其中一种 */
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
            /* C610：累积多圈角度（测速在 Mo_Contral_DJI 用多圈角度差分做） */
            DJI_Angle_Accumulate(c, id - 1, angle);
            return;
        }
        if (CAN3__C620_and_C610_Type_select[c] == C620) {
            C620_stat[c][id - 1].Speed.Actual += SPEED_LPF_ALPHA * (speed - C620_stat[c][id - 1].Speed.Actual);
            return;
        }
    }

    /* GM6020：反馈 ID = 0x204 + ID（1~7），同样用多圈角度差分（补上 C610 已有的测速能力） */
    if (rec_id > GM6020_CALLBACK && rec_id <= GM6020_CALLBACK + 7) {
        id = (uint8_t)(rec_id - GM6020_CALLBACK);  /* 1~7 */
        DJI_Angle_Accumulate(c, id - 1, angle);
    }
}
