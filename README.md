# 大疆电机串级控制（C610 / GM6020）使用例程

本工程对 DJI C610（M2006）和 GM6020 提供「速度环 / 绝对位置环 / 相对增量角度环」三种控制，
采用串级结构：**位置环（增量式 PID）→ 目标转速 → 速度环（位置式 PID）→ 力矩/电压**。

> 核心调用模型：你只在任意函数里**设置目标/参数**；真正的 PID 计算与 CAN 发送由
> `TIM17` 中断（20ms）里的 `Mo_PID_Callbakc_DJI()` 自动完成，**不要手动发 CAN**。

---

## 1. 电机类型与指令单位

| 类型 | 常量 | 控制量 | OutMax 建议 |
|------|------|--------|-------------|
| C610 (M2006) | `C610` | 电流指令 | `1000`（约 ±1A，保守） |
| C620 | `C620` | 电流指令 | `1000` |
| GM6020 | `GM6020` | 电压指令 | `30000`（约 ±30V 满量程） |

- C610/C620 用 `Mo_Select_DJI(can_ch, C610)` 指定某路 CAN 上的类型。
- 速度环 PID 输出限幅（`OutMax`）必须按上表设置，否则扭矩/电压量级不对。

---

## 2. 控制模式

```c
typedef enum {
    DJI_CTRL_POSITION  = 0,   // 绝对位置环（0~360°，取最短路径，默认）
    DJI_CTRL_SPEED     = 1,   // 速度环
    DJI_CTRL_INCREMENT = 2    // 相对增量角度环（可超 360°，多圈累计）
} DJI_CtrlMode;
```

---

## 3. API 一览

```c
// 电机初始化 + PID 参数配置统一入口，主函数只调一次
void Mo_Init_DJI(void);

// 指定某路 CAN 上 C610/C620 的类型（每路只会是其中一种）
void Mo_Select_DJI(uint8_t can_ch, uint8_t type);

// 速度环目标（RPM，带符号），进入速度环模式
void Mo_PID_Speed_Control_DJI(uint8_t type, uint8_t id, int16_t speed, uint8_t can_ch);

// 绝对位置目标（0~360°，度），进入位置环模式
void Mo_PID_Position_Control_DJI(uint8_t type, uint8_t id, float angle_deg, uint8_t can_ch);

// 相对增量角度（度，可超 360），以当前位置为起点，进入增量模式
void Mo_PID_Increment_Control_DJI(uint8_t type, uint8_t id, float delta_deg, uint8_t can_ch);

// 配置位置环（增量式 PID），out_max 为目标转速上限（RPM）
void Mo_PID_Position_Config_DJI(uint8_t type, uint8_t id, float kp, float ki, float kd, float out_max, uint8_t can_ch);

// 重力前馈：F = k * sin(角度 + phase_deg)，直接加在力矩上
void Mo_Set_Gravity_FF_DJI(uint8_t type, uint8_t id, float k, float phase_deg, uint8_t can_ch);

// 手动切模式
void Mo_Set_CtrlMode_DJI(uint8_t type, uint8_t id, uint8_t mode, uint8_t can_ch);

// 速度环参数（位置式 PID）：kp, ki, kd, out_max, int_max
void PID_config(PID_Data *pid, float kp, float ki, float kd, float out_max, float int_max);
```

电机编号约定：GM6020 / C610 每路 CAN 最多 8 个，`id` 为 1~8；
`id 1~4` 走 CH1，`id 5~8` 走 CH2。

---

## 4. 例程

### 4.0 初始化统一入口 `Mo_Init_DJI`

所有电机的「选型 + PID 参数配置 + 初始目标」都收敛到 `Mo_Init_DJI()` 里，主函数只需调一次：

```c
Mo_Init_DJI();   // 在 main() 的 USER CODE BEGIN 2 里，仅此一句
```

`Mo_Init_DJI()` 内部（C610 电调范例，位于 `Motor_Contral.c`）：

```c
void Mo_Init_DJI(void) {
    Mo_Select_DJI(1, C610);                                            // CAN1 选 C610
    PID_config(&C610_stat[0][4].Speed, 3.0f, 2.0f, 0.45f, 1000.0f, 3000.0f); // 速度环(内环)
    Mo_PID_Position_Config_DJI(C610, 5, 2.0f, 0.3f, 0.0f, 120.0f, 1);  // 位置环(外环)
    Mo_Set_Gravity_FF_DJI(C610, 5, 0.0f, 0.0f, 1);                     // 重力前馈(无负载)
    Mo_PID_Position_Control_DJI(C610, 5, 90.0f, 1);                    // 初始目标 90°
}
```

**使用规则**：
- 换电机 / 改参数 / 改初始目标，都在 `Mo_Init_DJI()` 内部改，**不要往 `main.c` 里堆初始化**。
- GM6020 初始化时：`PID_config` 的 `OutMax` 改 `30000`（电压指令）、数组用 `GM6020_stat`、`Mo_Set_Gravity_FF_DJI` 按机构标定，其余同理。
- 初始目标这行如需速度环，换成 `Mo_PID_Speed_Control_DJI`；不想要上电就动，删掉这行即可。

---

### 4.1 C610 速度环（已有自整定参数）

```c
Mo_Select_DJI(1, C610);
PID_config(&C610_stat[0][4].Speed, 3.0f, 2.0f, 0.45f, 1000.0f, 3000.0f);
Mo_PID_Speed_Control_DJI(C610, 5, 120, 1);   // CAN1, ID5, 120 RPM
```

### 4.2 C610 绝对位置（0~360°）

```c
Mo_Select_DJI(1, C610);
PID_config(&C610_stat[0][4].Speed, 3.0f, 2.0f, 0.45f, 1000.0f, 3000.0f); // 内环
Mo_PID_Position_Config_DJI(C610, 5, 2.0f, 0.3f, 0.0f, 120.0f, 1);          // 外环
Mo_Set_Gravity_FF_DJI(C610, 5, 0.0f, 0.0f, 1);                             // 无重力负载
Mo_PID_Position_Control_DJI(C610, 5, 90.0f, 1);                            // 转到 90°
```

### 4.3 C610 相对增量角度（可超 360°，多圈）

```c
// 从当前位置再转 2 圈（720°），多圈累计不丢圈
Mo_PID_Increment_Control_DJI(C610, 5, 720.0f, 1);
```

### 4.4 GM6020 位置环

```c
Mo_Select_DJI(1, GM6020);   // GM6020 恒为该类型，此句可省略
PID_config(&GM6020_stat[0][0].Speed, 3.0f, 2.0f, 0.45f, 30000.0f, 30000.0f); // 电压指令
Mo_PID_Position_Config_DJI(GM6020, 1, 2.0f, 0.3f, 0.0f, 120.0f, 1);           // 参数需标定
Mo_Set_Gravity_FF_DJI(GM6020, 1, 0.0f, 0.0f, 1);                              // 机械臂需标定
Mo_PID_Position_Control_DJI(GM6020, 1, 90.0f, 1);                             // 转到 90°
```

### 4.5 机械臂夹取任务（状态机示例）

```c
typedef enum { MOVE_TO_GRIP, GRIP, LIFT, DONE } TaskState;
static TaskState st = MOVE_TO_GRIP;

void Arm_Task(void) {
    switch (st) {
    case MOVE_TO_GRIP:
        Mo_PID_Position_Control_DJI(GM6020, 1, 30.0f, 1);   // 关节1 → 30°
        Mo_PID_Position_Control_DJI(GM6020, 2, 90.0f, 1);   // 关节2 → 90°
        if (fabsf(GM6020_stat[0][0].Pos.Error0) < 2.0f &&
            fabsf(GM6020_stat[0][1].Pos.Error0) < 2.0f) {
            st = GRIP;
        }
        break;
    case GRIP:
        Gripper_Close();      // 夹爪：舵机(PWM)或气动(IO)，不是这套 CAN 速度环
        st = LIFT;
        break;
    case LIFT:
        Mo_PID_Position_Control_DJI(GM6020, 2, 120.0f, 1); // 关节2 → 120°
        if (fabsf(GM6020_stat[0][1].Pos.Error0) < 2.0f) st = DONE;
        break;
    case DONE:
        break;
    }
}
```

把 `Arm_Task()` 放进主循环 `while(1)` 即可（只设目标/读误差，不碰 CAN）。

---

## 5. 到位判断

位置环的当前角度误差存在 `stat->Pos.Error0`（度，= 目标 − 实际）：

```c
// 到位：|误差| < 阈值（如 2°）
if (fabsf(C610_stat[0][4].Pos.Error0) < 2.0f) { /* 到位 */ }
```

> 该值在 TIM17 中断里每 20ms 更新一次，对机械臂到位判断足够。

---

## 6. 重力前馈标定（GM6020 机械臂）

位置环积分项能兜底消除稳态误差，但动态响应慢。标定前馈后响应更快：

```c
// F = k * sin(角度 + phase_deg)，加在电压指令上
Mo_Set_Gravity_FF_DJI(GM6020, 1, k, phase_deg, 1);
```

标定步骤（建议）：
1. `k=0, phase=0` 先跑位置环，记录稳定后的 `Pos.Error0`。
2. 逐步增大 `k`，直到静止时 `Pos.Error0` 接近 0 且不振荡。
3. `phase` 用于对齐「机构机械零点」与「电机电气零点」的相位差，按实际零点偏差整定。

---

## 7. 串口调参协议（速度环）

- 帧格式：`0x55 0xAA + Kp_hi Kp_lo + Ki_hi Ki_lo + Kd_hi Kd_lo`（8 字节）
- 三个 `int16` 大端，值为参数 ×100，范围 0~50
- 成功回复 `OK Kp=... Ki=... Kd=...`

---

## 8. 注意事项

- 速度环 `OutMax` 必须匹配电机类型（C610=1000 电流 / GM6020=30000 电压），否则量级错乱。
- 位置环参数（`Mo_PID_Position_Config_DJI`）和重力前馈（`Mo_Set_Gravity_FF_DJI`）当前为保守占位值，**需按实际负载/机构实测标定**。
- 不要在主循环和中断里同时调用 `Mo_PID_Callbakc_DJI`，会造成重入和重复发送。
- 增量角度环天然支持多圈（超 360°），由内部累计角度实现，不会丢圈。
