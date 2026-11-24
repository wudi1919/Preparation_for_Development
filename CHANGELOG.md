# 更新日志

## 2026-11-24

### 完成
- 填充 Motor_Contral 电机控制层：实现 `Mo_Set_Current`（按类型+编号缓冲目标电流/电压指令，带越界防护）与 `Mo_Sent_Data`（按 DJI 协议打包发送：C610/C620 每帧 4 个 int16 电流，GM6020 每电机单独一帧 int16 电压）
- Motor_Contral.h 补充两个函数的声明，方便跨文件调用

## 2026-11-22

### 完成
- 移植 BMI088 驱动（Core/BMI1088）+ 四元数解算（Mahony 滤波，imu_quaternion）
- TIM14 配置为 1ms 定时器，驱动 IMU 解算（My_Tim）
- 串口打印欧拉角（UART7, 115200, FireWater 格式）
- 开启 `-u _printf_float` 支持浮点打印
- 移植底盘控制代码（PID_Calculator、MecSolve、Serial、Power_Contral 等）
- 陀螺仪零偏标定 + 温度补偿（IMU_Gyro_Calibrate / IMU_Gyro_Compensate，缓解 yaw 漂移）
- 重构：把 IMU 初始化/标定/补偿统一移到 imu_quaternion 模块，新增 IMU_Init() 在 main 调用
- FDCAN：bsp_can_init/can_filter_init 改为带 FDCAN_HandleTypeDef* 形参，接收改 watermark 中断，缓冲区 8→64 字节
- 新增蜂鸣器驱动（Buzzer，TIM12 PWM）+ 非阻塞 Beep
- 新增非阻塞延时（bsp_delay / Delay_NonBlock）
- 目录调整：MecSolve → Chassis_Solve，HW_Config → Hardware
- FDCAN：新增运行时标准 CAN ↔ CAN FD 切换（`bsp_fdcan_set_baud` + 模式/波特率宏 + `fdcanx_send_data` 按 `FrameFormat` 自适应帧格式 + 总线关闭恢复回调）；main 启动时把三个 FDCAN 口统一切换为标准 CAN（经典模式 1Mbps）；bsp_fdcan.c 转码为 UTF-8
- FDCAN：新增 `bsp_can_switch()` 一步完成「切换 + 启动」，main 启动改为调用它；`set_baud` 内部补 `Delay_ms(1)` + `can_filter_init` 保证切换后滤波器可靠生效

### 关键修复
- **SPI2 CLKPolarity 必须 High**（CubeMX 默认 Low，会导致 BMI088 读到乱码/全 0）
- **SPI2_SCK = PB13**（不是 PB10）
- FDCAN 引脚按参考板：FDCAN1 = PD0/PD1，FDCAN2 = PB5/PB6，FDCAN3 = PD12/PD13

### 待办
- yaw 轴无磁力计会漂移，需加磁力计或做零偏标定
- FDCAN 引脚需在 CubeMX 中改成参考板的引脚
