# 更新日志

## 2026-11-22

### 完成
- 移植 BMI088 驱动（Core/BMI1088）+ 四元数解算（Mahony 滤波，imu_quaternion）
- TIM14 配置为 1ms 定时器，驱动 IMU 解算（My_Tim）
- 串口打印欧拉角（UART7, 115200, FireWater 格式）
- 开启 `-u _printf_float` 支持浮点打印
- 移植底盘控制代码（PID_Calculator、MecSolve、Serial、Power_Contral 等）

### 关键修复
- **SPI2 CLKPolarity 必须 High**（CubeMX 默认 Low，会导致 BMI088 读到乱码/全 0）
- **SPI2_SCK = PB13**（不是 PB10）
- FDCAN 引脚按参考板：FDCAN1 = PD0/PD1，FDCAN2 = PB5/PB6，FDCAN3 = PD12/PD13

### 待办
- yaw 轴无磁力计会漂移，需加磁力计或做零偏标定
- FDCAN 引脚需在 CubeMX 中改成参考板的引脚
