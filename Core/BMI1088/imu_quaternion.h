#ifndef IMU_QUATERNION_H
#define IMU_QUATERNION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 四元数：q0 为实部 w，q1/q2/q3 为虚部 x/y/z（单位四元数表示姿态） */
typedef struct {
    float q0;   // w
    float q1;   // x
    float q2;   // y
    float q3;   // z
} Quaternion_t;

/* 欧拉角（单位：弧度 rad） */
typedef struct {
    float yaw;    // 偏航角，绕 Z 轴
    float pitch;  // 俯仰角，绕 Y 轴
    float roll;   // 横滚角，绕 X 轴
} EulerAngle_t;

/* IMU 姿态解算完整结果：四元数 + 欧拉角 */
typedef struct {
    Quaternion_t quat;    // 四元数
    EulerAngle_t angle;   // 欧拉角
} IMU_Attitude_t;

/* 初始化为单位四元数（对应姿态角全为 0） */
void IMU_Quat_Init(Quaternion_t *q);

/* Mahony 互补滤波姿态解算（IMU 版：陀螺仪 + 加速度计，无磁力计）
 * gx/gy/gz : 角速度，单位 rad/s
 * ax/ay/az : 加速度，单位 g（内部会归一化，量纲不影响）
 * 注意：调用周期必须与 imu_quaternion.c 里的 IMU_SAMPLE_FREQ 一致 */
void IMU_Quat_Update(Quaternion_t *q, float gx, float gy, float gz,
                     float ax, float ay, float az);

/* 四元数转欧拉角（rad） */
void IMU_Quat_ToEuler(const Quaternion_t *q, EulerAngle_t *e);

/* 全局 IMU 姿态数据（四元数 + 欧拉角） */
extern IMU_Attitude_t imu;

/* IMU 初始化：上电静止标定（零偏+参考温度）+ 四元数初始化 */
void IMU_Init(void);

/* 陀螺仪零偏 + 温度标定（需板子静止） */
void IMU_Gyro_Calibrate(int samples);

/* 应用零偏 + 温度补偿 */
void IMU_Gyro_Compensate(float gyro[3], float temp);

#ifdef __cplusplus
}
#endif

#endif /* IMU_QUATERNION_H */
