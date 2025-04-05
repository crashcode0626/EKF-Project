#ifndef EKF_H
#define EKF_H

#include <stdint.h>
#include "arm_math.h"

// =========================================
// 固定电池参数（用户可在此修改默认值）
// =========================================
#define EKF_R0       0.01f    // 欧姆电阻 (Ohm)
#define EKF_R1       0.1f     // RC网络电阻 (Ohm)
#define EKF_C1       50.0f    // RC网络电容 (F)
#define EKF_ETA      0.97f    // 库仑效率
#define EKF_C_N      (2.9f * 3600.0f)  // 额定容量 (库仑, 2.9Ah)
#define EKF_Q        0.01f    // 过程噪声协方差
#define EKF_R        100.0f   // 观测噪声协方差

// =========================================
// EKF状态结构体
// =========================================
typedef struct {
    // 状态矩阵实例
    arm_matrix_instance_f32 P;     // 误差协方差矩阵 (2x2)
    arm_matrix_instance_f32 Ak;    // 状态转移矩阵 (2x2)
    arm_matrix_instance_f32 Ck;    // 观测矩阵 (1x2)
    arm_matrix_instance_f32 Kk;    // 卡尔曼增益 (2x1)
    
    // 状态向量与工作矩阵
    float32_t Xk_data[2];         // 状态向量 [极化电压, SOC]
    arm_matrix_instance_f32 P_pred;  // 预测协方差矩阵
    arm_matrix_instance_f32 temp_2x2; // 临时2x2矩阵
    arm_matrix_instance_f32 temp_2x1; // 临时2x1矩阵
} EKF_State;

// =========================================
// 函数声明
// =========================================
void EKF_Init(EKF_State* S);
float EKF_EstimateSOC(EKF_State* S, float Uk_obs, float Ik_obs, float dt);

#endif
