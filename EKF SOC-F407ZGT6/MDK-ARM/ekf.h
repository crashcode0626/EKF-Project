#ifndef EKF_H
#define EKF_H

#include <stdint.h>
#include "arm_math.h"

typedef struct {
    // 电池模型参数
    float R0;       // 欧姆电阻 (Ohm)
    float R1;       // RC网络电阻 (Ohm)
    float C1;       // RC网络电容 (F)
    float Eta;      // 库仑效率
    float C_N;      // 电池额定容量 (库仑)
    
    // EKF参数
    arm_matrix_instance_f32 P;     // 误差协方差矩阵 (2x2)
    arm_matrix_instance_f32 Ak;    // 状态转移矩阵 (2x2)
    arm_matrix_instance_f32 Ck;    // 观测矩阵 (1x2)
    arm_matrix_instance_f32 Kk;    // 卡尔曼增益 (2x1)
    
    float32_t Q;        // 过程噪声协方差
    float32_t R;        // 观测噪声协方差
    float32_t Wk_data[2]; // 过程噪声权重
    float32_t Xk_data[2]; // 状态向量 [极化电压, SOC]
    
    // 工作矩阵
    arm_matrix_instance_f32 P_pred;
    arm_matrix_instance_f32 temp_2x2;
    arm_matrix_instance_f32 temp_2x1;
} EKF_State;

void EKF_Init(EKF_State* S);
float EKF_EstimateSOC(EKF_State* S, float Uk_obs, float Ik_obs, float dt);

#endif