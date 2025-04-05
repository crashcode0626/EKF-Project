#include "ekf.h"
#include <math.h>  // 使用标准数学库替代CMSIS-DSP多项式函数

// =========================================
// 静态内存分配（4字节对齐）
// =========================================
__attribute__((aligned(4))) static float32_t P_data[4] = {1e-8f, 0, 0, 1e-6f};
__attribute__((aligned(4))) static float32_t Ak_data[4];
__attribute__((aligned(4))) static float32_t Ck_data[2];
__attribute__((aligned(4))) static float32_t Kk_data[2];
__attribute__((aligned(4))) static float32_t P_pred_data[4];
__attribute__((aligned(4))) static float32_t temp_2x2_data[4];
__attribute__((aligned(4))) static float32_t temp_2x1_data[2];

// =========================================
// OCV多项式系数（MATLAB导出值）
// =========================================
static const float32_t ocv_coeff[6] = {
    1.6026f, -8.2896f, 14.8878f, -12.0873f, 5.5781f, 2.5047f
};

// =========================================
// 初始化函数
// =========================================
void EKF_Init(EKF_State* S) {
    // 初始化矩阵实例
    arm_mat_init_f32(&S->P, 2, 2, P_data);
    arm_mat_init_f32(&S->Ak, 2, 2, Ak_data);
    arm_mat_init_f32(&S->Ck, 1, 2, Ck_data);
    arm_mat_init_f32(&S->Kk, 2, 1, Kk_data);
    
    // 工作矩阵初始化
    arm_mat_init_f32(&S->P_pred, 2, 2, P_pred_data);
    arm_mat_init_f32(&S->temp_2x2, 2, 2, temp_2x2_data);
    arm_mat_init_f32(&S->temp_2x1, 2, 1, temp_2x1_data);
    
    // 初始状态
    S->Xk_data[0] = 0.0f;    // 初始极化电压
    S->Xk_data[1] = 0.9f;    // 初始SOC=100%
}

// =========================================
// 多项式计算
// =========================================
static float32_t calculate_ocv(float32_t soc) {
    // 手动展开多项式计算: coeff[0]*soc^5 + coeff[1]*soc^4 + ... + coeff[5]
    const float32_t* c = ocv_coeff;
    return c[0] * powf(soc, 5) 
         + c[1] * powf(soc, 4) 
         + c[2] * powf(soc, 3) 
         + c[3] * powf(soc, 2) 
         + c[4] * soc 
         + c[5];
}

static float32_t calculate_docv_dsoc(float32_t soc) {
    // 手动计算导数值: d(OCV)/d(SOC) = 5*coeff[0]*soc^4 + ... + coeff[4]
    const float32_t deriv_coeff[5] = {
        8.013f, -33.1584f, 44.6634f, -24.1746f, 5.5781f
    };
    const float32_t* c = deriv_coeff;
    return c[0] * powf(soc, 4) 
         + c[1] * powf(soc, 3) 
         + c[2] * powf(soc, 2) 
         + c[3] * soc 
         + c[4];
}

// =========================================
// EKF估算主函数
// =========================================
float EKF_EstimateSOC(EKF_State* S, float Uk_obs, float Ik_obs, float dt) {
    // --- 状态预测 ---
    const float alpha = expf(-dt / (EKF_R1 * EKF_C1));
    const float beta = EKF_R1 * (1.0f - alpha);
    
    // 更新状态转移矩阵Ak
    Ak_data[0] = alpha; Ak_data[1] = 0.0f;
    Ak_data[2] = 0.0f;  Ak_data[3] = 1.0f;
    
    // 预测极化电压和SOC
    S->Xk_data[0] = alpha * S->Xk_data[0] + beta * Ik_obs;
    S->Xk_data[1] += (EKF_ETA * dt / EKF_C_N) * Ik_obs;
    S->Xk_data[1] = fmaxf(0.0f, fminf(1.0f, S->Xk_data[1]));  // 限制SOC在[0,1]范围
    
    // --- 协方差预测 ---
    arm_mat_mult_f32(&S->Ak, &S->P, &S->temp_2x2);      // temp = Ak * P
    arm_mat_trans_f32(&S->Ak, &S->temp_2x2);            // Ak^T
    arm_mat_mult_f32(&S->temp_2x2, &S->Ak, &S->P_pred); // P_pred = Ak * P * Ak^T
    
    // 添加过程噪声（Wk设为对角阵）
    P_pred_data[0] += EKF_Q * 0.001f * 0.001f;  // Wk[0]^2 * Q
    P_pred_data[3] += EKF_Q * 0.001f * 0.001f;  // Wk[1]^2 * Q
    
    // --- 观测更新 ---
    // 计算观测矩阵Ck
    Ck_data[0] = 1.0f;
    Ck_data[1] = calculate_docv_dsoc(S->Xk_data[1]);
    
    // 计算卡尔曼增益 Kk = P_pred * Ck^T / (Ck*P_pred*Ck^T + R)
    arm_mat_trans_f32(&S->Ck, &S->temp_2x1);           // Ck^T (2x1)
    arm_mat_mult_f32(&S->P_pred, &S->temp_2x1, &S->Kk);// Kk = P_pred * Ck^T
    
    float32_t denominator;
    arm_dot_prod_f32(Ck_data, S->P_pred.pData, 2, &denominator); // Ck*P_pred*Ck^T
    denominator += EKF_R;
    
    // 标量除法修正点
    arm_scale_f32(S->Kk.pData, 1.0f/denominator, S->Kk.pData, 2);
    
    // --- 状态更新 ---
    const float ocv = calculate_ocv(S->Xk_data[1]);
    const float residual = (Uk_obs - EKF_R0 * Ik_obs - ocv) - S->Xk_data[0];
    
    S->Xk_data[0] += S->Kk.pData[0] * residual;
    S->Xk_data[1] += S->Kk.pData[1] * residual;
    S->Xk_data[1] = fmaxf(0.0f, fminf(1.0f, S->Xk_data[1]));  // 再次限制SOC范围
    
    // --- 协方差更新 ---
    arm_mat_mult_f32(&S->Kk, &S->Ck, &S->temp_2x2);    // Kk*Ck (2x2)
    arm_mat_sub_f32(&S->P_pred, &S->temp_2x2, &S->P);  // P = P_pred - Kk*Ck*P_pred
    
    return S->Xk_data[1];
}
