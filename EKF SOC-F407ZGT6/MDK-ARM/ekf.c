#include "ekf.h"
#include <math.h>

// 静态内存分配（避免动态内存）
static float32_t P_data[4] = {1e-8f, 0, 0, 1e-6f};
static float32_t Ak_data[4], Ck_data[2], Kk_data[2];
static float32_t P_pred_data[4], temp_2x2_data[4], temp_2x1_data[2];

// 多项式系数
static const float32_t ocv_coeff[6] = {1.6026f, -8.2896f, 14.8878f, -12.0873f, 5.5781f, 2.5047f};

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
    S->Xk_data[1] = 1.0f;    // 初始SOC=100%
}

static float32_t calculate_ocv(float32_t soc) {
    // 使用霍纳法计算OCV（ARM优化多项式计算）
    return arm_poly_f32(soc, ocv_coeff, 5);
}

static float32_t calculate_docv_dsoc(float32_t soc) {
    // 导数多项式系数: 8.013*soc^4 -33.1584*soc^3 +44.6634*soc^2 -24.1746*soc +5.5781
    const float32_t coeff[5] = {8.013f, -33.1584f, 44.6634f, -24.1746f, 5.5781f};
    return arm_poly_f32(soc, coeff, 4);
}

float EKF_EstimateSOC(EKF_State* S, float Uk_obs, float Ik_obs, float dt) {
    //--- 状态预测 ---
    const float alpha = expf(-dt / (S->R1 * S->C1));
    const float beta = S->R1 * (1.0f - alpha);
    
    // 更新状态转移矩阵Ak
    Ak_data[0] = alpha; Ak_data[1] = 0;
    Ak_data[2] = 0;     Ak_data[3] = 1.0f;
    
    // 状态预测
    S->Xk_data[0] = alpha * S->Xk_data[0] + beta * Ik_obs;
    S->Xk_data[1] += (S->Eta * dt / S->C_N) * Ik_obs;
    
    //--- 协方差预测 (P_pred = Ak*P*Ak^T + Wk*Q*Wk^T) ---
    arm_mat_mult_f32(&S->Ak, &S->P, &S->temp_2x2);      // temp = Ak * P
    arm_mat_trans_f32(&S->Ak, &S->temp_2x2);            // Ak^T
    arm_mat_mult_f32(&S->temp_2x2, &S->Ak, &S->P_pred); // P_pred = temp * Ak^T
    
    // 添加过程噪声
    const float WkQ = S->Wk_data[0] * S->Q * S->Wk_data[0]; // 假设Wk为对角阵
    P_pred_data[0] += WkQ;
    P_pred_data[3] += WkQ;
    
    //--- 观测更新 ---
    // 计算观测矩阵Ck
    Ck_data[0] = 1.0f;
    Ck_data[1] = calculate_docv_dsoc(S->Xk_data[1]);
    
    // 计算卡尔曼增益 Kk = P_pred * Ck^T / (Ck*P_pred*Ck^T + R)
    arm_mat_trans_f32(&S->Ck, &S->temp_2x1);           // Ck^T (2x1)
    arm_mat_mult_f32(&S->P_pred, &S->temp_2x1, &S->Kk);// Kk = P_pred * Ck^T
    
    float32_t denominator;
    arm_dot_prod_f32(Ck_data, (float32_t*)&S->P_pred.data[0], 2, &denominator); // Ck*P_pred*Ck^T
    denominator += S->R;
    
    // 标量除法
    arm_scale_f32(S->Kk.data, 1.0f/denominator, S->Kk.data, 2);
    
    //--- 状态更新 ---
    const float ocv = calculate_ocv(S->Xk_data[1]);
    const float Uk_pola = Uk_obs - S->R0*Ik_obs - ocv;
    
    // 计算残差
    const float residual = Uk_pola - S->Xk_data[0];
    
    // 更新状态
    S->Xk_data[0] += S->Kk.data[0] * residual;
    S->Xk_data[1] += S->Kk.data[1] * residual;
    
    //--- 协方差更新 (P = (I - Kk*Ck) * P_pred) ---
    arm_mat_mult_f32(&S->Kk, &S->Ck, &S->temp_2x2);    // Kk*Ck (2x2)
    arm_mat_sub_f32(&S->P_pred, &S->temp_2x2, &S->P);  // P = P_pred - Kk*Ck*P_pred
    
    return S->Xk_data[1];
}