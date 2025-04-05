#include "EKF_SOC.h"
#include "user_get.h"
#include "timer.h"
#include <math.h>

// **变量声明**
extern uint16_t cell_voltage[50];  // 电池电压数组
extern float current;              // 电流

static uint32_t start_time = 0;    // 存储定时器时间

static float Q = 0.001f;      // **用 `static` 限制作用域，避免全局冲突**
static float R = 1000.0f;     // **用 `static` 限制作用域，避免全局冲突**

// **矩阵数据**
float P_data[4] = {1e-8, 0, 0, 1e-6};
float Wk_data[2] = {0.001f, 0.001f};
float Vk_data[2] = {1000.0f, 1000.0f};

arm_matrix_instance_f32 P, Wk, Vk;

// **SOC 估算变量**
static float Uk = 3.7f;
static float SOCk = 0.85f;
static float Ik = 0.0f;
static float Uoc_est = 0.0f;

// **EKF 初始化**
void EKF_SOC_Init(void) {
    arm_mat_init_f32(&P, 2, 2, P_data);
    arm_mat_init_f32(&Wk, 2, 1, Wk_data);
    arm_mat_init_f32(&Vk, 2, 1, Vk_data);

    Uk = 3.7f;
    SOCk = 1.0f;
    Ik = 0.0f;
}

// **SOC 计算**
void EKF_SOC_Update(float voltage_mv, float current, uint32_t dt_us, float *SOC_out) {
    float dt = (dt_us < 1000) ? 0.001f : (dt_us / 1e6f);
    float voltage = voltage_mv / 1000.0f;

    float exp_factor = expf(-dt / (R1 * C1));
    float Uk_pred = exp_factor * Uk + R1 * (1 - exp_factor) * Ik;
    float SOCk_pred = SOCk + (Eta * dt / C_N) * Ik;

    // **误差协方差预测**
    P_data[0] += Q;
    P_data[3] += Q;

    float SOCk2 = SOCk_pred * SOCk_pred;
    float SOCk3 = SOCk2 * SOCk_pred;
    float SOCk4 = SOCk3 * SOCk_pred;
    float SOCk5 = SOCk4 * SOCk_pred;

    float Uk_pola = voltage - R0 * current - (1.6026f * SOCk5 - 8.2896f * SOCk4 + 
                                              14.8878f * SOCk3 - 12.0873f * SOCk2 + 
                                              5.5781f * SOCk_pred + 2.5047f);

    float Ck = 8.013f * SOCk4 - 33.1584f * SOCk3 + 
               44.6634f * SOCk2 - 24.1746f * SOCk_pred + 5.5781f;

    float K0 = P_data[0] / (P_data[0] + R);
    float K1 = P_data[1] / (P_data[1] + R);

    Uk = Uk_pred + K0 * (Uk_pola - Uk_pred);
    SOCk = SOCk_pred + K1 * (SOCk_pred - SOCk_pred);
    Ik = current;

    Uoc_est = Uk + (1.6026f * SOCk5 - 8.2896f * SOCk4 + 14.8878f * SOCk3
                   - 12.0873f * SOCk2 + 5.5781f * SOCk_pred + 2.5047f) + Ik*R0;

    *SOC_out = SOCk;
}

// **SOC 任务**
void soc_update(void) {
    float SOC_result;
    uint32_t current_time, dt_us;

    Get_BQ_Current();
    Get_Cell_Voltage();

    current_time = TIM2_GetCounter();
    dt_us = TIM2_GetElapsedTime(start_time, current_time);
    start_time = current_time;

    EKF_SOC_Update(cell_voltage[0], current, dt_us, &SOC_result);
    cell_voltage[13] = (uint16_t)(SOC_result * 100.0f);
}
