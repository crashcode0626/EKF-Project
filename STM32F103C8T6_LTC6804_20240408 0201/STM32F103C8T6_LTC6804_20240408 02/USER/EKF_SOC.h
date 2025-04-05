#ifndef EKF_SOC_H
#define EKF_SOC_H

#include "arm_math.h"
#include "stm32f10x.h"

// 电池模型参数
#define R0  0.0096f   
#define R1  0.0071f  
#define C1  50.0f      
#define Eta 0.97f     
#define C_N (2.9f * 3600.0f) 

// **全局变量声明，避免重复定义**
extern float Q, R;  // **在头文件中只声明，不定义**
extern arm_matrix_instance_f32 P, Wk, Vk;

// SOC 计算函数
void EKF_SOC_Init(void);
void EKF_SOC_Update(float voltage_mv, float current, uint32_t dt_us, float *SOC_out);
void soc_update(void);

#endif // EKF_SOC_H
