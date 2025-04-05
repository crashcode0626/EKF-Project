#ifndef EKF_SOC_H
#define EKF_SOC_H

#include "arm_math.h"
#include "stm32f10x.h"

// ���ģ�Ͳ���
#define R0  0.0096f   
#define R1  0.0071f  
#define C1  50.0f      
#define Eta 0.97f     
#define C_N (2.9f * 3600.0f) 

// **ȫ�ֱ��������������ظ�����**
extern float Q, R;  // **��ͷ�ļ���ֻ������������**
extern arm_matrix_instance_f32 P, Wk, Vk;

// SOC ���㺯��
void EKF_SOC_Init(void);
void EKF_SOC_Update(float voltage_mv, float current, uint32_t dt_us, float *SOC_out);
void soc_update(void);

#endif // EKF_SOC_H
