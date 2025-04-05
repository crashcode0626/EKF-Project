#ifndef EKF_H
#define EKF_H

#include <stdint.h>
#include "arm_math.h"

typedef struct {
    // ���ģ�Ͳ���
    float R0;       // ŷķ���� (Ohm)
    float R1;       // RC������� (Ohm)
    float C1;       // RC������� (F)
    float Eta;      // ����Ч��
    float C_N;      // ��ض���� (����)
    
    // EKF����
    arm_matrix_instance_f32 P;     // ���Э������� (2x2)
    arm_matrix_instance_f32 Ak;    // ״̬ת�ƾ��� (2x2)
    arm_matrix_instance_f32 Ck;    // �۲���� (1x2)
    arm_matrix_instance_f32 Kk;    // ���������� (2x1)
    
    float32_t Q;        // ��������Э����
    float32_t R;        // �۲�����Э����
    float32_t Wk_data[2]; // ��������Ȩ��
    float32_t Xk_data[2]; // ״̬���� [������ѹ, SOC]
    
    // ��������
    arm_matrix_instance_f32 P_pred;
    arm_matrix_instance_f32 temp_2x2;
    arm_matrix_instance_f32 temp_2x1;
} EKF_State;

void EKF_Init(EKF_State* S);
float EKF_EstimateSOC(EKF_State* S, float Uk_obs, float Ik_obs, float dt);

#endif