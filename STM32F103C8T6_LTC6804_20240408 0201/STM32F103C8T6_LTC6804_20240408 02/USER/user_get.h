#ifndef USER_GET_H
#define USER_GET_H

#include "sys.h"
#include "LTC6804-1.h"


#define cell_zu  1

void Get_Update_ALL_Data(void);       //�����ѹ�ܺ�
void Get_Cell_Voltage_Max_Min(void);  //���������С��ص�ѹ
void Get_SOC(void);                   //����SOC(���ݵ�ѹ)
void Get_Cell_Voltage(void);          //�����ص�ѹ
void Get_BQ_Current(void);            //�������
void  Balance_task(uint16_t mv) ;     //�����ĸ������Ҫ����
//void init_gpio(void);               //spi --> sck (PB13�˿ڳ�ʼ��)
uint16_t adow_test(void);             //���߼��
//void Balance_Bat(uint16_t mask);               //����
#endif


