#ifndef __DRIVE_CAR_H
#define __DRIVE_CAR_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK��ӢSTM32������
//������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 
uint8_t RX_CheckSum(uint8_t *buf, uint8_t len); //bufΪ���飬lenΪ���鳤��
void drive_car (void);
void drive_handle(u16 power,u16 forback,u16 leftright);
				    
#endif
