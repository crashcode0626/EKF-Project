#include "stm32f10x.h"
#include "timer.h"
#include "sys.h" 
#include "delay.h"

#include "motor.h"

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


void forward(u16 left_speed ,u16 right_speed)
{
	TIM_SetCompare1(TIM3,left_speed);
	TIM_SetCompare2(TIM3,0);
	TIM_SetCompare3(TIM3,0);
	TIM_SetCompare4(TIM3,right_speed);
}

void backward(u16 left_speed ,u16 right_speed)
{
	TIM_SetCompare1(TIM3,0);
	TIM_SetCompare2(TIM3,left_speed);
	TIM_SetCompare3(TIM3,right_speed);
	TIM_SetCompare4(TIM3,0);
}



void brake (void)   //ɲ��
{
	TIM_SetCompare1(TIM3,899);
	TIM_SetCompare2(TIM3,899);
	TIM_SetCompare3(TIM3,899);
	TIM_SetCompare4(TIM3,899);
}
