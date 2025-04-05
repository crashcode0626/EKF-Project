#include "stm32f10x.h"
#include "timer.h"
#include "sys.h" 
#include "delay.h"

#include "motor.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板
//按键驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
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



void brake (void)   //刹车
{
	TIM_SetCompare1(TIM3,899);
	TIM_SetCompare2(TIM3,899);
	TIM_SetCompare3(TIM3,899);
	TIM_SetCompare4(TIM3,899);
}
