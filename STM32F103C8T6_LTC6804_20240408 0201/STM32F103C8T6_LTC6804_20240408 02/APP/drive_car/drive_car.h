#ifndef __DRIVE_CAR_H
#define __DRIVE_CAR_H	 
#include "sys.h"
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
uint8_t RX_CheckSum(uint8_t *buf, uint8_t len); //buf为数组，len为数组长度
void drive_car (void);
void drive_handle(u16 power,u16 forback,u16 leftright);
				    
#endif
