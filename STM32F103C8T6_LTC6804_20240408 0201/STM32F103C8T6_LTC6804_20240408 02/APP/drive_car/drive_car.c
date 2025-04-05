#include "stm32f10x.h"
#include "timer.h"
#include "sys.h" 
#include "delay.h"
#include "led.h"
#include "motor.h"
#include "24l01.h" 
#include "drive_car.h"
#include "usart.h"
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
u8 tmp_buf[33];	

static   u16  left_speed = 500,right_speed = 500,temp_L_R = 0,temp_U_D = 0,temp_H_X = 0,temp_Y_M = 0;

	//检验和
uint8_t RX_CheckSum(uint8_t *buf, uint8_t len) //buf为数组，len为数组长度
{ 
    uint8_t i, ret = 0;
 
    for(i=0; i<len; i++)
    {
        ret += *(buf++);
    }

    return ret;
}
	
//前进后退处理
uint8_t drive_forback(u16 forback)
{
	uint8_t ret = 0;
	if((forback >= 1530)&(forback <= 1535))    // 1530~1535  停止状态
	{
		ret = 0;
	}
	else if(forback>1535)     //>1535
	{
		ret = 1;
	}
	else                      //<1535
	{
		ret = 2;
	}
	return ret;
}
//油门处理
void drive_handle(u16 power,u16 forback,u16 leftright)
{
  u8 temp = 0;
	temp = drive_forback(forback);   //前后方向
	if(power  <=1050)        //停止
	{
		brake();   //油门
	}

	else  //1050~2000   
	{
		left_speed = (power -1000)*2.5;
		right_speed = (power -1000)*2.5;
		if(left_speed>=899)left_speed = 899;
		if(right_speed>=899)right_speed = 899;
		
		switch(temp)       //前进后退方向
		{
			case 0:    //停止
			{
				brake();   //刹车
				break;
			}
			case 1:    //前进
			{
				forward(left_speed ,right_speed);                  //前进
				break;
			}
			case 2:    //后退
			{
				backward(left_speed ,right_speed);                 //后腿
				break;
			}
			default:printf("data error");break;
		}
		
		
		
		

	}
}



//方向处理


void drive_car (void)
{
	u8 temp=0;
	if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
	{
		LED0=0;
		temp = RX_CheckSum(tmp_buf, 31);    //校验值
		if(temp == tmp_buf[31])   
		{
			temp_L_R = (tmp_buf[4]<<8)|tmp_buf[5];    //970~1990       1481    左右
			temp_U_D = (tmp_buf[6]<<8)|tmp_buf[7];    //1022~2042      1532    前后
			temp_Y_M = (tmp_buf[8]<<8)|tmp_buf[9];    //1000~2020              油门
			temp_H_X = (tmp_buf[10]<<8)|tmp_buf[11];  //976~1996       1497    航向
			
			drive_handle(temp_Y_M,temp_U_D,temp_L_R);   //驾驶处理
			
			
			
		}
		else
		{
			printf("校验失败\r\n ");
		}
		
	}
	else
	{
		LED0=1;		
	}	
}
	







