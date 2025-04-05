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
u8 tmp_buf[33];	

static   u16  left_speed = 500,right_speed = 500,temp_L_R = 0,temp_U_D = 0,temp_H_X = 0,temp_Y_M = 0;

	//�����
uint8_t RX_CheckSum(uint8_t *buf, uint8_t len) //bufΪ���飬lenΪ���鳤��
{ 
    uint8_t i, ret = 0;
 
    for(i=0; i<len; i++)
    {
        ret += *(buf++);
    }

    return ret;
}
	
//ǰ�����˴���
uint8_t drive_forback(u16 forback)
{
	uint8_t ret = 0;
	if((forback >= 1530)&(forback <= 1535))    // 1530~1535  ֹͣ״̬
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
//���Ŵ���
void drive_handle(u16 power,u16 forback,u16 leftright)
{
  u8 temp = 0;
	temp = drive_forback(forback);   //ǰ����
	if(power  <=1050)        //ֹͣ
	{
		brake();   //����
	}

	else  //1050~2000   
	{
		left_speed = (power -1000)*2.5;
		right_speed = (power -1000)*2.5;
		if(left_speed>=899)left_speed = 899;
		if(right_speed>=899)right_speed = 899;
		
		switch(temp)       //ǰ�����˷���
		{
			case 0:    //ֹͣ
			{
				brake();   //ɲ��
				break;
			}
			case 1:    //ǰ��
			{
				forward(left_speed ,right_speed);                  //ǰ��
				break;
			}
			case 2:    //����
			{
				backward(left_speed ,right_speed);                 //����
				break;
			}
			default:printf("data error");break;
		}
		
		
		
		

	}
}



//������


void drive_car (void)
{
	u8 temp=0;
	if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
	{
		LED0=0;
		temp = RX_CheckSum(tmp_buf, 31);    //У��ֵ
		if(temp == tmp_buf[31])   
		{
			temp_L_R = (tmp_buf[4]<<8)|tmp_buf[5];    //970~1990       1481    ����
			temp_U_D = (tmp_buf[6]<<8)|tmp_buf[7];    //1022~2042      1532    ǰ��
			temp_Y_M = (tmp_buf[8]<<8)|tmp_buf[9];    //1000~2020              ����
			temp_H_X = (tmp_buf[10]<<8)|tmp_buf[11];  //976~1996       1497    ����
			
			drive_handle(temp_Y_M,temp_U_D,temp_L_R);   //��ʻ����
			
			
			
		}
		else
		{
			printf("У��ʧ��\r\n ");
		}
		
	}
	else
	{
		LED0=1;		
	}	
}
	







