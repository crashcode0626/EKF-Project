/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��IO_CTRL.c
 * ����    ��IO����������     
 * ����    ��zhuoyingxingyu
 * �Ա�    ��Դ�ع�����http://vcc-gnd.taobao.com/
 * ��̳��ַ������԰��-Ƕ��ʽ������̳http://vcc-gnd.com/
 * �汾����: 2016-04-08
 * Ӳ������: D1->PC13;D2->PB0;D3->PB1
 * ���Է�ʽ��J-Link-OB
**********************************************************************************/	

//ͷ�ļ�
#include "IO_CTRL.h"

 /**
  * @file   GPIO_Config
  * @brief  IO����������
  * @param  ��
  * @retval ��
  */
void IO_CTRL_Config(void)
{	
    //����һ��GPIO_InitTypeDef ���͵Ľṹ��
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//ʹ��GPIO������ʱ��
	
	
	//��GPIO��ʱ�ӣ��ȴ򿪸��ò����޸��Ƿ�ͣ�ø��ù���
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
//�ر�JTAG��ʹ��SWD
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	/*D3*/
	GPIO_InitStructure.GPIO_Pin =CHG_POWER_EN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB_PORT, &GPIO_InitStructure);
	CHG_POWER_EN_ONOFF(1);
	
	GPIO_InitStructure.GPIO_Pin =  DSG_POWER_EN  ;//ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //��������ģʽΪ�������ģʽ						 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//���������ٶ�Ϊ50MHZ         
	GPIO_Init(GPIOB_PORT, &GPIO_InitStructure);//���ÿ⺯������ʼ��GPIO
	DSG_POWER_EN_ONOFF(1);

}
void Only_Open_CHG(void)
{
	CHG_POWER_EN_ONOFF(0);
}
void Only_Close_CHG(void)
{
	CHG_POWER_EN_ONOFF(1);
}
void Only_Open_DSG(void)
{
	DSG_POWER_EN_ONOFF(0);
}
void Only_Close_DSG(void)
{
	DSG_POWER_EN_ONOFF(1);
}
void Open_DSG_CHG(void)
{
	CHG_POWER_EN_ONOFF(0);
	DSG_POWER_EN_ONOFF(0);
}
void Close_DSG_CHG(void)
{
	CHG_POWER_EN_ONOFF(1);
	DSG_POWER_EN_ONOFF(1);
}

