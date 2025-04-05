/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：IO_CTRL.c
 * 描述    ：IO口引脚配置     
 * 作者    ：zhuoyingxingyu
 * 淘宝    ：源地工作室http://vcc-gnd.taobao.com/
 * 论坛地址：极客园地-嵌入式开发论坛http://vcc-gnd.com/
 * 版本更新: 2016-04-08
 * 硬件连接: D1->PC13;D2->PB0;D3->PB1
 * 调试方式：J-Link-OB
**********************************************************************************/	

//头文件
#include "IO_CTRL.h"

 /**
  * @file   GPIO_Config
  * @brief  IO口引脚配置
  * @param  无
  * @retval 无
  */
void IO_CTRL_Config(void)
{	
    //定义一个GPIO_InitTypeDef 类型的结构体
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能GPIO的外设时钟
	
	
	//打开GPIO口时钟，先打开复用才能修改是否停用复用功能
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
//关闭JTAG，使能SWD
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	/*D3*/
	GPIO_InitStructure.GPIO_Pin =CHG_POWER_EN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB_PORT, &GPIO_InitStructure);
	CHG_POWER_EN_ONOFF(1);
	
	GPIO_InitStructure.GPIO_Pin =  DSG_POWER_EN  ;//选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //设置引脚模式为推免输出模式						 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设置引脚速度为50MHZ         
	GPIO_Init(GPIOB_PORT, &GPIO_InitStructure);//调用库函数，初始化GPIO
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

