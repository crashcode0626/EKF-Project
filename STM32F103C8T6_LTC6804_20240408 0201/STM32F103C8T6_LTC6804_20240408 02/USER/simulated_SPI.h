#ifndef __SIMULATED_SPI_H
#define __SIMULATED_SPI_H
//#include "sys.h"
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//SPI驱动 代码	   
//修改日期:2021/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 小高霸气 2021-09-02
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

#define SPI_CS_1    GPIO_SetBits(GPIOB, GPIO_Pin_12)            /* SCK = 1 */
#define SPI_CS_0    GPIO_ResetBits(GPIOB, GPIO_Pin_12)        /* SCK = 0 */

#define SPI_SCK_1    GPIO_SetBits(GPIOB, GPIO_Pin_13)            /* SCK = 1 */
#define SPI_SCK_0    GPIO_ResetBits(GPIOB, GPIO_Pin_13)        /* SCK = 0 */

#define SPI_MOSI_1    GPIO_SetBits(GPIOB, GPIO_Pin_15)            /* MOSI = 1 */
#define SPI_MOSI_0    GPIO_ResetBits(GPIOB, GPIO_Pin_15)        /* MOSI = 0 */

#define SPI_READ_MISO    GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)    /* 读MISO口线状态 */

#define Dummy_Byte    0xFF    //读取时MISO发送的数据，可以为任意数据		

void Simulated_SPI_IoInit(void);
uint8_t SPI2_ReadWriteByte(uint8_t txData);
uint8_t SPI_ReadByte(void);
void SPI_WriteByte(uint8_t txData);
#endif


