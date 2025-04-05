#ifndef __SIMULATED_SPI_H
#define __SIMULATED_SPI_H
//#include "sys.h"
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//SPI���� ����	   
//�޸�����:2021/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) С�߰��� 2021-09-02
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

#define SPI_CS_1    GPIO_SetBits(GPIOB, GPIO_Pin_12)            /* SCK = 1 */
#define SPI_CS_0    GPIO_ResetBits(GPIOB, GPIO_Pin_12)        /* SCK = 0 */

#define SPI_SCK_1    GPIO_SetBits(GPIOB, GPIO_Pin_13)            /* SCK = 1 */
#define SPI_SCK_0    GPIO_ResetBits(GPIOB, GPIO_Pin_13)        /* SCK = 0 */

#define SPI_MOSI_1    GPIO_SetBits(GPIOB, GPIO_Pin_15)            /* MOSI = 1 */
#define SPI_MOSI_0    GPIO_ResetBits(GPIOB, GPIO_Pin_15)        /* MOSI = 0 */

#define SPI_READ_MISO    GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)    /* ��MISO����״̬ */

#define Dummy_Byte    0xFF    //��ȡʱMISO���͵����ݣ�����Ϊ��������		

void Simulated_SPI_IoInit(void);
uint8_t SPI2_ReadWriteByte(uint8_t txData);
uint8_t SPI_ReadByte(void);
void SPI_WriteByte(uint8_t txData);
#endif


