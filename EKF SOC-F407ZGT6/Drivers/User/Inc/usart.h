#ifndef __USART_H
#define __USART_H

#include "stm32f4xx_hal.h"
#include "stdio.h"

/*-------------------------------------------- USART1���ú� ---------------------------------------*/

#define  USART1_BaudRate  115200

#define  USART1_TX_PIN									GPIO_PIN_9								// TX ����
#define	USART1_TX_PORT									GPIOA										// TX ���Ŷ˿�
#define 	GPIO_USART1_TX_CLK_ENABLE        	   __HAL_RCC_GPIOA_CLK_ENABLE	 	// TX ����ʱ��

#define  USART1_RX_PIN									GPIO_PIN_10             			// RX ����
#define	USART1_RX_PORT									GPIOA                 				// RX ���Ŷ˿�
#define 	GPIO_USART1_RX_CLK_ENABLE         	   __HAL_RCC_GPIOA_CLK_ENABLE		// RX ����ʱ��

/*-------------------------------------------- USART2���ú� ---------------------------------------*/

#define  USART2_BaudRate  115200

#define  USART2_TX_PIN									GPIO_PIN_2								// TX ����
#define	USART2_TX_PORT									GPIOA										// TX ���Ŷ˿�
#define 	GPIO_USART2_TX_CLK_ENABLE        	   __HAL_RCC_GPIOA_CLK_ENABLE	 	// TX ����ʱ��

#define  USART2_RX_PIN									GPIO_PIN_3             			// RX ����
#define	USART2_RX_PORT									GPIOA                 				// RX ���Ŷ˿�
#define 	GPIO_USART2_RX_CLK_ENABLE         	   __HAL_RCC_GPIOA_CLK_ENABLE		// RX ����ʱ��


/*---------------------------------------------- �������� ---------------------------------------*/

void USART1_Init(void);	// USART1��ʼ������
void USART2_Init(void);// USART2��ʼ������

// usart.h
#define RX_BUFFER_SIZE 128  // ���ջ�������С

extern uint8_t rxBuffer[RX_BUFFER_SIZE];  // ���ջ�����
extern volatile uint16_t rxIndex;         // ��������
extern volatile uint8_t rxComplete;       // ������ɱ�־

#endif //__USART_H
