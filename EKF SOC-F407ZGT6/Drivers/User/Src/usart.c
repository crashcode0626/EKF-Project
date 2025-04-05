#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

UART_HandleTypeDef huart1; 
UART_HandleTypeDef huart2;

uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxIndex = 0;
volatile uint8_t rxComplete = 0;

/*************************************************************************************************
*	�� �� ��:	HAL_UART_MspInit
*	��ڲ���:	huart - UART_HandleTypeDef����ı���������ʾ����Ĵ���
*	�� �� ֵ:��
*	��������:	��ʼ����������
*	˵    ��:��		
*************************************************************************************************/

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();     // ���� USART1 ʱ��

        GPIO_USART1_TX_CLK_ENABLE();       // ���� USART1 TX ���ŵ� GPIO ʱ��
        GPIO_USART1_RX_CLK_ENABLE();       // ���� USART1 RX ���ŵ� GPIO ʱ��

        GPIO_InitStruct.Pin           = USART1_TX_PIN;                  // TX ����
        GPIO_InitStruct.Mode          = GPIO_MODE_AF_PP;                // �����������
        GPIO_InitStruct.Pull          = GPIO_PULLUP;                    // ����
        GPIO_InitStruct.Speed         = GPIO_SPEED_FREQ_VERY_HIGH;     // �ٶȵȼ� 100M
        GPIO_InitStruct.Alternate     = GPIO_AF7_USART1;               // ����ΪUSART1
        HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin           = USART1_RX_PIN;                  // RX ����
        HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);
    }
    else if(huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();     // ���� USART2 ʱ��

        GPIO_USART2_TX_CLK_ENABLE();       // ���� USART2 TX ���ŵ� GPIO ʱ��
        GPIO_USART2_RX_CLK_ENABLE();       // ���� USART2 RX ���ŵ� GPIO ʱ��

        GPIO_InitStruct.Pin           = USART2_TX_PIN;                  // TX ����
        GPIO_InitStruct.Mode          = GPIO_MODE_AF_PP;                // �����������
        GPIO_InitStruct.Pull          = GPIO_PULLUP;                    // ����
        GPIO_InitStruct.Speed         = GPIO_SPEED_FREQ_VERY_HIGH;     // �ٶȵȼ� 100M
        GPIO_InitStruct.Alternate     = GPIO_AF7_USART2;               // ����ΪUSART2
        HAL_GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin           = USART2_RX_PIN;                  // RX ����
        HAL_GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);
				// ����USART2�ж�
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
				
    }
}

/*************************************************************************************************
*	�� �� ��:	USART1_Init
*	��ڲ���:	��
*	�� �� ֵ:��
*	��������:	��ʼ��USART1����
*	˵    ��:��		 
*************************************************************************************************/

void USART1_Init(void)
{
    huart1.Instance = USART1;                        // USART1
    huart1.Init.BaudRate = USART1_BaudRate;          // ���ò�����
    huart1.Init.WordLength = UART_WORDLENGTH_8B;     // ���ݿ�� 8 λ
    huart1.Init.StopBits = UART_STOPBITS_1;           // ֹͣλ 1
    huart1.Init.Parity = UART_PARITY_NONE;            // ��У��
    huart1.Init.Mode = UART_MODE_TX_RX;               // ���ͺͽ���ģʽ
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;      // ������Ӳ��������
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;  // 16 ��������

    if (HAL_UART_Init(&huart1) != HAL_OK)            // ��ʼ�� USART1
    {
        // ������
    }
}

/*************************************************************************************************
*	�� �� ��:	MX_USART2_UART_Init
*	��ڲ���:	��
*	�� �� ֵ:��
*	��������:	��ʼ��USART2����
*	˵    ��:��		 
*************************************************************************************************/

// USART2��ʼ����������жϽ���ʹ��
void USART2_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    
    // ���ý����ж�
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void USART2_IRQHandler(void)
{
    // ֻ���RXNE��־��������HAL���ͨ�ô���
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
    {
        uint8_t data = (uint8_t)(huart2.Instance->DR & 0xFF);
        
        /* ���Ľ����߼� */
        if(rxIndex < RX_BUFFER_SIZE - 1)  // ��ֹ���
        {
            // ��������1���յ����з�
            if(data == '\n')
            {
                rxBuffer[rxIndex] = '\0';  // �ַ�����ֹ
                rxComplete = 1;            // ��λ��ɱ�־
                rxIndex = 0;               // ��������
            }
            // ��������2������������
            else if(rxIndex == RX_BUFFER_SIZE - 2)
            {
                rxBuffer[rxIndex] = '\0';
                rxComplete = 1;
                rxIndex = 0;
            }
            // ��Ч���ݴ洢�����Իس�����
            else if(data != '\r')
            {
                rxBuffer[rxIndex++] = data;
            }
        }
        else  // �������������
        {
            rxIndex = 0;
            rxBuffer[0] = '\0';
        }
    }
}

/*************************************************************************************************
*	�� �� ��:	uart_printf
*	��ڲ���:	huart - UART���, fmt - ��ʽ���ַ���
*	�� �� ֵ:	��ӡ���ַ�����
*	��������:	ͨ��UART���͸�ʽ������
*	˵    ��:	֧�ֶ����������
*************************************************************************************************/

int uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len > 0)
    {
        HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    }
    return len;
}

/*************************************************************************************************
*	�� �� ��:	uart_scanf
*	��ڲ���:	huart - UART���, fmt - ��ʽ���ַ���
*	�� �� ֵ:	���յ����ַ�����
*	��������:	ͨ��UART���ո�ʽ������
*	˵    ��:	֧�ֶ����������
*************************************************************************************************/

int uart_scanf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    int len = 0;
    uint8_t charReceived;
    int index = 0;

    // ��ջ�����
    memset(buffer, 0, sizeof(buffer));

    // ���ֽڽ��գ�ֱ�������������߳�ʱ
    while (index < sizeof(buffer) - 1)
    {
        if (HAL_UART_Receive(huart, &charReceived, 1, 100) == HAL_OK)  // ����100ms��ʱ
        {
            // ͨ�����յ����ַ����ж��Ƿ��������
            if (charReceived == '\r' || charReceived == '\n')  // ���Իس��ͻ��з�
            {
                if (index == 0) continue;  // ��������
                break;                    // ��Ч�������ֹ
            }

            buffer[index++] = charReceived;
        }
    }

    if (index == 0) return 0;  // ����Ч����

    buffer[index] = '\0';      // ��ֹ�ַ���

    va_start(args, fmt);
    len = vsscanf(buffer, fmt, args);
    va_end(args);

    return len;
}

/* ------------------ͨ���ض���printf����ӳ�䵽����1��-------------------*/
#if !defined(__MICROLIB)

//#pragma import(__use_no_semihosting)
__asm (".global __use_no_semihosting\n\t");
void _sys_exit(int x) //����ʹ�ð�����ģʽ
{
  x = x;
}
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
    ch = ch;
}
//struct __FILE
//{
//  int handle;
//};
FILE __stdout;

#endif

#if defined ( __GNUC__ ) && !defined (__clang__) 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
  /* ʵ�ִ��ڷ���һ���ֽ����ݵĺ��� */
  //serial_write(&serial1, (uint8_t)ch); //����һ���Լ������ݵ�����
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
  return ch;
}
