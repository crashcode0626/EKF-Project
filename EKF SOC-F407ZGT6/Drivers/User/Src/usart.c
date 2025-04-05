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
*	函 数 名:	HAL_UART_MspInit
*	入口参数:	huart - UART_HandleTypeDef定义的变量，即表示定义的串口
*	返 回 值:无
*	函数功能:	初始化串口引脚
*	说    明:无		
*************************************************************************************************/

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();     // 开启 USART1 时钟

        GPIO_USART1_TX_CLK_ENABLE();       // 启用 USART1 TX 引脚的 GPIO 时钟
        GPIO_USART1_RX_CLK_ENABLE();       // 启用 USART1 RX 引脚的 GPIO 时钟

        GPIO_InitStruct.Pin           = USART1_TX_PIN;                  // TX 引脚
        GPIO_InitStruct.Mode          = GPIO_MODE_AF_PP;                // 复用推挽输出
        GPIO_InitStruct.Pull          = GPIO_PULLUP;                    // 上拉
        GPIO_InitStruct.Speed         = GPIO_SPEED_FREQ_VERY_HIGH;     // 速度等级 100M
        GPIO_InitStruct.Alternate     = GPIO_AF7_USART1;               // 复用为USART1
        HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin           = USART1_RX_PIN;                  // RX 引脚
        HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);
    }
    else if(huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();     // 开启 USART2 时钟

        GPIO_USART2_TX_CLK_ENABLE();       // 启用 USART2 TX 引脚的 GPIO 时钟
        GPIO_USART2_RX_CLK_ENABLE();       // 启用 USART2 RX 引脚的 GPIO 时钟

        GPIO_InitStruct.Pin           = USART2_TX_PIN;                  // TX 引脚
        GPIO_InitStruct.Mode          = GPIO_MODE_AF_PP;                // 复用推挽输出
        GPIO_InitStruct.Pull          = GPIO_PULLUP;                    // 上拉
        GPIO_InitStruct.Speed         = GPIO_SPEED_FREQ_VERY_HIGH;     // 速度等级 100M
        GPIO_InitStruct.Alternate     = GPIO_AF7_USART2;               // 复用为USART2
        HAL_GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin           = USART2_RX_PIN;                  // RX 引脚
        HAL_GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);
				// 配置USART2中断
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
				
    }
}

/*************************************************************************************************
*	函 数 名:	USART1_Init
*	入口参数:	无
*	返 回 值:无
*	函数功能:	初始化USART1配置
*	说    明:无		 
*************************************************************************************************/

void USART1_Init(void)
{
    huart1.Instance = USART1;                        // USART1
    huart1.Init.BaudRate = USART1_BaudRate;          // 设置波特率
    huart1.Init.WordLength = UART_WORDLENGTH_8B;     // 数据宽度 8 位
    huart1.Init.StopBits = UART_STOPBITS_1;           // 停止位 1
    huart1.Init.Parity = UART_PARITY_NONE;            // 无校验
    huart1.Init.Mode = UART_MODE_TX_RX;               // 发送和接收模式
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;      // 不启用硬件流控制
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;  // 16 倍过采样

    if (HAL_UART_Init(&huart1) != HAL_OK)            // 初始化 USART1
    {
        // 错误处理
    }
}

/*************************************************************************************************
*	函 数 名:	MX_USART2_UART_Init
*	入口参数:	无
*	返 回 值:无
*	函数功能:	初始化USART2配置
*	说    明:无		 
*************************************************************************************************/

// USART2初始化函数添加中断接收使能
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
    
    // 启用接收中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void USART2_IRQHandler(void)
{
    // 只检查RXNE标志，不依赖HAL库的通用处理
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
    {
        uint8_t data = (uint8_t)(huart2.Instance->DR & 0xFF);
        
        /* 核心接收逻辑 */
        if(rxIndex < RX_BUFFER_SIZE - 1)  // 防止溢出
        {
            // 结束条件1：收到换行符
            if(data == '\n')
            {
                rxBuffer[rxIndex] = '\0';  // 字符串终止
                rxComplete = 1;            // 置位完成标志
                rxIndex = 0;               // 重置索引
            }
            // 结束条件2：缓冲区将满
            else if(rxIndex == RX_BUFFER_SIZE - 2)
            {
                rxBuffer[rxIndex] = '\0';
                rxComplete = 1;
                rxIndex = 0;
            }
            // 有效数据存储（忽略回车符）
            else if(data != '\r')
            {
                rxBuffer[rxIndex++] = data;
            }
        }
        else  // 缓冲区溢出处理
        {
            rxIndex = 0;
            rxBuffer[0] = '\0';
        }
    }
}

/*************************************************************************************************
*	函 数 名:	uart_printf
*	入口参数:	huart - UART句柄, fmt - 格式化字符串
*	返 回 值:	打印的字符个数
*	函数功能:	通过UART发送格式化数据
*	说    明:	支持多个参数传入
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
*	函 数 名:	uart_scanf
*	入口参数:	huart - UART句柄, fmt - 格式化字符串
*	返 回 值:	接收到的字符个数
*	函数功能:	通过UART接收格式化数据
*	说    明:	支持多个参数传入
*************************************************************************************************/

int uart_scanf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    int len = 0;
    uint8_t charReceived;
    int index = 0;

    // 清空缓冲区
    memset(buffer, 0, sizeof(buffer));

    // 逐字节接收，直到缓冲区满或者超时
    while (index < sizeof(buffer) - 1)
    {
        if (HAL_UART_Receive(huart, &charReceived, 1, 100) == HAL_OK)  // 设置100ms超时
        {
            // 通过接收到的字符来判断是否继续接收
            if (charReceived == '\r' || charReceived == '\n')  // 忽略回车和换行符
            {
                if (index == 0) continue;  // 跳过空行
                break;                    // 有效输入后终止
            }

            buffer[index++] = charReceived;
        }
    }

    if (index == 0) return 0;  // 无有效输入

    buffer[index] = '\0';      // 终止字符串

    va_start(args, fmt);
    len = vsscanf(buffer, fmt, args);
    va_end(args);

    return len;
}

/* ------------------通过重定向将printf函数映射到串口1上-------------------*/
#if !defined(__MICROLIB)

//#pragma import(__use_no_semihosting)
__asm (".global __use_no_semihosting\n\t");
void _sys_exit(int x) //避免使用半主机模式
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
  /* 实现串口发送一个字节数据的函数 */
  //serial_write(&serial1, (uint8_t)ch); //发送一个自己的数据到串口
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
  return ch;
}
