#include "simulated_SPI.h"
#include "delay.h"
#include "stm32f10x.h"

//初始化SPI
void Simulated_SPI_IoInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //CS引脚初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;           //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //SCK和MOSI引脚初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;           //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //MISO引脚初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;               //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_CS_1;
    SPI_SCK_1;
}

//SPI可以同时读取和写入数据，因此一个函数即可满足要求
uint8_t SPI2_ReadWriteByte(uint8_t txData)
{
    uint8_t i;
    uint8_t rxData = 0;

    for(i = 0; i < 8; i++)
    {
        SPI_SCK_0;
        delay_us(1);
        //数据发送
        if(txData & 0x80){
            SPI_MOSI_1;
        }else{
            SPI_MOSI_0;
        }
        txData <<= 1;
        delay_us(1);

        SPI_SCK_1;
        delay_us(1);
        //数据接收
        rxData <<= 1;
        if(SPI_READ_MISO){
            rxData |= 0x01;
        }
        delay_us(1);
    }
    SPI_SCK_0;

    return rxData;
}

uint8_t SPI_ReadByte(void)
{
    return SPI2_ReadWriteByte(Dummy_Byte);
}

void SPI_WriteByte(uint8_t txData)
{
    (void)SPI2_ReadWriteByte(txData);
}





