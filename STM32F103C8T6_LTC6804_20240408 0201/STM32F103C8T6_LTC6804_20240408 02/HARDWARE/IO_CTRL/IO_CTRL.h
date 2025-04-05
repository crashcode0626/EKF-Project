#ifndef __IO_CTRL_H
#define __IO_CTRL_H

#include "stm32f10x.h"


#define GPIOB_RCC                 RCC_APB2Periph_GPIOB
#define GPIOB_PORT                GPIOB
#define CHG_POWER_EN              GPIO_Pin_3
#define CHG_POWER_EN_ONOFF(x)     GPIO_WriteBit(GPIOB_PORT ,CHG_POWER_EN,x);

#define GPIOB_RCC                 RCC_APB2Periph_GPIOB
#define GPIOB_PORT                GPIOB
#define DSG_POWER_EN              GPIO_Pin_4
#define DSG_POWER_EN_ONOFF(x)     GPIO_WriteBit(GPIOB_PORT ,DSG_POWER_EN,x);


void IO_CTRL_Config(void);

void Only_Open_CHG(void);
void Only_Close_CHG(void);
void Only_Open_DSG(void);
void Only_Close_DSG(void);	
void Open_DSG_CHG(void);
void Close_DSG_CHG(void);	
#endif
