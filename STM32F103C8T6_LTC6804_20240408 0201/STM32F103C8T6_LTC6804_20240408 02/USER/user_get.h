#ifndef USER_GET_H
#define USER_GET_H

#include "sys.h"
#include "LTC6804-1.h"


#define cell_zu  1

void Get_Update_ALL_Data(void);       //计算电压总和
void Get_Cell_Voltage_Max_Min(void);  //计算最大最小电池电压
void Get_SOC(void);                   //计算SOC(根据电压)
void Get_Cell_Voltage(void);          //计算电池电压
void Get_BQ_Current(void);            //计算电流
void  Balance_task(uint16_t mv) ;     //计算哪个电池需要均衡
//void init_gpio(void);               //spi --> sck (PB13端口初始化)
uint16_t adow_test(void);             //断线检测
//void Balance_Bat(uint16_t mask);               //均衡
#endif


