#ifndef __24C02_H
#define __24C02_H 
#include "stm32f10x.h"


void write_6804eeprom(uint8_t total_ic,uint8_t addr,uint8_t data);
void read_6804eeprom(uint8_t total_ic,uint8_t addr,uint8_t data[2]);


#endif

