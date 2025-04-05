#include "24c02.h"
#include "LTC6804-1.h"
#include "usart.h"


uint8_t comm_config_init[][6]={{0x6a,0x08,0x00,0x08,0x0f,0xf9}};      //��֮ǰ��д����
uint8_t comm_config_rd[][6]={{0x6a,0x18,0x0f,0xf0,0x0f,0xf9}};      //��ȡ����

extern uint8_t r_comm_config[][8]; 

void write_6804eeprom(uint8_t total_ic,uint8_t addr,uint8_t data)
{
	uint16_t res=0,i =0;
	comm_config_init[0][2] = 0x00 | (addr>>4);
	comm_config_init[0][3] = 0x08 | (addr<<4);
	comm_config_init[0][4] = 0x00 | (data>>4);
	comm_config_init[0][5] = 0x09 | (data<<4);
	LTC6804_wrcomm_cfg(total_ic,comm_config_init);     //WRCOMM ����������� COMM �Ĵ���	
	stcomm_config(total_ic);      //STCOMM ��� 3 �������ֹ��l���� I2C �Č����� 
	res = rdcomm_rdcfg(total_ic,r_comm_config);   //���� RDCOMM ������ȡ�ؔ���	

//	for(i = 0;i<8;i++)
//	{
//		printf("�Ĵ���%d��0x%x  ;\r\n",i,r_comm_config[0][i]);
//	}
}


void read_6804eeprom(uint8_t total_ic,uint8_t addr,uint8_t data[2])
{
	uint16_t res=0,i =0;
	
	write_6804eeprom(1,0x00,0xff);        //��д���� Ȼ���ٶ�  ��ַ��0x00   ����0xff
	for(i = 0;i<addr;i++)
	{
		LTC6804_wrcomm_cfg(total_ic,comm_config_rd);     //WRCOMM ����������� COMM �Ĵ���	
		stcomm_config(total_ic);      //STCOMM ��� 3 �������ֹ��l���� I2C �Č����� 
		res = rdcomm_rdcfg(total_ic,r_comm_config);   //���� RDCOMM ������ȡ�ؔ���	
		data[i] = (r_comm_config[total_ic-1][2]<<4)|(r_comm_config[total_ic-1][3]>>4);
		data[i+1] = (r_comm_config[total_ic-1][4]<<4)|(r_comm_config[total_ic-1][5]>>4);
		i++;
	}
	for(i = 0;i<(addr+1);i++)
	{
		printf("��ַ��%d ;���ݣ�0x%x  ;\r\n",i+1,data[i]);

	}
	
}

