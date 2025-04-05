
连接线：
PB12 ---CS     片选
PB13 ---SCK    时钟
PB14 ---MISO   输出
PB15 ---MOSI   输入


ADC清除命令：
CLRCELL   :电池电压寄存器组A、B、C、D
CLRAUX    :辅助寄存器组A、B
CLRSTAT   :状态寄存器组A、B

測量電池電壓 (ADCV 命令)
測量電池電壓和 GPIO (ADCVAX 命令)
輔助 (GPIO) 測量 (ADAX 命令)
测量内部器件参数(ADSTAT)




set_adc(MD_FILTERED,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);      
        滤波模式    不允许放电  所有电池单元  所有gpio通道  
开启滤波模式，并且设置不允许放电，才能在测量阶段自动关闭均衡控制，电压测量才能正常，不会出现上一节电压不准的情况；
   //滤波模式 不允许放电  所有单元   是所有通道   测量阶段会自动关闭均衡控制，电压测量才能正常  但是缺出现温度采集不变化
 
IIC  能参照PDF写入读出，但是无法直接读出或者延时后读出，还需调试


/****************************20210909     修改wakeup_idle()     delayMicroseconds(5)改为delay_ms(3)*********************************/
void wakeup_idle()
{
  LTC6804_CS_low;
  //delayMicroseconds(5); //Guarantees the isoSPI will be in ready mode保证isoSPI将处于就绪模式
	delay_ms(3);
  LTC6804_CS_high;
}
 //更新以下代码后，可以开均衡可以关均衡，且没有太大的测量误差
 void Write_Balance_Commond(uint16_t Mask)
{
	uint8_t cmd[4];
	uint16_t cmd_pec;
  uint8_t balance_config[15][6]={{0xfc,0x52,0x00,0x00,0x00,0x00}};    //前四个字节为开启均衡命令
	//1
	cmd[0] = balance_config[0][0];
	cmd[1] = balance_config[0][1];
	
	cmd_pec = pec15_calc(2, cmd);

	balance_config[0][2] = (uint8_t)(cmd_pec >> 8);
	balance_config[0][3] = (uint8_t)(cmd_pec);
	
	balance_config[0][4] |= (  Mask & 0x00ff );
	balance_config[0][5] |= ( (Mask >> 8) & 0x000f );
	
	LTC6804_wrcfg(1,balance_config);   //开启均衡
}




/*****************************下面为IIC测试函数调试过程，调试未成功而记录**************************************/
#if 0    //GPIO4\5 读写IIC测试        //调试未成功           -----写命令能在STM32读写IIC例程中读到 LTC6804-1写入的数据；   A0  01   AA       A0代表eeprom地址   01代表内部地址  AA 代表写入的数据


Thanks for the input. The first IC has been successfully read from the 2451 daisy chain. The following read sequence was used:

ICOM[0] - START(0110)       ----- DATA[0] - (7bit-Addr)+R ----- FCOM[0] - MASTER NACK(1000)

ICOM[1] - SDA LOW(0000)  ----- DATA[1] - 8 HIGH BITS  ----- FCOM[1] - MASTER ACK(0000)

ICOM[2] - SDA LOW(0000)  ----- DATA[2] - 8 HIGH BITS  ----- FCOM[2] - SLAVE NACK + MASTER STOP(1001)

However, data from the next IC is not being read. We've tried writing the sequence you posted before our read sequence with no success. We tried delays too but the other ICs are still blocked. The 2451 are organized onto separate boards connected by resistor addresses from LTC4316s. All are connected to the GPIO4 and GPIO5 ports of the 6811.

Is there any adjustments that can be made to our sequence to allow the next IC to be read? 


//读写EEPROM 成功      调试时间 20210910
while(1)
{
	//write_6804eeprom(1,0x01,0x01);


	read_6804eeprom(1,0x05,read_data);

	printf("\r\n");
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	while(1);
}

