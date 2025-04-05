#include "temp.h"
#include "math.h"


// 利用GPIO2 GPIO3获取温度
#if 1      

#define B 3950.0//温度系数

#define TN 298.15//额定温度(绝对温度加常温:273.15+25)

#define RN 10// 额定阻值(绝对温度时的电阻值10k)

#define BaseVol 3 //ADC基准电压

float Get_Tempture(u16 adax)
{
	float RV,RT,Tmp;
	u16 buf;
	
	buf = adax*4096/30000;
	
	RV=BaseVol/4096.0*(float)buf;//ADC为10位ADC,求出NTC电压:RV=ADCValu/1024*BaseVoltag
	RT=RV*10/(BaseVol-RV);//求出当前温度阻值 (BaseVoltage-RV)/R16=RV/RT;
	Tmp=1/(1/TN+(log(RT/RN)/B))-273.15;//%RT = RN exp*B(1/T-1/TN)%
	return Tmp;
}

#endif

//利用两片LTC1380  +LTC6255   扩展16个温度采集通道   GPIO4(SDA)  GPIO5(SCL)






