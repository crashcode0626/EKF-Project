#include "temp.h"
#include "math.h"


// ����GPIO2 GPIO3��ȡ�¶�
#if 1      

#define B 3950.0//�¶�ϵ��

#define TN 298.15//��¶�(�����¶ȼӳ���:273.15+25)

#define RN 10// ���ֵ(�����¶�ʱ�ĵ���ֵ10k)

#define BaseVol 3 //ADC��׼��ѹ

float Get_Tempture(u16 adax)
{
	float RV,RT,Tmp;
	u16 buf;
	
	buf = adax*4096/30000;
	
	RV=BaseVol/4096.0*(float)buf;//ADCΪ10λADC,���NTC��ѹ:RV=ADCValu/1024*BaseVoltag
	RT=RV*10/(BaseVol-RV);//�����ǰ�¶���ֵ (BaseVoltage-RV)/R16=RV/RT;
	Tmp=1/(1/TN+(log(RT/RN)/B))-273.15;//%RT = RN exp*B(1/T-1/TN)%
	return Tmp;
}

#endif

//������ƬLTC1380  +LTC6255   ��չ16���¶Ȳɼ�ͨ��   GPIO4(SDA)  GPIO5(SCL)






