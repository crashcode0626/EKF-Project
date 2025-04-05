//========================================================================
//	�����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//	STM32���ᰮ����QQȺ: 810149456
//	���ߣ�С��
//	�绰:13728698082
//	����:1042763631@qq.com
//	���ڣ�2018.05.17
//	�汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include <string.h>
#include "filter.h"
#include <math.h>
#include "myMath.h"
 

/*=================================== ================================================================*/
/*====================================================================================================*
**���� : ��ֵ�˲�
**���� : 
**���� : 
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
int16_t MovMiddle(int16_t input)
{	
	uint8_t i,j;
	const uint8_t MOV_MIDDLE_NUM = 5;
	static int16_t middle[5]={0};
	int16_t middle_t[5];
//	MOV_MIDDLE_NUM = pidHeightRate.ki;
	for(i=1;i<MOV_MIDDLE_NUM;i++)
	{
		 middle[i-1] =  middle[i];
	}
	middle[MOV_MIDDLE_NUM-1] = input;
	memcpy(middle_t,middle,MOV_MIDDLE_NUM*sizeof(uint32_t));
	for(i=0;i<MOV_MIDDLE_NUM-1;i++)
	{
		for(j=i+1;j<MOV_MIDDLE_NUM;j++)
		{
			if(middle_t[i] > middle_t[j])
			{
				middle_t[i] ^= middle_t[j];
				middle_t[j] ^= middle_t[i];
				middle_t[i] ^= middle_t[j];
			}
		}
	}
	return middle_t[(MOV_MIDDLE_NUM+1)>>1];
}	
/*=================================== ================================================================*/
/*====================================================================================================*
**���� : ��λֵƽ���˲��������������ƽ���˲�����
**���� : 
**���� : 
**ݔ�� : None
**��ע : None
����������
�൱����λֵ�˲�+����ƽ���˲�����������N�����ݣ�ȥ��һ�����ֵ��һ����Сֵ��Ȼ�����N-2�����ݵ�����ƽ��ֵ��
Nֵ��ѡȡ��3-14
�ŵ㣺
�ں��������˲������ŵ�
����żȻ���ֵ������Ը��ţ������������������������Ĳ���ֵƫ�
ȱ�㣺
�����ٶȽ��������㷨ƽ���˲�һ�����˷�RAM��

**====================================================================================================*/
/*====================================================================================================*/
#define N 12
int16_t filter(int16_t input)
{
   char count,i,j;
   int16_t value_buf[N];
   int  sum=0,temp=0;
   for  (count=0;count<N;count++)
   {
      value_buf[count] = input;

   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   for(count=1;count<N-1;count++)
      sum += value_buf[count];
   return (int16_t)(sum/(N-2));
}

/*=================================== ================================================================*/
/*====================================================================================================*
**���� : �������ͻ�����ֵ�˲�
**���� : ÿ�β�����һ�������ݷ�����У���N�����ݽ�������ƽ������
**���� : 
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
uint16_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage)
{
		uint8_t i;	
		uint32_t sum=0;
		uint16_t max=0;
		uint16_t min=0xffff;
	
			_MovAverage->average[_MovAverage->cnt] = _MovAverage->input;	
			_MovAverage->cnt++;			
			if(_MovAverage->cnt==_MovAverage->max_cnt)
			{
				_MovAverage->cnt=0;
			}	
			for(i=0;i<_MovAverage->max_cnt;i++)
			{
					if(_MovAverage->average[i]>max)
							max = _MovAverage->average[i];
					else if(_MovAverage->average[i]<min)
							min = _MovAverage->average[i];
					sum += _MovAverage->average[i];
			}
		return ((sum-max-min)/(_MovAverage->max_cnt-2));                                    
}

uint16_t MovingAverage_Filter(MovAverage *_MovAverage)
{
		uint8_t i;	
		uint32_t sum=0;

			_MovAverage->average[_MovAverage->cnt] = _MovAverage->input;	
			_MovAverage->cnt++;			
			if(_MovAverage->cnt==_MovAverage->max_cnt)
			{
				_MovAverage->cnt=0;
			}	
			for(i=0;i<_MovAverage->max_cnt;i++)
			{
					sum += _MovAverage->average[i];
			}
		return (sum/_MovAverage->max_cnt);                                    
}

/*====================================================================================================*/
/*====================================================================================================*
** ��������: IIR_I_Filter
** ��������: IIRֱ��I���˲���
** ��    ��: InData Ϊ��ǰ����
**           *x     ����δ�˲�������
**           *y     �����˲��������
**           *b     ����ϵ��b
**           *a     ����ϵ��a
**           nb     ����*b�ĳ���
**           na     ����*a�ĳ���
**           LpfFactor
** ��    ��: OutData         
** ˵    ��: ��
** ����ԭ��: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
float IIR_I_Filter(float InputData, float *x, float *y,  const float *b, uint8_t nb, const float *a, uint8_t na)
{
  float z1,z2=0;
  int16_t i;
	
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
		y[i]=y[i-1];
  }
  x[0] = InputData;
	z1 = x[0] * b[0];
  for(i=1; i<nb; i++)
  {
    z1 += x[i]*b[i];
		z2 += y[i]*a[i];
  }
  y[0] = z1 - z2; 
  return y[0];
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : LPF_1st
**���� : һ���ͺ��˲�
**���� :  
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
//model 1:
float LPF_1_Filter_1(Filter_LPF_1 *LPF_1)
{
	return LPF_1->old_data * (1 - LPF_1->factor) + LPF_1->new_data *  LPF_1->factor;
}
//model 2:
//_LPF_1->factor = cut_frequent
float LPF_1_Filter_2(Filter_LPF_1 *LPF_1,float dt)
{
	 return LPF_1->old_data + (dt /( 1 / ( 2 * PI * LPF_1->factor ) + dt)) * (LPF_1->new_data - LPF_1->old_data);    
}
//---------------------------
// һ����ɢ��ͨ�˲���  type frequent.
// Examples for _filter:
//#define  _filter   7.9577e-3  // �� "1 / ( 2 * PI * f_cut )"�����ʽ�������; 
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
//======================================================================================================

/*====================================================================================================*/
/*====================================================================================================*
**���� : Moving_Median 
**���� : ��λֵ�˲���
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  4

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

uint8_t med_fil_cnt[MED_FIL_ITEM];

float Moving_Median(uint8_t item,uint8_t width_num,float in)
{
	uint8_t i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}		
		return ( tmp[(width_num/2)] );
	}
}
//======================================================================================================
/*====================================================================================================*/
/*====================================================================================================*
**���� : LPF2pSetCutoffFreq_1 
**���� : ���׵�ͨ�˲�
**���� : sample_freq:������  cutoff_freq����ֹƵ�ʣ�����//��ֹƵ��(����Ƶ��f0):30Hz ����Ƶ��fs:333Hz)
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
//static float           _cutoff_freq1; 
//static float           _a11;
//static float           _a21;
//static float           _b01;
//static float           _b11;
//static float           _b21;
//static float           _delay_element_11;        // buffered sample -1
//static float           _delay_element_21;        // buffered sample -2
//void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq)
//{
//		float fr =0;  
//    float ohm =0;
//    float c =0;
//	
//		fr= sample_freq/cutoff_freq;
//		ohm=tanf(PI/fr);
//		c=1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
//	
//    _cutoff_freq1 = cutoff_freq;
//    if (_cutoff_freq1 > 0.0f) 
//		{
//				_b01 = ohm*ohm/c;
//				_b11 = 2.0f*_b01;
//				_b21 = _b01;
//				_a11 = 2.0f*(ohm*ohm-1.0f)/c;
//				_a21 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
//		}
//}

/*====================================================================================================*/
/*====================================================================================================*
**���� : LPF2pApply_1 
**���� : ���׵�ͨ�˲�
**���� : sample���˲�ԭ����
**ݔ�� : �˲�������
**��ע : None
**====================================================================================================*/
///*====================================================================================================*/
//float LPF2pApply_1(float sample)
//{
//	
//		float delay_element_0 = 0, output=0;
//    if (_cutoff_freq1 <= 0.0f) {
//        // no filtering
//        return sample;
//    }
//		else
//		{
//				delay_element_0 = sample - _delay_element_11 * _a11 - _delay_element_21 * _a21;
//				// do the filtering
//				if (isnan(delay_element_0) || isinf(delay_element_0)) {
//						// don't allow bad values to propogate via the filter
//						delay_element_0 = sample;
//				}
//				output = delay_element_0 * _b01 + _delay_element_11 * _b11 + _delay_element_21 * _b21;
//				
//				_delay_element_21 = _delay_element_11;
//				_delay_element_11 = delay_element_0;

//				// return the value.  Should be no need to check limits
//				return output;
//		}
//}

//static float           _cutoff_freq2; 
//static float           _a12;
//static float           _a22;
//static float           _b02;
//static float           _b12;
//static float           _b22;
//static float           _delay_element_12;        // buffered sample -1
//static float           _delay_element_22;        // buffered sample -2
//void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq)
//{
//		float fr =0;  
//    float ohm =0;
//    float c =0;
//	
//		fr= sample_freq/cutoff_freq;
//		ohm=tanf(PI/fr);
//		c=1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
//	
//    _cutoff_freq2 = cutoff_freq;
//    if (_cutoff_freq2 > 0.0f) 
//		{
//				_b02 = ohm*ohm/c;
//				_b12 = 2.0f*_b02;
//				_b22 = _b02;
//				_a12 = 2.0f*(ohm*ohm-1.0f)/c;
//				_a22 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
//		}
//}

//float LPF2pApply_2(float sample)
//{
//	
//		float delay_element_0 = 0, output=0;
//    if (_cutoff_freq2 <= 0.0f) {
//        // no filtering
//        return sample;
//    }
//		else
//		{
//				delay_element_0 = sample - _delay_element_12 * _a12 - _delay_element_22 * _a22;
//				// do the filtering
//				if (isnan(delay_element_0) || isinf(delay_element_0)) {
//						// don't allow bad values to propogate via the filter
//						delay_element_0 = sample;
//				}
//				output = delay_element_0 * _b02 + _delay_element_12 * _b12 + _delay_element_22 * _b22;
//				
//				_delay_element_22 = _delay_element_12;
//				_delay_element_12 = delay_element_0;

//				// return the value.  Should be no need to check limits
//				return output;
//		}
//}

//static float           _cutoff_freq3; 
//static float           _a13;
//static float           _a23;
//static float           _b03;
//static float           _b13;
//static float           _b23;
//static float           _delay_element_13;        // buffered sample -1
//static float           _delay_element_23;        // buffered sample -2
//void LPF2pSetCutoffFreq_3(float sample_freq, float cutoff_freq)
//{
//		float fr =0;  
//    float ohm =0;
//    float c =0;
//	
//		fr= sample_freq/cutoff_freq;
//		ohm=tanf(PI/fr);
//		c=1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
//	
//    _cutoff_freq3 = cutoff_freq;
//    if (_cutoff_freq3 > 0.0f) 
//		{
//				_b03 = ohm*ohm/c;
//				_b13 = 2.0f*_b03;
//				_b23 = _b03;
//				_a13 = 2.0f*(ohm*ohm-1.0f)/c;
//				_a23 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
//		}
//}

//float LPF2pApply_3(float sample)
//{
//	
//		float delay_element_0 = 0, output=0;
//    if (_cutoff_freq3 <= 0.0f) {
//        // no filtering
//        return sample;
//    }
//		else
//		{
//				delay_element_0 = sample - _delay_element_13 * _a13 - _delay_element_23 * _a23;
//				// do the filtering
//				if (isnan(delay_element_0) || isinf(delay_element_0)) {
//						// don't allow bad values to propogate via the filter
//						delay_element_0 = sample;
//				}
//				output = delay_element_0 * _b03 + _delay_element_13 * _b13 + _delay_element_23 * _b23;
//				
//				_delay_element_23 = _delay_element_13;
//				_delay_element_13 = delay_element_0;

//				// return the value.  Should be no need to check limits
//				return output;
//		}
//}
  
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
/*
1.�޷��˲��㷨�������ж��˲��㷨��

����������
���ݾ����жϣ�ȷ�����β�����������ƫ��ֵ���趨ΪA����ÿ�μ�⵽��ֵʱ�жϣ�
�������ֵ���ϴ�ֵ֮��<=A���򱾴�ֵ��Ч��
�������ֵ���ϴ�ֵֻ��>A,�򱾴�ֵ��Ч����������ֵ�����ϴ�ֵ���汾��ֵ��
�ŵ㣺
����Ч�˷���żȻ����������������
ȱ�㣺
�޷��������������Եĸ��ţ�ƽ���Ȳ�

#define A 10
char value;
char filter()
{
   char  new_value;
   new_value = get_ad();
   if ( ( new_value - value > A ) || ( value - new_value > A )
      return value;
   return new_value;
}

2.��λֵ�˲���

����������
��������N�Σ�Nȡ����������N�β���ֵ����С���У�ȡ�м�ֵΪ������Чֵ
�ŵ㣺
����Ч�˷���żȻ��������Ĳ������ţ����¶ȣ�Һλ�ı仯�����ı�����������õ��˲�Ч��
ȱ�㣺
���������ٶȵȿ��ٱ仯�Ĳ�������

#define N  11
char filter()
{
   char value_buf[N];
   char count,i,j,temp;
   for ( count=0;count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   return value_buf[(N-1)/2];
}

3.����ƽ���˲�

����������
����ȡN������ֵ����ƽ�����㣬Nֵ�ϴ�ʱ���ź�ƽ���Ƚϸߣ��������Ƚϵ�
Nֵ��Сʱ���ź�ƽ���Ƚϵͣ��������Ƚϸߡ�Nֵ��ѡȡ��һ��12���ҡ�
�ŵ㣺
��Ӧ�ڶ�һ�����������ŵ��źŽ����˲��������źŵ��ص�����һ��ƽ��ֵ���ź���ĳһ��ֵ��Χ�������²���
ȱ�㣺
���ڲ����ٶȽ�����Ҫ�����ݼ����ٶȽϿ��ʵʱ���Ʋ������ã��Ƚ��˷�RAM

#define N 12
char filter()
{
   int  sum = 0;
   for ( count=0;count<N;count++)
   {
      sum + = get_ad();
      delay();
   }
   return (char)(sum/N);

4.����ƽ���˲�������ƽ���˲�����

����������
������ȡN������ֵ����һ�����У����еĳ��ȹ̶�ΪN��ÿ�β�����һ�������ݷ����β�����ӵ�ԭ�����׵�һ�����ݣ��Ƚ��ȳ�����
�Ѷ����е�N�����ݽ�������ƽ�����㣬�Ϳɻ���µ��˲������Nֵ��ѡȡ��һ��12.
�ŵ㣺
�������Ը��������õ��������ã�ƽ���ȸߣ���Ӧ�ڸ�Ƶ�񵴵�ϵͳ

ȱ�㣺
�����ȵͣ���żȻ���ֵ������Ը��ŵ��������ýϲ��������������������������Ĳ���ֵƫ���������������űȽ����صĳ����˷�RAM

#define N 12 
char value_buf[N];
char i=0;
char filter()
{
   char count;
   int  sum=0;
   value_buf[i++] = get_ad();
   if ( i == N )   i = 0;
   for ( count=0;count<N,count++)
      sum = value_buf[count];
   return (char)(sum/N);
}

5.��λֵƽ���˲��������������ƽ���˲�����

����������
�൱����λֵ�˲�+����ƽ���˲�����������N�����ݣ�ȥ��һ�����ֵ��һ����Сֵ��Ȼ�����N-2�����ݵ�����ƽ��ֵ��
Nֵ��ѡȡ��3-14
�ŵ㣺
�ں��������˲������ŵ�
����żȻ���ֵ������Ը��ţ������������������������Ĳ���ֵƫ�
ȱ�㣺
�����ٶȽ��������㷨ƽ���˲�һ�����˷�RAM��


#define N 12
char filter()
{
   char count,i,j;
   char value_buf[N];
   int  sum=0,temp=0;
   for  (count=0;count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   for(count=1;count<N-1;count++)
      sum += value[count];
   return (char)(sum/(N-2));
}

6һ���ͺ��˲���

����������
ȡa=0-1,�����˲����=��1-a��*���β���ֵ+a*�ϴ��˲����
�ŵ㣺
�������Ը��ž������õ��������ã������ڲ���Ƶ�ʽϸߵĳ���
ȱ�㣺
��λ�ͺ������ȵͣ��ͺ�̶�ȡ����aֵ�Ĵ�С�����������˲�Ƶ�ʸ��ڲ���Ƶ�ʵ�1/2�ĸ����ź�

#define a 50
char value;
char filter()
{
   char  new_value;
   new_value = get_ad();
   return (100-a)*value + a*new_value; 
}

7.��Ȩ����ƽ���˲���

����������
�ǶԵ���ƽ���˲����ĸĽ�������ͬʱ�̵����ݼ��Բ�ͬ��Ȩ;ͨ���ǣ�Խ�ӽ���ʱ�̵����ݣ�Ȩȡ��Խ�󣬸����²���ֵ��Ȩϵ��Խ����������Խ�ߣ����ź�ƽ����Խ�͡�
�ŵ㣺
�������нϴ��ͺ�ʱ�䳣���Ķ��󣬺Ͳ������ڽ϶̵�ϵͳ
ȱ��:
���ڴ��ͺ�ʱ�䳣����С���������ڽϳ����仯�������źţ�����Ѹ�ٷ�Ӧϵͳ��ǰ���ܸ��ŵ����س̶ȣ��˲�Ч���


#define N 12
char code coe[N] = {1,2,3,4,5,6,7,8,9,10,11,12};
char code sum_coe = 1+2+3+4+5+6+7+8+9+10+11+12;
char filter()
{
   char count;
   char value_buf[N];
   int  sum=0;
   for (count=0,count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (count=0,count<N;count++)
      sum += value_buf[count]*coe[count];
   return (char)(sum/sum_coe);
}


8.�����˲���

����������
����һ���˲�����������ÿ�β���ֵ�뵱ǰ��Чֵ�Ƚϣ�
�������ֵ����ǰ��Чֵ������������㣬�������ֵ<>��ǰ��Чֵ���������+1�����жϼ������Ƿ�>=����N(���)��������������,�򽫱���ֵ�滻��ǰ��Чֵ,���������
�ŵ㣺
���ڱ仯�����ı�������нϺõ��˲�Ч��,�ɱ������ٽ�ֵ�����������ķ�����/����������ʾ������ֵ������
ȱ�㣺
���ڿ��ٱ仯�Ĳ������ˣ�����ڼ������������һ�β�������ֵǡ���Ǹ���ֵ,��Ὣ����ֵ������Чֵ����ϵͳ


#define N 12
char filter()
{
   char count=0;
   char new_value;
   new_value = get_ad();
   while (value !=new_value);
   {
      count++;
      if (count>=N)   return new_value;
       delay();
      new_value = get_ad();
   }
   return value;    
}

10.��ͨ�����˲�

������
��ͨ�˲�Ҳ��һ���ͺ��˲�,�����ǵ�N�β������˲�������ֵ��(1-a)�˵�N�β���ֵ��a���ϴ��˲�������ֵ���ɼ�a<<1��
�÷��������ڱ仯���̱Ƚ����Ĳ������˲���C����������:


float low_filter(float low_buf[])
{
    float sample_value;
    float X=0.01;
    sample_value=(1_X)*low_buf[1]+X*low buf[0];
    retrun(sample_value);
}

*/


