//========================================================================
//	爱好者电子工作室-淘宝 https://devotee.taobao.com/
//	STM32四轴爱好者QQ群: 810149456
//	作者：小刘
//	电话:13728698082
//	邮箱:1042763631@qq.com
//	日期：2018.05.17
//	版本：V1.0
//========================================================================
//套件购买地址：https://devotee.taobao.com/
//                 爱好者电子工作室
//特此声明：
//
//         此程序只能用作学习，如用商业用途。必追究责任！
//          
//
//
#include <string.h>
#include "filter.h"
#include <math.h>
#include "myMath.h"
 

/*=================================== ================================================================*/
/*====================================================================================================*
**函数 : 中值滤波
**功能 : 
**输入 : 
**輸出 : None
**备注 : None
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
**函数 : 中位值平均滤波法（防脉冲干扰平均滤波法）
**功能 : 
**输入 : 
**輸出 : None
**备注 : None
方法解析：
相当于中位值滤波+算术平均滤波，连续采样N个数据，去掉一个最大值和一个最小值，然后计算N-2个数据的算术平均值。
N值的选取：3-14
优点：
融合了两种滤波法的优点
对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差。
缺点：
测量速度较慢，和算法平均滤波一样，浪费RAM。

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
**函数 : 抗干扰型滑动均值滤波
**功能 : 每次采样到一个新数据放入队列，对N个数据进行算术平均运算
**输入 : 
**輸出 : None
**备注 : None
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
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
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
**函数 : LPF_1st
**功能 : 一阶滞后滤波
**输入 :  
**輸出 : None
**备注 : None
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
// 一阶离散低通滤波器  type frequent.
// Examples for _filter:
//#define  _filter   7.9577e-3  // 由 "1 / ( 2 * PI * f_cut )"这个公式计算得来; 
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
//======================================================================================================

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Moving_Median 
**功能 : 中位值滤波法
**輸出 : None
**备注 : None
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
**函数 : LPF2pSetCutoffFreq_1 
**功能 : 二阶低通滤波
**输入 : sample_freq:采样率  cutoff_freq：截止频率（例：//截止频率(中心频率f0):30Hz 采样频率fs:333Hz)
**輸出 : None
**备注 : None
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
**函数 : LPF2pApply_1 
**功能 : 二阶低通滤波
**输入 : sample：滤波原数据
**輸出 : 滤波后数据
**备注 : None
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
1.限幅滤波算法（程序判断滤波算法）

方法解析：
根据经验判断，确定两次采样允许的最大偏差值（设定为A），每次检测到新值时判断：
如果本次值与上次值之差<=A，则本次值有效，
如果本次值与上次值只差>A,则本次值无效，放弃本次值，用上次值代替本次值。
优点：
能有效克服因偶然因素引起的脉冲干扰
缺点：
无法抑制那种周期性的干扰，平滑度差

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

2.中位值滤波法

方法解析：
连续采样N次（N取奇数），把N次采样值按大小排列，取中间值为本次有效值
优点：
能有效克服因偶然因素引起的波动干扰，对温度，液位的变化缓慢的被测参数有良好的滤波效果
缺点：
对流量，速度等快速变化的参数不宜

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

3.算术平均滤波

方法解析：
连续取N个采样值进行平均运算，N值较大时：信号平滑度较高，但灵敏度较低
N值较小时：信号平滑度较低，但灵敏度较高。N值的选取：一般12左右。
优点：
适应于对一般具有随机干扰的信号进行滤波，这样信号的特点是有一个平均值，信号在某一数值范围附近上下波动
缺点：
对于测量速度较慢或要求数据计算速度较快的实时控制并不适用，比较浪费RAM

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

4.递推平均滤波（滑动平均滤波法）

方法解析：
把连续取N个采样值看成一个队列，队列的长度固定为N，每次采样到一个新数据放入队尾，并扔掉原来队首的一次数据（先进先出）。
把队列中的N个数据进行算术平均运算，就可获得新的滤波结果。N值的选取：一般12.
优点：
对周期性干扰有良好的抑制作用，平滑度高，适应于高频振荡的系统

缺点：
灵敏度低，对偶然出现的脉冲性干扰的抑制作用较差。不易消除由于脉冲干扰所引起打的采样值偏差，不适用于脉冲干扰比较严重的场合浪费RAM

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

5.中位值平均滤波法（防脉冲干扰平均滤波法）

方法解析：
相当于中位值滤波+算术平均滤波，连续采样N个数据，去掉一个最大值和一个最小值，然后计算N-2个数据的算术平均值。
N值的选取：3-14
优点：
融合了两种滤波法的优点
对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差。
缺点：
测量速度较慢，和算法平均滤波一样，浪费RAM。


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

6一阶滞后滤波法

方法解析：
取a=0-1,本次滤波结果=（1-a）*本次采样值+a*上次滤波结果
优点：
对周期性干扰具有良好的抑制作用，适用于波动频率较高的场合
缺点：
相位滞后，灵敏度低，滞后程度取决于a值的大小，不能消除滤波频率高于采样频率的1/2的干扰信号

#define a 50
char value;
char filter()
{
   char  new_value;
   new_value = get_ad();
   return (100-a)*value + a*new_value; 
}

7.加权递推平均滤波法

方法解析：
是对递推平均滤波法的改进，即不同时刻的数据加以不同的权;通常是，越接近现时刻的数据，权取得越大，给予新采样值的权系数越大，则灵敏度越高，但信号平滑度越低。
优点：
适用于有较大纯滞后时间常数的对象，和采样周期较短的系统
缺点:
对于纯滞后时间常数较小，采样周期较长，变化缓慢的信号，不能迅速反应系统当前所受干扰的严重程度，滤波效果差。


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


8.消抖滤波法

方法解析：
设置一个滤波计数器，将每次采样值与当前有效值比较：
如果采样值＝当前有效值，则计数器清零，如果采样值<>当前有效值，则计数器+1，并判断计数器是否>=上限N(溢出)，如果计数器溢出,则将本次值替换当前有效值,并清计数器
优点：
对于变化缓慢的被测参数有较好的滤波效果,可避免在临界值附近控制器的反复开/关跳动或显示器上数值抖动。
缺点：
对于快速变化的参数不宜，如果在计数器溢出的那一次采样到的值恰好是干扰值,则会将干扰值当作有效值导入系统


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

10.低通数字滤波

解析：
低通滤波也称一阶滞后滤波,方法是第N次采样后滤波结果输出值是(1-a)乘第N次采样值加a乘上次滤波结果输出值。可见a<<1。
该方法适用于变化过程比较慢的参数的滤波的C程序函数如下:


float low_filter(float low_buf[])
{
    float sample_value;
    float X=0.01;
    sample_value=(1_X)*low_buf[1]+X*low buf[0];
    retrun(sample_value);
}

*/


