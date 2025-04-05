#include "user_get.h"
#include "LTC6804-1.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "motor.h"
#include "24l01.h" 
#include "lcd.h"
#include "drive_car.h"
#include "LTC6804-1.h"
#include "temp.h"
#include "stm32f10x.h"
#include "math.h"
 #include "kalman.h"
 #include "filter.h"
 #include "stdio.h"
 #include "EKF_SOC.h"
 #include "usart.h"

//unsigned char cell_zu=1;//一共几组电池
uint16_t    stat_codes[15][6];    //存储rdstat 等信息
uint16_t    aux_codes[15][6];  //存储GPIO1-5 VREF等信息
uint16_t    cell_codes[15][12];//调用adcvax和rdcv后，电池电压存在这个数组中
unsigned int cell_voltage[50];  //这个数组用来显示电池电压的
float SOC_result[12];
unsigned char shang[500]={0xAA,0x01};
unsigned char shang1[50]={0xAA,0x02};
unsigned char shang2[50]={0xAA,0x03};
unsigned char shang3[50]={0xAA,0x04};

unsigned char can_buf1[8]={0xAA,0x01};
unsigned char can_buf2[8]={0xAA,0x02};
unsigned char can_buf3[8]={0xAA,0x03};
unsigned char can_buf4[8]={0xAA,0x04};
unsigned char can_buf5[8]={0xAA,0x05};
unsigned char can_buf6[8]={0xAA,0x06};
unsigned char can_buf7[8]={0xAA,0x07};

int SOC;
unsigned int Max,Min;

float curren_mA = 0;
volatile float current;//用于EKF SOC调用计算  current<0放电 current>0充电

uint16_t MAX_mask = 0;
uint16_t MIN_mask = 0;
//extern int Batt[50];

extern u8 Temp_up_flag,OV_FLAG,UV_FLAG,OC_FLAG;//过压，欠压，过流标志
float temp[2] = {0};//温度

extern unsigned char BMS_sta,DSG_STA,CHG_STA,DSG_STA_FLAG,CHG_STA_FLAG;

void Get_Update_ALL_Data(void)
{
	#if 0    //所有电池相加计算总压
	int Sum_val;
	Sum_val= cell_voltage[0]+cell_voltage[1]+cell_voltage[2]+cell_voltage[3]+cell_voltage[4]+cell_voltage[5]
	+cell_voltage[6]+cell_voltage[7]+cell_voltage[8]	+cell_voltage[9]
	+cell_voltage[10]+cell_voltage[11];
	
	cell_voltage[12] = Sum_val;
	
	shang[32]=(char)(cell_voltage[12] >> 8);
  shang[33]=(char)(cell_voltage[12] &0XFF);
	#else    //内部寄存器采集总压
	//**********************总压*************************
	cell_voltage[12] = stat_codes[0][0]*20*0.0001*1000;   //mV
	shang[32]=(char)(cell_voltage[12] >> 8);
	shang[33]=(char)(cell_voltage[12] &0XFF);
	#endif
	
  shang2[14]=(char)shang[32];
  shang2[15]=(char)shang[33];
	
	can_buf6[2]=(char)shang[32];
	can_buf6[3]=(char)shang[33];
	

}

void Get_Cell_Voltage_Max_Min(void)
{
	char i;
	
	Max=cell_voltage[0];
	Min=cell_voltage[0];
	
	for(i=1;i<12;i++)
	{
		if ((i >= 0 && i <= 2) || (i >=6 && i <=8)){
			if(cell_voltage[i]>Max)
		{
			Max=cell_voltage[i];
		}
		if(cell_voltage[i]<Min)
		{
			Min=cell_voltage[i];
		}	
			}
		
					
	}
	
	shang[41]=(char)(Max >> 8);
	shang[42]=(char)(Max & 0x00FF);

	shang[43]=(char)(Min >> 8);
	shang[44]=(char)(Min & 0x00FF);
}


float SOC_result[12] = {0}; // 存储 12 节电芯 SOC 结果，默认初始化为 0
//ekf soc在此调用
void Get_SOC(void)
{
//    uint32_t current_time, last_time = 0, dt_us;
//    
//    // 采集电压、电流
//    Get_BQ_Current();
//    Get_Cell_Voltage();

//    // **获取时间间隔 (us)**
//    current_time = TIM2_GetCounter();
//    if (last_time == 0) {
//        dt_us = 100000; // 初次计算时默认 100ms
//    } else {
//        dt_us = TIM2_GetElapsedTime(last_time, current_time);
//    }
//    last_time = current_time; // 更新前一次时间

//    // **计算 6 组电池 SOC (0,1,2,6,7,8)**
//    int cell_indices[6] = {0, 1, 2, 6, 7, 8};  
//    for (int i = 0; i < 6; i++) {
//        int cell_idx = cell_indices[i];  // 取对应电芯
//        EKF_SOC_Update(cell_voltage[cell_idx], current, dt_us, &SOC_result[cell_idx]); // 计算 SOC
//    }

//    // **存储 SOC 结果**
//    cell_voltage[13] = (uint16_t)(SOC_result[0] * 100.0f); // 存储 0 号电芯 SOC

//    // **存储到通信 buffer**
//    shang[34] = (char)(cell_voltage[13] >> 8);
//    shang[35] = (char)(cell_voltage[13] & 0XFF);
//    shang2[16] = shang[34];
//    shang2[17] = shang[35];
//    can_buf6[4] = shang[34];
//    can_buf6[5] = shang[35];
}



 void Get_Cell_Voltage(void)
{
	char vi;
	for(vi=0;vi<12;vi++)
	{	
		cell_voltage[vi]=cell_codes[0][vi]/10;
		
	 shang[vi+2+vi]=(char)(cell_voltage[vi]>>8);
	 shang[vi+3+vi]=(char)(cell_voltage[vi]&0X00FF);
	
	}
}

	/*
		INA282固定增益G=50。REF=2.5V因此
     Vout=(Isense x Rshunt x 50)+ 2.5V
	*/
void Get_BQ_Current(void)           //计算电流        这里应该做滤波，取一段时间内的数据，做平滑滤波      2.5	V	基准
{
	uint32_t  temp = 0;
	//**********************电流计算*************************
	/*
	INA282固定增益G=50。REF=2.5V因此
	Vout=(Isense x Rshunt x 50)+ 2.5V
	*/
	static int state_i =10;
	curren_mA = (aux_codes[0][0]);
	//curren_mA = (25000 - curren_mA) ;       //REF3225 2.5V基准    实际基准电压误差 = （2.5-2.4964）/2.5 = 0.144%
	//curren_mA = (30000 - curren_mA) ;       //VREF2 3V基准    
	//curren_mA = (3*5 - curren_mA/10000*5)*1000 ;       //VREF2 3V基准    Vout=3V-(Isense x 0.004 x 50)    Isense(A) = (Vout - 3)/0.2 = 15 - 5Vout   Isense(mA) =1000(15 - 5Vout)
	//curren_mA = (3000/2 - curren_mA/20) ;   //0.04欧   3V基准
	//curren_mA = (2500/2 - curren_mA/20) ;   //0.04欧   2.5V基准
  // curren_mA = (25000 - curren_mA) ;            //0.002欧   2.5V基准      4毫欧 2个电阻
	curren_mA = (26330 - curren_mA)/175 ;       //VREF 2.5V基准   40毫欧  单个电阻
	//curren_mA =curren_mA *1.1;
	if(state_i==10)     //剔除因卡尔曼滤波前期导致的异常数据
	{
		while(state_i--)
		{
		 //curren_mA = KalmanFilter_1ADC(curren_mA );   //原始数据经过卡尔曼滤波
		 //curren_mA = KalmanFilter_2ADC(curren_mA );   //原始数据经过卡尔曼滤波
	   //curren_mA  = MovMiddle((uint16_t)curren_mA);             //中值滤波
			curren_mA = filter((uint16_t)curren_mA);      //中位值平均滤波法（防脉冲干扰平均滤波法）
		}
	}
	//if(abs((int)curren_mA)<=38)curren_mA=0;        //取绝对值，小电流忽略不计
	
	current=-curren_mA/100;//符号取反 curren_mA大于0为放电，反之为放电
	
	//printf("current %.2f",current);
	temp = abs((int)curren_mA);      //取绝对值  上位机不支持显示有符号
	shang[36]=(uint32_t)temp>> 8;     //无符号  上位机不支持显示有符号
	shang[37]=(uint32_t)temp & 0X00FF;
	
//	shang[36]=(uint32_t)curren_mA >> 8;     //无符号  上位机不支持显示有符号
//	shang[37]=(uint32_t)curren_mA & 0X00FF;
	can_buf6[6]=(char)shang[36];
	can_buf6[7]=(char)shang[37];
}

uint8_t Make_Balance_Decision(int mv)     //是否满足开启均衡的条件   mv 为 压差  可设置
{
	uint8_t balance_switch;
	if( (Max-Min)>mv )//均衡开启条件  
	{
		balance_switch=1;
	}
	else
	{
		balance_switch=0;
	}
	return(balance_switch);
}

void  Balance_task(uint16_t mv)    //计算哪个电池需要均衡          代码尚未完成，需要求出需要均衡的电池标号
{
	char i;
	uint16_t V_mask = 0;	
	uint8_t balance_switch = 0;
	balance_switch = Make_Balance_Decision(mv);    //判断最大最小压差＞100mv  则开启均衡
	for(i=0;i<12;i++)
	{
		if ((i >= 0 && i <= 2) || (i >=6 && i <=8)){
		if(cell_voltage[i]==Max)     //输出所有最大值的位置
		{
			MAX_mask = i+1; 
		}	
		if(cell_voltage[i]==Min)     //输出所有最大值的位置
		{
			MIN_mask = i+1; 
		}		
	}
			
	}
	V_mask = 0x0001 << (MAX_mask-1);
//	printf("电压最大的编码：%d \r\n",MAX_mask);
//	printf("电压最小的编码：%d \r\n",MIN_mask);
	if(balance_switch)                           //若满足均衡条件
	{
		
		Write_Balance_Commond(V_mask);	  //均衡控制     1为开启，0为关闭  总共12位，
	}
	else 
	{
		Write_Balance_Commond(0x0000);	  //均衡控制     1为开启，0为关闭  总共12位，
	}
}
/***************均衡***********************
//注意：开启均衡后会影响上一节电压，检测电压会偏大，实际测量电压正常，后面再优化
————————该问题已经解决：开启滤波模式MD_FILTERED测量但是温度采集有问题
***********************************************/
//void Balance_Bat(uint16_t mask)
//{
//	uint16_t V_mask = 0;		
//	
//	V_mask = 0x0001 << (mask-1);

//	Write_Balance_Commond(V_mask);	  //均衡控制     1为开启，0为关闭  总共12位，
//	//delay_ms(500);
//	//Write_Balance_Commond(0x0000);	  //均衡控制     1为开启，0为关闭  总共12位，
//}
uint16_t adow_test(void)          //断线检测
{
	uint16_t    adow_pu_codes[15][12];
	uint16_t    adow_pd_codes[15][12];
  int num;
	char i;
	
	uint16_t   buf = 0x0000;
	LTC6804_adcv();      //Starts cell voltage conversion 
	LTC6804_adow_set_pup();        //断线  检测 上拉
	LTC6804_adow_set_pup();        //断线  检测 上拉
	LTC6804_adow_set_pup();        //断线  检测 上拉
	LTC6804_rdcv(0,cell_zu,adow_pu_codes);        //6804获取12节电池电压  

	LTC6804_adow_reset_pup();        //断线  检测 下拉
	LTC6804_adow_reset_pup();        //断线  检测 下拉
	LTC6804_adow_reset_pup();        //断线  检测 下拉
	LTC6804_rdcv(0,cell_zu,adow_pd_codes);        //6804获取12节电池电压 
	
	#ifdef DEBUG
	for(i = 0;i<12;i++)
		{
			printf("电池%d电压：%d mv ; 电池电压：%d mv ;\r\n",(i+1),adow_pu_codes[0][i],adow_pd_codes[0][i]);
		}
	#endif
	
	for(i=1;i<12;i++)
	{
		//num = adow_pu_codes[0][i]-adow_pd_codes[0][i];
		num = abs(adow_pu_codes[0][i]-adow_pd_codes[0][i]);       //求绝对值
		if(num > 4000)
		{
			buf = buf|(0x0001<<i); 	
		}
		if(buf)
		{
			printf("请检查电池连接线，有断线情况，故障码：0x%x \r\n",buf);
		}
	}
	
	return buf;
}

//读取所有温度探头数据
void get_Alltemp(){
		temp[0] = Get_Tempture(aux_codes[0][1]);	
		temp[1] = Get_Tempture(aux_codes[0][2]);	
 }

//串口向上位机发送 0 1 2 6 7 8电芯的SOC 电压 current 以及充电与放电开关管的情况
//函数将在main.c调用
void send_info()
{
    // 格式化输出数据
    printf(   
           "%u,%u,%u,%u,%u,%u,"   // 电压数据
           "%f,"                  // 电流
           "%u,%u,"               // 放电和充电状态
           "%u,%u,"               // 过压、欠压标志
           "%f,%f\n",             // 温度探头1和2的温度
         
        cell_voltage[0], cell_voltage[1], cell_voltage[2], cell_voltage[6], cell_voltage[7], cell_voltage[8], 
        current,
        DSG_STA, CHG_STA,
        OV_FLAG, UV_FLAG,
        temp[0], temp[1]);
}





