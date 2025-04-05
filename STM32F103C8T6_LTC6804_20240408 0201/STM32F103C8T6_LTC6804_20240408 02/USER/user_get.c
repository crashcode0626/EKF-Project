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

//unsigned char cell_zu=1;//һ��������
uint16_t    stat_codes[15][6];    //�洢rdstat ����Ϣ
uint16_t    aux_codes[15][6];  //�洢GPIO1-5 VREF����Ϣ
uint16_t    cell_codes[15][12];//����adcvax��rdcv�󣬵�ص�ѹ�������������
unsigned int cell_voltage[50];  //�������������ʾ��ص�ѹ��
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
volatile float current;//����EKF SOC���ü���  current<0�ŵ� current>0���

uint16_t MAX_mask = 0;
uint16_t MIN_mask = 0;
//extern int Batt[50];

extern u8 Temp_up_flag,OV_FLAG,UV_FLAG,OC_FLAG;//��ѹ��Ƿѹ��������־
float temp[2] = {0};//�¶�

extern unsigned char BMS_sta,DSG_STA,CHG_STA,DSG_STA_FLAG,CHG_STA_FLAG;

void Get_Update_ALL_Data(void)
{
	#if 0    //���е����Ӽ�����ѹ
	int Sum_val;
	Sum_val= cell_voltage[0]+cell_voltage[1]+cell_voltage[2]+cell_voltage[3]+cell_voltage[4]+cell_voltage[5]
	+cell_voltage[6]+cell_voltage[7]+cell_voltage[8]	+cell_voltage[9]
	+cell_voltage[10]+cell_voltage[11];
	
	cell_voltage[12] = Sum_val;
	
	shang[32]=(char)(cell_voltage[12] >> 8);
  shang[33]=(char)(cell_voltage[12] &0XFF);
	#else    //�ڲ��Ĵ����ɼ���ѹ
	//**********************��ѹ*************************
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


float SOC_result[12] = {0}; // �洢 12 �ڵ�о SOC �����Ĭ�ϳ�ʼ��Ϊ 0
//ekf soc�ڴ˵���
void Get_SOC(void)
{
//    uint32_t current_time, last_time = 0, dt_us;
//    
//    // �ɼ���ѹ������
//    Get_BQ_Current();
//    Get_Cell_Voltage();

//    // **��ȡʱ���� (us)**
//    current_time = TIM2_GetCounter();
//    if (last_time == 0) {
//        dt_us = 100000; // ���μ���ʱĬ�� 100ms
//    } else {
//        dt_us = TIM2_GetElapsedTime(last_time, current_time);
//    }
//    last_time = current_time; // ����ǰһ��ʱ��

//    // **���� 6 ���� SOC (0,1,2,6,7,8)**
//    int cell_indices[6] = {0, 1, 2, 6, 7, 8};  
//    for (int i = 0; i < 6; i++) {
//        int cell_idx = cell_indices[i];  // ȡ��Ӧ��о
//        EKF_SOC_Update(cell_voltage[cell_idx], current, dt_us, &SOC_result[cell_idx]); // ���� SOC
//    }

//    // **�洢 SOC ���**
//    cell_voltage[13] = (uint16_t)(SOC_result[0] * 100.0f); // �洢 0 �ŵ�о SOC

//    // **�洢��ͨ�� buffer**
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
		INA282�̶�����G=50��REF=2.5V���
     Vout=(Isense x Rshunt x 50)+ 2.5V
	*/
void Get_BQ_Current(void)           //�������        ����Ӧ�����˲���ȡһ��ʱ���ڵ����ݣ���ƽ���˲�      2.5	V	��׼
{
	uint32_t  temp = 0;
	//**********************��������*************************
	/*
	INA282�̶�����G=50��REF=2.5V���
	Vout=(Isense x Rshunt x 50)+ 2.5V
	*/
	static int state_i =10;
	curren_mA = (aux_codes[0][0]);
	//curren_mA = (25000 - curren_mA) ;       //REF3225 2.5V��׼    ʵ�ʻ�׼��ѹ��� = ��2.5-2.4964��/2.5 = 0.144%
	//curren_mA = (30000 - curren_mA) ;       //VREF2 3V��׼    
	//curren_mA = (3*5 - curren_mA/10000*5)*1000 ;       //VREF2 3V��׼    Vout=3V-(Isense x 0.004 x 50)    Isense(A) = (Vout - 3)/0.2 = 15 - 5Vout   Isense(mA) =1000(15 - 5Vout)
	//curren_mA = (3000/2 - curren_mA/20) ;   //0.04ŷ   3V��׼
	//curren_mA = (2500/2 - curren_mA/20) ;   //0.04ŷ   2.5V��׼
  // curren_mA = (25000 - curren_mA) ;            //0.002ŷ   2.5V��׼      4��ŷ 2������
	curren_mA = (26330 - curren_mA)/175 ;       //VREF 2.5V��׼   40��ŷ  ��������
	//curren_mA =curren_mA *1.1;
	if(state_i==10)     //�޳��򿨶����˲�ǰ�ڵ��µ��쳣����
	{
		while(state_i--)
		{
		 //curren_mA = KalmanFilter_1ADC(curren_mA );   //ԭʼ���ݾ����������˲�
		 //curren_mA = KalmanFilter_2ADC(curren_mA );   //ԭʼ���ݾ����������˲�
	   //curren_mA  = MovMiddle((uint16_t)curren_mA);             //��ֵ�˲�
			curren_mA = filter((uint16_t)curren_mA);      //��λֵƽ���˲��������������ƽ���˲�����
		}
	}
	//if(abs((int)curren_mA)<=38)curren_mA=0;        //ȡ����ֵ��С�������Բ���
	
	current=-curren_mA/100;//����ȡ�� curren_mA����0Ϊ�ŵ磬��֮Ϊ�ŵ�
	
	//printf("current %.2f",current);
	temp = abs((int)curren_mA);      //ȡ����ֵ  ��λ����֧����ʾ�з���
	shang[36]=(uint32_t)temp>> 8;     //�޷���  ��λ����֧����ʾ�з���
	shang[37]=(uint32_t)temp & 0X00FF;
	
//	shang[36]=(uint32_t)curren_mA >> 8;     //�޷���  ��λ����֧����ʾ�з���
//	shang[37]=(uint32_t)curren_mA & 0X00FF;
	can_buf6[6]=(char)shang[36];
	can_buf6[7]=(char)shang[37];
}

uint8_t Make_Balance_Decision(int mv)     //�Ƿ����㿪�����������   mv Ϊ ѹ��  ������
{
	uint8_t balance_switch;
	if( (Max-Min)>mv )//���⿪������  
	{
		balance_switch=1;
	}
	else
	{
		balance_switch=0;
	}
	return(balance_switch);
}

void  Balance_task(uint16_t mv)    //�����ĸ������Ҫ����          ������δ��ɣ���Ҫ�����Ҫ����ĵ�ر��
{
	char i;
	uint16_t V_mask = 0;	
	uint8_t balance_switch = 0;
	balance_switch = Make_Balance_Decision(mv);    //�ж������Сѹ�100mv  ��������
	for(i=0;i<12;i++)
	{
		if ((i >= 0 && i <= 2) || (i >=6 && i <=8)){
		if(cell_voltage[i]==Max)     //����������ֵ��λ��
		{
			MAX_mask = i+1; 
		}	
		if(cell_voltage[i]==Min)     //����������ֵ��λ��
		{
			MIN_mask = i+1; 
		}		
	}
			
	}
	V_mask = 0x0001 << (MAX_mask-1);
//	printf("��ѹ���ı��룺%d \r\n",MAX_mask);
//	printf("��ѹ��С�ı��룺%d \r\n",MIN_mask);
	if(balance_switch)                           //�������������
	{
		
		Write_Balance_Commond(V_mask);	  //�������     1Ϊ������0Ϊ�ر�  �ܹ�12λ��
	}
	else 
	{
		Write_Balance_Commond(0x0000);	  //�������     1Ϊ������0Ϊ�ر�  �ܹ�12λ��
	}
}
/***************����***********************
//ע�⣺����������Ӱ����һ�ڵ�ѹ������ѹ��ƫ��ʵ�ʲ�����ѹ�������������Ż�
�����������������������Ѿ�����������˲�ģʽMD_FILTERED���������¶Ȳɼ�������
***********************************************/
//void Balance_Bat(uint16_t mask)
//{
//	uint16_t V_mask = 0;		
//	
//	V_mask = 0x0001 << (mask-1);

//	Write_Balance_Commond(V_mask);	  //�������     1Ϊ������0Ϊ�ر�  �ܹ�12λ��
//	//delay_ms(500);
//	//Write_Balance_Commond(0x0000);	  //�������     1Ϊ������0Ϊ�ر�  �ܹ�12λ��
//}
uint16_t adow_test(void)          //���߼��
{
	uint16_t    adow_pu_codes[15][12];
	uint16_t    adow_pd_codes[15][12];
  int num;
	char i;
	
	uint16_t   buf = 0x0000;
	LTC6804_adcv();      //Starts cell voltage conversion 
	LTC6804_adow_set_pup();        //����  ��� ����
	LTC6804_adow_set_pup();        //����  ��� ����
	LTC6804_adow_set_pup();        //����  ��� ����
	LTC6804_rdcv(0,cell_zu,adow_pu_codes);        //6804��ȡ12�ڵ�ص�ѹ  

	LTC6804_adow_reset_pup();        //����  ��� ����
	LTC6804_adow_reset_pup();        //����  ��� ����
	LTC6804_adow_reset_pup();        //����  ��� ����
	LTC6804_rdcv(0,cell_zu,adow_pd_codes);        //6804��ȡ12�ڵ�ص�ѹ 
	
	#ifdef DEBUG
	for(i = 0;i<12;i++)
		{
			printf("���%d��ѹ��%d mv ; ��ص�ѹ��%d mv ;\r\n",(i+1),adow_pu_codes[0][i],adow_pd_codes[0][i]);
		}
	#endif
	
	for(i=1;i<12;i++)
	{
		//num = adow_pu_codes[0][i]-adow_pd_codes[0][i];
		num = abs(adow_pu_codes[0][i]-adow_pd_codes[0][i]);       //�����ֵ
		if(num > 4000)
		{
			buf = buf|(0x0001<<i); 	
		}
		if(buf)
		{
			printf("�����������ߣ��ж�������������룺0x%x \r\n",buf);
		}
	}
	
	return buf;
}

//��ȡ�����¶�̽ͷ����
void get_Alltemp(){
		temp[0] = Get_Tempture(aux_codes[0][1]);	
		temp[1] = Get_Tempture(aux_codes[0][2]);	
 }

//��������λ������ 0 1 2 6 7 8��о��SOC ��ѹ current �Լ������ŵ翪�عܵ����
//��������main.c����
void send_info()
{
    // ��ʽ���������
    printf(   
           "%u,%u,%u,%u,%u,%u,"   // ��ѹ����
           "%f,"                  // ����
           "%u,%u,"               // �ŵ�ͳ��״̬
           "%u,%u,"               // ��ѹ��Ƿѹ��־
           "%f,%f\n",             // �¶�̽ͷ1��2���¶�
         
        cell_voltage[0], cell_voltage[1], cell_voltage[2], cell_voltage[6], cell_voltage[7], cell_voltage[8], 
        current,
        DSG_STA, CHG_STA,
        OV_FLAG, UV_FLAG,
        temp[0], temp[1]);
}





