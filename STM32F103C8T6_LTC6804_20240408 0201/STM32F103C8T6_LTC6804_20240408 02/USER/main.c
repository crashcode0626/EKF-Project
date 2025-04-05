#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "key.h"
//#include "spi.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "LTC6804-1.h"
#include "temp.h"
#include "user_get.h"
#include "simulated_SPI.h"
#include "bsp_clkconfig.h"
#include "24c02.h"
#include "lcd.h"
#include "rtc.h" 
#include "kalman.h"
#include "filter.h"
#include "IO_CTRL.h"
#include "wdg.h"
#include "beep.h"
#include "can.h"
#include "adc.h"
#include "math.h"
#include "arm_math.h"
/************************************************
 ALIENTEK��ӢSTM32������ʵ��9
 PWM���ʵ��  
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


extern uint16_t    stat_codes[15][6];  //�洢rdstat ����Ϣ
extern uint16_t    aux_codes[15][6];  //�洢GPIO1-5 VREF����Ϣ
extern uint16_t    cell_codes[15][12];//����adcvax��rdcv�󣬵�ص�ѹ�������������
extern unsigned int cell_voltage[50];  //�������������ʾ��ص�ѹ��
//extern unsigned char cell_zu;//һ��������
extern unsigned char shang[500];

extern int SOC;
extern unsigned int Max,Min;
//extern int Batt[50];
extern uint8_t comm_config[][6]; 
extern uint8_t r_comm_config[][8]; 
extern float curren_mA ;
extern uint16_t MAX_mask;
extern uint16_t MIN_mask;
u8 Temp_up_flag,OV_FLAG,UV_FLAG,OC_FLAG;//��ѹ��Ƿѹ��������־

uint8_t InfoFlag=0;


extern unsigned char can_buf1[8];
extern unsigned char can_buf2[8];
extern unsigned char can_buf3[8];
extern unsigned char can_buf4[8];
extern unsigned char can_buf5[8];
extern unsigned char can_buf6[8];
extern unsigned char can_buf7[8];



#if 1             //STM32F103C8T6����
void charge_management(void)
{
	u8 key;
	
	key = KEY_Scan(0);
	if(key ==3)
	{
		//DSG_POWER_EN_ONOFF(1);
		//Beep(50);
		Only_Close_CHG();   //�رճ��MOS
		Only_Close_DSG();   //�رշŵ�MOS
		LED0 = ~ LED0;
	}
	else if(key ==2)
	{
		//DSG_POWER_EN_ONOFF(0);
		//Beep(50);
		Only_Open_DSG();//ֻ�򿪷ŵ�MOS��
		LED0 = ~ LED0;
	}
	else if(key ==1)
	{
		Only_Open_CHG();//ֻ�򿪳��MOS��
		//Beep(50);
		LED0 = ~ LED0;
	}
	
	if(Max>4250)          //�������ѹ����4200mv
	{
		Only_Close_CHG(); //�رճ��MOS��
		OV_FLAG=1;
	}
	if(OV_FLAG==1)//��ѹ״̬
	{
		if(Max<4200)//�������ѹС��4150mv
		{
		//	Only_Open_CHG();//�򿪳��MOS��
			OV_FLAG=0;
		}
	}
	if(Min<2700)//�����С��ѹ����2800mv
	{
		Only_Close_DSG();//�رշŵ�MOS��
		UV_FLAG=1;
	}
	if(UV_FLAG==1)//Ƿѹ״̬
	{
		if(Min>3000)//�����С��ѹ����3000mv
		{
			//Only_Open_DSG();//�򿪷ŵ�MOS��
			UV_FLAG=0;
		}
	}
	if(curren_mA>2000)//�����������2000ma���رճ�ŵ�MOS��
	{
		Close_DSG_CHG();					
	}
}

 unsigned char BMS_sta,DSG_STA,CHG_STA,DSG_STA_FLAG,CHG_STA_FLAG;	

void BMS_STA(void)
	{
   DSG_STA = GPIO_ReadOutputDataBit (GPIOB,GPIO_Pin_4); 
	 CHG_STA = GPIO_ReadOutputDataBit (GPIOB,GPIO_Pin_3);
   if(DSG_STA) DSG_STA = 0;
		else DSG_STA = 1;
	 if(CHG_STA) CHG_STA= 0;
		else CHG_STA = 1;
	 shang[39]=DSG_STA;
	 shang[40]=CHG_STA;
		
	can_buf7[3]=(char)shang[39];
	can_buf7[4]=(char)shang[40];

	}
void CAN_SEND(void)
{
	can_buf1[2]=shang[2];	can_buf1[3]=shang[3];
		
	can_buf1[4]=shang[4];	can_buf1[5]=shang[5];
		
	can_buf1[6]=shang[6];	can_buf1[7]=shang[7];
		
	can_buf2[2]=shang[8];	can_buf2[3]=shang[9];
		
	can_buf2[4]=shang[10];	can_buf2[5]=shang[11];
		
	can_buf2[6]=shang[12];	can_buf2[7]=shang[13];
		
	can_buf3[2]=shang[14];	can_buf3[3]=shang[15];
		
	can_buf3[4]=shang[16];	can_buf3[5]=shang[17];
		
	can_buf3[6]=shang[18];	can_buf3[7]=shang[19];
		
	can_buf4[2]=shang[20];	can_buf4[3]=shang[21];
		
	can_buf4[4]=shang[22];	can_buf4[5]=shang[23];

	can_buf4[6]=shang[24];	can_buf4[7]=shang[25];
	
	Can_Send_Msg(can_buf1,8,0x0001);
	delay_ms(1);
	Can_Send_Msg(can_buf2,8,0x0002);
	delay_ms(1);
	Can_Send_Msg(can_buf3,8,0x0003);
	delay_ms(1);
	Can_Send_Msg(can_buf4,8,0x0004);
	delay_ms(1);
	Can_Send_Msg(can_buf5,8,0x0005);
	delay_ms(1);
	Can_Send_Msg(can_buf6,8,0x0006);
	delay_ms(1);
	Can_Send_Msg(can_buf7,8,0x0007);
	delay_ms(1);
	
}
int main(void)
{
	
	u16 adcx;
	float adc_v;
	
	HSE_SetSysClock(RCC_PLLMul_9);   //������ϵͳʱ��Ϊ��8MHZ * 9 = 72MHZ
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	LED_Init();			     //LED�˿ڳ�ʼ��
	KEY_Init();          //����IO��ʼ��
	IO_CTRL_Config(); //ϵͳ��һЩIO�����ã�
	Adc_Init();		  		//ADC��ʼ��
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps 
	
	LTC6804_initialize();//LTC6804��ʼ������   ��׼ģʽ ������ŵ�  ��ѹ���е�Ԫ   GPIO����ͨ��
	//SPI2_Init();   //Ӳ��SPI
	Simulated_SPI_IoInit(); //���SPI
	
	TIM2_Init(0xFFFF, 72 - 1);  // 1us����������������Լ65.535ms  EKFʱ��������
	LED0 = ~ LED0;
	while(1)
	{
		LTC6804_adcv();      //Starts cell voltage conversion                                 ��ʼ��ص�ѹת��
		
		LTC6804_rdcv(0,cell_zu,cell_codes);        //6804��ȡ12�ڵ�ص�ѹ    
		LTC6804_adax();      //Start an GPIO Conversion                                        ����GPIOת��

		LTC6804_rdaux(0,cell_zu,aux_codes);        //��ȡGPIO1-5������+VREF2   0 �������и����Ĵ���

		LTC6804_ADSTAT();                          //����״̬��ADCת�����˲�ģʽ
		LTC6804_rdstat(0,cell_zu,stat_codes);      //�y���Ȳ��������� (ADSTAT ����) 
		
		Get_Cell_Voltage();           //��ȡ��ص�ѹ��λ���� mv
		Get_Cell_Voltage_Max_Min();   //����ѹ����С��ѹ
		Get_Update_ALL_Data();        //�����������ѹ / �Ĵ�����ȡ
		//Get_SOC();                    //Ŀǰ���ݵ�ѹȡSOC  �ú����Ѿ�������ѹ�����ɼ�
		Get_BQ_Current();             //�������
		adow_test() ;                 //���߼��
		BMS_STA();                    //��ŵ�״̬���
		//***************�¶�***********************
		get_Alltemp();
		if(InfoFlag==1){
			Get_BQ_Current();
			LTC6804_rdcv(0,cell_zu,cell_codes);  
			Get_Cell_Voltage();
			send_info();
			LED0 = ~ LED0;
			InfoFlag=0;
		}
		

		Balance_task(30);   //�����ص�ѹMAX �ı�Ų���������  100mv ���⿪����ֵ
		charge_management();        //������ŵ�,�����������
		RECEICE_DATA_DEAL();    //����1������λ�����ݴ���
//		Usart_Send_Array(USART1, shang,50);       //TTL�ϴ�����
//		CAN_SEND();
//		adcx=Get_Adc_Average(ADC_Channel_5,10);
//	  printf(" ADC:%d\r\n",adcx); //��ӡADC��ֵ
//		adc_v=(float)adcx*(3.3/4096);
//		printf(" ��ѹֵ:%f V\r\n",adc_v); //��ӡ��ѹֵ
		
		//LED0 = ~ LED0;
		//printf("**************���ߣ�С�߰���********************** \r\n");

		
		
	}
}

#else                          //STM32F103ZET6����
void charge_management(void)
{
	u8 key;
	
	key = KEY_Scan(0);
	if(key ==1)
	{
		DSG_POWER_EN_ONOFF(1);
		Beep(50);
	}
	else if(key ==2)
	{
		DSG_POWER_EN_ONOFF(0);
		Beep(50);
	}
	else if(key ==3)
	{
		Only_Open_CHG();//�򿪳��MOS��
		Beep(50);
	}
	
	if(Max>4200)          //�������ѹ����4200mv
	{
		Only_Close_CHG(); //�رճ��MOS��
		OV_FLAG=1;
	}
	if(OV_FLAG==1)//��ѹ״̬
	{
		if(Max<4150)//�������ѹС��4150mv
		{
		//	Only_Open_CHG();//�򿪳��MOS��
			OV_FLAG=0;
		}
	}
	if(Min<2800)//�����С��ѹ����2800mv
	{
		Only_Close_DSG();//�رշŵ�MOS��
		UV_FLAG=1;
	}
	if(UV_FLAG==1)//Ƿѹ״̬
	{
		if(Min>3000)//�����С��ѹ����3000mv
		{
			//Only_Open_DSG();//�򿪷ŵ�MOS��
			UV_FLAG=0;
		}
	}
	if(curren_mA>2000)//�����������2000ma���رճ�ŵ�MOS��
	{
		Close_DSG_CHG();					
	}
}
void CAN_SEND(void)
{
	can_buf1[2]=shang[2];	can_buf1[3]=shang[3];
		
	can_buf1[4]=shang[4];	can_buf1[5]=shang[5];
		
	can_buf1[6]=shang[6];	can_buf1[7]=shang[7];
		
	can_buf2[2]=shang[8];	can_buf2[3]=shang[9];
		
	can_buf2[4]=shang[10];	can_buf2[5]=shang[11];
		
	can_buf2[6]=shang[12];	can_buf2[7]=shang[13];
		
	can_buf3[2]=shang[14];	can_buf3[3]=shang[15];
		
	can_buf3[4]=shang[16];	can_buf3[5]=shang[17];
		
	can_buf3[6]=shang[18];	can_buf3[7]=shang[19];
		
	can_buf4[2]=shang[20];	can_buf4[3]=shang[21];
		
	can_buf4[4]=shang[22];	can_buf4[5]=shang[23];

	can_buf4[6]=shang[24];	can_buf4[7]=shang[25];
	
	Can_Send_Msg(can_buf1,8,0x0001);
	delay_ms(1);
	Can_Send_Msg(can_buf2,8,0x0002);
	delay_ms(1);
	Can_Send_Msg(can_buf3,8,0x0003);
	delay_ms(1);
	Can_Send_Msg(can_buf4,8,0x0004);
	delay_ms(1);
	Can_Send_Msg(can_buf5,8,0x0005);
	delay_ms(1);
	Can_Send_Msg(can_buf6,8,0x0006);
	delay_ms(1);
	Can_Send_Msg(can_buf7,8,0x0007);
	delay_ms(1);
}


int main(void)
{		
	u8 lcd_buf_1[24],lcd_buf_2[24],lcd_buf_3[24],lcd_buf_4[24],lcd_buf_5[24],lcd_buf_6[24];			//���LCD �ַ���
	u8 lcd_buf_7[24],lcd_buf_8[24],lcd_buf_9[24],lcd_buf_10[24],lcd_buf_11[24],lcd_buf_12[24];			//���LCD �ַ���
	float test2_curren_mA = 0;
  float temp[2] = {0};
	int i = 0;
	uint8_t read_data[256];
	
	
	HSE_SetSysClock(RCC_PLLMul_9);   //������ϵͳʱ��Ϊ��8MHZ * 9 = 72MHZ
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps    
 	LED_Init();			     //LED�˿ڳ�ʼ��
	KEY_Init();          //����IO��ʼ��
	IO_CTRL_Config(); //ϵͳ��һЩIO�����ã�
	BEEP_Init();         	//��ʼ���������˿�
//	LCD_Init();
	RTC_Init();	  			//RTC��ʼ��
	
	LTC6804_initialize();//LTC6804��ʼ������   ��׼ģʽ ������ŵ�  ��ѹ���е�Ԫ   GPIO����ͨ��
	//SPI2_Init();   //Ӳ��SPI
	Simulated_SPI_IoInit(); //���SPI
	
//	IWDG_Init(6,1250);      //���Ź�4S����

//	printf("**************LTC6804_1 ���Գ���****************** \r\n");
//	printf("**************���ߣ�С�߰���********************** \r\n");
	
#if 0    //GPIO4\5 ��дIIC����        //���Գɹ�

    //write_6804eeprom(1,0x01,0x01);	
		read_6804eeprom(1,0x05,read_data);

		printf("\r\n");
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		read_6804eeprom(1,0x05,read_data);
		while(1);
#endif	

	while(1)             //��������
	{	
		//IWDG_Feed();//ι��
		
		//LTC6804_adcvax();  //6804��ʼ��  ������ϵ�ص�ѹ�Լ�GPIO1��GPIO2ת������ѯ״̬
		//wakeup_sleep();
		LTC6804_adcv();      //Starts cell voltage conversion                                 ��ʼ��ص�ѹת��
		
		//set_adc(MD_FILTERED,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL); //�˲�ģʽ
		//set_adc(MD_NORMAL,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);   //��׼ģʽ
		
		LTC6804_rdcv(0,cell_zu,cell_codes);        //6804��ȡ12�ڵ�ص�ѹ    
		LTC6804_adax();      //Start an GPIO Conversion                                        ����GPIOת��
		/**************************3ѡ1****************************************/
		LTC6804_rdaux(0,cell_zu,aux_codes);        //��ȡGPIO1-5������+VREF2   0 �������и����Ĵ���
		//LTC6804_rdaux(1,cell_zu,aux_codes);      //��ȡGPIO1-3������         1 ������A�� 1-3
		//LTC6804_rdaux(2,cell_zu,aux_codes);      //��ȡGPIO4-5+VREF2����     2 ������B�� 4-5 VREF2
    /**********************************************************************/
		
		LTC6804_ADSTAT();                          //����״̬��ADCת�����˲�ģʽ
		/*LTC6804_rdstat(uint8_t reg,uint8_t total_ic,uint16_t stat_codes[][6]��      SOC    ITMP  VA    VD  reg = 1������ֵʱ ������A�Ĵ���  reg = 2   ������B�Ĵ���*/
		LTC6804_rdstat(0,cell_zu,stat_codes);      //�y���Ȳ��������� (ADSTAT ����)   
		
		Get_Cell_Voltage();           //��ȡ��ص�ѹ��λ���� mv
		Get_Cell_Voltage_Max_Min();   //����ѹ����С��ѹ
		Get_Update_ALL_Data();        //�����������ѹ / �Ĵ�����ȡ
		Get_SOC();                    //Ŀǰ���ݵ�ѹȡSOC  ������ݵ������ַ�
		Get_BQ_Current();             //�������
		adow_test() ;                 //���߼��
		//***************�¶�***********************
		temp[0] = Get_Tempture(aux_codes[0][1]);	
		temp[1] = Get_Tempture(aux_codes[0][2]);	
		shang[38]=temp[0];	
		can_buf7[2]=(char)shang[38];
		
//		for(i = 0;i<12;i++)
//		{
//			printf("���%d��ѹ��%d mv ;\r\n",(i+1),cell_voltage[i]);
//		}
//		printf("��Ӽ�����ܵ�ѹ��%.3f v ;\r\n",((float)cell_voltage[12]/1000));
//		printf("MAX��ѹ��%d mv ;\r\n",Max);
//		printf("MIN��ѹ��%d mv ;\r\n",Min);
//		printf("���ѹ�%d mv ;\r\n",(Max-Min));
//		printf("SOC��%d %%\r\n",SOC);   
//		printf("�¶Ȳ�����׼��ѹVref2��%d mv ;\r\n",(aux_codes[0][5]/10));
//		printf("�¶�1��%.2f ��C ;\r\n",temp[0]);
//		printf("�¶�2��%.2f ��C ;\r\n",temp[1]);
		#if 0
		for(i = 0;i<6;i++)
		{
			printf("�ڲ�״̬�Ĵ���%d��%d  ;\r\n",(i+1),stat_codes[0][i]);
		}
		#endif
//		printf("�Ĵ����ɼ����ܵ�ѹ��%.3f  V;\r\n",(stat_codes[0][0]*20*0.0001));    //����늳�֮�� = SOC * 20 * 100��V
//		printf("�ڲ�оƬ�¶ȣ�%.3f  ��C;\r\n",(stat_codes[0][1]/0.0075*0.0001-273));  //�Ȳ�оƬ�ض� (��C) = (ITMP) * 100��V/(7.5mV)��C �C273��C
//		printf("ģ���ԴVrega��%.3f  V;\r\n",(stat_codes[0][2]*0.0001));   //ģ�M�Դ (VREG)   VREG �Ę˷Q������ 4.5V �� 5.5V
//		printf("���ֵ�ԴVregd��%.3f  V;\r\n",(stat_codes[0][3]*0.0001));  //�����Դ (VREGD)   VREGD �Ę˷Q������2.7V �� 3.6V

//		printf("����������ѹ��%d  ;\r\n",(aux_codes[0][0])); 
//    printf("����������ѹ��%d mv ;\r\n",(aux_codes[0][0]/10)); 
		


    Balance_task(200);   //�����ص�ѹMAX �ı�Ų���������  200mv ���⿪����ֵ


//***************��λ����ʾ***********************
		Usart_Send_Array(USART1, shang,50);       //TTL�ϴ�����
		
//		CAN_SEND();                               //CAN�ϴ�����
//***************��ŵ����***********************		
		//charge_management();
//*********************��ʾ����ʾ***********************		
#if 1
		sprintf((char*)lcd_buf_1,"V_1:%d mv,V_7:%d mv",cell_voltage[0],cell_voltage[6]);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,10,200,16,16,lcd_buf_1);		//��ʾ
    sprintf((char*)lcd_buf_2,"V_2:%d mv,V_8:%d mv",cell_voltage[1],cell_voltage[7]);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,30,200,16,16,lcd_buf_2);		//��ʾ
    sprintf((char*)lcd_buf_3,"V_3:%d mv,V_9:%d mv",cell_voltage[2],cell_voltage[8]);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,50,200,16,16,lcd_buf_3);		//��ʾ
		sprintf((char*)lcd_buf_4,"V_4:%d mv,V_10:%d mv",cell_voltage[3],cell_voltage[9]);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,70,200,16,16,lcd_buf_4);		//��ʾ
		sprintf((char*)lcd_buf_5,"V_5:%d mv,V_11:%d mv",cell_voltage[4],cell_voltage[10]);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,90,200,16,16,lcd_buf_5);		//��ʾ
		sprintf((char*)lcd_buf_6,"V_6:%d mv,V_12:%d mv",cell_voltage[5],cell_voltage[11]);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,110,200,16,16,lcd_buf_6);		//��ʾ

		sprintf((char*)lcd_buf_7,"V%d_Max:%d,V%d_Min:%d        ",MAX_mask,Max,MIN_mask,Min);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,130,200,16,16,lcd_buf_7);		//��ʾ�����С��ѹ
		
		sprintf((char*)lcd_buf_8,"Diff pressure:%d mv   ",(Max-Min));//��LCD ID��ӡ��lcd_id���顣 
		LCD_ShowString(10,150,200,16,16,lcd_buf_8);		//��ʾ���ѹ��
		
		sprintf((char*)lcd_buf_9,"V_BAT:%0.1f V,SOC:%d %%    ",(stat_codes[0][0]*20*0.0001),SOC);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,170,200,16,16,lcd_buf_9);		//��ʾ��ѹ��SOC
		
		sprintf((char*)lcd_buf_10,"T1:%0.1f C,T2:%0.1f C           ",temp[0],temp[1]);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,190,200,16,16,lcd_buf_10);		//��ʾ�¶�
	
		sprintf((char*)lcd_buf_11,"current:%0.0f mA      ",(float)curren_mA);//��LCD ID��ӡ��lcd_id���顣
		LCD_ShowString(10,210,200,16,16,lcd_buf_11);		//��ʾ����
		
		sprintf((char*)lcd_buf_12,"%d:%d:%d  %d-%d-%d       ",calendar.hour,calendar.min,calendar.sec,calendar.w_year,calendar.w_month,calendar.w_date);//��LCD ID��ӡ��lcd_id���顣
    LCD_ShowString(30,290,200,16,16,lcd_buf_12);		//��ʾ ʱ��
#endif		
			
		LED0 = ~ LED0;
		//delay_ms(1000);

		
	}

 }
#endif
