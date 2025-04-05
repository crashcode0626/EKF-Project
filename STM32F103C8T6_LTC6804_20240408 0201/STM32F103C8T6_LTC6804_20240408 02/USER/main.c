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
 ALIENTEK精英STM32开发板实验9
 PWM输出实验  
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


extern uint16_t    stat_codes[15][6];  //存储rdstat 等信息
extern uint16_t    aux_codes[15][6];  //存储GPIO1-5 VREF等信息
extern uint16_t    cell_codes[15][12];//调用adcvax和rdcv后，电池电压存在这个数组中
extern unsigned int cell_voltage[50];  //这个数组用来显示电池电压的
//extern unsigned char cell_zu;//一共几组电池
extern unsigned char shang[500];

extern int SOC;
extern unsigned int Max,Min;
//extern int Batt[50];
extern uint8_t comm_config[][6]; 
extern uint8_t r_comm_config[][8]; 
extern float curren_mA ;
extern uint16_t MAX_mask;
extern uint16_t MIN_mask;
u8 Temp_up_flag,OV_FLAG,UV_FLAG,OC_FLAG;//过压，欠压，过流标志

uint8_t InfoFlag=0;


extern unsigned char can_buf1[8];
extern unsigned char can_buf2[8];
extern unsigned char can_buf3[8];
extern unsigned char can_buf4[8];
extern unsigned char can_buf5[8];
extern unsigned char can_buf6[8];
extern unsigned char can_buf7[8];



#if 1             //STM32F103C8T6程序
void charge_management(void)
{
	u8 key;
	
	key = KEY_Scan(0);
	if(key ==3)
	{
		//DSG_POWER_EN_ONOFF(1);
		//Beep(50);
		Only_Close_CHG();   //关闭充电MOS
		Only_Close_DSG();   //关闭放电MOS
		LED0 = ~ LED0;
	}
	else if(key ==2)
	{
		//DSG_POWER_EN_ONOFF(0);
		//Beep(50);
		Only_Open_DSG();//只打开放电MOS管
		LED0 = ~ LED0;
	}
	else if(key ==1)
	{
		Only_Open_CHG();//只打开充电MOS管
		//Beep(50);
		LED0 = ~ LED0;
	}
	
	if(Max>4250)          //电池最大电压大于4200mv
	{
		Only_Close_CHG(); //关闭充电MOS管
		OV_FLAG=1;
	}
	if(OV_FLAG==1)//过压状态
	{
		if(Max<4200)//电池最大电压小于4150mv
		{
		//	Only_Open_CHG();//打开充电MOS管
			OV_FLAG=0;
		}
	}
	if(Min<2700)//电池最小电压低于2800mv
	{
		Only_Close_DSG();//关闭放电MOS管
		UV_FLAG=1;
	}
	if(UV_FLAG==1)//欠压状态
	{
		if(Min>3000)//电池最小电压大于3000mv
		{
			//Only_Open_DSG();//打开放电MOS管
			UV_FLAG=0;
		}
	}
	if(curren_mA>2000)//如果电流大于2000ma，关闭充放电MOS管
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
	
	HSE_SetSysClock(RCC_PLLMul_9);   //则设置系统时钟为：8MHZ * 9 = 72MHZ
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	LED_Init();			     //LED端口初始化
	KEY_Init();          //按键IO初始化
	IO_CTRL_Config(); //系统的一些IO口设置；
	Adc_Init();		  		//ADC初始化
	uart_init(115200);	 //串口初始化为115200
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps 
	
	LTC6804_initialize();//LTC6804初始化配置   标准模式 不允许放电  电压所有单元   GPIO所有通道
	//SPI2_Init();   //硬件SPI
	Simulated_SPI_IoInit(); //软件SPI
	
	TIM2_Init(0xFFFF, 72 - 1);  // 1us计数，最大计数周期约65.535ms  EKF时间间隔计算
	LED0 = ~ LED0;
	while(1)
	{
		LTC6804_adcv();      //Starts cell voltage conversion                                 开始电池电压转换
		
		LTC6804_rdcv(0,cell_zu,cell_codes);        //6804获取12节电池电压    
		LTC6804_adax();      //Start an GPIO Conversion                                        启动GPIO转换

		LTC6804_rdaux(0,cell_zu,aux_codes);        //获取GPIO1-5的数据+VREF2   0 读回所有辅助寄存器

		LTC6804_ADSTAT();                          //启动状态组ADC转换，滤波模式
		LTC6804_rdstat(0,cell_zu,stat_codes);      //測量內部器件參數 (ADSTAT 命令) 
		
		Get_Cell_Voltage();           //获取电池电压四位整数 mv
		Get_Cell_Voltage_Max_Min();   //最大电压、最小电压
		Get_Update_ALL_Data();        //各项相加求总压 / 寄存器读取
		//Get_SOC();                    //目前根据电压取SOC  该函数已经包括电压电流采集
		Get_BQ_Current();             //电流检测
		adow_test() ;                 //断线检测
		BMS_STA();                    //充放电状态检测
		//***************温度***********************
		get_Alltemp();
		if(InfoFlag==1){
			Get_BQ_Current();
			LTC6804_rdcv(0,cell_zu,cell_codes);  
			Get_Cell_Voltage();
			send_info();
			LED0 = ~ LED0;
			InfoFlag=0;
		}
		

		Balance_task(30);   //计算电池电压MAX 的标号并开启均衡  100mv 均衡开启阈值
		charge_management();        //按键充放电,并检测过冲过放
		RECEICE_DATA_DEAL();    //串口1接收上位机数据处理
//		Usart_Send_Array(USART1, shang,50);       //TTL上传数据
//		CAN_SEND();
//		adcx=Get_Adc_Average(ADC_Channel_5,10);
//	  printf(" ADC:%d\r\n",adcx); //打印ADC的值
//		adc_v=(float)adcx*(3.3/4096);
//		printf(" 电压值:%f V\r\n",adc_v); //打印电压值
		
		//LED0 = ~ LED0;
		//printf("**************作者：小高霸气********************** \r\n");

		
		
	}
}

#else                          //STM32F103ZET6程序
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
		Only_Open_CHG();//打开充电MOS管
		Beep(50);
	}
	
	if(Max>4200)          //电池最大电压大于4200mv
	{
		Only_Close_CHG(); //关闭充电MOS管
		OV_FLAG=1;
	}
	if(OV_FLAG==1)//过压状态
	{
		if(Max<4150)//电池最大电压小于4150mv
		{
		//	Only_Open_CHG();//打开充电MOS管
			OV_FLAG=0;
		}
	}
	if(Min<2800)//电池最小电压低于2800mv
	{
		Only_Close_DSG();//关闭放电MOS管
		UV_FLAG=1;
	}
	if(UV_FLAG==1)//欠压状态
	{
		if(Min>3000)//电池最小电压大于3000mv
		{
			//Only_Open_DSG();//打开放电MOS管
			UV_FLAG=0;
		}
	}
	if(curren_mA>2000)//如果电流大于2000ma，关闭充放电MOS管
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
	u8 lcd_buf_1[24],lcd_buf_2[24],lcd_buf_3[24],lcd_buf_4[24],lcd_buf_5[24],lcd_buf_6[24];			//存放LCD 字符串
	u8 lcd_buf_7[24],lcd_buf_8[24],lcd_buf_9[24],lcd_buf_10[24],lcd_buf_11[24],lcd_buf_12[24];			//存放LCD 字符串
	float test2_curren_mA = 0;
  float temp[2] = {0};
	int i = 0;
	uint8_t read_data[256];
	
	
	HSE_SetSysClock(RCC_PLLMul_9);   //则设置系统时钟为：8MHZ * 9 = 72MHZ
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps    
 	LED_Init();			     //LED端口初始化
	KEY_Init();          //按键IO初始化
	IO_CTRL_Config(); //系统的一些IO口设置；
	BEEP_Init();         	//初始化蜂鸣器端口
//	LCD_Init();
	RTC_Init();	  			//RTC初始化
	
	LTC6804_initialize();//LTC6804初始化配置   标准模式 不允许放电  电压所有单元   GPIO所有通道
	//SPI2_Init();   //硬件SPI
	Simulated_SPI_IoInit(); //软件SPI
	
//	IWDG_Init(6,1250);      //看门狗4S左右

//	printf("**************LTC6804_1 测试程序****************** \r\n");
//	printf("**************作者：小高霸气********************** \r\n");
	
#if 0    //GPIO4\5 读写IIC测试        //调试成功

    //write_6804eeprom(1,0x01,0x01);	
		read_6804eeprom(1,0x05,read_data);

		printf("\r\n");
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		read_6804eeprom(1,0x05,read_data);
		while(1);
#endif	

	while(1)             //正常程序
	{	
		//IWDG_Feed();//喂狗
		
		//LTC6804_adcvax();  //6804初始化  启动组合电池电压以及GPIO1、GPIO2转换和轮询状态
		//wakeup_sleep();
		LTC6804_adcv();      //Starts cell voltage conversion                                 开始电池电压转换
		
		//set_adc(MD_FILTERED,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL); //滤波模式
		//set_adc(MD_NORMAL,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);   //标准模式
		
		LTC6804_rdcv(0,cell_zu,cell_codes);        //6804获取12节电池电压    
		LTC6804_adax();      //Start an GPIO Conversion                                        启动GPIO转换
		/**************************3选1****************************************/
		LTC6804_rdaux(0,cell_zu,aux_codes);        //获取GPIO1-5的数据+VREF2   0 读回所有辅助寄存器
		//LTC6804_rdaux(1,cell_zu,aux_codes);      //获取GPIO1-3的数据         1 读辅助A组 1-3
		//LTC6804_rdaux(2,cell_zu,aux_codes);      //获取GPIO4-5+VREF2数据     2 读辅助B组 4-5 VREF2
    /**********************************************************************/
		
		LTC6804_ADSTAT();                          //启动状态组ADC转换，滤波模式
		/*LTC6804_rdstat(uint8_t reg,uint8_t total_ic,uint16_t stat_codes[][6]）      SOC    ITMP  VA    VD  reg = 1或其他值时 读辅助A寄存器  reg = 2   读辅助B寄存器*/
		LTC6804_rdstat(0,cell_zu,stat_codes);      //測量內部器件參數 (ADSTAT 命令)   
		
		Get_Cell_Voltage();           //获取电池电压四位整数 mv
		Get_Cell_Voltage_Max_Min();   //最大电压、最小电压
		Get_Update_ALL_Data();        //各项相加求总压 / 寄存器读取
		Get_SOC();                    //目前根据电压取SOC  后面根据电流积分法
		Get_BQ_Current();             //电流检测
		adow_test() ;                 //断线检测
		//***************温度***********************
		temp[0] = Get_Tempture(aux_codes[0][1]);	
		temp[1] = Get_Tempture(aux_codes[0][2]);	
		shang[38]=temp[0];	
		can_buf7[2]=(char)shang[38];
		
//		for(i = 0;i<12;i++)
//		{
//			printf("电池%d电压：%d mv ;\r\n",(i+1),cell_voltage[i]);
//		}
//		printf("相加计算的总电压：%.3f v ;\r\n",((float)cell_voltage[12]/1000));
//		printf("MAX电压：%d mv ;\r\n",Max);
//		printf("MIN电压：%d mv ;\r\n",Min);
//		printf("最大压差：%d mv ;\r\n",(Max-Min));
//		printf("SOC：%d %%\r\n",SOC);   
//		printf("温度测量基准电压Vref2：%d mv ;\r\n",(aux_codes[0][5]/10));
//		printf("温度1：%.2f °C ;\r\n",temp[0]);
//		printf("温度2：%.2f °C ;\r\n",temp[1]);
		#if 0
		for(i = 0;i<6;i++)
		{
			printf("内部状态寄存器%d：%d  ;\r\n",(i+1),stat_codes[0][i]);
		}
		#endif
//		printf("寄存器采集的总电压：%.3f  V;\r\n",(stat_codes[0][0]*20*0.0001));    //所有電池之和 = SOC * 20 * 100μV
//		printf("内部芯片温度：%.3f  °C;\r\n",(stat_codes[0][1]/0.0075*0.0001-273));  //內部芯片溫度 (°C) = (ITMP) * 100μV/(7.5mV)°C –273°C
//		printf("模拟电源Vrega：%.3f  V;\r\n",(stat_codes[0][2]*0.0001));   //模擬電源 (VREG)   VREG 的標稱範圍為 4.5V 至 5.5V
//		printf("数字电源Vregd：%.3f  V;\r\n",(stat_codes[0][3]*0.0001));  //數字電源 (VREGD)   VREGD 的標稱範圍為2.7V 至 3.6V

//		printf("电流测量电压：%d  ;\r\n",(aux_codes[0][0])); 
//    printf("电流测量电压：%d mv ;\r\n",(aux_codes[0][0]/10)); 
		


    Balance_task(200);   //计算电池电压MAX 的标号并开启均衡  200mv 均衡开启阈值


//***************上位机显示***********************
		Usart_Send_Array(USART1, shang,50);       //TTL上传数据
		
//		CAN_SEND();                               //CAN上传数据
//***************充放电管理***********************		
		//charge_management();
//*********************显示屏显示***********************		
#if 1
		sprintf((char*)lcd_buf_1,"V_1:%d mv,V_7:%d mv",cell_voltage[0],cell_voltage[6]);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,10,200,16,16,lcd_buf_1);		//显示
    sprintf((char*)lcd_buf_2,"V_2:%d mv,V_8:%d mv",cell_voltage[1],cell_voltage[7]);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,30,200,16,16,lcd_buf_2);		//显示
    sprintf((char*)lcd_buf_3,"V_3:%d mv,V_9:%d mv",cell_voltage[2],cell_voltage[8]);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,50,200,16,16,lcd_buf_3);		//显示
		sprintf((char*)lcd_buf_4,"V_4:%d mv,V_10:%d mv",cell_voltage[3],cell_voltage[9]);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,70,200,16,16,lcd_buf_4);		//显示
		sprintf((char*)lcd_buf_5,"V_5:%d mv,V_11:%d mv",cell_voltage[4],cell_voltage[10]);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,90,200,16,16,lcd_buf_5);		//显示
		sprintf((char*)lcd_buf_6,"V_6:%d mv,V_12:%d mv",cell_voltage[5],cell_voltage[11]);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,110,200,16,16,lcd_buf_6);		//显示

		sprintf((char*)lcd_buf_7,"V%d_Max:%d,V%d_Min:%d        ",MAX_mask,Max,MIN_mask,Min);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,130,200,16,16,lcd_buf_7);		//显示最大最小电压
		
		sprintf((char*)lcd_buf_8,"Diff pressure:%d mv   ",(Max-Min));//将LCD ID打印到lcd_id数组。 
		LCD_ShowString(10,150,200,16,16,lcd_buf_8);		//显示最大压差
		
		sprintf((char*)lcd_buf_9,"V_BAT:%0.1f V,SOC:%d %%    ",(stat_codes[0][0]*20*0.0001),SOC);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,170,200,16,16,lcd_buf_9);		//显示总压和SOC
		
		sprintf((char*)lcd_buf_10,"T1:%0.1f C,T2:%0.1f C           ",temp[0],temp[1]);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,190,200,16,16,lcd_buf_10);		//显示温度
	
		sprintf((char*)lcd_buf_11,"current:%0.0f mA      ",(float)curren_mA);//将LCD ID打印到lcd_id数组。
		LCD_ShowString(10,210,200,16,16,lcd_buf_11);		//显示电流
		
		sprintf((char*)lcd_buf_12,"%d:%d:%d  %d-%d-%d       ",calendar.hour,calendar.min,calendar.sec,calendar.w_year,calendar.w_month,calendar.w_date);//将LCD ID打印到lcd_id数组。
    LCD_ShowString(30,290,200,16,16,lcd_buf_12);		//显示 时间
#endif		
			
		LED0 = ~ LED0;
		//delay_ms(1000);

		
	}

 }
#endif
