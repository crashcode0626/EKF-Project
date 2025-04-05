/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*  @date    2023-3-15
	*	@author  ���ͿƼ�
	*	@brief   SPI������Ļ
   ************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32F407ZGT6���İ� ���ͺţ�FK407M2-ZGT6��+ 2.00��Һ��ģ�飨�ͺţ�SPI200M1-240*320��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> ����˵����
	*
	*	SPI����LCD���м򵥵���ʾ
	*
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "usart.h"
#include "lcd_spi_200.h"
#include "arm_math.h"
#include "ekf.h"
#include <assert.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#define BUFFER_SIZE 256
char receivedData[BUFFER_SIZE];
EKF_State ekf;

// LCD���Ժ��������������ڵײ�
void 	LCD_Test_Clear(void);			// ��������
void 	LCD_Test_Text(void);			   // �ı�����
void 	LCD_Test_Variable (void);	   // ������ʾ������������С��
void 	LCD_Test_Color(void);			// ����������
void 	LCD_Test_Grahic(void);		   // 2Dͼ�λ���
void 	LCD_Test_Image(void);			// ͼƬ��ʾ
void  LCD_Test_Direction(void);	   // ������ʾ����

void print_matrix(UART_HandleTypeDef *huart, const char* name, float* data);
void matrix_ops(UART_HandleTypeDef *huart);


	
/********************************************** BMS functions *******************************************/

	float c1_soc;
	float c2_soc;
	float c3_soc;
	float c4_soc;
	float c5_soc;
	float c6_soc;
	float c1_voltage;
	float c2_voltage;
	float c3_voltage;
	float c4_voltage;
	float c5_voltage;
	float c6_voltage;
	float current;//����
	uint8_t DSG_STA;//�ŵ�MOS״̬
	uint8_t CHG_STA;//���MOS״̬
	uint8_t OV_FLAG;//��ѹ����
	uint8_t UV_FLAG;//��ѹ����
	float temp1;
	float temp2;
	
	char cmd[10];

void display_value(void){
	 LCD_SetAsciiFont(&ASCII_Font16);
	 LCD_DisplayDecimals(0,0,c1_soc,3,2);
	 LCD_DisplayDecimals(50,0,c2_soc,3,2);
	 LCD_DisplayDecimals(100,0,c3_soc,3,2);
	 LCD_DisplayDecimals(150,0,c4_soc,3,2);
	 LCD_DisplayDecimals(200,0,c5_soc,3,2);
	 LCD_DisplayDecimals(250,0,c6_soc,3,2);
	 LCD_DisplayDecimals(0,20,c1_voltage,3,3);
	 LCD_DisplayDecimals(50,20,c2_voltage,3,3);
	 LCD_DisplayDecimals(100,20,c3_voltage,3,3);
	 LCD_DisplayDecimals(150,20,c4_voltage,3,3);
	 LCD_DisplayDecimals(200,20,c5_voltage,3,3);
	 LCD_DisplayDecimals(250,20,c6_voltage,3,3);
	 LCD_SetAsciiFont(&ASCII_Font20);
	 LCD_DisplayDecimals(0,50,current,3,4);
	 
	 LCD_DisplayText(0,100,"DSG_STA");
	 LCD_DisplayNumber(0,120,DSG_STA,1);
	 LCD_DisplayText(80,100,"CHG_STA");
	 LCD_DisplayNumber(80,120,CHG_STA,1);
	 LCD_DisplayText(160,100,"OV_FLAG");
	 LCD_DisplayNumber(160,120,OV_FLAG,1);
	 LCD_DisplayText(240,100,"UV_FLAG");
	 LCD_DisplayNumber(240,120,UV_FLAG,1);
	 
	 LCD_DisplayDecimals(0,180,temp1,4,4);
	 LCD_DisplayDecimals(100,180,temp2,4,4);
}

void send_info(UART_HandleTypeDef *huart)
{
    char buffer[128];
    
    // ��ʽ�����ݣ�ƥ��Python�ű������ĸ�ʽ��
    int len = snprintf(buffer, sizeof(buffer),
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"  // 6��SOCֵ
        "%u,%u,%u,%u,%u,%u,"              // 6����ѹֵ(mV)
        "%.3f,"                           // ����
        "%u,%u,%u,%u,"                    // 4��״̬��־
        "%.1f,%.1f\n",                    // 2���¶�ֵ
        c1_soc, c2_soc, c3_soc, c4_soc, c5_soc, c6_soc,
        (uint32_t)(c1_voltage*1000), (uint32_t)(c2_voltage*1000),
        (uint32_t)(c3_voltage*1000), (uint32_t)(c4_voltage*1000),
        (uint32_t)(c5_voltage*1000), (uint32_t)(c6_voltage*1000),
        current,
        DSG_STA, 0, CHG_STA, UV_FLAG,  // �ڶ�����־��Ϊ0��ƥ��Python��ʽ
        temp1, temp2);

    // ��ȫ����
    if(len > 0 && len < sizeof(buffer)) {
        HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    }
}

//����SOC
void get_soc() {
    
        // Ϊÿ�ڵ�ؼ��� SOC
        c1_soc = EKF_EstimateSOC(&ekf, c1_voltage, current, 0.75f);
        c2_soc = EKF_EstimateSOC(&ekf, c2_voltage, current, 0.75f);
        c3_soc = EKF_EstimateSOC(&ekf, c3_voltage, current, 0.75f);
        c4_soc = EKF_EstimateSOC(&ekf, c4_voltage, current, 0.75f);
        c5_soc = EKF_EstimateSOC(&ekf, c5_voltage, current, 0.75f);
        c6_soc = EKF_EstimateSOC(&ekf, c6_voltage, current, 0.75f);
    
}

// ����BMS���ݲ���ֵ��ȫ�ֱ���
void parse_battery_data(char *data)
{
    // ��ʱ�����洢�������
    uint32_t tmp_voltages[6];  // ��ѹֵΪ�޷���������BMS���͵�������mVֵ��
    float tmp_current;
    uint8_t tmp_dsg_sta, tmp_chg_sta, tmp_ov_flag, tmp_uv_flag;
    float tmp_temp1, tmp_temp2;

    // �ϸ�ƥ��BMS���͵����ݸ�ʽ
    int result = sscanf(data, "%u,%u,%u,%u,%u,%u,%f,%hhu,%hhu,%hhu,%hhu,%f,%f",
                       &tmp_voltages[0], &tmp_voltages[1], &tmp_voltages[2],
                       &tmp_voltages[3], &tmp_voltages[4], &tmp_voltages[5],
                       &tmp_current,
                       &tmp_dsg_sta, &tmp_chg_sta,
                       &tmp_ov_flag, &tmp_uv_flag,
                       &tmp_temp1, &tmp_temp2);

    // �������������13���ֶΣ�
    if (result == 13) {
        // ת����ѹ��λ��mV -> V������ֵ��ȫ�ֱ���
        c1_voltage = tmp_voltages[0] / 1000.0f;
        c2_voltage = tmp_voltages[1] / 1000.0f;
        c3_voltage = tmp_voltages[2] / 1000.0f;
        c4_voltage = tmp_voltages[3] / 1000.0f;
        c5_voltage = tmp_voltages[4] / 1000.0f;
        c6_voltage = tmp_voltages[5] / 1000.0f;

        // ��ֵ����ȫ�ֱ���
        current = tmp_current;
        DSG_STA = tmp_dsg_sta;
        CHG_STA = tmp_chg_sta;
        OV_FLAG = tmp_ov_flag;
        UV_FLAG = tmp_uv_flag;
        temp1 = tmp_temp1;
        temp2 = tmp_temp2;

        // �������
//        printf("Voltages: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f V\n", 
//               c1_voltage, c2_voltage, c3_voltage, 
//               c4_voltage, c5_voltage, c6_voltage);
//        printf("Current: %.3fA, Temp1: %.1fC, Temp2: %.1fC\n", 
//               current, temp1, temp2);
    } else {
//        printf("Parse error! Expected 13 fields, got %d\n", result);
//        printf("Raw data: %s\n", data);  // ��ӡԭʼ���ݰ�������
					HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
    }
}
/********************************************** �������� *******************************************/

void SystemClock_Config(void);		// ʱ�ӳ�ʼ��
void DWT_Init(void)
{
    // ȷ�� DWT ������
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // ���� DWT
    }
}



EKF_State ekf;


int main(void)
{
	HAL_Init();					// ��ʼ��HAL��
	SystemClock_Config();	// ����ϵͳʱ��
	LED_Init();					// ��ʼ��LED����
	
	USART1_Init();				// USART1��ʼ��	
	USART2_Init();	
	
	SPI_LCD_Init();			// SPI LCD��Ļ��ʼ��
	
	EKF_Init(&ekf);
	LCD_SetDirection(Direction_H); 	
	
	
	 
	 
	while (1)
		
	{
		 
			 // ���� huart1 ����
    if (HAL_UART_Receive(&huart1, cmd, 3, 100) == HAL_OK) {
        HAL_UART_Transmit(&huart2, cmd, 3, 100);	
				 if(rxComplete)
    {
        rxComplete = 0;  // �����ֶ������־
        
        // 1. �������ԭʼ���ݣ�ͨ��USART1��
        //printf("BMS RAW: %s\n", rxBuffer);
        
        // 2. ���ݽ���
        parse_battery_data((char*)rxBuffer);
        get_soc();
        // 3. ҵ����
        display_value();
				send_info(&huart1);
		}
    }
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


/*************************************************************************************************
*	�� �� ��:	LCD_Test_Clear
*
*	��������:	��������
*************************************************************************************************/
void LCD_Test_Clear(void)
{
	uint8_t	i = 0;			// ��������
			
	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_BLACK);				// ���û�����ɫ

	for(i=0;i<8;i++)
	{
		switch (i)		// �л�����ɫ
		{
			case 0: LCD_SetBackColor(LIGHT_RED); 		break;	
			case 1: LCD_SetBackColor(LIGHT_GREEN); 	break;				
			case 2: LCD_SetBackColor(LIGHT_BLUE); 		break;
			case 3: LCD_SetBackColor(LIGHT_YELLOW); 	break;
			case 4: LCD_SetBackColor(LIGHT_CYAN);		break;
			case 5: LCD_SetBackColor(LIGHT_GREY); 		break;
			case 6: LCD_SetBackColor(LIGHT_MAGENTA); 	break;
			case 7: LCD_SetBackColor(LCD_WHITE); 		break;			
			default:	break;			
		}
		LCD_Clear();		// ����
		LCD_DisplayText(13, 70,"STM32 ˢ������");
		LCD_DisplayText(13,106,"��Ļ�ֱ���:240*320");
		LCD_DisplayText(13,142,"������:ST7789");	
		HAL_Delay(1000);	// ��ʱ
	}
}

/*************************************************************************************************
*	�� �� ��:	LCD_Test_Text
*
*	��������:	�ı���ʾ����
*
*	˵    ��:	��ʾ�ı����������������С�����ĺ�ASCII�ַ� 
*************************************************************************************************/
void LCD_Test_Text(void)
{
	LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
	LCD_Clear(); 								// ����
	
	LCD_SetColor(LCD_WHITE);
	LCD_SetAsciiFont(&ASCII_Font32); LCD_DisplayString(0, 0,"!#$'()*+,-.0123"); 						    		
	LCD_SetAsciiFont(&ASCII_Font24); LCD_DisplayString(0,32,"!#$'()*+,-.012345678"); 				   
	LCD_SetAsciiFont(&ASCII_Font20); LCD_DisplayString(0,56,"!#$'()*+,-.0123456789:;<");      	
	LCD_SetAsciiFont(&ASCII_Font16); LCD_DisplayString(0,76,"!#$'()*+,-.0123456789:;<=>?@AB"); 	
	LCD_SetAsciiFont(&ASCII_Font12); LCD_DisplayString(0,92,"!#$'()*+,-.0123456789:;<=>?@ABCDEFGHIJKL"); 	
																																		
	LCD_SetColor(LCD_CYAN);                                                                             
	LCD_SetAsciiFont(&ASCII_Font12); LCD_DisplayString(0,104,"!#&'()*+,-.0123456789:;<=>?@ABCDEFGHIJKL"); 	
	LCD_SetAsciiFont(&ASCII_Font16); LCD_DisplayString(0,116,"!#&'()*+,-.0123456789:;<=>?@AB"); 	
	LCD_SetAsciiFont(&ASCII_Font20); LCD_DisplayString(0,132,"!#&'()*+,-.0123456789:;<");		  	
	LCD_SetAsciiFont(&ASCII_Font24); LCD_DisplayString(0,152,"!#&'()*+,-.012345678"); 				  	
	LCD_SetAsciiFont(&ASCII_Font32); LCD_DisplayString(0,176,"!#&'()*+,-.0123"); 							  		

	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_YELLOW);				// ���û��ʣ���ɫ
	LCD_DisplayText(0, 216,"ASCII�ַ���");

	HAL_Delay(2000);	// ��ʱ�ȴ�
	LCD_Clear(); 								// ����

	LCD_SetTextFont(&CH_Font12);			// ����1212��������,ASCII�����ӦΪ1206
	LCD_SetColor(0X8AC6D1);					// ���û���
	LCD_DisplayText(14, 10,"1212:���ͿƼ�");	
	
	LCD_SetTextFont(&CH_Font16);			// ����1616��������,ASCII�����ӦΪ1608
	LCD_SetColor(0XC5E1A5);					// ���û���
	LCD_DisplayText(14, 30,"1616:���ͿƼ�");		
	
	LCD_SetTextFont(&CH_Font20);			// ����2020��������,ASCII�����ӦΪ2010
	LCD_SetColor(0XFFB549);					// ���û���
	LCD_DisplayText(14, 60,"2020:���ͿƼ�");		

	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(0XFF585D);					// ���û���
	LCD_DisplayText(14, 90,"2424:���ͿƼ�");		

	LCD_SetTextFont(&CH_Font32);			// ����3232��������,ASCII�����ӦΪ3216
	LCD_SetColor(0xFFB6B9);					// ���û���
	LCD_DisplayText(14, 130,"3232:���ͿƼ�");		

	LCD_SetTextFont(&CH_Font24);			// ����2424��������,ASCII�����ӦΪ2412
	LCD_SetColor(LCD_WHITE);
 	LCD_DisplayText(14, 180,"������ʾ");	  

	HAL_Delay(2000);	// ��ʱ�ȴ�
}
/************************************************************************************************
*	�� �� ��:	LCD_Test_Variable
*
*	��������:	������ʾ������������С��
*************************************************************************************************/
void LCD_Test_Variable (void)
{
	uint16_t i;					// ��������
	int32_t	a = 0;			// �����������������ڲ���
	int32_t	b = 0;			// �����������������ڲ���
	int32_t	c = 0;			// �����������������ڲ���

	double p = 3.1415926;	// ���帡�������������ڲ���
	double f = -1234.1234;	// ���帡�������������ڲ���
	
	LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
	LCD_Clear(); 								// ����
	
   LCD_SetTextFont(&CH_Font24);     
	LCD_SetColor(LIGHT_CYAN);					// ���û��ʣ�����ɫ		
	LCD_DisplayText(0,10,"����:");				
	LCD_DisplayText(0,40,"����:");					
				
	LCD_SetColor(LIGHT_YELLOW);				// ���û��ʣ�����ɫ		
	LCD_DisplayText(0, 80,"���ո�:");	
	LCD_DisplayText(0,110,"���0:");	
	
	LCD_SetColor(LIGHT_RED);					// ���û���	������ɫ		
	LCD_DisplayText(0, 150,"��С��:");	
	LCD_DisplayText(0, 180,"��С��:");		
	
	for(i=0;i<100;i++)
   {
		LCD_SetColor(LIGHT_CYAN);								// ���û���	������ɫ	
		LCD_ShowNumMode(Fill_Space);							// ����λ���ո�
		LCD_DisplayNumber( 80,10, b+i*10, 4) ;				// ��ʾ����			
		LCD_DisplayNumber( 80,40, c-i*10, 4) ;				// ��ʾ����			
		
		LCD_SetColor(LIGHT_YELLOW);								// ���û��ʣ�����ɫ	

		LCD_ShowNumMode(Fill_Space);								// ����λ��� �ո�
		LCD_DisplayNumber( 130, 80, a+i*150, 8) ;				// ��ʾ����		

		LCD_ShowNumMode(Fill_Zero);								// ����λ���0      
		LCD_DisplayNumber( 130,110, b+i*150, 8) ;				// ��ʾ����			
		
		LCD_SetColor(LIGHT_RED);									// ���û��ʣ�����ɫ			
		LCD_ShowNumMode(Fill_Space);								// ����λ���ո�		
		LCD_DisplayDecimals( 100, 150, p+i*0.1,  6,3);		// ��ʾС��	
		LCD_DisplayDecimals( 100, 180, f+i*0.01, 11,4);		// ��ʾС��		
		
		HAL_Delay(15);				
   }
	HAL_Delay(2500);		
}
/*************************************************************************************************
*	�� �� ��:	LCD_Test_Color
*
*	��������:	��ɫ��
*************************************************************************************************/
void LCD_Test_Color(void)
{
	uint16_t i;					// ��������
	uint16_t y;
// ��ɫ����>>>>>	
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ
	
	LCD_SetTextFont(&CH_Font20);			// ��������
	LCD_SetColor(LCD_WHITE);				// ���û�����ɫ
	LCD_DisplayText(0,0,"RGB����ɫ:");

	//ʹ�û��ߺ�����������ɫɫ��
	for(i=0;i<240;i++)
	{
		LCD_SetColor( LCD_RED-(i<<16) );
      LCD_DrawLine_V(0+i,  20,10);
	}
	for(i=0;i<240;i++)
	{
		LCD_SetColor( LCD_GREEN-(i<<8) );
      LCD_DrawLine_V(0+i,  35,10);
	}
	for(i=0;i<240;i++)
	{
		LCD_SetColor( LCD_BLUE-i );
      LCD_DrawLine_V(0+i,  50,10);
	}	

   y = 70;
   LCD_SetColor(LIGHT_CYAN);    LCD_FillRect(150,y+5     ,90,10);  LCD_DisplayString(0,y     ,"LIGHT_CYAN");	   
	LCD_SetColor(LIGHT_MAGENTA); LCD_FillRect(150,y+20*1+5,90,10);  LCD_DisplayString(0,y+20*1,"LIGHT_MAGENTA");	
	LCD_SetColor(LIGHT_YELLOW);  LCD_FillRect(150,y+20*2+5,90,10);  LCD_DisplayString(0,y+20*2,"LIGHT_YELLOW");	
	LCD_SetColor(LIGHT_GREY);    LCD_FillRect(150,y+20*3+5,90,10);  LCD_DisplayString(0,y+20*3,"LIGHT_GREY");  	
	LCD_SetColor(LIGHT_RED);     LCD_FillRect(150,y+20*4+5,90,10);  LCD_DisplayString(0,y+20*4,"LIGHT_RED");  	
	LCD_SetColor(LIGHT_BLUE);    LCD_FillRect(150,y+20*5+5,90,10);  LCD_DisplayString(0,y+20*5,"LIGHT_BLUE");  	

   LCD_SetColor(DARK_CYAN);     LCD_FillRect(150,y+20*6+5,90,10);  LCD_DisplayString(0,y+20*6,"DARK_CYAN");		
	LCD_SetColor(DARK_MAGENTA);  LCD_FillRect(150,y+20*7+5,90,10);  LCD_DisplayString(0,y+20*7,"DARK_MAGENTA");	
	LCD_SetColor(DARK_YELLOW);   LCD_FillRect(150,y+20*8+5,90,10);  LCD_DisplayString(0,y+20*8,"DARK_YELLOW");	
	LCD_SetColor(DARK_GREY);     LCD_FillRect(150,y+20*9+5,90,10);	 LCD_DisplayString(0,y+20*9,"DARK_GREY");	
	LCD_SetColor(DARK_RED);   	  LCD_FillRect(150,y+20*10+5,90,10); LCD_DisplayString(0,y+20*10,"DARK_RED");	
	LCD_SetColor(DARK_GREEN);    LCD_FillRect(150,y+20*11+5,90,10); LCD_DisplayString(0,y+20*11,"DARK_GREEN");	

	HAL_Delay(2000);
}

/*************************************************************************************************
*	�� �� ��:	LCD_Test_Grahic
*
*	��������:	2Dͼ�λ���
*************************************************************************************************/
void LCD_Test_Grahic(void)
{
	LCD_SetBackColor(LCD_BLACK); //���ñ���ɫ
	LCD_Clear(); //������ˢ����ɫ	

	LCD_SetColor(LCD_WHITE);	
	LCD_DrawRect(0,0,240,320); 			//���ƾ���

	LCD_SetColor(LCD_RED);    LCD_FillCircle(140,50,30);		//���Բ��
	LCD_SetColor(LCD_GREEN);  LCD_FillCircle(170,50,30); 	
	LCD_SetColor(LCD_BLUE);   LCD_FillCircle(200,50,30);  	
	
	LCD_SetColor(LCD_YELLOW);
	LCD_DrawLine(26,26,113,64);				//��ֱ��
	LCD_DrawLine(35,22,106,81);				//��ֱ��
	LCD_DrawLine(45,20, 93,100);				//��ֱ��
	LCD_DrawLine(52,16, 69,108);				//��ֱ��
	LCD_DrawLine(62,16, 44,108);				//��ֱ��
	
	LCD_SetColor(LIGHT_CYAN);
	LCD_DrawCircle(120,170,30);			//����Բ��
	LCD_DrawCircle(120,170,20);   

	LCD_SetColor(LIGHT_RED);	
	LCD_DrawEllipse(120,170,90,40); 	//������Բ 
	LCD_DrawEllipse(120,170,70,40); 	//������Բ    
	LCD_SetColor(LIGHT_MAGENTA);	
	LCD_DrawEllipse(120,170,100,50); 	//������Բ
	LCD_DrawEllipse(120,170,110,60);  
	
	LCD_SetColor(DARK_RED);  	LCD_FillRect( 50, 250, 50, 50);			//������
	LCD_SetColor(DARK_BLUE); 	LCD_FillRect(100, 250, 50, 50);			//������	
	LCD_SetColor(LIGHT_GREEN); LCD_FillRect(150, 250, 50, 50);			//������	
	
	HAL_Delay(2000);		
}
/*************************************************************************************************
*	�� �� ��:	LCD_Test_Image
*
*	��������:	ͼƬ��ʾ����
*************************************************************************************************/
void LCD_Test_Image(void)
{
	LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
	LCD_Clear(); 								// ����
	
	LCD_SetColor( 0xffF6E58D);
	LCD_DrawImage( 19, 55, 83, 83, Image_Android_83x83) ;	   // ��ʾͼƬ

	LCD_SetColor( 0xffDFF9FB);
	LCD_DrawImage( 141, 55, 83, 83, Image_Message_83x83) ;	// ��ʾͼƬ
	
	LCD_SetColor( 0xff9DD3A8);
	LCD_DrawImage( 19, 175, 83, 83, Image_Toys_83x83) ;		// ��ʾͼƬ
	
	LCD_SetColor( 0xffFF8753);
	LCD_DrawImage( 141, 175, 83, 83, Image_Video_83x83) ;		// ��ʾͼƬ

	HAL_Delay(2000);	
}
/*************************************************************************************************
*	�� �� ��:	LCD_Test_Direction
*
*	��������:	������ʾ����
*************************************************************************************************/
void  LCD_Test_Direction(void)
{
	int i;
	for( i=0;i<4;i++ )
	{  
      LCD_SetBackColor(LCD_BLACK); 			//	���ñ���ɫ
      LCD_Clear(); 								// ����
      LCD_SetTextFont(&CH_Font24);  
	   LCD_SetColor( 0xffDFF9FB);         
		switch (i)		// �л�����ɫ
		{
			case 0:  
            LCD_SetDirection(Direction_V);		   
            LCD_DisplayText(20,20,"Direction_V"); 
         break;	

			case 1:  
            LCD_SetDirection(Direction_H); 	
            LCD_DisplayText(20,20,"Direction_H"); 
         break;	

			case 2:  
            LCD_SetDirection(Direction_V_Flip); 
            LCD_DisplayText(20,20,"Direction_V_Flip"); 
         break;
			case 3: 
            LCD_SetDirection(Direction_H_Flip); 	
            LCD_DisplayText(20,20,"Direction_H_Flip"); 
         break;
	
			default:	break;			
		}
      LCD_SetColor( 0xffF6E58D);
      LCD_DrawImage( 19, 80, 83, 83, Image_Android_83x83) ;	   // ��ʾͼƬ
      LCD_SetTextFont(&CH_Font32);
      LCD_SetColor( 0xff9DD3A8);  
      LCD_DisplayText(130,90,"����");
      LCD_DisplayText(130,130,"�Ƽ�");
      LCD_SetDirection(Direction_V);	
      HAL_Delay(1000);	// ��ʱ
	}
}

#define SIZE 4//matrix size

// ֱ��ʹ�ñ���������ָ��
 static float A[SIZE*SIZE] = {
		 4.00f, 1.00f, 2.00f, 1.00f,
     1.00f, 5.00f, 1.00f, 2.00f,
     2.00f, 1.00f, 6.00f, 3.00f,
     1.00f, 2.00f, 3.00f, 7.00f,
};
 static float B[SIZE*SIZE] = {
		 2.00f, 3.00f, 0.00f, 1.00f,
     0.00f, 3.00f, 2.00f, 0.00f,
     1.00f, 0.00f, 4.00f, 1.00f,
     0.00f, 2.00f, 0.00f, 5.00f,
};

// ��ӡ����
void print_matrix(UART_HandleTypeDef *huart, const char* name, float* data) {
    char buf[50];
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 100);
    HAL_UART_Transmit(huart, (uint8_t*)name, strlen(name), 100);
    
    for(int i=0; i<SIZE; i++) {
        for(int j=0; j<SIZE; j++) {
            sprintf(buf, "%8.6f ", data[i*SIZE + j]);
            HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), 100);
        }
        HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 100);
    }
}

void matrix_ops(UART_HandleTypeDef *huart) {
    // ��ʼ������ʵ��
    arm_matrix_instance_f32 matA, matB, matAdd, matMult, matInvA, matInvB;
    
    // ֱ�Ӿ�̬��������ڴ棨���⶯̬���䣩
     float AddResult[SIZE*SIZE];
     float MultResult[SIZE*SIZE];
     float InvA[SIZE*SIZE];
     float InvB[SIZE*SIZE];

    // ��ʼ������
    arm_mat_init_f32(&matA, SIZE, SIZE, A);
    arm_mat_init_f32(&matB, SIZE, SIZE, B);
    arm_mat_init_f32(&matAdd, SIZE, SIZE, AddResult);
    arm_mat_init_f32(&matMult, SIZE, SIZE, MultResult);
    arm_mat_init_f32(&matInvA, SIZE, SIZE, InvA);
    arm_mat_init_f32(&matInvB, SIZE, SIZE, InvB);

    // ����ӷ�
    if(arm_mat_add_f32(&matA, &matB, &matAdd) != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Add failed!\r\n", 12, 100);
    }

    // ����˷�
    if(arm_mat_mult_f32(&matA, &matB, &matMult) != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"Mult failed!\r\n", 13, 100);
    }

    // ��������
    if(arm_mat_inverse_f32(&matA, &matInvA) != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"A������\r\n", 12, 100);
    }
    if(arm_mat_inverse_f32(&matB, &matInvB) != ARM_MATH_SUCCESS) {
        HAL_UART_Transmit(huart, (uint8_t*)"B������\r\n", 12, 100);
    }

    // ������
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n===== ���������� =====\r\n", 28, 100);
    print_matrix(huart, "[A+B]", AddResult);
    print_matrix(huart, "[A��B]", MultResult);
    print_matrix(huart, "[A��]", InvA);
    print_matrix(huart, "[B��]", InvB);
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n=======================\r\n", 28, 100);
}




