#include "main.h"
#include "string.h"
#include "ADC_Voltage.h"
uint8_t g;
uint16_t flag_10ms = 0;
uint16_t flag_100ms = 0,a=0;

uint8_t testcb;
uint8_t adc;

float duty = 50;
uint8_t dir;
float x_ref = 5, x_measure;

float udk=0, delta_udk = 0;
uint8_t CardID[5] = { 0x38, 0x02, 0x03, 0x04, 0x05}; 
uint8_t ca = 0;
extern uint16_t sensorValues[5];

float v_left = 0, v_left_filter =0;
float	v_right=0,v_right_filter =0;
float v_ref = 15, v_measure =0;;

float v_lift = 0;

uint16_t line_position=0;
float f_line_position =0 ,line_ref =1000.0;

float c=0;
//uint16_t crc =0;
//uint8_t crc1,crc2;
uint8_t mess[6] ={ 0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};

float error =0,error_dot=0,pre_error=0,e_=0, edot_ =0,u_=0 ,u=0;

typedef struct{
	uint16_t Header;
	uint8_t FunctionCode; // 0x01
	uint8_t AGVID;
	float Velocity;
	float Udk;
	float Line_Position;
	float delta_Udk;
	
}__attribute__((packed)) SendAGVInfoStruct;

SendAGVInfoStruct send_frame;

int main(void)
{
	
	/* Enable SysTick at 10ms interrupt */
	SysTick_Config(SystemCoreClock/1000);

  /* GPIOD Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Config PWM
	
		PWM_Init();
		ENC_Init();
		//PID_Init();
		//BKPSRAM_Init();
		//TIM7_INT_Init();// QTR_5RC Init
		UART_Init();
		//TIM6_INT_Init();
		//TIM2_INT_Init();
		TM_MFRC522_Init();
	/* Initialize ADC1 */
	//TM_ADC_InitADC(ADC1);
	LCD_Init();
	LCD_Clear();
	LCD_BackLight(1);
	LCD_Puts("STM32F407VGT6");
	LCD_NewLine();
	LCD_Puts("I2C: PA1 - PA2");
	/* Enable vbat channel */
	//TM_ADC_EnableVbat();
// cconffig encoder
	//PID_Init1(1.25,0.5,0.1);
	//TM_BKPSRAM_WriteFloat(0x00, 10.25);
	//x_measure = TM_BKPSRAM_ReadFloat(0x00);
	//g = sizeof(x_measure);
	//Run_Motor(LEFT_MOTOR,FOWARD,50.0);
	//PID_Init();
	//GPIO_WriteBit(PWM_GPIO_PORT,GPIO_Pin_7,1);	
	//for(int i = 0; i < 300; i++) QTRSensorsCalibrate();
	GPIO_InitTypeDef TestCB;
	TestCB.GPIO_Mode = GPIO_Mode_IN ;
	TestCB.GPIO_OType = GPIO_OType_OD;
	TestCB.GPIO_Pin = GPIO_Pin_14;
	TestCB.GPIO_PuPd =   GPIO_PuPd_DOWN  ;
	TestCB.GPIO_Speed = GPIO_Fast_Speed  ;
	GPIO_Init(GPIOD, &TestCB);
	
	ADC_Config();

  while (1)
  {	
				
		if(tick_flag == 1) //1ms
			{
				//u = TM_ADC_ReadVbat(ADC1);
				//if (TM_MFRC522_Check(CardID) == MI_OK) {
				 //ca = 1;
				//}
				//else ca =0;
				
				flag_10ms++;
				flag_100ms++;
				tick_flag = 0;
				
				
			}
   if(flag_10ms == 10)
			{
					v_left = ENC_Velocity(ENCL_TIM,363);	//PD12 PD 13
					//v_left_filter = LowpassFilter(v_left,LEFT_MOTOR);
					
				 v_right = ENC_Velocity(ENCR_TIM,363);	//PA0 PA1
				 v_lift = ENC_Velocity(TIM8,374);
					//v_right_filter = LowpassFilter(v_right,RIGHT_MOTOR);
					testcb = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14);
				adc = Read_ADC();
					//v_measure = (v_left_filter+v_right_filter)/2;
				//v_lift = ENC_Velocity(TIM8,374);
					// QTRSensorsCalibrate();
					//line_position = QTRSensorsReadLine(sensorValues);
				//f_line_position = (float)(line_position);
				//	udk = PID_Velocity(v_ref, v_measure);
					//delta_udk = PID_Line(line_ref,line_position); // line ref = 1000
					//Run_Motor(LEFT_MOTOR,udk-delta_udk);
					//Run_Motor(RIGHT_MOTOR,udk+delta_udk);
					//UART_PrintNumber("%f.2",a++);
				//PID_GA25_Lifting(x_ref,vs1);
				//TIM_SetCompare1(TIM3,50);
				
				//error = line_ref - f_line_position;
				//error_dot = error-pre_error;
				// Fuzzy PI Controller
			//	e_ = saturation(error/1000);
				//edot_= saturation((error_dot)/0.08);
				//u_=giaim_trongtam(e_,edot_);
				//u = sat_100(10*u_);
				//pre_error = error;
				Run_Motor(LEFT_MOTOR,30);
				Run_Motor(RIGHT_MOTOR,50);
				Run_Motor(LIFT_MOTOR,50);
				flag_10ms = 0;
				
			}
		if(flag_100ms == 1000) // Truyen nhan UART
			{
				//UART_PrintNumber("%f.2",TIM_GetCounter(TIM4));
				send_frame.Header = 0xFFAA;
				send_frame.FunctionCode = 0xA0;
				send_frame.AGVID = 1;
				send_frame.Velocity= 15.2;
				send_frame.Udk= 75.23;
				send_frame.Line_Position = 1125.0;
				send_frame.delta_Udk = 30.15;
				//USART_SendData(USART2, x[1]);
				//delay_01ms(1);
				//USART_SendData(USART2, x[2]);
				UART_SendData((uint8_t *) &send_frame ,sizeof(send_frame) ) ;
				//UART_PrintNumber("%.2f",2.5);
				//UART_PrintNumber("%.2f",7.5);
				//UART_PrintNumber("%.2f",9.5);
				//UART_PrintNumber("%.2f",11.5);
				//UART_Print("AAAAAA");
				//UART_PrintNumber("%.2f",v_lift);
				//c=c+10;;
				//delay_01ms(200);
				LCD_Clear();
				//char s[] = "Ab";
				LCD_Puts("abcd");
				flag_100ms = 0;
			}
  }
}

void TIM6_DAC_IRQHandler()	// Overrides the weak implementation of the IRQ in the startup file
{
	TIM6->SR = ~TIM_SR_UIF;
	// Begin interrupt Timer6 40ms code
	//UART_PrintNumber("%.2f",c);
	//c++;
	__NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
}

void TIM2_IRQHandler()
{
    // Checks whether the TIM2 interrupt has occurred or not
    if (TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
        // Begin interrupt Timer3 10ms code
      c++;
				//UART_PrintNumber("%.2f",3.5);
        // Clears the TIM2 interrupt pending bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

