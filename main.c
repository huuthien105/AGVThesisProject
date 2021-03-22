#include "PWM.h"
#include "UART.h"
#include "Encoder.h"
#include "PID.h"
#include "Bkpsram.h"
#include "QTR_5RC.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_mfrc522.h"
#include "FuzzyLogic.h"
//#include "PID.h"
void Delay(int t);
void delay_us(uint16_t period);
void funcIntInit();
	void funcTimerInit();
uint8_t g;
uint16_t flag_10ms = 0;
uint16_t flag_100ms = 0,a=0;

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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
  // Config PWM
		
		//PWM_Init();
		//ENC_Init();
		//PID_Init();
		//BKPSRAM_Init();
		TIM_INT_Init();// QTR_5RC Init
		UART_Init();
		funcIntInit();
		funcTimerInit();
		//TM_MFRC522_Init();
	/* Initialize ADC1 */
	//TM_ADC_InitADC(ADC1);
	
	/* Enable vbat channel */
	//TM_ADC_EnableVbat();
// cconffig encoder
	//PID_Init1(1.25,0.5,0.1);
	//TM_BKPSRAM_WriteFloat(0x00, 10.25);
	//x_measure = TM_BKPSRAM_ReadFloat(0x00);
	//g = sizeof(x_measure);
	//Run_Motor(LEFT_MOTOR,FOWARD,50.0);
	//PID_Init();
	
	//for(int i = 0; i < 300; i++) QTRSensorsCalibrate();
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
					//v_left = ENC_Velocity(ENCL_TIM,363);	//PD12 PD 13
					//v_left_filter = LowpassFilter(v_left,LEFT_MOTOR);
					
				//	v_right = ENC_Velocity(ENCR_TIM,363);	//PA0 PA1
					//v_right_filter = LowpassFilter(v_right,RIGHT_MOTOR);
					
					//v_measure = (v_left_filter+v_right_filter)/2;
				
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
				//Run_Motor(LEFT_MOTOR,udk );
				//Run_Motor(RIGHT_MOTOR,udk );
				flag_10ms = 0;
				
			}
		if(flag_100ms == 1000) // Truyen nhan UART
			{
				//UART_PrintNumber("%f.2",TIM_GetCounter(TIM4));
				//send_frame.Header = 0xFFAA;
				//send_frame.FunctionCode = 0xA0;
				//send_frame.AGVID = 1;
				//send_frame.Velocity= c;
				//send_frame.Udk= 75.23;
				//send_frame.Line_Position = 1125.0;
				//send_frame.delta_Udk = 30.15;
				//USART_SendData(USART2, x[1]);
				//delay_01ms(1);
				//USART_SendData(USART2, x[2]);
					//UART_SendData((uint8_t *) &send_frame ,sizeof(send_frame) ) ;
				//UART_PrintNumber("%.2f",2.5);
				//UART_PrintNumber("%.2f",7.5);
				//UART_PrintNumber("%.2f",9.5);
				//UART_PrintNumber("%.2f",11.5);
				//UART_Print("AAAAAA");
				UART_PrintNumber("%.2f",2.5);
				c=c+10;;
				//delay_01ms(200);
				flag_100ms = 0;
			}
  }
}
void Delay(int t){
	int i;
	for (i=0; i<t; i++);
}




void funcIntInit()
{
	// Initialize the interrupt
	__NVIC_EnableIRQ(TIM6_DAC_IRQn);
 
	EXTI->IMR |= (1<<0);
	EXTI->EMR |= (1<<0);
	EXTI->FTSR &= ~(1<<0);
	EXTI->RTSR |= (1<<0);
 
	// SysCfg not needed for basic timer TIM6???
}
 
void funcTimerInit()
{
	// Peripheral clock
	RCC->APB1ENR |= (1<<4);	// Enables APB1 bus to use Timer 6
	// 2. Init timer
	TIM6->CR1 |= (1<<0);	// CEN: Enables counter
	TIM6->CR1 &= ~(1<<1);	// UDIS: Update event enabled
	TIM6->CR1 |= (1<<2);	// URS: Update request source enabled for overflow/underflow only
	TIM6->CR1 &= ~(1<<3);	// OPM: One Pulse Mode. Counter continues counting.
	TIM6->CR1 &= ~(1<<7);	// Auto reload preload enabled
	TIM6->DIER |= (1<<0);	// UIE: Update interrupt enabled
	TIM6->EGR |= (1<<0);	// UG: Update generation. Re-initializes the counter and updates registers
	//TIM6->CNT = 0x00FA;		// Counter goes up to 250 to have 1s timer ???
	TIM6->PSC = 0xFA00;		// Sets prescaler to 64000. Timer clock is now 16MHz/64000=250Hz
	TIM6->ARR = 0x00FA;		// Counter goes up to 250 to have 1s timer
 
 
}

void TIM6_DAC_IRQHandler()	// Overrides the weak implementation of the IRQ in the startup file
{
	// Toggle LED
	//UART_PrintNumber("%.2f",2.5);
	__NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
}