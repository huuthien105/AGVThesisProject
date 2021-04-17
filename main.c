#include "main.h"
#include "string.h"
#include "tm_stm32f4_adc.h"
#include "PID.h"
#include "QTR_5RC.h"
void TurnRight();
void TurnLeft();
void Reverse();
uint8_t g;
uint16_t flag_10ms = 0;
uint16_t flag_100ms = 0,a=0;
 float rightMotorSpeed;
 float leftMotorSpeed;
uint8_t testcb;
uint16_t adc;
extern float setVelocity;
float duty = 50;
uint8_t dir;
float x_ref = 5, x_measure;

float udk=0, delta_udk = 0,udk1=0;
uint8_t CardID[5] = { 0x38, 0x02, 0x03, 0x04, 0x05}; 
uint8_t ca = 0;
extern uint16_t sensorValues[5];

float v_left = 0, v_left_filter =0;
float	v_right=0,v_right_filter =0;
float v_ref = 10, v_measure =0;;

float v_lift = 0;

uint16_t line_position=0;
float f_line_position =0 ,line_ref =1000.0;

//uint16_t crc =0;
//uint8_t crc1,crc2;
uint8_t mess[6] ={ 0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};

float error =0,error_dot=0,pre_error=0,e_=0, edot_ =0,u_=0 ,u=0;
void TurnBack();
uint16_t QTR_ReadRawValue[6];
uint16_t QTR_ReadCalibValue[5];
float pos_left = 0,pos_right=0;
uint8_t PIDflag = 1;
typedef struct{
	uint16_t Header;
	uint8_t FunctionCode; // 0x01
	uint8_t AGVID;
	float Velocity;
	float Udk;
	float Line_Position;
	float delta_Udk;
	uint16_t EndOfFrame;
	
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
		PID_Init();
		//BKPSRAM_Init();
		TIM7_INT_Init();// QTR_5RC Init
		UART_Init();
		TIM6_INT_Init();
		TIM2_INT_Init();
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
	TestCB.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	TestCB.GPIO_PuPd =   GPIO_PuPd_UP  ;
	TestCB.GPIO_Speed = GPIO_Fast_Speed  ;
	GPIO_Init(GPIOB, &TestCB);
	
	//ADC_Config();
//		TM_ADC_Init(ADC1, ADC_Channel_3);
	//	TIM_SetCompare1(TIM3,1000);///16.13->3.3434
		 //QTRSensorsCalibrate();
		 //QTRSensorsCalibrate();
		//Run_Motor(LEFT_MOTOR,30);
		 //Run_Motor(RIGHT_MOTOR,30);
		 
		 //TurnLeft();
  while (1)
  {	
				
		if(tick_flag == 1) //1ms
			{
				 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
				//u = TM_ADC_ReadVbat(ADC1);
				if (TM_MFRC522_Check(CardID) == MI_OK) {
				//	ca = 1;
					//TurnLeft();
				//	TurnRight();
					// TurnBack();
					 Reverse();
					//Run_Motor(LEFT_MOTOR,0);
					//Run_Motor(RIGHT_MOTOR,0);
				}
				else ca =0;
				
				flag_100ms++;
				flag_10ms++;
				tick_flag = 0;
				//pos_right = ENC_Position(ENCR_TIM,363,pos_right);
				
			}
   if(flag_10ms == 10)
			{
					
					
				flag_10ms = 0;
				
			}
		if(flag_100ms == 1000) // Truyen nhan UART
			{
				
				//LCD_Clear();
				//char s[] = "Ab";
				//LCD_Puts("abcd");
				flag_100ms = 0;
				
			}
  }
}

void TIM6_DAC_IRQHandler()	// Overrides the weak implementation of the IRQ in the startup file
{
	TIM6->SR = ~TIM_SR_UIF;
	// Begin interrupt Timer6 10ms code
				v_left = ENC_Velocity(ENCL_TIM,363);	//PD12 PD 13
				v_left_filter = LowpassFilter(v_left,LEFT_MOTOR);
					
				v_right = ENC_Velocity(ENCR_TIM,363);	//PA0 PA1
				v_right_filter = LowpassFilter(v_right,RIGHT_MOTOR);
	
	
				v_lift = ENC_Velocity(TIM8,374);
				
				v_measure = (v_left_filter+v_right_filter)/2;
				udk = PID_Velocity(setVelocity,v_measure);
	      PID_Line(line_position, udk);
	
	
				__NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
}

void TIM2_IRQHandler()
{
    // Checks whether the TIM2 interrupt has occurred or not
    if (TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
       // Begin interrupt Timer3 100ms code
			
			  send_frame.Header = 0xFFAA;
				send_frame.FunctionCode = 0xA0;
				send_frame.AGVID = 1;
				send_frame.Velocity= v_measure;
				send_frame.Udk= udk;
				send_frame.Line_Position = line_position;
				send_frame.delta_Udk = 0;
				send_frame.EndOfFrame = 0x0A0D;
				UART_SendData((uint8_t *) &send_frame ,sizeof(send_frame)) ;
	
        // Clears the TIM2 interrupt pending bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void TurnRight()
{   
  PIDflag = 0;
	pos_right = 0;
	pos_left = 0;
	TIM_SetCounter(ENCR_TIM,30000);
	TIM_SetCounter(ENCL_TIM,30000) ;
	Run_Motor(LEFT_MOTOR,40);
	Run_Motor(RIGHT_MOTOR,-40);
	delay_ms(1000);
	
	
	/*while(pos_right < 12 || pos_left >-12)
	{
		pos_right = ENC_Position(ENCR_TIM,363,pos_right);
		pos_left = ENC_Position(ENCL_TIM,363,pos_left);
		delay_ms(1);
	}    */
	while(line_position < 300 || line_position > 1700)
	{
	  Run_Motor(LEFT_MOTOR,40);
  	Run_Motor(RIGHT_MOTOR,-40);
	}
	
		Run_Motor(LEFT_MOTOR,25);
		Run_Motor(RIGHT_MOTOR,25);
		delay_ms(750);
	PIDflag = 1;

}
void TurnLeft()
{ 
	PIDflag = 0;
	pos_right = 0;
	pos_left = 0;
	TIM_SetCounter(ENCR_TIM,30000);
	TIM_SetCounter(ENCL_TIM,30000) ;
	Run_Motor(LEFT_MOTOR,-45);
	Run_Motor(RIGHT_MOTOR,40);
	delay_ms(1000);
	
	
	/*while(pos_right < 12 || pos_left >-12)
	{
		pos_right = ENC_Position(ENCR_TIM,363,pos_right);
		pos_left = ENC_Position(ENCL_TIM,363,pos_left);
		delay_ms(1);
	}    */
	while(line_position < 300 || line_position > 1700)
	{
	  Run_Motor(LEFT_MOTOR,-45);
  	Run_Motor(RIGHT_MOTOR,40);
	}
	
		Run_Motor(LEFT_MOTOR,30);
		Run_Motor(RIGHT_MOTOR,30);
		delay_ms(450);
	PIDflag = 1;
}

void TurnBack()
{
  PIDflag = 0;
	Run_Motor(LEFT_MOTOR, 0);
	Run_Motor(RIGHT_MOTOR, 0);
	delay_ms(500);
		Run_Motor(LEFT_MOTOR, -30);
	Run_Motor(RIGHT_MOTOR, -30);
	delay_ms(3000);
		Run_Motor(LEFT_MOTOR, 0);
	Run_Motor(RIGHT_MOTOR, 0);
	delay_ms(500);
	PIDflag = 1;
	
}
void Reverse()
{ 
	PIDflag = 0;
	pos_right = 0;
	pos_left = 0;
	TIM_SetCounter(ENCR_TIM,30000);
	TIM_SetCounter(ENCL_TIM,30000) ;
	Run_Motor(LEFT_MOTOR,-45);
	Run_Motor(RIGHT_MOTOR,40);
	//delay_ms(1000);
	
	
	while(pos_right < 23 || pos_left >-23)
	{
		pos_right = ENC_Position(ENCR_TIM,363,pos_right);
		pos_left = ENC_Position(ENCL_TIM,363,pos_left);
		delay_ms(1);
	}    
/*	while(line_position < 300 || line_position > 1700)
	{
	  Run_Motor(LEFT_MOTOR,-45);
  	Run_Motor(RIGHT_MOTOR,40);
	}   */
	
		
		
	PIDflag = 1;
	delay_ms(1000);
}
