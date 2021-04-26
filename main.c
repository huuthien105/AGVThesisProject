#include "main.h"
#include "string.h"
#include "tm_stm32f4_adc.h"
#include "PID.h"
#include "QTR_5RC.h"

void TurnRight(void);
void TurnLeft(void);
void Reverse(void);
void TurnBack(void);
void StopAGV(void);
void RunPath(void);
uint8_t GetNode(void);
void ConvertOrient(uint8_t current_Orient);




uint8_t flagPathRunComplete = 1; 
uint8_t PIDflag = 1;
uint8_t flagCheckRFID = 0;
extern uint8_t flagPathReceived;

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
uint8_t currentNode,preNode =0;
uint8_t pre_Orient = 'N';
//uint8_t PathByteCount_Run = 9;
extern uint8_t PathByteCount_Run;
//uint8_t Path_Run[] = {'N', '0', 'A',4,'L',0, 'L',42, 'L', 46,'R',55, 'B',25,'R',28 ,'G', 'N', '0'};
//uint8_t Path_Run[] = {'N', '0', 'N',46,'E',50, 'N',8, 'W', 3,'S',11,'G', 'N', '0'};
//uint8_t Path_Run[] ={'N', '0', 'N',55,'N',46,'G', 'N', '0'};
extern uint8_t pathRunReceive[30];
uint8_t Path_Run[30];
	                  // 'N', '0', 'A', 46, 'A', 25, 'A',4,'L',0, 'L',21,'G', 'N', '0'};
char initExitNode = 55;
uint8_t test = 0;
uint8_t TagID[60][5] = {{0x99, 0x2E, 0x09, 0xF0, 0x4E},
												{0x69, 0x5C, 0x12, 0xEF, 0xC8},
												{0x49, 0x45, 0x09, 0xF0, 0xF5},
												{0x89, 0x64, 0x0C, 0xF0, 0x11},
												{0x89, 0x54, 0x12, 0xEF, 0x20},
												{0x99, 0x61, 0x12, 0xEF, 0x05},
												{0xB9, 0x57, 0x12, 0xEF, 0x13},
												{0xA9, 0x55, 0x12, 0xEF, 0x01},
												{0xA9, 0x56, 0x12, 0xEF, 0x02},
												{0x79, 0x0E, 0x0B, 0xF0, 0x8C},
												{0x69, 0x5B, 0x12, 0xEF, 0xCF},
												{0x59, 0x43, 0x09, 0xF0, 0xE3},
												{0xF9, 0x7B, 0x0A, 0xF0, 0x78},
												{0x89, 0x6F, 0x0A, 0xF0, 0x1C},
												{0x89, 0x6B, 0x0A, 0xF0, 0x18},
												{0x69, 0xE3, 0x09, 0xF0, 0x73},
												{0x69, 0xDF, 0x09, 0xF0, 0x4F},
												{0xE9, 0xCC, 0x09, 0xF0, 0xDC},
												{0xE9, 0xC8, 0x09, 0xF0, 0xD8},
												{0x49, 0x47, 0x09, 0xF0, 0xF7},
												{0x69, 0x41, 0x09, 0xF0, 0xD1},
												{0x29, 0x32, 0x09, 0xF0, 0xE2},
												{0x59, 0x1A, 0x07, 0xF0, 0xB4},
												{0x89, 0x17, 0x07, 0xF0, 0x69},
												{0xF9, 0x14, 0x07, 0xF0, 0x1A},
												{0x49, 0x9A, 0x06, 0xF0, 0x25},
												{0x79, 0x94, 0x06, 0xF0, 0x1B},
												{0x09, 0x99, 0x06, 0xF0, 0x66},
												{0xC9, 0x97, 0x06, 0xF0, 0xA8},
												{0x49, 0x96, 0x06, 0xF0, 0x29},
												{0x39, 0x92, 0x06, 0xF0, 0x5D},
												{0x59, 0x93, 0x06, 0xF0, 0x3C},
												{0xB9, 0x1B, 0x06, 0xF0, 0x54},
												{0xA9, 0x1B, 0x07, 0xF0, 0x45},
												{0x89, 0x95, 0x06, 0xF0, 0xEA},
												{0x49, 0x92, 0x06, 0xF0, 0x2D},
												{0xB9, 0x96, 0x06, 0xF0, 0xD9},
												{0x19, 0x99, 0x06, 0xF0, 0x76},
												{0xC9, 0x13, 0x07, 0xF0, 0x2D},
												{0x49, 0x16, 0x07, 0xF0, 0xA8},
												{0x99, 0x17, 0x07, 0xF0, 0x79},
												{0x09, 0x19, 0x07, 0xF0, 0xE7},
												{0xB9, 0x1B, 0x07, 0xF0, 0x55},
												{0x09, 0x99, 0x07, 0xF0, 0x67},
												{0x19, 0x9F, 0x07, 0xF0, 0x71},
												{0xE9, 0x9B, 0x07, 0xF0, 0x85},
												{0x19, 0xA2, 0x07, 0xF0, 0x4C},
												{0xA9, 0x1A, 0x08, 0xF0, 0x4B},
												{0x99, 0x1E, 0x08, 0xF0, 0x7F},
												{0xA9, 0x2E, 0x09, 0xF0, 0x7E},
												{0x09, 0x30, 0x08, 0xF0, 0xC1},
												{0x09, 0x2C, 0x08, 0xF0, 0xDD},
												{0x29, 0x62, 0x12, 0xEF, 0xB6},
												{0xA9, 0xAD, 0x08, 0xF0, 0xFC},
												{0xA9, 0xB1, 0x08, 0xF0, 0xE0},
												{0x19, 0xC0, 0x08, 0xF0, 0x21},
												{0x19, 0xC4, 0x08, 0xF0, 0x25},
												{0xC9, 0x61, 0x12, 0xEF, 0x55},
												{0x49, 0x69, 0x12, 0xEF, 0xDD},
												{0xF9, 0x98, 0x07, 0xF0, 0x96}}; 												
											
float error =0,error_dot=0,pre_error=0,e_=0, edot_ =0,u_=0 ,u=0;

uint16_t QTR_ReadRawValue[6];
uint16_t QTR_ReadCalibValue[5];
float pos_left = 0,pos_right=0;

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
  while (1)
  {	
				
		if(tick_flag == 1) //1ms
			{			
				 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
				//u = TM_ADC_ReadVbat(ADC1);
					 if(flagPathRunComplete == 1 && flagPathReceived == 1) // chay xong path hien tai && nhan frame path moi
           {
					    for(int i = 0; i< 30; i++) 
						  {
							 Path_Run[i] = pathRunReceive[i];          //luu path nhan duoc vao path_run
							}
					//	  pre_Orient = Path_Run[2];
							flagPathReceived = 0;
						  flagPathRunComplete = 0;
					 } 
				 if (TM_MFRC522_Check(CardID) == MI_OK)     // them vao && flagCheckRFID == 0 OR ket hop voi preCurrentNode
				 {
				   flagCheckRFID = 1;			
           RunPath();   
					// Reverse();

				}
				else flagCheckRFID =0;
				//	 currentNode = GetNode();
					
			/*	if(flagCheckRFID == 1)
				{
				       if(currentNode == 4 )
				       {
				          	TurnRight();
				       }
				       else if(currentNode == 25 || currentNode == 29 || currentNode == 50 ||currentNode == 46 || currentNode == 55  )
					        TurnRight();       
				}   */
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
	
//	flagCheckRFID = 0;
	PIDflag = 0;
	if(PIDflag == 0) flagCheckRFID = 0; 
//	pos_right = 0;
//	pos_left = 0;
	//TIM_SetCounter(ENCR_TIM,30000);
	//TIM_SetCounter(ENCL_TIM,30000) ;
	
	Run_Motor(LEFT_MOTOR, 0);
  Run_Motor(RIGHT_MOTOR, 0); 
	delay_ms(250);
	
	Run_Motor(LEFT_MOTOR,45);
	Run_Motor(RIGHT_MOTOR,0);
	delay_ms(1000);
	
/*	 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
	while(pos_right < 11.5 || pos_left >-11.5)
	{
		line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
		pos_right = ENC_Position(ENCR_TIM,363,pos_right);
		pos_left = ENC_Position(ENCL_TIM,363,pos_left);
		delay_ms(1);
	}      */
	
	 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
	while(line_position < 650 || line_position > 1350)
	{
		 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
	  Run_Motor(LEFT_MOTOR,45);
  	Run_Motor(RIGHT_MOTOR,0);
	}         
	//delay_ms(50);
//	Run_Motor(LEFT_MOTOR,0);
	//Run_Motor(RIGHT_MOTOR,0);
		//Run_Motor(LEFT_MOTOR,25);
		//Run_Motor(RIGHT_MOTOR,25);
	  //delay_ms(500);	
  	PIDflag = 1;
 	  //delay_ms(500);
}
void TurnLeft()
{ 
  //flagCheckRFID = 0;
	PIDflag = 0;
	if(PIDflag == 0) flagCheckRFID = 0; 
	//pos_right = 0;
	//pos_left = 0;
	//TIM_SetCounter(ENCR_TIM,30000);
	//TIM_SetCounter(ENCL_TIM,30000);
	
	Run_Motor(LEFT_MOTOR, 0);
  Run_Motor(RIGHT_MOTOR, 0); 
	delay_ms(250);
	
	Run_Motor(LEFT_MOTOR,0);   // - 39
	Run_Motor(RIGHT_MOTOR,47);
	delay_ms(1000);
	
	
	/*while(pos_right < 11.5 || pos_left >-11.5)
	{
		pos_right = ENC_Position(ENCR_TIM,363,pos_right);
		pos_left = ENC_Position(ENCL_TIM,363,pos_left);
		delay_ms(1);
	}      */
	 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
	while(line_position < 650 || line_position > 1350)
	{
		 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
	   Run_Motor(LEFT_MOTOR,0);
  	 Run_Motor(RIGHT_MOTOR,47);
	}
	//delay_ms(50);
	
	//	Run_Motor(LEFT_MOTOR,0);
		//Run_Motor(RIGHT_MOTOR,0);
	  //delay_ms(250);	
  	PIDflag = 1;
 	  //delay_ms(500);
}

void TurnBack()
{
  PIDflag = 0;
	Run_Motor(LEFT_MOTOR, 0);
	Run_Motor(RIGHT_MOTOR, 0);
	delay_ms(500);
		Run_Motor(LEFT_MOTOR, -30);
	Run_Motor(RIGHT_MOTOR, -30);
	delay_ms(2000);
		Run_Motor(LEFT_MOTOR, 0);
	Run_Motor(RIGHT_MOTOR, 0);
	delay_ms(500);
	PIDflag = 1;
	
}
void Reverse()
{ 
	PIDflag = 0;
	if(PIDflag == 0) flagCheckRFID = 0; 
	pos_right = 0;
	pos_left = 0;
	TIM_SetCounter(ENCR_TIM,30000);
	TIM_SetCounter(ENCL_TIM,30000) ;
	
	Run_Motor(LEFT_MOTOR, 0);
	Run_Motor(RIGHT_MOTOR, 0);
	delay_ms(500);
	
	Run_Motor(LEFT_MOTOR,-43);
	Run_Motor(RIGHT_MOTOR,41);
	//delay_ms(1900);
	
/*		 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
	while(line_position < 650 || line_position > 1350)
	{
		 line_position = QTRSensorsReadCalibrated(QTR_ReadCalibValue);
	   Run_Motor(LEFT_MOTOR,-41);
  	 Run_Motor(RIGHT_MOTOR,41);
	}
	*/
	
	while(pos_right < 23 || pos_left >-23)
	{
		pos_right = ENC_Position(ENCR_TIM,363,pos_right);
		pos_left = ENC_Position(ENCL_TIM,363,pos_left);
		delay_ms(1);
	} 
	
           
	Run_Motor(LEFT_MOTOR, 0);
	Run_Motor(RIGHT_MOTOR, 0);
	delay_ms(250);
		
	PIDflag = 1;
	//delay_ms(1000);
}

void StopAGV()
{
	PIDflag = 0;
	Run_Motor(LEFT_MOTOR, 0);
	Run_Motor(RIGHT_MOTOR, 0);
}

uint8_t GetNode()
{
	if(flagCheckRFID == 0) return currentNode;
	
   	for (int i = 0; i < 60; i++)
	  {
		  uint8_t compareID[5];
	  	for (int j = 0; j < 5; j++) compareID[j] = TagID[i][j];
		  if (TM_MFRC522_Compare(CardID, compareID) == MI_OK)   return i;
	  }
}

void RunPath()
{   
	
	 //PIDflag = 0;
	 currentNode = GetNode();	 

	 // tra ve dung node, cap nhat node hien tai
	 //exitNode = currentNode;	 
	// if (currentNode == initExitNode)
	// {
		// test++;
//	 }
	// else
	// {
		 // tim index cua node hien tai trong path_run
		 int indexCurrentNode = 0;
		 for (int i = 2; i < PathByteCount_Run  - 2; i++)         
		 {
			   
			   if (currentNode == Path_Run[i])
			   {
				     indexCurrentNode = i;
				     break;
			   }
		 }
		 // lay huong chay
		 uint8_t direction = Path_Run[indexCurrentNode + 1];
		 switch (direction)
		 { 
			 case 'N': 
			 {
					  switch(pre_Orient)
						{
						    case 'N': PIDflag = 1;  pre_Orient = 'N'; break;
							  case 'S':  Reverse();  pre_Orient = 'N'; break;
							  case 'W': TurnRight();  pre_Orient = 'N'; break;
						  	case 'E':  TurnLeft();  pre_Orient = 'N'; break;
						}
						break;
			 }
			 case 'S': 
			 {
					  switch(pre_Orient)
						{
						    case 'N':  Reverse(); pre_Orient = 'S';   break;
						  	case 'S':  PIDflag = 1; pre_Orient = 'S'; break;
						  	case 'W':  TurnLeft(); pre_Orient = 'S';  break;
						  	case 'E':  TurnRight(); pre_Orient = 'S';break;
						}
						break;
						
		   }
			 case 'W':
			 {
					  switch(pre_Orient)
						{
						    case 'N':  TurnLeft(); pre_Orient = 'W';  break;
							  case 'S':  TurnRight(); pre_Orient = 'W'; break;
							  case 'W':  PIDflag = 1; pre_Orient = 'W'; break;
							  case 'E':  Reverse();  pre_Orient = 'W'; break;
						}
						break;
		   }	
			 case 'E':
			 {
					  switch(pre_Orient)
						{
						    case 'N':  TurnRight(); pre_Orient = 'E'; break;
						  	case 'S':  TurnLeft(); pre_Orient = 'E'; break;
						  	case 'W':  Reverse(); pre_Orient = 'E';  break;
						  	case 'E':  PIDflag = 1; pre_Orient = 'E';break;
						}
						break;
			 }
			//  case 'A': {PIDflag = 1;   break;}
			 // case 'L': {TurnLeft();  break;}//	 case 'L': TurnLeft(); break;
			 // case 'R': {TurnRight(); break;}
			 // case 'B': {Reverse();  break;}
			  case 'G': {StopAGV();  flagPathRunComplete = 1;  break;}
			  default: {PIDflag = 1;    break;}
		 }	 
	 //}
}

void ConvertOrient(uint8_t current_Orient)
{
   if(pre_Orient == 'N')
	 {
		  switch(current_Orient)
	    {
				case 'N': PIDflag = 1; break;
				case 'E': TurnRight(); break;
				case 'W': TurnLeft(); break;
				case 'S': Reverse(); break;
			}			
		//	pre_Orient = current_Orient;
	 }
	 else if(pre_Orient == 'E')
	 {
		 switch(current_Orient)
	    {
				case 'N': TurnLeft(); break;
				case 'E': PIDflag = 1; break;
				case 'W': Reverse(); break;
				case 'S': TurnRight(); break;
			}			
			pre_Orient = current_Orient;
	 }
	 else if(pre_Orient == 'S')
	 {
		 switch(current_Orient)
	    {
				case 'N': Reverse(); break;
				case 'E': TurnLeft(); break;
				case 'W': TurnRight(); break;
				case 'S': PIDflag = 1; break;
			}			
			pre_Orient = current_Orient;
	 
	 }
	 
	  else if(pre_Orient == 'W')
	 {
		 switch(current_Orient)
	    {
				case 'N': TurnRight(); break;
				case 'E': Reverse(); break;
				case 'W': PIDflag = 1; break;
				case 'S': TurnLeft(); break;
			}			
			pre_Orient = current_Orient;
	 
	 }
	 
}
