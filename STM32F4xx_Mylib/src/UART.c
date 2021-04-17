#include "UART.h"
#include <string.h>
#include <stdio.h>
#define	  Tx_BUFF_SIZE		16
#define   Rx_BUFF_SIZE    1
void init_main(void);

uint8_t 	rxbuff[Rx_BUFF_SIZE], data_recieve[26];
char txbuff[Tx_BUFF_SIZE];	
float kp,ki,kd;
uint8_t rxdata=0, pre_rxdata;

int b = 0;
int on = 0;
int v_sv = 0;
float setVelocity = 20; 
float setki=0;
int16_t crc_cal=0, crc_receive =0;

uint8_t crc1=0,crc2=0;
float setkp_line =0, setki_line =0, setkd_line =0;
float setkp_speed =0, setki_speed =0, setkd_speed =0;
int flag_reciever =0, i=0;
void UART_Init(void)
{
	/* Create the typedef for GPIO,USART,DMA,NVIC */
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef   DMA_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;	
   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);



  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	/* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); 
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); 
	
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = Rx_BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* DMA1 Stream4 Channel4 for USART4 Tx configuration */			
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = Tx_BUFF_SIZE;
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream6, ENABLE);
	
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
}


void DMA1_Stream5_IRQHandler(void)
{
  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		
	pre_rxdata = rxdata;
	rxdata = rxbuff[0];
	
		if (flag_reciever == 1)
	{
		data_recieve[i] = rxdata;
		i++;
	}
	
	if (flag_reciever == 1 && pre_rxdata == 0x0A && rxdata == 0x0D ) 
	{
		
		if(data_recieve[0] == 0xAC && data_recieve[1] == 0x01 )
			{
				memcpy(&setkp_speed, &data_recieve[2], 4);
				memcpy(&setki_speed, &data_recieve[6], 4);
				memcpy(&setkd_speed, &data_recieve[10], 4);
				memcpy(&setVelocity, &data_recieve[14], 4);
			}
		
			else if(data_recieve[0] == 0xAD && data_recieve[1] == 0x01 )
			{
				memcpy(&setkp_line, &data_recieve[2], 4);
				memcpy(&setki_line, &data_recieve[6], 4);
				memcpy(&setkd_line, &data_recieve[10], 4);
				//memcpy(&setVelocity, &data_recieve[2], 4);
			}
		
		
		flag_reciever = 0;
		i =0;
		
		
	}	

	if ( flag_reciever==0 && pre_rxdata == 0xAA && rxdata == 0xFF)		flag_reciever = 1;

	
  //for(uint16_t i=0; i<Rx_BUFF_SIZE; i++)
    //Rx_Data[i] = rxbuff[i];
	

	
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
}
// Send simple data to UART
void UART_Print( const char *pcString ) 
{
 while(*pcString != '\0')
	{
		USART_SendData(USART2, *pcString);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		pcString++;
	}
 }
int check = -1;
 // Send multiple data to UART
void UART_PrintNumber(const char *templateC,float a)
{
	 check = snprintf(txbuff,8,templateC,a);
	 UART_Print(&txbuff[0]);
	 UART_Print("\r\n");
}

void UART_SendData( uint8_t *s_data, uint16_t size ) 
{
	
	for (int i=0; i < size;i++)
	{
		USART_SendData(USART2, *(s_data+i));
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	}
}
	



	uint16_t  CRC_Cal(uint8_t messageArray[],uint8_t size)
		{
		 uint16_t CRCFull = 0xFFFF;
            char CRCLSB;

            for (int i = 0; i < size ; i++)
            {
                CRCFull = (uint16_t)(CRCFull ^ messageArray[i]);

                for (int j = 0; j < 8; j++)
                {
                    CRCLSB = (char)(CRCFull & 0x0001);
                    CRCFull = (uint16_t)((CRCFull >> 1) & 0x7FFF);

                    if (CRCLSB == 1)
                        CRCFull = (uint16_t)(CRCFull ^ 0xA001);
                }
            }
						
		return CRCFull;
	}
		
	