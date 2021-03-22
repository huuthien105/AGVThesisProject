/*
 * Encoder.c
 *
 *  Created on: Jun 21, 2020
 *      Author: Nhan
 */

#include "QTR_5RC.h"

#define sensorCount 5
#define maxValue 1000
uint32_t tick = 0;
uint16_t sensorValues[sensorCount];
uint16_t sensorPins[sensorCount] = {GPIO_Pin_0,GPIO_Pin_1,GPIO_Pin_2,GPIO_Pin_3,GPIO_Pin_4};//GPIOA
char calibInitialized = '0'; // Calibration status
uint16_t minCalibValues[sensorCount]; // Lowest readings seen during calibration
uint16_t maxCalibValues[sensorCount]; // Highest readings seen during calibration
uint16_t position;

void TIM_INT_Init(void)
{
    // Enable clock for TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0 ;
    TIM_TimeBaseInitStruct.TIM_Period = 83 ;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    // Nested vectored interrupt settings
    // TIM2 interrupt is most important (PreemptionPriority and 
    // SubPriority = 0)
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
	// TIM2 initialize
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);
    // Enable TIM2 interrupt
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    // Start TIM2
    TIM_Cmd(TIM7, ENABLE);
}	

void TIM7_IRQHandler()
{
    // Checks whether the TIM2 interrupt has occurred or not
    if (TIM_GetITStatus(TIM7, TIM_IT_Update))
    {
        tick++;

        // Clears the TIM2 interrupt pending bit
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    }
}

void QTRPins_OUTPUT_Mode(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 
                          |GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
}
void QTRPins_INPUT_Mode(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 
                          |GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
}

//----------QTR Read Function-------------------------------
// Reads the raw values of the sensors
void QTRSensorsRead(uint16_t *_sensorValues)
{
	uint16_t __sensorValues[sensorCount];
	for(int i = 0; i < sensorCount; i++)
		__sensorValues[i] = maxValue; // maxValue = 1000
	// make sensor line an output
	QTRPins_OUTPUT_Mode();
	// drive sensor line high
	GPIO_WriteBit(GPIOD, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 
                          |GPIO_Pin_4, Bit_SET);
	// charge lines for 10 us
	// HAL_Delay(1);
	// disable interrupts so we can switch all the pins as close to the same
  // time as possible
	//HAL_TIM_Base_Stop_IT(&htim5);
	// Enable TIM2 interrupt
    TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
		TIM_Cmd(TIM7, DISABLE);
	// record start time before the first sensor is switched to input
  // (similarly, time is checked before the first sensor is read in the
  // loop below)
	uint32_t startTime = tick;
	uint16_t time = 0;
	// make sensor line an input (should also ensure pull-up is disabled)
	QTRPins_INPUT_Mode();
	// Enable TIM2 interrupt
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
TIM_Cmd(TIM7, ENABLE);
	while(time < maxValue)
	{
		// disable interrupts so we can read all the pins as close to the same
    // time as possible
		TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
		TIM_Cmd(TIM7, DISABLE);
		time = tick - startTime;
		for (int i = 0; i < sensorCount; i++)
    {
			if ((GPIO_ReadInputDataBit(GPIOD, sensorPins[i]) == Bit_RESET) && (time < __sensorValues[i]))
			{
				// record the first time the line reads low
				__sensorValues[i] = time;
				_sensorValues[i] = __sensorValues[i];
			}
    }
		TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM7, ENABLE);
	}
}

//-----------QTR Calibrate Function-------------------------
// Find the min & max values during calibration and store in min/maxCalibValues
void QTRSensorsCalibrate(void)
{
	uint16_t _sensorValues[sensorCount];
  static uint16_t maxSensorValues[sensorCount];
  static uint16_t minSensorValues[sensorCount];
	if(calibInitialized == '0')
	{
		// Initialize the max and min calibrated values to values that
    // will cause the first reading to update them.
		for(int i = 0; i < sensorCount; i++)
    {
      minCalibValues[i] = maxValue;
      maxCalibValues[i] = 0;
    }
		calibInitialized = '1';
	}
	
	for(int j = 0; j < 10; j++)
  {
    QTRSensorsRead(_sensorValues);
    for(int i = 0; i < sensorCount; i++)
    {
      // set the max we found THIS time
      if((j == 0) || (_sensorValues[i] > maxSensorValues[i]))
				maxSensorValues[i] = _sensorValues[i];
      // set the min we found THIS time
      if((j == 0) || (_sensorValues[i] < minSensorValues[i]))
        minSensorValues[i] = _sensorValues[i];
    }
  }
	// record the min and max calibration values
  for(int i = 0; i < sensorCount; i++)
  {
    if(maxSensorValues[i] > maxCalibValues[i])
			maxCalibValues[i] = maxSensorValues[i];
    if(minSensorValues[i] < minCalibValues[i])
      minCalibValues[i] = minSensorValues[i];
  }
}
//-----------QTR Read Calibrated Values Function------------
//Reads the sensors and provides calibrated values between 0 and 500.
void QTRSensorsReadCalibrated(uint16_t *_sensorValues)
{
	uint16_t __sensorValue[sensorCount];
	// if not calibrated, do nothing
	if(calibInitialized == '0') return;
	// read the needed values
	QTRSensorsRead(__sensorValue);
	for(int i = 0; i < sensorCount; i++)
	{
		uint16_t calibmin, calibmax;
		calibmin = minCalibValues[i];
		calibmax = maxCalibValues[i];
		uint16_t denominator = calibmax - calibmin;
		
    int16_t value = 0;
		if(denominator != 0)
      value = (((int32_t)__sensorValue[i]) - calibmin) * 500 / denominator;
		
    if(value < 0) {value = 0;}
    else if(value > 500) {value = 500;}

    _sensorValues[i] = value;
	}	
}
//-----------QTR Read Line Function-------------------------
// Note: This project only use 3 middle-sensor,
// so 'sensorCount-2' and _sensorValues[i+1] will be used
uint16_t QTRSensorsReadLine(uint16_t *_sensorValues)
{
	char onLine = '0';
  uint32_t avg = 0; // this is for the weighted total
  uint16_t sum = 0; // this is for the denominator, which is <= 64000
	static uint16_t _lastPosition = 0;
	QTRSensorsReadCalibrated(_sensorValues);
	for(int i = 0; i < sensorCount-2; i++)
	{
		uint16_t value = _sensorValues[i+1];
		// keep track of whether we see the line at all
		if(value > 100) {onLine = '1';}
		// only average in values that are above a noise threshold
		if(value > 50)
		{
			avg += (uint32_t)value * (i * 1000);
      sum += value;
		}
	}
	if(onLine == '0')
	{
		// If it last read to the left of center, return 0.
    if (_lastPosition < (sensorCount-2 - 1) * 1000 / 2)
      return 0;
    // If it last read to the right of center, return the max.
    else
      return (sensorCount-2 - 1) * 1000;
	}
	_lastPosition = avg / sum;
  return _lastPosition;
}
