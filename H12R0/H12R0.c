/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H12R0.c
    Description   : Source code for module H12R0.
									Indoors sensor hub: Temp and humidity (HDC1080DMBT), 
																			Proximity, RGB and ambient light Sensor (APDS-9950),
																			MEMS microphone (SPM1423HM4H-B)
																			PIR motion detector (EKMC1601111)		
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,4,5,6 for module ports.
			>> Timer 3 (Ch2-Ch4) for RGB PWM.
			>> GPIOB 0, GPIOB 1, GPIOA 7 for RGB LED.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/




/* Private function prototypes -----------------------------------------------*/	




/* Create CLI commands --------------------------------------------------------*/


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H01R0 module initialization. 
*/
void Module_Init(void)
{
	/* Peripheral clock enable */

	
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
	

}

/*-----------------------------------------------------------*/

/* --- H12R0 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H12R0_OK;
	
	switch (code)
	{

		
		default:
			result = H12R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART6)
		return P3;
	else if (huart->Instance == USART3)
		return P4;
	else if (huart->Instance == USART1)
		return P5;
	else if (huart->Instance == USART5)
		return P6;

	return 0;
}

/*-----------------------------------------------------------*/




/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/




/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/


/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
