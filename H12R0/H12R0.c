/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H12R0.c
    Description   : Source code for module H12R0.
									Indoors sensor hub: Temp and humidity (HDC1080DMBT), 
																			Proximity, RGB and ambient light (APDS-9950),
																			MEMS microphone (SPM1423HM4H-B)
																			PIR motion detector (EKMC1601111)		
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,4,5,6 for module ports.
			>> I2C2 for for HDC1080DMBT and APDS-9950.
			>> I2S2 for SPM1423HM4H-B.
			
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
Module_Status HDC_Init(void);
Module_Status EKMC_Init(void);
Module_Status SPM_Init(void);
Module_Status APDS_Init(void);


/* Create CLI commands --------------------------------------------------------*/


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H01R0 module initialization. 
*/
void Module_Init(void)
{

	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
	
	/* I2C 2 initialization */
	MX_I2C2_Init();
	
	/* Humidity and temperature sensor */
	HDC_Init();
	
	/* PIR motion detector */
	EKMC_Init();
	
	/* MEMS microphone */
	SPM_Init();
	
	/* Proximity, RGB and ambient light sensor */
	APDS_Init();
	
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

/* --- HDC1080DMBT humidity and temperature sensor initialization. 
*/
Module_Status HDC_Init(void)
{
	Module_Status result = H12R0_OK;

	
	return result;
}

/*-----------------------------------------------------------*/

/* --- EKMC1601111 PIR motion detector initialization. 
*/
Module_Status EKMC_Init(void)
{
	Module_Status result = H12R0_OK;

	
	return result;
}

/*-----------------------------------------------------------*/

/* --- SPM1423HM4H-B MEMS microphone initialization. 
*/
Module_Status SPM_Init(void)
{
	Module_Status result = H12R0_OK;

	
	return result;
}

/*-----------------------------------------------------------*/

/* --- APDS-9950 Proximity, RGB and ambient light sensor initialization. 
*/
Module_Status APDS_Init(void)
{
	Module_Status result = H12R0_OK;

	
	return result;
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
