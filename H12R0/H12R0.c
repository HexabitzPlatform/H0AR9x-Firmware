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

HDC_t HDC;

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
	uint16_t manID = 0, DevID = 0, serialID[3] = {0};
	uint8_t reg = 0;
	
	/* Read IDs */
	reg = HDC_MANUFACTURER_ID;
	HAL_I2C_Master_Transmit(&hi2c2, HDC_ADD, &reg, HDC_REG_ADD_SIZE, 10);
	HAL_I2C_Master_Receive(&hi2c2, HDC_ADD, (uint8_t *)&manID, HDC_REG_SIZE, 10);
	//HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_MANUFACTURER_ID, HDC_REG_ADD_SIZE, (uint8_t *)&manID, HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_DEVICE_ID, HDC_REG_ADD_SIZE, (uint8_t *)&DevID, HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_SERIAL_ID_1, HDC_REG_ADD_SIZE, (uint8_t *)&serialID[0], HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_SERIAL_ID_2, HDC_REG_ADD_SIZE, (uint8_t *)&serialID[1], HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_SERIAL_ID_3, HDC_REG_ADD_SIZE, (uint8_t *)&serialID[2], HDC_REG_SIZE, 10);
	
	/* Check Manufacturer ID */
	if (manID != 0x5449)	
		return H12R0_ERR_HDC1080_MID;
	
	/* Configure for sequence acquisition, 14-bit temperature and 14-bit humidity */
	if (HDC_Config() != H12R0_OK)
		return H12R0_ERR_HDC1080_CONFIG;
	
	/* Read the sensors */
	if (HDC_Read(HDC_BOTH) != H12R0_OK)
		return H12R0_ERR_HDC1080_READ;
	
	return H12R0_OK;
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

/* --- Configure HDC1080DMBT. 
*/
Module_Status HDC_Config(void)
{
	uint16_t reg = 0;
	
	/* Parse the configuration 
	*/
	if (HDC.config.reset == HDC_RESET)
		HDC_CONFIG_RESET(reg);
	
	if (HDC.config.heat == HDC_HEAT_ENABLE)
		HDC_CONFIG_HEAT_ENABLE(reg);		
	else if (HDC.config.heat == HDC_HEAT_DISABLE)
		HDC_CONFIG_HEAT_DISABLE(reg);	
	
	if (HDC.config.mode == HDC_MODE_TEMP_AND_HUMID)
		HDC_CONFIG_TEMP_AND_HUMID(reg);		
	else if (HDC.config.mode == HDC_MODE_TEMP_OR_HUMID)
		HDC_CONFIG_TEMP_OR_HUMID(reg);	
	
	if (HDC.config.temp_res == HDC_RES_14_BITS)
		HDC_CONFIG_TEMP_14_BITS(reg);		
	else if (HDC.config.temp_res == HDC_RES_11_BITS)
		HDC_CONFIG_TEMP_11_BITS(reg);	
	
	if (HDC.config.humid_res == HDC_RES_14_BITS)
		HDC_CONFIG_HUMID_14_BITS(reg);		
	else if (HDC.config.humid_res == HDC_RES_11_BITS)
		HDC_CONFIG_HUMID_11_BITS(reg);	
	else if (HDC.config.humid_res == HDC_RES_8_BITS)
		HDC_CONFIG_HUMID_8_BITS(reg);	
	
	/* Write the CONFIG register */
	if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD, HDC_CONFIG_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) == HAL_OK)
		return H12R0_OK;
	else
		return H12R0_ERR_HDC1080_WRITE;
}

/*-----------------------------------------------------------*/

/* --- Read HDC1080DMBT humidity and temperature sensors. 
*/
Module_Status HDC_Read(uint8_t sensor)
{
	Module_Status result = H12R0_OK;
	uint16_t reg[2] = {0};
	
	/* Trigger measurements */
	if (HDC.config.mode == HDC_MODE_TEMP_AND_HUMID && sensor == HDC_BOTH)
	{
		if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, 2*HDC_REG_SIZE, 10) != HAL_OK)
			return H12R0_ERR_HDC1080_WRITE;		
	}
	else if (HDC.config.mode == HDC_MODE_TEMP_OR_HUMID)
	{
		if (sensor == HDC_TEMPERATURE)
		{
			if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H12R0_ERR_HDC1080_WRITE;		
		}
		else if (sensor == HDC_HUMIDITY)
		{
			if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD, HDC_HUMID_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H12R0_ERR_HDC1080_WRITE;		
		}
	}
	
	/* Wait until measurements are ready */
	Delay_ms(10);
	
	/* Read sensor data */
	if (HDC.config.mode == HDC_MODE_TEMP_AND_HUMID && sensor == HDC_BOTH)
	{
		if (HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, 2*HDC_REG_SIZE, 10) != HAL_OK)
			return H12R0_ERR_HDC1080_READ;		
		
		HDC.temperature = ((float)reg[0]/65536.0f) * 165.0f - 40.0f;
		HDC.humidity = ((float)reg[1]/65536.0f) * 100.0f;
	}
	else if (HDC.config.mode == HDC_MODE_TEMP_OR_HUMID)
	{
		if (sensor == HDC_TEMPERATURE)
		{
			if (HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H12R0_ERR_HDC1080_READ;	

			HDC.temperature = ((float)reg[0]/65536.0f) * 165.0f - 40.0f;			
		}
		else if (sensor == HDC_HUMIDITY)
		{
			if (HAL_I2C_Mem_Read(&hi2c2, HDC_ADD, HDC_HUMID_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H12R0_ERR_HDC1080_READ;

			HDC.humidity = ((float)reg[1]/65536.0f) * 100.0f;
		}
	}
	
	return result;
}


/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/


/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
