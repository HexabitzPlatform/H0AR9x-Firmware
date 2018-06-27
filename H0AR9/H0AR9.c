/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H0AR9.c
    Description   : Source code for module H0AR9.
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
#include "utils.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

HDC_t HDC;
EKMC_t EKMC;


/* Private variables ---------------------------------------------------------*/

int print(const char *const format, ...)
{
	va_list vaarg;
	va_start(vaarg, format);
	
	char buffer[100];
	vsnprintf(buffer, sizeof(buffer)/sizeof(buffer[0]), format, vaarg);
	HAL_UART_Transmit(GetUart(P2), (uint8_t *)buffer, strlen(buffer), 100);
	
	va_end(vaarg);
	return 0;
}



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

/* --- H0AR9 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H0AR9_OK;
	
	switch (code)
	{

		
		default:
			result = H0AR9_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands 
*/
void RegisterModuleCLICommands(void)
{

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
	HAL_I2C_Master_Transmit(&hi2c2, HDC_ADD << 1, &reg, HDC_REG_ADD_SIZE, 10);
	HAL_I2C_Master_Receive(&hi2c2, HDC_ADD << 1, (uint8_t *)&manID, HDC_REG_SIZE, 10);
	//HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_MANUFACTURER_ID, HDC_REG_ADD_SIZE, (uint8_t *)&manID, HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_DEVICE_ID, HDC_REG_ADD_SIZE, (uint8_t *)&DevID, HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_SERIAL_ID_1, HDC_REG_ADD_SIZE, (uint8_t *)&serialID[0], HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_SERIAL_ID_2, HDC_REG_ADD_SIZE, (uint8_t *)&serialID[1], HDC_REG_SIZE, 10);
	HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_SERIAL_ID_3, HDC_REG_ADD_SIZE, (uint8_t *)&serialID[2], HDC_REG_SIZE, 10);
	
	/* Check Manufacturer ID */
	if (manID != 0x5449)	
		return H0AR9_ERR_HDC1080_MID;
	
	/* Configure for sequence acquisition, 14-bit temperature and 14-bit humidity */
	if (HDC_Config() != H0AR9_OK)
		return H0AR9_ERR_HDC1080_CONFIG;
	
	/* Read the sensors */
	if (HDC_Read(HDC_BOTH) != H0AR9_OK)
		return H0AR9_ERR_HDC1080_READ;
	
	return H0AR9_OK;
}

/*-----------------------------------------------------------*/

/* --- EKMC1601111 PIR motion detector initialization. 
*/
// PRECAUTION: Max. Output Current of the sense
Module_Status EKMC_Init(void)
{
	Module_Status result = H0AR9_OK;
	memset(&EKMC, 0, sizeof(EKMC));
	// This sensor requires 30s initialization time. We need to add provision to return errors on
	// reading this sensor before 30s.
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- SPM1423HM4H-B MEMS microphone initialization. 
*/
Module_Status SPM_Init(void)
{
	Module_Status result = H0AR9_OK;

	
	return result;
}

/*-----------------------------------------------------------*/

/* --- APDS-9950 Proximity, RGB and ambient light sensor initialization. 
*/
#if 0

// Static Inline Function to convert time in miliseconds to 2nd complement form to be saved in a register.
// Used for Integration and Wait time.
__STATIC_INLINE uint8_t APDS_ConvertMSTo2Comp(int ms)
{
	return (uint8_t)((256 - (ms / 2.4f)) + 0.5f);
}

// Static Inline Function to conver 2nd complement form to time in miliseconds to to be saved in a register.
// Used for Integration and Wait time.
__STATIC_INLINE uint8_t APDS_Convert2CompToMS(int comp2)
{
	return (uint8_t)((2.4f * (256 - comp2)) + 0.5f);
}

__STATIC_INLINE Module_Status APDS_SetProxThres(uint16_t lowThreshold, uint16_t highThreshold)
{
	uint8_t buffer[5];
	buffer[0] = APDS_TRX_AUTO_INC(APDS_PILTL_ADDR);	// Auto Increment Mode
	buffer[1] = lowByte(lowThreshold);
	buffer[2] = highByte(lowThreshold);
	buffer[3] = lowByte(highThreshold);
	buffer[4] = highByte(highThreshold);
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	
	return H0AR9_OK;
}

__STATIC_INLINE Module_Status APDS_SetClearThres(uint16_t lowThreshold, uint16_t highThreshold)
{
	uint8_t buffer[5];
	buffer[0] = APDS_TRX_AUTO_INC(APDS_AILTL_ADDR); // Auto Increment Mode
	buffer[1] = lowByte(lowThreshold);
	buffer[2] = highByte(lowThreshold);
	buffer[3] = lowByte(highThreshold);
	buffer[4] = highByte(highThreshold);
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	
	return H0AR9_OK;
}

__STATIC_INLINE Module_Status APDS_setWaitTime(bool enableLongWait, uint8_t waitTimeMs)
{
	uint8_t buffer[2];
	// Configuring WLONG Mode
	buffer[0] = APDS_CFG_ADDR;
	buffer[1] = (enableLongWait ? (APDS_WLONG_EN) : (APDS_WLONG_DISABLE));
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	
	// Configuring Delay
	buffer[0] = APDS_WTIME_ADDR;
	buffer[1] = APDS_ConvertMSTo2Comp(waitTimeMs);
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	
	return H0AR9_OK;
}

__STATIC_INLINE Module_Status APDS_setProxPulse(uint8_t numPulse)
{
	uint8_t buffer[2];
	buffer[0] = APDS_PPULSE_ADDR;
	buffer[1] = numPulse;
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	return H0AR9_OK;
}

__STATIC_INLINE Module_Status APDS_CfgReg(uint8_t again, uint8_t ledDrive)
{
	uint8_t controlReg = (ledDrive & APDS_PDRIVE_MASK) | APDS_PDIODE_IR | APDS_PGAIN_1X | (again & APDS_AGAIN_MASK);
	uint8_t buffer[2];
	buffer[0] = APDS_CTRL_ADDR;
	buffer[1] = controlReg;
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	return H0AR9_OK;
}

__STATIC_INLINE Module_Status APDS_ReadID(uint8_t *id)
{
	if (HAL_I2C_Mem_Read(&hi2c2, APDS_I2C_ADDR << 1, APDS_ID_ADDR, 1, id, sizeof(*id), 100) != HAL_OK)
		return H0AR9_ERROR;
	return H0AR9_OK;
}

Module_Status APDS_Start(void)
{
	uint8_t buffer[2];
	buffer[0] = APDS_EN_ADDR;
	buffer[1] = APDS_EN_BIT_PON | APDS_EN_BIT_AEN | APDS_EN_BIT_PEN;
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	
	return H0AR9_OK;
}

// The sensor won't stop immediately if doing some ADC operations of past command.
Module_Status APDS_Stop(void)
{
	uint8_t buffer[2];
	buffer[0] = APDS_EN_ADDR;
	buffer[1] = 0x00;
	
	if (HAL_I2C_Master_Transmit(&hi2c2, APDS_I2C_ADDR << 1, buffer, sizeof(buffer), 100) != HAL_OK)
		return H0AR9_ERROR;
	
	return H0AR9_OK;
}

Module_Status APDS_Reset(void)
{
	return APDS_Stop();
}


Module_Status APDS_CheckID(void)
{
	Module_Status result = H0AR9_OK;
	uint8_t id;
	
	if ((result = APDS_ReadID(&id)) != H0AR9_OK)
		return result;
	if (id != APDS_ID_ADDR)
		return H0AR9_ERROR;
	
	return H0AR9_OK;
}

Module_Status APDS_ReadStatus(uint8_t *status)
{
	if (HAL_I2C_Mem_Read(&hi2c2, APDS_I2C_ADDR << 1, APDS_STAT_ADDR, 1, status, sizeof(*status), 100) != HAL_OK)
		return H0AR9_ERROR;
	
	return H0AR9_OK;
}

Module_Status APDS_Read(void)
{
	Module_Status result = H0AR9_OK;
	uint8_t regAddr = APDS_TRX_AUTO_INC(APDS_CDATAL_ADDR);
	uint8_t values[10];
	
	// Checking if results are valid.
	uint8_t statusReg;
	if ((result = APDS_ReadStatus(&statusReg)) != H0AR9_OK)
		return result;
	
	if (!APDS_STAT_IS_AVALID(statusReg) || !(APDS_STAT_IS_PVALID(statusReg)))
		return H0AR9_ERROR;
	
	// TODO: ADD Auto Inc TRX Mode
	if (HAL_I2C_Mem_Read(&hi2c2, APDS_I2C_ADDR << 1, regAddr, sizeof(regAddr), values, sizeof(values), 100) != HAL_OK)
		return H0AR9_ERROR;

	APDS.clear = concatBytes(values[0], values[1]);
	APDS.red = concatBytes(values[2], values[3]);
	APDS.green = concatBytes(values[4], values[5]);
	APDS.blue = concatBytes(values[6], values[7]);
	APDS.proximity = concatBytes(values[8], values[9]);
	
	return H0AR9_OK;
}

Module_Status APDS_Init(void)
{
	Module_Status result = H0AR9_OK;
	if ((result = APDS_Reset()) != H0AR9_OK)
		return result;
	if ((result = APDS_CheckID()) != H0AR9_OK)
		return result;
	
	// 2.4ms delay
	return result;
}

Module_Status APDS_Config(void)
{
	Module_Status result = H0AR9_OK;

	return result;
}

#else

Module_Status APDS_Init(void)
{
	Module_Status result = H0AR9_OK;
	return result;
}

Module_Status APDS_Config(void)
{
	Module_Status result = H0AR9_OK;

	return result;
}

#endif

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
	if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD << 1, HDC_CONFIG_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) == HAL_OK)
		return H0AR9_OK;
	else
		return H0AR9_ERR_HDC1080_WRITE;
}

/*-----------------------------------------------------------*/

/* --- Read HDC1080DMBT humidity and temperature sensors. 
*/
Module_Status HDC_Read(uint8_t sensor)
{
	Module_Status result = H0AR9_OK;
	uint16_t reg[2] = {0};
	
	/* Trigger measurements */
	if (HDC.config.mode == HDC_MODE_TEMP_AND_HUMID && sensor == HDC_BOTH)
	{
		if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD << 1, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, 2*HDC_REG_SIZE, 10) != HAL_OK)
			return H0AR9_ERR_HDC1080_WRITE;		
	}
	else if (HDC.config.mode == HDC_MODE_TEMP_OR_HUMID)
	{
		if (sensor == HDC_TEMPERATURE)
		{
			if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD << 1, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H0AR9_ERR_HDC1080_WRITE;		
		}
		else if (sensor == HDC_HUMIDITY)
		{
			if (HAL_I2C_Mem_Write(&hi2c2, HDC_ADD << 1, HDC_HUMID_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H0AR9_ERR_HDC1080_WRITE;		
		}
	}
	
	/* Wait until measurements are ready */
	Delay_ms(10);
	
	/* Read sensor data */
	if (HDC.config.mode == HDC_MODE_TEMP_AND_HUMID && sensor == HDC_BOTH)
	{
		if (HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, 2*HDC_REG_SIZE, 10) != HAL_OK)
			return H0AR9_ERR_HDC1080_READ;		
		
		HDC.temperature = ((float)reg[0]/65536.0f) * 165.0f - 40.0f;
		HDC.humidity = ((float)reg[1]/65536.0f) * 100.0f;
	}
	else if (HDC.config.mode == HDC_MODE_TEMP_OR_HUMID)
	{
		if (sensor == HDC_TEMPERATURE)
		{
			if (HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_TEMP_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H0AR9_ERR_HDC1080_READ;	

			HDC.temperature = ((float)reg[0]/65536.0f) * 165.0f - 40.0f;			
		}
		else if (sensor == HDC_HUMIDITY)
		{
			if (HAL_I2C_Mem_Read(&hi2c2, HDC_ADD << 1, HDC_HUMID_REG_ADD, HDC_REG_ADD_SIZE, (uint8_t *)&reg, HDC_REG_SIZE, 10) != HAL_OK)
				return H0AR9_ERR_HDC1080_READ;

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
