/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H0AR9_i2c.c
 Description: Configures I2C2 peripheral for H0AR9 module.
 Features: Initializes I2C2 for 100 kHz communication in master mode with 7-bit addressing.
 GPIO: Sets up PA6 (SDA) and PA7 (SCL) as alternate function open-drain.
 Filters: Enables analog filter, disables digital filter and clock stretching.
 Functions: I2C initialization, MSP init/de-init for GPIO and clock management.
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include <string.h>
#include <stdio.h>

/* Exported Variables ******************************************************/
I2C_HandleTypeDef hi2c2;

/* Exported Functions ******************************************************/
void MX_I2C_Init(void);
void MX_I2C2_Init(void);

/***************************************************************************/
/* Configure I2C ***********************************************************/
/***************************************************************************/
void MX_I2C_Init(void) {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	MX_I2C2_Init();

}

/***************************************************************************/
/* I2C2 init function */
void MX_I2C2_Init(void) {

	/* Initialize I2C2 peripheral */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10B17DB5; // Normal mode (100 kHz)
	hi2c2.Init.OwnAddress1 = 0; // No specific address required for master mode
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7-bit addressing mode
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual address mode
	hi2c2.Init.OwnAddress2 = 0; // Not used, set to 0
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK; // No mask for second address
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Disable clock stretching
	HAL_I2C_Init(&hi2c2);

	/** Configure Analogue filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE); // Enable analog filter

	/** Configure Digital filter */
	HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0); // Digital filter set to 0 (disabled)
}

/***************************************************************************/
void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle){

	GPIO_InitTypeDef GPIO_InitStruct ={0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit ={0};

	/**I2C2 GPIO Configuration
	 PA6     ------> I2C2_SDA
	 PA7     ------> I2C2_SCL
	 */
	GPIO_InitStruct.Pin = SENSOR_I2C_SCL_PIN | SENSOR_I2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_I2C2;
	HAL_GPIO_Init(SENSOR_I2C_PORT, &GPIO_InitStruct);
	__HAL_RCC_I2C2_CLK_ENABLE();

}

/***************************************************************************/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle) {

	/* Peripheral clock disable */
	__HAL_RCC_I2C2_CLK_DISABLE();

	HAL_GPIO_DeInit(SENSOR_I2C_PORT, SENSOR_I2C_SCL_PIN);

	HAL_GPIO_DeInit(SENSOR_I2C_PORT, SENSOR_I2C_SDA_PIN);
}
/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
