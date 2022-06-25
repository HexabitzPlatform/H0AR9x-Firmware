/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name          : H0AR9_i2c.c
 Description        : This file provides code for the configuration
 of the I2C instances.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <string.h>
#include <stdio.h>

//initialize i2c
void MX_I2C_Init(void);
void MX_I2C1_Init(void);


I2C_HandleTypeDef hi2c1;
static const uint8_t colorProximityAdd = (0x39)<<1;
uint8_t receive[2];
uint8_t send[2];



/*----------------------------------------------------------------------------*/
/* Configure I2C                                                             */
/*----------------------------------------------------------------------------*/

/** I2C Configuration
*/
void MX_I2C_Init(void)
{
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();   // for HSE and Boot0

  MX_I2C1_Init();
}



void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c1.Instance = I2C2;
  hi2c1.Init.Timing = 0x2000090E/*Standard mode*/;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }


}

void WriteRegData(uint8_t reg, uint8_t data)
{
	send[0]= 0x80 | reg;
	send[1]= data;
	HAL_I2C_Master_Transmit(&hi2c1, colorProximityAdd, send, 2, HAL_MAX_DELAY);

}


uint16_t Read_Word(uint8_t reg)
{
   send[0]= 0xA0 | reg;
   HAL_I2C_Master_Transmit(&hi2c1, colorProximityAdd, send, 1, HAL_MAX_DELAY);
   HAL_I2C_Master_Receive(&hi2c1, colorProximityAdd, receive, 2, HAL_MAX_DELAY);
    return (uint16_t)(receive[0] + (256 * receive[1]));

}
/*-----------------------------------------------------------*/

