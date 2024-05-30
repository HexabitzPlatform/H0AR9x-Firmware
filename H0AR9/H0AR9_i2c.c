/*
 BitzOS (BOS) V0.3.3 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name          : H0AR9_i2c.c
 Description        : This file provides code for the configuration
                      of the I2C instances.

  */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

#define Max_delay 100
I2C_HandleTypeDef hi2c2;

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

  MX_I2C2_Init();
}

//-- Configure indicator LED
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  /* hi2c2.Init.Timing = 0x2010091A; */ /* fast mode: 400 KHz */
  hi2c2.Init.Timing = 0x20303E5D; /* Standard mode: 100 KHz */
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);

    /**Configure Analogue filter
    */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);

    /**Configure Digital filter
    */
  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB13     ------> I2C2_SCL
    PB14     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB13     ------> I2C2_SCL
    PB14     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}
/*-----------------------------------------------------------*/

/**
* @brief Writes the supplied byte buffer to the device
*/
int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t *buff;

  buff = (uint8_t *)malloc(sizeof(uint8_t)*(count + 1));
  buff[0] = index;
  memcpy(&buff[1],pdata, sizeof(uint8_t)*count);

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, (count + 1), HAL_MAX_DELAY);

  free(buff);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads the requested number of bytes from the device
*/
int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count)
{
  HAL_StatusTypeDef result = HAL_ERROR;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, pdata, count, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Writes a single byte to the device
*/
int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t data)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[2];

  buff[0] = index;
  buff[1] = data;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, 2, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Writes a single word (16-bit unsigned) to the device
*/
int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[3];

  buff[0] = index;
  buff[1] = (data >> 8);
  buff[2] = data & 0xFF;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, 3, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Writes a single dword (32-bit unsigned) to the device
*/
int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[5];

  buff[0] = index;
  buff[1] = (data >> 24);
  buff[2] = (data >> 16);
  buff[3] = (data >> 8);
  buff[4] = data & 0xFF;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, 5, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads a single byte from the device
*/
int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata)
{
  HAL_StatusTypeDef result = HAL_ERROR;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, pdata, 1, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads a single word (16-bit unsigned) from the device
*/
int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[2];

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, buff, 2, HAL_MAX_DELAY);

  *pdata = buff[0];
  *pdata <<= 8;
  *pdata |= buff[1];

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads a single dword (32-bit unsigned) from the device
*/
int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[4];

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, buff, 4, HAL_MAX_DELAY);

  *pdata = buff[0];
  *pdata <<= 8;
  *pdata |= buff[1];
  *pdata <<= 8;
  *pdata |= buff[2];
  *pdata <<= 8;
  *pdata |= buff[3];

  return (uint32_t)result;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
