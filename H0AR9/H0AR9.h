/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H0AR9.h
 Description   : Header file for module H0AR9.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H0AR9_H
#define H0AR9_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H0AR9_MemoryMap.h"
#include "H0AR9_uart.h"
#include "H0AR9_gpio.h"
#include "H0AR9_dma.h"
#include "H0AR9_inputs.h"
#include "H0AR9_eeprom.h"
#include "H0AR9_i2c.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H0AR9

/* Port-related Definitions */
#define	NUM_OF_PORTS	6
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available Ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart1
#define UART_P5 &huart5
#define UART_P6 &huart6

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* I2C Pin Definition */
#define PIR_INPUT_PIN       GPIO_PIN_6
#define PIR_INPUT_PORT      GPIOB

#define SENSOR_I2C_SCL_PIN  GPIO_PIN_7
#define SENSOR_I2C_SDA_PIN  GPIO_PIN_6
#define SENSOR_I2C_PORT     GPIOA

#define I2C_HANDLER         &hi2c2

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_14

/* Module-specific Macro Definitions ***************************************/
/* Registers Addresses */
#define CONTROL_REG          0x0F
#define Enable_REG           0x00
#define ATIME_REG            0x01
#define WTIME_REG            0x03
#define PPULSE_REG           0x0E
#define RED_REG              0x16
#define GREEN_REG            0x18
#define BLUE_REG             0x1A
#define DISTANCE_REG         0x1C
#define TEMP_REG             0x00
#define TEMP_HUM_REG        (0x40) << 1
#define HUMIDITY_REG         0x01
#define COLOR_PROXIMITY_REG (0x39) << 1

#define UNSNGD_HALF_WORD_MAX_VAL    0xFFFF
#define UNSNGD_HALF_WORD_MIN_VAL	0x0000

#define MIN_MEMS_PERIOD_MS			100
#define MAX_MEMS_TIMEOUT_MS			0xFFFFFFFF

#define NUM_MODULE_PARAMS		    7

#define STREAM_MODE_TO_PORT         1
#define STREAM_MODE_TO_TERMINAL     2

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum
{
  H0AR9_OK = 0,
  H0AR9_ERR_UnknownMessage,
  H0AR9_ERR_RGB,
  H0AR9_ERR_PROXIMITY,
  H0AR9_ERR_TEMPRATURE,
  H0AR9_ERR_HUMIDITY,
  H0AR9_ERR_PIR,
  H0AR9_ERR_BUSY,
  H0AR9_ERR_TIMEOUT,
  H0AR9_ERR_IO,
  H0AR9_ERR_TERMINATED,
  H0AR9_ERR_WRONGPARAMS,
  H0AR9_ERROR = 25
} Module_Status;

/* */
typedef enum {
	COLOR=0,
	PIR,
	DISTANCE,
	TEMPERATURE,
	HUMIDITY,
}All_Data;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SamplePIR(bool *pir);
Module_Status SampleDistance(uint16_t *distance);
Module_Status SampleColor(uint16_t *Red, uint16_t *Green, uint16_t *Blue);
Module_Status SampleTemperature(float *temperature);
Module_Status SampleHumidity(float *humidity);
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction);
Module_Status StreamToPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToTerminal(uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout);

#endif /* H0AR9_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
