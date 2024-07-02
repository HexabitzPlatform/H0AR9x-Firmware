/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H0AR9.h
 Description   : Header file for module H0AR9.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H0AR9_H
#define H0AR9_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H0AR9_MemoryMap.h"
#include "H0AR9_uart.h"
#include "H0AR9_gpio.h"
#include "H0AR9_dma.h"
#include "H0AR9_inputs.h"
#include "H0AR9_eeprom.h"
#include "H0AR9_i2c.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H0AR9


/* Port-related definitions */
#define	NumOfPorts			6

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */

#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart6


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

/* Module-specific Definitions */
#define MIN_PERIOD_MS				100
#define UNSNGD_HALF_WORD_MAX_VAL    0xFFFF
#define UNSNGD_HALF_WORD_MIN_VAL	0x0000

#define MIN_MEMS_PERIOD_MS				100
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF

#define SAMPLE_TEM              0
#define SAMPLE_TO_PORT          1
#define STREAM_TO_PORT          2
#define STREAM_TO_Terminal      3
#define DEFAULT                 4

#define NUM_MODULE_PARAMS			  7
#define STOP_MEASUREMENT_RANGING      0
#define START_MEASUREMENT_RANGING     1

/* Module EEPROM Variables */

// Module Addressing Space 500 - 599
#define _EE_MODULE							500

/* Module_Status Type Definition */
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
  H0AR9_ERR_WrongParams,
  H0AR9_ERROR = 25
} Module_Status;

typedef enum {
	Color=0,
	PIR,
	Distance,
	Temperature,
	Humidity,

}All_Data;



/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_14

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
extern void ExecuteMonitor(void);

/* -----------------------------------------------------------------------
 |                               APIs                                    |
 -----------------------------------------------------------------------
 */
Module_Status SamplePIR(bool *pir);
Module_Status SampleDistance(uint16_t *distance);
Module_Status SampleColor(uint16_t *Red, uint16_t *Green, uint16_t *Blue);
Module_Status SampleTemperature(float *temperature);
Module_Status SampleHumidity(float *humidity);
Module_Status StreamtoPort(uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout);
Module_Status SampletoPort(uint8_t module,uint8_t port,All_Data function);
Module_Status StreamToTerminal(uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout);

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
/* -----------------------------------------------------------------------
 |                             Commands                                  |
 -----------------------------------------------------------------------
 */

#endif /* H0AR9_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
