/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
		
    File Name     : H12R0.c
    Description   : Header file for module H12R0.
									Indoors sensor hub: Temp and humidity (HDC1080DMBT), 
																			Proximity, RGB and ambient light Sensor (APDS-9950),
																			MEMS microphone (SPM1423HM4H-B)
																			PIR motion detector (EKMC1601111)														
																			
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H12R0_H
#define H12R0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H12R0_uart.h"	
#include "H12R0_gpio.h"	
#include "H12R0_dma.h"		
#include "H12R0_i2c.h"	
	
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H12R0

/* Port-related definitions */
#define	NumOfPorts		6
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
#define P3uart &huart6
#define P4uart &huart3
#define P5uart &huart1
#define P6uart &huart5
	
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT	GPIOA
#define	USART1_RX_PORT	GPIOA
#define	USART1_AF				GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT	GPIOA
#define	USART2_RX_PORT	GPIOA
#define	USART2_AF				GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT	GPIOB
#define	USART3_RX_PORT	GPIOB
#define	USART3_AF				GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT	GPIOA
#define	USART4_RX_PORT	GPIOA
#define	USART4_AF				GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT	GPIOB
#define	USART5_RX_PORT	GPIOB
#define	USART5_AF				GPIO_AF4_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT	GPIOA
#define	USART6_RX_PORT	GPIOA
#define	USART6_AF				GPIO_AF5_USART6

/* Module-specific Definitions */
#ifdef H12R0
	#define _I2C2_GPIO_PORT				GPIOA
	#define _I2C2_SDA_PIN					GPIO_PIN_12
	#define _I2C2_SCL_PIN					GPIO_PIN_11
	#define _I2C2_GPIO_AF					GPIO_AF5_I2C2
	
	/* HDC1080 section */
	#define HDC_ADD												0x40		/* 7-bit address */
	#define HDC_TEMP_REG_ADD							0x00
	#define HDC_HUMID_REG_ADD							0x01
	#define HDC_CONFIG_REG_ADD						0x02
	#define HDC_SERIAL_ID_3								0xFB
	#define HDC_SERIAL_ID_2								0xFC
	#define HDC_SERIAL_ID_1								0xFD
	#define HDC_MANUFACTURER_ID						0xFE
	#define HDC_DEVICE_ID									0xFF
	#define HDC_REG_SIZE									2
	#define HDC_REG_ADD_SIZE							1
	#define HDC_CONFIG_RESET(regval)						(regval |= 0x8000)		
	#define HDC_CONFIG_HEAT_ENABLE(regval)			(regval |= 0x2000)
	#define HDC_CONFIG_HEAT_DISABLE(regval)			(regval &= 0xDFFF)
	#define HDC_CONFIG_TEMP_AND_HUMID(regval)		(regval |= 0x1000)
	#define HDC_CONFIG_TEMP_OR_HUMID(regval)		(regval &= 0xEFFF)
	#define HDC_CONFIG_BAT_BELOW_2_8(regval)		(regval >> 11)		
	#define HDC_CONFIG_TEMP_14_BITS(regval)			(regval &= 0xFBFF)
	#define HDC_CONFIG_TEMP_11_BITS(regval)			(regval |= 0x0400)
	#define HDC_CONFIG_HUMID_14_BITS(regval)		(regval &= 0xFCFF)
	#define HDC_CONFIG_HUMID_11_BITS(regval)		(regval &= 0xFDFF)
	#define HDC_CONFIG_HUMID_8_BITS(regval)			(regval &= 0xFEFF)
	enum HDC_e{HDC_RESET, HDC_HEAT_ENABLE, HDC_HEAT_DISABLE, HDC_MODE_TEMP_AND_HUMID, HDC_MODE_TEMP_OR_HUMID,
									HDC_RES_14_BITS, HDC_RES_11_BITS, HDC_RES_8_BITS, HDC_TEMPERATURE, HDC_HUMIDITY, HDC_BOTH};
	
#endif

/* Module_Status Type Definition */  
typedef enum 
{
  H12R0_OK = 0,
	H12R0_ERR_UnknownMessage,
  H12R0_ERR_HDC1080_MID,
	H12R0_ERR_HDC1080_CONFIG,
	H12R0_ERR_HDC1080_READ,
	H12R0_ERR_HDC1080_WRITE,
	H12R0_ERROR = 255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT		GPIOA
#define _IND_LED_PIN		GPIO_PIN_7


/* HDC1080 configuration struct */
typedef struct
{
	uint8_t reset;				// Software Reset: HDC_RESET
	uint8_t heat;					// Heater element: HDC_HEAT_ENABLE, HDC_HEAT_DISABLE
	uint8_t mode;					// Mode of acquisition: HDC_MODE_TEMP_AND_HUMID, HDC_MODE_TEMP_OR_HUMID
	uint8_t temp_res;			// Temperature Measurement Resolution: HDC_RES_14_BITS, HDC_RES_11_BITS
	uint8_t humid_res;		// Humidity Measurement Resolution: HDC_RES_14_BITS, HDC_RES_11_BITS, HDC_RES_8_BITS
} HDC_config_t;

/* HDC1080 struct */
typedef struct
{
	HDC_config_t config;
	float temperature;
	float humidity;
} HDC_t;
extern HDC_t HDC;


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



/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   ----------------------------------------------------------------------- 
*/

//#define	CODE_H01R0_on							100


	
/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

#define HDC_Sample_Temperature()		HDC_Read(HDC_TEMPERATURE)
#define HDC_Sample_Humidity()				HDC_Read(HDC_HUMIDITY)
#define HDC_Sample_Sensors()				HDC_Read(HDC_BOTH)

extern Module_Status HDC_Config(void);
extern Module_Status HDC_Read(uint8_t sensor);

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



#endif /* H12R0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
