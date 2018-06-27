/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
		
    File Name     : H0AR9.c
    Description   : Header file for module H0AR9.
									Indoors sensor hub: Temp and humidity (HDC1080DMBT), 
																			Proximity, RGB and ambient light Sensor (APDS-9950),
																			MEMS microphone (SPM1423HM4H-B)
																			PIR motion detector (EKMC1601111)														
																			
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H0AR9_H
#define H0AR9_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H0AR9_uart.h"	
#include "H0AR9_gpio.h"	
#include "H0AR9_dma.h"		
#include "H0AR9_i2c.h"	
	
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H0AR9

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
#ifdef H0AR9
	#define _I2C2_GPIO_PORT				GPIOA
	#define _I2C2_SDA_PIN					GPIO_PIN_12
	#define _I2C2_SCL_PIN					GPIO_PIN_11
	#define _I2C2_GPIO_AF					GPIO_AF5_I2C2
	
	#define _EKMC_PIR_PORT				GPIOB
	#define _EKMC_PIR_PIN					GPIO_PIN_7
	
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
	
									
	/* APDS-9950 */
	// TODO: Add Persistance Support, Interrupt, Wait Time capabilites.
									
	#define APDS_TRX_REP_BYTE(regval)						(regval & (0b10011111))
	#define APDS_TRX_AUTO_INC(regval)						((regval & (0b10011111)) | (0b00100000))
	
	#define APDS_CMD_PI_CLR											0b11100101
	#define APDS_CMD_CI_CLR											0b11100110
	#define APDS_CMD_PCI_CLR										0b11100111
									
	#define APDS_I2C_ADDR												0x39
									
	// Default Type of Transaction: Repeated Byte Protocol Transaction.
	#define APDS_EN_ADDR												0x80	// Reset Value: 0x00
	#define APDS_ATIME_ADDR          						0x81	// Reset Value: 0xFF
	#define APDS_WTIME_ADDR				          		0x83	// Reset Value: 0xFF
	#define APDS_AILTL_ADDR						          0x84	// Reset Value: 0x00
	#define APDS_AILTH_ADDR						          0x85	// Reset Value: 0x00
	#define APDS_AIHTL_ADDR						          0x86	// Reset Value: 0x00
	#define APDS_AIHTH_ADDR						          0x87	// Reset Value: 0x00
	#define APDS_PILTL_ADDR						          0x88	// Reset Value: 0x00
	#define APDS_PILTH_ADDR						          0x89	// Reset Value: 0x00
	#define APDS_PIHTL_ADDR						          0x8A	// Reset Value: 0x00
	#define APDS_PIHTH_ADDR						          0x8B	// Reset Value: 0x00
	#define APDS_PERS_ADDR						          0x8C	// Reset Value: 0x00
	#define APDS_CFG_ADDR	        							0x8D	// Reset Value: 0x00
	#define APDS_PPULSE_ADDR         						0x8E	// Reset Value: 0x00
	#define APDS_CTRL_ADDR        							0x8F	// Reset Value: 0x00
	#define APDS_ID_ADDR             						0x92
	#define APDS_STAT_ADDR         							0x93	// Reset Value: 0x00
	#define APDS_CDATAL_ADDR						        0x94	// Reset Value: 0x00
	#define APDS_CDATAH_ADDR						        0x95	// Reset Value: 0x00
	#define APDS_RDATAL_ADDR						        0x96	// Reset Value: 0x00
	#define APDS_RDATAH_ADDR						        0x97	// Reset Value: 0x00
	#define APDS_GDATAL_ADDR						        0x98	// Reset Value: 0x00
	#define APDS_GDATAH_ADDR					         	0x99	// Reset Value: 0x00
	#define APDS_BDATAL_ADDR						        0x9A	// Reset Value: 0x00
	#define APDS_BDATAH_ADDR						        0x9B	// Reset Value: 0x00
	#define APDS_PDATAL_ADDR						        0x9C	// Reset Value: 0x00
	#define APDS_PDATAH_ADDR						        0x9D	// Reset Value: 0x00
	
	
	/* APDS-9950 Enable Register Bit fields */
	#define APDS_EN_BIT_PON            			0b00000001
	#define APDS_EN_BIT_AEN            			0b00000010
	#define APDS_EN_BIT_PEN            			0b00000100
	#define APDS_EN_BIT_WEN            			0b00001000
	#define APSD_EN_BIT_AIEN           			0b00010000
	#define APDS_EN_BIT_PIEN           			0b00100000
	
	
	/* APDS-9950 Status Register Bit fields */
	#define APDS_STAT_IS_AVALID(regValue)   bitRead(regValue,0)
	#define APDS_STAT_IS_PVALID(regValue)		bitRead(regValue,1)
	#define APDS_STAT_IS_AINT(regValue) 		bitRead(regValue,4)
	#define APDS_STAT_IS_PINT(regValue) 		bitRead(regValue,5)


	/* APDS-9950 LED Drive values */
	#define APDS_PDRIVE_MASK								0b11000000
	
	#define APDS_PDRIVE_100MA         			0b00000000
	#define APDS_PDRIVE_50MA          			0b01000000
	#define APDS_PDRIVE_25MA          			0b10000000
	#define APDS_PDRIVE_12_5MA        			0b11000000
	
	/* APDS-9950 AGain values */
	#define APDS_AGAIN_MASK									0b00000011
	
	#define APDS_AGAIN_1X                		0b00000000
	#define APDS_AGAIN_4X                		0b00000001
	#define APDS_AGAIN_16X               		0b00000010
	#define APDS_AGAIN_60X               		0b00000011
	
	/* APDS-9950 PGain values */
	#define APDS_PGAIN_MASK									0b00001100
	
	#define APDS_PGAIN_1X                		0b00000000

	/* APDS-9950 PDiode values */
	#define APDS_PDIODE_MASK								0b00110000
	
	#define APDS_PDIODE_IR               		0b00100000
	
	/* APDS-9950 WLONG values */
	#define APDS_WLONG_MASK									0b00000010
	
	#define APDS_WLONG_EN               		0b00000010
	#define APDS_WLONG_DISABLE           		0b00000000
	
	/* APDS-9950 ID Register */
	#define APDS_ID_REG_VALUE								0x69
	
		/* APDS-9950 configuration struct */
	typedef struct {
		bool enWait;	// WEN = 1, PEN = 1, AEN = 0 is unsupported and will lead to erroneous proximity readings.
		bool enRGBC;
		bool enProximity;
		
		uint8_t intTimeMS;
	} APDS_config_t;

/* APDS-9950 struct */
	typedef struct {
		APDS_config_t config;
		
		uint16_t red;
		uint16_t green;
		uint16_t blue;
		uint16_t clear;
		uint16_t proximity;
	} APDS_t;
	
	extern APDS_t APDS;
	
	/* EKMC1601111 Specific Macros, enum, struct and APIs */

	/* EKMC1601111 struct */
	typedef struct {
		unsigned numOfDetections;
		unsigned lengthOfLastDectMs;
	} EKMC_t;
	extern EKMC_t EKMC;
	
#endif

/* Module_Status Type Definition */  
typedef enum 
{
  H0AR9_OK = 0,
	H0AR9_ERR_UnknownMessage,
  H0AR9_ERR_HDC1080_MID,
	H0AR9_ERR_HDC1080_CONFIG,
	H0AR9_ERR_HDC1080_READ,
	H0AR9_ERR_HDC1080_WRITE,
	H0AR9_ERR_APDS9950_MID,
	H0AR9_ERR_APDS9950_CONFIG,
	H0AR9_ERR_APDS9950_READ,
	H0AR9_ERR_APDS9950_WRITE,
	H0AR9_ERR_APDS9950_BUSY,
	H0AR9_ERROR = 255
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



#endif /* H0AR9_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
