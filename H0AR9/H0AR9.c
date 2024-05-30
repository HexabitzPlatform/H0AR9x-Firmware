/*
 BitzOS (BOS) V0.3.3 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H0AR9.c
 Description   : Source code for module H0AR9.
 IR Time-if-Flight (ToF) Sensor (ST VL53L1CX)

 Required MCU resources :

 >> USARTs 1,2,3,4,5,6 for module ports (H0AR9).
 >> I2C2 for the ToF sensor.
 >> GPIOB 1 for ToF interrupt (INT).
 >> GPIOA 5 for ToF shutdown (XSHUT).

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <stdlib.h>


Module_Status statusD = H0AR9_OK;

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
extern I2C_HandleTypeDef hi2c2;
EventGroupHandle_t handleNewReadyData = NULL;


/* Module exported parameters ------------------------------------------------*/
float temp __attribute__((section(".mySection")));
float sample __attribute__((section(".mySection")));
module_param_t modParam[NUM_MODULE_PARAMS];

/* Private variables ---------------------------------------------------------*/
TaskHandle_t ToFHandle = NULL;
static const uint8_t colorProximityAdd = (0x39)<<1;
uint8_t coun;
uint16_t Dist;
uint8_t flag ;
uint8_t receive[2];
uint8_t send[2];




static bool stopStream = false;
/* Private function prototypes -----------------------------------------------*/
void ToFTask(void *argument);
Module_Status WriteRegData(uint8_t reg, uint8_t data);
Module_Status APDS9950_init(void);
/* Create CLI commands --------------------------------------------------------*/

///*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |                        Private Functions                              |
 -----------------------------------------------------------------------
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC
			| RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport,
		uint8_t outport) {

	uint8_t myOutport = 0, lastModule = 0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport = FindRoute(myID, dst);
	if (outport && dst == myID) { /* This is a 'via port' update and I'm the last module */
		myOutport = outport;
		lastModule = myID;
	} else if (outport == 0) { /* This is a remote update */
		if (NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if (src == myID) {
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		if (outport == 0)		// This is a remote module update
			sprintf((char*) pcOutputString, pcRemoteBootloaderUpdateMessage,
					dst);
		else
			// This is a 'via port' remote update
			sprintf((char*) pcOutputString,
					pcRemoteBootloaderUpdateViaPortMessage, dst, outport);

		strcat((char*) pcOutputString, pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport, myID, myOutport, myID, BIDIRECTIONAL,
			0xFFFFFFFF, 0xFFFFFFFF, false);
}

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port) {
	UART_HandleTypeDef *huart = GetUart(port);

	huart->Init.BaudRate = 57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* --- H0AR9 module initialization.
 */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	/* initialize GPIO for module */
	  SENSORS_GPIO_Init();
	  /* initialize I2C for module */
	  MX_I2C_Init();
	  /* initialize color&proximity sensor */
	  APDS9950_init();

	//Circulating DMA Channels ON All Module
	for (int i = 1; i <= NumOfPorts; i++) {
		if (GetUart(i) == &huart1) {
			index_dma[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			index_dma[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			index_dma[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* create a event group for measurement ranging */
	handleNewReadyData = xEventGroupCreate();


	/* Create a ToF task */
	xTaskCreate(ToFTask, (const char*) "ToFTask",
			(2 * configMINIMAL_STACK_SIZE), NULL,
			osPriorityNormal - osPriorityIdle, &ToFHandle);

}

/*-----------------------------------------------------------*/

/* --- Save array topology and Command Snippets in Flash RO ---
 */
uint8_t SaveToRO(void) {
	BOS_Status result = BOS_OK;
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 8;
	uint16_t temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] = { 0 };

	HAL_FLASH_Unlock();
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1, RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1, RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	if (FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if (myID) {
		temp = (uint16_t) (N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RO_START_ADDRESS, temp);
		//TOBECHECKED
		FlashStatus = FLASH_WaitForLastOperation(
				(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

		/* Save topology */
		for (uint8_t i = 1; i <= N; i++) {
			for (uint8_t j = 0; j <= MaxNumOfPorts; j++) {
				if (array[i - 1][0]) {

					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
							RO_START_ADDRESS + add, array[i - 1][j]);
					//HALFWORD 	//TOBECHECKED
					FlashStatus = FLASH_WaitForLastOperation(
							(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
						add += 8;
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for (uint8_t s = 0; s < numOfRecordedSnippets; s++) {
		if (snippets[s].cond.conditionType) {
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy((uint32_t*) &snipBuffer[1], (uint8_t*) &snippets[s],
					sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for (uint8_t j = 0; j < (sizeof(snippet_t) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd,
						*(uint64_t*) &snipBuffer[j * 8]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for (uint8_t j = 0; j < ((strlen(snippets[s].cmd) + 1) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd,
						*(uint64_t*) (snippets[s].cmd + j * 4));
				//HALFWORD
				//TOBECHECKED
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void) {
	// Clear the array
	memset(array, 0, sizeof(array));
	N = 1;
	myID = 0;

	return SaveToRO();
}

/* --- H0AR9 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
  Module_Status result = H0AR9_OK;
  uint32_t Numofsamples;
  uint32_t timeout;

  switch (code)
  {
//	case CODE_H0AR9_GET_INFO:
//		break;
//	case CODE_H0AR9_SAMPLE_PORT:
//		SampletoPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift]);
//		break;
//	case CODE_H0AR9_STREAM_PORT:
//			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] << 24);
//			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] << 24);
//			StreamDistanceToPort( cMessage[port-1][shift],cMessage[port-1][shift+1], Numofsamples, timeout);
//		break;
//	default:
//		result = H0AR9_ERR_UnknownMessage;
//		break;
  }

  return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void) {


}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART.
 */
uint8_t GetPort(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P3;
	else if (huart->Instance == USART1)
		return P4;
	else if (huart->Instance == USART5)
		return P5;
	else if (huart->Instance == USART6)
		return P6;

	return 0;
}

/*-----------------------------------------------------------*/

/* --- ToF streaming task 
 */

void ToFTask(void *argument) {
	/* Initialization Tof VL53L1 */
	Module_Status st;


	while (1) {

		// Process data when it's ready from the sensor or when the period timer is expired
//		if (tofState == REQ_MEASUREMENT_READY
//				|| (HAL_GetTick() - t0) >= tofPeriod) {
//			switch (tofMode) {
//			case SAMPLE_TOF:
//
//				if (tofModeMeasurement(Dev, PresetMode_User, DistanceMode_User,
//						InterruptMode_User, dynamicZone_s_User,
//						&ToFStructure_User) == STATUS_OK) {
//					statusD = H0AR9_OK;
//				} else {
//					statusD = H0AR9_ERROR;
//				}
//				Dist = ToFStructure_User.ObjectNumber[0].tofDistanceMm;
//
//				break;
//
//			default:
//				break;
//			}
//
//			t0 = HAL_GetTick();			// Reset the timer
//		}

//		tofState = REQ_IDLE;
		taskYIELD();
	}
}

/* -----------------------------------------------------------------------
 |                               APIs                                    |
 -----------------------------------------------------------------------
 */
uint16_t Read_Word(uint8_t reg)
{
   send[0]= 0xA0 | reg;
   HAL_I2C_Master_Transmit(&hi2c2, colorProximityAdd, send, 1, HAL_MAX_DELAY);
   HAL_I2C_Master_Receive(&hi2c2, colorProximityAdd, receive, 2, HAL_MAX_DELAY);
    return (uint16_t)(receive[0] + (256 * receive[1]));

}
Module_Status WriteRegData(uint8_t reg, uint8_t data)
 {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;
	send[0] = 0x80 | reg;
	send[1] = data;
	HAL_status = HAL_I2C_Master_Transmit(&hi2c2, colorProximityAdd, send, 2,
	HAL_MAX_DELAY);

	switch (HAL_status) {
	case HAL_ERROR:
		status = H0AR9_ERROR;
		break;
	case HAL_BUSY:
		status = H0AR9_ERR_BUSY;
		break;
	case HAL_OK:
		status = H0AR9_OK;
		break;
	default:
		break;
	}
	return status;

}
/*------------------------------------------------------------*/
//initialize APDS9950 sensor
Module_Status APDS9950_init(void)
 {
	Module_Status status = H0AR9_OK;
	uint8_t CONTROL, Enable, ATIME, WTIME, PPULSE;
	uint8_t redReg, greenReg, blueReg, distanceReg;
    //registers addresses
	CONTROL = 0x0F;
	Enable = 0x00;
	ATIME = 0x01;
	WTIME = 0x03;
	PPULSE = 0x0E;
	redReg = 0x16;
	greenReg = 0x18;
	blueReg = 0x1A;
	distanceReg = 0x1C;

	if (H0AR9_OK != WriteRegData(Enable, 0x00))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(ATIME, 0x00))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(WTIME, 0xff))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(PPULSE, 0x01))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(CONTROL, 0x20))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(Enable, 0x0F))
		return status = H0AR9_ERROR;

	return status;
}
/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
 |                             Commands                                  |
 -----------------------------------------------------------------------
 */


/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
