/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name: H0AR9.c
 Description: Source code for H0AR9 module with ST VL53L1CX ToF IR sensor.
 MCU Resources:
 - UARTs: USART1-6 for six ports (P1-P6).
 - I2C2: For APDS9950 sensor (color, proximity, temperature, humidity).
 - GPIO: PB5 (PIR input), Timer8 (stream timer).
 CLI: "sample" and "stream" commands for color, distance, humidity, temperature, and PIR data.
 Functions: Initializes peripherals, samples sensor data, and supports sampling and streaming to ports/terminal/buffer.
 */


/* Includes ****************************************************************/
#include "BOS.h"
#include <stdlib.h>

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

All_Data PortFunction;
All_Data TerminalFunction;

TimerHandle_t xTimerStream = NULL;

extern I2C_HandleTypeDef hi2c2;

/* Private Variables *******************************************************/
/* I2C Buffer */
uint8_t TXBuffer[2];
uint8_t RXBuffer[2];

/* Streaming variables */
static bool stopStream = false;         /* Flag to indicate whether to stop streaming process */
uint8_t PortModule = 0u;                /* Module ID for the destination port */
uint8_t PortNumber = 0u;                /* Physical port number used for streaming */
uint8_t StreamMode = 0u;                /* Current active streaming mode (to port, terminal, etc.) */
uint8_t TerminalPort = 0u;              /* Port number used to output data to a terminal */
uint8_t StopeCliStreamFlag = 0u;        /* Flag to request stopping a CLI stream operation */
uint32_t SampleCount = 0u;              /* Counter to track the number of samples streamed */
uint32_t PortNumOfSamples = 0u;         /* Total number of samples to be sent through the port */
uint32_t TerminalNumOfSamples = 0u;     /* Total number of samples to be streamed to the terminal */

/* Global variables for sensor data used in ModuleParam */
bool H0AR9_pir = false;
uint16_t H0AR9_red = 0;
uint16_t H0AR9_green = 0;
uint16_t H0AR9_blue = 0;
uint16_t H0AR9_distance = 0;
float H0AR9_temp = 0.0f;
float H0AR9_humidity = 0.0f;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={
    { .ParamPtr = &H0AR9_pir,      .ParamFormat = FMT_BOOL,   .ParamName = "pir" },
    { .ParamPtr = &H0AR9_distance, .ParamFormat = FMT_UINT16, .ParamName = "distance" },
    { .ParamPtr = &H0AR9_red,      .ParamFormat = FMT_UINT16, .ParamName = "red" },
    { .ParamPtr = &H0AR9_green,    .ParamFormat = FMT_UINT16, .ParamName = "green" },
    { .ParamPtr = &H0AR9_blue,     .ParamFormat = FMT_UINT16, .ParamName = "blue" },
    { .ParamPtr = &H0AR9_temp,     .ParamFormat = FMT_FLOAT,  .ParamName = "temperature" },
    { .ParamPtr = &H0AR9_humidity, .ParamFormat = FMT_FLOAT,  .ParamName = "humidity" }
};

/* Local Typedef related to stream functions */
typedef void (*SampleToString)(char *, size_t);
typedef void (*SampleToBuffer)(float *buffer);

/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
Module_Status APDS9950_init(void);
Module_Status WriteRegData(uint8_t reg, uint8_t data);
Module_Status Read_Word(uint8_t reg , uint16_t *Data );

/* Stream Functions */
void StreamTimeCallback(TimerHandle_t xTimerStream);

void SamplePIRToString(char *cstring, size_t maxLen);
void SampleDistanceToString(char *cstring, size_t maxLen);
void SampleTemperatureToString(char *cstring, size_t maxLen);
void SampleHumidityToString(char *cstring, size_t maxLen);
void SampleColorToString(char *cstring, size_t maxLen);

void SamplePIRBuf(float *buffer);
void SampleDistanceBuf(float *buffer);
void SampleColorBuf(float *buffer);
void SampleTemperatureBuf(float *buffer);
void SampleHumidityBuf(float *buffer);

//Module_Status Exportstreamtoterminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port,All_Data function);
//Module_Status ExportStreamToPort (uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout);
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction);

static Module_Status ExportToTerminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port, SampleToString function);
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function);
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples);

/* Create CLI commands *****************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "sample:\r\n Syntax: sample [color]/[distance]/[temp]/[humidity]/[pir].\r\n\r\n",
	SampleSensorCommand,
	1
};

/***************************************************************************/
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [color]/[distance]/[temp]/[humidity]/[pir] (period in ms) (time in ms) [port] [module].\r\n\r\n",
	StreamSensorCommand,
	-1
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to RXBuffer */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status StreamCLIFlag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the StreamCLIFlag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;

	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port) {

	UART_HandleTypeDef *huart = GetUart(port);

	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

}

/***************************************************************************/
/* H0AR9 module initialization */
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

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* Create a timeout software timer StreamSamplsToPort() API */
	xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(1000),pdTRUE,(void* )1,StreamTimeCallback);

}

/***************************************************************************/
/* H0AR9 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src,
		uint8_t dst, uint8_t shift) {
	Module_Status result = H0AR9_OK;
	uint32_t Numofsamples;
	uint32_t timeout;

	switch (code) {
	case CODE_H0AR9_SAMPLE_COLOR:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], COLOR);
		break;

	case CODE_H0AR9_SAMPLE_DISTANCE:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], DISTANCE);
		break;

	case CODE_H0AR9_SAMPLE_TEMP:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], TEMPERATURE);
		break;

	case CODE_H0AR9_SAMPLE_HUMIDITY:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], HUMIDITY);
		break;

	case CODE_H0AR9_SAMPLE_PIR:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], PIR);
		break;

	default:
		result = H0AR9_ERR_UnknownMessage;
		break;
	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
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

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);

}

/***************************************************************************/
/* Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
	Module_Status status = H0AR9_OK;

	switch (paramIndex) {
	/* Sample PIR sensor (convert bool to float) */
	case 1: {
		bool temp = false;
		status = SamplePIR(&temp);
		if (status == H0AR9_OK)
			*value = (float) temp;
		break;
	}

		/* Sample Distance sensor */
	case 2: {
		uint16_t temp = 0;
		status = SampleDistance(&temp);
		if (status == H0AR9_OK)
			*value = (float) temp;
		break;
	}

		/* Sample Color - Red */
	case 3: {
		uint16_t temp = 0;
		status = SampleColor(&temp, NULL, NULL);
		if (status == H0AR9_OK)
			*value = (float) temp;
		break;
	}

		/* Sample Color - Green */
	case 4: {
		uint16_t temp = 0;
		status = SampleColor(NULL, &temp, NULL);
		if (status == H0AR9_OK)
			*value = (float) temp;
		break;
	}

		/* Sample Color - Blue */
	case 5: {
		uint16_t temp = 0;
		status = SampleColor(NULL, NULL, &temp);
		if (status == H0AR9_OK)
			*value = (float) temp;
		break;
	}

		/* Sample Temperature */
	case 6:
		status = SampleTemperature(value);
		break;

		/* Sample Humidity */
	case 7:
		status = SampleHumidity(value);
		break;

		/* Invalid parameter index */
	default:
		status = H0AR9_ERR_WRONGPARAMS;
		break;
	}

	return status;
}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* Callback function triggered by a timer to manage data streaming.
 * xTimerStream: Handle of the timer that triggered the callback.
 */
void StreamTimeCallback(TimerHandle_t xTimerStream) {
	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if (STREAM_MODE_TO_PORT == StreamMode) {
		if ((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)) {
			SampleToPort(PortModule, PortNumber, PortFunction);

		} else {
			SampleCount = 0;
			xTimerStop(xTimerStream, 0);
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if (STREAM_MODE_TO_TERMINAL == StreamMode) {
		if ((SampleCount <= TerminalNumOfSamples)
				|| (0 == TerminalNumOfSamples)) {
			SampleToTerminal(TerminalPort, TerminalFunction);
		} else {
			SampleCount = 0;
			xTimerStop(xTimerStream, 0);
		}
	}
}

/***************************************************************************/
/* Samples PIR sensor data into a buffer.
 * buffer: Pointer to the buffer where PIR data will be stored.
 */
void SamplePIRBuf(float *buffer) {
    bool pir;
    SamplePIR(&pir);
    *buffer = (float)pir;
}

/***************************************************************************/
/* Samples distance sensor data into a buffer.
 * buffer: Pointer to the buffer where distance data will be stored.
 */
void SampleDistanceBuf(float *buffer) {
    uint16_t distance;
    SampleDistance(&distance);
    *buffer = (float)distance;
}

/***************************************************************************/
/* Samples color sensor data into a buffer.
 * buffer: Pointer to the buffer where color data will be stored.
 */
void SampleColorBuf(float *buffer) {
    uint16_t color[3];
    SampleColor(color, color + 1, color + 2);
    buffer[0] = (float)color[0];
    buffer[1] = (float)color[1];
    buffer[2] = (float)color[2];
}

/***************************************************************************/
/* Samples temperature sensor data into a buffer.
 * buffer: Pointer to the buffer where temperature data will be stored.
 */
void SampleTemperatureBuf(float *buffer) {
    float temperature;
    SampleTemperature(&temperature);
    *buffer = temperature;
}

/***************************************************************************/
/* Samples humidity sensor data into a buffer.
 * buffer: Pointer to the buffer where humidity data will be stored.
 */
void SampleHumidityBuf(float *buffer) {
    float humidity;
    SampleHumidity(&humidity);
    *buffer = humidity;
}

/***************************************************************************/
/* Streams a single sensor data sample to the terminal.
 * dstPort: Port number to stream data to.
 * dataFunction: Function to sample data (e.g., PIR, DISTANCE, COLOR, TEMPERATURE, HUMIDITY).
 */
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction) {
    Module_Status Status = H0AR9_OK; /* Initialize operation status as success */
    int8_t *PcOutputString = NULL; /* Pointer to CLI output buffer */
    uint32_t Period = 0u; /* Calculated period for the operation */
    char CString[100] = {0}; /* Buffer for formatted output string */
    bool pir = false; /* Variable for PIR data */
    uint16_t distance = 0; /* Variable for distance data */
    uint16_t red = 0, green = 0, blue = 0; /* Variables for color data */
    float temperature = 0.0f; /* Variable for temperature data */
    float humidity = 0.0f; /* Variable for humidity data */

    /* Process data based on the requested sensor function */
    switch (dataFunction) {
        case PIR:
            /* Get the CLI output buffer for writing */
            PcOutputString = FreeRTOS_CLIGetOutputBuffer();
            /* Sample PIR data */
            if (SamplePIR(&pir) != H0AR9_OK) {
                return H0AR9_ERROR; /* Return error if sampling fails */
            }
            /* Format PIR data into a string */
            snprintf(CString, 50, "PIR | %s\r\n", pir ? "Detected" : "Not Detected");
            /* Send the formatted string to the specified port */
            writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
            break;

        case DISTANCE:
            /* Get the CLI output buffer for writing */
            PcOutputString = FreeRTOS_CLIGetOutputBuffer();
            /* Sample distance data in millimeters */
            if (SampleDistance(&distance) != H0AR9_OK) {
                return H0AR9_ERROR; /* Return error if sampling fails */
            }
            /* Format distance data into a string */
            snprintf(CString, 50, "Distance(mm) | %u\r\n", distance);
            /* Send the formatted string to the specified port */
            writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
            break;

        case COLOR:
            /* Get the CLI output buffer for writing */
            PcOutputString = FreeRTOS_CLIGetOutputBuffer();
            /* Sample color data (Red, Green, Blue) */
            if (SampleColor(&red, &green, &blue) != H0AR9_OK) {
                return H0AR9_ERROR; /* Return error if sampling fails */
            }
            /* Format color data into a string */
            snprintf(CString, 50, "Color | R: %u, G: %u, B: %u\r\n", red, green, blue);
            /* Send the formatted string to the specified port */
            writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
            break;

        case TEMPERATURE:
            /* Get the CLI output buffer for writing */
            PcOutputString = FreeRTOS_CLIGetOutputBuffer();
            /* Sample temperature data in Celsius */
            if (SampleTemperature(&temperature) != H0AR9_OK) {
                return H0AR9_ERROR; /* Return error if sampling fails */
            }
            /* Format temperature data into a string */
            snprintf(CString, 50, "Temp(Celsius) | %.2f\r\n", temperature);
            /* Send the formatted string to the specified port */
            writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
            break;

        case HUMIDITY:
            /* Get the CLI output buffer for writing */
            PcOutputString = FreeRTOS_CLIGetOutputBuffer();
            /* Sample humidity data in percentage */
            if (SampleHumidity(&humidity) != H0AR9_OK) {
                return H0AR9_ERROR; /* Return error if sampling fails */
            }
            /* Format humidity data into a string */
            snprintf(CString, 50, "Humidity(%%) | %.2f\r\n", humidity);
            /* Send the formatted string to the specified port */
            writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
            break;

        default:
            /* Return error for invalid sensor function */
            return H0AR9_ERR_WRONGPARAMS;
    }

    /* Return final status indicating success or prior error */
    return Status;
}

/***************************************************************************/
/* Streams sensor data to a buffer.
 * buffer: Pointer to the buffer where data will be stored.
 * Numofsamples: Number of samples to take.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the sampling function (e.g., SamplePIRBuf, SampleColorBuf).
 */
static Module_Status StreamToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleToBuffer function) {
    Module_Status status = H0AR9_OK;
    uint16_t StreamIndex = 0;
    uint32_t period = timeout / Numofsamples;

    /* Check if the calculated period is valid */
    if (period < MIN_PERIOD_MS)
        return H0AR9_ERR_WRONGPARAMS;

    stopStream = false;

    /* Stream data to buffer */
    while ((Numofsamples-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
        if (function == SamplePIRBuf || function == SampleDistanceBuf || function == SampleTemperatureBuf || function == SampleHumidityBuf) {
            float sample;
            function(&sample);
            buffer[StreamIndex] = sample;
            StreamIndex++;
        } else {
            float Axis[3];
            function(Axis);
            buffer[StreamIndex] = Axis[0];
            buffer[StreamIndex + 1] = Axis[1];
            buffer[StreamIndex + 2] = Axis[2];
            StreamIndex += 3;
        }

        /* Delay for the specified period */
        vTaskDelay(pdMS_TO_TICKS(period));

        /* Check if streaming should be stopped */
        if (stopStream) {
            status = H0AR9_ERR_TERMINATED;
            break;
        }
    }

    return status;
}
/***************************************************************************/
Module_Status WriteRegData(uint8_t reg, uint8_t data) {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;

	TXBuffer[0] = 0x80 | reg;
	TXBuffer[1] = data;

	taskENTER_CRITICAL();
	HAL_status = HAL_I2C_Master_Transmit(I2C_HANDLER, COLOR_PROXIMITY_REG, TXBuffer, 2, HAL_MAX_DELAY);
	taskEXIT_CRITICAL();

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

/***************************************************************************/
Module_Status Read_Word(uint8_t reg, uint16_t *Data) {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;

	TXBuffer[0] = 0xA0 | reg;

	taskENTER_CRITICAL();
	HAL_status = HAL_I2C_Master_Transmit(I2C_HANDLER, COLOR_PROXIMITY_REG, TXBuffer, 1,
			HAL_MAX_DELAY);
	taskEXIT_CRITICAL();

	switch (HAL_status) {
	case HAL_ERROR:
		return status = H0AR9_ERROR;
		break;

	case HAL_BUSY:
		return status = H0AR9_ERR_BUSY;
		break;

	case HAL_OK:
		status = H0AR9_OK;
		break;

	default:
		break;
	}

	taskENTER_CRITICAL();
	HAL_status = HAL_I2C_Master_Receive(I2C_HANDLER, COLOR_PROXIMITY_REG, RXBuffer, 2, HAL_MAX_DELAY);
	taskEXIT_CRITICAL();

	switch (HAL_status) {
	case HAL_ERROR:
		return status = H0AR9_ERROR;
		break;

	case HAL_BUSY:
		return status = H0AR9_ERR_BUSY;
		break;

	case HAL_OK:
		status = H0AR9_OK;
		break;

	default:
		break;
	}

	*Data = (uint16_t) (RXBuffer[0] + (256 * RXBuffer[1]));

	return status;
}
/***************************************************************************/
/*initialize APDS9950 sensor */
Module_Status APDS9950_init(void) {
	Module_Status status = H0AR9_OK;


	if (H0AR9_OK != WriteRegData(Enable_REG, 0x00))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(ATIME_REG, 0x00))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(WTIME_REG, 0xff))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(PPULSE_REG, 0x01))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(CONTROL_REG, 0x20))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != WriteRegData(Enable_REG, 0x0F))
		return status = H0AR9_ERROR;

	return status;
}


/***************************************************************************/
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples)
{
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay =  period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr=1 ; chr<MSG_RX_BUF_SIZE ; chr++)
		{
			if (UARTRxBuf[pcPort-1][chr] == '\r') {
				UARTRxBuf[pcPort-1][chr] = 0;
				StopeCliStreamFlag =1;
				return H0AR9_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H0AR9_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H0AR9_OK;
}

/***************************************************************************/
void SampleColorToString(char *cstring, size_t maxLen) {
	uint16_t red = 0, green = 0, blue = 0;

	uint16_t Red;
	uint16_t Green;
	uint16_t Blue;

	Red = red;
	Green = green;
	Blue = blue;
	SampleColor(&red, &green, &blue);
	;
	snprintf(cstring, maxLen, "Red: %d, Green: %d, Blue: %d\r\n", red, green, blue);
}

/***************************************************************************/
void SampleDistanceToString(char *cstring, size_t maxLen) {

	uint16_t distance = 0;
	uint16_t distance1;

	SampleDistance(&distance);
	distance1 = distance;
	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);

}

/***************************************************************************/
void SampleTemperatureToString(char *cstring, size_t maxLen) {

	float temp;
	float temprature = 0;
	SampleTemperature(&temprature);
	temp = temprature;
	char Number[5] = { 0 };
	uint16_t x;
	volatile uint32_t temp1 = 1;
	uint16_t x0, x1, x2;
	temp1 = temp;

	x0 = (uint8_t) (temp * 10 - temp1 * 10);
	x1 = (uint8_t) (temp1 % 10);
	x2 = (uint8_t) (temp1 / 10 % 10);

	if (x2 == 0) {
		Number[0] = 0x20;
	} else {
		Number[0] = x2 + 0x30;
	}
	Number[1] = x1 + 0x30;
	Number[2] = '.';
	Number[3] = x0 + 0x30;
	Number[4] = 0;
	snprintf(cstring, maxLen, "Temperature:  %.4s\r\n", Number);

}

/***************************************************************************/
void SampleHumidityToString(char *cstring, size_t maxLen) {

	float hum;
	float humidity = 0;
	SampleHumidity(&humidity);
	hum = humidity;
	char Number[5] = { 0 };
	uint16_t x;
	volatile uint32_t temp1 = 1;
	uint16_t x0, x1, x2;
	temp1 = hum;

	x0 = (uint8_t) (hum * 10 - temp1 * 10);
	x1 = (uint8_t) (temp1 % 10);
	x2 = (uint8_t) (temp1 / 10 % 10);

	if (x2 == 0) {
		Number[0] = 0x20;
	} else {
		Number[0] = x2 + 0x30;
	}
	Number[1] = x1 + 0x30;
	Number[2] = '.';
	Number[3] = x0 + 0x30;
	Number[4] = 0;
	snprintf(cstring, maxLen, "Humidity: %.4s\r\n", Number);

}
/*-----------------------------------------------------------*/

void SamplePIRToString(char *cstring, size_t maxLen) {

	bool sample;
	uint8_t Sample;

	SamplePIR(&sample);
	Sample = sample;
	snprintf(cstring, maxLen, "PIR: %d\r\n", sample);

}
/***************************************************************************/
/* Streams sensor data to the CLI (Command Line Interface).
 * Numofsamples: Number of samples to take.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the sampling function (e.g., SampleAccGToString, SampleGyroDPSToString).
 */
static Module_Status StreamToCLI(uint32_t Numofsamples,uint32_t timeout,SampleToString function){
	Module_Status status =H0AR9_OK; /* Initialize status to OK */
	int8_t *pcOutputString = NULL;  /* Pointer to output string */
	uint32_t period =timeout / Numofsamples; /* Calculate the period for each sample */

	/* Check if the calculated period is valid */
	if(period < MIN_PERIOD_MS)
		return H0AR9_ERR_WRONGPARAMS;

	/* Check if CLI is enabled */
	for(uint8_t chr =0; chr < MSG_RX_BUF_SIZE; chr++){
		if(UARTRxBuf[pcPort - 1][chr] == '\r'){
			UARTRxBuf[pcPort - 1][chr] =0; /* Null-terminate the buffer */
		}
	}

	/* Check if streaming should be stopped */
	if(1 == StopeCliStreamFlag){
		StopeCliStreamFlag =0;
		static char *pcOKMessage =(int8_t* )"Stop stream!\n\r";
		writePxITMutex(pcPort,pcOKMessage,strlen(pcOKMessage),10);
		return status;
	}

	/* Adjust timeout period if necessary */
	if(period > timeout)
		timeout =period;

	long numTimes =timeout / period;
	stopStream = false;

	/* Stream data to CLI */
	while((numTimes-- > 0) || (timeout >= MAX_TIMEOUT_MS)){
		pcOutputString =FreeRTOS_CLIGetOutputBuffer(); /* Get output buffer for CLI */
		function((char* )pcOutputString,100); /* Call the sampling function to get data */
		writePxMutex(pcPort,(char* )pcOutputString,strlen((char* )pcOutputString),cmd500ms,HAL_MAX_DELAY);

		if(PollingSleepCLISafe(period,Numofsamples) != H0AR9_OK)
			break;
	}

	memset((char* )pcOutputString,0,configCOMMAND_INT_MAX_OUTPUT_SIZE); /* Clear the output buffer */
	sprintf((char* )pcOutputString,"\r\n"); /* Add newline to output buffer */

	return status; /* Return the status of the operation */
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SampleColor(uint16_t *Red, uint16_t *Green, uint16_t *Blue) {

	Module_Status status = H0AR9_OK;

	if (H0AR9_OK != Read_Word(RED_REG, Red))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != Read_Word(GREEN_REG, Green))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != Read_Word(BLUE_REG, Blue))
		return status = H0AR9_ERROR;

	return status;

}

/***************************************************************************/
Module_Status SampleDistance(uint16_t *distance) {
	Module_Status status = H0AR9_OK;
	uint16_t count = 0;
	uint16_t Raw;
	uint32_t summ = 0.0f;
	uint16_t median_value;

	/*median filter*/
	while (count < 300) {
		count++;
		Read_Word(DISTANCE_REG, &Raw);
		summ += Raw;
		Raw = 0;
		Delay_ms(1.5);
	}

	median_value = 1078 - (summ / 300);

	if (median_value >= 1020 && median_value < 1080)
		*distance = 100;

	if (median_value >= 1010 && median_value < 1020)
		*distance = 90 + (median_value % 10);

	if (median_value >= 1000 && median_value < 1010)
		*distance = 80 + (median_value % 10);

	if (median_value >= 990 && median_value < 1000)
		*distance = 70 + (median_value % 10);

	if (median_value >= 970 && median_value < 990)
		*distance = 60 + ((median_value % 10) / 2);

	if (median_value >= 950 && median_value < 970)
		*distance = 50 + ((median_value % 10) / 2);

	if (median_value >= 850 && median_value < 950)
		*distance = 40 + (((median_value + 50) % 100) / 10);

	if (median_value >= 700 && median_value < 850)
		*distance = 30 + ((median_value - 700) / 15);

	if (median_value >= 400 && median_value < 700)
		*distance = 20;

	if (median_value >= 0 && median_value < 400)
		*distance = 0;

	return status;

}

/***************************************************************************/
Module_Status SamplePIR(bool *pir) {
	Module_Status status = H0AR9_OK;
	*pir = HAL_GPIO_ReadPin(PIR_INPUT_PORT, PIR_INPUT_PIN);/* USER CODE END WHILE */
	Delay_ms(500);
	return status;
}

/***************************************************************************/
Module_Status SampleTemperature(float *temperature) {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;
	uint8_t buf[2];
	uint16_t val;
	buf[0] = TEMP_REG;

	taskENTER_CRITICAL();
	if (HAL_OK != HAL_I2C_Master_Transmit(I2C_HANDLER, TEMP_HUM_REG, buf, 1, HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	HAL_Delay(20);

	taskENTER_CRITICAL();
	if (HAL_OK != HAL_I2C_Master_Receive(I2C_HANDLER, TEMP_HUM_REG, buf, 2, HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	val = buf[0] << 8 | buf[1];
	*temperature = ((float) val / 65536) * 165.0 - 40.0;

	return status;

}

/***************************************************************************/
Module_Status SampleHumidity(float *humidity) {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;
	uint8_t buf[2];
	uint16_t val;
	buf[0] = HUMIDITY_REG;

	taskENTER_CRITICAL();
	if (HAL_OK != HAL_I2C_Master_Transmit(I2C_HANDLER, TEMP_HUM_REG, buf, 1, HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	HAL_Delay(20);

	taskENTER_CRITICAL();
	if (HAL_OK != HAL_I2C_Master_Receive(I2C_HANDLER, TEMP_HUM_REG, buf, 2, HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	val = buf[0] << 8 | buf[1];
	*humidity = (((float) val * 100) / 65536);

	return status;

}
/***************************************************************************/
/*
 * @brief: Samples data and exports it to a specified port for the H0AR9 sensor module.
 * @param dstModule: The module number to export data from.
 * @param dstPort: The port number to export data to.
 * @param dataFunction: Function to sample data (e.g., PIR, DISTANCE, COLOR, TEMPERATURE, HUMIDITY).
 * @retval: Module status indicating success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction) {
    Module_Status Status = H0AR9_OK;
    static uint8_t Temp[12] = {0}; /* Buffer for data transmission */
    bool pir = false;
    uint16_t distance = 0, red = 0, green = 0, blue = 0;
    float temperature = 0.0f, humidity = 0.0f;

    /* Check if the port and module ID are valid */
    if ((dstPort == 0) && (dstModule == myID)) {
        return H0AR9_ERR_WRONGPARAMS;
    }

    /* Sample and export data based on function type */
    switch (dataFunction) {
        case PIR:
            if (SamplePIR(&pir) != H0AR9_OK) {
                return H0AR9_ERROR;
            }

            if (dstModule == myID) {
                Temp[0] = (uint8_t)pir;
                writePxITMutex(dstPort, (char*)&Temp[0], 1 * sizeof(uint8_t), 10);
            } else {
                /* LSB first */
                MessageParams[0] = FMT_BOOL;                                     /* Data format: bool */
                MessageParams[1] = (H0AR9_OK == Status) ? BOS_OK : BOS_ERROR;   /* Operation status */
                MessageParams[2] = 1;                                           /* Number of elements (pir) */
                MessageParams[3] = (uint8_t)(CODE_H0AR9_SAMPLE_PIR >> 0);       /* Command code LSB */
                MessageParams[4] = (uint8_t)(CODE_H0AR9_SAMPLE_PIR >> 8);       /* Command code MSB */
                MessageParams[5] = (uint8_t)pir;                                /* PIR value */
                SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(bool) * 1) + 5);
            }
            break;

        case DISTANCE:
            if (SampleDistance(&distance) != H0AR9_OK) {
                return H0AR9_ERROR;
            }

            if (dstModule == myID) {
                /* LSB first */
                Temp[0] = (uint8_t)(distance);         /* Distance byte 0 */
                Temp[1] = (uint8_t)(distance >> 8);    /* Distance byte 1 */
                writePxITMutex(dstPort, (char*)&Temp[0], 2 * sizeof(uint8_t), 10);
            } else {
                /* LSB first */
                MessageParams[0] = FMT_UINT16;                                   /* Data format: uint16 */
                MessageParams[1] = (H0AR9_OK == Status) ? BOS_OK : BOS_ERROR;   /* Operation status */
                MessageParams[2] = 1;                                           /* Number of elements (distance) */
                MessageParams[3] = (uint8_t)(CODE_H0AR9_SAMPLE_DISTANCE >> 0);  /* Command code LSB */
                MessageParams[4] = (uint8_t)(CODE_H0AR9_SAMPLE_DISTANCE >> 8);  /* Command code MSB */
                MessageParams[5] = (uint8_t)(distance);                         /* Distance byte 0 */
                MessageParams[6] = (uint8_t)(distance >> 8);                    /* Distance byte 1 */
                SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(uint16_t) * 1) + 5);
            }
            break;

        case COLOR:
            if (SampleColor(&red, &green, &blue) != H0AR9_OK) {
                return H0AR9_ERROR;
            }

            if (dstModule == myID) {
                /* LSB first */
                Temp[0] = (uint8_t)(red);         /* Red byte 0 */
                Temp[1] = (uint8_t)(red >> 8);    /* Red byte 1 */
                Temp[2] = (uint8_t)(green);       /* Green byte 0 */
                Temp[3] = (uint8_t)(green >> 8);  /* Green byte 1 */
                Temp[4] = (uint8_t)(blue);        /* Blue byte 0 */
                Temp[5] = (uint8_t)(blue >> 8);   /* Blue byte 1 */
                writePxITMutex(dstPort, (char*)&Temp[0], 6 * sizeof(uint8_t), 10);
            } else {
                /* LSB first */
                MessageParams[0] = FMT_UINT16;                                   /* Data format: uint16 */
                MessageParams[1] = (H0AR9_OK == Status) ? BOS_OK : BOS_ERROR;   /* Operation status */
                MessageParams[2] = 3;                                           /* Number of elements (red, green, blue) */
                MessageParams[3] = (uint8_t)(CODE_H0AR9_SAMPLE_COLOR >> 0);     /* Command code LSB */
                MessageParams[4] = (uint8_t)(CODE_H0AR9_SAMPLE_COLOR >> 8);     /* Command code MSB */
                MessageParams[5] = (uint8_t)(red);                              /* Red byte 0 */
                MessageParams[6] = (uint8_t)(red >> 8);                         /* Red byte 1 */
                MessageParams[7] = (uint8_t)(green);                            /* Green byte 0 */
                MessageParams[8] = (uint8_t)(green >> 8);                       /* Green byte 1 */
                MessageParams[9] = (uint8_t)(blue);                             /* Blue byte 0 */
                MessageParams[10] = (uint8_t)(blue >> 8);                       /* Blue byte 1 */
                SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(uint16_t) * 3) + 5);
            }
            break;

        case TEMPERATURE:
            if (SampleTemperature(&temperature) != H0AR9_OK) {
                return H0AR9_ERROR;
            }

            if (dstModule == myID) {
                /* LSB first */
                Temp[0] = (uint8_t)(*(uint32_t*)&temperature);         /* Temperature byte 0 */
                Temp[1] = (uint8_t)((*(uint32_t*)&temperature) >> 8);  /* Temperature byte 1 */
                Temp[2] = (uint8_t)((*(uint32_t*)&temperature) >> 16); /* Temperature byte 2 */
                Temp[3] = (uint8_t)((*(uint32_t*)&temperature) >> 24); /* Temperature byte 3 */
                writePxITMutex(dstPort, (char*)&Temp[0], 4 * sizeof(uint8_t), 10);
            } else {
                /* LSB first */
                MessageParams[0] = FMT_FLOAT;                                    /* Data format: float */
                MessageParams[1] = (H0AR9_OK == Status) ? BOS_OK : BOS_ERROR;   /* Operation status */
                MessageParams[2] = 1;                                           /* Number of elements (temperature) */
                MessageParams[3] = (uint8_t)(CODE_H0AR9_SAMPLE_TEMP >> 0);      /* Command code LSB */
                MessageParams[4] = (uint8_t)(CODE_H0AR9_SAMPLE_TEMP >> 8);      /* Command code MSB */
                MessageParams[5] = (uint8_t)(*(uint32_t*)&temperature);         /* Temperature byte 0 */
                MessageParams[6] = (uint8_t)((*(uint32_t*)&temperature) >> 8);  /* Temperature byte 1 */
                MessageParams[7] = (uint8_t)((*(uint32_t*)&temperature) >> 16); /* Temperature byte 2 */
                MessageParams[8] = (uint8_t)((*(uint32_t*)&temperature) >> 24); /* Temperature byte 3 */
                SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(float) * 1) + 5);
            }
            break;

        case HUMIDITY:
            if (SampleHumidity(&humidity) != H0AR9_OK) {
                return H0AR9_ERROR;
            }

            if (dstModule == myID) {
                /* LSB first */
                Temp[0] = (uint8_t)(*(uint32_t*)&humidity);         /* Humidity byte 0 */
                Temp[1] = (uint8_t)((*(uint32_t*)&humidity) >> 8);  /* Humidity byte 1 */
                Temp[2] = (uint8_t)((*(uint32_t*)&humidity) >> 16); /* Humidity byte 2 */
                Temp[3] = (uint8_t)((*(uint32_t*)&humidity) >> 24); /* Humidity byte 3 */
                writePxITMutex(dstPort, (char*)&Temp[0], 4 * sizeof(uint8_t), 10);
            } else {
                /* LSB first */
                MessageParams[0] = FMT_FLOAT;                                    /* Data format: float */
                MessageParams[1] = (H0AR9_OK == Status) ? BOS_OK : BOS_ERROR;   /* Operation status */
                MessageParams[2] = 1;                                           /* Number of elements (humidity) */
                MessageParams[3] = (uint8_t)(CODE_H0AR9_SAMPLE_HUMIDITY >> 0);  /* Command code LSB */
                MessageParams[4] = (uint8_t)(CODE_H0AR9_SAMPLE_HUMIDITY >> 8);  /* Command code MSB */
                MessageParams[5] = (uint8_t)(*(uint32_t*)&humidity);            /* Humidity byte 0 */
                MessageParams[6] = (uint8_t)((*(uint32_t*)&humidity) >> 8);     /* Humidity byte 1 */
                MessageParams[7] = (uint8_t)((*(uint32_t*)&humidity) >> 16);    /* Humidity byte 2 */
                MessageParams[8] = (uint8_t)((*(uint32_t*)&humidity) >> 24);    /* Humidity byte 3 */
                SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(float) * 1) + 5);
            }
            break;

        default:
            return H0AR9_ERR_WRONGPARAMS;
    }

    /* Clear the temp buffer */
    memset(&Temp[0], 0, sizeof(Temp));

    return Status;
}
/***************************************************************************/
/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (PIR, DISTANCE, COLOR, TEMPERATURE, HUMIDITY).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples, uint32_t streamTimeout) {
    Module_Status Status = H0AR9_OK;
    uint32_t SamplePeriod = 0u;

    /* Check timer handle and timeout validity */
    if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)) {
        return H0AR9_ERROR; /* Assuming H0AR9_ERROR is defined in Module_Status */
    }

    /* Set streaming parameters */
    StreamMode = STREAM_MODE_TO_PORT;
    PortModule = dstModule;
    PortNumber = dstPort;
    PortFunction = dataFunction;
    PortNumOfSamples = numOfSamples;

    /* Calculate the period from timeout and number of samples */
    SamplePeriod = streamTimeout / numOfSamples;

    /* Stop (Reset) the TimerStream if it's already running */
    if (xTimerIsTimerActive(xTimerStream)) {
        if (pdFAIL == xTimerStop(xTimerStream, 100)) {
            return H0AR9_ERROR;
        }
    }

    /* Start the stream timer */
    if (pdFAIL == xTimerStart(xTimerStream, 100)) {
        return H0AR9_ERROR;
    }

    /* Update timer timeout - This also restarts the timer */
    if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100)) {
        return H0AR9_ERROR;
    }

    return Status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (PIR, DISTANCE, COLOR, TEMPERATURE, HUMIDITY).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort, All_Data dataFunction, uint32_t numOfSamples, uint32_t streamTimeout) {
    Module_Status Status = H0AR9_OK;
    uint32_t SamplePeriod = 0u;

    /* Check timer handle and timeout validity */
    if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)) {
        return H0AR9_ERROR; /* Assuming H0AR9_ERROR is defined in Module_Status */
    }

    /* Set streaming parameters */
    StreamMode = STREAM_MODE_TO_TERMINAL;
    TerminalPort = dstPort;
    TerminalFunction = dataFunction;
    TerminalNumOfSamples = numOfSamples;

    /* Calculate the period from timeout and number of samples */
    SamplePeriod = streamTimeout / numOfSamples;

    /* Stop (Reset) the TimerStream if it's already running */
    if (xTimerIsTimerActive(xTimerStream)) {
        if (pdFAIL == xTimerStop(xTimerStream, 100)) {
            return H0AR9_ERROR;
        }
    }

    /* Start the stream timer */
    if (pdFAIL == xTimerStart(xTimerStream, 100)) {
        return H0AR9_ERROR;
    }

    /* Update timer timeout - This also restarts the timer */
    if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100)) {
        return H0AR9_ERROR;
    }

    return Status;
}
/***************************************************************************/
/*
 * @brief: Streams data to a buffer.
 * @param buffer: Pointer to the buffer where data will be stored.
 * @param function: Function to sample data (e.g., PIR, DISTANCE, COLOR, TEMPERATURE, HUMIDITY).
 * @param Numofsamples: Number of samples to take.
 * @param timeout: Timeout period for the operation.
 * @retval: Module status indicating success or error.
 */
Module_Status StreamToBuffer(float *buffer, All_Data function, uint32_t Numofsamples, uint32_t timeout) {
    switch (function) {
        case PIR:
            return StreamToBuf(buffer, Numofsamples, timeout, SamplePIRBuf);
            break;
        case DISTANCE:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleDistanceBuf);
            break;
        case COLOR:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleColorBuf);
            break;
        case TEMPERATURE:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleTemperatureBuf);
            break;
        case HUMIDITY:
            return StreamToBuf(buffer, Numofsamples, timeout, SampleHumidityBuf);
            break;
        default:
            break;
    }
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	const char *const colorCmdName = "color";
	const char *const distanceCmdName = "distance";
	const char *const temperatureCmdName = "temp";
	const char *const humidityCmdName = "humidity";
	const char *const pirCmdName = "pir";

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	pSensName = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &sensNameLen);

	if (pSensName == NULL) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, colorCmdName, strlen(colorCmdName))) {
			SampleColorToString((char *)pcWriteBuffer, xWriteBufferLen);

		} else if (!strncmp(pSensName, distanceCmdName, strlen(distanceCmdName))) {
			SampleDistanceToString((char *)pcWriteBuffer, xWriteBufferLen);


		} else if (!strncmp(pSensName, temperatureCmdName, strlen(temperatureCmdName))) {
			SampleTemperatureToString((char *)pcWriteBuffer, xWriteBufferLen);


		} else if (!strncmp(pSensName, humidityCmdName, strlen(humidityCmdName))) {
			SampleHumidityToString((char *)pcWriteBuffer, xWriteBufferLen);


		} else if (!strncmp(pSensName, pirCmdName, strlen(pirCmdName))) {
			SamplePIRToString((char *)pcWriteBuffer, xWriteBufferLen);

		}
		else {
			snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}

		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

/***************************************************************************/
/* Port Mode => false and CLI Mode => true */
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
														bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule)
{
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	*ppSensName = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
	pPeriodMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
	pTimeoutMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

	// At least 3 Parameters are required!
	if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
	pModStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

/***************************************************************************/
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	const char *const colorCmdName = "color";
	const char *const distanceCmdName = "distance";
	const char *const temperatureCmdName = "temp";
	const char *const humidityCmdName = "humidity";
	const char *const pirCmdName = "pir";

	uint32_t Numofsamples = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port, &module)) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, colorCmdName, strlen(colorCmdName))) {
			if (portOrCLI) {

				StreamToCLI(Numofsamples, timeout, SampleColorToString);
			} else {
				StreamToPort(module, port,COLOR, Numofsamples, timeout );

			}

		} else if (!strncmp(pSensName, distanceCmdName, strlen(distanceCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SampleDistanceToString);

			} else {
				StreamToPort(module, port,DISTANCE, Numofsamples, timeout);

			}

		}
		else if (!strncmp(pSensName, temperatureCmdName, strlen(temperatureCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SampleTemperatureToString);

			} else {
				StreamToPort(module, port,TEMPERATURE, Numofsamples, timeout);

			}

		} else if (!strncmp(pSensName, humidityCmdName, strlen(humidityCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SampleColorToString);

			} else {
				StreamToPort(module, port,HUMIDITY, Numofsamples, timeout);

			}

		} else if (!strncmp(pSensName, pirCmdName, strlen(pirCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SamplePIRToString);

			} else {
				StreamToPort(module, port,PIR, Numofsamples, timeout);

			}

		}
		else {
			snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}

		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/

