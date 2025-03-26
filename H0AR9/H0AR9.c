/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
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
TaskHandle_t SensorHubTaskHandle = NULL;
static bool stopStream = false;
static const uint8_t colorProximityAdd = (0x39)<<1;
//temprature and humidity sensor addresses
static const uint8_t tempHumAdd = (0x40)<<1; // Use 7-bit address
static const uint8_t tempReg = 0x00;
static const uint8_t humidityReg = 0x01;
typedef void (*SampleToString)(char *, size_t);
//typedef void (*SampleToPort)(uint8_t, uint8_t);
typedef void (*SampleToBuffer)(float *buffer);
uint8_t coun;
uint16_t Dist;
uint8_t flag ;
uint8_t receive[2];
uint8_t send[2];

TimerHandle_t xTimerStream = NULL;

/* Stream to port variables */
volatile uint32_t PortNumOfSamples = 0u;    /* Number of samples for port streaming */
volatile uint32_t PortSamples = 0u;         /* Current sample count for port (if needed separately) */
uint8_t PortModule = 0u;           /* Module ID for port streaming */
uint8_t PortNumber = 0u;           /* Port number for streaming */
All_Data PortFunction;                    /* Function pointer or struct for port streaming */

/* Stream to terminal variables */
volatile uint32_t TerminalNumOfSamples = 0u; /* Number of samples for terminal streaming */
volatile uint8_t TerminalPort = 0u;          /* Port number for terminal streaming */
All_Data TerminalFunction;                   /* Function pointer or struct for terminal streaming */
uint32_t TerminalTimeout = 0u;               /* Timeout value for terminal streaming */
uint8_t StreamMode = 0u;                     /* Streaming mode selector (port or terminal) */
uint8_t StopeCliStreamFlag = 0u;             /* Flag to stop CLI streaming */
/* General streaming variable */
uint32_t SampleCount = 0u;                   /* Total sample counter */


/* Global variables for sensor data */
bool H0AR9_pir = false;
uint16_t H0AR9_distance = 0;
uint16_t H0AR9_red = 0, H0AR9_green = 0, H0AR9_blue = 0;
float H0AR9_temperature = 0.0f;
float H0AR9_humidity = 0.0f;


/* Exported Typedef */
module_param_t modParam[NUM_MODULE_PARAMS] = {
    { .paramPtr = &H0AR9_pir, .paramFormat = FMT_BOOL, .paramName = "pir" },
    { .paramPtr = &H0AR9_distance, .paramFormat = FMT_UINT16, .paramName = "distance" },
    { .paramPtr = &H0AR9_red, .paramFormat = FMT_UINT16, .paramName = "red" },
    { .paramPtr = &H0AR9_green, .paramFormat = FMT_UINT16, .paramName = "green" },
    { .paramPtr = &H0AR9_blue, .paramFormat = FMT_UINT16, .paramName = "blue" },
    { .paramPtr = &H0AR9_temperature, .paramFormat = FMT_FLOAT, .paramName = "temperature" },
    { .paramPtr = &H0AR9_humidity, .paramFormat = FMT_FLOAT, .paramName = "humidity" }
};

uint8_t CONTROL, Enable, ATIME, WTIME, PPULSE;
uint8_t redReg, greenReg, blueReg, distanceReg;
uint16_t Red __attribute__((section(".mySection")));
uint16_t Green __attribute__((section(".mySection")));
uint16_t Blue __attribute__((section(".mySection")));
uint16_t distance1 __attribute__((section(".mySection")));
float temp __attribute__((section(".mySection")));
float hum __attribute__((section(".mySection")));
uint8_t Sample __attribute__((section(".mySection")));
/* Private function prototypes -----------------------------------------------*/
//void SensorHub(void *argument);
void StreamTimeCallback(TimerHandle_t xTimerStream);
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction);
void SamplePIRToString(char *cstring, size_t maxLen);
void SampleDistanceToString(char *cstring, size_t maxLen);
void SampleTemperatureToString(char *cstring, size_t maxLen);
void SampleHumidityToString(char *cstring, size_t maxLen);
void SampleColorToString(char *cstring, size_t maxLen);
Module_Status WriteRegData(uint8_t reg, uint8_t data);
Module_Status APDS9950_init(void);
Module_Status Read_Word(uint8_t reg , uint16_t *Data );
Module_Status ExportStreamToPort (uint8_t module,uint8_t port,All_Data function,uint32_t Numofsamples,uint32_t timeout);
static Module_Status ExportToTerminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port, SampleToString function);
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples);
Module_Status Exportstreamtoterminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port,All_Data function);
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function);


/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "sample:\r\n Syntax: sample [color]/[distance]/[temp]/[humidity]/[pir].\r\n\r\n",
	SampleSensorCommand,
	1
};
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [color]/[distance]/[temp]/[humidity]/[pir] (period in ms) (time in ms) [port] [module].\r\n\r\n",
	StreamSensorCommand,
	-1
};

///*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |                        Private Functions                              |
 -----------------------------------------------------------------------
 */

/**
* @brief  System Clock Configuration
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
* @param  None
* @retval None
*/
void SystemClock_Config(void){
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   /** Configure the main internal regulator output voltage */
   HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

   /** Initializes the RCC Oscillators according to the specified parameters
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
   RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
   RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
   RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
   HAL_RCC_OscConfig(&RCC_OscInitStruct);

   /** Initializes the CPU, AHB and APB buses clocks */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
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

//	DMA_NVIC_Setup();

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

	/* Create a timeout software timer StreamSamplsToPort() API */
		xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(1000),pdTRUE,(void* )1,StreamTimeCallback);

}

/*-----------------------------------------------------------*/

/* --- Save Command Topology in Flash RO --- */

uint8_t SaveTopologyToRO(void)
{
	HAL_StatusTypeDef flashStatus =HAL_OK;
	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
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
			for(uint8_t column =0; column <= MaxNumOfPorts; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(array[row - 1][0]){
					/* Save each element in topology array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,array[row - 1][column]);
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
						flashAdd += 8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/* --- Save Command Snippets in Flash RO --- */

uint8_t SaveSnippetsToRO(void)
{
	HAL_StatusTypeDef FlashStatus =HAL_OK;
    uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

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
	for(uint8_t index = 0; index < numOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(snippets[index].cond.conditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[index],sizeof(snippet_t));
			/* Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
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
			for(uint8_t j = 0; j < ((strlen(snippets[index].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[index].cmd + j*4 ));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array
	memset(array,0,sizeof(array));
	N =1;
	myID =0;

	return SaveTopologyToRO();
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
	case CODE_H0AR9_SAMPLE_COLOR:
		{
			SampleToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Color);
			break;
		}
		case CODE_H0AR9_SAMPLE_DISTANCE:
		{
			SampleToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Distance);
			break;
		}
		case CODE_H0AR9_SAMPLE_TEMP:
		{
			SampleToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Temperature);
			break;
		}
		case CODE_H0AR9_SAMPLE_HUMIDITY:
		{
			SampleToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Humidity);
			break;
		}
		case CODE_H0AR9_SAMPLE_PIR:
		{
			SampleToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],PIR);
			break;
		}

		case CODE_H0AR9_STREAM_COLOR:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] << 24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] << 24);
			StreamToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Color, Numofsamples, timeout);
			break;
		}

		case CODE_H0AR9_STREAM_DISTANCE:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] <<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift]<<24);
			StreamToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Distance, Numofsamples, timeout);
			break;
		}
		case CODE_H0AR9_STREAM_TEMP:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift]<<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift]<<24);
			StreamToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Temperature, Numofsamples, timeout);
			break;
		}
		case CODE_H0AR9_STREAM_HUMIDITY:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift]<<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift]<<24);
			StreamToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],Humidity, Numofsamples, timeout);
			break;
		}
		case CODE_H0AR9_STREAM_PIR:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] <<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] <<24);
			StreamToPort(cMessage[port-1][shift] ,cMessage[port-1][1+shift],PIR, Numofsamples, timeout);
			break;
		}


		default:
			result = H0AR9_ERR_UnknownMessage;
			break;
  }

  return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand( &SampleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StreamCommandDefinition );


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

/***************************************************************************/
/*
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
    Module_Status status = H0AR9_OK;

    switch (paramIndex) {
        /* Sample PIR sensor (convert bool to float) */
        case 1:
        {
            bool temp = false;
            status = SamplePIR(&temp);
            if (status == H0AR9_OK) *value = (float)temp;
            break;
        }

        /* Sample Distance sensor */
        case 2:
        {
            uint16_t temp = 0;
            status = SampleDistance(&temp);
            if (status == H0AR9_OK) *value = (float)temp;
            break;
        }

        /* Sample Color - Red */
        case 3:
        {
            uint16_t temp = 0;
            status = SampleColor(&temp, NULL, NULL);
            if (status == H0AR9_OK) *value = (float)temp;
            break;
        }

        /* Sample Color - Green */
        case 4:
        {
            uint16_t temp = 0;
            status = SampleColor(NULL, &temp, NULL);
            if (status == H0AR9_OK) *value = (float)temp;
            break;
        }

        /* Sample Color - Blue */
        case 5:
        {
            uint16_t temp = 0;
            status = SampleColor(NULL, NULL, &temp);
            if (status == H0AR9_OK) *value = (float)temp;
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
            status = H0AR9_ERR_WrongParams;
            break;
    }

    return status;
}

/*-----------------------------------------------------------*/

/* --- ToF streaming task 
 */
//
//void SensorHub(void *argument) {
//
//	/* Infinite loop */
//	for (;;) {
//		/*  */
//
//		switch (tofMode) {
////		case STREAM_TO_PORT:
////			ExportStreamToPort(module1, port1, mode1, Numofsamples1, timeout1);
////			break;
////		case SAMPLE_TO_PORT:
////
////			SampleToPort(module2, port2, mode2);
////			break;
////		case STREAM_TO_Terminal:
////			Exportstreamtoterminal(Numofsamples3, timeout3, port3, mode3);
////
////			break;
//
//			break;
//		default:
//			osDelay(10);
//			break;
//		}
//
//		taskYIELD();
//	}
//
//}

/***************************************************************************/
/*
 * brief: Callback function triggered by a timer to manage data streaming.
 * param xTimerStream: Handle of the timer that triggered the callback.
 * retval: None
 */
void StreamTimeCallback(TimerHandle_t xTimerStream){
	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if(STREAM_MODE_TO_PORT == StreamMode){
		if((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)){
			SampleToPort(PortModule,PortNumber,PortFunction);

		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if(STREAM_MODE_TO_TERMINAL == StreamMode){
		if((SampleCount <= TerminalNumOfSamples) || (0 == TerminalNumOfSamples)){
			SampleToTerminal(TerminalPort,TerminalFunction);
		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
}
/* -----------------------------------------------------------------------
 |                               APIs                                    |
 -----------------------------------------------------------------------
 */

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
Module_Status Read_Word(uint8_t reg , uint16_t *Data )
 {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;
	send[0] = 0xA0 | reg;

	taskENTER_CRITICAL();
	HAL_status =HAL_I2C_Master_Transmit(&hi2c2, colorProximityAdd, send, 1, HAL_MAX_DELAY);
	taskEXIT_CRITICAL();

	switch (HAL_status) {
	case HAL_ERROR:
		return	status = H0AR9_ERROR;
		break;
	case HAL_BUSY:
		return	status = H0AR9_ERR_BUSY;
		break;
	case HAL_OK:
		status = H0AR9_OK;
		break;
	default:
		break;
	}
	taskENTER_CRITICAL();
	HAL_status =HAL_I2C_Master_Receive(&hi2c2, colorProximityAdd, receive, 2,HAL_MAX_DELAY);
	taskEXIT_CRITICAL();

	switch (HAL_status) {
	case HAL_ERROR:
		return	status = H0AR9_ERROR;
		break;
	case HAL_BUSY:
		return	status = H0AR9_ERR_BUSY;
		break;
	case HAL_OK:
		status = H0AR9_OK;
		break;
	default:
		break;
	}

	*Data = (uint16_t) (receive[0] + (256 * receive[1]));

	return status;
}
/*-----------------------------------------------------------*/
Module_Status SampleColor(uint16_t *Red, uint16_t *Green, uint16_t *Blue) {

	Module_Status status = H0AR9_OK;

	if (H0AR9_OK != Read_Word(redReg, Red))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != Read_Word(greenReg, Green))
		return status = H0AR9_ERROR;

	if (H0AR9_OK != Read_Word(blueReg, Blue))
		return status = H0AR9_ERROR;

	return status;

}

/*-----------------------------------------------------------*/

Module_Status SampleDistance(uint16_t *distance)
 {
	Module_Status status = H0AR9_OK;
	uint16_t count = 0;
	uint16_t Raw;
	uint32_t summ = 0.0f;
	uint16_t median_value;
	/*median filter*/

	while (count < 300) {
		count++;
		Read_Word(distanceReg, &Raw);
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
/*-----------------------------------------------------------*/
Module_Status SamplePIR(bool *pir)
 {
	Module_Status status = H0AR9_OK;
	*pir = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);/* USER CODE END WHILE */
	Delay_ms(500);
	return status;
}
/*-----------------------------------------------------------*/


Module_Status SampleTemperature(float *temperature)
 {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;
	uint8_t buf[2];
	uint16_t val;
	buf[0] = tempReg;

	taskENTER_CRITICAL();
	if (HAL_OK!= HAL_I2C_Master_Transmit(&hi2c2, tempHumAdd, buf, 1,HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	HAL_Delay(20);

	taskENTER_CRITICAL();
	if (HAL_OK!= HAL_I2C_Master_Receive(&hi2c2, tempHumAdd, buf, 2,HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	val = buf[0] << 8 | buf[1];
	*temperature = ((float) val / 65536) * 165.0 - 40.0;

	return status;

}
/*-----------------------------------------------------------*/
Module_Status SampleHumidity(float *humidity)
 {
	Module_Status status = H0AR9_OK;
	HAL_StatusTypeDef HAL_status;
	uint8_t buf[2];
	uint16_t val;
	buf[0] = humidityReg;

	taskENTER_CRITICAL();
	if (HAL_OK!= HAL_I2C_Master_Transmit(&hi2c2, tempHumAdd, buf, 1,HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	HAL_Delay(20);

	taskENTER_CRITICAL();
	if (HAL_OK!= HAL_I2C_Master_Receive(&hi2c2, tempHumAdd, buf, 2,HAL_MAX_DELAY))
		return status = H0AR9_ERROR;
	taskEXIT_CRITICAL();

	val = buf[0] << 8 | buf[1];
	*humidity = (((float) val * 100) / 65536);

	return status;

}

/***************************************************************************/
/*
 * @brief  Streams a single sensor data sample to the terminal.
 * @param  dstPort: Port number to stream data to.
 * @param  dataFunction: Function to sample data (e.g., Color, PIR, Humidity, Temperature, Distance).
 * @param  numOfSamples: Number of samples (kept for compatibility, not used for repetition).
 * @param  streamTimeout: Timeout period for the operation (in milliseconds).
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction) {
	Module_Status status = H0AR9_OK; /* Initialize operation status as success */
	int8_t *pcOutputString = NULL; /* Pointer to CLI output buffer */
	uint32_t period = 0u; /* Calculated period for the operation */
	char cstring[100] = { 0 }; /* Buffer for formatted output string */


	/* Process data based on the requested sensor function */
	switch (dataFunction) {
	case Color: {
		uint16_t red = 0, green = 0, blue = 0;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample color sensor data */
		if (SampleColor(&red, &green, &blue) != H0AR9_OK) {
			return H0AR9_ERROR; /* Return error if sampling fails */
		}

		/* Format color sensor data into a string */
		snprintf(cstring, sizeof(cstring), "Red: %d, Green: %d, Blue: %d\r\n",
				red, green, blue);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case PIR: {
		bool sample = false;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample PIR sensor data */
		if (SamplePIR(&sample) != H0AR9_OK) {
			return H0AR9_ERROR; /* Return error if sampling fails */
		}

		/* Format PIR sensor data into a string */
		snprintf(cstring, sizeof(cstring), "PIR: %d\r\n", sample);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case Humidity: {
		float humidity = 0.0f;
		char Number[5] = { 0 };
		uint16_t x0, x1, x2;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample humidity sensor data */
		if (SampleHumidity(&humidity) != H0AR9_OK) {
			return H0AR9_ERROR; /* Return error if sampling fails */
		}

		/* Convert float value to formatted string */
		uint16_t temp1 = humidity;
		x0 = (uint8_t) ((humidity * 10) - (temp1 * 10));
		x1 = (uint8_t) (temp1 % 10);
		x2 = (uint8_t) ((temp1 / 10) % 10);

		Number[0] = (x2 == 0) ? ' ' : (x2 + '0');
		Number[1] = x1 + '0';
		Number[2] = '.';
		Number[3] = x0 + '0';
		Number[4] = '\0';

		/* Format humidity data into a string */
		snprintf(cstring, sizeof(cstring), "Humidity: %.4s\r\n", Number);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case Temperature: {
		float temperature = 0.0f;
		char Number[5] = { 0 };
		uint16_t x0, x1, x2;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample temperature sensor data */
		if (SampleTemperature(&temperature) != H0AR9_OK) {
			return H0AR9_ERROR; /* Return error if sampling fails */
		}

		/* Convert float value to formatted string */
		uint16_t temp1 = temperature;
		x0 = (uint8_t) ((temperature * 10) - (temp1 * 10));
		x1 = (uint8_t) (temp1 % 10);
		x2 = (uint8_t) ((temp1 / 10) % 10);

		Number[0] = (x2 == 0) ? ' ' : (x2 + '0');
		Number[1] = x1 + '0';
		Number[2] = '.';
		Number[3] = x0 + '0';
		Number[4] = '\0';

		/* Format temperature data into a string */
		snprintf(cstring, sizeof(cstring), "Temperature: %.4s\r\n", Number);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	case Distance: {
		uint16_t distance = 0;
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		/* Sample distance sensor data */
		if (SampleDistance(&distance) != H0AR9_OK) {
			return H0AR9_ERROR; /* Return error if sampling fails */
		}

		/* Format distance data into a string */
		snprintf(cstring, sizeof(cstring), "Distance: %d\r\n", distance);

		/* Send the formatted string to the specified port */
		writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring),
				cmd500ms, HAL_MAX_DELAY);
		break;
	}

	default:
		return H0AR9_ERR_WrongParams; /* Return error for invalid sensor function */
	}

	/* Return final status indicating success or prior error */
	return status;
}

/***************************************************************************/
/*
 * @brief  Samples data from a sensor and exports it to a specified port or module.
 * @param  dstModule: The module number to export data to.
 * @param  dstPort: The port number to export data to.
 * @param  dataFunction: Function to sample data (e.g., Color, PIR, Humidity, Temperature, Distance).
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction)
{
    static uint8_t temp[6] = {0};       /* Buffer for data transmission */
    Module_Status status = H0AR9_OK;    /* Initialize operation status as success */

    /* Check if the port and module ID are valid */
    if (dstPort == 0 && dstModule == myID)
    {
        return H0AR9_ERR_WrongParams;   /* Return error for invalid parameters */
    }

    /* Process data based on the requested sensor function */
    switch (dataFunction)
    {
        case Color:
        {
            uint16_t red = 0, green = 0, blue = 0;
            status = SampleColor(&red, &green, &blue);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                /* Pack data into temp buffer */
                temp[0] = (uint8_t)(red);
                temp[1] = (uint8_t)(red >> 8);
                temp[2] = (uint8_t)(green);
                temp[3] = (uint8_t)(green >> 8);
                temp[4] = (uint8_t)(blue);
                temp[5] = (uint8_t)(blue >> 8);

                writePxITMutex(dstPort, (char*)temp, 6 * sizeof(uint8_t), 10);
            }
            else
            {
                /* Send data to another module */
                messageParams[1] = (status == H0AR9_OK) ? BOS_OK : BOS_ERROR;
                messageParams[0] = FMT_UINT16;
                messageParams[2] = 3;
                messageParams[3] = temp[0];
                messageParams[4] = temp[1];
                messageParams[5] = temp[2];
                messageParams[6] = temp[3];
                messageParams[7] = temp[4];
                messageParams[8] = temp[5];

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, (sizeof(uint16_t) * 3) + 3);
            }
            break;
        }

        case PIR:
        {
            bool pirStatus = false;
            status = SamplePIR(&pirStatus);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                temp[0] = pirStatus;
                writePxITMutex(dstPort, (char*)temp, sizeof(bool), 10);
            }
            else
            {
                /* Send data to another module */
                messageParams[1] = (status == H0AR9_OK) ? BOS_OK : BOS_ERROR;
                messageParams[0] = FMT_BOOL;
                messageParams[2] = 1;
                messageParams[3] = pirStatus;

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(bool) + 3);
            }
            break;
        }

        case Distance:
        {
            uint16_t distance = 0;
            status = SampleDistance(&distance);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                temp[0] = (uint8_t)(distance);
                temp[1] = (uint8_t)(distance >> 8);

                writePxITMutex(dstPort, (char*)temp, sizeof(uint16_t), 10);
            }
            else
            {
                /* Send data to another module */
                messageParams[1] = (status == H0AR9_OK) ? BOS_OK : BOS_ERROR;
                messageParams[0] = FMT_UINT16;
                messageParams[2] = 1;
                messageParams[3] = temp[0];
                messageParams[4] = temp[1];

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint16_t) + 3);
            }
            break;
        }

        case Temperature:
        {
            float temperature = 0.0f;
            status = SampleTemperature(&temperature);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                memcpy(temp, &temperature, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                messageParams[1] = (status == H0AR9_OK) ? BOS_OK : BOS_ERROR;
                messageParams[0] = FMT_FLOAT;
                messageParams[2] = 1;
                memcpy(&messageParams[3], &temperature, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        case Humidity:
        {
            float humidity = 0.0f;
            status = SampleHumidity(&humidity);

            /* If data is to be sent locally */
            if (dstModule == myID || dstModule == 0)
            {
                memcpy(temp, &humidity, sizeof(float));
                writePxITMutex(dstPort, (char*)temp, sizeof(float), 10);
            }
            else
            {
                /* Send data to another module */
                messageParams[1] = (status == H0AR9_OK) ? BOS_OK : BOS_ERROR;
                messageParams[0] = FMT_FLOAT;
                messageParams[2] = 1;
                memcpy(&messageParams[3], &humidity, sizeof(float));

                SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(float) + 3);
            }
            break;
        }

        default:
            return H0AR9_ERR_WrongParams;  /* Return error for invalid sensor function */
    }

    /* Clear the temp buffer */
    memset(temp, 0, sizeof(temp));

    /* Return final status indicating success or prior error */
    return status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H0AR9_OK;
	uint32_t SamplePeriod =0u;

	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
		return H0AR9_ERROR; /* Assuming H0AR9_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule =dstModule;
	PortNumber =dstPort;
	PortFunction =dataFunction;
	PortNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H0AR9_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H0AR9_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H0AR9_ERROR;
	}

	return Status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H0AR9_OK;
	uint32_t SamplePeriod =0u;
	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout)){
		return H0AR9_ERROR; /* Assuming H0AR9_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_TERMINAL;
	TerminalPort =dstPort;
	TerminalFunction =dataFunction;
	TerminalTimeout =streamTimeout;
	TerminalNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H0AR9_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H0AR9_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H0AR9_ERROR;
	}

	return Status;
}

/*-----------------------------------------------------------*/
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
			if (UARTRxBuf[PcPort-1][chr] == '\r') {
				UARTRxBuf[PcPort-1][chr] = 0;
				flag=1;
				return H0AR9_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H0AR9_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H0AR9_OK;
}



/*-----------------------------------------------------------*/
void SampleColorToString(char *cstring, size_t maxLen)
 {
	uint16_t red = 0, green = 0, blue = 0;
	Red=red;
	Green=green;
	Blue=blue;
	SampleColor(&red, &green, &blue);;
	snprintf(cstring, maxLen, "Red: %d, Green: %d, Blue: %d\r\n", red, green,blue);
 }
/*-----------------------------------------------------------*/
void SampleDistanceToString(char *cstring, size_t maxLen)
{

	uint16_t distance = 0;
	SampleDistance(&distance);
	distance1=distance;
	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);

}

/*-----------------------------------------------------------*/
void SampleTemperatureToString(char *cstring, size_t maxLen)
 {


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
/*-----------------------------------------------------------*/

void SampleHumidityToString(char *cstring, size_t maxLen)
{

	float humidity =0;
	SampleHumidity(&humidity);
	hum =humidity;
	char Number[5] ={0};
	uint16_t x;
	volatile uint32_t temp1 =1;
	uint16_t x0, x1, x2;
	temp1 =hum;

	x0 =(uint8_t )(hum * 10 - temp1 * 10);
	x1 =(uint8_t )(temp1 % 10);
	x2 =(uint8_t )(temp1 / 10 % 10);

	if(x2 == 0){
		Number[0] =0x20;
	}
	else{
		Number[0] =x2 + 0x30;
	}
	Number[1] =x1 + 0x30;
	Number[2] ='.';
	Number[3] =x0 + 0x30;
	Number[4] =0;
	snprintf(cstring,maxLen,"Humidity: %.4s\r\n",Number);

}
/*-----------------------------------------------------------*/

void SamplePIRToString(char *cstring, size_t maxLen)
{

	bool sample;
    SamplePIR(&sample);
	Sample=sample;
	snprintf(cstring, maxLen, "PIR: %d\r\n", sample);

}
/*-----------------------------------------------------------*/
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function)
{
	Module_Status status =H0AR9_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period =timeout / Numofsamples;
	if(period < MIN_MEMS_PERIOD_MS)
		return H0AR9_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	for(uint8_t chr =0; chr < MSG_RX_BUF_SIZE; chr++){
		if(UARTRxBuf[PcPort - 1][chr] == '\r'){
			UARTRxBuf[PcPort - 1][chr] =0;
		}
	}
	if(1 == flag){
		flag =0;
		static char *pcOKMessage =(int8_t* )"Stop stream !\n\r";
		writePxITMutex(PcPort,pcOKMessage,strlen(pcOKMessage),10);
		return status;
	}
	if(period > timeout)
		timeout =period;

	long numTimes =timeout / period;
	stopStream = false;

	while((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)){
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();
		function((char* )pcOutputString,100);

		writePxMutex(PcPort,(char* )pcOutputString,strlen((char* )pcOutputString),cmd500ms,HAL_MAX_DELAY);
		if(PollingSleepCLISafe(period,Numofsamples) != H0AR9_OK)
			break;
	}

	memset((char* )pcOutputString,0,configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char* )pcOutputString,"\r\n");
	return status;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |                             Commands                                  |
 -----------------------------------------------------------------------
 */
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
/*-----------------------------------------------------------*/
// Port Mode => false and CLI Mode => true
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
/*-----------------------------------------------------------*/
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
				StreamToPort(module, port,Color, Numofsamples, timeout );

			}

		} else if (!strncmp(pSensName, distanceCmdName, strlen(distanceCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SampleDistanceToString);

			} else {
				StreamToPort(module, port,Distance, Numofsamples, timeout);

			}

		}
		else if (!strncmp(pSensName, temperatureCmdName, strlen(temperatureCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SampleTemperatureToString);

			} else {
				StreamToPort(module, port,Temperature, Numofsamples, timeout);

			}

		} else if (!strncmp(pSensName, humidityCmdName, strlen(humidityCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples, timeout, SampleColorToString);

			} else {
				StreamToPort(module, port,Humidity, Numofsamples, timeout);

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
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/

