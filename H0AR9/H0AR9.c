/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H0AR9.c
 Description   : Source code for module H0AR9.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "H0AR9_inputs.h"
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

/* Module exported parameters ------------------------------------------------*/
uint8_t H0AR9_PIR;
uint16_t H0AR9_RGBred;
uint16_t H0AR9_RGBgreen;
uint16_t H0AR9_RGBblue;
uint16_t H0AR9_proximity;
float H0AR9_temperature;
float H0AR9_humidity;
module_param_t modParam[NUM_MODULE_PARAMS] = {{.paramPtr=&H0AR9_RGBred, .paramFormat=FMT_UINT16, .paramName="RGBred"},
{.paramPtr=&H0AR9_RGBgreen, .paramFormat=FMT_UINT16, .paramName="RGBgreen"},
{.paramPtr=&H0AR9_RGBblue,  .paramFormat=FMT_UINT16, .paramName="RGBblue"},
{.paramPtr=&H0AR9_proximity, .paramFormat=FMT_UINT16, .paramName="proximity"},
{.paramPtr=&H0AR9_temperature, .paramFormat=FMT_FLOAT, .paramName="temperature"},
{.paramPtr=&H0AR9_humidity, .paramFormat=FMT_FLOAT, .paramName="humidity"},
{.paramPtr=&H0AR9_PIR, .paramFormat=FMT_UINT8, .paramName="PIR"},
};




#define MIN_MEMS_PERIOD_MS				100
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF
float H0AR9_temperature;
/* Private variables ---------------------------------------------------------*/
typedef void (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char *, size_t);
typedef void (*SampleMemsToBuffer)(float *buffer);
//temprature and humidity sensor addresses
static const uint8_t tempHumAdd = (0x40)<<1; // Use 7-bit address
static const uint8_t tempReg = 0x00;
static const uint8_t humidityReg = 0x01;

/*Define private variables*/
TaskHandle_t SensorHubTaskHandle = NULL;
static bool stopStream = false;
extern I2C_HandleTypeDef hi2c2;
uint8_t tofMode ;
uint8_t buf[10];
uint8_t CONTROL, Enable, ATIME, WTIME, PPULSE;
uint8_t redReg, greenReg, blueReg, distanceReg;
uint16_t RED_data, GREEN_data, BLUE_data, Prox_data;
uint16_t val;
uint8_t pir;
uint8_t CONTROL, Enable, ATIME, WTIME, PPULSE;
uint16_t Red __attribute__((section(".mySection")));
uint16_t Green __attribute__((section(".mySection")));
uint16_t Blue __attribute__((section(".mySection")));
uint16_t distance1 __attribute__((section(".mySection")));
float temp __attribute__((section(".mySection")));
float hum __attribute__((section(".mySection")));
uint8_t Sample __attribute__((section(".mySection")));
uint8_t port1, module1;
uint8_t port2 ,module2,mode2,mode1;
uint32_t Numofsamples1 ,timeout1;
uint8_t port3 ,module3,mode3;
uint32_t Numofsamples2 ,timeout2;
uint32_t Numofsamples3 ,timeout3;
uint8_t flag ;
uint8_t cont ;
uint8_t tofMode ;
/* Private function prototypes -----------------------------------------------*/
static Module_Status StreamMemsToBuf( float *buffer, uint32_t period, uint32_t timeout, SampleMemsToBuffer function);
static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout, SampleMemsToPort function);
static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function);
static Module_Status PollingSleepCLISafe(uint32_t period);
void FLASH_Page_Eras(uint32_t Addr );
void ExecuteMonitor(void);
void SensorHub(void *argument);
/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

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
/* CLI command structure : stop */
const CLI_Command_Definition_t StopCommandDefinition = {
	(const int8_t *) "stop",
	(const int8_t *) "stop:\r\n Syntax: stop\r\n \
\tStop the current streaming of MEMS values. r\n\r\n",
	StopStreamCommand,
	0
};

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
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
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
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
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =8;
    uint16_t temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1,RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
          	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS + add,array[i - 1][j]);
				 //HALFWORD 	//TOBECHECKED

					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						add +=8;
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				//HALFWORD
				//TOBECHECKED
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
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[s].cmd + j*4 ));
				//HALFWORD
				//TOBECHECKED
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
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
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

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H0AR9 module initialization.
 */
void Module_Peripheral_Init(void){

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

	/* Create module special task (if needed) */

	  //Circulating DMA Channels ON All Module
	  		 for(int i=1;i<=NumOfPorts;i++)
	  			{
	  			  if(GetUart(i)==&huart1)
	  			           { index_dma[i-1]=&(DMA1_Channel1->CNDTR); }
	  			  else if(GetUart(i)==&huart2)
	  					   { index_dma[i-1]=&(DMA1_Channel2->CNDTR); }
	  			  else if(GetUart(i)==&huart3)
	  					   { index_dma[i-1]=&(DMA1_Channel3->CNDTR); }
	  			  else if(GetUart(i)==&huart4)
	  					   { index_dma[i-1]=&(DMA1_Channel4->CNDTR); }
	  			  else if(GetUart(i)==&huart5)
	  					   { index_dma[i-1]=&(DMA1_Channel5->CNDTR); }
	  			  else if(GetUart(i)==&huart6)
	  					   { index_dma[i-1]=&(DMA1_Channel6->CNDTR); }
	  			}
	  		xTaskCreate(SensorHub,(const char* ) "SensorHub",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&SensorHubTaskHandle);

}
/*-----------------------------------------------------------*/
int v;
void SensorHub(void *argument){

	/* Infinite loop */
	for(;;){
		/*  */
v++;

		switch(tofMode){
			case STREAM_TO_PORT:

				switch(mode2){
					case Distance:
						StreamMemsToPort(port2,module2,Numofsamples2,timeout2,SampleDistanceToPort);
						break;
					case Temperature:
						StreamMemsToPort(port2,module2,Numofsamples2,timeout2,SampleTemperatureToPort);
						break;
					case Humidity:
						StreamMemsToPort(port2,module2,Numofsamples2,timeout2,SampleHumidityToPort);
						break;
					case PIR:
						StreamMemsToPort(port2,module2,Numofsamples2,timeout2,SamplePIRToPort);
						break;
					case Color:
						StreamMemsToPort(port2,module2,Numofsamples2,timeout2,SampleColorToPort);
						break;
					default:
						break;
				}

			break;
			default:
				osDelay(10);
				break;
		}

		taskYIELD();
	}

}
/*-----------------------------------------------------------*/
/* --- H0AR9 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H0AR9_OK;
	uint32_t Numofsamples = 0, timeout = 0;

	switch (code)
	{
		case CODE_H0AR9_SAMPLE_COLOR:
		{
			SampleColorToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H0AR9_SAMPLE_DISTANCE:
		{
			SampleDistanceToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H0AR9_SAMPLE_TEMP:
		{
			SampleTemperatureToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H0AR9_SAMPLE_HUMIDITY:
		{
			SampleHumidityToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H0AR9_SAMPLE_PIR:
		{
			SamplePIRToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}

		case CODE_H0AR9_STREAM_COLOR:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] << 24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] << 24);
			StreamToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], Numofsamples, timeout,Color);
			break;
		}

		case CODE_H0AR9_STREAM_DISTANCE:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] <<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift]<<24);
			StreamToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], Numofsamples, timeout,Distance);
			break;
		}
		case CODE_H0AR9_STREAM_TEMP:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift]<<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift]<<24);
			StreamToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], Numofsamples, timeout,Temperature);
			break;
		}
		case CODE_H0AR9_STREAM_HUMIDITY:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift]<<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift]<<24);
			StreamToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], Numofsamples, timeout,Humidity);
			break;
		}
		case CODE_H0AR9_STREAM_PIR:
		{
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] <<24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] <<24);
			StreamToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], Numofsamples, timeout,PIR);
			break;
		}
		case CODE_H0AR9_STREAM_STOP:
		{
			stopStreamMems();
			result = H0AR9_OK;
			break;
		}

		default:
			result = H0AR9_ERR_UnknownMessage;
			break;
	}

	return result;
}

/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART5)
		return P5;
	else if(huart->Instance == USART6)
		return P6;
	
	return 0;
}
/*------------------------------------------------------------*/
//initialize APDS9950 sensor
void APDS9950_init(void)
{
//registers addresses
	CONTROL = 0x0F;
	Enable = 0x00;
	ATIME  = 0x01;
	WTIME  = 0x03;
	PPULSE = 0x0E;
	redReg = 0x16;
	greenReg = 0x18;
    blueReg = 0x1A;
    distanceReg = 0x1C;

    WriteRegData (Enable,0x00);
    WriteRegData (ATIME,0x00);
    WriteRegData (WTIME,0xff);
    WriteRegData (PPULSE,0x01);
    WriteRegData (CONTROL, 0x20);
    WriteRegData (Enable, 0x0F);
}

/*-----------------------------------------------------------*/
void initialValue(void)
{
	Red=0;
	Green=0;
	Blue=0;
	distance1=0;
	temp=0;
	hum=0;
	Sample=0;
}
/*-----------------------------------------------------------*/


/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand( &SampleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StreamCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StopCommandDefinition);

}

/*-----------------------------------------------------------*/


/* Module special task function (if needed) */
//void Module_Special_Task(void *argument){
//
//	/* Infinite loop */
//	uint8_t cases; // Test variable.
//	for(;;){
//		/*  */
//		switch(cases){
//
//
//			default:
//				osDelay(10);
//				break;
//		}
//
//		taskYIELD();
//	}
//
//}


/*-----------------------------------------------------------*/




/*-----------------------------------------------------------*/
static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t Numofsamples, uint32_t timeout, SampleMemsToPort function)
{
	Module_Status status = H0AR9_OK;
	uint32_t period = timeout / Numofsamples;

	if (period < MIN_MEMS_PERIOD_MS)
		return H0AR9_ERR_WrongParams;
	if (port == 0)
		return H0AR9_ERR_WrongParams;
	if (port == PcPort) // Check if CLI is not enabled at that port!
		return H0AR9_ERR_BUSY;

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		function(port, module);

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H0AR9_ERR_TERMINATED;
			break;
		}
	}
	tofMode=20;
	return status;
}
/*-----------------------------------------------------------*/

static Module_Status StreamMemsToBuf( float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleMemsToBuffer function)

{
	Module_Status status =H0AR9_OK;
	uint32_t period =timeout / Numofsamples;
	if(period < MIN_MEMS_PERIOD_MS)
		return H0AR9_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if(period > timeout)
		timeout =period;

	long numTimes =timeout / period;
	stopStream = false;

	while((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)){
		if(function == SampleColorBuf){
			float color[3];
			function(color);
			buffer[cont] =color[0];
			buffer[cont+1] =color[1];
			buffer[cont+2] =color[2];
			cont+=3;
		} else {
			float sample;
		function(&sample);
		buffer[cont] =sample;
		cont++;
		}

		vTaskDelay(pdMS_TO_TICKS(period));
		if(stopStream){
			status =H0AR9_ERR_TERMINATED;
			break;
		}
	}
	return status;
}
/*-----------------------------------------------------------*/

static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function)
{
	Module_Status status = H0AR9_OK;
	int8_t *pcOutputString = NULL;

	if (period < MIN_MEMS_PERIOD_MS)
		return H0AR9_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period) != H0AR9_OK)
			break;
	}

	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcOutputString, "\r\n");
	return status;
}
/*-----------------------------------------------------------*/

/* --- Save array topology and Command Snippets in Flash RO ---
*/
static Module_Status PollingSleepCLISafe(uint32_t period)
{
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay =  period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr=0 ; chr<MSG_RX_BUF_SIZE ; chr++)
		{
			if (UARTRxBuf[PcPort-1][chr] == '\r') {
				UARTRxBuf[PcPort-1][chr] = 0;
				return H0AR9_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H0AR9_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H0AR9_OK;
}
/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */
void SampleColorToPort(uint8_t port,uint8_t module)
{
	float buffer[3]; // Three Samples RED, GREEN, BLUE
	static uint8_t temp[12];

	SampleColorBuf(buffer);

	if(module == myID || module == 0){
		temp[0] =*((__IO uint8_t* )(&buffer[0]) + 3);
		temp[1] =*((__IO uint8_t* )(&buffer[0]) + 2);
		temp[2] =*((__IO uint8_t* )(&buffer[0]) + 1);
		temp[3] =*((__IO uint8_t* )(&buffer[0]) + 0);

		temp[4] =*((__IO uint8_t* )(&buffer[1]) + 3);
		temp[5] =*((__IO uint8_t* )(&buffer[1]) + 2);
		temp[6] =*((__IO uint8_t* )(&buffer[1]) + 1);
		temp[7] =*((__IO uint8_t* )(&buffer[1]) + 0);

		temp[8] =*((__IO uint8_t* )(&buffer[2]) + 3);
		temp[9] =*((__IO uint8_t* )(&buffer[2]) + 2);
		temp[10] =*((__IO uint8_t* )(&buffer[2]) + 1);
		temp[11] =*((__IO uint8_t* )(&buffer[2]) + 0);

		writePxITMutex(port,(char* )&temp[0],12 * sizeof(uint8_t),10);
	}
	else{
		messageParams[0] =port;
		messageParams[1] =*((__IO uint8_t* )(&buffer[0]) + 3);
		messageParams[2] =*((__IO uint8_t* )(&buffer[0]) + 2);
		messageParams[3] =*((__IO uint8_t* )(&buffer[0]) + 1);
		messageParams[4] =*((__IO uint8_t* )(&buffer[0]) + 0);

		messageParams[5] =*((__IO uint8_t* )(&buffer[1]) + 3);
		messageParams[6] =*((__IO uint8_t* )(&buffer[1]) + 2);
		messageParams[7] =*((__IO uint8_t* )(&buffer[1]) + 1);
		messageParams[8] =*((__IO uint8_t* )(&buffer[1]) + 0);

		messageParams[9] =*((__IO uint8_t* )(&buffer[2]) + 3);
		messageParams[10] =*((__IO uint8_t* )(&buffer[2]) + 2);
		messageParams[11] =*((__IO uint8_t* )(&buffer[2]) + 1);
		messageParams[12] =*((__IO uint8_t* )(&buffer[2]) + 0);

		SendMessageToModule(module,CODE_PORT_FORWARD,(sizeof(float) * 3) + 1);

	}

}
/*-----------------------------------------------------------*/

void SampleDistanceToPort(uint8_t port,uint8_t module)
{
	float buffer[1]; // Three Samples X, Y, Z
	static uint8_t temp[4];

	SampleDistanceBuff(buffer);
	if(module == myID || module == 0){
		temp[0] =*((__IO uint8_t* )(&buffer[0]) + 3);
		temp[1] =*((__IO uint8_t* )(&buffer[0]) + 2);
		temp[2] =*((__IO uint8_t* )(&buffer[0]) + 1);
		temp[3] =*((__IO uint8_t* )(&buffer[0]) + 0);

		writePxITMutex(port,(char* )&temp[0],4 * sizeof(uint8_t),10);
	}
	else{
		messageParams[0] =port;
		messageParams[1] =*((__IO uint8_t* )(&buffer[0]) + 3);
		messageParams[2] =*((__IO uint8_t* )(&buffer[0]) + 2);
		messageParams[3] =*((__IO uint8_t* )(&buffer[0]) + 1);
		messageParams[4] =*((__IO uint8_t* )(&buffer[0]) + 0);
		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float) + 1);
	}
}
/*-----------------------------------------------------------*/
void SampleTemperatureToPort(uint8_t port,uint8_t module)
{
	float buffer[1];
	static uint8_t temp[4];

	SampleTemperatureBuf(buffer);

	if(module == myID || module == 0){
		temp[0] =*((__IO uint8_t* )(&buffer[0]) + 3);
		temp[1] =*((__IO uint8_t* )(&buffer[0]) + 2);
		temp[2] =*((__IO uint8_t* )(&buffer[0]) + 1);
		temp[3] =*((__IO uint8_t* )(&buffer[0]) + 0);

		writePxITMutex(port,(char* )&temp[0],4 * sizeof(uint8_t),10);
	}
	else{
		messageParams[0] =port;
		messageParams[1] =*((__IO uint8_t* )(&buffer[0]) + 3);
		messageParams[2] =*((__IO uint8_t* )(&buffer[0]) + 2);
		messageParams[3] =*((__IO uint8_t* )(&buffer[0]) + 1);
		messageParams[4] =*((__IO uint8_t* )(&buffer[0]) + 0);

		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float) + 1);
	}

}
/*-----------------------------------------------------------*/
void SampleHumidityToPort(uint8_t port,uint8_t module)
{
	float buffer[1];
	static uint8_t temp[4];

	SampleHumidityBuf(buffer);
	if(module == myID || module == 0){
		temp[0] =*((__IO uint8_t* )(&buffer[0]) + 3);
		temp[1] =*((__IO uint8_t* )(&buffer[0]) + 2);
		temp[2] =*((__IO uint8_t* )(&buffer[0]) + 1);
		temp[3] =*((__IO uint8_t* )(&buffer[0]) + 0);

		writePxITMutex(port,(char* )&temp[0],4 * sizeof(uint8_t),10);
	}
	else{
		messageParams[0] =port;
		messageParams[1] =*((__IO uint8_t* )(&buffer[0]) + 3);
		messageParams[2] =*((__IO uint8_t* )(&buffer[0]) + 2);
		messageParams[3] =*((__IO uint8_t* )(&buffer[0]) + 1);
		messageParams[4] =*((__IO uint8_t* )(&buffer[0]) + 0);

		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float) + 1);
	}
}
/*-----------------------------------------------------------*/
void SamplePIRToPort(uint8_t port,uint8_t module)
{
	float buffer;
	bool temp;

	SamplePIRBuf(&buffer);

	if(module == myID || module == 0){
		temp =buffer;
		writePxITMutex(port,(char* )&temp,sizeof(bool),10);
	}
	else{
		messageParams[0] =port;
		messageParams[1] =buffer;
		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(char) + 1);
	}
}
/*-----------------------------------------------------------*/

void SampleColorBuf(float *buffer)
{
	uint16_t rgb[3];
	SampleColor(rgb,rgb+1,rgb+2);
	buffer[0]=rgb[0];
	buffer[1]=rgb[1];
	buffer[2]=rgb[2];
}
/*-----------------------------------------------------------*/
void SampleDistanceBuff(float *buffer)
{
	uint16_t distance;
	SampleDistance(&distance);
	*buffer = distance;
}
/*-----------------------------------------------------------*/
void SampleTemperatureBuf(float *buffer)
{
	SampleTemperature(buffer);
}
/*-----------------------------------------------------------*/
void SampleHumidityBuf(float *buffer)
{
	SampleHumidity(buffer);
}
/*-----------------------------------------------------------*/
void SamplePIRBuf(float *buffer)
{
	bool pir;
    SamplePIR(&pir);
    *buffer = pir;
}
/*-----------------------------------------------------------*/

void SampleColorToString(char *cstring, size_t maxLen)
{
	uint16_t red = 0, green = 0, blue = 0;
	SampleColor(&red, &green, &blue);
	Red=red;
	Green=green;
	Blue=blue;
	snprintf(cstring, maxLen, "Red: %d, Green: %d, Blue: %d\r\n", red, green, blue);
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
	temp=temprature;
	char Number[5]={0};
	uint16_t x;
	volatile uint32_t temp1=1;
	uint16_t x0,x1,x2;
	temp1=temp;

x0 = (uint8_t) (temp*10 - temp1*10);
x1 = (uint8_t) (temp1%10);
x2 = (uint8_t) (temp1/10%10);

if(x2 == 0)  {Number[0] = 0x20;} else {Number[0] = x2 +0x30;}
Number[1] = x1 +0x30;
Number[2] = '.';
Number[3] = x0 +0x30;
Number[4] = 0;
//    x=*((long long*)&temp);
//	temp=atoff(Number);
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


void SampleTemperature(float *temperature)
{

	buf[0] = tempReg;
	HAL_I2C_Master_Transmit(&hi2c2, tempHumAdd, buf, 1, HAL_MAX_DELAY);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c2, tempHumAdd, buf, 2, HAL_MAX_DELAY);
	val = buf[0] << 8 | buf[1];
	*temperature=((float)val/65536)*165.0-40.0;



}
/*-----------------------------------------------------------*/
void SampleColor(uint16_t *Red, uint16_t *Green, uint16_t *Blue)
{
	*Red = Read_Word(redReg);
	*Green = Read_Word(greenReg);
	*Blue = Read_Word(blueReg);
}
/*-----------------------------------------------------------*/
void SampleDistance(uint16_t *distance)
{
//	*distance = Read_Word(distanceReg)/6.39;
	/*   */
	uint16_t median_value;
	uint16_t count = 0;
	float summ = 0.0f;



		while(count<300)
		{	count++;
		    summ+=Read_Word(distanceReg);
			Delay_ms(1.5);
		}
median_value=1078-(summ/300);
		if(median_value>=1020&&median_value<1080)
			*distance=100;

		if(median_value>=1010&&median_value<1020)
			*distance=90+(median_value%10);

		if(median_value>=1000&&median_value<1010)
			*distance=80+(median_value%10);

		if(median_value>=990&&median_value<1000)
			*distance=70+(median_value%10);

		if(median_value>=970&&median_value<990)
			*distance=60+((median_value%10)/2);

		if(median_value>=950&&median_value<970)
			*distance=50+((median_value%10)/2);

		if(median_value>=850&&median_value<950)
			*distance=40+(((median_value+50)%100)/10);

		if(median_value>=700&&median_value<850)
			*distance=30+((median_value-700)/15);

		if(median_value>=400&&median_value<700)
			*distance=20;

		if(median_value>=0&&median_value<400)
			*distance=0;

}

/*-----------------------------------------------------------*/
void SampleHumidity(float *humidity)
{
	buf[0] = humidityReg;
	HAL_I2C_Master_Transmit(&hi2c2, tempHumAdd, buf, 1, HAL_MAX_DELAY);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c2, tempHumAdd, buf, 2, HAL_MAX_DELAY);
	val = buf[0] << 8 | buf[1];
	*humidity = (((float)val*100)/65536);
}
/*-----------------------------------------------------------*/
void SamplePIR(bool *pir)
{
	*pir=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);/* USER CODE END WHILE */
	 Delay_ms(1000);
}

/*-----------------------------------------------------------*/
Module_Status StreamToPort(uint8_t port, uint8_t module, uint32_t Numofsamples, uint32_t timeout,All_Data function)
{
	Module_Status status =H0AR9_OK;
	tofMode =STREAM_TO_PORT;
	port2 =port;
	module2 =module;
	Numofsamples2 =Numofsamples;
	timeout2 =timeout;
	mode2 =function;
	return status;

}
/*-----------------------------------------------------------*/

Module_Status StreamToBuffer(float *buffer, uint32_t Numofsamples, uint32_t timeout,All_Data function)
{
	switch (function) {
		case Color:
			return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleColorBuf);
			break;
		case PIR:
			return StreamMemsToBuf(buffer, Numofsamples, timeout, SamplePIRBuf);
			break;
		case Humidity:
			return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleHumidityBuf);
			break;
		case Temperature:
			return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleTemperatureBuf);
			break;
		case Distance:
			return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleDistanceBuff);
			break;
		default:
			break;
	}

}

/*-----------------------------------------------------------*/
Module_Status StreamColorToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleColorToString);
}
/*-----------------------------------------------------------*/

Module_Status StreamDistanceToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleDistanceToString);
}
/*-----------------------------------------------------------*/

Module_Status StreamTemperatureToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleTemperatureToString);
}

/*-----------------------------------------------------------*/

Module_Status StreamHumidityToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleHumidityToString);
}
/*-----------------------------------------------------------*/

Module_Status StreamPIRToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SamplePIRToString);
}
/*-----------------------------------------------------------*/

void stopStreamMems(void)
{
	stopStream = true;
}
/* -----------------------------------------------------------------------
 |								Commands							      |
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
//				StreamColorToCLI(period, timeout);

			} else {
				StreamToPort(port, module, Numofsamples, timeout ,Color);

			}

		} else if (!strncmp(pSensName, distanceCmdName, strlen(distanceCmdName))) {
			if (portOrCLI) {
//				StreamDistanceToCLI(period, timeout);

			} else {
				StreamToPort(port, module, Numofsamples, timeout ,Distance);

			}

		}
		else if (!strncmp(pSensName, temperatureCmdName, strlen(temperatureCmdName))) {
			if (portOrCLI) {
//				StreamTemperatureToCLI(period, timeout);

			} else {
				StreamToPort(port, module, Numofsamples, timeout ,Temperature);

			}

		} else if (!strncmp(pSensName, humidityCmdName, strlen(humidityCmdName))) {
			if (portOrCLI) {
//				StreamHumidityToCLI(period, timeout);

			} else {
				StreamToPort(port, module, Numofsamples, timeout ,Humidity);

			}

		} else if (!strncmp(pSensName, pirCmdName, strlen(pirCmdName))) {
			if (portOrCLI) {
//				StreamPIRToCLI(period, timeout);

			} else {
				StreamToPort(port, module, Numofsamples, timeout ,PIR);

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
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	pcWriteBuffer[0] = '\0';
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Stopping Streaming MEMS...\r\n");

	stopStreamMems();
	return pdFALSE;
}


/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
