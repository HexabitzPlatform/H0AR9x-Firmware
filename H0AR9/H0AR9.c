/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
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
static bool stopStream = false;
extern I2C_HandleTypeDef hi2c2;

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
/* Private function prototypes -----------------------------------------------*/
static Module_Status StreamMemsToBuf( float *buffer, uint32_t period, uint32_t timeout, SampleMemsToBuffer function);
static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout, SampleMemsToPort function);
static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function);
static Module_Status PollingSleepCLISafe(uint32_t period);
void ExecuteMonitor(void);

/* Create CLI commands --------------------------------------------------------*/

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

	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

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
	uint16_t add =2, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
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
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,              //HALFWORD
						//TOBECHECKED
					RO_START_ADDRESS + add,array[i - 1][j]);
					add +=2;
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
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
			memcpy((uint8_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )&snipBuffer[j * 2]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )(snippets[s].cmd + j * 2));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
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
}

/*-----------------------------------------------------------*/
/* --- H0AR9 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H0AR9_OK;


	switch(code){

		default:
			result =H0AR9_ERR_UnknownMessage;
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
void SampleTemperatureToString(char *cstring, size_t maxLen)
{
	float temprature = 0;
	SampleTemperature(&temprature);
	temp=temprature;
	snprintf(cstring, maxLen, "Temperature: %.2f\r\n", temprature);
}
/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */

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
	*distance = Read_Word(distanceReg)/6.39;
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
}
/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */



/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
