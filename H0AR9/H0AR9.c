/*
    BitzOS (BOS)V0.2.6 - Copyright (C) 2017-2022 Hexabitz
    All rights reserved

    File Name     : H0AR9.c
    Description   : Source code for module H0AR9.
					Load cell (strain gauge) Whatstone bridge sensor (HX711)

		Required MCU resources :

			>> USARTs 2,3,4,5,6 for module ports.
			>> PA6 for HX711 RATE control.
			>> PA9 for HX711 PD_SCK.
			>> PA10 for HX711 DOUT.
			>> Gain of ch1 is fixed at 128.
			>> Gain of ch2 is fixed at 32.

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
extern I2C_HandleTypeDef hi2c1;
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


typedef void (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char *, size_t);
typedef void (*SampleMemsToBuffer)(float *buffer);


//temprature and humidity sensor addresses
static const uint8_t tempHumAdd = (0x40)<<1; // Use 7-bit address
static const uint8_t tempReg = 0x00;
static const uint8_t humidityReg = 0x01;

/*Define private variables*/
static bool stopStream = false;
extern I2C_HandleTypeDef hi2c1;

uint8_t buf[10];
uint8_t CONTROL, Enable, ATIME, WTIME, PPULSE;
uint8_t redReg, greenReg, blueReg, distanceReg;
uint16_t RED_data, GREEN_data, BLUE_data, Prox_data;
uint16_t val;
uint8_t pir;

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
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue =16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	__SYSCFG_CLK_ENABLE();

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);

}



/* --- Register this module CLI Commands
*/
void RegisterModuleCLICommands(void)
{
	FreeRTOS_CLIRegisterCommand( &SampleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StreamCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StopCommandDefinition);
}


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
void Module_Peripheral_Init(void)
{
	/* Array ports */
//	MX_USART1_UART_Init();
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

}

Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H0AR9_OK;
	uint32_t period = 0, timeout = 0;

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
			period = ((uint32_t) cMessage[port - 1][2 + shift] << 24) + ((uint32_t) cMessage[port - 1][3 + shift] << 16) + ((uint32_t) cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] << 24) + ((uint32_t) cMessage[port - 1][7 + shift] << 16) + ((uint32_t) cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][9 + shift];
			StreamColorToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], period, timeout);
			break;
		}

		case CODE_H0AR9_STREAM_DISTANCE:
		{
			period = ((uint32_t) cMessage[port - 1][2 + shift] << 24) + ((uint32_t) cMessage[port - 1][3 + shift] << 16) + ((uint32_t) cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] << 24) + ((uint32_t) cMessage[port - 1][7 + shift] << 16) + ((uint32_t) cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][9 + shift];
			StreamDistanceToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], period, timeout);
			break;
		}
		case CODE_H0AR9_STREAM_TEMP:
		{
			period = ((uint32_t) cMessage[port - 1][2 + shift] << 24) + ((uint32_t) cMessage[port - 1][3 + shift] << 16) + ((uint32_t) cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] << 24) + ((uint32_t) cMessage[port - 1][7 + shift] << 16) + ((uint32_t) cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][9 + shift];
			StreamTemperatureToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], period, timeout);
			break;
		}
		case CODE_H0AR9_STREAM_HUMIDITY:
		{
			period = ((uint32_t) cMessage[port - 1][2 + shift] << 24) + ((uint32_t) cMessage[port - 1][3 + shift] << 16) + ((uint32_t) cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] << 24) + ((uint32_t) cMessage[port - 1][7 + shift] << 16) + ((uint32_t) cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][9 + shift];
			StreamHumidityToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], period, timeout);
			break;
		}
		case CODE_H0AR9_STREAM_PIR:
		{
			period = ((uint32_t) cMessage[port - 1][2 + shift] << 24) + ((uint32_t) cMessage[port - 1][3 + shift] << 16) + ((uint32_t) cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] << 24) + ((uint32_t) cMessage[port - 1][7 + shift] << 16) + ((uint32_t) cMessage[port - 1][8 + shift] << 8) + cMessage[port - 1][9 + shift];
			StreamPIRToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift], period, timeout);
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

static Module_Status StreamMemsToBuf( float *buffer, uint32_t period, uint32_t timeout, SampleMemsToBuffer function)

{
	Module_Status status = H0AR9_OK;

	if (period < MIN_MEMS_PERIOD_MS)
		return H0AR9_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		function(buffer);

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H0AR9_ERR_TERMINATED;
			break;
		}
	}
	return status;
}

static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout, SampleMemsToPort function)
{
	Module_Status status = H0AR9_OK;


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
	return status;
}

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
//initialize APDS9950 sensor
    WriteRegData (Enable,0x00);
    WriteRegData (ATIME,0x00);
    WriteRegData (WTIME,0xff);
    WriteRegData (PPULSE,0x01);
    WriteRegData (CONTROL, 0x20);
    WriteRegData (Enable, 0x0F);
}


void SampleColor(uint16_t *Red, uint16_t *Green, uint16_t *Blue)
{
	*Red = Read_Word(redReg);
	*Green = Read_Word(greenReg);
	*Blue = Read_Word(blueReg);
}


void SampleDistance(uint16_t *distance)
{
	*distance = Read_Word(distanceReg)/6.39;
}

void SampleTemperature(float *temperature)
{
	buf[0] = tempReg;
	HAL_I2C_Master_Transmit(&hi2c1, tempHumAdd, buf, 1, HAL_MAX_DELAY);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, tempHumAdd, buf, 2, HAL_MAX_DELAY);
	val = buf[0] << 8 | buf[1];
	*temperature=((float)val/65536)*165.0-40.0;
}

void SampleHumidity(float *humidity)
{
	buf[0] = humidityReg;
	HAL_I2C_Master_Transmit(&hi2c1, tempHumAdd, buf, 1, HAL_MAX_DELAY);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, tempHumAdd, buf, 2, HAL_MAX_DELAY);
	val = buf[0] << 8 | buf[1];
	*humidity = (((float)val*100)/65536);
}

void SamplePIR(bool *pir)
{
	*pir=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);/* USER CODE END WHILE */
}

void SampleColorBuf(float *buffer)
{
	uint16_t rgb[3];
	SampleColor(rgb,rgb+1,rgb+2);
	buffer[0]=rgb[0];
	buffer[1]=rgb[1];
	buffer[2]=rgb[2];
}

void SampleDistanceBuff(float *buffer)
{
	uint16_t distance;
	SampleDistance(&distance);
	*buffer = distance;
}

void SampleTemperatureBuf(float *buffer)
{
	SampleTemperature(buffer);
}

void SampleHumidityBuf(float *buffer)
{
	SampleHumidity(buffer);
}

void SamplePIRBuf(float *buffer)
{
	bool pir;
    SamplePIR(&pir);
    *buffer = pir;
}

void SampleColorToPort(uint8_t port,uint8_t module)
{
	float buffer[3]; // Three Samples RED, GREEN, BLUE
	static uint8_t temp[12];

	SampleColorBuf(buffer);


	if(module == myID){
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

void SampleDistanceToPort(uint8_t port,uint8_t module)
{
	float buffer[1]; // Three Samples X, Y, Z
	static uint8_t temp[4];

	SampleDistanceBuff(buffer);


	if(module == myID){
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

		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float)+1);
	}
}

void SampleTemperatureToPort(uint8_t port,uint8_t module)
{
	float buffer[1];
	static uint8_t temp[4];

	SampleTemperatureBuf(buffer);


	if(module == myID){
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

		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float)+1);
	}
}

void SampleHumidityToPort(uint8_t port,uint8_t module)
{
	float buffer[1];
	static uint8_t temp[4];

	SampleHumidityBuf(buffer);


	if(module == myID){
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

		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float)+1);
	}
}

void SamplePIRToPort(uint8_t port,uint8_t module)
{
	float buffer;
	bool temp;

	SamplePIRBuf(&buffer);


	if(module == myID){
		temp = buffer;
		writePxITMutex(port,(char* )&temp,sizeof(bool),10);
	}
	else{
		messageParams[0] =port;
		messageParams[1] =buffer;
		SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(char)+1);
	}
}

void SampleColorToString(char *cstring, size_t maxLen)
{
	uint16_t red = 0, green = 0, blue = 0;
	SampleColor(&red, &green, &blue);
	Red=red;
	Green=green;
	Blue=blue;
	snprintf(cstring, maxLen, "Red: %d, Green: %d, Blue: %d\r\n", red, green, blue);
}

void SampleDistanceToString(char *cstring, size_t maxLen)
{
	uint16_t distance = 0;
	SampleDistance(&distance);
	distance1=distance;
	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);
}

void SampleTemperatureToString(char *cstring, size_t maxLen)
{
	float temprature = 0;
	SampleTemperature(&temprature);
	temp=temprature;
	snprintf(cstring, maxLen, "Temperature: %.2f\r\n", temprature);
}

void SampleHumidityToString(char *cstring, size_t maxLen)
{
	float humidity = 0;
	SampleHumidity(&humidity);
	hum=humidity;
	snprintf(cstring, maxLen, "Humidity: %.2f\r\n", humidity);
}

void SamplePIRToString(char *cstring, size_t maxLen)
{
	bool sample;
	SamplePIR(&sample);
	Sample=sample;
	snprintf(cstring, maxLen, "PIR: %d\r\n", sample);
}


Module_Status StreamColorToBuffer(float *buffer, uint32_t period, uint32_t timeout)
{
	 return StreamMemsToBuf(buffer, period, timeout, SampleColorBuf);
}

Module_Status StreamDistanceToBuffer(float *buffer, uint32_t period, uint32_t timeout)
{
	return StreamMemsToBuf(buffer, period, timeout, SampleDistanceBuff);
}

Module_Status StreamTemperatureToBuffer(float *buffer, uint32_t period, uint32_t timeout)
{
	return StreamMemsToBuf(buffer, period, timeout, SampleTemperatureBuf);
}

Module_Status StreamHumidityToBuffer(float *buffer, uint32_t period, uint32_t timeout)
{
	return StreamMemsToBuf(buffer, period, timeout, SampleHumidityBuf);
}

Module_Status StreamPIRToBuffer(float *buffer, uint32_t period, uint32_t timeout)
{
	return StreamMemsToBuf(buffer, period, timeout, SamplePIRBuf);
}

Module_Status StreamColorToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleColorToPort);
}

Module_Status StreamDistanceToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleDistanceToPort);
}

Module_Status StreamTemperatureToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleTemperatureToPort);
}

Module_Status StreamHumidityToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleHumidityToPort);
}

Module_Status StreamPIRToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SamplePIRToPort);
}

Module_Status StreamColorToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleColorToString);
}


Module_Status StreamDistanceToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleDistanceToString);
}

Module_Status StreamTemperatureToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleTemperatureToString);
}


Module_Status StreamHumidityToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SampleHumidityToString);
}

Module_Status StreamPIRToCLI(uint32_t period, uint32_t timeout)
{
	return StreamMemsToCLI(period, timeout, SamplePIRToString);
}

void stopStreamMems(void)
{
	stopStream = true;
}

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

static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	const char *const colorCmdName = "color";
	const char *const distanceCmdName = "distance";
	const char *const temperatureCmdName = "temp";
	const char *const humidityCmdName = "humidity";
	const char *const pirCmdName = "pir";

	uint32_t period = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &period, &timeout, &port, &module)) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, colorCmdName, strlen(colorCmdName))) {
			if (portOrCLI) {
				StreamColorToCLI(period, timeout);

			} else {
				StreamColorToPort(port, module, period, timeout);

			}

		} else if (!strncmp(pSensName, distanceCmdName, strlen(distanceCmdName))) {
			if (portOrCLI) {
				StreamDistanceToCLI(period, timeout);

			} else {
				StreamDistanceToPort(port, module, period, timeout);

			}

		}
		else if (!strncmp(pSensName, temperatureCmdName, strlen(temperatureCmdName))) {
			if (portOrCLI) {
				StreamTemperatureToCLI(period, timeout);

			} else {
				StreamTemperatureToPort(port, module, period, timeout);

			}

		} else if (!strncmp(pSensName, humidityCmdName, strlen(humidityCmdName))) {
			if (portOrCLI) {
				StreamHumidityToCLI(period, timeout);

			} else {
				StreamHumidityToPort(port, module, period, timeout);

			}

		} else if (!strncmp(pSensName, pirCmdName, strlen(pirCmdName))) {
			if (portOrCLI) {
				StreamPIRToCLI(period, timeout);

			} else {
				StreamPIRToPort(port, module, period, timeout);

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

static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	pcWriteBuffer[0] = '\0';
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Stopping Streaming MEMS...\r\n");

	stopStreamMems();
	return pdFALSE;
}

uint8_t SaveToRO(void)
{
	BOS_Status result = BOS_OK;
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 2, temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t)+1] = {0};

	HAL_FLASH_Unlock();

	/* Erase RO area */
	FLASH_PageErase(RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if (myID)
	{
		temp = (uint16_t) (N<<8) + myID;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS, temp);
		FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

	/* Save topology */
		for(uint8_t i=1 ; i<=N ; i++)
		{
			for(uint8_t j=0 ; j<=MaxNumOfPorts ; j++)
			{
				if (array[i-1][0]) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS+add, array[i-1][j]);
					add += 2;
					FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s=0 ; s<numOfRecordedSnippets ; s++)
	{
		if (snippets[s].cond.conditionType)
		{
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy( (uint8_t *)&snipBuffer[1], (uint8_t *)&snippets[s], sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j=0 ; j<(sizeof(snippet_t)/2) ; j++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)&snipBuffer[j*2]);
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j=0 ; j<((strlen(snippets[s].cmd)+1)/2) ; j++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)(snippets[s].cmd+j*2));
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO ---
*/
uint8_t ClearROtopology(void)
{
	// Clear the array
	memset(array, 0, sizeof(array));
	N = 1; myID = 0;

	return SaveToRO();
}
/*-----------------------------------------------------------*/


/* --- Get the port for a given UART.
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
			return P1;
	else if (huart->Instance == USART2)
			return P2;
	else if (huart->Instance == USART6)
			return P3;
	else if (huart->Instance == USART3)
			return P4;
	else if (huart->Instance == USART1)
			return P5;
	else if (huart->Instance == USART5)
			return P6;
	return 0;
}

/*-----------------------------------------------------------*/
