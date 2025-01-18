/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
float TempBuffer[100];
uint16_t Index11 = 0;

volatile float VAR1 = 1.3;
volatile float VAR2 = 2.3;
volatile float VAR3 = 3.3;
volatile float VAR4 = 4.3;
volatile float VAR5 = 5.3;
volatile float VAR6 = 6.3;
volatile float VAR7 = 7.3;
volatile float VAR8 = 8.3;
volatile float VAR9 = 9.3;
volatile float VAR10 = 10.5;


/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void) {

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for (;;) {
	}
}

/*------------------------------------------------------------*/

/* User Task */
void UserTask(void *argument) {

//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR1);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR2);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR3);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR4);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR5);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR6);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR7);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR8);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR9);
//	AddBOSvar(FMT_FLOAT, (uint32_t) &VAR10);

	// put your code here, to run repeatedly.
	while (1) {
//		SampleTemperature(&TempBuffer[Index11]);
//		HAL_Delay(50);
//		Index11++;
//		if (Index11 > 100)
//			Index11 = 0;

	}
}

/*-----------------------------------------------------------*/
