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
uint8_t f;
uint8_t str[13] = "helloaaaaaaaa";
uint8_t str1[10] = "abdulrhman";
uint8_t str2[11] = "hello world";
/* User Task */
void UserTask(void *argument) {
//	AddPortButton(MOMENTARY_NO, 2);   //Define a button connected to port P1
//		SetButtonEvents(2, 1, 0, 3, 0, 0, 0, 0, 0,1);
//	StartScastDMAStream(P5, 1, P6, 1, FORWARD, 10, 0xffffffff, false);
//	StreamToModule(5, 2, str1, 10, 0xffffffff, 2);
////	HAL_Delay(3000);
//	StreamToModule(5, 2, str, 13, 0xffffffff, 2);
////	HAL_Delay(3000);
//	StreamToModule(5, 2, str2, 11, 0xffffffff, 2);
////	HAL_Delay(3000);
//	StreamToModule(5, 2, str1, 10, 0xffffffff, 2);
////	HAL_Delay(3000);
//	StreamToModule(5, 2, str, 13, 0xffffffff, 2);
////	HAL_Delay(3000);
//	StreamToModule(5, 2, str2, 11, 0xffffffff, 2);
//	StreamToModule(5, 2, str1, 10, 0xffffffff, 2);
//
//	SendMessageToModule(2, CODE_PING, 0);
//	SendMessageToModule(2, CODE_PING, 0);
//	SendMessageToModule(2, CODE_PING, 0);
//	SendMessageToModule(2, CODE_PING, 0);
//
//
//	StreamToModule(5, 2, str2, 11, 0xffffffff, 2);
//	StreamToModule(5, 2, str, 13, 0xffffffff, 2);
//
//	StreamToModule(5, 2, str, 13, 0xffffffff, 1);
//	HAL_Delay(3000);
//	StreamToModule(5, 2, str2, 10, 0xffffffff, 1);
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
//	StartScastDMAStream(P3, 1, P6, 2, BIDIRECTIONAL, 100, 0xFFFFFFFF, false);
	// put your code here, to run repeatedly.
	HAL_Delay(500);
	while (1) {
//		if(f==1)
//		{
//			Bridge(6,2);
//			f=0;
//		}
//
//		if(f==2)
//		{
//			Unbridge(6, 2);
//			f=0;
//		}
//		SendMessageToModule(3, CODE_PING, 0);
//		HAL_Delay(500);
//		SendMessageToModule(2, CODE_PING, 0);
//		HAL_Delay(500);
//		SendMessageToModule(3, CODE_PING, 0);
//				HAL_Delay(500);
//		SampleTemperature(&TempBuffer[Index11]);
//		HAL_Delay(50);
//		Index11++;
//		if (Index11 > 100)
//			Index11 = 0;

	}
}
void buttonClickedCallback(uint8_t port){
	SendMessageToModule(2, CODE_PING, 0);
}
/*-----------------------------------------------------------*/
