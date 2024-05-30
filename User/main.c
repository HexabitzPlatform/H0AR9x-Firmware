/*
 BitzOS (BOS) V0.3.3 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void) {

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for (;;) {
	}
}

/*----------- ------------------------------------------------*/
uint16_t Red ,Green ,Blue ,d;
float ww ;
Module_Status s ;
bool g  ;

/* User Task */
void UserTask(void *argument) {

	// put your code here, to run repeatedly.
	while (1) {
//		SampleTemperature(&ww) ;
		SampleHumidity(&ww);
//		SampleDistance(&d);
//		 s= SampleColor(&Red, &Green, &Blue);
	}
}

/*-----------------------------------------------------------*/
