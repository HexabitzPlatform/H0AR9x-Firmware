/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){
	StreamTemperatureToPort(2, 0, 100, 5000);
//	StreamColorToPort(2, 1, 100, 5000);
//	StreamDistanceToPort(2, 5, 100, 5000);
	// put your code here, to run repeatedly.
	while(1){

	}
}

/*-----------------------------------------------------------*/
