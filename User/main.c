/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
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

/*------------------------------------------------------------*/
//All_Data dataFunction;
//Module_Status sampleToPort_Status;

//
//All_Data dataFunction;
//uint32_t numOfSamples;
//uint32_t streamTimeout;
//Module_Status streamToPort_Status;
//
//
//All_Data dataFunction;
//uint32_t numOfSamples;
//uint32_t streamTimeout;
//Module_Status streamToTerminal_Status;
//
//
//float buffer[100];
//All_Data function;
//uint32_t Numofsamples;
//uint32_t timeout;
//Module_Status streamToBuffer_Status;

/* Global variables */
bool pir;
Module_Status pir_Status;
uint16_t distance;
Module_Status distance_Status;
uint16_t Red, Green, Blue;
Module_Status color_Status;
float temperature;
Module_Status temperature_Status;
float humidity;
Module_Status humidity_Status;


/* User Task */
void UserTask(void *argument){
//	streamToPort_Status = StreamToPort(0, 3, COLOR, 10, 10000);
//	streamToTerminal_Status = StreamToTerminal(3,COLOR, 10, 10000);
//	streamToBuffer_Status = StreamToBuffer(buffer,COLOR, 10, 10000);
	while (1) {
		/* Function calls */
//		pir_Status = SamplePIR(&pir);
//		distance_Status = SampleDistance(&distance);
//		color_Status = SampleColor(&Red, &Green, &Blue);
//		temperature_Status = SampleTemperature(&temperature);
//		humidity_Status = SampleHumidity(&humidity);
	  }

}
//  // put your code here, to run repeatedly.
//  while(1){
////    ReadADCChannel(P2,"top",&adcalue);
////    ReadADCChannel(P2,"bottom",&adcalue2);
////    ReadADCChannel(P1,"bottom",&adcalue3);
////    ReadADCChannel(P1,"top",&adcalue4);
//  }
//}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
