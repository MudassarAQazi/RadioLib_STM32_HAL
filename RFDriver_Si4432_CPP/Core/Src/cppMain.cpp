/*
 * cppMain.cpp
 *
 *  Created on: Aug 2, 2024
 *      Author: Mudassar Ahmed
 */
#include <cppMain.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#include <Timer_milliseconds.h>
#include <RFDriver_Si4432.h>


/* LOG UART BUFFER VARIABLE */
char		msg[500]	=	{0};

/* HEART BEAT LED VARIABLES BEGINS */
uint32_t	heartBeatEpoch		=	0,
			heartBeatDuration	=	750;
/* HEART BEAT LED VARIABLES ENDS */

/* --------------------- [RF Variables STARTs] ---------------------------*/

uint32_t	rfModuleBatteryLevelEpoch		=	0,
			rfModuleBatteryLevelDuration	=	10;
/* ---------------------- [RF Variables ENDs] ----------------------------*/



///*---------------------------------- [LOG UART Function STARTs] -----------------------------------*/
///
UART_HandleTypeDef* LOGUart;
void LOGInit(UART_HandleTypeDef* huart)
{
	LOGUart = huart;
}
/// LOG Function.
void LOG(char* buffer)
{
	HAL_UART_Transmit(LOGUart, (const uint8_t*)buffer, strlen(buffer), 1000);
	memset(buffer, 0, strlen(buffer));
}
///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [LOG UART Function ENDs] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/




///*------------------------------------------------------ [ Main Code STARTs ] -------------------------------------------------------*/
void cppMain()
{
  LOGInit(&debugUart);

  /** Initialize RF Driver Module **/
  RFDriver_Si4432 RF(	&RF_SPI,
		  	  	  	  	RF_nSEL_Si4432_GPIO_Port, 	RF_nSEL_Si4432_Pin,
		  	  	  	  	RF_SDN_Si4432_GPIO_Port,	RF_SDN_Si4432_Pin,
						RF_nIRQ_Si4432_GPIO_Port,	RF_nIRQ_Si4432_Pin
  	  	  	  	  	  );

//  RF.setOperatingMode(RF.IDLE_READY); /* Enable XTAL ON */
//  RF.reloadConfigurations();

  #if !defined(noDelay)
  HAL_Delay(500);
  #endif

  /** Read Battery Level **/
  uint8_t battLevel	=	0;
  RF.getBatteryLevel(&battLevel);
  sprintf(	msg,
		  	"Battery Level: %d\r\n",
			battLevel
	  	 );
  LOG(msg);

  #if !defined(noDelay)
  HAL_Delay(100);
  #endif

  /** Read Frequency Settings **/
  float freq		=	0;
  float freqOffset	=	0;
  RF.readFrequencySettings(&freq, &freqOffset);
  sprintf(	msg,
			"Frequency is Set: %ld.%ld \t\t Frequency Offset: %ld\r\n",
			(uint32_t(freq)),
			((uint32_t)(freq*1'000)%1'000),
			(int32_t)freqOffset
		  );
  LOG(msg);

  /** Read and Print RF Module Register Settings **/
  uint8_t regReadSettings[0x7F] = { 0 };
   RF.readSettings(regReadSettings);
   for(uint8_t i = 0; i < 0x7F; i++)
   /// ---------------------Exclude Reserved Registers Starts------------------------.
   if(	(i==0x2F) ||
		(i==0x4C) || (i==0x4D) || (i==0x4E) ||
		(i==0x50) || (i==0x51) || (i==0x52) || (i==0x53) || (i==0x54) || (i==0x55) || (i==0x56) || (i==0x57) || (i==0x58) || (i==0x59) || (i==0x5A) || (i==0x5B) || (i==0x5C) || (i==0x5D) || (i==0x5E) || (i==0x5F) ||
		(i==0x61) || (i==0x63) || (i==0x64) || (i==0x65) || (i==0x66) || (i==0x67) || (i==0x68) || (i==0x6A) || (i==0x6B) || (i==0x6C) ||
		(i==0x78) || (i==0x7B)
   	 ) continue;
   /// ---------------------Exclude Reserved Registers Ends------------------------.
   else
   {
 	  sprintf(	msg,
 			  	"Reg(0x%X),\tValue: 0x%X\r\n",
 				i, regReadSettings[i]
 			  );
 	  LOG(msg);
 	  #if !defined(noDelay)
 	  	  HAL_Delay(10);
 	  #endif
   }

///*------------------------------- [ RF Module Testing One Time Run Code BEGINs ] -----------------------------*/
//   #define TRANSMIT
   #define RECEIVE

   #ifdef TRANSMIT
      uint32_t txTime	=	0;
   #endif

   #ifdef RECEIVE
      uint32_t rxTime	=	0;
   #endif

///*------------------------------- [ RX Part Testing One Time Run Code BEGINs ] -----------------------------*/
   RF.startListening();
///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RX Part Testing One Time Run Code ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/


///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RF Module One Time Run Code Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

   while(1)
   {
///*-------------------------------- [ Heart Beat LED BEGINs ] -------------------------------*/
		if(timerCompleted(&heartBeatEpoch, &heartBeatDuration))
		{
			HAL_GPIO_TogglePin(heartBeatLed_GPIO_Port, heartBeatLed_Pin);
			HAL_Delay(0);
		}
///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [ Heart Beat LED ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
///
///*------------------------------- [ RF Module Testing BEGINs ] -----------------------------*/
//		if(timerCompleted(&rfModuleBatteryLevelEpoch, &rfModuleBatteryLevelDuration))
//		{
//			uint8_t devType = 0, devVersion = 0;
//			uint8_t batLevel = 0.0;
//			RF.getDeviceType(&devType);
//			RF.getDeviceVersion(&devVersion);
//			RF.getBatteryLevel(&batLevel);
////			uint16_t battLevel = (uint16_t)(batLevel * 100);
//			sprintf(msg,
//						"Battery Level: %d %% \t Device Type ID: 0x%x \t Device Version: 0x%x\r\n",
//						(uint8_t)(batLevel) ,
//						devType, devVersion
//					);
//			LOG(msg);
//		}
///*------------------------------- [ TX Part Testing BEGINs ] -----------------------------*/
#ifdef TRANSMIT
		if(timerCompleted(&rfModuleBatteryLevelEpoch, &rfModuleBatteryLevelDuration))
		{
			uint8_t txFifoData[]	=	{ 	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
											0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
											0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
											0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f
										};
			uint8_t rxFifoData[64]	=	{ 0 };
			uint8_t rxFifoSize	=	0;
			bool txResponse = RF.sendDataPacket(txFifoData, sizeof(txFifoData), SET, 1500, rxFifoData, &rxFifoSize);
			if(txResponse)
			{
				uint8_t txRSSI	=	0;
				RF.getRSSI(&txRSSI);
				sprintf(	msg,
							"\n-----Data Received-----\n RSSI: %d dB\n Time Duration: %lu\nData: \r\n ",
							txRSSI,
							(HAL_GetTick()	-	txTime)
				); LOG(msg);

				uint8_t size = 0;

				for(uint8_t i = 0; i < rxFifoSize; i++)
				{
					char str[20];
					size_t len = sprintf(str, "%02X, ", rxFifoData[i]);
					for(uint8_t j = 0; j < len; j++)
						msg[(uint8_t)size+j]	=	str[j];
					size+=(uint8_t)len;
				}
				LOG(msg);
				sprintf(	msg,
							"\n\r\n"
				);LOG(msg);

				txTime	=	HAL_GetTick();

			}
			else
			{
				sprintf(	msg,
							"Not Acknowledged! \r\n "
							);
				LOG(msg);
			}
		}
#endif /* TRANSMIT */
///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [TX Part Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/


///*------------------------------- [ RX Part Testing BEGINs ] -----------------------------*/
#ifdef RECEIVE

//			HAL_Delay(0);
			bool rxResponse = RF.isDataPacketReceived();
			if(rxResponse)
			{
				uint8_t rxData[64]	=	{ 0 };
				uint8_t rxDataSize	=	0;
				RF.getReceivedDataPacket(rxData, &rxDataSize);
				uint8_t rxRSSI	=	0;
				RF.getRSSI(&rxRSSI);
				sprintf(	msg,
							"\n-----Data Received-----\n RSSI: %d dB\n Time Duration: %lu\nData: \r\n ",
							rxRSSI,
							(HAL_GetTick()	-	rxTime)
				);	LOG(msg);

				uint8_t size = 0;

				for(uint8_t i = 0; i < rxDataSize; i++)
				{
					char str[20];
					size_t len = sprintf(str, "%02X, ", rxData[i]);
					for(uint8_t j = 0; j < len; j++)
						msg[(uint8_t)size+j]	=	str[j];
					size+=(uint8_t)len;
				}	LOG(msg);


				rxTime	=	HAL_GetTick();

				uint8_t txResponseData[]	=	{ 0xAE, 0x26, 0xD8, 0x93 };
//												{ 	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
//													0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
//													0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
//													0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f
//												};
				while(!RF.sendDataPacket(txResponseData, sizeof(txResponseData)));// HAL_Delay(1);

				sprintf(
							msg,
							"\n^^^^^ Response Sent ^^^^^\r\n "
				); LOG(msg);

				RF.startListening();
			}
#endif /* RECEIVE */
///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RX Part Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/



///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RF Module Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
	} /* WHILE 1 */
} /* MAIN */

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == RF_nIRQ_Si4432_Pin)
//	{
//		rfInterrupt	=	1;
//	}
//}






















