/*
 * RFDriver_Si4432.h
 *
 *  Created on: MAY 16, 2024
 *      Author: MUDASSAR AHMED
 */
/**
*	This Driver file is Sole Property of the company named "bluell.se".
*   This RF driver covers All the functionality of the SI4432 RF Module
*   connected with STM32 MCU.
*/
#include "stm32l0xx_hal.h"
#include <main.h>

#ifdef debug
#include <stdio.h>  // For s_print_f function
#include "string.h"	// For string manipulation
#endif

#ifndef RFDriver_Si4432_H
#define RFDriver_Si4432_H

//#define noDelay

	/// Interrupts Type Definition
	enum interrupt_t
	{
		INT_RX_CRC_ERROR			=	0x0001,
		INT_RX_VALID_PACKET			=	0x0002,
		INT_TX_PACKET_SENT			=	0x0004,
		INT_EXTERNAL				=	0x0008,
		INT_RX_FIFO_ALMOST_FUL		=	0x0010,
		INT_TX_FIFO_ALMOST_EMPTY	=	0x0020,
		INT_TX_FIFO_ALMOST_FUL		=	0x0040,
		INT_FIFO_ERROR				=	0x0080,
		INT_POWER_ON_RESET			=	0x0100,
		INT_CHIP_READY				=	0x0200,
		INT_LOW_BATTERY_DETECT		=	0x0400,
		INT_WAKE_UP_TIMER			=	0x0800,
		INT_RSSI_HIGH				=	0x1000,
		INT_INVALID_PREAMBLE_DET	=	0x2000,
		INT_VALID_PREAMBLE_DET		=	0x4000,
		INT_SYNC_WORD_DET			=	0x8000
	};

	/// Operating Mode Type Definition
	enum opMode_t
	{
		STANDBY		=	0x00,
		IDLE_READY	=	0x01,
		TUNE		=	0x02,
		RX			=	0x04,
		TX			=	0x08,
		XTAL32		=	0x10,
		SLEEP		=	0x20,
		SENSOR		=	0x40,
		Soft_Reset	=	0x80
	};

	/// Modulation Type Definitions
	enum modulation_t
	{
		UNMODULATED,
		OOK,
		FSK,
		GFSK
	};

	/// CRC Polynomials Type Definition.
	enum crcPolynomial_t
	{
		CCIT,
		CRC_16_IBM,
		IEC_16,
		BIACHEVA
	};

	/// Parametric Constructor.
	HAL_StatusTypeDef RFDriver_Si4432_begin(	SPI_HandleTypeDef*	hspi,
							GPIO_TypeDef*		nSelPort,	uint16_t	nSelPin,
							GPIO_TypeDef*		sdnPort,	uint16_t	sdnPin,
							GPIO_TypeDef*		nIrqPort,	uint16_t	nIrqPin);

	//Constructor Overloading.
	/// Parametric Constructor.
	HAL_StatusTypeDef RFDriver_Si4432_begin_(	SPI_HandleTypeDef*	hspi,
							GPIO_TypeDef*		nSelPort,	uint16_t	nSelPin,
						//	GPIO_TypeDef*		sdnPort,	uint16_t*	sdnPin,
							GPIO_TypeDef*		nIrqPort,	uint16_t	nIrqPin);

	///-1-
	/*
		Function to Read Device Type.
		-	*type:	Variable to get device Type.
		Expected Type.
		->	EZRadioPRO: 01000.
	*/
	HAL_StatusTypeDef getDeviceType(uint8_t* type);

	///-2-
	/*
		Function to Read Device Version.
		-	*version:	Variable to get device Version.
		Expected Versions.
		->	Si4430/31/32 REV B1: 00110.
		->	Si100x REV C, Si101x REV A, Si102x/3x REV A: 00110.
		->	Si100x REV E, Si101x REV B: Si102x/3x REV B: 00111.
	*/
	HAL_StatusTypeDef getDeviceVersion(uint8_t* version);

	///-3-
	/*
		Function to Read Device Status.
		-	*deviceStatus:	Variable to get device Status.
		Expected Status.
		->	FIFO Overflow both TX/RX.
		->	FIFO Underflow both TX/RX.
		->	FIFO Empty RX only.
		->	Header Error.
		->	Frequency Error.
		->	Chip Power State.
			->	00: Idle State.
			->	01:	RX State.
			->	10:	TX State.
	*/
	HAL_StatusTypeDef getDeviceStatus(uint8_t* deviceStatus);

	///-4-
	/*
		Function to Read Interrupts Status.
		-	*interrupt:	Interrupts vector Table.
		** Note: This function gets triggered when "nIRQ" pin transition from HIGH to LOW.
	*/
	HAL_StatusTypeDef readAndClearInterrupts_(uint16_t* interrupts);

	///-5-
	HAL_StatusTypeDef readAndClearInterrupts();

	///-6-
	HAL_StatusTypeDef setInterrupts(uint16_t setInterruptValue);

	///-6-
//	HAL_StatusTypeDef clearInterrupts();

	///-7-
	HAL_StatusTypeDef resetInterrupts();

	///-8-
	/*
	 	 Function to Shutdown the Module using SDN pin.
	 	 -	sdnEn:	Shutdown Enable | Disable (SET | RESET).
	 	 **Note:	"RF_SDN" Must be defined in the main program to use this function.
					Otherwise it will return "HAL_ERROR".
	*/
	HAL_StatusTypeDef shutdown(_Bool sdnEn);

	///-9-
	/*
		Function to Set Operating Mode.
		-	mode:	Variable to Set Operating Mode.
		Expected Modes.
		->	Idle Mode.
		 	 ->	 Standby Mode.
		 	 ->	 Sleep Mode.
		 	 ->	 Sensor Mode (Low Battery Detect | Temperature Sensor).
		 	 ->	 Idle Ready Mode (Default).
		 	 ->	 Tune Mode.
		->	TX State.
		->	RX State.
		-
		 ** Shutdown can be achieved by using SDN Pin Only.
	*/
	HAL_StatusTypeDef setOperatingMode(uint8_t mode); /* opMode_t */

	///-10-
	/*
		Function to set Frequency of the RF Module.
		-	*Frequency (in MHz): 240.00 MHz -> 960.00 MHz.
	*/
	HAL_StatusTypeDef setFrequency(float frequency);

	///-10-
	/*
		Function to set Frequency of the RF Module.
		-	*Frequency (in MHz): 240.00 MHz -> 960.00 MHz.
	*/
	HAL_StatusTypeDef setFrequency_(float frequency, float frequencyOffset, uint8_t channel, uint8_t stepSize);



	///-11-
	/*
		Function to set Frequency Hopping Channel of the RF Module.
		-	channel (Multiples of StepSize)	:	0 -> 255	 => ( 0 -> 2.56MHz if stepSize is 1 ).
		-	stepSize (Multiples of 10kHz)	:	0 -> 255	 =>	( 0 -> 2.56MHz)
	*/
	HAL_StatusTypeDef setFrequencyHoppingChannel(uint8_t channel, uint8_t stepSize);

	///-12-
		/*
			Function to Read Frequency Setting from the RF Module.
			-	channel (Multiples of StepSize)	:	0 -> 255	 => ( 0 -> 2.56MHz if stepSize is 1 ).
			-	stepSize (Multiples of 10kHz)	:	0 -> 255	 =>	( 0 -> 2.56MHz)
		*/
	HAL_StatusTypeDef readFrequencySettings(float* frequency, float* frequencyOffsetValue);

	///-13-
	/*
		Function to Set TX Data Rate.
		-	*txDataRate:	Data Rate (in KBPS) 0.123 -> 256.
		**Note: In case of MANCHESTER Encoding Data rate is Limited to 128k.
	*/
	HAL_StatusTypeDef setTxDataRate(uint32_t txDataRate);

	///-14-
	/*
		Function to Set Modulation Type.
		-	Modulation Type (OOK, FSK, GFSK).
	*/
	HAL_StatusTypeDef setModulationType(uint8_t modulation);

	///-15-
	/*
		Function to Set Modulation Type
		-	modulation:		Modulation Type (OOK, FSK, GFSK).
		-	freqDeviation:	Frequency Deviation (625 Hz -> 320'000 Hz).
	*/
	HAL_StatusTypeDef setFrequencyParametersSettings(uint8_t modulation, uint32_t freqDeviation);

	///-16-
	/*
		Function to Set Modulation Type.
		-	TX Power Options for.
			-	SI4432:			+1,	+2,	+5,	+8,	+11, +14, +17, +20.
	*/
	HAL_StatusTypeDef setTxPower(uint8_t txPower);

	///-17-
	/*
		Function to Set Encoding.
		-	dataWhiteningEn:		Data Whitening Enable|Disable (SET|RESET).
		-	manchEncodingEn:		MANCHESTER Encoding Enable|Disable (SET|RESET).
		-	manchEncInvEn:			Inversion of MANCHESTER Encoding Enable|Disable (SET|RESET).
		-	manPreamblePolarity:	Pre-amble Polarity of MANCHESTER Encoding Set|Reset (SET|RESET).
	*/
	HAL_StatusTypeDef setEncoding(
			_Bool dataWhiteningEn, _Bool manchEncodingEn,
			_Bool manchInvEn, _Bool manchPreambPolarity);

	///-18-
	/*
		Function to Set Data Access Control like CRC, Enable TX/RX Packet Handling, LSB First & etc.
		-	crcEn:					CRC Cyclic Redundancy Check Enable | Disable (SET | RESET).
		-	crcDataOnlyEn:			CRC on Data Bytes only Enable | Disable (SET | RESET).
		-	crcPolySlct:			CRC Polynomials Selection from type definition of crcPolynomial_t.
		-	txPackHandleEn:			TX Packet Handling Enable | Disable (SET | RESET).
		-	txPackHandleEn:			RX Packet Handling Enable | Disable (SET | RESET).
		-	lsbFirstEn:				LSB Least Significant Bit First Enable | Disable (SET | RESET).
		-	skip2ndPhPreambDet:		Skip 2nd Phase of Pre-amble Detection Set | Reset (SET | RESET).
	*/
	HAL_StatusTypeDef setDataAccessControl(
			_Bool crcEn, _Bool crcDataOnlyEn, uint8_t crcPolySlct,
			_Bool txPackHandleEn, _Bool rxPackHandleEn, _Bool lsbFirstEn, _Bool skip2ndPhPreambDet);

	///-19-
	/*
		Function to Set Pre-amble Length (Must be used on Transmitting End).
		-	preambLength:		Pre-amble Length (1 -> 511 Nibbles).
		-	preambThresh:		Pre-amble Detection Threshold (1 -> 15 Nibbles).
	*/
	HAL_StatusTypeDef setPreamble(uint16_t preambLength, uint8_t preambThresh);

	///-20-
	/*
		Function to Set Synchronization Word.
		-	syncWord:	Synchronization Word (usually 1 -> 4 bytes).
		-	size:		Size of Synch Word.
	*/
	HAL_StatusTypeDef setSyncWord(uint8_t* syncWord, size_t size);

	///-21-
	/*
		Function to Set Transmit Header Word.
		-	*txHeader:	Header Word (usually 1 -> 4 bytes).
		-	size:		Size of Synch Word.
	*/
	HAL_StatusTypeDef setTxHeader(uint8_t* txHeader, size_t size);

	///-22-
	/*
		Function to set Received Header Check Word
		-	*rxHeader:	Receive Header Check Word (usually 1 -> 4 bytes)
		-	size:		Size of Synch Word.
	*/
	HAL_StatusTypeDef setRxCheckHeader(uint8_t* rxCheckHeader, size_t size);

	///-23-
	/*
		Function to get Received Header Word.
		-	*rxHeader:	Synchronization Word (usually 1 -> 4 bytes).
		-	size:		Size of Synch Word.
	*/
	HAL_StatusTypeDef getRxHeader(uint8_t* receivedHeader, size_t size);

	///-24-
	/*
		 Function to Enable/Disable Low Battery Mode.
		 -	lowBattThresh:	Input Low Battery Threshold Level (0% -> 100%) Means (1.7V -> 3.2599V) Volts.
		 -	interruptEnable:	Input Interrupt Enable (SET/RESET).
		 ** Note: To use this function Enable Sensor Mode using operatingMode Function.
	*/
	HAL_StatusTypeDef setLowBatteryLevel(uint8_t lowBattThresh, _Bool interruptEnable);

	///-25-
	/*
		 Function to Enable/Disable Low Battery Mode.
		 -	*batteryVoltLevel:	Output Battery Voltage Level (1.7 -> 3.25) Volts.
	*/
	HAL_StatusTypeDef getBatteryLevel(uint8_t* batteryVoltLevel);

	///-26-
	/*
		Function to Set Encoding.
		-	*time:		Wake-up Time after (in Milliseconds).
	*/
	HAL_StatusTypeDef setWakeupTimer(uint8_t* time);

	///-27-
	/*
		Function to Set Encoding.
		-	*dutyCycle:		Low Power On Time Duty (in Milliseconds).
	*/
	HAL_StatusTypeDef setLowDutyCycleMode(uint8_t* dutyCycle);

	/// GPIO Configuration. Antenna Diversity.

	///-28-
	/*
		Function to Set Minimum RSSI Threshold.
		-	rssiThresh:		RSSI Threshold for indication of Low Signals Level (in dB).
		**Note:	Check the status of iRSSI in Interrupt/Status 2 Register.
	*/
	HAL_StatusTypeDef setRssiThresh(const uint8_t rssiThresh);

	///-29-
	/*
		Function to Set Minimum RSSI Threshold.
		-	rssiOffset:		Offset value to RSSI Calculation (1 -> 7) => (+4dB -> 28dB).
		**Note:	Check the status of iRSSI in Interrupt/Status 2 Register.
	*/
	HAL_StatusTypeDef setRssiOffset(const uint8_t rssiOffset);

	///-30-
	/*
		Function to Get RSSI Value.
		-	*rssiValue:		RSSI Threshold for indication of Low Signals Level (in dB).
	*/
	HAL_StatusTypeDef getRSSI(uint8_t* rssiValue);

	///-30-
	/*
		Function to Read Received Data from the RF Module.
		-
		-
	*/
	HAL_StatusTypeDef startListening();

	///-31-
	/*
		Function to Check, does the Data Packet Received.
		-
		-
	*/
	_Bool isDataPacketReceived();

	///-32-
	/*
		Function to Send Data from the RF Module.
		-	*data:		Data to Write on the Transmit Registers.
		-	size:		Size of the TX Data.
	*/
	_Bool sendDataPacket_getResponse(uint8_t* data, uint8_t size, _Bool waitForAckResponse, uint32_t responseTimeout, uint8_t* responseBuffer, uint8_t* responseSize);

	_Bool sendDataPacket(uint8_t* data, uint8_t size);

	///-33-
	_Bool waitForAckPacket(uint32_t waitingTime);

	///-32-
	/*
		Function to Get Received Data from the RF Module.
		-	*data:		Data to Read from the Received Registers.
		-	size:		Size of the RX Data.
	*/
	HAL_StatusTypeDef getReceivedDataPacket(uint8_t* data, uint8_t* size);

	///-33-
	HAL_StatusTypeDef clearTxFIFO();

	///-34-
	HAL_StatusTypeDef clearRxFIFO();

	///-35-
	HAL_StatusTypeDef clearFIFOs();

	///-36-
	HAL_StatusTypeDef softReset();

	///-37-
	HAL_StatusTypeDef hardReset();

	///-38-
	HAL_StatusTypeDef reloadConfigurations();

	///-39-
	/*
		Function to Read Module Setting from the RF Module.
		-	*settings: Acquire Setting from the RF Module.
		-
	*/
	HAL_StatusTypeDef readSettings(uint8_t* settings);

	///-40-
	/// Function to Read Settings from the RF module through SPI.
	HAL_StatusTypeDef readRegisters(uint8_t* registers, uint8_t size);

#endif /* RFDriver_Si4432_H */
