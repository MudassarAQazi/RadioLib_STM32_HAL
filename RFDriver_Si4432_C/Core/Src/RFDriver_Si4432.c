/*
 * RFDriver_Si4432.CPP
 *
 *  Created on: MAY 16, 2024
 *      Author: MUDASSAR AHMED
 */

#include <RFDriver_Si4432.h>

/// Variables
uint32_t	MAX_TRANSMIT_TIMEOUT	=	1500;
/// RF Interrupts Status Holding Register.
uint16_t 	interruptStatus			=	0;


static SPI_HandleTypeDef	*rf_spi;

static GPIO_TypeDef			*nSelPort;
static uint16_t				nSelPin	=	0;

static GPIO_TypeDef			*sdnPort;
static uint16_t				sdnPin	=	0;

static GPIO_TypeDef			*nIrqPort;
static uint16_t				nIrqPin	=	0;

/// Addresses of Data Register
static uint8_t			deviceTypeRegAddr		=	0x00,
						intrEn1RegAddr			=	0x05,
						intrEn2RegAddr			=	0x06,
						opFuncCtrlReg1Addr		=	0x07,
						opFuncCtrlReg2Addr		=	0x08,
						dataAcsCtrlRegAddr		=	0x30,
						headerCtrl1RegAddr		=	0x32,
						headerCtrl2RegAddr		=	0x33,
						moduloModeCtrl1RegAddr	=	0x70,
						moduloModeCtrl2RegAddr	=	0x71,
						fifoBaseRegAddr			=	0x7F;

/// Data Registers
static uint8_t 			interruptEnable1Reg 	=	0,	// 0x05 | 05h.
						interruptEnable2Reg 	=	0,	// 0x06 | 06h.
						operAndFuncCtrl1Reg 	=	0,	// 0x07 | 07h.
						dataAcsCtrlReg			=	0,	// 0x30 | 30h.
						headerCtrl1Reg			=	0,	// 0x32 | 32h.
						headerCtrl2Reg 			=	0,	// 0x33 | 33h.
						preambDetectionCtrlReg	=	0,	// 0x35 | 35h.
						moduloModeCtrl1Reg		=	0,	// 0x70 | 70h.
						moduloModeCtrl2Reg		=	0;	// 0X71 | 71h.

static uint8_t 			txRxReg[10] 			=	{ 0 };

static HAL_StatusTypeDef 		status 			= 	HAL_ERROR;








///-41-
/*
	Function to Read Received Data from the RF Module.
	-	*fifo:		Data to Read from the FIFO.
	-	size:		Size of the RX Data.
*/
static HAL_StatusTypeDef readDataFromRF(uint8_t* fifo, uint8_t* size);

///-42-
/*
	Function to Write Transmitting Data to the RF Module.
	-	*fifo:		Data to Send to the FIFO.
	-	size:		Size of the RX Data.
*/
static HAL_StatusTypeDef writeDataToRF(uint8_t* fifo, size_t size);

///-43-
/*
	Function to write data to RF SPI.
	-	*address:	Address Pointer for the register to be written.
	-	*data:		Data Pointer for the data array to be written.
	-	size:		Data Array size to be written.
*/
static HAL_StatusTypeDef write_Reg_To_RF_SPI(uint8_t* address, uint8_t* data, size_t size);

///-44-
/*
	Function to Read data from RF SPI.
	-	*address:	Address Pointer for the register to be written.
	-	*data:		Data Pointer for the data array to be written.
	-	size:		Data Array size to be written.
*/
static HAL_StatusTypeDef read_Reg_From_RF_SPI(uint8_t* address, uint8_t* data, size_t size);
///-45-
static double power(float x, int y);







/// Parametric Constructor.
HAL_StatusTypeDef RFDriver_Si4432_begin(	SPI_HandleTypeDef*	hspi,
											GPIO_TypeDef* 		nselport,	uint16_t	nselpin,
											GPIO_TypeDef*		sdnport,	uint16_t	sdnpin,
											GPIO_TypeDef*		nirqport,	uint16_t	nirqpin)
{
	rf_spi	=	hspi;
	nSelPort	=	nselport;	nSelPin	=	nselpin;
	sdnPort		=	sdnport;	sdnPin	=	sdnpin;
	nIrqPort	=	nirqport;	nIrqPin	=	nirqpin;

	HAL_GPIO_WritePin(sdnPort,	sdnPin,		GPIO_PIN_RESET);	// Reset (LOW) SDN pin to make the module work.
	HAL_GPIO_WritePin(nSelPort,	nSelPin,	GPIO_PIN_SET);		// Set (HIGH) the Chip Select pin -
	// 																- to disable the write operation for now.
	#if !defined(noDelay)
		HAL_Delay(1000);
	#endif
//	softReset();
//	setOperatingMode(SENSOR);

	reloadConfigurations();

	return HAL_OK;
}

// Constructor Overloading
/// Parametric Constructor.
HAL_StatusTypeDef RFDriver_Si4432_begin_(	SPI_HandleTypeDef*	hspi,
											GPIO_TypeDef* 		nselport,	uint16_t	nselpin,
										//	GPIO_TypeDef*		sdnport,	uint16_t	sdnpin,
											GPIO_TypeDef*		nirqport,	uint16_t	nirqpin)
{
	rf_spi		=	hspi;
	nSelPort	=	nselport;	nSelPin	=	nselpin;
	//sdnPort	=	sdnport;	sdnPin	=	sdnpin;
	nIrqPort	=	nirqport;	nIrqPin	=	nirqpin;

//	HAL_GPIO_WritePin(sdnPort,	*sdnPin,	GPIO_PIN_RESET);	// Reset (LOW) SDN pin to make the module work.
	HAL_GPIO_WritePin(nSelPort,	nSelPin,	GPIO_PIN_SET);		// Set (HIGH) the Chip Select pin -
	// 																- to disable the write operation for now.
//	softReset();
	#if !defined(noDelay)
		HAL_Delay(1000);
	#endif

//	setOperatingMode(SENSOR);

	reloadConfigurations();

	return HAL_OK;
}



///-40-
/// Function to Write Data into the RF module through SPI.
HAL_StatusTypeDef reloadConfigurations()
{

//#if !defined(noDelay)
//	HAL_Delay(2000);
//#endif
//	setOperatingMode(SENSOR);
//#if !defined(noDelay)
//	HAL_Delay(1000);
//#endif

	/// Write AFC Timing Settings.
	uint8_t AFCTimingControlRegAddr	=	0x1E;
	uint8_t AFCTimingControlReg		=	0x02;
	write_Reg_To_RF_SPI(&AFCTimingControlRegAddr, &AFCTimingControlReg, sizeof(AFCTimingControlReg));
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

	/// Write AFC Limiter Settings.
	uint8_t AFCLimterRegAddr		=	0x2A;
	uint8_t AFCLimiterReg			=	0xFF;
	write_Reg_To_RF_SPI(&AFCLimterRegAddr, &AFCLimiterReg, sizeof(AFCLimiterReg));
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

	/// Write AGC Override1 Settings.
	uint8_t AGCOverRide1RegAddr		=	0x69;
	uint8_t AGCOverRide1Reg			=	0x18;
	write_Reg_To_RF_SPI(&AGCOverRide1RegAddr, &AGCOverRide1Reg, sizeof(AGCOverRide1Reg));
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

	/// Write AFC Gear-shift Override Settings.
	uint8_t AFCLoopGearshiftOverrideRegAddr	=	0x1D;
	uint8_t AFCLoopGearshiftOverrideReg		=	0x3C;
	write_Reg_To_RF_SPI(&AFCLoopGearshiftOverrideRegAddr, &AFCLoopGearshiftOverrideReg, sizeof(AFCLoopGearshiftOverrideReg));
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

	/// Data Access and Control.
	setDataAccessControl(SET, SET, CRC_16_IBM, SET, SET, RESET, RESET); /** (crcEn, crcDataOnlyEn, crcPolySlct,txPackHandleEn, rxPackHandleEn, lsbFirstEn, skip2ndPhPreambDet) **/
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

	/// Set TX/RX Header + Check Header.
    uint8_t header[] = {0xDE, 0xAD};
    setTxHeader(header, sizeof(header));
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif
    setRxCheckHeader(header, sizeof(header));
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

    /// Set Pre-amble + Pre-amble Threshold.
    setPreamble(8, 7); // Pre-Amble: 10 Nibbles = 40 bit, Threshold: 7 Nibbles = 28 bits.
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

    /// Set Synch word.
    //  uint8_t synchWord[4] = {0xAE, 0x26, 0xD8, 0x93};
    uint8_t synchWord[] = {0x2D, 0xD4};
    setSyncWord(synchWord, sizeof(synchWord));
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

    /// Set TX Power.
    //  uint8_t txPower = 11; // 11dBm
    setTxPower(20);
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

    /// Set Frequency + Set Frequency Offset + Set Channel + Step Size (1 MHz)
    setFrequency_(433.00, 0, 0, 0x64); /** (frequency, frequencyOffset, channel, stepSize) **/
    //  RF.setFrequencyHoppingChannel(10, 1);
	#if !defined(noDelay)
    	HAL_Delay(10);
	#endif

    /// Set Air Baud Rate.
    //  uint32_t airBaud =	8000; // 8kbps
    setTxDataRate(70000);	//70000
	#if !defined(noDelay)
    	HAL_Delay(10);
	#endif

    /// Set Encoding Parameters.
    //setEncoding(SET, SET, SET, SET); /** (dataWhiteningEn, manchEncodingEn, manchInvEn, manchPreambPolarity) **/
    setEncoding(RESET, RESET, SET, SET); /** (dataWhiteningEn, manchEncodingEn, manchInvEn, manchPreambPolarity) **/
	#if !defined(noDelay)
    	HAL_Delay(10);
	#endif

    /// Set Frequency and Modulation Types.
    //  RF.setModulationType(RF.GFSK);
    setFrequencyParametersSettings( GFSK, 150000);
	#if !defined(noDelay)
    	HAL_Delay(10);
	#endif

	/// Set RSSI Offset.
//	setRssiOffset(0);
//	#if !defined(noDelay)
//		HAL_Delay(10);
//	#endif

    /// Set Operating Mode.
    setOperatingMode(SENSOR | IDLE_READY | TUNE);
	#if !defined(noDelay)
    	HAL_Delay(10);
	#endif


	/// Set Low Battery Threshold.
	setLowBatteryLevel(20, SET); /// 20%
	#if !defined(noDelay)
		HAL_Delay(10);
	#endif

    /// Read and Clear Interrupts.
//    readAndClearInterrupts();

	return status;
}


///-1-
/// Function to Read Device Type.
HAL_StatusTypeDef getDeviceType(uint8_t* type)
{
	/*
	 	 Read Device Type
	 	 -	EZRadioPRO: 01000.
	*/
	/// Local Variables.
	uint8_t deviceTypeRegAddr		=	0;

	/// Read Register 00h from SPI.
	status = read_Reg_From_RF_SPI(&deviceTypeRegAddr, type, (size_t)1);
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-2-
/// Function to Read Device Version.
HAL_StatusTypeDef getDeviceVersion(uint8_t* version)
{
	/*
	 	Read Device Status
	 	-	Si4430/31/32 REV B1: 00110.
		-	Si100x REV C, Si101x REV A, Si102x/3x REV A: 00110.
		-	Si100x REV E, Si101x REV B: Si102x/3x REV B: 00111.
	*/
	/// Local Variables.
	uint8_t deviceVersionRegAddr	=	0x01;

	/// Read Register 01h from SPI.
	status = read_Reg_From_RF_SPI(&deviceVersionRegAddr, version, (size_t)1);
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-3-
/// Function to Read Device Status.
HAL_StatusTypeDef getDeviceStatus(uint8_t* deviceStatus)
{
	/*
	 	Read Device Status
	 	-	FIFO Overflow both TX/RX.
	 	-	FIFO Underflow both TX/RX.
	 	-	FIFO Empty RX only.
	 	-	Header Error.
	 	-	Frequency Error.
		-	Chip Power State.
			-	00: Idle State
			-	01:	RX State
			-	10:	TX State
	*/
	/// Local Variables.
	uint8_t deviceStatusRegAddr		=	0x02;

	/// Read Register 02h from SPI.
	status = read_Reg_From_RF_SPI(&deviceStatusRegAddr, deviceStatus, (size_t)1);
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-4-
/// Function to Read Interrupt Status.
HAL_StatusTypeDef readAndClearInterrupts_(uint16_t* interrupts)
{
	/*
		Function to Read Interrupts Status
		-	This function gets triggered when "nIRQ" pin transitions from HIGH to LOW.
	*/
	/// Local Variables.
	uint8_t interruptStatusRegAddr	=	0x04;
//	uint8_t intRegSize	=	0x02;

	/// Read two bytes of interrupts from Register 03h & 04h from SPI.
	status = read_Reg_From_RF_SPI(&interruptStatusRegAddr, txRxReg, 2);
//	if(status != HAL_OK) return status;

	/// Pack both interrupt register into single 16 bit register.
	interruptStatus	=	(uint16_t)(txRxReg[1] << 8) | (uint16_t)txRxReg[0];
	*interrupts				=	interruptStatus;
//	return HAL_OK;
	return status;
}

///-5-
/// Function to Read Interrupt Status.
HAL_StatusTypeDef readAndClearInterrupts()
{
	/*
		Function to Read Interrupts Status
		-	This function gets triggered when "nIRQ" pin transitions from HIGH to LOW.
	*/
	/// Local Variables.
	uint8_t interruptStatus1RegAddr	=	0x03; /* 0x03: Interrupt Status 1 Register Address */
//	uint8_t intRegSize	=	0x02;

	/// Read two bytes of interrupts from Register 03h & 04h from SPI.
	status = read_Reg_From_RF_SPI(&interruptStatus1RegAddr, txRxReg, 2);
//	if(status != HAL_OK) return status;

	/// Pack both interrupt register into single 16 bit register.
	interruptStatus	=	(uint16_t)(txRxReg[1] << 8) | (uint16_t)txRxReg[0];
//	*interrupts				=	interruptStatus;
//	return HAL_OK;
	return status;
}

///-6-
/// Function to Get RSSI Value of the Received Signals.
HAL_StatusTypeDef setInterrupts(uint16_t setInterruptValue)
{
	/*
		Function to Get RSSI Value.
		-	setInterruptValue:		Values to .
	*/
	/// Setting Interrupt Registers.
	/// //	if((setInterruptValue >= 0) && (setInterruptValue < 8))
	interruptEnable1Reg	|=	(uint8_t)setInterruptValue;
//	if((setInterruptValue >= 8) && (setInterruptValue < 16))
	interruptEnable2Reg	|=	(uint8_t)(setInterruptValue >> 8);

	/// Populate values into an array.
	txRxReg[0]	=	interruptEnable1Reg;
	txRxReg[1]	=	interruptEnable2Reg;

	/// Update RF Module Registers.
	status	=	write_Reg_To_RF_SPI(&intrEn1RegAddr, txRxReg, 2); /* 2 is Must. */
	return status;
}

///-6-
/// Function to Clear Interrupts.
/// Enabled Interrupts are gets cleared when read. And Disabled Interrupts will retain theirs state.
//HAL_StatusTypeDef clearInterrupts()
//{
//	/*
//		Function to Reset All Interrupts.
//		-
//	*/
//	/// Local Variables.
//	uint8_t interruptStatusRegAddr	=	0x03;
//	/// Reset All Interrupt Bits to Zero.
//	uint8_t interruptStatus1Reg		=	0;
//	uint8_t interruptStatus2Reg		=	0x0;
//
//	/// Clear Internal Stored Interrupt Status.
//	interruptStatus			=	0;
//
//	/// Populate values into an array.
//	txRxReg[0]	=	interruptStatus1Reg;
//	txRxReg[1]	=	interruptStatus2Reg;
//
//	/// Update RF Module Registers.
//	status	=	write_Reg_To_RF_SPI(&interruptStatusRegAddr, txRxReg, 2); /* 2 is Must. */
//	return status;
//}

///-7-
/// Function to Get RSSI Value of the Received Signals.
HAL_StatusTypeDef resetInterrupts()
{
	/*
		Function to Reset All Interrupts.
		-
	*/
	/// Reset All Interrupt Bits to Zero.
	interruptEnable1Reg	=	0;
	interruptEnable2Reg	=	0x03;

	/// Populate values into an array.
	txRxReg[0]	=	interruptEnable1Reg;
	txRxReg[1]	=	interruptEnable2Reg;

	/// Update RF Module Registers.
	status	=	write_Reg_To_RF_SPI(&intrEn1RegAddr, txRxReg, 2); /* 2 is Must. */
	return status;
}


///-8-
/// Function to Shut down the whole Module.
HAL_StatusTypeDef shutdown(_Bool sdnEn)
{
	/*
	 	 Function to Shutdown the Module using SDN pin.
	 	 -	sdnEn:	Shutdown Enable | Disable (SET | RESET).
	 	 **Note:	"RF_SDN" Must be defined (same name) in the main program to use this function.
					Otherwise it will return "HAL_ERROR".
	*/

	/// The statement will only work when SDN pin is declared.
	if(sdnPin != 0)
	{
		if(sdnEn)
			HAL_GPIO_WritePin(sdnPort,	sdnPin,	GPIO_PIN_SET);		// Set (HIGH) SDN pin to power down the module work.
		else
			HAL_GPIO_WritePin(sdnPort,	sdnPin,	GPIO_PIN_RESET);	// Reset (LOW) SDN pin to make the module work.
		/// In normal case, when SDN pin is predefined this function will return OK.
		return HAL_OK;
	}
	/// When the SDN pin is not defined this function will return error.
	else return HAL_ERROR;
}

///-9-
/// Function to Select Operating Mode.
HAL_StatusTypeDef setOperatingMode(uint8_t mode) /* opMode_t */
{
	/*
		Operating Modes
		- Idle State
		 -	Standby Mode
		 -	Sleep Mode
		 -	Sensor Mode	(Low Battery Detect | Temperature Sensor)
		 -	Idle Ready Mode (Default)
		 -	Tune Mode
		-	TX State
		-	RX State
		-
		** Shutdown can be achieved by using SDN Pin Only.
		*** Use "Register 07h. Operating Mode and Function Control 1" for Modes Selection.
	*/

//	uint8_t regAddr	=	0x07; // Register Address is 0x07.
	/// Mode Selection Process.
//	switch(mode)
//	{
//	case STANDBY:
//		operAndFuncCtrl1Reg		=	0x0;					// Clear Whole Register
//		break;
//	case IDLE_READY:
//		operAndFuncCtrl1Reg		=	0x01;					// XTalON
//		break;
//	case SLEEP:
//		operAndFuncCtrl1Reg		=	0x20;					// ENWT 0x40 & 0x20 Confusion b/w Data sheet and Application Note.
//		break;
//	case SENSOR:
//		operAndFuncCtrl1Reg		=	(0x40 | 0x01);			// ENLBD + XtalON
//		//operAndFuncCtrl1Reg		|=	0x01; 						// XtalON
//		break;
//	case TUNE:
//		operAndFuncCtrl1Reg		|=	0x02;					// PLLON
//		break;
//	case TX:
////		operAndFuncCtrl1Reg		&=	(!0x04);				// Clearing RXON
//		operAndFuncCtrl1Reg		=	(0x08 | 0x01 | 0x02);	// TXON + XtalON + PLLON
//		break;
//	case RX:
////		operAndFuncCtrl1Reg		&=	(!0x08);				// Clearing TXON
//		operAndFuncCtrl1Reg		=	(0x04 | 0x01 | 0x02);	// RXON + XtalON + PLLON
//		break;
//	default:
//		operAndFuncCtrl1Reg		=	0x0;					// Clear Whole Register
//	}
	operAndFuncCtrl1Reg	=	mode;
	/// Write Register 07h to SPI.
	status = write_Reg_To_RF_SPI(&(opFuncCtrlReg1Addr), &(operAndFuncCtrl1Reg), sizeof(operAndFuncCtrl1Reg));
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-10-
/// Function to Set Frequency of the Module.
HAL_StatusTypeDef setFrequency(float frequency)//, float frequencyOffset, uint8_t channel, uint8_t stepSize)
{
	/*
		Function to set Frequency of the RF Module
		-	Frequency(in MHz): 240.00 MHz to 960.00 MHz
	*/

	/// Local Variables.
	uint8_t freqOffsetRegAddr			=	0x73;
//	uint8_t freqBandRegAddr			=	0x75;
//	uint8_t freqHoppingChannelRegAddr	=	0x79;
//	uint8_t freqHoppngStepSizeRegAddr	=	0x7A;

	//uint16_t	freqOffsetReg			=	0;
	uint8_t		hbsel 					=	0;
	uint8_t		frequencyBand_Fb 		=	0;
	uint16_t	carrierFrequency_Fc 	=	0;

	/// Determine High Band or Low Band and program HBSEL Register.
	if		((frequency >= 240.00) && (frequency <= 479.9))		hbsel = 0;	// LOW Band
	else if	((frequency >= 480.00) && (frequency <= 960.00))	hbsel = 1;	// HIGH Band
	else return HAL_ERROR;

	/// Determine Integer Part(N) and set FB[4:0].
	if(hbsel)
	{
		frequencyBand_Fb =	(uint8_t)((float)(frequency - 480) / 20) & 0x1F;	// HIGH Band
		frequencyBand_Fb &=	~0x60;	// Clearing HBSEL.
		frequencyBand_Fb |=	0x60;	// Setting HBSEL & SBSEL.
	}
	else
	{
		frequencyBand_Fb =	(uint8_t)((float)(frequency - 240) / 10) & 0x1F;	// LOW Band
		frequencyBand_Fb &=	~0x60;	// Clearing HBSEL.
		frequencyBand_Fb |=	0x40;	// Setting SBSEL.
	}

	/// Determine Fractional Part(F) and set FC[15:0].
	carrierFrequency_Fc = (uint16_t)((float)(((frequency) / (10*(hbsel+1))) - (frequencyBand_Fb & 0x1F) - 24) * 64000);

	/// Determine Frequency Offset.
//	if(frequencyOffset < 0)
//	{
//		freqOffsetReg   =   uint16_t((-frequencyOffset)/((hbsel+1)*156.25));
//		freqOffsetReg   =   ~freqOffsetReg + 1;
//	   // freqOffsetReg   +=  1;
//	}
//	else
//		freqOffsetReg   =   	uint16_t((frequencyOffset)/((1+1)*156.25));

	/// Packing Data(to be written) into single array.
	//txRxReg[0]	=	uint8_t(freqOffsetReg);					/** REG: 0x73 **/
	//txRxReg[1]	=	uint8_t(freqOffsetReg>>8);				/** REG: 0x74 **/
	txRxReg[0]	=	frequencyBand_Fb;						/** REG: 0x75 **/
	txRxReg[1]	=	(uint8_t)(carrierFrequency_Fc >> 8);		/** REG: 0x76 **/
	txRxReg[2]	=	(uint8_t)(carrierFrequency_Fc);			/** REG: 0x77 **/
	//txRxReg[5]	=	0;										/** REG: 0x78 (RESERVED) **/
	//txRxReg[6]	=	channel;								/** REG: 0x79 **/
	//txRxReg[7]	=	stepSize;								/** REG: 0x7A **/
//	uint8_t txBuffer[3] = {frequencyBand_Fb, ((uint8_t)(carrierFrequency_Fc >> 8)), ((uint8_t)carrierFrequency_Fc)};

	/// Write Frequency Registers (75 -> 77) to SPI.
	status = write_Reg_To_RF_SPI(&freqOffsetRegAddr, txRxReg, 3);
//	if(status != HAL_OK)
	return status;

//	/// Write Frequency Hopping Settings Registers (79 & 7A).
//	status	=	write_Reg_To_RF_SPI(&freqHoppingChannelRegAddr, &channel, 1);
//	if(status != HAL_OK) return status;
//	status	=	write_Reg_To_RF_SPI(&freqHoppngStepSizeRegAddr, &stepSize, 1);
//	return status;

	/// Set "Crystal Oscillator Load Capacitance" Register to 0 if external 30 MHz oscillator is being used.
//	return HAL_OK;
}

///-10-
/// Function to Set Frequency of the Module.
HAL_StatusTypeDef setFrequency_(float frequency, float frequencyOffset, uint8_t channel, uint8_t stepSize)
{
	/*
		Function to set Frequency of the RF Module
		-	Frequency(in MHz): 240.00 MHz to 960.00 MHz
	*/

	/// Local Variables.
	uint8_t freqOffsetRegAddr			=	0x73;
//	uint8_t freqBandRegAddr			=	0x75;
//	uint8_t freqHoppingChannelRegAddr	=	0x79;
//	uint8_t freqHoppngStepSizeRegAddr	=	0x7A;

	uint16_t	freqOffsetReg			=	0;
	uint8_t		hbsel 					=	0;
	uint8_t		frequencyBand_Fb 		=	0;
	uint16_t	carrierFrequency_Fc 	=	0;

	/// Determine High Band or Low Band and program HBSEL Register.
	if		((frequency >= 240.00) && (frequency <= 479.9))		hbsel = 0;	// LOW Band
	else if	((frequency >= 480.00) && (frequency <= 960.00))	hbsel = 1;	// HIGH Band
	else return HAL_ERROR;

	/// Determine Integer Part(N) and set FB[4:0].
	if(hbsel)
	{
		frequencyBand_Fb =	(uint8_t)((float)(frequency - 480) / 20) & 0x1F;	// HIGH Band
		frequencyBand_Fb &=	~0x60;	// Clearing HBSEL.
		frequencyBand_Fb |=	0x60;	// Setting HBSEL & SBSEL.
	}
	else
	{
		frequencyBand_Fb =	(uint8_t)((float)(frequency - 240) / 10) & 0x1F;	// LOW Band
		frequencyBand_Fb &=	~0x60;	// Clearing HBSEL.
		frequencyBand_Fb |=	0x40;	// Setting SBSEL.
	}

	/// Determine Fractional Part(F) and set FC[15:0].
	carrierFrequency_Fc = (uint16_t)((float)(((frequency) / (10*(hbsel+1))) - (frequencyBand_Fb & 0x1F) - 24) * 64000);

	/// Determine Frequency Offset.
	if(frequencyOffset < 0)
	{
		freqOffsetReg   =   (uint16_t)((-frequencyOffset)/((hbsel+1)*156.25));
		freqOffsetReg   =   ~freqOffsetReg + 1;
	   // freqOffsetReg   +=  1;
	}
	else
		freqOffsetReg   =   	(uint16_t)((frequencyOffset)/((1+1)*156.25));

	/// Packing Data(to be written) into single array.
	txRxReg[0]	=	(uint8_t)(freqOffsetReg);					/** REG: 0x73 **/
	txRxReg[1]	=	(uint8_t)(freqOffsetReg>>8);				/** REG: 0x74 **/
	txRxReg[2]	=	frequencyBand_Fb;						/** REG: 0x75 **/
	txRxReg[3]	=	(uint8_t)(carrierFrequency_Fc >> 8);		/** REG: 0x76 **/
	txRxReg[4]	=	(uint8_t)(carrierFrequency_Fc);			/** REG: 0x77 **/
	txRxReg[5]	=	0;										/** REG: 0x78 (RESERVED) **/
	txRxReg[6]	=	channel;								/** REG: 0x79 **/
	txRxReg[7]	=	stepSize;								/** REG: 0x7A **/
//	uint8_t txBuffer[3] = {frequencyBand_Fb, ((uint8_t)(carrierFrequency_Fc >> 8)), ((uint8_t)carrierFrequency_Fc)};

	/// Write Frequency Registers (75 -> 77) to SPI.
	status = write_Reg_To_RF_SPI(&freqOffsetRegAddr, txRxReg, 8);
//	if(status != HAL_OK)
	return status;

//	/// Write Frequency Hopping Settings Registers (79 & 7A).
//	status	=	write_Reg_To_RF_SPI(&freqHoppingChannelRegAddr, &channel, 1);
//	if(status != HAL_OK) return status;
//	status	=	write_Reg_To_RF_SPI(&freqHoppngStepSizeRegAddr, &stepSize, 1);
//	return status;

	/// Set "Crystal Oscillator Load Capacitance" Register to 0 if external 30 MHz oscillator is being used.
//	return HAL_OK;
}

///-11-
/// Function to Set Frequency Hopping Channel and Hopping Step Size to RF Module.
HAL_StatusTypeDef setFrequencyHoppingChannel(uint8_t channel, uint8_t stepSize)
{
	/*
		Function to set Frequency Hopping Channel of the RF Module.
		-	channel (Multiples of StepSize)	:	0 -> 255	 => ( 0 -> 2.56MHz if stepSize is 1 ).
		-	stepSize (Multiples of 10kHz)	:	0 -> 255	 =>	( 0 -> 2.56MHz)
	*/
	/// Local Variables.
	uint8_t freqHoppingChannelRegAddr	=	0x79;
	uint8_t freqHoppngStepSizeRegAddr	=	0x7A;

	/// Write Frequency Hopping Settings Registers (79 & 7A).
	status	=	write_Reg_To_RF_SPI(&freqHoppingChannelRegAddr, &channel, 1);
	if(status != HAL_OK) return status;
	status	=	write_Reg_To_RF_SPI(&freqHoppngStepSizeRegAddr, &stepSize, 1);
	return status;
}

///-12-
/// Function to Read Frequency Settings from the RF Module.
HAL_StatusTypeDef readFrequencySettings(float* frequency, float* frequencyOffsetValue)
{

	/// Local Variables.
//	uint8_t freqHoppingChannelRegAddr	=	0x79;
//	uint8_t freqHoppngStepSizeRegAddr	=	0x7A;
//	uint8_t freqHoppingChannelReg		=	0;
//	uint8_t freqHoppngStepSizeReg		=	0;
//
//	uint8_t freqBandRegAddr				=	0x75;

	uint8_t	freqOffsetRegAddr			=	0x73;
	uint8_t	freqOffsetReg				=	0;
//	float frequencyOffsetValue			=	0;

	/// Read Frequency Hopping Channel Registers Settings.
//	status	=	read_Reg_From_RF_SPI(&freqHoppingChannelRegAddr, &freqHoppingChannelReg, 1);
//	if(status != HAL_OK) return status;
//	status	=	read_Reg_From_RF_SPI(&freqHoppngStepSizeRegAddr, &freqHoppngStepSizeReg, 1);
//	if(status != HAL_OK) return status;
//
//	/// Read Carrier Frequency Registers Settings.
//	status	=	read_Reg_From_RF_SPI(&freqBandRegAddr, txRxReg, 3);
//	if(status != HAL_OK) return status;

	status	=	read_Reg_From_RF_SPI(&freqOffsetRegAddr, txRxReg, 8);
	if(status != HAL_OK) return status;

	freqOffsetReg				=	(txRxReg[1]<<8) | txRxReg[0];
	/// Determine Frequency Offset.
	if(freqOffsetReg > 0x200)
	{
    	freqOffsetReg   		=   ((~freqOffsetReg) + 1) & 0x3FF;
    	*frequencyOffsetValue	=   -((float)freqOffsetReg * (float)((_Bool)(txRxReg[2] & 0x20)+1)*156.25);	/** txRxReg[2] contains HBSEL **/
	}
	else
		*frequencyOffsetValue   	=   (float)(freqOffsetReg) * (float)((_Bool)(txRxReg[2] & 0x20)+1)*156.25;	/** txRxReg[2] contains HBSEL **/

	/** Frequency =	FHopping + FtX **/
	*frequency	=	/* float(frequencyOffsetValue/1'000'000.0) */
					/** FHopping = FHS[7:0] x FHCH[7:0] x 10kHz **/
					(txRxReg[6] * txRxReg[7] * 10000.0)/100000.0
					+	/** FtX	=	10MHz*(HBSEL+1)*(FB[4:0]+24+FC[15:0]/64000) **/
					(10.0 * ((_Bool)(txRxReg[2] & 0x20) + 1) * ((txRxReg[2] & 0x1F) + 24.0 + ((txRxReg[3]<<8) | txRxReg[4])/64000.0));
//					(10.0 * ((_Bool)(txRxReg[0] & 0x20) + 1) * ((txRxReg[0] & 0x1F) + 24.0 + ((txRxReg[1]<<8) | txRxReg[2])/64'000.0));
//					10'000'000 * ((_Bool)(txRxReg[0] & 0x20) + 1) * ((txRxReg[0] & 0x1F) + 24 + (uint16_t)((txRxReg[1]<<8) | txRxReg[2])/64'000);
	return status;
}

///-13-
/// Function to Set Transmission Data Rate.
HAL_StatusTypeDef setTxDataRate(uint32_t txDataRate)
{
	/*
		Function to Set TX Data Rate
		-	Data Rate (bits per seconds) 0.123 -> 256k
		**Note: In case of MANCHESTER Encoding Data rate is Limited to 128k.
	*/
	/// Local Variables.
	uint8_t	txDataRateRegAddr			=	0x6E;

	_Bool 			txDataRateScale 	=	RESET;
	float			txDataRateInput		=	0;
	uint16_t		txDataRateValue		=	0;
//	uint8_t			txDataRateReg[3]	=	{0};
	///	Limiting Data Rate within the allowable range.
	if		(txDataRate < 125)		txDataRateInput =	125;
	else if	(txDataRate > 256000)	txDataRateInput =	256000;
	else							txDataRateInput	=	txDataRate;

	///	Scaling Bit set/reset.
	if(txDataRate < 30000)		txDataRateScale	=	SET;
	else						txDataRateScale	=	RESET;

	///	TX Data Rate Register calculation.
//	txDataRateValue	=	(uint16_t)((float)((txDataRateInput * power(2,(16+5*txDataRateScale)))) / power(10,6));
	txDataRateValue	=	(uint16_t)((float)((txDataRateInput * (txDataRateScale ? 2097152 : 65536))) / 1000000);
	txDataRateInput	=	((float)((txDataRateInput * (txDataRateScale ? 2097152 : 65536))) / 1000000);
	txDataRateValue +=	((txDataRateInput - txDataRateValue) >= 0.5);

	/// TX Data Rate Scaling Set / Reset.
	if(txDataRateScale) moduloModeCtrl1Reg	|=	0x20;	else moduloModeCtrl1Reg	&=	~0x20;

	/// Packing Data(to be written) into single array.
	txRxReg[0] = ((uint8_t)(txDataRateValue >> 8));
	txRxReg[1] = ((uint8_t)txDataRateValue);
	txRxReg[2] = ((uint8_t)(moduloModeCtrl1Reg));

	/// Write Data Rate Register (6Eh, 6Fh, & 70h) to SPI.
	status = write_Reg_To_RF_SPI(&txDataRateRegAddr, txRxReg, 3);
//	if(status != HAL_OK)
//	return status;

	/** -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- Copied Part -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- **/

	const uint16_t IFFilterTable[][2] = { 	{ 322, 0x26 },	{ 3355, 0x88 },	{ 3618, 0x89 },	{ 4202, 0x8A },
											{ 4684, 0x8B },	{ 5188, 0x8C },	{ 5770, 0x8D },	{ 6207, 0x8E }
										};
	//now set the timings
		uint16_t kbps		=	txDataRate / 1000;
		uint16_t freqDev	=	150;
		uint16_t minBandwidth = (2 * freqDev) + kbps;

		uint8_t IFValue = 0xFF;
		//since the table is ordered (from low to high), just find the 'minimum bandwidth which is greater than required'
		for (uint8_t i = 0; i < 8; ++i) {
			if (IFFilterTable[i][0] >= (minBandwidth * 10)) {
				IFValue = IFFilterTable[i][1];
				break;
			}
		}

		uint8_t REG_IF_FILTER_BW_Addr	=	0x1C;
		write_Reg_To_RF_SPI(&REG_IF_FILTER_BW_Addr, &IFValue, 1);

		uint8_t dwn3_bypass = (IFValue & 0x80) ? 1 : 0; // if msb is set
		uint8_t ndec_exp = (IFValue >> 4) & 0x07; // only 3 bits

		uint16_t rxOversampling = ((500.0 * (1 + 2 * dwn3_bypass)) / ((power(2, ndec_exp - 3)) * (double ) kbps));

		float ncOffsetVal 	= 	(((double) kbps * (power(2, ndec_exp + 20))) / (500.0 * (1 + 2 * dwn3_bypass)));
		int32_t ncOffset	=	(uint32_t)ncOffsetVal;
		ncOffset +=	((ncOffsetVal - ncOffset) >= 0.5);

		uint16_t crGain = 2 + ((65535 * (uint64_t) kbps) / ((uint64_t) rxOversampling * freqDev));
		uint8_t crMultiplier = 0x00;
		if (crGain > 0x7FF) {
			crGain = 0x7FF;
		}

		uint8_t timingVals[] = { 	(uint8_t)(rxOversampling & 0x00FF),
									(uint8_t)(((rxOversampling & 0x0700) >> 3) | ((ncOffset >> 16) & 0x0F)),
									(uint8_t)((ncOffset >> 8) & 0xFF),
									(uint8_t)(ncOffset & 0xFF),
									(uint8_t) (((crGain & 0x0700) >> 8) | crMultiplier),
									(uint8_t)(crGain & 0xFF)
								};

		uint8_t REG_CLOCK_RECOVERY_OVERSAMPLING	=	0x20;
		write_Reg_To_RF_SPI(&REG_CLOCK_RECOVERY_OVERSAMPLING, timingVals, 6);

	/** -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- Copied Part -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- **/

	return status;

//	return HAL_OK;
}

///-14-
/// Function to Set Modulation Type.
HAL_StatusTypeDef setModulationType(uint8_t modulation)//, uint16_t freqDeviation)
{
	/*
		Function to Set Modulation Type
		-	Modulation Type (OOK, FSK, GFSK)
	*/
	/// Local Variables.
	uint8_t MODULATION_TYPE_Mask	=	0x03;
	uint8_t MODULATION_SOURCE_Mask	=	0x30;

	/// Setting modulation type.
	moduloModeCtrl2Reg &=	~MODULATION_TYPE_Mask;
	moduloModeCtrl2Reg |=	modulation;

	/// Setting Modulation Source to FIFO Mode.
	moduloModeCtrl2Reg &=	~MODULATION_SOURCE_Mask;
	moduloModeCtrl2Reg |=	0x20;

	/// Write Modulation Register (71H) to SPI.
	status = write_Reg_To_RF_SPI(&moduloModeCtrl2RegAddr, &moduloModeCtrl2Reg, sizeof(moduloModeCtrl2Reg));
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}


///-15-
/// Function to Set Modulation Type.
HAL_StatusTypeDef setFrequencyParametersSettings(uint8_t modulation, uint32_t freqDeviation)//, uint16_t freqDeviation)
{
	/*
		Function to Set Modulation Type
		-	modulation:		Modulation Type (OOK, FSK, GFSK).
		-	freqDeviation:	Frequency Deviation (625 Hz -> 320'000 Hz).
	*/
	/// Local Variables.
	uint8_t MODULATION_TYPE_Mask	=	0x03;
	uint8_t MODULATION_SOURCE_Mask	=	0x30;

//	uint8_t frequencyDeviationRegAddr	=	0x72;

	/// Setting modulation type.
	moduloModeCtrl2Reg &=	~MODULATION_TYPE_Mask;
	moduloModeCtrl2Reg |=	modulation;

	/// Setting Modulation Source to FIFO Mode.
	moduloModeCtrl2Reg &=	~MODULATION_SOURCE_Mask;
	moduloModeCtrl2Reg |=	0x20;	/** Fixed to FIFO Mode **/

	if(freqDeviation < 625)		freqDeviation	=	625;
	if(freqDeviation > 320000)	freqDeviation	=	320000;

	float freqDev	=	(freqDeviation/625);
	freqDev			+=	((freqDev - (uint32_t)(freqDev)) >= 0.5);
	/// Assign Data to an Array.
	txRxReg[0]	=	moduloModeCtrl2Reg;
//	txRxReg[1]	=	uint8_t(freqDeviation/625 - 1);
	txRxReg[1]	=	(uint8_t)(freqDev);

	/// Write Modulation Register (71H) to SPI.
	status = write_Reg_To_RF_SPI(&moduloModeCtrl2RegAddr, txRxReg, 2);
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-16-
/// Function to Set RF Transmit Power Level.
HAL_StatusTypeDef setTxPower(uint8_t txPower)
{
	/*
		Function to Set Modulation Type
		-	TX Power Options for
			-	SI4432:			+1,	+2,	+5,	+8,	+11, +14, +17, +20.
	*/
	/// Local Variables.
	uint8_t txPowerAddr	=	0x6D;
	uint8_t txPowerReg	=	0;

	/// Choosing TX Power level.
	switch(txPower)
	{
	case 1:
		txPowerReg	=	0x0;
		break;
//	case 2:
//		txPowerReg	=	0x01;
//			break;
//	case 5:
//		txPowerReg	=	0x02;
//			break;
//	case 8:
//		txPowerReg	=	0x03;
//			break;
//	case 11:
//		txPowerReg	=	0x04;
//			break;
//	case 14:
//		txPowerReg	=	0x05;
//			break;
//	case 17:
//		txPowerReg	=	0x06;
//			break;
//	case 20:
//		txPowerReg	=	0x07;
//			break;
	default:
		txPowerReg	=	(uint8_t)(float)((txPower - 2) / 3 + 1);
		if(txPowerReg > 0x07)	txPowerReg = 0x07;
	}
	/// Enable LNA_SW and A unknown Reserved Bit.
	txPowerReg |= 0x18; /** 0x18 Must be Used **/

	/// Send txPowerReg Register 6Dh to SPI.
	status = write_Reg_To_RF_SPI(&txPowerAddr, &txPowerReg, sizeof(txPowerReg));
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-17-
/// Function set Encoding Type.
HAL_StatusTypeDef setEncoding(
		_Bool dataWhiteningEn,
		_Bool manchEncodingEn,
		_Bool manchInvEn,
		_Bool manchPreambPolarity)
{
	/*
		Function to Set Encoding.
		-	dataWhiteningEn:		Data Whitening Enable/Disable (SET/RESET).
		-	manchEncodingEn:		MANCHESTER Encoding Enable/Disable (SET/RESET).
		-	manchEncInvEn:			Inversion of MANCHESTER Encoding Enable/Disable (SET/RESET).
		-	manPreamblePolarity:	Pre-amble Polarity of MANCHESTER Encoding Set/Reset (SET/RESET).
 	*/
	/// Enabling / Disabling Encoding Operations.
	if(dataWhiteningEn) moduloModeCtrl1Reg		|=	0x01;	else moduloModeCtrl1Reg	&=	~0x01;

	if(manchEncodingEn) moduloModeCtrl1Reg		|=	0x02;	else moduloModeCtrl1Reg	&=	~0x02;

	if(manchInvEn) 		moduloModeCtrl1Reg		|=	0x04;	else moduloModeCtrl1Reg	&=	~0x04;

	if(manchPreambPolarity) moduloModeCtrl1Reg	|=	0x08;	else moduloModeCtrl1Reg	&=	~0x08;

	/// Write Encoding Set/Reset Data of Register 70h to SPI.
	status = write_Reg_To_RF_SPI(&moduloModeCtrl1RegAddr, &moduloModeCtrl1Reg, sizeof(moduloModeCtrl1Reg));
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-18-
/// Function to Set Data Access Control.
HAL_StatusTypeDef setDataAccessControl(
		_Bool crcEn, _Bool crcDataOnlyEn, uint8_t crcPolySlct,
		_Bool txPackHandleEn, _Bool rxPackHandleEn, _Bool lsbFirstEn, _Bool skip2ndPhPreambDet)
{
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
	/// Set / Reset Data Access Control Operations.
	if(crcEn)					dataAcsCtrlReg	|=	0x04;	else	dataAcsCtrlReg	&=	~0x04;
	if(crcDataOnlyEn)			dataAcsCtrlReg	|=	0x20;	else	dataAcsCtrlReg	&=	~0x20;
	if(txPackHandleEn)			dataAcsCtrlReg	|=	0x08;	else	dataAcsCtrlReg	&=	~0x08;
	if(rxPackHandleEn)			dataAcsCtrlReg	|=	0x80;	else	dataAcsCtrlReg	&=	~0x80;
	if(lsbFirstEn)				dataAcsCtrlReg	|=	0x40;	else	dataAcsCtrlReg	&=	~0x40;
	if(skip2ndPhPreambDet)		dataAcsCtrlReg	|=	0x10;	else	dataAcsCtrlReg	&=	~0x10;

	dataAcsCtrlReg	&=	~0x03;	dataAcsCtrlReg	|=	crcPolySlct;

	/// Write Data Access Control Set/Reset Data of Register 30h to SPI.
	status = write_Reg_To_RF_SPI(&dataAcsCtrlRegAddr, &dataAcsCtrlReg, sizeof(dataAcsCtrlReg));
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-19-
/// Function to Set Pre-amble bits and Pre-amble Threshold in Receiving Mode.
HAL_StatusTypeDef setPreamble(uint16_t preambLength, uint8_t preambThresh)
{
	/*
		Function to Set Pre-amble Length (Must be used on Transmitting End).
		-	preambLength:		Pre-amble Length (1 -> 511 Nibbles).
		-	preambThresh:		Pre-amble Detection Threshold (1 -> 15 Nibbles).
	*/
	/// Local Variables.
	uint8_t preambleAddr	=	0x33;
//	uint16_t lengthInNibbs	=	0;

	/// Set/Reset MSB.
	if(preambLength >= 256) headerCtrl2Reg |= 0x01;
	else headerCtrl2Reg &= ~0x01;

	/// Packing Data(to be written) into single array.
	txRxReg[0] = headerCtrl2Reg;
	txRxReg[1] = (uint8_t)(preambLength & 0xFF);

	if(preambThresh < 0x20) // 0x20 = 32 Nibbles, 32x4 = 128 bits.
	{
//		preambDetectionCtrlReg =	0x2A; // Reset Value.
		preambDetectionCtrlReg &= ~0xF8;
		preambDetectionCtrlReg |= (uint8_t)(preambThresh << 3);
	}
	else
	{
//		preambDetectionCtrlReg =	0x2A; // Reset Value.
		preambDetectionCtrlReg &= ~0xF8;
		preambDetectionCtrlReg |= 0xF8;
	}

	/// Set Pre-amble threshold.
	txRxReg[2]	=	preambDetectionCtrlReg;

	/// Write Pre-amble nibbles length Register (33h -> 35h) to SPI.
	status = write_Reg_To_RF_SPI(&preambleAddr, txRxReg, (size_t)0x03);
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-20-
/// Function to Set Synchronization Word.
HAL_StatusTypeDef setSyncWord(uint8_t* syncWord, size_t size)
{
	/*
		Function to Set Synchronization Word
		-	syncWord:	Synchronization Word (usually 1 -> 4 bytes)
		-	size:		Size of Synch Word.
	*/
	uint8_t syncWordRegAddr	=	0x36;

	/// Settings of Sync Word Length.
	headerCtrl2Reg	&= ~0x06;	headerCtrl2Reg	|=	((uint8_t)((size-0x01) << 1)) & 0x06;

	/// Write Setting of Sync Word Length for Register (33h) to SPI.
	status = write_Reg_To_RF_SPI(&headerCtrl2RegAddr, &headerCtrl2Reg, sizeof(headerCtrl2Reg));
	if(status != HAL_OK) return status;

	/// Write Sync Words for Register (36h -> 39h) to SPI.
	status = write_Reg_To_RF_SPI(&syncWordRegAddr, syncWord, size);
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-21-
/// Function to Set Transmit Header.
HAL_StatusTypeDef setTxHeader(uint8_t* txHeader, size_t size)
{
	/*
		Function to Set Transmit Header Word
		-	*txHeader:	Header Word (usually 1 -> 4 bytes)
		-	size:		Size of Synch Word.
	*/
	/// Local Variables.
	uint8_t txHeaderRegAddr = 0x3A;

	/// Assign Values to the Library Register in Descending order.
//	for(uint8_t i = 0 ; i < size; i++) txRxReg[size - i -1]  = txHeader[i];

	/// Write TX Headers for Register (3Ah -> 3Dh) to SPI.
//	status = write_Reg_To_RF_SPI(&txHeaderRegAddr, txRxReg, size);
	status = write_Reg_To_RF_SPI(&txHeaderRegAddr, txHeader, size);
	return status;
}

///-22-
/// Function to Get Received Header.
HAL_StatusTypeDef setRxCheckHeader(uint8_t* rxCheckHeader, size_t size)
{
	/*
		Function to set Received Header Check Word
		-	*rxHeader:	Receive Header Check Word (usually 1 -> 4 bytes)
		-	size:		Size of Synch Word.
	*/

	/// Set header length
	uint8_t rxHeaderCheckAddr	=	0x3F; /* 0x3F */
	/// Lookup table for Header Check HDCH[3:0].
//	uint8_t lutHDCH[]	=	{ 0x01, 0x03, 0x07, 0x0F};	// One hot encoding for HDCH.
//	uint8_t lutBCEN[]	=	{ 0x10, 0x30, 0x70, 0xF0};	// One hot encoding for BCEN.
	uint8_t lutHDCH[]	=	{ 0x08, 0x0C, 0x0E, 0x0F};	// One hot encoding for HDCH.
///	uint8_t lutBCEN[]	=	{ 0x80, 0xC0, 0xE0, 0xF0};	// One hot encoding for BCEN.

	/// Enable Broadcast Checker. BCEN[3:0].
	/*
	 * Broadcast Address (FFh) Check Enable.
	 * If it is enabled together with Header Byte Check then the header check is OK if the
	 * incoming header byte equals with the appropriate check byte or FFh). One hot encoding.
	 	 * 0000: No broadcast address enable.
	 	 * 0001: Broadcast address enable for header byte 0.
	 	 * 0010: Broadcast address enable for header byte 1.
	 	 * 0011: Broadcast address enable for header bytes 0 & 1.
	 	 * 0100: …
	 */
///	headerCtrl1Reg	&=	~0xF0; // 0xF0 is Masking for BCEN[3:0].
///	headerCtrl1Reg	|=	(uint8_t)(lutBCEN[(uint8_t)size-1] & 0xF0); // 0xF0 is Masking for BCEN[3:0].

	/// Enable Header bytes to be checked. HDCH[3:0].
	/* Received Header Bytes to be Checked Against the Check Header Bytes.
 	 * One hot encoding. The receiver will use HDCH[2:0] to know the position of the Header Bytes.
 	 * Set Broadcast header bytes. BCEN[3:0].
 	 	 * 0000: No Received Header check.
 	 	 * 0001: Received Header check for byte 0.
 	 	 * 0010: Received Header check for bytes 1.
 	 	 * 0011: Received header check for bytes 0 & 1.
 	 	 * 0100: …
 	 */
	headerCtrl1Reg	&=	~0x0F; // Clearing all BCEN[3:0] bits, 0xF0 is Masking for BCEN[3:0].
	headerCtrl1Reg	|=	(uint8_t)(lutHDCH[(uint8_t)size-1] & 0x0F); // 0x0F is Masking for HDCH[3:0].

	/// Enable Header Length. HDLEN[2:0].
	/* Header Length.
	 * Transmit/Receive Header Length. Length of header used if packet handler is enabled for
	 * TX/RX (ENPACTX/RX). Headers are transmitted/received in descending order.
	 	 * 000: No TX/RX header
	 	 * 001: Header 3
	 	 * 010: Header 3 and 2
	 	 * 011: Header 3 and 2 and 1
	 	 * 100: Header 3 and 2 and 1 and 0
	 */
	headerCtrl2Reg = (uint8_t)(((uint8_t)size<<4) & 0x70); // 0x70 is Masking for HDLEN[2:0].

	/// Write Header Control 1 Register (32h) data to SPI.
	status	=	write_Reg_To_RF_SPI(&headerCtrl1RegAddr, &headerCtrl1Reg, (size_t)1);
	if(status != HAL_OK) return status;

	/// Write Header Control 2 Register (33h) data to SPI.
	status	=	write_Reg_To_RF_SPI(&headerCtrl2RegAddr, &headerCtrl2Reg, (size_t)1);
	if(status != HAL_OK) return status;

	/// Write Expected Header values Register (3Fh -> 42h) to SPI. CHHD[31:0].
	status	=	write_Reg_To_RF_SPI(&rxHeaderCheckAddr, rxCheckHeader, size);
//	if(status != HAL_OK)
	return status;

//	return HAL_OK;
}

///-23-
/// Function to Get Received Header.
HAL_StatusTypeDef getRxHeader(uint8_t* receivedHeader, size_t size)
{
	/*
		Function to get Received Header Word
		-	*rxHeader:	Synchronization Word (usually 1 -> 4 bytes)
		-	size:		Size of Synch Word.
	*/
	/// Variables.
	uint8_t receivedHeaderRegAddr	=	0x47;

	/// Read Received Header Register (47h -> 4Ah) from SPI.
	status	=	read_Reg_From_RF_SPI(&receivedHeaderRegAddr, receivedHeader, size);
//	if(status != HAL_OK)
	return status;
//	return HAL_OK;
}

///-24-
/// Function to Set Low Battery Level and Interrupt on IRQ pin.
HAL_StatusTypeDef setLowBatteryLevel(uint8_t lowBattThresh, _Bool interruptEnable)
{
	/*
		 Function to Enable/Disable Low Battery Mode
		 -	lowBattThresh:	Input Low Battery Threshold Level (0% -> 100%) Means (1.7V -> 3.2599V) Volts.
		 -	interruptEn:	Input Interrupt Enable (SET/RESET).
		 ** Note: To use this function Enable Sensor Mode using operatingMode Function.
	*/
	/// Local Variables.
	uint8_t lowBattThreshAddr		=	0x1A;
	uint8_t lowBatteryThresholdReg	=	0;

	/// Limiting Battery Threshold.
	if((lowBattThresh < 0) && (lowBattThresh > 100)) return HAL_ERROR;
//	if((*lowBattThresh < 1.7) && (*lowBattThresh > 3.25)) return HAL_ERROR;

	/// Conversion of voltage to bits.
	lowBatteryThresholdReg = (uint8_t)((float)(lowBattThresh / 100 * 32));

	/// Low battery Interrupt Enable.
	if(interruptEnable)
		interruptEnable2Reg |= 0x04;
	else
		interruptEnable2Reg &= ~0x04;

	/// Write threshold to Register 1Ah to SPI.
	status = write_Reg_To_RF_SPI(&lowBattThreshAddr, &lowBatteryThresholdReg, sizeof(lowBatteryThresholdReg));
	if(status != HAL_OK) return status;

	/// Write Interrupt Enable to Register 06h to SPI.
	status = write_Reg_To_RF_SPI(&intrEn2RegAddr, &interruptEnable2Reg, sizeof(interruptEnable2Reg));
//	if(status != HAL_OK)
		return status;
	//** Use "Register 07h. Operating Mode and Function Control 1" for Modes Selection.
	//		See "8.5. Low Battery Detector" on page 55 for more information on these features.
//	return HAL_OK;
}

///-25-
/// Function to Read Battery Voltage Level.
HAL_StatusTypeDef getBatteryLevel(uint8_t* batteryVoltLevel)
{
	/*
		 Function to Enable/Disable Low Battery Mode
		 -	batteryVoltLevel:	Output Battery Voltage Level (1.7 -> 3.25) Volts.
		  ** Note: To use this function Enable Sensor Mode using operatingMode Function.
	*/
	/// Local Variables.
	uint8_t battVoltLevelRegAddr	=	0x1B;
	uint8_t battVoltLevelReg		=	0;

	/// Read Register 1Bh from SPI.
	status = read_Reg_From_RF_SPI(&battVoltLevelRegAddr, &battVoltLevelReg, sizeof(battVoltLevelReg));
//	if(status != HAL_OK) return status;

	/// Level Conversion from bits to volts.
//	*batteryVoltLevel = (float)(1.7 + (float)(0.05 * battVoltLevelReg));
//	*batteryVoltLevel = battVoltLevelReg;
	if(battVoltLevelReg >= (uint8_t)(4)) {	*batteryVoltLevel = (uint8_t)((float)(battVoltLevelReg*100/32));	}
	else {	*batteryVoltLevel = 0;	}

//	return HAL_OK;
	return status;
}

///-26-
/// Function to Set Wake-Up-Timer.
HAL_StatusTypeDef setWakeupTimer(uint8_t* time)
{
	/*
		Function to Set Encoding.
		-	*time:		Wake-up Time after (in Milliseconds).
	*/

	return HAL_OK;
}

///-27-
/// Function to Set Low Duty Cycle Mode.
HAL_StatusTypeDef setLowDutyCycleMode(uint8_t* dutyCycle)
{
	/*
		Function to Set Encoding.
		-	*dutyCycle:		Low Power On Time Duty (in Milliseconds).
	*/

	return HAL_OK;
}

///-28-
/// Function to Set RSSI Threshold for the Low Level Received Signals.
HAL_StatusTypeDef setRssiThresh(const uint8_t rssiThresh)
{
	/*
		Function to Set Minimum RSSI Threshold.
		-	*rssiThresh:		RSSI Threshold for indication of Low Signals Level (in dB).
		**Note:	Check the status of iRSSI in Interrupt/Status 2 Register.
	*/
	/// Local Variables.
	uint8_t rssiThreshRegAddr	=	0x27;
	uint8_t rssiThreshReg		=	0;

	/// dB to Binary Conversion.
	rssiThreshReg	=	(uint8_t)((float)(rssiThresh / 0.5));

	/// Write Pre-amble nibbles length Register (33 & 34H) to SPI.
	status = write_Reg_To_RF_SPI(&rssiThreshRegAddr, &rssiThreshReg, sizeof(rssiThreshReg));
	return status;
}

///-29-
/// Function to Set RSSI Offset in the Received Signals Strength.
HAL_StatusTypeDef setRssiOffset(const uint8_t rssiOffset)
{
	/*
		Function to Set Minimum RSSI Threshold.
		-	rssiOffset:		Offset value to RSSI Calculation (1 -> 7) => (+4dB -> 28dB).
		**Note:	Check the status of iRSSI in Interrupt/Status 2 Register.
	*/
	/// Local Variables.
	uint8_t rssiOffsetRegAddr		=	0x27;

	/// Update Memory Register.
	preambDetectionCtrlReg	&=	~0x07;
	preambDetectionCtrlReg	|=	(uint8_t)(rssiOffset & 0x07);

	/// Write Pre-amble nibbles length Register (33 & 34H) to SPI.
	status = write_Reg_To_RF_SPI(&rssiOffsetRegAddr, &preambDetectionCtrlReg, sizeof(preambDetectionCtrlReg));
	return status;
}
///-29-
/// Function to Get RSSI Value of the Received Signals.
HAL_StatusTypeDef getRSSI(uint8_t* rssiValue)
{
	/*
		Function to Get RSSI Value.
		-	*rssiValue:		RSSI Threshold for indication of Low Signals Level (in dB).
	*/
	/// Local Variables.
	uint8_t rssiValueRegAddr	=	0x26;
	uint8_t	rssiValueReg		=	0;

	/// Read RSSI Register (26h) to SPI.
	status = read_Reg_From_RF_SPI(&rssiValueRegAddr, &rssiValueReg, sizeof(rssiValueReg));

	/// Convert Binary to dB.
	*rssiValue	=	(uint8_t)((float)(rssiValueReg * 0.5));

	return status;
}



///-32-
///
_Bool sendDataPacket_getResponse(uint8_t* data, uint8_t size, _Bool waitForAckResponse, uint32_t responseTimeout, uint8_t* responseBuffer, uint8_t* responseSize)
{
	/*
		Function to Send Data from the RF Module.
		-	*data:		Data to Write on the Transmit Registers.
		-	size:		Size of the TX Data.
	*/
	/// Clear Old TX Data.
	clearTxFIFO();

	/// Write New Data to the FIFO and Its Length.
	status	=	writeDataToRF(data, size);

	/// Reset Previous Enabled Interrupts
	setInterrupts(INT_TX_PACKET_SENT);

	/// Read interrupts to Remove previous status.
	status	=	readAndClearInterrupts();

	/// Change Mode to TX Mode.
	status	=	setOperatingMode(IDLE_READY | TX);

	/// Local Variables.
	uint32_t entryTimeTx	=	HAL_GetTick();
	uint32_t timeStampTx	=	entryTimeTx;
//	timeStampTx				=	entryTimeTx;

	/// Check for Timeout.
	while((timeStampTx - entryTimeTx) <= MAX_TRANSMIT_TIMEOUT)
	{
		//HAL_Delay(0);

		/// Get Time.
		timeStampTx	=	HAL_GetTick();

		/// In case of timer overflow.
		if(entryTimeTx >  timeStampTx) entryTimeTx	=	timeStampTx;

		/// Check for Interrupt Pin.
		if(HAL_GPIO_ReadPin(nIrqPort, nIrqPin) == GPIO_PIN_SET) continue;

		/// Read Interrupts.
		readAndClearInterrupts();

		/// Check for Packet Sending Success.
		if((interruptStatus & INT_TX_PACKET_SENT) == INT_TX_PACKET_SENT)
		{
			setOperatingMode(IDLE_READY | TUNE | SENSOR);
			//HAL_Delay(30);

			/// Check for Response or just return True.
			if(waitForAckResponse)
			{
				/// Wait for Acknowledgment Packet.
				if(waitForAckPacket(responseTimeout))
				{
					/// Check for Received Packet.
					getReceivedDataPacket(responseBuffer, responseSize);
					return SET;
				}
				else return RESET;
			}
			else return SET;
		}
	}

	return RESET;
}


///-32-
///
_Bool sendDataPacket(uint8_t* data, uint8_t size)
{
	/*
		Function to Send Data from the RF Module.
		-	*data:		Data to Write on the Transmit Registers.
		-	size:		Size of the TX Data.
	*/
	/// Clear Old TX Data.
	clearTxFIFO();

	/// Write New Data to the FIFO and Its Length.
	status	=	writeDataToRF(data, size);

	/// Reset Previous Enabled Interrupts
	setInterrupts(INT_TX_PACKET_SENT);

	/// Read interrupts to Remove previous status.
	status	=	readAndClearInterrupts();

	/// Change Mode to TX Mode.
	status	=	setOperatingMode(IDLE_READY | TX);

	/// Local Variables.
	uint32_t entryTimeTx	=	HAL_GetTick();
	uint32_t timeStampTx	=	entryTimeTx;
//	timeStampTx				=	entryTimeTx;

	/// Check for Timeout.
	while((timeStampTx - entryTimeTx) <= MAX_TRANSMIT_TIMEOUT)
	{
		//HAL_Delay(0);

		/// Get Time.
		timeStampTx	=	HAL_GetTick();

		/// In case of timer overflow.
		if(entryTimeTx >  timeStampTx) entryTimeTx	=	timeStampTx;

		/// Check for Interrupt Pin.
		if(HAL_GPIO_ReadPin(nIrqPort, nIrqPin) == GPIO_PIN_SET) continue;

		/// Read Interrupts.
		readAndClearInterrupts();

		/// Check for Packet Sending Success.
		if((interruptStatus & INT_TX_PACKET_SENT) == INT_TX_PACKET_SENT)
		{
			setOperatingMode(IDLE_READY | TUNE | SENSOR);
			//HAL_Delay(30);
			return SET;
		}
	}

	return RESET;
}

///-33-
/// Function to Wait for the Acknowledgment Packet.
_Bool waitForAckPacket(uint32_t responseTimeout)
{
	/*
		Function to Send Data from the RF Module.
		-	waitingTime:	Response Waiting Time.
		-
	*/
	/// Start Receiving Data.
	startListening();

	/// Local Variable.
	uint32_t enterTimeWait	=	HAL_GetTick();
	uint32_t timeStampWait	=	enterTimeWait;
//	timeStampWait	=	enterTimeWait;

	while ((timeStampWait - enterTimeWait) <= responseTimeout)
	{
		//HAL_Delay(0);

		/// Get Time.
		timeStampWait	=	HAL_GetTick();

		/// In case of timer overflow.
		if(enterTimeWait > timeStampWait)	enterTimeWait	=	timeStampWait;

		/// Check for Packet.
		if (isDataPacketReceived() == RESET)	continue;
		else	return SET;

	}

	/// Update RF Mode to Ready and Sensor Mode.
	setOperatingMode(IDLE_READY | SENSOR);

	/// Clear RX FIFO to avoid overflow.
	clearRxFIFO();

	return RESET;
}
///-30-
/// Function to Prepare for the RX Mode.
HAL_StatusTypeDef startListening()
{
	/*
		Function to Put the RF Module into RX Mode.
		-
		-
	*/

	/// Clear RX FIFO to Avoid overflow issues and Start Listening.
	status	=	clearRxFIFO();

	/// Set Interrupts for RX Mode.
	status	=	setInterrupts((INT_RX_VALID_PACKET | INT_RX_CRC_ERROR));

	/// Read Interrupts to Clear them.
	status	=	readAndClearInterrupts();

	/// Enable RX Mode.
	status	=	setOperatingMode(IDLE_READY | RX | SENSOR);

	return status;
}

///-31-
/// Function to Check for Received.
_Bool isDataPacketReceived()
{
	/*
		Function to Check, does the Data Packet Received.
		-
		-
	*/
	//HAL_Delay(0);
	/** Check Status of Interrupt Pin **/
	if(HAL_GPIO_ReadPin(nIrqPort, nIrqPin) == GPIO_PIN_SET) return RESET;

	/** Read and Clear Interrupts **/
	readAndClearInterrupts();

	/** Check for the Received Packet Interrupt **/
	if((interruptStatus & INT_RX_VALID_PACKET) == INT_RX_VALID_PACKET)
	{
		setOperatingMode(IDLE_READY | TUNE| SENSOR);
		return SET;
	}
	else if((interruptStatus & INT_RX_CRC_ERROR) == INT_RX_CRC_ERROR)
	{
		/** Disable RX Mode to Clear RX FIFO **/
		setOperatingMode(IDLE_READY| SENSOR);
		clearRxFIFO();
		/** Enable RX Mode Again **/
		setOperatingMode(IDLE_READY | RX | SENSOR);
		return RESET;
	}
	return RESET;
}

///-32-
///
HAL_StatusTypeDef getReceivedDataPacket(uint8_t* data, uint8_t* size)
{
	/*
		Function to Get Received Data from the RF Module.
		-	*data:		Data to Read from the Received Registers.
		-	size:		Size of the RX Data.
	*/
	/// Read Data from the RF Module including Size of the Received Packet.
	status	=	readDataFromRF(data, size);
	if(status != HAL_OK) return status;
	/// Clear Old Data.
	status	=	clearRxFIFO();
	return status;
}

///-33-
///
HAL_StatusTypeDef clearTxFIFO()
{
	/// Local Variables.
	uint8_t opFuncCtrlReg	=	0x01; /* 0x01 for TX Clearance */

	/// Setting FFCLRRX =1 followed by FFCLRRX = 0 will clear the contents of the RX FIFO.
	status	=	write_Reg_To_RF_SPI(&opFuncCtrlReg2Addr, &opFuncCtrlReg, sizeof(opFuncCtrlReg));
	if(status != HAL_OK) return status;
	opFuncCtrlReg	=	0x00;
	status	=	write_Reg_To_RF_SPI(&opFuncCtrlReg2Addr, &opFuncCtrlReg, sizeof(opFuncCtrlReg));
	return status;
}

///-34-
///
HAL_StatusTypeDef clearRxFIFO()
{
	/// Local Variables.
	uint8_t opFuncCtrlReg	=	0x02; /* 0x02 for RX Clearance */

	/// Setting FFCLRRX =1 followed by FFCLRRX = 0 will clear the contents of the RX FIFO.
	status	=	write_Reg_To_RF_SPI(&opFuncCtrlReg2Addr, &opFuncCtrlReg, sizeof(opFuncCtrlReg));
	if(status != HAL_OK) return status;
	opFuncCtrlReg	=	0x00;
	status	=	write_Reg_To_RF_SPI(&opFuncCtrlReg2Addr, &opFuncCtrlReg, sizeof(opFuncCtrlReg));
	return status;
}

///-35-
///
HAL_StatusTypeDef clearFIFOs()
{
	/// Local Variables.
	uint8_t opFuncCtrlReg	=	0x03; /* 0x03 for both TX and RX Clearance */

	/// Setting FFCLRRX =1 followed by FFCLRRX = 0 will clear the contents of the RX FIFO.
	status	=	write_Reg_To_RF_SPI(&opFuncCtrlReg2Addr, &opFuncCtrlReg, sizeof(opFuncCtrlReg));
	if(status != HAL_OK) return status;
	opFuncCtrlReg	=	0x00;
	status	=	write_Reg_To_RF_SPI(&opFuncCtrlReg2Addr, &opFuncCtrlReg, sizeof(opFuncCtrlReg));
	return status;
}

///-36-
/// Function to Write Data to the RF Module through SPI.
static HAL_StatusTypeDef writeDataToRF(uint8_t* fifo, size_t size)
{
	/*
		Function to Write Transmitting Data to the RF Module.
		-	*fifo:		Data to Send to the FIFO.
		-	size:		Size of the RX Data.
	*/
	/// Local Variables.
	uint8_t txPacketLengthRegAddr	=	0x3E;	/* TX Packet Length Register */
	uint8_t fifoWriteRegAddr		=	fifoBaseRegAddr;//(uint8_t)(fifoBaseRegAddr	| 0x80); // Read from MSB First.
	uint8_t length					=	(uint8_t)size;

	/// Write Packet Length to the Length Register.
	status	=	write_Reg_To_RF_SPI(&txPacketLengthRegAddr, &length, sizeof(length));
	if(status != HAL_OK) return status;

	/// Write Data to the RF Module FIFO.
	status	=	write_Reg_To_RF_SPI(&fifoWriteRegAddr, fifo, length);
	return status;
}

///-37-
/// Function to Read Data from the RF Module through SPI.
static HAL_StatusTypeDef readDataFromRF(uint8_t* fifo, uint8_t* size)
{
	/*
		Function to Read Received Data from the RF Module.
		-	*fifo:		Data to Read from the FIFO.
		-	size:		Size of the RX Data.
	*/
	/// Local Variables.
	uint8_t rxPacketLengthRegAddr	=	0x4B; /* RX Packet Length Register */
	uint8_t rxPacketLengthReg		=	0;
	uint8_t fifoReadRegAddr			=	fifoBaseRegAddr;//(uint8_t)(fifoBaseRegAddr & 0x7F);
//	uint8_t	emptyFifo[64]			=	{ 0 };

	/// Read Packet Length from the RF Module FIFO.
	status	=	read_Reg_From_RF_SPI(&rxPacketLengthRegAddr, &rxPacketLengthReg, sizeof(rxPacketLengthReg));
	if(status != HAL_OK) return status;

	/// Read Data from the RF Module FIFO.
	status	=	read_Reg_From_RF_SPI(&fifoReadRegAddr, fifo, rxPacketLengthReg);
	if(status != HAL_OK) return status;

//	/// Write Empty Data to the RF Module FIFO.
//	status	=	write_Reg_To_RF_SPI(&fifoReadRegAddr, emptyFifo, sizeof(emptyFifo));
//	if(status != HAL_OK)

	*size	=	rxPacketLengthReg;

	return status;

//	return HAL_OK;
}

///-38-
/// Function to Write Data into the RF module through SPI.
HAL_StatusTypeDef softReset()
{
	/// Set Reset Register.
	operAndFuncCtrl1Reg	=	0x80;

	/// Send Reset Command.
	status	=	write_Reg_To_RF_SPI(&opFuncCtrlReg1Addr, &operAndFuncCtrl1Reg, 1);
	if(status != HAL_OK) return status;

	/// Max Delay for Power On Reset. For Further Details see on Page 50 of the Data-sheet.
	#if !defined(noDelay)
		HAL_Delay(0);
	#endif

	uint32_t timeOut	=	5000;
	uint32_t lastTime	=	HAL_GetTick();
	uint32_t timeStamp	=	lastTime;

	while(((timeStamp - lastTime) <= timeOut))// && (!_Bool(interruptStatus & INT_CHIP_READY)))
	{
		if((interruptStatus & INT_CHIP_READY) == INT_CHIP_READY)
		{ 	setInterrupts(INT_POWER_ON_RESET);	readAndClearInterrupts();		break; }
		else
		{	readAndClearInterrupts();		timeStamp	=	HAL_GetTick();	HAL_Delay(0);	}
	}

	/// Reload the Previous Configurations.
	status	=	reloadConfigurations();
	return status;
}

///-39-
/// Function to Write Data into the RF module through SPI.
HAL_StatusTypeDef hardReset()
{
	/// Enter Shutdown Mode.
	status	=	shutdown(SET); 	/* Enter RF Module into Shutdown Mode */
	if(status != HAL_OK) return status;
	HAL_Delay(10-1);
	/// Enter Power UP Mode.
	status	=	shutdown(RESET);	/* Enable RF Module Power ON */
	if(status != HAL_OK) return status;

	/// Max Delay for Power On Reset. For Further Details see on Page 50 of the Data-sheet.
	HAL_Delay(50-1);

	/// Reconfigure the module with previous Configurations.
	status	=	reloadConfigurations();
	return status;
}





///-41-
/// Function to Read Settings from the RF module through SPI.
HAL_StatusTypeDef readSettings(uint8_t* settings)
{
	status = read_Reg_From_RF_SPI(&deviceTypeRegAddr, settings, 0x7F);
	return status;
}

///-42-
/// Function to Read Settings from the RF module through SPI.
HAL_StatusTypeDef readRegisters(uint8_t* registers, uint8_t size)
{
	status = read_Reg_From_RF_SPI(&deviceTypeRegAddr, registers, size);
	return status;
}

///-43-
/// Function to Write Data into the RF module through SPI.
static HAL_StatusTypeDef write_Reg_To_RF_SPI(uint8_t* address, uint8_t* data, size_t size)
{
	/*
		Function to write data to RF SPI.
		-	*address:	Address Pointer for the register to be written.
		-	*data:		Data Pointer for the data array to be written.
		-	size:		Data Array size to be written.
	*/
	uint8_t writeAddress = (*address | 0x80); /* 0x80 is Mandatory for Writing Data i.e. MSB High is for Writing */

	/// Enable Chip Select (Write Active LOW).
	HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_RESET);

	/// Write Register.
	status = HAL_SPI_Transmit(rf_spi, &writeAddress, 1, 10);
	/// Malfunction Action.
	if(status != HAL_OK) { HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_SET);	return status; }

	/// Write Register.
	status = HAL_SPI_Transmit(rf_spi, data, size, 100);
	/// Malfunction Action.
	if(status != HAL_OK) { HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_SET);	return status; }

	/// Disable Chip Select (Write Active HIGH).
	HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_SET);

	return HAL_OK;
}

///-44-
/// Function to Read Data from the RF Module through SPI.
static HAL_StatusTypeDef read_Reg_From_RF_SPI(uint8_t* address, uint8_t* data, size_t size)
{
	/*
		Function to Read data from RF SPI.
		-	*address:	Address Pointer for the register to be written.
		-	*data:		Data Pointer for the data array to be written.
		-	size:		Data Array size to be written.
	*/
	uint8_t readAddress = (*address & 0x7F);

	/// Enable Chip Select (Write Active LOW).
	HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_RESET);

	/// Read Register.
	status = HAL_SPI_Transmit(rf_spi, &readAddress, 1, 10);
	/// Malfunction Action.
	if(status != HAL_OK) { HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_SET);	return status; }

	/// Read Register.
	status = HAL_SPI_Receive(rf_spi, data, size, 100);
	/// Malfunction Action.
	if(status != HAL_OK) { HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_SET);	return status; }

	/// Disable Chip Select (Write Active HIGH).
	HAL_GPIO_WritePin(nSelPort, nSelPin, GPIO_PIN_SET);

	return HAL_OK;
}

///-45-
///
static double power(float x, int y)
{
    float temp;
    if (y == 0)
        return 1;
    temp = power(x, y / 2);
    if (y % 2 == 0)
        return temp * temp;
    else {
        if (y > 0)
            return x * temp * temp;
        else
            return (temp * temp) / x;
    }
}





