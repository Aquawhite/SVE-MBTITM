//*****************************************************************************
// SVE MBTI V2.6
// SVE MBTI V2.7 External Feeder Page upgraded (Have timer mode and can monitor variables)
// Add External Feeder Timer Mode
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"

#define TRUE	1
#define FALSE	0

#define FORWARD 1
#define BACKWARD 0

#define ENCPULSE 			144
#define SERVOPULSE 			4000
#define RATIONUMERATOR		4
#define RATIODENOMINATOR	1
#define SERVOOFFSET			0
#define SERVOLOWERSAFETY 	600
#define ACTIVEPHASEACCEL	156
#define ACTIVEPHASEDECEL	156
#define PASSIVEPHASEACCEL   156
#define PASSIVEPHASEDECEL   254

#define UARTDISABLEDCOUNT			1000
#define PROGRAMVERSION		8
//*****************************************************************************
// Defines for the pins that are used in the device
//*****************************************************************************

// Button PIN Mapping

#define PIN_RX_BASE					GPIO_PORTA_BASE
#define PIN_RX_PIN					GPIO_PIN_0
#define PORT_TX_BASE				GPIO_PORTA_BASE
#define PORT_TX_PIN					GPIO_PIN_1
#define PORT_SSI_CLK_BASE			GPIO_PORTA_BASE
#define PORT_SSI_CLK_PIN			GPIO_PIN_2
#define PORT_SSI_FSS_BASE			GPIO_PORTA_BASE
#define PORT_SSI_FSS_PIN			GPIO_PIN_3
#define PIN_SSI_RX_BASE				GPIO_PORTA_BASE
#define PIN_SSI_RX_PIN				GPIO_PIN_4
#define PORT_SSI_TX_BASE			GPIO_PORTA_BASE
#define PORT_SSI_TX_PIN				GPIO_PIN_5
#define PORT_SCL_BASE				GPIO_PORTA_BASE
#define PORT_SCL_PIN				GPIO_PIN_6
#define PIN_SDA_BASE				GPIO_PORTA_BASE
#define PIN_SDA_PIN					GPIO_PIN_7

#define PIN_REWINDER_FJ1_BASE		GPIO_PORTB_BASE
#define PIN_REWINDER_FJ1_PIN		GPIO_PIN_0
#define PIN_REWINDER_FJ2_BASE		GPIO_PORTB_BASE
#define PIN_REWINDER_FJ2_PIN		GPIO_PIN_1
#define PORT_SERVO_ALMRST_BASE		GPIO_PORTB_BASE
#define PORT_SERVO_ALMRST_PIN		GPIO_PIN_2
#define PORT_SERVO_ON_BASE			GPIO_PORTB_BASE
#define PORT_SERVO_ON_PIN			GPIO_PIN_3
#define PIN_SERVO2_ALARM_BASE		GPIO_PORTB_BASE
#define PIN_SERVO2_ALARM_PIN		GPIO_PIN_4
#define PIN_SERVO2_READY_BASE		GPIO_PORTB_BASE
#define PIN_SERVO2_READY_PIN		GPIO_PIN_5

#define PORT_TCK_BASE				GPIO_PORTC_BASE
#define PORT_TCK_PIN				GPIO_PIN_0
#define PORT_TMS_BASE				GPIO_PORTC_BASE
#define PORT_TMS_PIN				GPIO_PIN_1
#define PIN_TDI_BASE				GPIO_PORTC_BASE
#define PIN_TDI_PIN					GPIO_PIN_2
#define PORT_TDO_BASE				GPIO_PORTC_BASE
#define PORT_TDO_PIN				GPIO_PIN_3
#define PIN_PULSE_Z_BASE			GPIO_PORTC_BASE
#define PIN_PULSE_Z_PIN				GPIO_PIN_4
#define PIN_PULSE_A_BASE			GPIO_PORTC_BASE
#define PIN_PULSE_A_PIN				GPIO_PIN_5
#define PIN_PULSE_B_BASE			GPIO_PORTC_BASE
#define PIN_PULSE_B_PIN				GPIO_PIN_6
#define PORT_EX1_BASE				GPIO_PORTC_BASE
#define PORT_EX1_PIN				GPIO_PIN_7

#define PORT_TRO2_BASE				GPIO_PORTD_BASE
#define PORT_TRO2_PIN				GPIO_PIN_0
#define PIN_PRESSURE_SW_BASE		GPIO_PORTD_BASE
#define PIN_PRESSURE_SW_PIN			GPIO_PIN_1
#define PIN_EYEMARK_BASE			GPIO_PORTD_BASE
#define PIN_EYEMARK_PIN				GPIO_PIN_2
#define PIN_HOPPER_BASE				GPIO_PORTD_BASE
#define PIN_HOPPER_PIN				GPIO_PIN_3
#define PORT_COUNTERC_BASE			GPIO_PORTD_BASE
#define PORT_COUNTERC_PIN			GPIO_PIN_4
#define PIN_TRI1_BASE				GPIO_PORTD_BASE
#define PIN_TRI1_PIN				GPIO_PIN_5
#define PIN_TRI2_BASE				GPIO_PORTD_BASE
#define PIN_TRI2_PIN				GPIO_PIN_6
#define PORT_TRO1_BASE				GPIO_PORTD_BASE
#define PORT_TRO1_PIN				GPIO_PIN_7

#define PORT_REWINDER_RELAY1_BASE	GPIO_PORTE_BASE
#define PORT_REWINDER_RELAY1_PIN	GPIO_PIN_0
#define PORT_REWINDER_RELAY2_BASE	GPIO_PORTE_BASE
#define PORT_REWINDER_RELAY2_PIN	GPIO_PIN_1
#define PORT_GUSSET_VALVE_ROLL_BASE	GPIO_PORTE_BASE
#define PORT_GUSSET_VALVE_ROLL_PIN	GPIO_PIN_2
#define PORT_GUSSET_VALVE_BLOW_BASE	GPIO_PORTE_BASE
#define PORT_GUSSET_VALVE_BLOW_PIN	GPIO_PIN_3
#define PORT_SERVO2_ON_BASE			GPIO_PORTE_BASE
#define PORT_SERVO2_ON_PIN			GPIO_PIN_4
#define PORT_SERVO2_ALMRST_BASE		GPIO_PORTE_BASE
#define PORT_SERVO2_ALMRST_PIN		GPIO_PIN_5
#define PIN_PROX3_BASE				GPIO_PORTE_BASE
#define PIN_PROX3_PIN				GPIO_PIN_6
#define PORT_VIB_BASE				GPIO_PORTE_BASE
#define PORT_VIB_PIN				GPIO_PIN_7

#define PIN_SSI1_RX_BASE			GPIO_PORTF_BASE
#define PIN_SSI1_RX_PIN				GPIO_PIN_0
#define PORT_SSI1_TX_BASE			GPIO_PORTF_BASE
#define PORT_SSI1_TX_PIN			GPIO_PIN_1
#define PORT_SSI1_CLK_BASE			GPIO_PORTF_BASE
#define PORT_SSI1_CLK_PIN			GPIO_PIN_2
#define PORT_SSI1_FSS_BASE			GPIO_PORTF_BASE
#define PORT_SSI1_FSS_PIN			GPIO_PIN_3
#define PORT_BRAKE_BASE				GPIO_PORTF_BASE
#define PORT_BRAKE_PIN				GPIO_PIN_4
#define PORT_CLUTCH_BASE			GPIO_PORTF_BASE
#define PORT_CLUTCH_PIN				GPIO_PIN_5
#define PORT_INV1_BASE				GPIO_PORTF_BASE
#define PORT_INV1_PIN				GPIO_PIN_6
#define PIN_DC_SENS_BASE			GPIO_PORTF_BASE
#define PIN_DC_SENS_PIN				GPIO_PIN_7

#define PIN_FOILJAM_BASE			GPIO_PORTG_BASE
#define PIN_FOILJAM_PIN				GPIO_PIN_0
#define PIN_HEATERERROR_BASE		GPIO_PORTG_BASE
#define PIN_HEATERERROR_PIN			GPIO_PIN_1
#define PIN_LIMIT_SW_BASE			GPIO_PORTG_BASE
#define PIN_LIMIT_SW_PIN			GPIO_PIN_2
#define PIN_AIR_ERROR_BASE			GPIO_PORTG_BASE
#define PIN_AIR_ERROR_PIN			GPIO_PIN_3
#define PIN_SERVO_ALARM_BASE		GPIO_PORTG_BASE
#define PIN_SERVO_ALARM_PIN			GPIO_PIN_4
#define PIN_SERVO_READY_BASE		GPIO_PORTG_BASE
#define PIN_SERVO_READY_PIN			GPIO_PIN_5
#define PIN_PROX1_BASE				GPIO_PORTG_BASE
#define PIN_PROX1_PIN				GPIO_PIN_6
#define PIN_PROX2_BASE				GPIO_PORTG_BASE
#define PIN_PROX2_PIN				GPIO_PIN_7

#define PORT_GAS_BASE				GPIO_PORTH_BASE
#define PORT_GAS_PIN				GPIO_PIN_0
#define PORT_EX4_BASE				GPIO_PORTH_BASE
#define PORT_EX4_PIN				GPIO_PIN_1
#define PORT_EX3_BASE				GPIO_PORTH_BASE
#define PORT_EX3_PIN				GPIO_PIN_2
#define PORT_EX2_BASE				GPIO_PORTH_BASE
#define PORT_EX2_PIN				GPIO_PIN_3
#define PORT_SERVO2_PWM_BASE		GPIO_PORTH_BASE
#define PORT_SERVO2_PWM_PIN			GPIO_PIN_4
#define PORT_SERVO2_DIR_BASE		GPIO_PORTH_BASE
#define PORT_SERVO2_DIR_PIN			GPIO_PIN_5
#define PORT_SERVO_DIR_BASE			GPIO_PORTH_BASE
#define PORT_SERVO_DIR_PIN			GPIO_PIN_6
#define PORT_SERVO_PWM_BASE			GPIO_PORTH_BASE
#define PORT_SERVO_PWM_PIN			GPIO_PIN_7

#define PIN_REWINDER_SENSA_BASE		GPIO_PORTJ_BASE
#define PIN_REWINDER_SENSA_PIN		GPIO_PIN_0
#define PIN_REWINDER_SENSB_BASE		GPIO_PORTJ_BASE
#define PIN_REWINDER_SENSB_PIN		GPIO_PIN_1
#define PORT_GUSSET_VALVE3_BASE		GPIO_PORTJ_BASE
#define PORT_GUSSET_VALVE3_PIN		GPIO_PIN_2

#define PIN_BTN_RUN_BASE			GPIO_PORTK_BASE
#define PIN_BTN_RUN_PIN				GPIO_PIN_0
#define PIN_BTN_STOP_BASE			GPIO_PORTK_BASE
#define PIN_BTN_STOP_PIN			GPIO_PIN_1
#define PIN_GUSSET_BTN_VALVE_BASE	GPIO_PORTK_BASE
#define PIN_GUSSET_BTN_VALVE_PIN	GPIO_PIN_2
#define PORT_DATECODE_BASE			GPIO_PORTK_BASE
#define PORT_DATECODE_PIN			GPIO_PIN_3

/* Control Data */
unsigned char MotorON = FALSE, DatecodeON = FALSE, ClutchON = FALSE, BreakON = FALSE;
unsigned char VibratorHopperON = FALSE, VibratorFormerON = FALSE, CutterON = FALSE, CounterCutterON = FALSE, ZeroIndexON = FALSE, CheckPulseCom = FALSE;
unsigned char StopCommand = FALSE, EyemarkPosition, LowSpeedMode = FALSE;
short DACSpeed;
unsigned int  ProductCountBitCounter;
unsigned int  EyemarkLampCounter, HeaterErrorCounter = 3000, FoilJamCounter = 1000, PulseErrorCounter = 800, ZeroIndexCounter = 0,CheckPulseCounter = 0;
unsigned int  PulseCheckOKCounter = 3000, CounterEyemarkIgnore = 300, PulseErrorTimerCounter = 5000;
unsigned int  InitialSpeed;
unsigned char TROOnce, ComFlag = FALSE, ExCycle = FALSE, ExCycleFlag = FALSE;
unsigned char HeaterErrorMsgDuration, HeaterErrorMsg, FoilJamMsgDuration, FoilJamMsg, PulseErrorMsgDuration, PulseErrorMsg, NoAirErrorMsg, NoAirErrorMsgDuration;
signed char OffsetSpeed;
int EncoderBuffer;
unsigned char OCycEyemark = FALSE;
unsigned char BrakePos = 68, CounterCutterPos = 72;
unsigned char EyemarkIgnoreCounter = 0, EyemarkIgnore;
unsigned char ZeroIndex32, ZeroIndex16, ZeroIndex8, ZeroIndex4, ZeroIndex2, ZeroIndex1;
unsigned char ZeroIndex1Counter, ZeroIndex2Counter, ZeroIndex4Counter, ZeroIndex8Counter, ZeroIndex16Counter, ZeroIndex32Counter;
unsigned char GasStartPos, GasEndPos;
unsigned char MotorFlag;
unsigned long int LifeDuration = 0;
unsigned char RunButton = 100, StopButton = 100, GussetButton, BufferSensor = 12, LongBufferSensor = 100;
/* Status Data */
unsigned char ServiceMode = FALSE, LastServiceMode = FALSE, MachineRunning = FALSE, Initialised = FALSE, InchingMode = FALSE;
unsigned char Error = FALSE, PulseError = FALSE, ZIError = FALSE, FatalError = FALSE;
unsigned char CNBStopTimerFlag = FALSE, CounterEyemark = FALSE, CheckPulseMode = FALSE;
unsigned int  SpeedperMinEncoder = 0, SpeedperMinCounter = 0;
/* Communication : TouchScreen */
// Operation Page
    // Input
unsigned char EyemarkEN, DatecodeEN, CounterCutterEN, ToggleAuto, MotorONBtn, MotorOFFBtn;                                                  // HMI BITOUT
unsigned char VibratorCounter, CounterCutterSetting;                                                                                                        // HMI BYTEOUT
unsigned char HopperBufferSensor, HopperBufferSensorCounter;
    // Output
unsigned char EyemarkON /* DatecodeON, VibratorFormerON, ClutchON, BreakON, PulseError*/, PulseCheckOK, FoilJam;                                            // HMI BITIN1
unsigned char /* MachineRunning, ServiceMode, */ HeaterError, HopperError, EyemarkError, /*ZIError, */ DoorOpen/*,CounterCutterON*/, BlinkCom;              // HMI BITIN2
unsigned char SpeedperMin, CounterCutterCounter /* ZIErrorCounter = 0, EyemarkErrorCounter = 0 */, ProductCountBit, DWLimFoil, FWLimFoil;
unsigned long int ProductCounter;                                                                                                                           // HMI BYTEIN
unsigned char ComDACSpeed;
unsigned char PulseCheckError;
unsigned char PulseCommutationError;
unsigned char SpeedModeSelector;
unsigned char CounterCutterCounterCom;
unsigned char ExternalFeederDuration, ExternalFeederDurationCounter;
unsigned char ExternalFeederOFFTimer, ExternalFeederOFFTimerCounter;
bool ExternalFeederON, ExternalFeederEnable, ExternalFeederManualEnable, ExternalFeederAgitatorEnable;
bool ExternalFeederTimerMode;
bool HopperSensorEnable;
unsigned char HopperErrorCounter, HopperSensorONCounter;
/* Counter Data */
unsigned int CNBStopTimer = 0;
unsigned char ZIErrorCounter = 0, EyemarkErrorCounter = 0;
unsigned int CounterCutterTimer;
/* Communication Data Declaration */
unsigned int  ReadACKCounter = 1000;
unsigned char dataIndex = 0, dataRXIndex = 0, dataTXIndex = 0;
unsigned char  ComAddress = 0, BytestoRead = 0;
unsigned char ComOUT[120], ComIN[120], CRCSum, CRCTens, CRCSingles;
bool ComDirection = FALSE, ComEnable = FALSE, DoubleSpeed = FALSE, UARTDisabled = FALSE;
unsigned char DataTimerTrigger = FALSE, DataLock = FALSE, DataReady = FALSE, ReadACK = FALSE;
unsigned char TransferIdle = FALSE, ReceiveIdle = FALSE;
unsigned int  IntTXCounter = 0, IntRXCounter = 0;
unsigned long int DataTimerCounter = 0;
unsigned int  UARTTimeOut = UARTDISABLEDCOUNT, UARTDisabledCounter = 0;
    // Communicated Data
unsigned char DatecodeEnable, VibratorEnable, CounterCutterEnable, AutoEnable;
unsigned char StopAt, StandbyAt, DatecodeEnable, VibratorEnable, CounterCutterEnable, AutoEnable;
/* Display System Declaration*/
unsigned char unchrPageNo, unchrPageMax, unchrPageMax2, unchrModeSelect, unchrModeSelectmax, unchrCategorySel, unchrCategorySelmax;
bool blinkFlag = FALSE, ModifiableFlag = FALSE, DispInitFlag = FALSE;
unsigned int Displayinitcounter = 15000,overdisplaycnt, blinkON = 400, blinkOFF = 400;
unsigned char unchrDecimal, unchrDecimalmax, unchrDisplaycounter;
/* Buttons Declaration
unsigned int PBBufferUP, PBBufferDW, PBBufferMD, PBBufferSHF;
unsigned int ButtonBufferValue = 7, HoldButtonValue = 700, SpecialButtonValue = 3000;
bit FlagUPButton = FALSE, FlagDWButton = FALSE, FlagMDButton = FALSE, FlagMDButton2 = FALSE, FlagMDButton3=  FALSE, FlagSHFButton = FALSE;
bit PBBufferUPFlag = FALSE, PBBufferDWFlag = FALSE, PBBufferMDFlag = FALSE; */
/* Encoder Data */
int EncoderData, LastEncoder;
unsigned char ErrorFlag = FALSE, ZeroingEncoder = FALSE;
unsigned char CurrentPos = 0;
unsigned int ErrorRefresh = 0;
/* Debug Data */
unsigned int debugcounter0 = 0, debugcounter1 = 0;
/* Saved Data */
unsigned long int ProductCounterepr;
unsigned long int Eepromsave = 1000000;
unsigned char autosaveflag;
// Communication SPI
unsigned long SPIOut[26], SPIIn[26];
unsigned char SPICounter, SPIReiterationCounterIN, SPIReiterationCounterOUT, SPICounterSkip;
unsigned char SPIDataLock, SPIDataReady, SPIReiterationMode;                            // bit
unsigned char SPIHeaterON1, SPIHeaterON2, SPIHeaterON3, SPIHeaterON4;
unsigned char SPIInTEST, SPIOutTEST, SPIStartTimer = 250;
unsigned int  SPITimerCounter = 0;
unsigned char SPIComErrorFlag, SPIComError;
unsigned int  SPIComErrorCounter;
unsigned char SPITransmitted = FALSE;

unsigned long SPIOut2[26], SPIIn2[26];
unsigned char SPICounter2,  SPICounterSkip2;
unsigned char SPIDataLock2, SPIDataReady2;
unsigned char SPIStartTimer2 = 250;
unsigned int  SPITimerCounter2 = 0;
unsigned char SPIComErrorFlag2, SPIComError2;
unsigned int  SPIComErrorCounter2;
unsigned char SPITransmitted2 = FALSE;
// Heater Module
unsigned char Heater1SV, Heater2SV, Heater3SV, Heater4SV;
unsigned char Heater1PV, Heater2PV, Heater3PV, Heater4PV;
unsigned char Heater1ON, Heater2ON, Heater3ON, Heater4ON, Heater1Ready, Heater2Ready, Heater3Ready, Heater4Ready;               // bit
unsigned char Heater1HLimit, Heater2HLimit, Heater3HLimit, Heater4HLimit;
unsigned char Heater1LLimit, Heater2LLimit, Heater3LLimit, Heater4LLimit;
int TempHeater1HLimit, TempHeater2HLimit, TempHeater3HLimit, TempHeater4HLimit;
int TempHeater1LLimit, TempHeater2LLimit, TempHeater3LLimit, TempHeater4LLimit;
unsigned char HeaterNoConnection;// YesHeaterNoConnection;
unsigned char Heater1Enable, Heater2Enable, Heater3Enable, Heater4Enable;
unsigned char PIDmultiplier;
// Standby Module
unsigned char StandbyMode, ContinuousMode;
unsigned int StandbyModeCounter = 3000;
unsigned char TROPosition, TROLaunched, TRIReceived;
unsigned int TRIDelay, TRIDelayTimer;
unsigned char TROCoil, TRICoil, TRIOnDelay, FirstCycleFlag;
unsigned int TRICoilCounter, TROCoilCounter;
unsigned char TROTrail, TROTrailCounter;
unsigned char TROManual, LastTROManual;
unsigned char TROMin, TROMinCounter;
signed int Offset[4], Gradient[4];
// Servo Module
unsigned char PulseON, LastPulseON, ServoError, ServoReady, PulseManualON, LastPulseManualON, ServoAlmRst;
unsigned long int ServoSpeed, PulseManual = 1600, PulseManualCounter = 0, ServoDrainSpeed;
unsigned int ServoTimerCounter;
unsigned char DrainingMode = FALSE, StoppingMode = FALSE;
unsigned char ServoStartPos, ServoEnable;
unsigned int ServoSignalDly, ServoSignalDlyCount;
unsigned long int konstant = 4800000, konstant10 = 48000000, longinttemp;    // (Pulse/sec = konstant/rpm) for 200 pulse/rev is 24000000, for 1000 pulse/rev is 4800000
unsigned char StopOnBtn;
unsigned char ServoBacklashEnable, ServoBacklashMode, BacklashON, LastBacklashON, LastServoDrain, ServoDrain;
unsigned long int ServoBacklashPulse, ServoBacklashSpeed;
unsigned int ServoBacklashTimer, ServoBacklashTimerCounter;
unsigned char ServoDirection;
unsigned char PWMWatch;
unsigned int PulseSum;
unsigned long int ServoAccelRate;
unsigned int ServoErrorCounter;
// No need ServoErrorMsg because need to be reset when happening, and cause confusion if not gone right away
	// Servo Valve
	unsigned int ServoValveDelay, ServoValveDelayCounter;			// It Opens the valve as soon as PulseON = TRUE, and Closes with optional delay
	unsigned int ServoValveHeadStart, ServoValveHeadstartVar;
	bool ServoValveEnable, ServoValveSensorEnable;
	unsigned char ServoValveHeadStartTimerDrain, ServoValveHeadStartTimerPMON, ServoValveHeadStartTimerBLON;
	bool ServoDrainFlag, PulseManualONFlag, BacklashONFlag;
	// Accel Decel
	unsigned long int ServoTargetSpeed = 32000, ServoTargetSpeedRPM, ServoCurrentSpeed, ServoCurrentRPM10 , LastServoTargetSpeed = 32000, ServoDecelAt, ServoAccelPulse;
	unsigned long int ServoAccelTime, ServoSpeedRPM;
	bool ServoAccDccEnable;
	bool ServoAccelFlag, ServoDecelFlag;
	unsigned char ServoValveErrorMsg, ServoValveErrorMsgDuration;
// Gusset Module
unsigned char GussetEnable;
unsigned int VlvBlowTimerCounter, VlvBlowTimer;
unsigned char VlvBlowPos, VlvRollerON, VlvBlowON;
unsigned char VlvRollerButtonFlag;
unsigned char VlvCutStart, VlvCutEnd;
unsigned char NoAirError, NoAirErrorCounter = 255, IgnoreAirError = TRUE;
// Rewind Module
unsigned char RewindEnable;
// Datecode Sensor Module
unsigned char DatesensEN, DatesensError;
unsigned int  DatesensCounter;
// Agitator Operation
unsigned char AgitatorTimer = 30, AgitatorStarter;
unsigned char AgitatorEnable, AgitatorManual;
unsigned char VibratorMode;
// Air Cutter Module
unsigned char AirCutterEnable, AirCutterPos, AirCutterTimer, AirCutterTimerCounter;
unsigned char AirCutterON, AirCutterMode;
unsigned int AirCutterPosCounter;
// Output Module
bool OUTEX1Enable, OUTEX2Enable, OUTEX3Enable, OUTEX4Enable, OUTEX01Enable, OUTEX02Enable, OUTEX03Enable;
bool OUTEX1ON, OUTEX2ON, OUTEX3ON, OUTEX4ON, OUTEX01ON, OUTEX02ON, OUTEX03ON;
unsigned char OUTEX1StartPos, OUTEX2StartPos, OUTEX3StartPos, OUTEX4StartPos, OUTEX01StartPos, OUTEX02StartPos, OUTEX03StartPos;
unsigned char OUTEX1EndPos, OUTEX2EndPos, OUTEX3EndPos, OUTEX4EndPos, OUTEX01EndPos, OUTEX02EndPos, OUTEX03EndPos;

// Output Datecode
unsigned char DatecodeStartPos, DatecodeONDuration, DatecodeONDurationCounter;

unsigned char AgitatorON, AgitatorError;
unsigned int AgitatorErrorCounter;
/* Timer Variables */
unsigned char ms10counter, ms100counter;
unsigned long int longintdelay;

/* Test Module */
unsigned char Prox2, TempChar, ZCounter = 0;

/* Loop Analyze */
unsigned int LoopCycle, LoopTime;

/* PCB/HMI Selector */
unsigned char IsMBTITMHMI, IsMBTITMPCB = 1;
unsigned char PORTAMBTI, PORTBMBTI, PORTCMBTI, PORTDMBTI, PORTEMBTI;
unsigned char PORTFMBTI, PORTGMBTI, PORTHMBTI, PORTJMBTI;

unsigned char Digitize(unsigned long Port, unsigned long Pin)
{
	if(ROM_GPIOPinRead(Port, Pin))
		return 1;
	else
		return 0;

}
void RunServo(void)
{
	if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
	{
		PulseManualCounter = PulseManual;
		if(ServoDirection)         // forward
			ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
		else
			ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
		PulseON = TRUE;
		// Enable the PWM
		PulseSum = 0;
		if(ServoAccDccEnable)
		{
			ServoTargetSpeed = ServoSpeed;
			ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
		}
		else
		{
			ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoSpeed);
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoSpeed >> 1);  // default 50%
		}
		ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
		ServoSignalDlyCount = ServoSignalDly;
	}
}
//*****************************************************************************
// Interrupt handler for the SSI0 interrupt
//*****************************************************************************
void IntSSI0(void)
{
	SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );
	SSIDataGetNonBlocking(SSI0_BASE, &SPIIn[SPICounter]);//SPIIn[SPICounter] = SPDR;
	//PORTSS = TRUE;
	switch(SPICounter)
	{
		case 0:
			if(SPIIn[0] != 0xF0)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
				HeaterNoConnection = 0;
			}
		break;
		case 1:
			if(SPIIn[1] != 0x64)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
				HeaterNoConnection = 0;
			}
			break;
		case 2:
			if(SPIIn[2] != 0x04)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
				HeaterNoConnection = 0;
			}
			break;
		case 24:
			if(SPIIn[24] != 0x0F)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
				HeaterNoConnection = 0;
			}
			break;
		case 25:
			if(SPIIn[25] != 0x0F)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
				HeaterNoConnection = 0;
			}
	}

	if(SPICounter >= 25)                              // SPI Communication has read all the frame successfully
	{
		SPIComErrorFlag = FALSE;
		Heater1ON =        SPIIn[3] % 2;
		Heater2ON =       (SPIIn[3] >> 1) % 2;
		Heater3ON =       (SPIIn[3] >> 2) % 2;
		Heater4ON =       (SPIIn[3] >> 3) % 2;
		Heater1PV = SPIIn[4];
		Heater2PV = SPIIn[5];
		Heater3PV = SPIIn[6];
		Heater4PV = SPIIn[7];
		HeaterNoConnection = SPIIn[8];
		SPICounter = 0;
		SPICounterSkip = TRUE;
		SPIDataLock = FALSE;
	}

	if(SPICounterSkip)
		SPICounterSkip = FALSE;
	else
		SPICounter++;

	SPITransmitted = FALSE;
}

//*****************************************************************************
// Interrupt handler for the SSI1 interrupt
//*****************************************************************************
/*
void IntSSI1(void)
{
	SSIIntClear(SSI1_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );
	SSIDataGetNonBlocking(SSI1_BASE, &SPIIn2[SPICounter2]);//SPIIn[SPICounter] = SPDR;
	//PORTSS = TRUE;
	switch(SPICounter2)
	{
		case 0:
			if(SPIIn2[0] != 0xF0)
			{
				SPICounter2 = 0;
				SPICounterSkip2 = TRUE;
				SPIComErrorFlag2 = TRUE;
			}
			break;
		case 1:
			if(SPIIn2[1] != 0x64)
			{
				SPICounter2 = 0;
				SPICounterSkip2 = TRUE;
				SPIComErrorFlag2 = TRUE;
			}
			break;
		case 2:
			if(SPIIn2[2] != 0x10)
			{
				SPICounter2 = 0;
				SPICounterSkip2 = TRUE;
				SPIComErrorFlag2 = TRUE;
			}
			break;
		case 24:
			if(SPIIn2[24] != 0x0F)
			{
				SPICounter2 = 0;
				SPICounterSkip2 = TRUE;
				SPIComErrorFlag2 = TRUE;
			}
			break;
		case 25:
			if(SPIIn2[25] != 0x0F)
			{
				SPICounter2 = 0;
				SPICounterSkip2 = TRUE;
				SPIComErrorFlag2 = TRUE;
			}
	}

	if(SPICounter2 >= 25)                              // SPI Communication has read all the frame successfully
	{
		SPIComErrorFlag2 = FALSE;
		S2DdrA = SPIIn2[3];
		S2DdrB = SPIIn2[4];
		S2DdrC = SPIIn2[5];
		S2DdrD = SPIIn2[6];
		S2DdrE = SPIIn2[7];
		S2DdrF = SPIIn2[8];
		S2DdrG = SPIIn2[9];
		S2DdrH = SPIIn2[10];
		S2DdrI = SPIIn2[11];
		S2DdrJ = SPIIn2[12];
		S2PortA = SPIIn2[13];
		S2PortB = SPIIn2[14];
		S2PortC = SPIIn2[15];
		S2PortD = SPIIn2[16];
		S2PortE = SPIIn2[17];
		S2PortF = SPIIn2[18];
		S2PortG = SPIIn2[19];
		S2PortH = SPIIn2[20];
		S2PortI = SPIIn2[21];
		S2PortJ = SPIIn2[22];
		SPICounter2 = 0;
		SPICounterSkip2 = TRUE;
		//SPIDataLock = FALSE;
	}

	if(SPICounterSkip2)
		SPICounterSkip2 = FALSE;
	else
		SPICounter2++;

	SPITransmitted2 = FALSE;
}
*/

//*****************************************************************************
// Interrupt handler for the UART0 interrupt
//*****************************************************************************
void UART0IntHandler(void)
{
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = ROM_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART0_BASE, ulStatus);

    // Loop while there are characters in the receive FIFO.
    if( (ulStatus & UART_INT_RX) == UART_INT_RX )
    {
		if(ROM_UARTCharsAvail(UART0_BASE))
		{
			if(!ReadACK)
			{
				ComIN[dataRXIndex] = ROM_UARTCharGetNonBlocking(UART0_BASE);
	            switch(dataRXIndex)
	            {
	                case 0:                                         // 0x0F
	                    break;
	                case 1:             // station number
	                    if(ComIN[1] != 0x01)
	                        dataRXIndex = 0;
	                    break;
	                case 2:             // function number
	                    if(ComIN[2] != 0x03)
	                        dataRXIndex = 0;
	                    break;
	                case 3:
	                    ComAddress = ComIN[3];
	                    break;
	                case 4:
	                    BytestoRead = ComIN[4];
	                    break;
	                case 5:             // Vibrator Counter Setting
	                    if(ComAddress == 1)
	                    {
	                        //ServiceMode = FALSE;
	                        MotorONBtn =        ComIN[5] % 2;
	                        MotorOFFBtn =       (ComIN[5] >> 1) % 2;
	                        ServiceMode =       (ComIN[5] >> 2) % 2;
	                        EyemarkEN =         (ComIN[5] >> 3) % 2;
	                        DatecodeEN =        (ComIN[5] >> 4) % 2;
	                        CounterCutterEN =   (ComIN[5] >> 5) % 2;
	                        ToggleAuto =        (ComIN[5] >> 6) % 2;
	                        CounterEyemark =    (ComIN[5] >> 7) % 2;
	                    }
	                    break;
	                case 6:
	                    if(ComAddress == 1)
	                    {
	                        //ServiceMode = TRUE;
	                        //if(ServiceMode)
	                        if(!MachineRunning)
	                    	{
	                            DatecodeON =        ComIN[6] % 2;
	                            VibratorFormerON =  (ComIN[6] >> 1) % 2;                                          // Vibrator ON/OFF
	                            CounterCutterON =   (ComIN[6] >> 2) % 2;
	                            CheckPulseCom =     (ComIN[6] >> 3) % 2;
	                            if(CheckPulseCom && (!CheckPulseMode) )
	                            {
	                                Initialised = FALSE;
	                                CheckPulseMode = TRUE;
	                                MotorON = TRUE;
	                            }
	                            ClutchON =          (ComIN[6] >> 4) % 2;
	                            BreakON =           (ComIN[6] >> 5) % 2;
	                        }
	                        ContinuousMode = (ComIN[6] >> 6) % 2;
	                        SpeedModeSelector = (ComIN[6] >> 7) % 2;
	                    }
	                    break;
	                case 7:
	                    if(ComAddress == 1)
	                    {
	                        if(ComIN[7] > 0 && ComIN[7] <= 64)
	                            VibratorCounter = ComIN[7] + 79;
	                        else if( ComIN[7] >= 65 && ComIN[7] <= 125 )
	                            VibratorCounter = ComIN[7] - 64;
	                        else
	                            VibratorCounter = 0;
	                    }
	                    break;
	                case 8:
	                    if(ComAddress == 1)
	                    {
	                        if(ComIN[8])
	                            CounterCutterSetting = ComIN[8] - 1;
	                    }
	                    break;
	                case 9:
	                    if(ComAddress == 1)
	                    {
	                        ComDACSpeed = ComIN[9];
	                    }
	                    break;
	                case 10:
	                    if(ComAddress == 1)
	                    {
	                        StopAt = ComIN[10];
	                    }
	                    break;
	                case 11:
	                    if(ComAddress == 1)
	                    {
	                        InitialSpeed = ComIN[11];
	                    }
	                    break;
	                case 12:
	                    if(ComAddress == 1)
	                    {
	                        Heater1SV = ComIN[12];
	                        if(!Heater1SV)          // If heater value is 0, make it 25 (because 0 in this case means heater putus)
	                            Heater1SV = 25;
	                    }
	                    break;
	                case 13:
	                    if(ComAddress == 1)
	                    {
	                        Heater2SV = ComIN[13];
	                        if(!Heater2SV)          // If heater value is 0, make it 25 (because 0 in this case means heater putus)
	                            Heater2SV = 25;
	                    }
	                    break;
	                case 14:
	                    if(ComAddress == 1)
	                    {
	                        Heater3SV = ComIN[14];
	                        if(!Heater3SV)          // If heater value is 0, make it 25 (because 0 in this case means heater putus)
	                            Heater3SV = 25;
	                    }
	                    break;
	                case 15:
	                    if(ComAddress == 1)
	                    {
	                        Heater4SV = ComIN[15];
	                        if(!Heater4SV)          // If heater value is 0, make it 25 (because 0 in this case means heater putus)
	                            Heater4SV = 25;
	                    }
	                    break;
	                case 16:
	                    if(ComAddress == 1)
	                    {
	                        Heater1HLimit = ComIN[16];
	                    }
	                    break;
	                case 17:
	                    if(ComAddress == 1)
	                    {
	                        Heater2HLimit = ComIN[17];
	                    }
	                    break;
	                case 18:
	                    if(ComAddress == 1)
	                    {
	                        Heater3HLimit = ComIN[18];
	                    }
	                    break;
	                case 19:
	                    if(ComAddress == 1)
	                    {
	                        Heater4HLimit = ComIN[19];
	                    }
	                    break;
	                case 20:
	                    if(ComAddress == 1)
	                    {
	                        Heater1LLimit = ComIN[20];
	                    }
	                    break;
	                case 21:
	                    if(ComAddress == 1)
	                    {
	                        Heater2LLimit = ComIN[21];
	                    }
	                    break;
	                case 22:
	                    if(ComAddress == 1)
	                    {
	                        Heater3LLimit = ComIN[22];
	                    }
	                    break;
	                case 23:
	                    if(ComAddress == 1)
	                    {
	                        Heater4LLimit = ComIN[23];
	                    }
	                    break;
	                case 24:
	                    if(ComAddress == 1)
	                    {
	                        TROPosition = ComIN[24];                    // v1.1 ComIN[24] + 60
	                    }
	                    break;
	                case 25:
	                    if(ComAddress == 1)
	                    {
	                        TRIDelay = ((unsigned int)ComIN[25]) * 10;
	                    }
	                    break;
	                case 27:
	                    if(ComAddress == 1)
	                    {
	                        Offset[0] = (ComIN[26] << 8) + ComIN[27];
	                    }
	                    break;
	                case 29:
	                    if(ComAddress == 1)
	                    {
	                        Offset[1] = (ComIN[28] << 8) + ComIN[29];
	                    }
	                    break;
	                case 31:
	                    if(ComAddress == 1)
	                    {
	                        Offset[2] = (ComIN[30] << 8) + ComIN[31];
	                    }
	                    break;
	                case 33:
	                    if(ComAddress == 1)
	                    {
	                        Offset[3] = (ComIN[32] << 8) + ComIN[33];
	                    }
	                    break;
	                case 35:
	                    if(ComAddress == 1)
	                    {
	                        Gradient[0] = (ComIN[34] << 8) + ComIN[35];
	                    }
	                    break;
	                case 37:
	                    if(ComAddress == 1)
	                    {
	                        Gradient[1] = (ComIN[36] << 8) + ComIN[37];
	                    }
	                    break;
	                case 39:
	                    if(ComAddress == 1)
	                    {
	                        Gradient[2] = (ComIN[38] << 8) + ComIN[39];
	                    }
	                    break;
	                case 41:
	                    if(ComAddress == 1)
	                    {
	                        Gradient[3] = (ComIN[40] << 8) + ComIN[41];
	                    }
	                    break;
	                case 42:
	                    if(ComAddress == 1)
	                    {
	                        Heater1Enable = ComIN[42] % 2;
	                        Heater2Enable = (ComIN[42] >> 1) % 2;                                          // Vibrator ON/OFF
	                        Heater3Enable = (ComIN[42] >> 2) % 2;
	                        Heater4Enable = (ComIN[42] >> 3) % 2;

	                        if( LastPulseManualON != ((ComIN[42] >> 4) % 2 ) )
	                        	PulseManualON = (ComIN[42] >> 4) % 2;
	                        LastPulseManualON = (ComIN[42] >> 4) % 2;

	                        ServoAlmRst   = (ComIN[42] >> 5) % 2;
	                        if(!MachineRunning)
	                            VlvBlowON     = (ComIN[42] >> 6) % 2;
	                        ServoDirection  = (ComIN[42] >> 7) % 2;
	                        if(ServoBacklashMode)
	                        {
	            				if(!ServoDirection)         // backward
	            					ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
	            				else
	            					ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
	                        }
	                        else
	                        {
	            				if(ServoDirection)         // back to forward
	            					ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
	            				else
	            					ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
	                        }

	                    }
	                    break;
	                case 44:
	                    if(ComAddress == 1)
	                    {
	                        PulseManual = (ComIN[43] << 8) + ComIN[44];
	                    }
	                    break;
	                case 46:
	                    if(ComAddress == 1)
	                    {
	                        //ICR1 = (ComIN[45] << 8) + ComIN[46];
	                        //ICR1 = konstant / ((ComIN[45] << 8) + ComIN[46]);
	                    	ServoSpeedRPM = (ComIN[45] << 8) + ComIN[46];
	                        ServoSpeed = konstant / ServoSpeedRPM;
	                        if(ServoSpeed > 32767)
	                        	ServoSpeed = 32767;					// limit 15 bit
	                    }
	                    break;
	                case 47:
	                    if(ComAddress == 1)
	                    {
	                        CounterCutterCounterCom = ComIN[47];
	                        if(CounterCutterCounterCom)
	                            CounterCutterCounter = CounterCutterCounterCom;
	                    }
	                    break;
	                case 48:
	                    if(ComAddress == 1)
	                    {
	                        VlvBlowPos = ComIN[48];
	                    }
	                    break;
	                case 50:
	                    if(ComAddress == 1)
	                    {
	                        VlvBlowTimer = (ComIN[49] << 8) + ComIN[50];
	                    }
	                    break;
	                case 51:
	                    if(ComAddress == 1)
	                    {
	                        EyemarkIgnore = ComIN[51];
	                    }
	                    break;
	                case 52:
	                    if(ComAddress == 1)
	                    {
	                        BrakePos = ComIN[52];
	                    }
	                    break;
	                case 53:
	                    if(ComAddress == 1)
	                    {
	                        CounterCutterPos = ComIN[53];
	                    }
	                    break;
	                case 54:
	                    if(ComAddress == 1)
	                    {
	                        ServoEnable = ComIN[54] % 2;
	                        GussetEnable = (ComIN[54] >> 1) % 2;
	                        RewindEnable = (ComIN[54] >> 2) % 2;
	                        IgnoreAirError = (ComIN[54] >> 3) % 2;
	                        DatesensEN = (ComIN[54] >> 4) % 2;
	                        TROManual = (ComIN[54] >> 5) % 2;
	                        if(!MachineRunning)
	                        {
	                            if(TROManual)
	                                TROMinCounter = TROMin;
	                        }
	                        AgitatorEnable = (ComIN[54] >> 6) % 2;
	                        AirCutterEnable =  (ComIN[54] >> 7) % 2;	// GasEnable

	                    }
	                    break;
	                case 55:
	                    if(ComAddress == 1)
	                    {
	                        ServoStartPos = ComIN[55];
	                    }
	                    break;
	                case 57:
	                    if(ComAddress == 1)
	                    {
	                        ServoSignalDly = (ComIN[56] << 8) + ComIN[57];
	                    }
	                    break;
	                case 58:
	                    if(ComAddress == 1)
	                    {
	                        ExternalFeederOFFTimer = ComIN[58]; // Was TROTrail
	                    }
	                    break;
	                case 59:
	                    if(ComAddress == 1)
	                    {
	                        TROMin = ComIN[59];
	                    }
	                    break;
	                case 60:
	                    if(ComAddress == 1)
	                    {
	                        AirCutterPos = ComIN[60];
	                    }
	                    break;
	                case 61:
	                    if(ComAddress == 1)
	                    {
	                        AirCutterTimer = ComIN[61];
	                    }
	                    break;
	                case 62:
	                    if(ComAddress == 1)
	                    {
	                        /*if(AirCutterEnable)
	                        {
	                            AirCutterON = ComIN[62] % 2;
	                            if(ServiceMode)
	                                PORTGAS = AirCutterON;
	                        }*/
	                        StopOnBtn = (ComIN[62] >> 1) % 2;
	                        ServoBacklashEnable = (ComIN[62] >> 2) % 2;
	                        if(LastBacklashON != ((ComIN[62] >> 3) % 2))
	                           BacklashON = (ComIN[62] >> 3) % 2;
	                        LastBacklashON = (ComIN[62] >> 3) % 2;

	                        AgitatorManual = (ComIN[62] >> 4) % 2;

	                        if(!MachineRunning)
	                        {
	                            ServoDrain = (ComIN[62] >> 5) % 2;
	                            if( (LastServoDrain != ServoDrain) && (ServoDrain == FALSE) )
	                            {
	                                PulseManualCounter = 0;
	                                PulseON = FALSE;
	                                ServoDrainFlag = FALSE;
	                                if(ServoAccDccEnable)
	                                	ServoTargetSpeed = 32000;					// To reset back accel decel module
	                            }
	                            LastServoDrain = ServoDrain;
	                        }
	                        ExCycle = (ComIN[62] >> 6) % 2;
	                        VibratorMode = (ComIN[62] >> 7) % 2;
	                    }
	                    break;
	                case 64:
	                    if(ComAddress == 1)
	                    {
	                        ServoBacklashPulse = (ComIN[63] << 8) + ComIN[64];
	                    }
	                    break;
	                case 66:
	                    if(ComAddress == 1)
	                    {
	                        //ICR1 = (ComIN[45] << 8) + ComIN[46];
	                        ServoBacklashSpeed = konstant / ((ComIN[65] << 8) + ComIN[66]);
	                    }
	                    break;
	                case 67:
	                    if(ComAddress == 1)
	                    {
	                        ServoBacklashTimer = ComIN[67];
	                    }
	                    break;
	                case 68:
	                    if(ComAddress == 1)
	                    {
	                        GasStartPos = ComIN[68];
	                    }
	                    break;
	                case 69:
	                    if(ComAddress == 1)
	                    {
	                        GasEndPos = ComIN[69];
	                    }
	                    break;
	                case 70:
	                    if(ComAddress == 1)
	                    {
	                        StandbyAt = ComIN[70];
	                    }
	                    break;
	                case 71:
	                    if(ComAddress == 1)
	                    {
	                    	OUTEX1Enable = ComIN[71] % 2;
	                    	OUTEX2Enable = (ComIN[71] >> 1) % 2;
	                    	OUTEX3Enable = (ComIN[71] >> 2) % 2;
	                    	OUTEX4Enable = (ComIN[71] >> 3) % 2;
	                    	OUTEX01Enable = (ComIN[71] >> 4) % 2;
	                    	OUTEX02Enable = (ComIN[71] >> 5) % 2;
	                    	OUTEX03Enable = (ComIN[71] >> 6) % 2;
	                    	ServoValveEnable = (ComIN[71] >> 7) % 2;		// using Prox 1
	                    }
	                    break;
	                case 72:
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {
		                    	OUTEX1ON = ComIN[72] % 2;
		                    	OUTEX2ON = (ComIN[72] >> 1) % 2;
		                    	OUTEX3ON = (ComIN[72] >> 2) % 2;
		                    	OUTEX4ON = (ComIN[72] >> 3) % 2;
		                    	OUTEX01ON = (ComIN[72] >> 4) % 2;
		                    	OUTEX02ON = (ComIN[72] >> 5) % 2;
		                    	OUTEX03ON = (ComIN[72] >> 6) % 2;
	                        }
	                        ExternalFeederEnable = (ComIN[72] >> 7) % 2;
	                    }
	                    break;
	                case 73:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX1StartPos = ComIN[73];
	                    }
	                    break;
	                case 74:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX1EndPos = ComIN[74];
	                    }
	                    break;
	                case 75:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX2StartPos = ComIN[75];
	                    }
	                    break;
	                case 76:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX2EndPos = ComIN[76];
	                    }
	                    break;
	                case 77:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX3StartPos = ComIN[77];
	                    }
	                    break;
	                case 78:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX3EndPos = ComIN[78];
	                    }
	                    break;
	                case 79:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX4StartPos = ComIN[79];
	                    }
	                    break;
	                case 80:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX4EndPos = ComIN[80];
	                    }
	                    break;
	                case 81:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX01StartPos = ComIN[81];
	                    }
	                    break;
	                case 82:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX01EndPos = ComIN[82];
	                    }
	                    break;
	                case 83:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX02StartPos = ComIN[83];
	                    }
	                    break;
	                case 84:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX02EndPos = ComIN[84];
	                    }
	                    break;
	                case 85:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX03StartPos = ComIN[85];
	                    }
	                    break;
	                case 86:
	                    if(ComAddress == 1)
	                    {
	                        OUTEX03EndPos = ComIN[86];
	                    }
	                    break;
	                case 87:
	                    if(ComAddress == 1)
	                    {
	                        DatecodeStartPos = ComIN[87];
	                    }
	                    break;
	                case 88:
	                    if(ComAddress == 1)
	                    {
	                        DatecodeONDuration = ComIN[88];
	                    }
	                    break;
	                case 89:	// case 89 - 97 is for Direct PORTOUT  PORTA = 89, PORTJ = 97
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {

	                        }
	                    }
	                    break;
	                case 90:	// PORTB
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {

	                        }
	                    }
	                    break;
	                case 91:	// PORTC
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {
	                        	if((ComIN[91] >> 4) % 2)
	                    			ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, PORT_GAS_PIN);
	                    		else
	                    			ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, 0);

	                        	VibratorFormerON = (ComIN[91] >> 6) % 2;
	                        }
	                    }
	                    break;
	                case 97:	// PORTJ
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {
	                            if(ExternalFeederManualEnable)
	                            {
	                            	ExternalFeederON = (ComIN[97] >> 1) % 2;
	                            	ExternalFeederDurationCounter = 0;
	                            }
	                        }
	                    }
	                    break;
	                case 98:
	                    if(ComAddress == 1)
	                    {
	                    	AirCutterMode = ComIN[98] % 2;
	                    }
	                    break;
	                case 99:
	                    if(ComAddress == 1)
	                    {
	                    	ServoValveDelay = ComIN[99];
	                    }
	                    break;
	                case 100:
	                    if(ComAddress == 1)
	                    {
	                    	ExternalFeederDuration = ComIN[100];
	                    }
	                    break;
	                case 101:
	                    if(ComAddress == 1)
	                    {
	                    	HopperBufferSensor = ComIN[101];
	                    }
	                    break;
	                case 102:
	                    if(ComAddress == 1)
	                    {
	                    	ExternalFeederManualEnable = 	ComIN[102] % 2;
	                    	ServoValveSensorEnable = 		(ComIN[102] >> 1) % 2;
	                    	HopperSensorEnable = 			(ComIN[102] >> 2) % 2;
	                    	ExternalFeederAgitatorEnable =  (ComIN[102] >> 3) % 2;
	                    	ServoAccDccEnable			 =  (ComIN[102] >> 4) % 2;
	                    	IsMBTITMHMI					 =  (ComIN[102] >> 5) % 2;
	                    	ExternalFeederTimerMode		 =  (ComIN[102] >> 6) % 2;
	                    }
	                    break;
	                case 103:
	                    if(ComAddress == 1)
	                    {
	                    	if(ComIN[103])
	                    		ServoValveHeadstartVar = ComIN[103];
	                    	else
	                    		ServoValveHeadstartVar = 15;				// If the touchscreen is from previous version, put default of 15
	                    }
	                    break;
	                case 105:
	                	if(ComAddress == 1)
	                	{
	                		if(!((ComIN[104] << 8) + ComIN[105]) )	// if servodrainspeed = 0 (because of older version)
	                			ServoDrainSpeed = ServoSpeed;
	                		else
	                		{
								ServoDrainSpeed = konstant / ((ComIN[104] << 8) + ComIN[105]);
								if(ServoDrainSpeed > 32767)
									ServoDrainSpeed = 32767;					// limit 15 bit
	                		}
	                	}
	                	break;
	                case 106:
	                    if(ComAddress == 1)
	                    {
	                    	if(ComIN[106])						// if HMI is previous version then ServoAccelRate = 30
	                    		ServoAccelRate = ComIN[106];
	                    	else
	                    		ServoAccelRate = 30;
	                    }
	                    break;
	                case 107:
	                    if(ComAddress == 1)
	                    {
	                    	PIDmultiplier = ComIN[107] / 10;
	                    	if(PIDmultiplier < 3)
	                    		PIDmultiplier = 3;
	                    	else if(PIDmultiplier > 15)
	                    		PIDmultiplier = 15;
	                    }
	                    break;
	            }
				if(dataRXIndex == 109)
				{
					UARTTimeOut = UARTDISABLEDCOUNT;
                    ComAddress = ComIN[3];
                    BytestoRead = ComIN[4];
					ReadACK = TRUE;
					dataTXIndex = ComAddress * 20;
					dataRXIndex = 0;
	                //ROM_UARTCharPutNonBlocking(UART0_BASE, ComOUT[dataTXIndex]);
	                //dataTXIndex++;
					IntRXCounter++;
					ComFlag = TRUE;                         // Have communicated with TouchScreen Successfully
				}

				if( (ComIN[0] != 0xF0) || (ReadACK == TRUE) )
				{
					dataRXIndex = 0;
					//debugcounter1++;
				}
				else
					dataRXIndex++;
			}
		}
    }


    // Interrupt Transmit Handler
    if( (ulStatus & UART_INT_TX) == UART_INT_TX )
    {
    }
}

//*****************************************************************************
// Interrupt handler for the PORTC interrupt
//*****************************************************************************
void IntGPIOc(void)
{
	unsigned long ulStatus;
	ulStatus = GPIOIntStatus(GPIO_PORTC_BASE, TRUE);
	GPIOIntClear(GPIO_PORTC_BASE, ulStatus);

	if( (ulStatus & PIN_PULSE_A_PIN) == PIN_PULSE_A_PIN)
	{
		SpeedperMinEncoder++;
		// For Eyemark Ignore Function
	    if(EyemarkIgnoreCounter && ClutchON)
	        EyemarkIgnoreCounter--;

		EncoderBuffer = ROM_QEIPositionGet(QEI1_BASE);
		if(EncoderBuffer < 0)
			EncoderData = 0 - EncoderBuffer;
		else
			EncoderData = EncoderBuffer;

	    if(Initialised && MachineRunning)
	    {
			switch(EncoderData)
			{
				case 0:                 // When Zero
					OCycEyemark = FALSE;
					TROOnce = FALSE;
					BreakON = FALSE;
					ClutchON = TRUE;
					CounterCutterON = FALSE;
					CutterON = FALSE;                  // CounterEyemark OFF, Eyemark Error Increment Latch Reset
					//DatecodeON = FALSE;
					//TROLaunched = FALSE;                // Reset to initial state
					//TRIReceived = FALSE;                // Reset to initial state
					break;
				case ((ENCPULSE * 17) / 36):                // Safety Break, Vibrator Start
					if(VibratorCounter > 0)
						VibratorFormerON = TRUE;
					break;
				//case ((ENCPULSE * 25) / 36):               // Datecode
				//        if(DatecodeEN == TRUE)
				//            DatecodeON = TRUE;
				//	break;
				case ((ENCPULSE * 35) / 36):
					if(CounterEyemark == FALSE)         // CounterEyemark OFF, Eyemark Error Increment Latch Reset
						ProductCountBitCounter = 350;
					break;
				case ENCPULSE:
					OCycEyemark = FALSE;
					TROOnce = FALSE;
					BreakON = FALSE;
					ClutchON = TRUE;
					CounterCutterON = FALSE;
					CutterON = FALSE;                  // CounterEyemark OFF, Eyemark Error Increment Latch Reset
					//DatecodeON = FALSE;
					ZeroIndexCounter++;
					EncoderData = 0;
					EncoderBuffer = 0;
					break;

			}

			if(DatecodeEN)
			{
				if(EncoderData == DatecodeStartPos)
					DatecodeONDurationCounter = DatecodeONDuration;
			}
			if(AirCutterEnable && (!AirCutterMode))		// Gas ON
			{
				if(EncoderData == GasEndPos)
					ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, 0);
				if(EncoderData == GasStartPos)
					ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, PORT_GAS_PIN);
			}
			if(OUTEX1Enable)
			{
				if(EncoderData == OUTEX1EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, 0);
				}
				if(EncoderData == OUTEX1StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, PORT_EX1_PIN);
				}
			}
			if(OUTEX2Enable)
			{
				if(EncoderData == OUTEX2EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, 0);
				}
				if(EncoderData == OUTEX2StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, PORT_EX2_PIN);
				}
			}
			if(OUTEX3Enable)
			{
				if(EncoderData == OUTEX3EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, 0);
				}
				if(EncoderData == OUTEX3StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, PORT_EX3_PIN);
				}
			}
			if(OUTEX4Enable)
			{
				if(EncoderData == OUTEX4EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);
				}
				if(EncoderData == OUTEX4StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);
				}
			}
			if(EncoderData == BrakePos)
			{
				if(EyemarkEN && ClutchON)                       // Reach Safety Brake even though eyemark is ON (Eyemark scan fails)
					EyemarkErrorCounter++;

				ClutchON = FALSE;
				BreakON = TRUE;
			}
			if(EncoderData == CounterCutterPos)             // Counter Cutter
			{
				if(CounterCutterEN == TRUE)
				{
					if(CounterCutterCounter == 0)
					{
						CounterCutterON = TRUE;
						CounterCutterCounter = CounterCutterSetting;
					}
					else
						CounterCutterCounter--;
				}
			}
			if(EncoderData == StandbyAt)
			{
				if( (TROLaunched) || (TRIReceived && (TRIDelayTimer > 1)) || (ServoSignalDlyCount) )                     // if not TRI hasnt been received during pos 90, stop the motor and standby
				{
					MotorON = FALSE;
					StandbyMode = TRUE;
				}
			}
			if(EncoderData == VibratorCounter)
			{
				if(VibratorCounter > 0)
					VibratorFormerON = FALSE;
			}
			if(GussetEnable)
			{
				if(EncoderData == VlvBlowPos)
				{
					VlvBlowTimerCounter = VlvBlowTimer;
				}
			}
			if((EncoderData == ServoStartPos) && ServoEnable)  	// if ServoValveEnabled, Pin PROX2 is used for ServoValveSensor
			{
				if(StopOnBtn)
				{
					if(!StoppingMode)
					{
						if(ServoValveEnable && ServoValveSensorEnable)
						{
							if(!ROM_GPIOPinRead(PIN_PROX2_BASE, PIN_PROX2_PIN))
							{
								RunServo();
							}
							else
							{
								// Put Error Stopper Here
								ServoValveErrorMsgDuration = 80;
								ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);  // Close the valve if pneu not open
							}
						}
						else
						{
							RunServo();
						}
					}
				}
				else
				{
					if(ServoValveEnable && ServoValveSensorEnable)
					{
						if(!ROM_GPIOPinRead(PIN_PROX2_BASE, PIN_PROX2_PIN))
						{
							RunServo();
						}
						else
						{
							// Put Error Stopper Here
							ServoValveErrorMsgDuration = 80;
							ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);  // Close the valve if pneu not open
						}
					}
					else
					{
						RunServo();
					}
				}
			}
			if(!ContinuousMode)                        // if on standby mode  ,  not continuous mode
			{
				if( (EncoderData == TROPosition) && (!TROOnce) )
				{
					TROLaunched = TRUE;
					TROMinCounter = TROMin;
					ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, PORT_TRO1_PIN);//PORTTRO = TRUE;
					TRIReceived = FALSE;
					TROCoilCounter = 600;
					TROOnce = TRUE;
				}
			}
	    }

		if(StopCommand)
		{
			if(EncoderData == StopAt)
			{
				MotorON = FALSE;
				CNBStopTimerFlag = TRUE;
			}
		}
	}

	if( (ulStatus & PIN_PULSE_B_PIN) == PIN_PULSE_B_PIN)
	{
		SpeedperMinEncoder++;
		// For Eyemark Ignore Function
	    if(EyemarkIgnoreCounter && ClutchON)
	        EyemarkIgnoreCounter--;

		EncoderBuffer = ROM_QEIPositionGet(QEI1_BASE);
		if(EncoderBuffer < 0)
			EncoderData = 0 - EncoderBuffer;
		else
			EncoderData = EncoderBuffer;

	    if(Initialised && MachineRunning)
	    {
			switch(EncoderData)
			{
				case 0:                 // When Zero
					OCycEyemark = FALSE;
					TROOnce = FALSE;
					BreakON = FALSE;
					ClutchON = TRUE;
					CounterCutterON = FALSE;
					CutterON = FALSE;                  // CounterEyemark OFF, Eyemark Error Increment Latch Reset
					//DatecodeON = FALSE;
					//TROLaunched = FALSE;                // Reset to initial state
					//TRIReceived = FALSE;                // Reset to initial state
					break;
				case ((ENCPULSE * 17) / 36):                // Safety Break, Vibrator Start
					if(VibratorCounter > 0)
						VibratorFormerON = TRUE;
					break;
				//case ((ENCPULSE * 25) / 36):               // Datecode
				//        if(DatecodeEN == TRUE)
				//            DatecodeON = TRUE;
				//	break;
				case ((ENCPULSE * 35) / 36):
					if(CounterEyemark == FALSE)         // CounterEyemark OFF, Eyemark Error Increment Latch Reset
						ProductCountBitCounter = 350;
					break;
				case ENCPULSE:
					OCycEyemark = FALSE;
					TROOnce = FALSE;
					BreakON = FALSE;
					ClutchON = TRUE;
					CounterCutterON = FALSE;
					CutterON = FALSE;                  // CounterEyemark OFF, Eyemark Error Increment Latch Reset
					//DatecodeON = FALSE;
					ZeroIndexCounter++;
					EncoderData = 0;
					EncoderBuffer = 0;
					break;

			}

			if(DatecodeEN)
			{
				if(EncoderData == DatecodeStartPos)
					DatecodeONDurationCounter = DatecodeONDuration;
			}
			if(AirCutterEnable && (!AirCutterMode))		// Gas ON
			{
				if(EncoderData == GasEndPos)
					ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, 0);
				if(EncoderData == GasStartPos)
					ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, PORT_GAS_PIN);
			}
			if(OUTEX1Enable)
			{
				if(EncoderData == OUTEX1EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, 0);
				}
				if(EncoderData == OUTEX1StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, PORT_EX1_PIN);
				}
			}
			if(OUTEX2Enable)
			{
				if(EncoderData == OUTEX2EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, 0);
				}
				if(EncoderData == OUTEX2StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, PORT_EX2_PIN);
				}
			}
			if(OUTEX3Enable)
			{
				if(EncoderData == OUTEX3EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, 0);
				}
				if(EncoderData == OUTEX3StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, PORT_EX3_PIN);
				}
			}
			if(OUTEX4Enable)
			{
				if(EncoderData == OUTEX4EndPos)
				{
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);
				}
				if(EncoderData == OUTEX4StartPos)
				{
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);
				}
			}
			if(EncoderData == BrakePos)
			{
				if(EyemarkEN && ClutchON)                       // Reach Safety Brake even though eyemark is ON (Eyemark scan fails)
					EyemarkErrorCounter++;

				ClutchON = FALSE;
				BreakON = TRUE;
			}
			if(EncoderData == CounterCutterPos)             // Counter Cutter
			{
				if(CounterCutterEN == TRUE)
				{
					if(CounterCutterCounter == 0)
					{
						CounterCutterON = TRUE;
						CounterCutterCounter = CounterCutterSetting;
					}
					else
						CounterCutterCounter--;
				}
			}
			if(EncoderData == StandbyAt)
			{
				if( (TROLaunched) || (TRIReceived && (TRIDelayTimer > 1)) || (ServoSignalDlyCount) )                     // if not TRI hasnt been received during pos 90, stop the motor and standby
				{
					MotorON = FALSE;
					StandbyMode = TRUE;
				}
			}
			if(EncoderData == VibratorCounter)
			{
				if(VibratorCounter > 0)
					VibratorFormerON = FALSE;
			}
			if(GussetEnable)
			{
				if(EncoderData == VlvBlowPos)
				{
					VlvBlowTimerCounter = VlvBlowTimer;
				}
			}
			if((EncoderData == ServoStartPos) && ServoEnable)  	// if ServoValveEnabled, Pin PROX2 is used for ServoValveSensor
			{
				if(StopOnBtn)
				{
					if(!StoppingMode)
					{
						if(ServoValveEnable && ServoValveSensorEnable)
						{
							if(!ROM_GPIOPinRead(PIN_PROX2_BASE, PIN_PROX2_PIN))
							{
								RunServo();
							}
							else
							{
								// Put Error Stopper Here
								ServoValveErrorMsgDuration = 80;
								ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);  // Close the valve if pneu not open
							}
						}
						else
						{
							RunServo();
						}
					}
				}
				else
				{
					if(ServoValveEnable && ServoValveSensorEnable)
					{
						if(!ROM_GPIOPinRead(PIN_PROX2_BASE, PIN_PROX2_PIN))
						{
							RunServo();
						}
						else
						{
							// Put Error Stopper Here
							ServoValveErrorMsgDuration = 80;
							ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);  // Close the valve if pneu not open
						}
					}
					else
					{
						RunServo();
					}
				}
			}
			if(!ContinuousMode)                        // if on standby mode  ,  not continuous mode
			{
				if( (EncoderData == TROPosition) && (!TROOnce) )
				{
					TROLaunched = TRUE;
					TROMinCounter = TROMin;
					ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, PORT_TRO1_PIN);//PORTTRO = TRUE;
					TRIReceived = FALSE;
					TROCoilCounter = 600;
					TROOnce = TRUE;
				}
			}
	    }
		if(StopCommand)
		{
			if(EncoderData == StopAt)
			{
				MotorON = FALSE;
				CNBStopTimerFlag = TRUE;
			}
		}
	}

	if( (ulStatus & PIN_PULSE_Z_PIN) == PIN_PULSE_Z_PIN )
	{
		if(MotorON)
			Initialised = TRUE;

		ZCounter++;
	}

}

//*****************************************************************************
// Interrupt handler for the PORTD interrupt
//*****************************************************************************
void IntGPIOd(void)
{
	GPIOIntClear(PIN_EYEMARK_BASE, PIN_EYEMARK_PIN);

    //Eyemark catched
    if(!EyemarkIgnoreCounter)		// per 10 ms
    {
        EyemarkIgnoreCounter = EyemarkIgnore;

        if(Initialised && EyemarkEN && MachineRunning)
        {
            ClutchON = FALSE;
            BreakON = TRUE;
            EyemarkPosition = EncoderData;
        }
        EyemarkLampCounter = 350;
        EyemarkErrorCounter = 0;
        if(CounterEyemark)
        {
            if( (!CounterEyemarkIgnore) && MachineRunning && (!OCycEyemark) )                       // OCycEyemark -> Only counted once per cycle
            {
                ProductCountBitCounter = 250;               // Pulse Product Counter Bit for 200 ms
                CounterEyemarkIgnore = 300;
            }
        }
        OCycEyemark = TRUE;
    }
}

//*****************************************************************************
// Interrupt handler for the Timer0 interrupt
//*****************************************************************************
void Timer0IntHandler(void)
{
	char DeltaSpeed;
    //unsigned long ulPinStatus;

    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    LoopTime = LoopCycle;
    LoopCycle = 0;

    if(blinkFlag == TRUE)
    {
        if(blinkON != 0)
            blinkON--;
        else
        {
            blinkOFF = 1500;
            blinkFlag = FALSE;
        }

    }
    else
    {
        if(blinkOFF != 0)
            blinkOFF--;
        else
        {
            blinkON = 1500;
            blinkFlag = TRUE;
        }
    }
    /* Button Buffer v */
    if(!ROM_GPIOPinRead(PIN_BTN_STOP_BASE, PIN_BTN_STOP_PIN))
    {
    	if(StopButton)
    		StopButton--;
    }
    else
    	StopButton = BufferSensor;
    if(!ROM_GPIOPinRead(PIN_BTN_RUN_BASE, PIN_BTN_RUN_PIN))
    {
    	if(RunButton)
    		RunButton--;
    }
    else
    	RunButton = BufferSensor;

    if(!ROM_GPIOPinRead(PIN_GUSSET_BTN_VALVE_BASE, PIN_GUSSET_BTN_VALVE_PIN))
    {
    	if(!MachineRunning)
    	{
			if(GussetButton)
			{
				GussetButton--;
				if(!GussetButton)
				{
					if(VlvRollerON)
						VlvRollerON = FALSE;
					else
						VlvRollerON = TRUE;
				}
			}
    	}
    }
    else
    	GussetButton = LongBufferSensor;

    /* Button Buffer ^ */

	if(ProductCountBitCounter)
	{
		ProductCountBit = TRUE;
		ProductCountBitCounter--;
	}
	else
		ProductCountBit = FALSE;

	/* Datecode */
	if(DatecodeONDurationCounter)
	{
		DatecodeONDurationCounter--;
		DatecodeON = TRUE;
	}
	else
		DatecodeON = FALSE;

	/* Counter for TIMER */                                          // SETTING : Change Timer value HERE
	if(CNBStopTimerFlag)
	{
		CNBStopTimer++;
		if(FatalError)
			CNBStopTimer = CNBStopTimer + 2;                        // If fatal error, timer runs 3x faster (3s -> 1s)
		if(CNBStopTimer > 800)
		{
			ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, 0);
            ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, 0);
            ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, 0);
            ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, 0);
            ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);
			ClutchON = FALSE;
			BreakON = FALSE;
			CutterON = FALSE;
			CounterCutterON = FALSE;
			VibratorFormerON = FALSE;
			CNBStopTimer = 0;
			CNBStopTimerFlag = FALSE;
			MachineRunning = FALSE;
			StopCommand = FALSE;                                // Re-enable the stop command
		}
	}

	/* Input Page v */
	// Eyemark : Lengthen the Eyemark indicator
	if(EyemarkLampCounter)
	{
		EyemarkLampCounter--;
		EyemarkON = TRUE;
	}
	else
		EyemarkON = FALSE;

	if(CounterEyemarkIgnore)
		CounterEyemarkIgnore--;
	// Foil Jam Input (Only activated when Rewinder module is not used)
	if(!RewindEnable)		// If rewind is enabled, foil jam is controlled by BACKREWINDER MODULE
	{
		if(!(ROM_GPIOPinRead(PIN_FOILJAM_BASE, PIN_FOILJAM_PIN)))                                               // If Foil Jam MS is ON
		{
			if(FoilJamCounter)
				FoilJamCounter--;
			else
	        {
	        	FoilJamMsgDuration = 80;
	            FoilJam = TRUE;
	        }
		}
		else
		{
			FoilJamCounter = 1000;
			FoilJam  = FALSE;
		}
	}
	// Input Page : PROX 2 (Must use SVE-MBTIu)
	if(DatesensEN)
	{
		if(ROM_GPIOPinRead(PIN_PROX1_BASE, PIN_PROX1_PIN))            // PINDATESENS if the sensor doesnt pick the film
		{
			if(DatesensCounter)
				DatesensCounter--;
			else
				DatesensError = TRUE;
		}
		else
		{
			DatesensCounter = 500;
			DatesensError = FALSE;
		}
	}
	else
	{
		DatesensCounter = 500;
		DatesensError = FALSE;
	}
	/* Input Page ^ */
	if(PulseCheckOK || PulseCheckError)
	{
		if(PulseCheckOKCounter > 0)
			PulseCheckOKCounter--;
		else
		{
			PulseCheckOK = FALSE;
			PulseCheckError = FALSE;
			PulseCheckOKCounter = 3000;
		}
	}

	/* Encoder Module v */
	if(MotorON)
	{
		// Pulse Error Module : Pulse Error Trigger
		if(LastEncoder == EncoderData)
		{
			if(PulseErrorCounter)
				PulseErrorCounter--;
			else
			{
				PulseErrorMsgDuration = 80;
				PulseError = TRUE;
			}
		}
		else
		{
			if(StopCommand)
				PulseErrorCounter = 1500;
			else
				PulseErrorCounter = 800;
			PulseError = FALSE;
		}
		LastEncoder = EncoderData;

		// Pulse Error Module : 5 secs reset for pulse error
		if(PulseError || PulseCommutationError)
		{
			if(PulseErrorTimerCounter)
				PulseErrorTimerCounter--;
			else
			{
				PulseError = FALSE;
				PulseCommutationError = FALSE;
			}
		}
		else
			PulseErrorTimerCounter = 5000;

		// Speed Search Module (Auto-sync with encoder)
		if(!(CheckPulseMode))                                       // If not in Check Pulse Mode
		{
			if(SpeedperMinCounter < 1999)
				SpeedperMinCounter++;
			else
			{
				//SpeedperMin = ROM_QEIVelocityGet(QEI1_BASE) * 60 / ENCPULSE; // Calculate every 1 sec
				SpeedperMin = (SpeedperMinEncoder * 30) / ENCPULSE; // Calculate every 1 sec

				if(SpeedperMin > ComDACSpeed)                               // if Speed is faster than the desired speed, lower it
				{
					DeltaSpeed = SpeedperMin - ComDACSpeed;
					if(DeltaSpeed <= 2)
						OffsetSpeed--;
					else if(DeltaSpeed <= 4)
						OffsetSpeed = OffsetSpeed - 3;
					else
						OffsetSpeed = OffsetSpeed - (DeltaSpeed * 2);
				}
				else if(SpeedperMin < ComDACSpeed)
				{
					DeltaSpeed = ComDACSpeed - SpeedperMin;
					if(DeltaSpeed <= 2)
						OffsetSpeed++;
					else if(DeltaSpeed <= 4)
						OffsetSpeed = OffsetSpeed + 3;
					else
						OffsetSpeed = OffsetSpeed + 7;
				}
				SpeedperMinEncoder = 0;
				SpeedperMinCounter = 0;
			}
		}
	}
	if(!MotorON)
	{
		PulseError = FALSE;
	}
	/* Encoder Module ^ */

	/* Communication Module v */
	if(ReadACK)                                                         // ReadACK time out 1500 ms
	{                                                                   // if doesnt transfer during 1500 ms then terminate transfer
		if(ReadACKCounter)
			ReadACKCounter--;
		else
		{
			DataLock = FALSE;
			ReadACK = FALSE;
			dataTXIndex = 0;
		}
	}
	else
		ReadACKCounter = 1000;

	if(!ReadACK)     				// if reading mode
	{
		UARTTimeOut--;
		if(!UARTTimeOut)
		{
		    // Disable the Uart to be re-enabled
		    UARTDisable(UART0_BASE);
		    UARTDisabled = TRUE;
		    UARTTimeOut = UARTDISABLEDCOUNT;
		}
	}
	else
		UARTTimeOut = UARTDISABLEDCOUNT;

    if(!SPIStartTimer)
    {
		if(SPIDataReady && (!SPITransmitted))
		{
			//PORTSS = FALSE;
			SSIDataPutNonBlocking(SSI0_BASE, SPIOut[SPICounter]);//SPDR = SPIOut[SPICounter];
			SPITransmitted = TRUE;
		}
    }
    else
        SPIStartTimer--;
	/* Communication Module ^ */

	/* Standby Module v */
	if(ROM_GPIOPinRead(PORT_TRO1_BASE, PORT_TRO1_PIN))
		TROCoilCounter = 600;

	if(!(ROM_GPIOPinRead(PIN_TRI1_BASE, PIN_TRI1_PIN)))                                // TriggerIN Came
	{
		if(TROLaunched)                         // TRI expected
		{

			TROTrailCounter = TROTrail;
			if( (!TROTrailCounter) && (!TROMinCounter) )
			{
                ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, 0);//PORTTRO = FALSE;
			}
			if ((EncoderData > BrakePos) || OCycEyemark ) // WARNING NEW 09/02/2012
			{
				ClutchON = FALSE;                   // WARNING NEW 26/01/2012
				BreakON = TRUE;                     // WARNING NEW 26/01/2012
			}
			else
			{
				BreakON = FALSE;                    // WARNING NEW 09/02/2012
				ClutchON = TRUE;                    // WARNING NEW 09/02/2012
			}
			TRICoilCounter = 600;
			TROLaunched = FALSE;
			StandbyMode = FALSE;
			if(FirstCycleFlag)
			{
				TRIDelayTimer = 1000;
				FirstCycleFlag = FALSE;
			}
			else
				TRIDelayTimer = TRIDelay + 1;
			TRIReceived = TRUE;
		}
		else                                    // TRI not expected -> error message maybe or warning?
		{
		}
	}

	if(TROCoilCounter)
	{
		TROCoil = TRUE;
		TROCoilCounter--;
	}
	else
		TROCoil = FALSE;

	if(TROMinCounter)
	{
        ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, PORT_TRO1_PIN);//PORTTRO = TRUE;
		TROMinCounter--;
		if( (!TROMinCounter) && (TRIReceived || (!MachineRunning)) && (!TROTrailCounter) )
		{
            ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, 0);//PORTTRO = FALSE;
		}
	}

	if(MotorONBtn || !RunButton)//if(!PINBTNRUN)                                 // Run Button pressed while on standby mode -> do one cycle
	{
		if(TROLaunched)                         // PINBTNRUN also act as self-trigger during standby mode
		{

            ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, 0);			// Put TRO OFF after got TRI

			if( (!ClutchON) && (!BreakON) )
			{
				if ((EncoderData > BrakePos) || OCycEyemark ) // WARNING NEW 09/02/2012
				{
					ClutchON = FALSE;                   // WARNING NEW 26/01/2012
					BreakON = TRUE;                     // WARNING NEW 26/01/2012
				}
				else
				{
					BreakON = FALSE;                    // WARNING NEW 09/02/2012
					ClutchON = TRUE;                    // WARNING NEW 09/02/2012
				}
			}
			TROLaunched = FALSE;
			StandbyMode = FALSE;
			if(FirstCycleFlag)
			{
				TRIDelayTimer = 1000;
				FirstCycleFlag = FALSE;
			}
			else
				TRIDelayTimer = TRIDelay + 1;
			TRIReceived = TRUE;
		}
		else                                    // PINBTNRUN do nothing outside standby mode (except the normal function)
		{
		}
	}

	if(TRIDelayTimer)
	{
		if((TRIDelayTimer == 1) && (ServoSignalDlyCount < 2))
			MotorON = TRUE;
		TRIDelayTimer--;
	}

	if(TRICoilCounter)
	{
		TRICoil = TRUE;
		TRICoilCounter--;
	}
	else
		TRICoil = FALSE;
	if(StandbyMode)
	{
		if(!StandbyModeCounter)
		{
			BreakON = FALSE;
			ClutchON = FALSE;
		}
		else
			StandbyModeCounter--;
	}
	else
		StandbyModeCounter = 3000;
	/* Standby Module ^ */

	if(MachineRunning)		//if(!ServiceMode)
	{
		if(VlvBlowTimerCounter)
		{
			VlvBlowON = TRUE;
			VlvBlowTimerCounter--;
		}
		else
			VlvBlowON = FALSE;

		if(AirCutterPosCounter)
		{
			AirCutterPosCounter--;
			if(!AirCutterPosCounter)
				AirCutterTimerCounter = AirCutterTimer;
		}

		if(AirCutterMode)
		{
			if(AirCutterTimerCounter)
			{
				ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, PORT_GAS_PIN);	//PORTGAS = TRUE;
				AirCutterTimerCounter--;
			}
			else
				ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, 0);
		}
		else
				AirCutterTimerCounter = 0;
	}

    /* Servo Module v */
	if(ServoEnable)
	{
		if( ROM_GPIOPinRead(PIN_SERVO_ALARM_BASE, PIN_SERVO_ALARM_PIN) )		// Active High
		{
			if(ServoErrorCounter)
				ServoErrorCounter--;

			if(!ServoErrorCounter)
				ServoError = TRUE;
		}
		else
		{
			ServoErrorCounter = 50;
			ServoError = FALSE;
		}

		if( ROM_GPIOPinRead(PIN_SERVO_READY_BASE, PIN_SERVO_READY_PIN) )
			ServoReady = FALSE;
		else
			ServoReady = TRUE;
	}
	else
	{
		ServoError = FALSE;
		ServoErrorCounter = 50;
		ServoReady = FALSE;
	}
    // Monitoring servo running duration
    if(LastPulseON != PulseON)
    {
        if(PulseON)
            ServoTimerCounter = 0;
    }
    LastPulseON = PulseON;

    if(PulseON)
        ServoTimerCounter++;

    // Servo Timing : Delaying the main motor to wait the servo till finished (set manually)
	if(ServoSignalDlyCount)
	{
		if((ServoSignalDlyCount == 1) && (TRIDelayTimer < 2))
			MotorON = TRUE;
		ServoSignalDlyCount--;
	}

	// Servo Backlash : When there is timer in between forward and backward motion
	if(ServoBacklashTimerCounter)
	{
		ServoBacklashTimerCounter--;
		if(!ServoBacklashTimerCounter)
		{
			if(ServoBacklashPulse)      // If backlash pulse is not 0
			{
				if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
				{
					PulseManualCounter = ServoBacklashPulse;        // fill pulse with backlash pulse
					if(!ServoDirection)         // backward
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
					else
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
					if(ServoAccDccEnable)
					{
						ServoTargetSpeed = ServoBacklashSpeed;
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
					}
					else
					{
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoBacklashSpeed);
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoBacklashSpeed >> 1);  // default 50%
					}
					PulseON = TRUE;
					ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);  //PulseOutputON
				}
			}
			else
			{
				ServoBacklashMode = FALSE;
				if(ServoAccDccEnable)
				{
					ServoTargetSpeed = ServoSpeed;
					ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
					ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
				}
				else
				{
					ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoSpeed);
					ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoSpeed >> 1);  // default 50%
				}
				if(ServoDirection)         // back to forward
					ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
				else
					ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
			}
		}
	}

	// Servo Valve Delay Module on ms10counter
	// Servo Valve Headstart
	if(MachineRunning && Initialised && ServoEnable && ServoValveEnable)
	{
		if(ServoStartPos > ServoValveHeadStart) // For example ServoStartPos 100, servoheadstart would be 90
		{
			if((EncoderData > ServoValveHeadStart) && (EncoderData < ServoStartPos))
			{
				ServoValveDelayCounter = 0;
				ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);			// Open the Valve
			}
		}
		else									// For example ServoStartPos 5, ServoValveHeadStart would be 139
		{
			if((EncoderData > ServoValveHeadStart) || (EncoderData < ServoStartPos))
			{
				ServoValveDelayCounter = 0;
				ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);			// Open the Valve
			}
		}
	}

	// Add headstart of 200ms to open the valve before draining, this enables us to check if the valve is OPEN before driving the servo (using servovalvesensor)
	if(ServoValveHeadStartTimerDrain)
	{
		ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);
		ServoValveHeadStartTimerDrain--;
		if(!ServoValveHeadStartTimerDrain)
		{
			if(!ServoValveSensorEnable)		// Check whether we use ServoValveEnable or not (on PROX2)
			{
				if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
				{
					PulseManualCounter = 9999999;
					if(ServoDirection)         // forward
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
					else
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
					PulseON = TRUE;
					// Enable the PWM
					PulseSum = 0;
					if(ServoAccDccEnable)
					{
						ServoTargetSpeed = ServoDrainSpeed;
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
					}
					else
					{
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoDrainSpeed);
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoDrainSpeed >> 1);  // default 50%
					}
					ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
				}
			}
			else
			{
				if( (!(ROM_GPIOPinRead(PIN_PROX2_BASE, PIN_PROX2_PIN))) )
				{
					if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
					{
						PulseManualCounter = 9999999;
						if(ServoDirection)         // forward
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
						else
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
						PulseON = TRUE;
						// Enable the PWM
						PulseSum = 0;
						if(ServoAccDccEnable)
						{
							ServoTargetSpeed = ServoDrainSpeed;
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
						}
						else
						{
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoDrainSpeed);
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoDrainSpeed >> 1);  // default 50%
						}
						ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
					}
				}
				else
				{
					ServoValveErrorMsgDuration = 80;
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);  // Close the valve if pneu not open
				}
			}
		}
	}
	if(ServoValveHeadStartTimerPMON)
	{
		ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);
		ServoValveHeadStartTimerPMON--;
		if(!ServoValveHeadStartTimerPMON)
		{
			if(!ServoValveSensorEnable)		// Check whether we use ServoValveEnable or not (on PROX2)
			{
				if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
				{
					PulseManualCounter = PulseManual;
					if(ServoDirection)         // forward
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
					else
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
					PulseON = TRUE;
					// Enable the PWM
					PulseSum = 0;
					if(ServoAccDccEnable)
					{
						ServoTargetSpeed = ServoSpeed;
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
					}
					else
					{
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoSpeed);
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoSpeed >> 1);  // default 50%
					}
					ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
				}
			}
			else
			{
				if( (!(ROM_GPIOPinRead(PIN_PROX2_BASE, PIN_PROX2_PIN))) )
				{
					if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
					{
						PulseManualCounter = PulseManual;
						if(ServoDirection)         // forward
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
						else
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
						PulseON = TRUE;
						// Enable the PWM
						PulseSum = 0;
						if(ServoAccDccEnable)
						{
							ServoTargetSpeed = ServoSpeed;
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
						}
						else
						{
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoSpeed);
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoSpeed >> 1);  // default 50%
						}
						ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
					}
				}
				else
				{
					// Reset the PulseManualON Module
					ServoValveErrorMsgDuration = 80;
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);  // Close the valve if pneu not open
					PulseManualONFlag = FALSE;
				}
			}
		}
	}
	if(ServoValveHeadStartTimerBLON)
	{
		ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);
		ServoValveHeadStartTimerBLON--;
		if(!ServoValveHeadStartTimerBLON)
		{
			if(!ServoValveSensorEnable)		// Check whether we use ServoValveEnable or not (on PROX2)
			{
				if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
				{
					ServoBacklashMode = TRUE;
					PulseManualCounter = ServoBacklashPulse;

					if(!ServoDirection)         // backward
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
					else
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
					PulseON = TRUE;
					// Enable the PWM
					PulseSum = 0;
					if(ServoAccDccEnable)
					{
						ServoTargetSpeed = ServoBacklashSpeed;
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
					}
					else
					{
						ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoBacklashSpeed);
						ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoBacklashSpeed >> 1);  // default 50%
					}
					ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
				}
			}
			else
			{
				if( (!(ROM_GPIOPinRead(PIN_PROX2_BASE, PIN_PROX2_PIN))) )
				{
					if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
					{
						ServoBacklashMode = TRUE;
						PulseManualCounter = ServoBacklashPulse;

						if(!ServoDirection)         // backward
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
						else
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
						PulseON = TRUE;
						// Enable the PWM
						PulseSum = 0;
						if(ServoAccDccEnable)
						{
							ServoTargetSpeed = ServoBacklashSpeed;
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
						}
						else
						{
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoBacklashSpeed);
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoBacklashSpeed >> 1);  // default 50%
						}
						ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
					}
				}
				else
				{
					// Reset the backlashON Module
					ServoValveErrorMsgDuration = 80;
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);  // Close the valve if pneu not open
					BacklashONFlag = FALSE;
				}
			}
		}
	}

    /* Servo Module ^ */

    /* Heater Module v */
    // Heater Module Communication Check
    if(Heater1Enable || Heater2Enable || Heater3Enable || Heater4Enable)        // check the com only if one of the heater is enabled
    {
        if(SPIComErrorFlag)
        {
            if(SPIComErrorCounter)
                SPIComErrorCounter--;
            else
                SPIComError = TRUE;
        }
        else
        {
            SPIComError = FALSE;
            SPIComErrorCounter = 2000;
        }

        // Heater Module error message : "Heater Not Ready"
    	if(Heater1Ready && Heater2Ready && Heater3Ready && Heater4Ready)                                                  // If heater not ready pin is ON
    	{
    		HeaterErrorCounter = 3000;
    		HeaterError = FALSE;
    	}
    	else
    	{
    		if(HeaterErrorCounter)
    			HeaterErrorCounter--;
    		else
    		{
    			HeaterErrorMsgDuration = 80;
    			HeaterError = TRUE;
    		}
    	}
    }
    else											// only in TI tree
    {
    	SPIComError = FALSE;
    	SPIComErrorCounter = 2000;
    	HeaterNoConnection = 0;						// WARNING NEW
    }


	// Using External Heater and PIN HEATER ERROR
    if(!(Heater1Enable || Heater2Enable || Heater3Enable || Heater4Enable))        // if all the heater modules are disabled
    {
    	if(!(ROM_GPIOPinRead(PIN_HEATERERROR_BASE, PIN_HEATERERROR_PIN)))		// Active Low , Heater Error is triggered
    	{
    		if(HeaterErrorCounter)
    			HeaterErrorCounter--;
    		else
    		{
    			HeaterErrorMsgDuration = 80;
    			HeaterError = TRUE;
    		}
    	}
    	else
    	{
    		HeaterErrorCounter = 3000;
    		HeaterError = FALSE;
    	}
    }

	/* Heater Module ^ */

	/* Back Rewinder Module v */
	if(RewindEnable)
	{
		// Motor ON and OFF Routines
		if(!(ROM_GPIOPinRead(PIN_REWINDER_SENSA_BASE, PIN_REWINDER_SENSA_PIN)))		// if upper sensor is ON -> Motor OFF
	    {
			ROM_GPIOPinWrite(PORT_REWINDER_RELAY1_BASE, PORT_REWINDER_RELAY1_PIN, 0);	// Turn Motor/Relay1 OFF
	    }
	    else if(!(ROM_GPIOPinRead(PIN_REWINDER_SENSB_BASE, PIN_REWINDER_SENSB_PIN)))	// if lower sensor is ON -> Motor ON
	    {
	    	if(!FoilJam)																// 10 Nov 2015 by Stefanus
	    		ROM_GPIOPinWrite(PORT_REWINDER_RELAY1_BASE, PORT_REWINDER_RELAY1_PIN, PORT_REWINDER_RELAY1_PIN);
	    }

		// FOIL JAM Routines
	    if( (!(ROM_GPIOPinRead(PIN_REWINDER_FJ1_BASE, PIN_REWINDER_FJ1_PIN))) || (!(ROM_GPIOPinRead(PIN_REWINDER_FJ2_BASE, PIN_REWINDER_FJ2_PIN))) )		// if upper foil jam sensor is ON -> Foil jam error
	    {
	        if(FoilJamCounter)
	            FoilJamCounter--;
	        else
	        {
	        	FoilJamMsgDuration = 80;
	            FoilJam = TRUE;
	            ROM_GPIOPinWrite(PORT_REWINDER_RELAY1_BASE, PORT_REWINDER_RELAY1_PIN, 0);	// Turn Motor/Relay1 OFF 10 Nov 2015 by Stefanus
	        }
	    }
	    else
	    {
	        FoilJamCounter = 1000;
	        FoilJam  = FALSE;
	    }
	}
	else
	{
        ROM_GPIOPinWrite(PORT_REWINDER_RELAY1_BASE, PORT_REWINDER_RELAY1_PIN, 0);
        ROM_GPIOPinWrite(PORT_REWINDER_RELAY2_BASE, PORT_REWINDER_RELAY2_PIN, 0);
	}

	/* Back Rewinder Module ^ */

	// 10 ms timer Zone
	if(!ms10counter)
	{
		// ServoValve : To close the Valve after Servo Stops with delay
		if(ServoValveDelayCounter)
		{
			ServoValveDelayCounter--;
			if(!ServoValveDelayCounter)
			{
				ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);		// For immediate response
			}
		}
	    /* Servo Module ^ */
		if(TROTrailCounter)
		{
            ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, PORT_TRO1_PIN);//PORTTRO = TRUE;
			TROTrailCounter--;
			if( (!TROTrailCounter) || (!TROMinCounter) )
			{
                ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, 0);//PORTTRO = FALSE;
			}
		}

		/* Air Check Subroutine v */
        if(!IgnoreAirError)
        {
        	if(ROM_GPIOPinRead(PIN_AIR_ERROR_BASE, PIN_AIR_ERROR_PIN))
        	{
        		if(NoAirErrorCounter)
        			NoAirErrorCounter--;
        		else
                {
                	NoAirError = TRUE;
                	NoAirErrorMsgDuration = 80;
                }
        	}
            else
            {
            	NoAirError = FALSE;
                NoAirErrorCounter = 20;
            }
        }
        else
        {
        	NoAirErrorCounter = 20;
        	NoAirError = FALSE;
        }
        /* Air Check Subroutine ^ */

		ms10counter = 9;
	}
	else
		ms10counter--;

	// 10 ms timer Zone
	if(!ms100counter)
	{
		// To detect if it resets (Counting how long since turned ON)
		LifeDuration++;

		//AgitatorOperation
		if(AgitatorEnable)
		{
			if(MachineRunning)
			{
				AgitatorTimer = 30;            // Timer was 15 second now 3 secs
			}
			else
			{
				if(AgitatorTimer)
					AgitatorTimer--;
			}

			if(!VibratorMode)
			{
				if(AgitatorTimer)
					ROM_GPIOPinWrite(PORT_VIB_BASE, PORT_VIB_PIN, PORT_VIB_PIN);                  // PORTAGITATOR = TRUE;
				else
					ROM_GPIOPinWrite(PORT_VIB_BASE, PORT_VIB_PIN, 0);             // PORTAGITATOR = FALSE;
			}
		}
		else
		{
			AgitatorTimer = 0;
			if(!VibratorMode)
				ROM_GPIOPinWrite(PORT_VIB_BASE, PORT_VIB_PIN, 0);             // PORTAGITATOR = FALSE;
		}

		if(UARTDisabled)
		{
		    // Enable the UART.
		    UARTEnable(UART0_BASE);

		    // Unfortunately, in StellarisWare, UARTEnable() silently re-enables the FIFO,
		    // so to get rid of that, move UARTFIFODisable() call after UARTEnable()
		    UARTFIFODisable(UART0_BASE);
			dataTXIndex = ComAddress * 20;
			dataRXIndex = 0;
			UARTDisabled = FALSE;
			UARTDisabledCounter++;
			ReadACK = FALSE;
		}

		if(ExternalFeederEnable)
		{
			if(MachineRunning || (!ExternalFeederManualEnable))		// Operates Full on Machine Running, and Still operates when not running but if the Manual is disabled
			{
				if(!ExternalFeederTimerMode)
				{	// Sensor Mode (External Feeder is Driven by Sensor)
					if(ROM_GPIOPinRead(PIN_TRI2_BASE, PIN_TRI2_PIN))	// Material NOT Detected -> PINTRI2 = HIGH
					{
						if(HopperBufferSensorCounter)
							HopperBufferSensorCounter--;
						else
							ExternalFeederDurationCounter = ExternalFeederDuration;
					}
					else
						HopperBufferSensorCounter = HopperBufferSensor;
				}
				else
				{	// Timer Mode (External Feeder is Driven by Timer)
					// Timer Mode will be shutted off when machinerunning
					if(ExternalFeederOFFTimerCounter)
					{
						ExternalFeederOFFTimerCounter--;
						if(!ExternalFeederOFFTimerCounter)
							ExternalFeederDurationCounter = ExternalFeederDuration;
					}
					if(!ExternalFeederDurationCounter && !ExternalFeederOFFTimerCounter)
						ExternalFeederOFFTimerCounter = ExternalFeederOFFTimer;


				}

				if(ExternalFeederDurationCounter)
				{
					ExternalFeederDurationCounter--;
					ExternalFeederON = TRUE;
				}
				else
					ExternalFeederON = FALSE;
			}

		}
		else
		{
			ExternalFeederON = FALSE;
			ExternalFeederDurationCounter = 0;
		}

		/* Hopper Subroutine v */
	    // Hopper Subroutine error message : "Hopper Empty", ON means Material available, OFF means no material/hopper empty
		if(HopperSensorEnable)
		{
			// If material is available for more than 3 seconds, reset the hopper error counter
			if(!(ROM_GPIOPinRead(PIN_HOPPER_BASE, PIN_HOPPER_PIN)))                                               // If Hopper is ON
			{
				if(HopperSensorONCounter)
					HopperSensorONCounter--;
				if(!HopperSensorONCounter)
				{
					HopperErrorCounter = 30;
					HopperError = FALSE;
				}
			}
			else
			{
				HopperSensorONCounter = 30;
				if(HopperErrorCounter)
					HopperErrorCounter--;
				else
					HopperError = TRUE;
			}
		}
		else
			HopperError = FALSE;
		/* Hopper Subroutine ^ */

		/* Error Message for HMI Subroutine v */
		if(HeaterErrorMsgDuration)
		{
			HeaterErrorMsgDuration--;
			HeaterErrorMsg = TRUE;
		}
		else
			HeaterErrorMsg = FALSE;

		if(FoilJamMsgDuration)
		{
			FoilJamMsgDuration--;
			FoilJamMsg = TRUE;
		}
		else
			FoilJamMsg = FALSE;

		if(PulseErrorMsgDuration)
		{
			PulseErrorMsgDuration--;
			PulseErrorMsg = TRUE;
		}
		else
			PulseErrorMsg = FALSE;

		if(NoAirErrorMsgDuration)
		{
			NoAirErrorMsgDuration--;
			NoAirErrorMsg = TRUE;
		}
		else
			NoAirErrorMsg = FALSE;
		if(ServoValveErrorMsgDuration)
		{
			ServoValveErrorMsgDuration--;
			ServoValveErrorMsg = TRUE;
		}
		else
			ServoValveErrorMsg = FALSE;
		/* Error Message for HMI Subroutine ^ */

		ms100counter = 99;

	}
	else
		ms100counter--;
}

//*****************************************************************************
// Interrupt handler for the Timer0 interrupt
//*****************************************************************************
void Timer1IntHandler(void)
{
    //unsigned long ulPinStatus;

    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    if(ReadACK)
    {
		if(DataTimerTrigger)
		{
			if(DataReady)
			{
				DataLock = TRUE;
				/*while( dataTXIndex <= ((ComAddress * 20) + (BytestoRead) + 5 - 1) )
				{
					ROM_UARTCharPut(UART0_BASE, ComOUT[dataTXIndex]);
					dataTXIndex++;
				}
				DataTimerCounter++;
				DataLock = FALSE;
				ReadACK = FALSE;
				dataTXIndex = ComAddress * 20;*/
				if(ROM_UARTSpaceAvail(UART0_BASE))
				{
					ROM_UARTCharPutNonBlocking(UART0_BASE, ComOUT[dataTXIndex]);
					if( dataTXIndex == ((ComAddress * 20) + (BytestoRead) + 5 - 1) )
					{
						DataLock = FALSE;
						ReadACK = FALSE;
						dataTXIndex = 0;
					}
					else
						dataTXIndex++;

					DataTimerTrigger = FALSE;
				}
				else
					DataTimerTrigger = TRUE;


			}
			DataTimerCounter++;
		}
    }

    /* Servo Module 0.1ms v */
	// Servo Speed, Accel & Decel, All Speed Variable are actually PERIOD of PWM, therefore "<" become ">"
	if(LastServoTargetSpeed != ServoTargetSpeed)
	{
		if(ServoTargetSpeed < LastServoTargetSpeed)						// Less is more (Period not frequency)
		{
			ServoAccelFlag = TRUE;
			ServoTargetSpeedRPM = konstant / ServoTargetSpeed;
			// Calculating ServoDecelAt from ServoAccelTime
			//ServoAccelTime = ((ServoSpeedRPM - 150) * 10) / ServoAccelRate;  															// in milisecond, * 10 for greater accuracy
			ServoAccelTime = ((ServoTargetSpeedRPM - 150) * 10) / ServoAccelRate;
			//ServoAccelPulse = ((2500 * ServoAccelTime) + (250 * ServoAccelTime * ServoAccelTime)) / 1000;			// Simple Vo.t + 1/2.a.t2
			//ServoAccelPulse = ((2500 * ServoAccelTime) + (( ServoAccelRate * 25 * ServoAccelTime * ServoAccelTime)) / 3) / 1000;			// Simple Vo.t + 1/2.a.t2
			ServoAccelPulse = ((250 * ServoAccelTime) + (( ServoAccelRate * 25 * ServoAccelTime * ServoAccelTime)) / 300) / 1000;
			ServoCurrentRPM10 = 1500;																					// Initial Speed
			ServoCurrentSpeed = 32000;
			if( PulseManualCounter > (ServoAccelPulse * 2) )
				ServoDecelAt = ServoAccelPulse;
			else
				ServoDecelAt = PulseManualCounter / 2;
		}
		else
			ServoDecelFlag = TRUE;
	}
	LastServoTargetSpeed = ServoTargetSpeed;

	if(ServoAccelFlag)
	{
		if(ServoCurrentSpeed > ServoTargetSpeed)					// Servo Speed below target speed
		{
			ServoCurrentRPM10 += ServoAccelRate;
			ServoCurrentSpeed = konstant10 / ServoCurrentRPM10;
			ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoCurrentSpeed);
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoCurrentSpeed >> 1);  // default 50%
		}
		else												// Servo Speed has reached target speed
		{
			ServoCurrentSpeed = ServoTargetSpeed;
			ServoCurrentRPM10 = konstant10 / ServoCurrentSpeed;
			ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoCurrentSpeed);
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoCurrentSpeed >> 1);  // default 50%
			ServoAccelFlag = FALSE;
		}
	}

	if(ServoDecelFlag)
	{
		if(ServoCurrentSpeed < ServoTargetSpeed)					// Servo Speed more than target speed
		{
			if(ServoCurrentRPM10 > (1500 + ServoAccelRate))
				ServoCurrentRPM10 -= ServoAccelRate;
			else
				ServoCurrentRPM10 = 1500;
			ServoCurrentSpeed = konstant10 / ServoCurrentRPM10;
			ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoCurrentSpeed);
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoCurrentSpeed >> 1);  // default 50%
		}
		else												// Servo Speed has reached target speed
		{
			ServoCurrentSpeed = ServoTargetSpeed;
			ServoCurrentRPM10 = konstant10 / ServoCurrentSpeed;
			ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoCurrentSpeed);
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoCurrentSpeed >> 1);  // default 50%
			ServoDecelFlag = FALSE;
		}
	}
	/* Servo Module 0.1ms ^ */

    if(!MachineRunning)
    {
    	if(!ServoValveEnable)
    	{
			if(ServoDrain)
			{
				if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
				{
					PulseManualCounter = 9999999;
					if(ServoDirection)         // forward
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
					else
						ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
					PulseON = TRUE;
					// Enable the PWM
					PulseSum = 0;
					ServoTargetSpeed = ServoDrainSpeed;
					ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
					ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
					ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
				}
			}
			else
			{
				if(PulseManualON)
				{
					if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
					{
						PulseManualCounter = PulseManual;
						if(ServoDirection)         // forward
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
						else
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
						PulseON = TRUE;
						// Enable the PWM
						PulseSum = 0;
						if(ServoAccDccEnable)
						{
							ServoTargetSpeed = ServoSpeed;
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
						}
						else
						{
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoSpeed);
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoSpeed >> 1);  // default 50%
						}
						ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
					}
					PulseManualON = FALSE;
				}

				if(BacklashON)
				{
					if(!PulseON)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
					{
						if(ServoBacklashEnable)
							ServoBacklashMode = TRUE;
						PulseManualCounter = ServoBacklashPulse;

						if(!ServoDirection)         // backward
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
						else
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
						PulseON = TRUE;
						// Enable the PWM
						PulseSum = 0;
						if(ServoAccDccEnable)
						{
							ServoTargetSpeed = ServoBacklashSpeed;
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
						}
						else
						{
							ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoBacklashSpeed);
							ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoBacklashSpeed >> 1);  // default 50%
						}
						ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
					}
					BacklashON = FALSE;
				}
			}
    	}
    	else		// if(servovalveenable == TRUE) these codes below are for giving 200ms headstart for the valve to open first before servo turning
    	{
			if(ServoDrain)
			{
				if(!ServoDrainFlag)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
				{
		    		ServoDrainFlag = TRUE;
		    		ServoValveHeadStartTimerDrain = 200;   	// 200 ms
				}
			}
			else
			{
				if(PulseManualON)
				{
					if(!PulseManualONFlag)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
					{
			    		PulseManualONFlag = TRUE;
			    		ServoValveHeadStartTimerPMON = 200;   	// 200 ms
					}
					PulseManualON = FALSE;
				}

				if(BacklashON)
				{
					if(!BacklashONFlag)               // if on standby mode, and clutch is not on (that means machine is not running), and on auto mode
					{
						BacklashONFlag = TRUE;
						ServoValveHeadStartTimerBLON = 200;   	// 200 ms
					}
					BacklashON = FALSE;
				}
			}
    	}
    }
}

//*****************************************************************************
// Interrupt handler for the PWM Output
//*****************************************************************************
void IntPWMGen3(void)
{
	ROM_PWMGenIntClear(PWM0_BASE, PWM_GEN_3, PWM_INT_CNT_ZERO);

	// Place your code here
	if(PulseON)
	{
		PulseSum++;

		if(ServoValveEnable)			// If Enabled, whenever PulseON is TRUE, activate Valve
		{
			//ServoValveDelayCounter = 0;
			ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);
		}
		if(!DrainingMode)
		{
			if(PulseManualCounter > 1)
			{
				PulseManualCounter--;
				if(!ServoDrain)
				{
					if(PulseManualCounter == ServoDecelAt)			// Start Decelerating at ServoDecelAt
					{
						ServoTargetSpeed = 32000;					// servospeed 150 RPM (slowest)
						ServoAccelFlag = FALSE;
					}
				}
			}
			else
			{
				ROM_PWMGenDisable(PWM0_BASE, PWM_GEN_3);
				PWMWatch = 2;
				PulseON = FALSE;

				if(AirCutterEnable && AirCutterMode)
					AirCutterPosCounter = AirCutterPos << 2;             // Timer x 4 ms

				if(ServoBacklashEnable)
				{
					ServoDecelFlag = FALSE;
					if(ServoBacklashMode)	// After finishing backlash
					{
						ServoBacklashMode = FALSE;

						if(ServoDirection)         // back to forward
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
						else
							ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);

						if(ServoValveEnable)
						{
							ServoValveDelayCounter = ServoValveDelay;
							BacklashONFlag = FALSE;			// Reset For Valve Headstart when manual Backlash ON
							PulseManualONFlag = FALSE;			// Reset For Valve Headstart when Pulsemanual (OCyc) ON
						}
					}
					else					// Havent done backlash yet -> do the backlash
					{
						ServoBacklashMode = TRUE;

						if(ServoBacklashTimer)
							ServoBacklashTimerCounter = ServoBacklashTimer;
						else
						{
							if(ServoBacklashPulse)      // If backlash pulse is not 0
							{
								PulseManualCounter = ServoBacklashPulse;        // fill pulse with backlash pulse
								if(!ServoDirection)         // backward
									ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
								else
									ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
								if(ServoAccDccEnable)
								{
									ServoTargetSpeed = ServoBacklashSpeed;
									ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
									ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
								}
								else
								{
									ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoBacklashSpeed);
									ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoBacklashSpeed >> 1);  // default 50%
								}
								PulseON = TRUE;
								ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);  //PulseOutputON
							}
							else						// If backlash pulse is 0
							{
								ServoBacklashMode = FALSE;
								if(ServoAccDccEnable)
								{
									ServoTargetSpeed = ServoSpeed;
									ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 32000);   // servospeed 150 RPM (slowest)
									ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 16000);  // default 50%
								}
								else
								{
									ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ServoSpeed);
									ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ServoSpeed >> 1);  // default 50%
								}
								if(ServoDirection)         // back to forward
									ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, PORT_SERVO_DIR_PIN);
								else
									ROM_GPIOPinWrite(PORT_SERVO_DIR_BASE, PORT_SERVO_DIR_PIN, 0);
							}
						}
					}
				}
				else			// if backlash disabled, start ServoValveDelay
				{
					if(ServoValveEnable)
					{
						ServoValveDelayCounter = ServoValveDelay;
						PulseManualONFlag = FALSE;			// Reset For Valve Headstart when Pulsemanual (OCyc) ON
					}
				}
			}
		}
	}
	else                                    // To check if the program to this subroutine themselves
	{
		ROM_PWMGenDisable(PWM0_BASE, PWM_GEN_3);		// PulseOutputOFF
		PWMWatch = 1;
	}
}

int
main(void)
{

    // Set the system clock 80MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOG);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOH);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOJ);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOK);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_QEI1);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_SSI0);
    //ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_SSI1);
    //ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_SSI2);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_I2C1);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM0);

	// Change PD7 PF0 into GPIO outputs. First open the lock and select
	// the bits we want to modify in the GPIO commit register.

	// unlocking PORTD7
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;

	// unlocking PORTF0
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;

    // Initialize the GPIO
    ROM_GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, (0xFF - 0xEE));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, 0xEE);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, (0x3F - 0x3C));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, 0x3C);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, (0xFF - 0x88));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, 0x88);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTD_BASE, (0xFF - 0x91));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, 0x91);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTE_BASE, (0xFF - 0xBF));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0xBF);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTF_BASE, (0xFF - 0x7E));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x7E);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTG_BASE, (0xFF - 0x00));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, 0x00);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTH_BASE, (0xFF - 0xFF));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, 0xFF);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTJ_BASE, (0x07 - 0x04));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, 0x04);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTK_BASE, (0x0F - 0x08));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, 0x08);
    //*****************************************************************************
    // Interrupt Settings
    //*****************************************************************************
	ROM_GPIOIntTypeSet(PIN_EYEMARK_BASE, PIN_EYEMARK_PIN, GPIO_FALLING_EDGE);
	ROM_GPIOIntTypeSet(PIN_PULSE_Z_BASE, PIN_PULSE_Z_PIN, GPIO_FALLING_EDGE);
	ROM_GPIOIntTypeSet(PIN_PULSE_A_BASE, PIN_PULSE_A_PIN, GPIO_BOTH_EDGES);
	ROM_GPIOIntTypeSet(PIN_PULSE_B_BASE, PIN_PULSE_B_PIN, GPIO_BOTH_EDGES);

	GPIOIntEnable(PIN_EYEMARK_BASE, PIN_EYEMARK_PIN);
	GPIOIntEnable(PIN_PULSE_Z_BASE, PIN_PULSE_Z_PIN);
	GPIOIntEnable(PIN_PULSE_A_BASE, PIN_PULSE_A_PIN);
	GPIOIntEnable(PIN_PULSE_B_BASE, PIN_PULSE_B_PIN);

    //*****************************************************************************
    // Timer 0 Setup
    //*****************************************************************************
    // Configure Timer0 as 32-bit periodic timer
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // Set Timer0 period as 1/1000 of the system clock, i.e. 1000 interrupts per second
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000);
    // Configure the timer to generate an interrupt on time-out
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Enable the TImer Interrupt in the NVIC
    IntEnable(INT_TIMER0A);
    // Enable the timer
    TimerEnable(TIMER0_BASE, TIMER_A);

    //*****************************************************************************
    // Timer 1 Setup
    //*****************************************************************************
    // Configure Timer0 as 32-bit periodic timer
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    // Set Timer0 period as 1/10000 of the system clock, i.e. 10000 interrupts per second
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 10000);
    // Configure the timer to generate an interrupt on time-out
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Enable the timer Interrupt in the NVIC
    IntEnable(INT_TIMER1A);
    // Enable the timer
    TimerEnable(TIMER1_BASE, TIMER_A);

    //*****************************************************************************
    // PWM Setup
    //*****************************************************************************
    /*
    // Setting up Generator 2 PWM 4
    GPIOPinConfigure(GPIO_PH4_M0PWM4);
    ROM_GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_4);
	ROM_PWMGenConfigure(PWM0_BASE ,//unsigned long ulBase,
						PWM_GEN_2,//unsigned long ulGen,
						PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN |
						PWM_GEN_MODE_GEN_NO_SYNC | PWM_GEN_MODE_DB_NO_SYNC |
						PWM_GEN_MODE_FAULT_LATCHED | PWM_GEN_MODE_FAULT_NO_MINPER |
						PWM_GEN_MODE_FAULT_LEGACY );//unsigned long ulConfig)

	ROM_PWMOutputInvert(PWM0_BASE, PWM_OUT_4_BIT, true);
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 60000);   // default 1000pulse/sec

	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 30000);  // default 50%

	//PWMGenIntRegister(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
	ROM_PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);
	ROM_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);		// Interrupt at every pulse (when timercounter reach 0)

	ROM_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_2);
	IntEnable(INT_PWM0_2); // enable PWM Generator 2 interrupts

	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
	*/
    // Setting up Generator 3 PWM 7
    GPIOPinConfigure(GPIO_PH7_M0PWM7);
    ROM_GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_7);
	ROM_PWMGenConfigure(PWM0_BASE ,//unsigned long ulBase,
						PWM_GEN_3,//unsigned long ulGen,
						PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN |
						PWM_GEN_MODE_GEN_NO_SYNC | PWM_GEN_MODE_DB_NO_SYNC |
						PWM_GEN_MODE_FAULT_LATCHED | PWM_GEN_MODE_FAULT_NO_MINPER |
						PWM_GEN_MODE_FAULT_LEGACY );//unsigned long ulConfig)

	ROM_PWMOutputInvert(PWM0_BASE, PWM_OUT_7_BIT, true);
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 60000);   // default 1000pulse/sec

	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 30000);  // default 50%

	//PWMGenIntRegister(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
	ROM_PWMGenIntClear(PWM0_BASE, PWM_GEN_3, PWM_INT_CNT_ZERO);
	ROM_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_3, PWM_INT_CNT_ZERO);		// Interrupt at every pulse (when timercounter reach 0)

	ROM_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_3);
	IntEnable(INT_PWM0_3); // enable PWM Generator 2 interrupts

	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
	//*****************************************************************************
	// QEI Setup
	//*****************************************************************************
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);
	GPIOPinConfigure(GPIO_PC4_IDX1);

	ROM_GPIOPinTypeQEI(PIN_PULSE_Z_BASE , PIN_PULSE_Z_PIN);
	ROM_GPIOPinTypeQEI(PIN_PULSE_B_BASE , PIN_PULSE_B_PIN);
	ROM_GPIOPinTypeQEI(PIN_PULSE_A_BASE , PIN_PULSE_A_PIN);

	ROM_QEIConfigure(QEI1_BASE,
					 QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_RESET_IDX | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP,//unsigned long ulConfig,
					 //QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP,//unsigned long ulConfig,
					 (ENCPULSE - 1));//unsigned long ulMaxPosition)
					 //4294967295);
	ROM_QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 80000000);				//every 1 secs

	ROM_QEIEnable(QEI1_BASE);
	ROM_QEIVelocityEnable(QEI1_BASE);

    //*****************************************************************************
    // UART Setup
    //*****************************************************************************
 	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
 	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);

	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 |
						GPIO_PIN_1);

	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));

    // Configuring the UART interrupt.
	//ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
	UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX);
    ROM_IntEnable(INT_UART0);

    // Enable the UART.
    UARTEnable(UART0_BASE);

    // Unfortunately, in StellarisWare, UARTEnable() silently re-enables the FIFO,
    // so to get rid of that, move UARTFIFODisable() call after UARTEnable()
    UARTFIFODisable(UART0_BASE);

    //*****************************************************************************
	// I2C Setup
	//*****************************************************************************
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);   //   I2CSCL    *** note - this function has not yet made it into MCU ROM!
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
	//ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE , GPIO_PIN_6 | GPIO_PIN_7);		// For LM3 Only
	I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), FALSE);		// speed 400kbps
	I2CMasterSlaveAddrSet(I2C1_BASE, 0x28, FALSE);    			// slave address 0b 0101 0000, writing the slave

    //*****************************************************************************
    // SSI0 Setup
    //*****************************************************************************

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    ROM_GPIOPinTypeSSI(PORT_SSI_CLK_BASE, PORT_SSI_CLK_PIN | PORT_SSI_FSS_PIN |
                   PIN_SSI_RX_PIN | PORT_SSI_TX_PIN);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 125000, 8);

    SSIEnable(SSI0_BASE);

    // Clear SPI Buffer
    while(SSIDataGetNonBlocking(SSI0_BASE, &SPIIn[0]))
    {
    }

    // switch to interrupt driven SPI

    SSIIntDisable(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );
    SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );

    HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT; // switch tx interrupt to eot int

    SSIIntEnable(SSI0_BASE, SSI_TXFF ); // SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR

    ROM_IntEnable(INT_SSI0);

    //*****************************************************************************
    // SSI1 Setup
    //*****************************************************************************
    /*
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);
    GPIOPinConfigure(GPIO_PF0_SSI1RX);
    GPIOPinConfigure(GPIO_PF1_SSI1TX);
    ROM_GPIOPinTypeSSI(PORT_SSI1_CLK_BASE, PORT_SSI1_CLK_PIN | PORT_SSI1_FSS_PIN |
                   PIN_SSI1_RX_PIN | PORT_SSI1_TX_PIN);

    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 125000, 8);

    SSIEnable(SSI1_BASE);

    // Clear SPI Buffer
    while(SSIDataGetNonBlocking(SSI1_BASE, &SPIIn2[0]))
    {
    }

    // switch to interrupt driven SPI

    SSIIntDisable(SSI1_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );
    SSIIntClear(SSI1_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );

    HWREG(SSI1_BASE + SSI_O_CR1) |= SSI_CR1_EOT; // switch tx interrupt to eot int

    SSIIntEnable(SSI1_BASE, SSI_TXFF ); // SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR

    ROM_IntEnable(INT_SSI1);
	*/

    // Enable Interrupt
    ROM_IntMasterEnable();

	ROM_IntEnable(INT_GPIOC);
	ROM_IntEnable(INT_GPIOD);

    while(1)
    {
    	LoopCycle++;

    	if(ServoStartPos >= 10)
    		ServoValveHeadStart = ServoStartPos - ServoValveHeadstartVar;
    	else
    		ServoValveHeadStart = (ENCPULSE + ServoStartPos) - ServoValveHeadstartVar;

        /* Limit Value v */
        if( CounterCutterCounter > CounterCutterSetting )
            CounterCutterCounter = CounterCutterSetting;
        /* Limit Value ^ */

        /* DAC Value v */
        ROM_I2CMasterDataPut(I2C1_BASE, 0xA9);
        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(ROM_I2CMasterBusy(I2C1_BASE));
        if(SpeedModeSelector)                                       // Goes into manual speed control and ignore offset and close loop control
		{
			DACSpeed = InitialSpeed;
			OffsetSpeed = 0;
		}
		else
			DACSpeed = InitialSpeed + OffsetSpeed;

		if(OffsetSpeed > 15)                                        // Limit the offset
			OffsetSpeed = 15;
		if(OffsetSpeed < -15)                                       // Limit the offset
			OffsetSpeed = -15;

		if(DACSpeed > 255){DACSpeed = 255;}
		if(DACSpeed < 0){DACSpeed = 0;}

		if(LowSpeedMode)
			ROM_I2CMasterDataPut(I2C1_BASE,40);
		else
	       	ROM_I2CMasterDataPut(I2C1_BASE,DACSpeed);

        ROM_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        //while(ROM_I2CMasterBusy(I2C1_BASE));
        /* DAC Value ^ */

        /* Control Program v */
        if(LastServiceMode != ServiceMode)                                                                      // Rising or Falling Check Flag
        {
            if(ServiceMode == FALSE)                                                                            // Falling     Service Mode -> Normal Mode
            {
                CheckPulseMode = FALSE;
                Initialised = FALSE;
                ClutchON = FALSE;
                BreakON = FALSE;
                VlvBlowON = FALSE;
                ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, 0);
                ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, 0);
                ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, 0);
                ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, 0);
                ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);
            }
            if(ServiceMode == TRUE)                                                                             // Rising      Normal Mode -> Service Mode
            {
            }
        }
        LastServiceMode = ServiceMode;
        if(CheckPulseMode)
            LowSpeedMode = TRUE;
        else
            LowSpeedMode = FALSE;

        if(!MachineRunning)
            if(AgitatorEnable)
                if(AgitatorManual)
                    AgitatorTimer = 2;

        if((!ServiceMode) && (ComFlag))                                                                               // Operation Page (Not Service Mode)
        {
            if((!MachineRunning) && (!ServoDrain))                                                                             // when machine is not running
            {
                if( MotorONBtn || !RunButton ) // (!PINBTNRUN)
                {
                    if((!AgitatorTimer) && AgitatorEnable)
                    {
                        if(!AgitatorTimer)
                        {
                            AgitatorTimer = 30;                       // Turn on the agitator
                            longintdelay = 40000000;				// assume per loop consumes 4 cycles
                            //SysCtlDelay(160000000);						// Cause Interrupt to stop hence "MC NOT RESPONDING"
                        }
                    }
                    if( (!VlvRollerON) ) // PORTAGITATOR is not moving
                    {
                        if(!VlvRollerON)
                        {
                            VlvRollerON = TRUE;
                            ROM_GPIOPinWrite(PORT_GUSSET_VALVE_ROLL_BASE, PORT_GUSSET_VALVE_ROLL_PIN, PORT_GUSSET_VALVE_ROLL_PIN);  // VlvRollerON = TRUE;
                            longintdelay += 25000000;				// assume per loop consumes 10 cycles
                            //SysCtlDelay(100000000);                     // Wait for the roller to close -> Cause Interrupt to stop hence "MC NOT RESPONDING"
                        }
                    }
                    while(longintdelay)
                    	longintdelay--;
                    if(ZIErrorCounter > 2)
                        ZIErrorCounter = 2;
                    if(EyemarkErrorCounter > 2)
                        EyemarkErrorCounter = 2;
                    ZIError = FALSE;
                    if(Error == FALSE)                             // Run Button is pushed
                    {
                        //VlvRollerON = TRUE;                        // WARNING, NO TIMER YET
                        SpeedperMinEncoder = 0;
                        SpeedperMinCounter = 0;
                        MotorON = TRUE;
                        MachineRunning = TRUE;
                        PulseErrorCounter = 800;
                        PulseError = FALSE;
                        FoilJamCounter = 1000;
                        FoilJam  = FALSE;
                        FirstCycleFlag = TRUE;
                        StopCommand = FALSE;
                        HeaterErrorMsgDuration = 0;
                        FoilJamMsgDuration = 0;
                        PulseErrorMsgDuration = 0;
                        NoAirErrorMsgDuration = 0;
                        ServoValveErrorMsgDuration = 0;

                        // Reload the EyemarkIgnore
                        EyemarkIgnoreCounter = EyemarkIgnore;
                        if(Initialised)
                        {
                            if(EyemarkEN)
                            {
                                if((OCycEyemark) || (EncoderData >= BrakePos) )                                             // In that cycle : Eyemark has been triggered
                                {
                                    ClutchON = FALSE;
                                    BreakON = TRUE;
                                }
                                else
                                {
                                    BreakON = FALSE;
                                    ClutchON = TRUE;
                                }
                            }
                            else
                            {
                                if(EncoderData < BrakePos)
                                {
                                    BreakON = FALSE;
                                    ClutchON = TRUE;
                                }
                                else
                                {
                                    ClutchON = FALSE;
                                    BreakON = TRUE;
                                }
                            }
                        }
                    }
                }
            }
            else                                                                                                    // when machine is RUNNING
            {
                if(MotorOFFBtn || Error || !StopButton)         // Motor OFF Btn/Inching pushed or Error Triggered
                {
                    if(StopCommand == FALSE)                                              // Stop the machine at designated STOP AT position
                    {
                        if(TROLaunched || StandbyMode)
                        {
                            TROLaunched = FALSE;
    	                    ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, 0);		//PORTTRO = FALSE;
                            StandbyMode = FALSE;
                            MotorON = FALSE;
                            CNBStopTimerFlag = TRUE;
                            //MotorON = TRUE;
                        }
                        else
                        {
                            StopCommand = TRUE;
                            PulseErrorCounter = 1500;
                        }

                    }
                }
                if(FatalError)
                {
                    MotorON = FALSE;
                    Initialised = FALSE;
                    CNBStopTimerFlag = TRUE;
                    StandbyMode = FALSE;                    // warning new
                    TROLaunched = FALSE;                    // warning new
                    ROM_GPIOPinWrite(PORT_TRO1_BASE, PORT_TRO1_PIN, 0);   //PORTTRO = FALSE;
                }
            }
        }

        if( (!MachineRunning) && (!CheckPulseMode) )                                                                             // when machine is not running
        {
            if(MotorOFFBtn || !StopButton)
            {                                                                       // Motor OFF Button Pressed
                MotorON = TRUE;
                Initialised = FALSE;
            }
            else
                MotorON = FALSE;
        }
        if(CheckPulseMode && FatalError)
        {
            CheckPulseMode = FALSE;
            MotorON = FALSE;
            Initialised = FALSE;
        }
        //FoilSub();                                                                        // Motor Foil Subroutine
        /* Control Program ^ */

        /* Heater Subroutine v */
        TempHeater1HLimit = Heater1SV + Heater1HLimit;
        if(TempHeater1HLimit > 255)
            TempHeater1HLimit = 255;

        TempHeater1LLimit = Heater1SV - Heater1LLimit;
        if(TempHeater1LLimit < 0)
            TempHeater1LLimit = 0;

        TempHeater2HLimit = Heater2SV + Heater2HLimit;
        if(TempHeater2HLimit > 255)
            TempHeater2HLimit = 255;

        TempHeater2LLimit = Heater2SV - Heater2LLimit;
        if(TempHeater2LLimit < 0)
            TempHeater2LLimit = 0;

        TempHeater3HLimit = Heater3SV + Heater3HLimit;
        if(TempHeater3HLimit > 255)
            TempHeater3HLimit = 255;

        TempHeater3LLimit = Heater3SV - Heater3LLimit;
        if(TempHeater3LLimit < 0)
            TempHeater3LLimit = 0;

        TempHeater4HLimit = Heater4SV + Heater4HLimit;
        if(TempHeater4HLimit > 255)
            TempHeater4HLimit = 255;

        TempHeater4LLimit = Heater4SV - Heater4LLimit;
        if(TempHeater4LLimit < 0)
            TempHeater4LLimit = 0;

        if(Heater1Enable)
        {
            if( (Heater1PV <=  TempHeater1HLimit) && (Heater1PV >= TempHeater1LLimit) )               // Checking if Heater is Ready(In the limit range)
                Heater1Ready = TRUE;
            else
                Heater1Ready = FALSE;
        }
        else
            Heater1Ready = TRUE;

        if(Heater2Enable)
        {
            if( (Heater2PV <=  TempHeater2HLimit) && (Heater2PV >= TempHeater2LLimit) )               // Checking if Heater is Ready(In the limit range)
                Heater2Ready = TRUE;
            else
                Heater2Ready = FALSE;
        }
        else
            Heater2Ready = TRUE;

        if(Heater3Enable)
        {
            if( (Heater3PV <=  TempHeater3HLimit) && (Heater3PV >= TempHeater3LLimit) )               // Checking if Heater is Ready(In the limit range)
                Heater3Ready = TRUE;
            else
                Heater3Ready = FALSE;
        }
            else
                Heater3Ready = TRUE;

        if(Heater4Enable)
        {
            if( (Heater4PV <=  TempHeater4HLimit) && (Heater4PV >= TempHeater4LLimit) )               // Checking if Heater is Ready(In the limit range)
                Heater4Ready = TRUE;
            else
                Heater4Ready = FALSE;
        }
        else
            Heater4Ready = TRUE;

        /* Heater Subroutine ^ */

        /* Speed Searching Routine v */
        //InitialSpeed = ((ComDACSpeed * 290) / 100) + 6;                                                    // InitialSpeed = ((ComDACSpeed * 217) / 100) + 3;
        /* Speed Searching Routine ^ */

        /* PIN Status v */

        /* PIN Status ^ */

        /* Coiling to PORT */
        if(!MachineRunning)
        {
        	if(OUTEX1Enable)
        	{
				if(OUTEX1ON)
					ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, PORT_EX1_PIN);
				else
					ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, 0);
        	}
        	if(OUTEX2Enable)
        	{
				if(OUTEX2ON)
					ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, PORT_EX2_PIN);
				else
					ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, 0);
        	}
        	if(OUTEX3Enable)
        	{
				if(OUTEX3ON)
					ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, PORT_EX3_PIN);
				else
					ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, 0);
        	}
        	// Servo Valve Enable manual control button only works when not on One Cycle Servo Period and Off delay Time
        	// Servo Valve Enable manual control button only works when there is no pre-valveopen (headstart)
        	if(OUTEX4Enable || (ServoValveEnable && (!PulseON) && (!ServoValveDelayCounter) && (!ServoValveHeadStartTimerDrain) && (!ServoValveHeadStartTimerPMON)) && (!ServoValveHeadStartTimerBLON) )
        	{
				if(OUTEX4ON)
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, PORT_EX4_PIN);
				else
					ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);
        	}
        }
        if(ClutchON)
        	ROM_GPIOPinWrite(PORT_CLUTCH_BASE, PORT_CLUTCH_PIN, PORT_CLUTCH_PIN);
        else
        	ROM_GPIOPinWrite(PORT_CLUTCH_BASE, PORT_CLUTCH_PIN, 0);
        if(BreakON)
        	ROM_GPIOPinWrite(PORT_BRAKE_BASE, PORT_BRAKE_PIN, PORT_BRAKE_PIN);
        else
        	ROM_GPIOPinWrite(PORT_BRAKE_BASE, PORT_BRAKE_PIN, 0);
        if(MotorON)
        	ROM_GPIOPinWrite(PORT_INV1_BASE, PORT_INV1_PIN, PORT_INV1_PIN);
        else
        	ROM_GPIOPinWrite(PORT_INV1_BASE, PORT_INV1_PIN, 0);
        if(VibratorMode)
        {
			if(VibratorFormerON)
				ROM_GPIOPinWrite(PORT_VIB_BASE, PORT_VIB_PIN, PORT_VIB_PIN);
			else
				ROM_GPIOPinWrite(PORT_VIB_BASE, PORT_VIB_PIN, 0);
        }

		if(CounterCutterON)
		{
			ROM_GPIOPinWrite(PORT_COUNTERC_BASE, PORT_COUNTERC_PIN, PORT_COUNTERC_PIN);
			ROM_GPIOPinWrite(PORT_GUSSET_VALVE3_BASE, PORT_GUSSET_VALVE3_PIN, PORT_GUSSET_VALVE3_PIN);    		//PORTVLVCUT = TRUE;
		}
		else
		{
			ROM_GPIOPinWrite(PORT_COUNTERC_BASE, PORT_COUNTERC_PIN, 0);
			ROM_GPIOPinWrite(PORT_GUSSET_VALVE3_BASE, PORT_GUSSET_VALVE3_PIN, 0);    		//PORTVLVCUT = FALSE;
		}

		if(ExternalFeederEnable)
		{
			if(!ExternalFeederAgitatorEnable)		// Normal Mode
			{
				if(ExternalFeederON)
					ROM_GPIOPinWrite(PORT_TRO2_BASE, PORT_TRO2_PIN, PORT_TRO2_PIN);
				else
					ROM_GPIOPinWrite(PORT_TRO2_BASE, PORT_TRO2_PIN, 0);
			}
			else
			{
				if(ExternalFeederON && AgitatorEnable && AgitatorTimer) // If External Feeder can only move when agitator is on
					ROM_GPIOPinWrite(PORT_TRO2_BASE, PORT_TRO2_PIN, PORT_TRO2_PIN);
				else
					ROM_GPIOPinWrite(PORT_TRO2_BASE, PORT_TRO2_PIN, 0);
			}

        }

		if(!ExternalFeederEnable)      								// If TRO2 is controlled by something else as well, just add "AND" on this line
			ROM_GPIOPinWrite(PORT_TRO2_BASE, PORT_TRO2_PIN, 0);

        if(VlvRollerON)
            ROM_GPIOPinWrite(PORT_GUSSET_VALVE_ROLL_BASE, PORT_GUSSET_VALVE_ROLL_PIN, PORT_GUSSET_VALVE_ROLL_PIN);
        else
        	ROM_GPIOPinWrite(PORT_GUSSET_VALVE_ROLL_BASE, PORT_GUSSET_VALVE_ROLL_PIN, 0);
        if(GussetEnable)
        {
            if(VlvBlowON)
            	ROM_GPIOPinWrite(PORT_GUSSET_VALVE_BLOW_BASE, PORT_GUSSET_VALVE_BLOW_PIN, PORT_GUSSET_VALVE_BLOW_PIN);
            else
				ROM_GPIOPinWrite(PORT_GUSSET_VALVE_BLOW_BASE, PORT_GUSSET_VALVE_BLOW_PIN, 0);
        }
        if(DatecodeON)
            ROM_GPIOPinWrite(PORT_DATECODE_BASE, PORT_DATECODE_PIN, PORT_DATECODE_PIN);
        else
        	ROM_GPIOPinWrite(PORT_DATECODE_BASE, PORT_DATECODE_PIN, 0);
        if(ServoEnable)
        	ROM_GPIOPinWrite(PORT_SERVO_ON_BASE, PORT_SERVO_ON_PIN, PORT_SERVO_ON_PIN);
        else
        	ROM_GPIOPinWrite(PORT_SERVO_ON_BASE, PORT_SERVO_ON_PIN, 0);

        if(TRIDelayTimer)
            TRIOnDelay = TRUE;
        else
            TRIOnDelay = FALSE;

        if(ServoAlmRst)
        	ROM_GPIOPinWrite(PORT_SERVO_ALMRST_BASE, PORT_SERVO_ALMRST_PIN, PORT_SERVO_ALMRST_PIN);
        else
        	ROM_GPIOPinWrite(PORT_SERVO_ALMRST_BASE, PORT_SERVO_ALMRST_PIN, 0);

        /* Coiling to PORT */

        /* Coiling to PIN */

        /* Coiling to PIN */

        /* Disabling PORTS when Enable is OFF */
        if(!AirCutterEnable)
        	ROM_GPIOPinWrite(PORT_GAS_BASE, PORT_GAS_PIN, 0);
        if(!OUTEX1Enable)
        	ROM_GPIOPinWrite(PORT_EX1_BASE, PORT_EX1_PIN, 0);
        if(!OUTEX2Enable)
        	ROM_GPIOPinWrite(PORT_EX2_BASE, PORT_EX2_PIN, 0);
        if(!OUTEX3Enable)
        	ROM_GPIOPinWrite(PORT_EX3_BASE, PORT_EX3_PIN, 0);
        if((!OUTEX4Enable) && (!ServoValveEnable))
        	ROM_GPIOPinWrite(PORT_EX4_BASE, PORT_EX4_PIN, 0);
        /* Disabling PORTS when Enable is OFF */

        /* Error Identifier v */

        if(ToggleAuto)
        {
            if(EyemarkErrorCounter >= 3)
                EyemarkError = TRUE;
            else
                EyemarkError = FALSE;

            if(ZIErrorCounter >= 3)
                ZIError = TRUE;
            else
                ZIError = FALSE;
        }
        else
        {
            EyemarkError = FALSE;
            ZIError = FALSE;
            EyemarkErrorCounter = 2;
            ZIErrorCounter = 2;
        }
        //DoorOpen = (!(ROM_GPIOPinRead(PIN_DOOROPEN_BASE, PIN_DOOROPEN_PIN)));

        if(FoilJam || PulseError || NoAirError || DatesensError || ServoError)
            FatalError = TRUE;
        else
            FatalError = FALSE;

        if(HeaterError || DoorOpen || EyemarkError || ZIError || HopperError || ServoValveErrorMsg)
            Error = TRUE;
        else
            Error = FALSE;

        /* Error Identifier ^ */

        /* Error Handler v */

        if(Error || FatalError || PulseCommutationError || HeaterErrorMsg || FoilJamMsg || PulseErrorMsg || NoAirErrorMsg)
        {
            if(blinkFlag)                           // Warning, took DoorOpen place in com
                BlinkCom = TRUE;
            else
                BlinkCom = FALSE;
        }
        else
            BlinkCom = FALSE;

        /* Error Handler ^ */
        if(ReadACK)
        {
            if(DataReady)
            {
                DataLock = TRUE;
				while( dataTXIndex <= ((ComAddress * 20) + (BytestoRead) + 5 - 1) )
				{
					ROM_UARTCharPut(UART0_BASE, ComOUT[dataTXIndex]);
					dataTXIndex++;
				}
				DataLock = FALSE;
				ReadACK = FALSE;
				dataTXIndex = ComAddress * 20;
            }
            else
                DataTimerTrigger = TRUE;                        // Activate the transfer manually
        }

        /* Porting Compatibility with MBTI v */
        PORTAMBTI = ROM_GPIOPinRead(GPIO_PORTA_BASE, 0xFF);
        PORTBMBTI = Digitize(GPIO_PORTH_BASE, GPIO_PIN_3) +
        			(Digitize(GPIO_PORTH_BASE, GPIO_PIN_2) << 1) +
					(Digitize(GPIO_PORTH_BASE, GPIO_PIN_1) << 2) +
					(Digitize(GPIO_PORTC_BASE, GPIO_PIN_7) << 3) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_0) << 4) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_1) << 5) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_2) << 6) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_3) << 7);
        PORTCMBTI = Digitize(GPIO_PORTC_BASE, GPIO_PIN_0) +
        			(Digitize(GPIO_PORTC_BASE, GPIO_PIN_1) << 1) +
					(Digitize(GPIO_PORTC_BASE, GPIO_PIN_2) << 2) +
					(Digitize(GPIO_PORTC_BASE, GPIO_PIN_3) << 3) +
					(Digitize(GPIO_PORTH_BASE, GPIO_PIN_0) << 4) +
					(Digitize(GPIO_PORTD_BASE, GPIO_PIN_4) << 5) +
					(Digitize(GPIO_PORTE_BASE, GPIO_PIN_7) << 6) +
					(Digitize(GPIO_PORTC_BASE, GPIO_PIN_6) << 7);
        PORTDMBTI = Digitize(GPIO_PORTC_BASE, GPIO_PIN_4) +
        			(Digitize(GPIO_PORTC_BASE, GPIO_PIN_5) << 1) +
					(Digitize(GPIO_PORTB_BASE, GPIO_PIN_1) << 2) +
					(Digitize(GPIO_PORTB_BASE, GPIO_PIN_0) << 3) +
					(Digitize(GPIO_PORTB_BASE, GPIO_PIN_2) << 4) +
					(Digitize(GPIO_PORTB_BASE, GPIO_PIN_3) << 5) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_4) << 6) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_5) << 7);
        PORTEMBTI = Digitize(GPIO_PORTF_BASE, GPIO_PIN_4) +
        			(Digitize(GPIO_PORTF_BASE, GPIO_PIN_5) << 1) +
					(Digitize(GPIO_PORTK_BASE, GPIO_PIN_0) << 2) +
					(Digitize(GPIO_PORTK_BASE, GPIO_PIN_1) << 3) +
					(Digitize(GPIO_PORTD_BASE, GPIO_PIN_3) << 4) +
					(Digitize(GPIO_PORTD_BASE, GPIO_PIN_2) << 5) +
					(Digitize(GPIO_PORTH_BASE, GPIO_PIN_6) << 6) +
					(Digitize(GPIO_PORTH_BASE, GPIO_PIN_7) << 7);
        PORTFMBTI = Digitize(GPIO_PORTK_BASE, GPIO_PIN_3) +
        			(Digitize(GPIO_PORTJ_BASE, GPIO_PIN_2) << 1) +
					(Digitize(GPIO_PORTD_BASE, GPIO_PIN_1) << 2) +
					(Digitize(GPIO_PORTK_BASE, GPIO_PIN_2) << 3);
        PORTGMBTI = Digitize(GPIO_PORTE_BASE, GPIO_PIN_1) +
        			(Digitize(GPIO_PORTE_BASE, GPIO_PIN_0) << 1) +
					(Digitize(GPIO_PORTF_BASE, GPIO_PIN_6) << 7);
        PORTHMBTI = Digitize(GPIO_PORTD_BASE, GPIO_PIN_7) +
        			(Digitize(GPIO_PORTD_BASE, GPIO_PIN_6) << 1) +
					(Digitize(GPIO_PORTD_BASE, GPIO_PIN_5) << 2) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_7) << 3) +
					(Digitize(GPIO_PORTG_BASE, GPIO_PIN_6) << 4) +
					(Digitize(GPIO_PORTE_BASE, GPIO_PIN_2) << 5) +
					(Digitize(GPIO_PORTE_BASE, GPIO_PIN_3) << 6) +
					(Digitize(GPIO_PORTJ_BASE, GPIO_PIN_0) << 7);
        PORTJMBTI = Digitize(GPIO_PORTJ_BASE, GPIO_PIN_1) +
        			(Digitize(GPIO_PORTD_BASE, GPIO_PIN_0) << 1);
        /* Porting Compatibility with MBTI ^ */

    	// UART Communication Subroutine
        if(DataLock == FALSE)
        {   //Init Page ComOUT[0] - ComOUT[19]
            DataReady = FALSE;
            ComOUT[0]= 0xF0;              // Start
            // OUT: Position: Station No = 0
            ComOUT[1]= 0x01;
            // OUT: Function: Number 3
            ComOUT[2]= 0x03;
            // OUT : END
            ComOUT[3] = 0x0F;
            ComOUT[4] = 0x0F;

            //Main Page ComOUT[20] - ComOUT[39]
            ComOUT[20] = 0xF0;              // Start
            // OUT: Position: Station No = 0
            ComOUT[21] = 0x01;
            // OUT: Function: Number 3
            ComOUT[22] = 0x03;
            // OUT: Data
            ComOUT[23] = EyemarkON + (DatecodeON << 1) + (VibratorFormerON << 2) + (ClutchON << 3) + (BreakON << 4) + (PulseErrorMsg << 5) + (PulseCheckOK << 6) + (FoilJamMsg << 7);
            ComOUT[24] = MachineRunning + (ServiceMode << 1) + (HeaterErrorMsg << 2) + (HopperError << 3) + (EyemarkError << 4) + (ZIError << 5) + (DoorOpen << 6) + (CounterCutterON << 7);
            ComOUT[25] = ZeroIndexON + (BlinkCom << 1) + (CheckPulseMode << 2) + (ProductCountBit << 3) + (PulseCommutationError << 4) + (DWLimFoil << 5) + (FWLimFoil << 6) + (Initialised << 7);
            ComOUT[26] = SpeedperMin;
            ComOUT[27] = CounterCutterCounter;
            ComOUT[28] = ZIErrorCounter;
            ComOUT[29] = EyemarkErrorCounter;
            ComOUT[30] = Heater1ON + (Heater2ON << 1) + (Heater3ON << 2) + (Heater4ON << 3) + (Heater1Ready << 4) + (Heater2Ready << 5) + (Heater3Ready << 6) + (Heater4Ready << 7);

            if(!HeaterNoConnection)
            {
                ComOUT[31] = Heater1PV;
                ComOUT[32] = Heater2PV;
                ComOUT[33] = Heater3PV;
                ComOUT[34] = Heater4PV;
            }
            ComOUT[35] = TROCoil + (TRICoil << 1) + (TRIOnDelay << 2) + (TROLaunched << 3) + (HeaterNoConnection << 4);
            ComOUT[36] = EncoderData >> 8;
            ComOUT[37] = EncoderData;
            ComOUT[38] = ServoError + (ServoReady << 1) + (AgitatorON << 2) + (AgitatorError << 3) + (SPIComError << 4) + (NoAirErrorMsg << 5) + (DatesensError << 6) + (ServoValveErrorMsg << 7);
            ComOUT[39] = ZeroIndex32 + (ZeroIndex16 << 1) + (ZeroIndex8 << 2) + (ZeroIndex4 << 3) + (ZeroIndex2 << 4) + (ZeroIndex1 << 5) + (IsMBTITMPCB << 6);
            // All Pins
            if(IsMBTITMHMI)
            {
				ComOUT[40] = ROM_GPIOPinRead(GPIO_PORTA_BASE, 0xFF);
				ComOUT[41] = ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xFF);
				ComOUT[42] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xFF);
				ComOUT[43] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xFF);
				ComOUT[44] = ROM_GPIOPinRead(GPIO_PORTE_BASE, 0xFF);
				ComOUT[45] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0xFF);
				ComOUT[46] = ROM_GPIOPinRead(GPIO_PORTG_BASE, 0xFF);
				ComOUT[47] = ROM_GPIOPinRead(GPIO_PORTH_BASE, 0xFF);
				ComOUT[48] = ROM_GPIOPinRead(GPIO_PORTJ_BASE, 0xFF);
            }
            else
            {
				ComOUT[40] = PORTAMBTI;
				ComOUT[41] = PORTBMBTI;
				ComOUT[42] = PORTCMBTI;
				ComOUT[43] = PORTDMBTI;
				ComOUT[44] = PORTEMBTI;
				ComOUT[45] = PORTFMBTI;
				ComOUT[46] = PORTGMBTI;
				ComOUT[47] = PORTHMBTI;
				ComOUT[48] = PORTJMBTI;
            }
            ComOUT[49] = ServoTimerCounter >> 8;
            ComOUT[50] = ServoTimerCounter;
            ComOUT[51] = HopperErrorCounter >> 8;
            ComOUT[52] = HopperErrorCounter;
            ComOUT[53] = EyemarkPosition;
            ComOUT[54] = PulseSum >> 8;
            ComOUT[55] = PulseSum;
            ComOUT[56] = LifeDuration >> 24;
            ComOUT[57] = LifeDuration >> 16;
            ComOUT[58] = LifeDuration >> 8;
            ComOUT[59] = LifeDuration;
            ComOUT[60] = 27;									// Program Version
            ComOUT[61] = HopperBufferSensorCounter;
            ComOUT[62] = ExternalFeederDurationCounter;
            ComOUT[63] = ExternalFeederOFFTimerCounter;
			ComOUT[64] = 0;
			ComOUT[65] = 0;
			ComOUT[66] = 0;
			ComOUT[67] = 0;
			if(IsMBTITMHMI)
				ComOUT[68] = ROM_GPIOPinRead(GPIO_PORTK_BASE, 0xFF);
			else
				ComOUT[68] = 0;

            // OUT : END
            ComOUT[69] = 0x0F;
            ComOUT[70] = 0x0F;
            DataReady = TRUE;
        }

    	// SPI Communication Subroutine
        if(!SPIDataLock)
        {
            SPIDataReady = FALSE;
            SPIOut[0] = 0xF0;
            SPIOut[1] = 0x64;
            SPIOut[2] = 0x04;
            SPIOut[3] = Heater1Enable + ( Heater2Enable << 1 ) + ( Heater3Enable << 2 ) + ( Heater4Enable << 3 ) + ( PIDmultiplier << 4 );
            //if(YesHeaterNoConnection)                                   // When Heater Putus occured, put all SV to zero to disable the heater relay/ssr output
            //{
            //    SPIOut[4] = 0;
            //    SPIOut[5] = 0;
            //    SPIOut[6] = 0;
            //    SPIOut[7] = 0;
            //}
            //else
            //{
                SPIOut[4] = Heater1SV;
                SPIOut[5] = Heater2SV;
                SPIOut[6] = Heater3SV;
                SPIOut[7] = Heater4SV;
            //}
            SPIOut[8]  = ComIN[26];
            SPIOut[9]  = ComIN[27];
            SPIOut[10] = ComIN[28];
            SPIOut[11] = ComIN[29];
            SPIOut[12] = ComIN[30];
            SPIOut[13] = ComIN[31];
            SPIOut[14] = ComIN[32];
            SPIOut[15] = ComIN[33];
            SPIOut[16] = ComIN[34];
            SPIOut[17] = ComIN[35];
            SPIOut[18] = ComIN[36];
            SPIOut[19] = ComIN[37];
            SPIOut[20] = ComIN[38];
            SPIOut[21] = ComIN[39];
            SPIOut[22] = ComIN[40];
            SPIOut[23] = ComIN[41];
            SPIOut[24] = 0x0F;
            SPIOut[25] = 0x0F;
            SPIDataReady = TRUE;
        }
    }
}
