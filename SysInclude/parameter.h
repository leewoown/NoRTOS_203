/* ==============================================================================
System Name:  현대자동차 수소 지게차 80V

File Name:		PARAMETER.H

Description:	현대
          	    Orientation Control for a Three Phase AC Induction Motor. 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DMC1500 board.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
=================================================================================  */
//#include "build.h"
//#include "math.h"
//#include "IQmathLib.h"
#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"
#ifndef PARAMETER_H
#define PARAMETER_H

/* Bit 위치 정의(주로 위에서 정의한 매크로 함수에서 사용하기 위해 정의함) */
#define BIT0_POS    	0
#define BIT1_POS    	1
#define BIT2_POS    	2
#define BIT3_POS    	3
#define BIT4_POS    	4
#define BIT5_POS    	5
#define BIT6_POS    	6
#define BIT7_POS    	7
#define BIT8_POS    	8
#define BIT9_POS    	9
#define BIT10_POS   	10
#define BIT11_POS   	11
#define BIT12_POS   	12
#define BIT13_POS   	13
#define BIT14_POS   	14
#define BIT15_POS   	15

/* Bit Mask Data 정의 */
#define	BIT0_MASK    	0x0001
#define	BIT1_MASK    	0x0002
#define	BIT2_MASK    	0x0004
#define	BIT3_MASK    	0x0008
#define	BIT4_MASK    	0x0010
#define	BIT5_MASK    	0x0020
#define	IT6_MASK    	0x0040
#define	BIT7_MASK    	0x0080
#define	BIT8_MASK    	0x0100
#define	BIT9_MASK    	0x0200
#define	BIT10_MASK   	0x0400
#define	BIT11_MASK   	0x0800
#define	BIT12_MASK   	0x1000
#define	BIT13_MASK   	0x2000
#define BIT14_MASK   	0x4000
#define BIT15_MASK   	0x8000

#define	SCIA_BUFRX		50			// Monstar와 맞추어야 함

#define UL_BYTE(x)		    (x >> 16)
#define HI_BYTE(x)		    (x >> 8)
#define LO_BYTE(x)          (x & 0xff)
#define MAKE_WORD(msb,lsb)	((msb<<8) | (lsb))
#define WordLShift(md,ml)   (md<<ml)
#define WordRShift(md,ml)   (md>>ml)



// Define the ISR frequency (kHz)
//#define ISR_FREQUENCY 	    10
//#define SYSTEM_FREQUENCY    90
//#define CPUCLK			    80000000L					// CPU Main Clock
//#define CPLDCLK			    100000000L					// CPLD Clock

//#define CPU_CLOCK_SPEED     6.6667L   			    // for a 150MHz CPU clock speed
//#define CPU_CLOCK_SPEED     11.111L                     // for a 90MHz CPU clock speed
//#define ADC_usDELAY 	    5000L



#define	Uint16Max		    65536


/*
#define TxA_RDY_flag        SciaRegs.SCICTL2.bit.TXRDY
#define TxB_RDY_flag	    ScibRegs.SCICTL2.bit.TXRDY
#define TxC_RDY_flag        ScicRegs.SCICTL2.bit.TXRDY
#define TxA_Empty_flag      SciaRegs.SCICTL2.bit.TXEMPTY
#define TxB_Empty_flag      ScibRegs.SCICTL2.bit.TXEMPTY
#define TxC_Empty_flag	    ScicRegs.SCICTL2.bit.TXEMPTY
#define AD_START   	   	    AdcRegs.ADCTRL2.bit.SOC_SEQ1
#define IS_AD_BUSY 	        AdcRegs.ADCST.bit.SEQ1_BSY
*/

/*
 * LED00 indicates SYSTEM STATE status
 */
#define LEDSysState_H              GpioDataRegs.GPBSET.bit.GPIO58=1 //System Fault State  LED
#define LEDSysState_L              GpioDataRegs.GPBCLEAR.bit.GPIO58=1
#define LEDSysState_T              GpioDataRegs.GPBTOGGLE.bit.GPIO58=1

/*
 *  LED01 indicates SYSTEM Fault status
 */
#define LEDFault_H              GpioDataRegs.GPBSET.bit.GPIO44=1
#define LEDFault_L              GpioDataRegs.GPBCLEAR.bit.GPIO44=1
#define LEDFault_T              GpioDataRegs.GPBTOGGLE.bit.GPIO44=1

/*
 * LED02 indicates SYSTEM CANSTAT status
 */

#define LEDCANState_H            GpioDataRegs.GPBSET.bit.GPIO51=1
#define LEDCANState_L            GpioDataRegs.GPBCLEAR.bit.GPIO51=1
#define LEDCANState_T            GpioDataRegs.GPBTOGGLE.bit.GPIO51=1


/*
 * DIP SW
 */
#define IDSW00           GpioDataRegs.GPADAT.bit.GPIO16
#define IDSW01           GpioDataRegs.GPADAT.bit.GPIO17
//#define IDSW02           GpioDataRegs.GPADAT.bit.GPIO18
#define IDSW03           GpioDataRegs.GPADAT.bit.GPIO19
/*
 * SPI chip RTC CS, MFP DIO
 */

#define RTC_CS            GpioDataRegs.GPACLEAR.bit.GPIO9=1
#define RTC_DS            GpioDataRegs.GPASET.bit.GPIO9=1
#define RTC_MF            GpioDataRegs.GPCDAT.bit.GPIO8
/*
 * SPI chip NVRAM CS
 */
#define NvramCS            GpioDataRegs.GPBCLEAR.bit.GPIO11=1
#define NvramDS            GpioDataRegs.GPBSET.bit.GPIO11=1

/*
 *  SPI TCP IP CS, REST, RDY, INT
 */
#define TcpCS             GpioDataRegs.GPBCLEAR.bit.GPIO42=1
#define TcpDS             GpioDataRegs.GPBSET.bit.GPIO42=1

#define TcpResetOn        GpioDataRegs.GPACLEAR.bit.GPIO15=1
#define TcpResetOff       GpioDataRegs.GPASET.bit.GPIO15=1

#define TcpINT            GpioDataRegs.GPADAT.bit.GPIO13

/*
 *  SPI CAN FOR EN, CANINT, CANRX0INT, CANRX1INT,
 */
#define CANBCS             GpioDataRegs.GPBCLEAR.bit.GPIO43=1
#define CANBDS             GpioDataRegs.GPBSET.bit.GPIO43=1

#define CANINT             GpioDataRegs.GPADAT.bit.GPIO12=1
#define CANRX0INT          GpioDataRegs.GPADAT.bit.GPIO14
#define CANRX1INT          GpioDataRegs.GPBDAT.bit.GPIO52

/*
 * RS845 EN
 */
#define RS485EN            GpioDataRegs.GPBCLEAR.bit.GPIO50=1
#define RS485DS            GpioDataRegs.GPBSET.bit.GPIO50=1
/*
 *  BAT IC EN
 */
#define BATEN             GpioDataRegs.GPACLEAR.bit.GPIO10=1
#define BATDS             GpioDataRegs.GPASET.bit.GPIO10=1

/*
 * 80VBAT PROTECT Relay OUT, AUX
 */

#define PRlyOn              GpioDataRegs.GPASET.bit.GPIO22=1
#define PRlyOff             GpioDataRegs.GPACLEAR.bit.GPIO22=1
#define PRlyState           GpioDataRegs.GPADAT.bit.GPIO23

#define NRlyOn              GpioDataRegs.GPASET.bit.GPIO6=1
#define NRlyOff             GpioDataRegs.GPACLEAR.bit.GPIO6=1
#define NRlyState           GpioDataRegs.GPADAT.bit.GPIO7

#define PRORlyOn            GpioDataRegs.GPBSET.bit.GPIO32=1
#define PRORlyOff           GpioDataRegs.GPBCLEAR.bit.GPIO32=1
#define PRORlyState         GpioDataRegs.GPBDAT.bit.GPIO33


#define LatchSetRlyON       GpioDataRegs.GPASET.bit.GPIO20=1;
#define LatchSetRlyOFF      GpioDataRegs.GPACLEAR.bit.GPIO20=1

#define LatchResetRlyON     GpioDataRegs.GPASET.bit.GPIO21=1
#define LatchResetRlyOFF    GpioDataRegs.GPACLEAR.bit.GPIO21=1

/*
 * Insulation Resistance Measurement, IMDTopON. IMDTopOFF,IMDBOTOn,IMDBOTOff
 */

#define IMDTOPOn           GpioDataRegs.GPASET.bit.GPIO26=1
#define IMDTopOff          GpioDataRegs.GPACLEAR.bit.GPIO26=1
#define IMDBOTOn           GpioDataRegs.GPASET.bit.GPIO27=1
#define IMDBOTOff          GpioDataRegs.GPACLEAR.bit.GPIO27=1


// Bit 연산시 일반적으로 쓰이는 부분을 매크로 함수로 정의함  

#define BIT_MASK(bit)			(1 << (bit))
#define GetBit(val, bit)		(((val) & BIT_MASK(bit)) >> (bit))
#define SetBit(val, bit)		(val |= BIT_MASK(bit))
#define ClearBit(val, bit)		(val &= ~BIT_MASK(bit))
#define ToggleBit(val, bit)		(val ^= BIT_MASK(bit))
#define bit_is_set(val, bit)	(val & BIT_MASK(bit))
#define bit_is_clear(val, bit)	(~val & BIT_MASK(bit))

//------------------------------------------------------------------------------------------------
//#define A_PTR(y)			*(volatile unsigned int *)(y)
//#define FPGA_Addr(X)		*(volatile unsigned int *)(0x4000+X) 
//#define PARA_Addr(X)		*(volatile unsigned int *)(0x4200+X)

#define PBYTE(X)                *(volatile unsigned char      *)(X)
#define PWORD(X)                *(volatile unsigned int       *)(X)
#define PLONG(X)                *(volatile unsigned long      *)(X)
#define PLLONG(X)               *(volatile unsigned long long *)(X)



/*-------------------------------------------------------------------------------
 TMS320F28069 CLK SET UP
-------------------------------------------------------------------------------*/
#define	CPUCLK				    80000000L							// CPU Main Clock
/*-------------------------------------------------------------------------------
 TMS320F28335 CLK SET UP 
-------------------------------------------------------------------------------*/
#define	SCIA_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-A
#define	SCIA_BAUDRATE			9600L								// SCI-A Baudrate
#define	SCIA_BRR_VAL			(SCIA_LSPCLK/(8*SCIA_BAUDRATE)-1)	// SCI-A BaudRate 설정 Register 값

#define	SCIB_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-B
#define	SCIB_BAUDRATE			9600L								// SCI-B Baudrate
#define	SCIB_BRR_VAL			(SCIB_LSPCLK/(8*SCIB_BAUDRATE)-1)	// SCI-B BaudRate 설정 Register 값

#define	SCIC_LSPCLK				(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-C
#define	SCIC_BAUDRATE			9600L								// SCI-C Baudrate
#define	SCIC_BRR_VAL			(SCIC_LSPCLK/(8*SCIC_BAUDRATE)-1)	// SCI-C BaudRate 설정 Register 값

/*-------------------------------------------------------------------------------
Parameter
-------------------------------------------------------------------------------*/

// Define the Power Source Parameter
#define PI 							3.14159265358979
#define PIn							-3.14159265358979
#define PI2							6.283185307
#define WE							376.9911184
#define AdcNormalizerBipolar        0.00048828125           // 1 / 4096으로 나눗값
#define AdcNormalizerUnipolar       0.000244140625          // 1 / 2048으로 나눗값
#define AdcNormalizerpolar          0.000322997416          // 1 / 3096으로 나눗값
#define Inverse3					0.333333333			//1/3
#define InverseSQRT3				0.577350269			//1/root3
#define SQRT3						1.732050808			//root3
#define WL_Grid				 		0.150796447368		// 2pi * 60Hz * 400uH
#define TwoBySQRT3 					1.154700			// TwoBySQRT3   = 2/root3
#define Vdc_Minimum					50.0
#define Inverse_Vdc_Minimum			0.02
#define SectoHour                   0.00027778 // 1(h)/3600(sec)
#define Func_Hz                     20 // 1(h)/3600(sec)


/*
 * 162S1P, BATTERY PACK Protect Parameter setup
 */

//#define     Pack_ID                     1


/*
 *
    000 (0) : Battery system initial
    001 (1) : Battery System Ready
    010 (2) : Battery system StandBY
    011 (3) : Battery system discharging
    100 (4) : Battery system Balancing
 */

#define     SysRegTimer5msec     4
#define     SysRegTimer10msec    9
#define     SysRegTimer50msec    49
#define     SysRegTimer100msec   99
#define     SysRegTimer300msec   299
#define     SysRegTimer500msec   499
#define     SysRegTimer1000msec  1000
#define     CellVoltSampleTime   100
#define     CellTempSampleTime   250

#define     Product_Type                       3   // 24.09.21
#define     Product_Version                    10   // 24.09.21
#define     Product_SysCellVauleS              15
#define     Product_SysCellVauleP              2
#define     Product_Voltage                    512 // 3.664*24
#define     Product_Capacity                   460  //


#define     C_PackVoltMax                   540 //3.65*24
#define     C_PackVoltMin                   450 //3.0*2.4


#define     C_CellShutdownFault             2.6
#define     C_CellBalanLimtVolt             3.0
#define     C_BalanceDivVoltage             0.01
#define     C_PackBalanCurrent              5.0



#define     C_CTDirection                 1.0
#define     C_Cell_Capacity               52

#define     C_SysModuleEa                 2 // Module 개수를  정의함
//#define     C_SlaveMEAEa                  7 // Slave BMS에서 측정하는 전압 및 온도 개수 정의함
#define     C_ModuleMEAEa                 15 // Module에서 측정하는 셀 전압 및 온도 개수를 정의함
#define     C_SlaveBMSEa                  4

#define     C_SysCellVoltEa               30//C_SysModuleEa*C_ModuleMEAEa // Battery Pack 내 Cell 전압 EA
#define     C_SysCellTempEa               30//C_SysModuleEa*C_ModuleMEAEa // Battery Pack 내 Cell 온도 EA
#define     C_CellNum                     30
#define     C_HmiCellVoltCount            9//C_SysCellVoltEa/3 //CAN TX  전송 위한 Cell 전압 개수 정의함
#define     C_HmiCellTempCount            9//C_SysCellTempEa/3 //CAN TX  전송 위한 Cell 온도 개수 정의함
#define     C_HMIISOSPIErrCount           C_SlaveBMSEa/3


#define     C_SysNoramlState              0
#define     C_SysAlarmlState              1
#define     C_SysAlertState               2
#define     C_SysFaultState               3



// Alarm Set Vaule  //
#define     C_Bat80VOVPackCurrentAlarm                     450.0//450.0//480.0
#define     C_Bat80VOVPkACKSOCAlarm                        100.0
#define     C_Bat80VUDPkACKSOCAlarm                        5.0
#define     C_Bat80VOVPackVoltageAlarm                     108.8   // Cell 4.20V * 24
#define     C_Bat80VUDPackVoltageAlarm                     72.0   // Cell 3.00V * 24
#define     C_Bat80VOVPackTemperatureAlarm                 55.0
#define     C_Bat80VUNPackTemperatureAlarm                -15.0
#define     C_Bat80VOVCellVoltageAlarm                     4.20
#define     C_Bat80VUDCellVoltageAlarm                     3.00
#define     C_Bat80VDIVCellVoltageAlarm                    0.2
#define     C_Bat80VOVCellTemperatureAlarm                 55.0
#define     C_Bat80VUDCellTemperatureAlarm                -15.0
#define     C_Bat80VDIVCellTemperatureAlarm                10.0


//Fault Set Vaule
#define     C_PackDISCH_OCFault                     500.0//500.0
#define     C_PackCHARG_OCFault                     180.0//500.0
#define     C_PackSOC_OVFault                       101.0
#define     C_PackSOC_UNFault                       -0.1//-0.1
#define     C_PackVolt_OVFault                      58.5 // Cell 4.25V * 24
#define     C_PackVolt_UNFault                      40.6 // Cell 2.80V * 24
#define     C_CellVolt_OVFault                      4.25
#define     C_CellVolt_UNFault                      2.80
#define     C_CellVolt_UBFault                      0.5
#define     C_CellTempsDISCH_OTFault                60.0
#define     C_CellTempsCHARG_OTFault                50.0
#define     C_CellTempsDISCH_UTFault                -25.0
#define     C_CellTempsCHARG_UTFault                0.0
#define     C_CellTemps_UBFault                     15.0
#define     C_PackPWRDISCH_UBFault                  2.0//kW
#define     C_PackPWRCHARG_UBFault                  0.5//kW
#define     C_PackISORegsister_URFault              50.0//MOhm


//Protect Set Vaule
#define     C_PackDISCH_OCPrtct                     500.0//500.0
#define     C_PackCHARG_OCPrtct                     180.0//500.0
#define     C_PackSOC_OVPrtct                       101.0
#define     C_PackSOC_UNPrtct                       -0.1//-0.1
#define     C_PackVolt_OVPrtct                      58.5 // Cell 4.25V * 24
#define     C_PackVolt_UNPrtct                      40.6 // Cell 2.80V * 24
#define     C_CellVolt_OVPrtct                      4.25
#define     C_CellVolt_UNPrtct                      2.80
#define     C_CellVolt_UBPrtct                      0.5
#define     C_CellTempsDISCH_OTPrtct                60.0
#define     C_CellTempsCHARG_OTPrtct                50.0
#define     C_CellTempsDISCH_UTPrtct                -25.0
#define     C_CellTempsCHARG_UTPrtct                0.0
#define     C_CellTemps_UBPrtct                     15.0
#define     C_PackPWRDISCH_UBPrtct                  2.0//kW
#define     C_PackPWRCHARG_UBPrtct                  0.5//kW
#define     C_PackISORegsister_URPrtct              50.0//MOhm


//Fault Delay Time 1 = 1msec
#define     C_PackDISCHOC_PrtectDelay                  1
#define     C_PackCHARGOC_PrtectDelay                  0
#define     C_PackSOCOV_PrtectDelay                    0
#define     C_PackSOCUN_PrtectDelay                    0
#define     C_PackVoltOV_PrtectDelay                   0
#define     C_PackVoltUV_PrtectDelay                   0
#define     C_CellVoltOV_PrtectDelay                   0
#define     C_CellVoltUN_PrtectDelay                   0
#define     C_CellVoltUB_PrtectDelay                   0
#define     C_CellTempsDISCHOT_PrtectDelay             0
#define     C_CellTempsCHARGOT_PrtectDelay             0
#define     C_CellTempsDISCHUT_PrtectDelay             0
#define     C_CellTempsCHARGUT_PrtectDelay             0
#define     C_CellTempsUB_PrtectDelay                  0
#define     C_PackPWRCHARGUB_PrtectDelay               0
#define     C_PackPWRDISCHUB_PrtectDelay               0
#define     C_ISOSPIPrtectCont                         200


#define SectoHour                   0.00027778 // 1(h)/3600(sec)
#define Func_Hz                     20 // 1(h)/3600(sec)
#define SocCumulativeTime           0.00027778 //1/3600
#define SocCurrentSampleTime        0.05


#endif  // end of PARAMETER.H definition



//===========================================================================
// No more.
//===========================================================================
