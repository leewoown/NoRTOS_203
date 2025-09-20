
//###########################################################################
//
// FILE:   DSP28x_Project.h
//
// TITLE:  DSP28x Project Headerfile and Examples Include File
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2009-2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#ifndef DSP28x_PROJECT_H
#define DSP28x_PROJECT_H

//
// Included Files
//
#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"   	// F2806x Examples Include File
#include "parameter.h"

//EDLAY 매크로 선언 --------------------------------------------------------------------//

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define delay_us(us)        DELAY_US(us)
// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define delay_ms(ms)        DELAY_US(ms*1000)
//#define  SPI_Read()          SPI_READ()
/*-------------------------------------------------------------------------------
Next, definitions used in main file.
-------------------------------------------------------------------------------*/
#define TRUE    1
#define FALSE   0
#define TRUE    1
#define ON      1
#define OFF     0
#define         Shift_RIGHT(val, bit)           ((val) >> (al))
#define         Shift_LEFT(val,  bit)           ((val) << (val))
#define         ComBine(Val_H, Val_L)           (((Val_H) << 8) | (Val_L))
#define         BetweenRange(val, Val_L, Val_H) ((val) >= (Val_L) && (val) <= (Val_H))
#define         BIT_MASK(bit)                   (1 << (bit))
#define         GetBit(val, bit)                (((val) & BIT_MASK(bit)) >> (bit))
#define         SetBit(val, bit)                (val |= BIT_MASK(bit))
#define         ClearBit(val, bit)              (val &= ~BIT_MASK(bit))
#define         ToggleBit(val, bit)             (val ^= BIT_MASK(bit))
#define         bit_is_set(val, bit)            (val & BIT_MASK(bit))
#define         bit_is_clear(val, bit)          (~val & BIT_MASK(bit))

#define IS_OVER_AND_UNDER(A, MIN, MAX)   ((A) >= (MIN) && (A) <= (MAX))  // 이상 ~ 이하
#define IS_ABOVE_AND_UNDER(A, MIN, MAX)  ((A) >  (MIN) && (A) <= (MAX))  // 초과 ~ 이하
#define IS_OVER_AND_BELOW(A, MIN, MAX)   ((A) >= (MIN) && (A) <  (MAX))  // 이상 ~ 미만
#define IS_ABOVE_AND_BELOW(A, MIN, MAX)  ((A) >  (MIN) && (A) <  (MAX))  // 초과 ~ 미만

struct Data_WORD
{
    unsigned int DataL;
    unsigned int DataH;
};

struct ParentDeviceCMD_BIT
{
    // bits   description
   unsigned int     POWEREN                 :1;   // 0
   unsigned int     SW01                    :1;   // 2
   unsigned int     SW02                    :1;   // 3
   unsigned int     SW03                    :1;   // 4
   unsigned int     SW04                    :1;   // 5
   unsigned int     SW05                    :1;   // 6
   unsigned int     SW06                    :1;   // 6
   unsigned int     SW07                    :1;   // 7
   unsigned int     SW08                    :1;   // 8
   unsigned int     SW09                    :1;   // 9
   unsigned int     SW10                    :1;   // 11
   unsigned int     SW11                    :1;   // 12
   unsigned int     SW12                    :1;   // 13
   unsigned int     SW13                    :1;   // 14
   unsigned int     SW14                    :1;   // 15
   unsigned int     SW15                    :1;   // 16
};
union ParentDeviceCMD_REG
{
   unsigned int     all;
   struct ParentDeviceCMD_BIT bit;
};
struct IDSW_BIT
{       // bits   description
   unsigned int     SW00                    :1;   // 0
   unsigned int     SW01                    :1;   // 1
   unsigned int     SW02                    :1;   // 2
   unsigned int     SW03                    :1;   // 3
   unsigned int     SW04                    :1;   // 4
   unsigned int     SW05                    :1;   // 5
   unsigned int     SW06                    :1;   // 6
   unsigned int     SW07                    :1;   // 7
   unsigned int     SW08                    :1;   // 8
   unsigned int     SW09                    :1;   // 9
   unsigned int     SW10                    :1;   // 10
   unsigned int     SW11                    :1;   // 11
   unsigned int     SW12                    :1;   // 12
   unsigned int     SW13                    :1;   // 13
   unsigned int     SW14                    :1;   // 14
   unsigned int     SW15                    :1;   // 15

};
union IDSW_REG
{
   unsigned int     all;
   struct IDSW_BIT bit;
};
struct DigitalInPut_BIT
{       // bits   description
   unsigned int     PAUX                    :1;   // 0
   unsigned int     NAUX                    :1;   // 1
   unsigned int     EMGSWStauts             :1;   // 2
   unsigned int     CANRX0                  :1;   // 3
   unsigned int     CANRX1                  :1;   // 4
   unsigned int     SW05                    :1;   // 5
   unsigned int     SW06                    :1;   // 6
   unsigned int     SW07                    :1;   // 7
   unsigned int     SW08                    :1;   // 8
   unsigned int     SW09                    :1;   // 9
   unsigned int     SW10                    :1;   // 10
   unsigned int     SW11                    :1;   // 11
   unsigned int     SW12                    :1;   // 12
   unsigned int     SW13                    :1;   // 13
   unsigned int     SW14                    :1;   // 14
   unsigned int     SW15                    :1;   // 15

};
union DigitalInput_REG
{
   unsigned int     all;
   struct DigitalInPut_BIT bit;
};

struct DigitalOutPut_BIT
{       // bits   description
    unsigned int     NRlyOUT                :1; // 0
    unsigned int     PRlyOUT                :1; // 1
    unsigned int     ProRlyOUT              :1; // 2
    unsigned int     StartBATOUT            :1; // 3
    unsigned int     IMDTOPOUT              :1; // 4
    unsigned int     IMDBOTOUT              :1; // 5
    unsigned int     LEDSysOUT              :1; // 6
    unsigned int     LEDCAnOUT              :1; // 7
    unsigned int     LEDAlarmOUT            :1; // 8
    unsigned int     LEDFaultOUT            :1; // 9
    unsigned int     LEDProtectOUT          :1; // 10
    unsigned int     PWRLAMPOUT             :1; // 11
    unsigned int     EMGSWLAMPOUT           :1; // 12
    unsigned int     PWRHOLD                :1; // 13
    unsigned int     DO014                  :1; // 14
    unsigned int     DO015                  :1; // 15
};
union DigitalOutPut_REG
{
   unsigned int     all;
   struct DigitalOutPut_BIT bit;
};
typedef enum
{
   System_STATE_INIT,
   System_STATE_STANDBY,
   System_STATE_READY,
   System_STATE_RUNING,
   System_STATE_PROTECTER,
   System_STATE_DATALOG,
   System_STATE_ProtectHistory,
   System_STATE_MANUALMode,
   System_STATE_CLEAR
} SysState;
struct SystemState_BIT
{       // bits   description
    unsigned int     SysStatus              :2; // 0,1
    unsigned int     SysProtectStatus       :2; // 2,3
    unsigned int     Systate                :3; // 4,5,6
    unsigned int     SysRlyStatus           :3; // 7,8,9
    unsigned int     SysSOCStatus           :2; // 10,11
    unsigned int     SysSocMode             :1; // 12
    unsigned int     INITOK                 :1; // 13
    unsigned int     SysBalanceEn           :1; // 14
    unsigned int     SysBalanceMode         :1; // 15
    unsigned int     SysDisCharMode         :1; // 16
    unsigned int     SysChargerEn           :1; // 17
    unsigned int     SysAalarm              :1; // 18
    unsigned int     SysProtect             :1; // 19
    unsigned int     NRlyDOStatus           :1; // 20
    unsigned int     PRlyDOStatus           :1; // 21
    unsigned int     PreRlyDOStatus         :1; // 22
    unsigned int     PwrHoldRlyDOStatus     :1; // 23
    unsigned int     MSDERR                 :1; // 24
    unsigned int     RlyERR                 :1; // 25
    unsigned int     HMICOMEnable           :1; // 26
    unsigned int     HMIBalanceMode         :1; // 27
    unsigned int     CANCOMERR              :1; // 28
    unsigned int     ISOSPICOMERR           :1; // 29
    unsigned int     CellVoltOk             :1; // 30
    unsigned int     CellTempsOk            :1; // 31
//    unsigned int     ISORegERR              :1; // 27
//    unsigned int     RlyERR                 :1; // 29

};
union SystemState_REG
{
    struct SystemState_BIT   bit;
    struct Data_WORD        Word;
    Uint32                   all;
};
struct SystemAlarm_BIT
{       // bits   description
    unsigned int     PackVDisChar_OC            :1; // 0
    unsigned int     PackVChar_OC               :1; // 1
    unsigned int     PackVSOC_OV                :1; // 2
    unsigned int     PackVSOC_UN                :1; // 3
    unsigned int     PackVolt_OV                :1; // 4
    unsigned int     PackVolt_UN                :1; // 5
    unsigned int     CellVolt_OV                :1; // 6
    unsigned int     CellVolt_UN                :1; // 7
    unsigned int     CellVolt_BL                :1; // 8
    unsigned int     CellTempsDisch_OT          :1; // 9
    unsigned int     CellTempsCharg_OT          :1; // 10
    unsigned int     CellTempsDisCh_UT          :1; // 11
    unsigned int     CellTempsCharg_UT          :1; // 12
    unsigned int     CellTemp_BL                :1; // 13
    unsigned int     PackDischarUnPWR_BL        :1; // 14
    unsigned int     PackCharUnPWR_BL           :1; // 15
    unsigned int     SW16                       :1; // 16
    unsigned int     SW17                       :1; // 17
    unsigned int     SW18                       :1; // 18
    unsigned int     SW19                       :1; // 19
    unsigned int     SW20                       :1; // 20
    unsigned int     SW21                       :1; // 21
    unsigned int     SW22                       :1; // 22
    unsigned int     SW23                       :1; // 23
    unsigned int     SW24                       :1; // 24
    unsigned int     SW25                       :1; // 25
    unsigned int     SW26                       :1; // 26
    unsigned int     SW27                       :1; // 27
    unsigned int     SW28                       :1; // 28
    unsigned int     SW29                       :1; // 29
    unsigned int     SW30                       :1; // 30
    unsigned int     SW31                       :1; // 31
};
union SystemAlarm_REG
{
    struct SystemAlarm_BIT   bit;
    struct Data_WORD        Word;
    Uint32                   all;
};
struct SystemFault_BIT
{       // bits   description
    unsigned int     PackVDisChar_OC            :1; // 0
    unsigned int     PackVChar_OC               :1; // 1
    unsigned int     PackVSOC_OV                :1; // 2
    unsigned int     PackVSOC_UN                :1; // 3
    unsigned int     PackVolt_OV                :1; // 4
    unsigned int     PackVolt_UN                :1; // 5
    unsigned int     CellVolt_OV                :1; // 6
    unsigned int     CellVolt_UN                :1; // 7
    unsigned int     CellVolt_BL                :1; // 8
    unsigned int     CellTempsDisch_OT          :1; // 9
    unsigned int     CellTempsCharg_OT          :1; // 10
    unsigned int     CellTempsDisCh_UT          :1; // 11
    unsigned int     CellTempsCharg_UT          :1; // 12
    unsigned int     CellTemp_BL                :1; // 13
    unsigned int     PackDischarUnPWR_BL        :1; // 14
    unsigned int     PackCharUnPWR_BL           :1; // 15
    unsigned int     SW16                       :1; // 16
    unsigned int     SW17                       :1; // 17
    unsigned int     SW18                       :1; // 18
    unsigned int     SW19                       :1; // 19
    unsigned int     SW20                       :1; // 20
    unsigned int     SW21                       :1; // 21
    unsigned int     SW22                       :1; // 22
    unsigned int     SW23                       :1; // 23
    unsigned int     SW24                       :1; // 24
    unsigned int     SW25                       :1; // 25
    unsigned int     SW26                       :1; // 26
    unsigned int     SW27                       :1; // 27
    unsigned int     SW28                       :1; // 28
    unsigned int     SW29                       :1; // 29
    unsigned int     SW30                       :1; // 30
    unsigned int     SW31                       :1; // 31

};
union SystemFault_REG
{
    struct SystemFault_BIT  bit;
    struct Data_WORD       Word;
    Uint32                  all;
};
struct SystemProtect_BIT
{       // bits   description
    unsigned int     PackVDisChar_OC            :1; // 0
    unsigned int     PackVChar_OC               :1; // 1
    unsigned int     PackVSOC_OV                :1; // 2
    unsigned int     PackVSOC_UN                :1; // 3
    unsigned int     PackVolt_OV                :1; // 4
    unsigned int     PackVolt_UN                :1; // 5
    unsigned int     CellVolt_OV                :1; // 6
    unsigned int     CellVolt_UN                :1; // 7
    unsigned int     CellVolt_BL                :1; // 8
    unsigned int     CellTempsDisch_OT          :1; // 9
    unsigned int     CellTempsCharg_OT          :1; // 10
    unsigned int     CellTempsDisCh_UT          :1; // 11
    unsigned int     CellTempsCharg_UT          :1; // 12
    unsigned int     CellTemp_BL                :1; // 13
    unsigned int     PackDischarUnPWR_BL        :1; // 14
    unsigned int     PackCharUnPWR_BL           :1; // 15
    unsigned int     PackRly_Err                :1; // 16
    unsigned int     PackIOSPI_Err              :1; // 17
    unsigned int     PackCAN_Err                :1; // 18
    unsigned int     PackCT_Err                 :1; // 19
    unsigned int     PackISOReg_Err             :1; // 20
    unsigned int     SW21                       :1; // 21
    unsigned int     SW22                       :1; // 22
    unsigned int     SW23                       :1; // 23
    unsigned int     SW24                       :1; // 24
    unsigned int     SW25                       :1; // 25
    unsigned int     SW26                       :1; // 26
    unsigned int     SW27                       :1; // 27
    unsigned int     SW28                       :1; // 28
    unsigned int     SW29                       :1; // 29
    unsigned int     SW30                       :1; // 30
    unsigned int     SW31                       :1; // 31
};
union SystemProtect_REG
{
    Uint32                    all;
    struct Data_WORD         Word;
    struct SystemProtect_BIT  bit;
};
struct SlaveBMSCOMERR_BIT
{       // bits   description
    unsigned int     SlaveBMS00         :1; // 0
    unsigned int     SlaveBMS01         :1; // 1
    unsigned int     SlaveBMS02         :1; // 2
    unsigned int     SlaveBMS03         :1; // 3
    unsigned int     SlaveBMS04         :1; // 4
    unsigned int     SlaveBMS05         :1; // 5
    unsigned int     SlaveBMS06         :1; // 6
    unsigned int     SlaveBMS07         :1; // 7
    unsigned int     SlaveBMS08         :1; // 8
    unsigned int     SlaveBMS09         :1; // 9
    unsigned int     SlaveBMS10         :1; // 10
    unsigned int     SlaveBMS11         :1; // 11
    unsigned int     SlaveBMS12         :1; // 12
    unsigned int     SlaveBMS13         :1; // 13
    unsigned int     SlaveBMS14         :1; // 14
    unsigned int     SlaveBMS15         :1; // 15
    unsigned int     SlaveBMS16         :1; // 16
    unsigned int     SlaveBMS17         :1; // 17
    unsigned int     SlaveBMS18         :1; // 18
    unsigned int     SlaveBMS19         :1; // 19
    unsigned int     SlaveBMS20         :1; // 20
    unsigned int     SlaveBMS21         :1; // 21
    unsigned int     SlaveBMS22         :1; // 22
    unsigned int     SlaveBMS23         :1; // 23
    unsigned int     SlaveBMS24         :1; // 24
    unsigned int     SlaveBMS25         :1; // 25
    unsigned int     SlaveBMS26         :1; // 26
    unsigned int     SlaveBMS27         :1; // 27
    unsigned int     SlaveBMS28         :1; // 28
    unsigned int     SlaveBMS29         :1; // 29
    unsigned int     SlaveBMS30         :1; // 30
    unsigned int     SlaveBMS31         :1; // 31
};
union SlaveBMSCOMERR_REG
{
    Uint32                    all;
    struct Data_WORD         Word;
    struct SlaveBMSCOMERR_BIT  bit;
};
struct SlaveEnable_BIT
{       // bits   description
    unsigned int     SlaveBMS00         :1; // 0
    unsigned int     SlaveBMS01         :1; // 1
    unsigned int     SlaveBMS02         :1; // 2
    unsigned int     SlaveBMS03         :1; // 3
    unsigned int     SlaveBMS04         :1; // 4
    unsigned int     SlaveBMS05         :1; // 5
    unsigned int     SlaveBMS06         :1; // 6
    unsigned int     SlaveBMS07         :1; // 7
    unsigned int     SlaveBMS08         :1; // 8
    unsigned int     SlaveBMS09         :1; // 9
    unsigned int     SlaveBMS10         :1; // 10
    unsigned int     SlaveBMS11         :1; // 11
    unsigned int     SlaveBMS12         :1; // 12
    unsigned int     SlaveBMS13         :1; // 13
    unsigned int     SlaveBMS14         :1; // 14
    unsigned int     SlaveBMS15         :1; // 15
    unsigned int     SlaveBMS16         :1; // 16
    unsigned int     SlaveBMS17         :1; // 17
    unsigned int     SlaveBMS18         :1; // 18
    unsigned int     SlaveBMS19         :1; // 19
    unsigned int     SlaveBMS20         :1; // 20
    unsigned int     SlaveBMS21         :1; // 21
    unsigned int     SlaveBMS22         :1; // 22
    unsigned int     SlaveBMS23         :1; // 23
    unsigned int     SlaveBMS24         :1; // 24
    unsigned int     SlaveBMS25         :1; // 25
    unsigned int     SlaveBMS26         :1; // 26
    unsigned int     SlaveBMS27         :1; // 27
    unsigned int     SlaveBMS28         :1; // 28
    unsigned int     SlaveBMS29         :1; // 29
    unsigned int     SlaveBMS30         :1; // 30
    unsigned int     SlaveBMS31         :1; // 31
};
union SlaveBMSEnable_REG
{
    Uint32                    all;
    struct Data_WORD         Word;
    struct SlaveEnable_BIT  bit;
};
struct Current_byte
{
    unsigned int CurrentL;
    unsigned int CurrentH;
};
union Currnet_Reg
{
    long                all;
    struct Current_byte byte;
};
struct WORD2BYTE_byte
{
    unsigned int BYTEL;
    unsigned int BYTEH;
};
union WORD2BYTE_Reg
{
    unsigned int           all;
    struct WORD2BYTE_byte byte;
};
/*
typedef struct Module_Date
{
    Uint16 MDCellMaxVolt[C_SysModuleEa];
    Uint16 MDCellMinVolt[C_SysModuleEa];
    Uint16 MDCellAgvVolt[C_SysModuleEa];
    Uint16 MDCellDivVolt[C_SysModuleEa];
    int16  MDCellMaxTemps[C_SysModuleEa];
    int16  MDCellMinTemps[C_SysModuleEa];
    int16  MDCellAgvTemps[C_SysModuleEa];
    Uint16 MDCellDivTemps[C_SysModuleEa];
    Uint16 MDISOSPIErrcount[C_SysModuleEa];

}ModulemReg;
*/
typedef struct System_Date
{
    /*
     *
     */
    Uint16  Test;
    Uint16  Maincount;
    Uint16  MainIsr1;
    Uint16  CANRXCOUNT;
    Uint16  CanComEable;
    Uint16  CANRXMailBox00Count;
    Uint16  CANRXMailBox01Count;
    Uint16  CANRXMailBox02Count;
    Uint16  CANRXMailBox03Count;
    Uint16  CANRXMailBox04Count;
    Uint16  SysRegTimer5msecCount;
    Uint16  SysRegTimer10msecCount;
    Uint16  SysRegTimer50msecCount;
    Uint16  SysRegTimer100msecCount;
    Uint16  SysRegTimer300msecCount;
    Uint16  SysRegTimer500msecCount;
    Uint16  SysRegTimer1000msecCount;
    Uint16  CellVoltsampling;
    Uint16  CellTempssampling;
    Uint16  SysCanRxCount;
    Uint16  AlarmStatecount;
    Uint16  ProtectStatecount;
    Uint16  BalanceModeCount;
    Uint16  BalanceTimeCount;
    Uint16  VoltTempsReadCount;
    Uint16  RelayCheck;
    Uint16  VCUCANErrCheck;
    Uint16  CTCANErrCheck;
    Uint16  HMICANErrCheck;
    Uint16  SysVoltageMaxNum;
    Uint16  SysVoltageMinNum;
    Uint16  SysTemperatureMaxNum;
    Uint16  SysTemperatureMinNum;
    float32 SysCellVoltageF[C_SysCellVoltEa];
    float32 SysCelltemperatureF[C_SysCellTempEa];

    float32 SysPackVoltageF;
    float32 SysPackParallelVoltageF;
    float32 SysPackCurrentAsbF;
    float32 SysPackCurrentF;
    float32 SysSOCF;
    float32 SysSOHF;
    float32 SysAhF;

    float32 MDVoltageF[6];
    float32 MDCellVoltAgvF[6];
    float32 MDCellTempsAgvF[6];
    Uint16  MDNumber;
    Uint16  MDNumCout;

    float32 SysCellMaxVoltageF;
    float32 SysCellMinVoltageF;
    float32 SysCellAgvVoltageF;
    float32 SysCellDivVoltageF;
    float32 BalanceRefVoltageF;
    float32 SysMaxTemperatureF;
    float32 SysCellMaxTemperatureF;
    float32 SysCellMinTemperatureF;
    float32 SysCellAgvTemperatureF;
    float32 SysCellDivTemperatureF;

    float32 SysCHARGPWRContintyF;
    float32 SysCHARGPWRContintyTGTF;
    float32 SysCHARGPWRContintyDivF;

    float32 SysDISCHAPWRContintyF;
    float32 SysDISCHAPWRContintyTGTF;
    float32 SysDISCHAPWRContintyDivF;

    float32 SysCHARGPWRPeakF;
    float32 SysDISCHAPWRPeakF;
    float32 SysPackIsoRegsF;
    float32 SysISOResisF;

//    float32 Bat12VCellVoltage[Sys12VCellVoltCount];
    //float32 Bat80VCellTe[Sys80VCellVoltCount];
    //float32 Bat12VCellVoltage[Sys12VCellVoltCount];
    /*
     *
    */
    unsigned int LEDSycCount;
    unsigned int LEDFaultCount;
    unsigned int LEDCanCount;
    unsigned int StartBATOUTOnCount;
    unsigned int StartBATOUTOffCount;


    Uint16 ProtectDelayCount[32];
    Uint16 SlaveTempsErrCount[32];
    Uint16 SlaveBalanErrCount[32];
    Uint16 SlaveVoltErrCount[32];

    SysState    SysMachine;
    union       ParentDeviceCMD_REG         PMSysCMDResg;
    union       SystemState_REG             SysStateReg;
    union       SystemAlarm_REG             SysAlarmReg;
    union       SystemFault_REG             SysFaultReg;
  //  union       SystemFault_REG             SysFaultBufReg;
    union       SystemProtect_REG           SysProtectReg;
    union       SystemProtect_REG           SysProtectBufReg;
    union       DigitalInput_REG            SysDigitalInputReg;
    union       DigitalOutPut_REG           SysDigitalOutPutReg;
    union       Currnet_Reg                 SysCurrentData;
 //   union       WORD2BYTE_Reg               PackCOMERR;
    union       SlaveBMSCOMERR_REG          SlaveISOSPIErrReg;
    union       SlaveBMSEnable_REG          SlaveReadTempsEn;
    union       SlaveBMSEnable_REG          SlaveReadVoltEn;
    union       IDSW_REG                    IDSWReg;

}SystemReg;

struct VCUCOMMAND_BIT
{       // bits   description
   unsigned int     PCCMD00         :1; // 0
   unsigned int     PCCMD01         :1; // 1
   unsigned int     PCCMD02         :1; // 2
   unsigned int     PCCMD03         :1; // 3
   unsigned int     PCCMD04         :1; // 4
   unsigned int     PCCMD05         :1; // 5
   unsigned int     PCCMD06         :1; // 6
   unsigned int     PCCMD07         :1; // 7
   unsigned int     PrtctReset      :1; // 8
   unsigned int     RUNStatus       :1; // 9
   unsigned int     PCCMD10         :1; // 10
   unsigned int     PCCMD11         :1; // 11
   unsigned int     PCCMD12         :1; // 12
   unsigned int     PCCMD13         :1; // 13
   unsigned int     PCCMD14         :1; // 14
   unsigned int     PCCMD15         :1; // 15
};
union VCUCOMMAND_REG
{
   unsigned int     all;
   struct VCUCOMMAND_BIT bit;
};
struct HMICOMMAND_BIT
{       // bits   description
   unsigned int     HMI_MODE            :1; // 0
   unsigned int     HMI_RlyEN           :1; // 1
   unsigned int     HMI_CellVoltReq     :1; // 2
   unsigned int     HMI_CellTempsReq    :1; // 3
   unsigned int     HMI_CellBalaEn      :1; // 4
   unsigned int     HMICMD05            :1; // 5
   unsigned int     HMICMD06            :1; // 6
   unsigned int     HMICMD07            :1; // 7
   unsigned int     HMI_Reset           :1; // 8
   unsigned int     HMICMD09            :1; // 9
   unsigned int     HMICMD10            :1; // 10
   unsigned int     HMICMD11            :1; // 11
   unsigned int     HMICMD12            :1; // 12
   unsigned int     HMICMD13            :1; // 13
   unsigned int     HMICMD14            :1; // 14
   unsigned int     HMICMD15            :1; // 15
};
union HMICOMMAND_REG
{
   unsigned int     all;
   struct HMICOMMAND_BIT bit;
};
struct BATStatus_BIT
{       // bits   description
    unsigned int     BalanceMode      :1; // 0
    unsigned int     NegRly           :1; // 1
    unsigned int     PoRly            :1; // 2
    unsigned int     PreCharRly       :1; // 3
    unsigned int     MSDAux           :1; // 4
    unsigned int     STATE05          :1; // 5
    unsigned int     STATE06          :1; // 6
    unsigned int     STATE07          :1; // 7
    unsigned int     STATE08          :1; // 8
    unsigned int     STATE09          :1; // 9
    unsigned int     STATE10          :1; // 10
    unsigned int     STATE11          :1; // 11
    unsigned int     STATE12          :1; // 12
    unsigned int     STATE13          :1; // 13
    unsigned int     STATE14          :1; // 14
    unsigned int     STATE15          :1; // 15
};
union BATStatus_REG
{
   unsigned int     all;
   struct BATStatus_BIT bit;
};
struct ChargerState_BIT
{       // bits   description
    unsigned int     BatPTOC            :1; // 0
    unsigned int     BatSOCOV           :1; // 1
    unsigned int     BatSOCUn           :1; // 2
    unsigned int     BatNRly            :1; // 3
    unsigned int     BatPRly            :1; // 4
    unsigned int     BatPreRly          :1; // 5
    unsigned int     BatRlyErr          :1; // 6
    unsigned int     BSACHAEnable       :1; // 7
    unsigned int     NotUsed08          :1; // 8
    unsigned int     NotUsed09          :1; // 9
    unsigned int     NotUsed10          :1; // 10
    unsigned int     NotUsed11          :1; // 11
    unsigned int     NotUsed12          :1; // 12
    unsigned int     NotUsed13          :1; // 13
    unsigned int     NotUsed14          :1; // 14
    unsigned int     NotUsed15          :1; // 15
};
union ChargerSate_REG
{
   unsigned int     all;
   struct ChargerState_BIT bit;
};
typedef struct CANA_DATA
{

    Uint16 MailBoxRxCount;
    Uint16 MailBox0RxCount;
    Uint16 MailBox1RxCount;
    Uint16 MailBox2RxCount;
    Uint16 MailBox3RxCount;
    Uint16 ProductInfro;
    Uint16 SysConFig;
    Uint16 SysPackPT;
    int16  SysPackCT;
    int16  SysPackSOC;
    Uint16 SysPackSOH;

    Uint16 SysState;
    Uint16 SysPackVotageBuf;
    Uint16 SysCHARGPWRContinty;
    Uint16 SysDISCHAPWRContinty;
    Uint16 SysCHARGPWRPeak;
    Uint16 SysDISCHAPWRPeak;
    int16  SysPackAh;
    Uint16 SysPackIsoRegs;
    Uint16 SysCellInRegsMax;
    Uint16 SysSoHCapacity;

    Uint16 CharCONSTVolt;
    int16  CahrConstantCurrt;
    Uint16 MDNumber;
    Uint16 MDVoltage[6];
    Uint16 MDCellVoltAgv[6];
    int16  MDCellTempsAgv[6];

    Uint16 CellNumCount;
    Uint16 SysCellVoltage[C_SysCellVoltEa];
    int16  SysCelltemperature[C_SysCellTempEa];
    Uint16 SysCellInRegs[C_SysCellVoltEa];

    Uint16 CellVoltageMax;
    Uint16 CellVoltageMin;
    Uint16 CellVoltageAgv;
    Uint16 CellVoltageDiv;
    Uint16 CellVoltageMaxNum;
    Uint16 CellVoltageMinNum;
    int16  CellTemperaturelMAX;
    int16  CellTemperaturelMIN;
    int16  CellTemperatureAVG;
    Uint16 CellTemperatureDiv;
    Uint16 CellTemperatureMaxNum;
    Uint16 CellTemperatureMinNUM;
    Uint16 HMICEllVoltMin;
    Uint16 HMICEllTempsAgv;
    Uint16 HMICellVoltCout;
    Uint16 HMICellTempsCout;
    Uint16 HMIISOSPIErrCount;
    Uint16 HMICellVoltNum;
    Uint16 HMICellTempsNum;
    Uint16 HMIISOSPIErrNum;
    Uint16 CANCom_0x61DDate0;
    Uint16 CANCom_0x61DDate1;
    Uint16 CANCom_0x61DDate2;
    Uint16 CANCom_0x61DDate3;
    Uint16 SlaveBMSNumCout;

    Uint16 SlaveBMSErrCout[4];
    //Uint16
    Uint16 VcuRxFlg;
    Uint16 CharRxFlg;
    Uint16 VcuCharRxCout;
    union  BATStatus_REG              SysStatus;
    union  DigitalOutPut_REG          DigitalOutPutReg;
    union  VCUCOMMAND_REG             PMSCMDRegs;
    union  HMICOMMAND_REG             HMICMDRegs;
    union  ChargerSate_REG            ChargerStateRegs;
}CANAReg;

typedef enum
{
  TIMER_STATE_IDLE,
  TIMER_STATE_RUNNING,
  TIMER_STATE_EXPIRED,
  TIMER_STATE_CLEAR
}TimerState;
typedef struct
{
  TimerState state;
  int TimeCount;
  int Start,Stop,Reset,OutState;
  unsigned int TimerVaule;
}TimerReg;

#endif  // end of DSP28x_PROJECT_H definition

