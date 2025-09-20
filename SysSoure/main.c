

/**
 * main.c
 */

#include "DSP28x_Project.h"
#include "parameter.h"
#include "SysVariable.h"
#include "ProtectRelay.h"
#include "BAT_LTC6802.h"
#include "BATAlgorithm.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/*
 *
 */
void InitGpio(void);

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

/*
 *
 */
void InitECanaGpio(void);
void InitECana(void);
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);

/*
 *
 */
void SysTimerINIT(SystemReg *s);
void SysVarINIT(SystemReg *s);
void CANRegVarINIT(CANAReg *P);
void SysDigitalInput(SystemReg *sys);
void SysDigitalOutput(SystemReg *sys);
void MDCalInit(SystemReg *P);
/*
 *
 */

void ProtectRlySateCheck(PrtectRelayReg *P);
void ProtectRlyVarINIT(PrtectRelayReg *P);
void ProtectRlyOnInit(PrtectRelayReg *P);
void ProtectRlyOnHandle(PrtectRelayReg *P);
void ProtectRlyOffInit(PrtectRelayReg *P);
void ProtectRlyOffHandle(PrtectRelayReg *P);
void ProtectRlyEMSHandle(PrtectRelayReg *P);

/*
 *
 */
void SysCalVoltageHandle(SystemReg *s);
void SysCalCurrentHandle(SystemReg *s);
void SysCalTemperatureHandle(SystemReg *s);
void MDCalVoltandTemsHandle(SystemReg *P);

//void CalFarasis52AhRegsInit(SocReg *P);
//void CalFarasis52AhSocInit(SocReg *P);
//void CalFarasis52AhSocHandle(SocReg *P);

//extern void CalFrey60AhRegsInit(SocReg *P);
//extern void CalFrey60AhSocInit(SocReg *P);
//extern void CalFrey60AhSocHandle(SocReg *P);

void SysFaultCheck(SystemReg *s);
void SysAlarmtCheck(SystemReg *s);
void SysProtectCheck(SystemReg *s);
/*
 *
 */


//void CalFrey60AhRegsInit(SocReg *P);
//void CalFrey60AhSocInit(SocReg *P);
//void CalFrey60AhSocHandle(SocReg *P);




/*
 *
 */
int float32ToInt(float32 Vaule, Uint32 Num);
/*
 *
 */

/*
 *
 */
void SPI_Write(unsigned int WRData);
unsigned int SPI_Read(void);
//void BAT_InitSPI(void);
void SPI_BATWrite(unsigned int WRData);
/*
 *
 */
int LTC6804_read_cmd(char address, short command, char data[], int len);
int LTC6804_write_cmd(char address, short command, char data[], int len);
void init_PEC15_Table(void);
unsigned short pec15(char *data, int len);
int SlaveBMSIint(SlaveReg *s);
int SlaveBmsBalance(SlaveReg *s);
void SlaveVoltagHandler(SlaveReg *s);
void SlaveVoltagBalaHandler(SlaveReg *s);
void SlaveBMSDigiteldoutOHandler(SlaveReg *P);
void SalveTempsHandler(SlaveReg *s);
void TempTemps(SystemReg *s);
/*
 *  인터럽트 함수 선언
 */
interrupt void cpu_timer0_isr(void);
interrupt void ISR_CANRXINTA(void);
//interrupt void cpu_timer2_isr(void);

SystemReg       SysRegs;
float32 randomCT=0;
float32 randomA=0;
float32 randomC=0;
PrtectRelayReg  PrtectRelayRegs;
SlaveReg        Slave1Regs;
SlaveReg        Slave2Regs;


CANAReg         CANARegs;
//SocReg          Farasis52AhSocRegs;
//SocReg          Frey60AhSocRegs;
float32         NCMsocTestVoltAGV =3.210;

float32         NUMsocTestVCT =0.0;
float32         LFPsocTestVoltAGV =3.050;
float32         LFPsocTestVCT =0.0;

float32         TesVoltAGV =3.050;
float32         TescutCt =0.0;
float32         TescutCtabs =0.0;



unsigned int    ProtectRelayCyle=0;
//extern unsigned int    CellVoltUnBalaneFaulCount=0;

void main(void)
{
//    struct ECAN_REGS ECanaShadow;
    InitSysCtrl();
    /*
     * To check the clock status of the C2000 in operation
     */
  //  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
  //  SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 0; //XCLOCKOUT = 1/2* SYSCLK

// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// For this example use the following configuration:
// Step 3. Clear all interrupts and initialize PIE vector table:
    DINT;
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
    InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;  // This is needed to write to EALLOW protected registers

    /*
     *  인터럽트 함수 선언
     */
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.ECAN0INTA  = &ISR_CANRXINTA;
//    PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    InitGpio();
    //GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
    //SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2; //XCLOCKOUT = SYSCLK
    InitSpiGpio();
    InitSpi();
    InitECanGpio();
    InitECan();

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();

    ConfigCpuTimer(&CpuTimer0, 80, 1000);
    //CpuTimer0Regs.PRD.all = 80000;// 90000 is 1msec
    CpuTimer0Regs.PRD.all = 80400;// 90000 is 1msec
    //   ConfigCpuTimer(&CpuTimer1, 80, 1000000);
    //   ConfigCpuTimer(&CpuTimer2, 80, 1000000);
    CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
//    InitAdc();
//    AdcOffsetSelfCal();
    EALLOW;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT9;//test
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;      // Enable ECAN-A interrupt of PIE group 9
//  PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // SCIA RX interrupt of PIE group
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    SysRegs.SysMachine = System_STATE_INIT;
    PrtectRelayRegs.StateMachine= PrtctRly_INIT;
    CANARegs.PMSCMDRegs.all=0;
    CANARegs.HMICMDRegs.all=0;
    while(1)
    {

        SysRegs.Maincount++;
        switch(SysRegs.SysMachine)
        {
            case System_STATE_INIT:
                 SysRegs.SysStateReg.bit.SysState =0;
                 SysRegs.SysDigitalOutPutReg.bit.LEDAlarmOUT=0;
                 SysRegs.SysDigitalOutPutReg.bit.LEDFaultOUT=0;
                 SysRegs.SysDigitalOutPutReg.bit.LEDProtectOUT=0;
                 SysRegs.SysStateReg.bit.SysBalanceMode=0;
                 SysTimerINIT(&SysRegs);
                 SysVarINIT(&SysRegs);
                 CANRegVarINIT(&CANARegs);
                 MDCalInit(&SysRegs);
              //   CalFrey60AhRegsInit(&Frey60AhSocRegs);

                 Slave1Regs.ID=BMS_ID_1;
                 Slave1Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave1Regs);

                 Slave2Regs.ID=BMS_ID_2;
                 Slave2Regs.SlaveCh=C_Slave_ACh;
                 SlaveBMSIint(&Slave2Regs);

                 SysRegs.SysStateReg.bit.SysState=0;
                 SysRegs.SysStateReg.bit.INITOK=0;
                 SysRegs.SysMachine=System_STATE_STANDBY;
                 PrtectRelayRegs.StateMachine=PrtctRly_STANDBY;
            break;
            case System_STATE_STANDBY:// INIT
                  SysRegs.SysStateReg.bit.SysState =1;
                  SysRegs.SysStateReg.bit.INITOK=0;
                  SysRegs.SysStateReg.bit.CellVoltOk=0;
                  SysRegs.SysStateReg.bit.CellTempsOk=0;
                 if(SysRegs.SysStateReg.bit.INITOK==0)
                 {
                     // Cell Voltage Read;

                     for(SysRegs.CellTempInit=0;SysRegs.CellTempInit<50 ;SysRegs.CellTempInit++)
                     {
                         Slave1Regs.ID=BMS_ID_1;
                         Slave1Regs.SlaveCh=C_Slave_ACh;
                         Slave1Regs.Balance.all = 0x0000;
                         SlaveBmsBalance(&Slave1Regs);
                         Slave1Regs.StateMachine = STATE_BATREAD;
                         Slave1Regs.ID=BMS_ID_1;
                         Slave1Regs.SlaveCh=C_Slave_ACh;
                         SlaveVoltagHandler(&Slave1Regs);
                         delay_ms(1);

                         Slave2Regs.ID=BMS_ID_2;
                         Slave2Regs.SlaveCh=C_Slave_ACh;
                         Slave2Regs.Balance.all = 0x0000;
                         SlaveBmsBalance(&Slave2Regs);
                         Slave2Regs.StateMachine = STATE_BATREAD;
                         Slave2Regs.ID=BMS_ID_2;
                         Slave2Regs.SlaveCh=C_Slave_ACh;
                         SlaveVoltagHandler(&Slave2Regs);
                         delay_ms(1);

                         // Cell Voltage Slave -> System
                         memcpy(&SysRegs.SysCellVoltageF[0],        &Slave1Regs.CellVoltageF[0],sizeof(float32)*8);
                         memcpy(&SysRegs.SysCellVoltageF[7],        &Slave2Regs.CellVoltageF[0],sizeof(float32)*6);

                         SysCalVoltageHandle(&SysRegs);
                         SysRegs.SysStateReg.bit.CellVoltOk=0;


                         // Cell temperature Read;
                         Slave1Regs.ID=BMS_ID_1;
                         Slave1Regs.SlaveCh=C_Slave_ACh;
                         Slave1Regs.BATICDO.bit.GPIO1=1;
                         SlaveBMSDigiteldoutOHandler(&Slave1Regs);
                         delay_ms(1);
                         SalveTempsHandler(&Slave1Regs);
                         delay_ms(1);

                         Slave2Regs.ID=BMS_ID_1;
                         Slave2Regs.SlaveCh=C_Slave_ACh;
                         Slave2Regs.BATICDO.bit.GPIO1=1;
                         SlaveBMSDigiteldoutOHandler(&Slave2Regs);
                         delay_ms(5);
                         SalveTempsHandler(&Slave2Regs);
                         delay_ms(1);

                         // Cell temperature Slave -> System
                         memcpy(&SysRegs.SysCelltemperatureF[0],     &Slave1Regs.CellTemperatureF[0],sizeof(float32)*12);
                         memcpy(&SysRegs.SysCelltemperatureF[12],    &Slave2Regs.CellTemperatureF[0],sizeof(float32)*2);
                         SysCalTemperatureHandle(&SysRegs);
                         SysRegs.SysStateReg.bit.CellTempsOk=0;
                     }
                     // System SOC CAL
                     // EV240AhSocRegs.CellAgvVoltageF=SysRegs.SysCellAgvVoltageF;
                     // CalEVE240AhSocInit(&EV240AhSocRegs);

                     //EV240AhSocRegs.state =SOC_STATE_RUNNING;
                    // Frey60AhSocRegs.CellAgvVoltageF=SysRegs.SysCellAgvVoltageF;
                    // CalFrey60AhSocInit(&Frey60AhSocRegs);

                     SysRegs.SysStateReg.bit.INITOK=1;
                     SysRegs.SysStateReg.bit.CellVoltOk=1;
                     SysRegs.SysStateReg.bit.CellTempsOk=1;
                     SysRegs.SysStateReg.bit.CANEnable=1;
                     SysRegs.SysMachine=System_STATE_READY;
                    ///Frey60AhSocRegs.state =SOC_STATE_RUNNING;
                 }
            break;
            case System_STATE_READY:
                  if(SysRegs.SysStateReg.bit.SysDisCharMode ==1)
                  {
                      SysRegs.SysStateReg.bit.SysState =3;
                  }
                  else
                  {
                       SysRegs.SysStateReg.bit.SysState =2;
                  }
              //   SysRegs.SysStateReg.bit.INITOK=1;
                 if(SysRegs.SysStateReg.bit.HMICOMEnable==0)
                 {
                     CANARegs.HMICMDRegs.all=0;
                     CANARegs.HMICEllTempsAgv=0;
                     CANARegs.HMICEllVoltMin=0;
                     if(SysRegs.SysStateReg.bit.SysProtect==0)
                     {
                         PrtectRelayRegs.State.bit.WakeUpEN=1;
                         if(CANARegs.PMSCMDRegs.bit.PrtctReset==1)
                         {
                             SysRegs.SysAlarmReg.all=0;
                             SysRegs.SysFaultReg.all=0;
                             SysRegs.SysProtectReg.all=0;
                             SysRegs.SysStateReg.all=0;
 //                            SysRegs.SysStateReg.bit.SysFault=0;
                             PrtectRelayRegs.StateMachine= PrtctRly_INIT;
                             SysRegs.SysMachine=System_STATE_INIT;
                         }
                     }
                 }
                 PrtectRelayRegs.State.bit.WakeUpEN=1;
                // Frey60AhSocRegs.state =SOC_STATE_RUNNING;
                 if(SysRegs.SysStateReg.bit.HMICOMEnable==1)
                 {
                     CANARegs.PMSCMDRegs.bit.PrtctReset=0;
                     if(SysRegs.SysStateReg.bit.SysProtect==0)
                     {
                         if(CANARegs.HMICMDRegs.bit.HMI_Reset==0)
                         {
//                             PrtectRelayRegs.State.bit.WakeUpEN = CANARegs.HMICMDRegs.bit.HMI_RlyEN;
                         }
                         if(CANARegs.HMICMDRegs.bit.HMI_Reset==1)
                         {
                             CANARegs.HMICMDRegs.bit.HMI_RlyEN =0;
//                             PrtectRelayRegs.State.bit.WakeUpEN=0;
                             SysRegs.SysAlarmReg.all=0;
                             SysRegs.SysFaultReg.all=0;
                             SysRegs.SysProtectReg.all=0;
                             SysRegs.SysStateReg.all=0;
 //                            SysRegs.SysStateReg.bit.SysFault=0;
                             PrtectRelayRegs.StateMachine= PrtctRly_INIT;
                             SysRegs.SysMachine=System_STATE_INIT;
                         }
                     }
                 }

                 if(PrtectRelayRegs.State.bit.WakeuPOnEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_RUNING;

                 }
                 if(SysRegs.SysStateReg.bit.SysProtect==1)
                 {
                     SysRegs.SysMachine=System_STATE_PROTECTER;
                 }

            break;
            case System_STATE_RUNING:

                     if(SysRegs.SysStateReg.bit.SysDisCharMode ==1)
                     {
                         SysRegs.SysStateReg.bit.SysState =3;
                     }
                     else
                     {
                          SysRegs.SysStateReg.bit.SysState =2;
                     }
                     if(SysRegs.SysStateReg.bit.HMICOMEnable==0)
                     {
                         PrtectRelayRegs.State.bit.WakeUpEN=1;
                         if(SysRegs.SysStateReg.bit.SysProtect==0)
                         {
                             if(CANARegs.PMSCMDRegs.bit.PrtctReset==1)
                             {
                                 SysRegs.SysAlarmReg.all=0;
                                 SysRegs.SysFaultReg.all=0;
                                 SysRegs.SysProtectReg.all=0;
                                 SysRegs.SysStateReg.all=0;
    //                             SysRegs.SysStateReg.bit.SysFault=0;
                                 PrtectRelayRegs.StateMachine= PrtctRly_INIT;
                                 SysRegs.SysMachine=System_STATE_INIT;
                             }
                         }
                     }
                     if(SysRegs.SysStateReg.bit.HMICOMEnable==1)
                     {
                         CANARegs.PMSCMDRegs.bit.PrtctReset=0;
                         if(SysRegs.SysStateReg.bit.SysProtect==0)
                         {
    //                         PrtectRelayRegs.State.bit.WakeUpEN = CANARegs.HMICMDRegs.bit.HMI_RlyEN;
                             if(CANARegs.HMICMDRegs.bit.HMI_Reset==1)
                             {
    //                             CANARegs.HMICMDRegs.bit.HMI_RlyEN =0;
    //                             PrtectRelayRegs.State.bit.WakeUpEN=0;
                                 SysRegs.SysAlarmReg.all=0;
                                 SysRegs.SysFaultReg.all=0;
                                 SysRegs.SysProtectReg.all=0;
                                 SysRegs.SysStateReg.all=0;
     //                            SysRegs.SysStateReg.bit.SysFault=0;
                                 PrtectRelayRegs.StateMachine= PrtctRly_INIT;
                                 SysRegs.SysMachine=System_STATE_INIT;
                             }

                         }
                     }
                     if(PrtectRelayRegs.State.bit.WakeuPOnEND==0)
                     {
                         SysRegs.SysMachine=System_STATE_READY;
                     }

            break;
            case System_STATE_PROTECTER:
                 PrtectRelayRegs.StateMachine=PrtctRly_ProtectpOFF;
                 if(SysRegs.SysStateReg.bit.HMICOMEnable==0)
                 {
                     if(CANARegs.PMSCMDRegs.bit.PrtctReset==1)
                     {
                         CANARegs.PMSCMDRegs.bit.PrtctReset=0;
                         SysRegs.SysAlarmReg.all=0;
                         SysRegs.SysFaultReg.all=0;
                         SysRegs.SysProtectReg.all=0;
                         SysRegs.SysStateReg.all=0;
//                         SysRegs.SysStateReg.bit.SysFault=0;
                         delay_ms(200);
                         if(SysRegs.SysStateReg.bit.SysProtect==0)
                         {
                             PrtectRelayRegs.StateMachine= PrtctRly_INIT;
                             SysRegs.SysMachine=System_STATE_STANDBY;
                         }
                     }
                 }
                 if(SysRegs.SysStateReg.bit.HMICOMEnable==1)
                 {

                 }
                 if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_Reset==1))
                 {
                     CANARegs.HMICMDRegs.bit.HMI_Reset=0;
                     CANARegs.PMSCMDRegs.bit.PrtctReset=0;
                     SysRegs.SysAlarmReg.all=0;
                     SysRegs.SysFaultReg.all=0;
                     SysRegs.SysProtectReg.all=0;
                     SysRegs.SysStateReg.all=0;
//                     SysRegs.SysStateReg.bit.SysFault=0;
                     delay_ms(200);
                     if(SysRegs.SysStateReg.bit.SysProtect==0)
                     {
                         PrtectRelayRegs.StateMachine= PrtctRly_INIT;
                         SysRegs.SysMachine=System_STATE_STANDBY;
                     }
                 }

            break;
            case System_STATE_CLEAR:

            break;
            default :
            break;
        }
        if(SysRegs.CellVoltsampling>=CellVoltSampleTime)
        {
            //Balance 위한 전류 조건
            if(SysRegs.SysPackCurrentAsbF<=C_PackBalanCurrent)
            {
                SysRegs.BalanceModeCount++;
                if(SysRegs.BalanceModeCount>=100)
                {
                    SysRegs.BalanceModeCount=101;
                    SysRegs.SysStateReg.bit.SysBalanceMode=1;
                }
            }
            else if(SysRegs.SysPackCurrentAsbF>C_PackBalanCurrent)
            {
                SysRegs.SysStateReg.bit.SysBalanceMode=0;
                SysRegs.SysStateReg.bit.SysBalanceEn=0;
                SysRegs.BalanceModeCount=0;
                SysRegs.BalanceTimeCount=0;
            }
            //Balance 위한 셀 최조 전압 조건
            if(SysRegs.SysCellMinVoltageF<C_CellBalanLimtVolt)
            {
                SysRegs.SysStateReg.bit.SysBalanceMode=0;
                SysRegs.SysStateReg.bit.SysBalanceEn=0;
                SysRegs.BalanceModeCount=0;
                SysRegs.BalanceTimeCount=0;
            }
            SysRegs.SysStateReg.bit.SysBalanceMode=0;
            SysRegs.SysStateReg.bit.SysBalanceEn=0;
            //Balance 시간 조정
            if(SysRegs.SysStateReg.bit.SysBalanceMode==1)
            {
                SysRegs.BalanceTimeCount++;
                if(SysRegs.BalanceTimeCount>10)
                {
                   SysRegs.SysStateReg.bit.SysBalanceEn = !  SysRegs.SysStateReg.bit.SysBalanceEn;
                   SysRegs.BalanceTimeCount=0;
                }
            }
            else
            {
                SysRegs.SysStateReg.bit.SysBalanceEn=0;
            }
            // 셀 전압 Balance
            if(SysRegs.SysStateReg.bit.SysBalanceEn==1)
            {
                if(SysRegs.SysStateReg.bit.HMICOMEnable==0)
                {
                    SysRegs.HMICANErrCheck=0;
                    CANARegs.HMICMDRegs.all=0;
                    CANARegs.HMICEllTempsAgv=250;
                    CANARegs.HMICEllVoltMin=4200;
                    SysRegs.BalanceRefVoltageF = SysRegs.SysCellMinVoltageF;
                }
                if(SysRegs.SysStateReg.bit.HMICOMEnable==1)
                {
                    SysRegs.HMICANErrCheck++;
                    if(SysRegs.SysStateReg.bit.HMIBalanceMode==0)
                    {
                        SysRegs.BalanceRefVoltageF = SysRegs.SysCellMinVoltageF;
                    }
                    if(SysRegs.SysStateReg.bit.HMIBalanceMode==1)
                    {
                       SysRegs.BalanceRefVoltageF = (float32)(CANARegs.HMICEllVoltMin*0.001);
                    }
                    if(SysRegs.HMICANErrCheck>200)
                    {
                       SysRegs.HMICANErrCheck=210;
                       CANARegs.HMICEllTempsAgv=250;
                      // CANARegs.HMICEllVoltMin=4200;
                       SysRegs.BalanceRefVoltageF = (float32)(CANARegs.HMICEllVoltMin*0.001);
                       SysRegs.SysStateReg.bit.SysBalanceEn=0;
                    }

                }
                if(SysRegs.SysStateReg.bit.SysBalanceEn==1)
                {


                    Slave1Regs.ID=BMS_ID_1;
                    Slave1Regs.SlaveCh=C_Slave_ACh;
                    Slave1Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave1Regs);
                    Slave1Regs.Balance.bit.B_Cell07=0;
                    Slave1Regs.Balance.bit.B_Cell08=0;
                    Slave1Regs.Balance.bit.B_Cell09=0;
                    Slave1Regs.Balance.bit.B_Cell10=0;
                    Slave1Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave1Regs);
                    SysRegs.SlaveVoltErrCount[1]=Slave1Regs.ErrorCount;


                    Slave2Regs.ID=BMS_ID_2;
                    Slave2Regs.SlaveCh=C_Slave_ACh;
                    Slave2Regs.SysCellMinVoltage = SysRegs.BalanceRefVoltageF;
                    SlaveVoltagBalaHandler(&Slave2Regs);
                    Slave2Regs.Balance.bit.B_Cell07=0;
                    Slave2Regs.Balance.bit.B_Cell08=0;
                    Slave2Regs.Balance.bit.B_Cell09=0;
                    Slave2Regs.Balance.bit.B_Cell10=0;
                    Slave2Regs.Balance.bit.B_Cell11=0;
                    SlaveBmsBalance(&Slave2Regs);
                    SysRegs.SlaveVoltErrCount[2]=Slave2Regs.ErrorCount;

                }
            }
            if(SysRegs.SysStateReg.bit.SysBalanceEn==0)
            {
                if(SysRegs.SysStateReg.bit.CellVoltOk==0)
                {
                     LEDSysState_H;
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS01==1)
                    {
                        Slave1Regs.ID=BMS_ID_1;
                        Slave1Regs.SlaveCh=C_Slave_ACh;

                        Slave1Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave1Regs);
                        SysRegs.SlaveBalanErrCount[1]=Slave1Regs.ErrorCount;
                        if(SysRegs.SlaveBalanErrCount[1]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00=1;
                            SysRegs.SlaveBalanErrCount[1]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS00 =0;
                        }

                        Slave1Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave1Regs);
                        SysRegs.SlaveVoltErrCount[1]=Slave1Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[1]>C_ISOSPIPrtectCont)
                        {
                             SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                             SysRegs.SlaveVoltErrCount[1]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                             SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01 =0;
                        }
                    }
                    if(SysRegs.SlaveReadVoltEn.bit.SlaveBMS02==1)
                    {
                        Slave2Regs.ID=BMS_ID_2;
                        Slave2Regs.SlaveCh=C_Slave_ACh;

                        Slave2Regs.Balance.all = 0x0000;
                        SlaveBmsBalance(&Slave2Regs);
                        SysRegs.SlaveBalanErrCount[2]=Slave2Regs.ErrorCount;
                        if(SysRegs.SlaveBalanErrCount[2]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                            SysRegs.SlaveBalanErrCount[2]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01 =0;
                        }

                        Slave2Regs.StateMachine = STATE_BATREAD;
                        SlaveVoltagHandler(&Slave2Regs);
                        SysRegs.SlaveVoltErrCount[2]=Slave2Regs.ErrorCount;
                        if(SysRegs.SlaveVoltErrCount[2]>C_ISOSPIPrtectCont)
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;
                            SysRegs.SlaveVoltErrCount[2]=C_ISOSPIPrtectCont+10;
                        }
                        else
                        {
                            SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=0;
                        }
                    }
                    LEDSysState_L;
            }

          }
          SysRegs.SysStateReg.bit.CellVoltOk=1;
          SysRegs.CellVoltsampling=0;
       }
       if(SysRegs.CellTempssampling>CellTempSampleTime)
       {
           if(SysRegs.SysStateReg.bit.CellTempsOk==0)
           {
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS01==1)
               {
                   Slave1Regs.ID=BMS_ID_1;
                   Slave1Regs.SlaveCh=C_Slave_ACh;

                   Slave1Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave1Regs);
                   delay_ms(1);
                   SysRegs.SlaveTempsErrCount[1]=Slave1Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[1]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                       SysRegs.SlaveTempsErrCount[1]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=0;
                   }

                   SalveTempsHandler(&Slave1Regs);
                   delay_ms(1);
                   SysRegs.SlaveTempsErrCount[1]=Slave1Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[1]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;
                       SysRegs.SlaveTempsErrCount[1]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=0;
                   }
               }
               if(SysRegs.SlaveReadTempsEn.bit.SlaveBMS02==1)
               {
                   Slave2Regs.ID=BMS_ID_2;
                   Slave2Regs.SlaveCh=C_Slave_ACh;
                   Slave2Regs.BATICDO.bit.GPIO1=1;
                   SlaveBMSDigiteldoutOHandler(&Slave2Regs);
                   delay_ms(1);
                   SysRegs.SlaveTempsErrCount[2]=Slave2Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[2]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;
                       SysRegs.SlaveTempsErrCount[2]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=0;
                   }

                   SalveTempsHandler(&Slave2Regs);
                   delay_ms(1);
                   SysRegs.SlaveTempsErrCount[2]=Slave2Regs.ErrorCount;
                   if(SysRegs.SlaveTempsErrCount[2]>C_ISOSPIPrtectCont)
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;
                       SysRegs.SlaveTempsErrCount[2]=C_ISOSPIPrtectCont+10;
                   }
                   else
                   {
                       SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=0;
                   }

               }
           }
           SysRegs.CellTempssampling=0;
           SysRegs.VoltTempsReadCount++;
           SysRegs.SysStateReg.bit.CellTempsOk=1;
           if(SysRegs.VoltTempsReadCount>=10)
           {
              SysRegs.VoltTempsReadCount=100;
           }
       }
       if(Slave1Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS01=1;}
       if(Slave2Regs.ErrorCount>200) {SysRegs.SlaveISOSPIErrReg.bit.SlaveBMS02=1;}


       memcpy(&CANARegs.SysCellVoltage[0],         &Slave1Regs.CellVoltage[0],sizeof(Uint16)*8);
       memcpy(&CANARegs.SysCellVoltage[8],         &Slave2Regs.CellVoltage[0],sizeof(Uint16)*6);

       memcpy(&CANARegs.SysCelltemperature[0],     &Slave1Regs.CellTemperature[0],sizeof(int16)*12);
       memcpy(&CANARegs.SysCelltemperature[12],    &Slave1Regs.CellTemperature[0],sizeof(int16)*2);



       if(SysRegs.Maincount>3000){SysRegs.Maincount=0;}

    }
}

interrupt void cpu_timer0_isr(void)
{
   //LEDSysState_T;
   SysRegs.MainIsr1++;
   SysRegs.SysRegTimer5msecCount++;
   SysRegs.SysRegTimer10msecCount++;
   SysRegs.SysRegTimer50msecCount++;
   SysRegs.SysRegTimer100msecCount++;
   SysRegs.SysRegTimer300msecCount++;
   SysRegs.SysRegTimer500msecCount++;
   SysRegs.SysRegTimer1000msecCount++;
   SysRegs.CellVoltsampling++;
   SysRegs.CellTempssampling++;
   if(SysRegs.SysRegTimer5msecCount   >SysRegTimer5msec)    {SysRegs.SysRegTimer5msecCount=0;}
   if(SysRegs.SysRegTimer10msecCount  >SysRegTimer10msec)   {SysRegs.SysRegTimer10msecCount=0;}
   if(SysRegs.SysRegTimer50msecCount  >SysRegTimer50msec)   {SysRegs.SysRegTimer50msecCount=0;}
   if(SysRegs.SysRegTimer100msecCount >SysRegTimer100msec)  {SysRegs.SysRegTimer100msecCount=0;}
   if(SysRegs.SysRegTimer300msecCount >SysRegTimer300msec)   {SysRegs.SysRegTimer300msecCount=0;}
   if(SysRegs.SysRegTimer1000msecCount>SysRegTimer1000msec)  {SysRegs.SysRegTimer1000msecCount=0;}
   //
   if(CANARegs.PMSCMDRegs.bit.RUNStatus==1)
   {
      SysRegs.SysDigitalOutPutReg.bit.PWRHOLD=1;
   }
   else
   {
       SysRegs.SysDigitalOutPutReg.bit.PWRHOLD=0;
   }
/*
 *
 */
   if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_Reset==1))
   {
        CANARegs.HMICMDRegs.bit.HMI_Reset=0;
        CANARegs.PMSCMDRegs.bit.PrtctReset=0;
        //SysRegs.SysAlarmReg.all=0;
        //SysRegs.SysFaultReg.all=0;
        //SysRegs.SysProtectReg.all=0;
        //SysRegs.SysStateReg.all=0;
        //SysRegs.SysStateReg.bit.SysFault=0;
        delay_ms(200);
        if(SysRegs.SysStateReg.bit.SysProtect==0)
        {
           PrtectRelayRegs.StateMachine= PrtctRly_INIT;
           SysRegs.SysMachine=System_STATE_STANDBY;
         }
   }

   /*
    * DigitalInput detection
    * 릴레이 변경으로 삭제
    */
   //SysDigitalInput(&SysRegs);


  /*
   * current sensing detection
  */
    SysCalCurrentHandle(&SysRegs);


   // Frey60AhSocRegs.CellAgvVoltageF=LFPsocTestVoltAGV;
   // CalFrey60AhSocInit(&Frey60AhSocRegs);
   // SysRegs.SysMachine=System_STATE_READY;
   // Frey60AhSocRegs.state =SOC_STATE_RUNNING;
   /*
    *  Farasis52AhSocRegs.CellAgvVoltageF = NCMsocTestVoltAGV;
    *  Farasis52AhSocRegs.SysSoCCTF       = NUMsocTestVCT;
    *  Farasis52AhSocRegs.SysSoCCTAbsF    = NUMsocTestVCT;
    */

   /*
    * float32         TesVoltAGV =3.050;
      float32         TescutCt =0.0;
      float32         TescutCtabs =0.0;
    */

   // Frey60AhSocRegs.CellAgvVoltageF = TesVoltAGV;
    // Frey60AhSocRegs.SysSoCCTF       = TescutCt;
    // Frey60AhSocRegs.SysSoCCTAbsF    = TescutCtabs;


    //Frey60AhSocRegs.SysPackAhF
   // Frey60AhSocRegs.CellAgvVoltageF = SysRegs.SysCellAgvVoltageF;
   // Frey60AhSocRegs.SysSoCCTF       = SysRegs.SysPackCurrentF;
   // Frey60AhSocRegs.SysSoCCTAbsF    = SysRegs.SysPackCurrentAsbF;
   // Frey60AhSocRegs.state =SOC_STATE_RUNNING;
   // CalFrey60AhSocHandle(&Frey60AhSocRegs);
/*   if(Frey60AhSocRegs.SoCStateRegs.bit.CalMeth==0)
   {
       SysRegs.SysSOCF=Frey60AhSocRegs.SysSocInitF;
   }
   if(Frey60AhSocRegs.SoCStateRegs.bit.CalMeth==1)
   {
       SysRegs.SysSOCF=Frey60AhSocRegs.SysPackSOCF;
   }
*/
   /*
    * Frey60AhSocRegs.CellAgvVoltageF = LFPsocTestVoltAGV;
    * Frey60AhSocRegs.SysSoCCTF       = LFPsocTestVCT;
    * Frey60AhSocRegs.SysSoCCTAbsF    = LFPsocTestVCT;
    */

   /*
    * Battery Alarm & Fault & Protect Check
    */
   //SysAlarmtCheck(&SysRegs);
   if((SysRegs.SysAlarmReg.all != 0)&&(SysRegs.SysStateReg.bit.INITOK==1))
   {
       SysRegs.SysStateReg.bit.SysAalarm=1;
   }
   else
   {
       SysRegs.SysStateReg.bit.SysAalarm=0;
   }
   SysRegs.SysStateReg.bit.SysProtectStatus=0;
   //SysFaultCheck(&SysRegs);
   if((SysRegs.SysFaultReg.all != 0)&&(SysRegs.SysStateReg.bit.INITOK==1))
   {

 //      SysRegs.SysStateReg.bit.SysFault=1;
       SysRegs.SysStateReg.bit.SysProtectStatus=1;
   }
   else
   {
 //      SysRegs.SysStateReg.bit.SysFault=0;
   }
   //SysProtectCheck(&SysRegs)
   if((SysRegs.SysProtectReg.all !=0)&&(SysRegs.SysStateReg.bit.INITOK==1))
   {
       SysRegs.SysStateReg.bit.SysProtect=1;
       SysRegs.SysStateReg.bit.SysProtectStatus=2;
   }
   switch(SysRegs.SysRegTimer5msecCount)
   {
       case 1:
               SysRegs.SysStateReg.bit.SysProtectStatus=0;
               if(SysRegs.SysStateReg.bit.SysAalarm==1)
               {
                   SysRegs.SysStateReg.bit.SysProtectStatus=1;
                   SysRegs.SysDigitalOutPutReg.bit.LEDAlarmOUT=1;
               }
               if(SysRegs.SysStateReg.bit.SysProtect==1)
               {
                   SysRegs.SysStateReg.bit.SysProtectStatus=2;
                   SysRegs.SysDigitalOutPutReg.bit.LEDProtectOUT=1;
                   SysRegs.SysMachine=System_STATE_PROTECTER;
               }
       break;
       default :
       break;

   }
   switch(SysRegs.SysRegTimer10msecCount)
   {
       case 1:
               if(SysRegs.SysStateReg.bit.CellTempsOk==1)
               {


                   memcpy(&SysRegs.SysCelltemperatureF[0],     &Slave1Regs.CellTemperatureF[0],sizeof(float32)*12);
                   memcpy(&SysRegs.SysCelltemperatureF[12],    &Slave2Regs.CellTemperatureF[0],sizeof(float32)*2);
                   SysCalTemperatureHandle(&SysRegs);
                   SysRegs.SysStateReg.bit.CellTempsOk=0;
               }
       break;
       case 2:
               if(SysRegs.SysStateReg.bit.CellVoltOk==1)
               {
                   memcpy(&SysRegs.SysCellVoltageF[0],        &Slave1Regs.CellVoltageF[0],sizeof(float32)*8);
                   memcpy(&SysRegs.SysCellVoltageF[8],        &Slave2Regs.CellVoltageF[0],sizeof(float32)*6);
                  SysCalVoltageHandle(&SysRegs);
                  SysRegs.SysStateReg.bit.CellVoltOk=0;
               }
       break;
       case 3:
               if(SysRegs.SysStateReg.bit.INITOK==1)
               {

                   //SysRegs.SysStateReg.bit.CellTempsOk=0;
               }
       break;
       case 4:
               if(SysRegs.SysStateReg.bit.INITOK==1)
               {

                   MDCalVoltandTemsHandle(&SysRegs);
                   SysRegs.MDNumber++;
               }
       break;
       case 5:


       break;
       case 6:

       break;
       case 7:


       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer50msecCount)
   {
       case 1:

       break;
       case 5:
              //LEDSysState_H;
              //LEDSysState_L;
       break;
       case 10:
               //LEDSysState_H;
               //At 80MHZ, operation time is 33usec
             //  Cal80VSysVoltageHandle(&SysRegs);
               //LEDSysState_L;
       break;
       case 20:
               //LEDSysState_H;
               //At 80MHZ, operation time is 33usec

               //LEDSysState_L;
       break;
       case 30 :


       break;
       default :
       break;
   }

   switch(SysRegs.SysRegTimer100msecCount)
   {
       case 5:
          //     memcpy(&CANARegs.Salve1VoltageCell[0], &Slave1Regs.CellVoltage[0],sizeof(unsigned int)*12);
       break;
       case 8:
               CANARegs.ProductInfro = ComBine(Product_Version,Product_Type);
               CANARegs.SysConFig    = ComBine(Product_SysCellVauleP,Product_SysCellVauleS);
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x610,8,CANARegs.ProductInfro,CANARegs.SysConFig,Product_Voltage,Product_Capacity);
               }
       break;
       case 10:

               SysRegs.SysSOHF=100.0;
            //   SysRegs.SysSOCF=50.0;
               CANARegs.SysPackPT  = (unsigned int)(SysRegs.SysPackVoltageF*10);
              // CANARegs.SysPackPT  = (unsigned int)(SysRegs.SysPackParallelVoltageF*10);
               CANARegs.SysPackCT  = (int)(SysRegs.SysPackCurrentF*10);
               CANARegs.SysPackSOC = (int)(SysRegs.SysSOCF*10);
               CANARegs.SysPackSOH = (unsigned int)(SysRegs.SysSOHF*10);
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x611,8,CANARegs.SysPackPT,CANARegs.SysPackCT,CANARegs.SysPackSOC,CANARegs.SysPackSOH);
               }
       break;
       case 12:

  //           SysRegs.SysAhF=Frey60AhSocRegs.SysPackAhF;
               CANARegs.SysState = ComBine(SysRegs.SysStateReg.bit.SysProtectStatus,SysRegs.SysStateReg.bit.SysState);
               CANARegs.SysStatus.bit.BalanceMode = SysRegs.SysStateReg.bit.SysBalanceMode;
               CANARegs.SysStatus.bit.NegRly      = SysRegs.SysStateReg.bit.NRlyDOStatus;
               CANARegs.SysStatus.bit.PoRly       = SysRegs.SysDigitalOutPutReg.bit.PRlyOUT;
               CANARegs.SysStatus.bit.PreCharRly  = SysRegs.SysDigitalOutPutReg.bit.ProRlyOUT;
               CANARegs.SysStatus.bit.MSDAux      = 0;


                SysRegs.SysStateReg.bit.SysStatus=SysRegs.SysMachine;
                SysRegs.SysStateReg.bit.SysRlyStatus=PrtectRelayRegs.StateMachine;
         //     SysRegs.SysStateReg.bit.SysSOCStatus=Frey60AhSocRegs.state;

         //      CANARegs.SysPackAh                 =(int)(SysRegs.SysAhF*10);
                if(SysRegs.SysStateReg.bit.CANEnable==1)
                {
                    CANATX(0x612,8,CANARegs.SysState,CANARegs.SysStatus.all,SysRegs.SysStateReg.Word.DataL,SysRegs.SysStateReg.Word.DataH);
                }
       break;
       case 14:
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x613,8,SysRegs.SysAlarmReg.Word.DataL,SysRegs.SysAlarmReg.Word.DataH,SysRegs.SysProtectReg.Word.DataL,SysRegs.SysProtectReg.Word.DataH);
               }
       break;
       case 17:

               SysRegs.SysCHARGPWRPeakF        = 12.0;
               SysRegs.SysDISCHAPWRPeakF       = 36.0;
               SysRegs.SysCHARGPWRContintyF    = 6.0;
               SysRegs.SysDISCHAPWRContintyF   = 21.0;
               CANARegs.SysCHARGPWRContinty    = (Uint16)(SysRegs.SysCHARGPWRContintyF*10);
               CANARegs.SysDISCHAPWRContinty   = (Uint16)(SysRegs.SysDISCHAPWRContintyF*10);
               CANARegs.SysCHARGPWRPeak        = (Uint16)(SysRegs.SysCHARGPWRPeakF*10);
               CANARegs.SysDISCHAPWRPeak       = (Uint16)(SysRegs.SysDISCHAPWRPeakF*10);
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x614,8,CANARegs.SysCHARGPWRContinty,CANARegs.SysDISCHAPWRContinty,CANARegs.SysCHARGPWRPeak,CANARegs.SysDISCHAPWRPeak);
               }
       break;
       case 20:
               CANARegs.CellVoltageMax          = (Uint16)(SysRegs.SysCellMaxVoltageF*1000);
               CANARegs.CellVoltageMin          = (Uint16)(SysRegs.SysCellMinVoltageF*1000);
               CANARegs.CellVoltageAgv          = (Uint16)(SysRegs.SysCellAgvVoltageF*1000);
               CANARegs.CellVoltageDiv          = (Uint16)(SysRegs.SysCellDivVoltageF*1000);
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x615,8,CANARegs.CellVoltageMax,CANARegs.CellVoltageMin,CANARegs.CellVoltageAgv,CANARegs.CellVoltageDiv);
               }
       break;
       case 23:
               CANARegs.CellTemperaturelMAX    = (int16)(SysRegs.SysCellMaxTemperatureF*10);
               CANARegs.CellTemperaturelMIN    = (int16)(SysRegs.SysCellMinTemperatureF*10);
               CANARegs.CellTemperatureAVG     = (int16)(SysRegs.SysCellAgvTemperatureF*10);
               CANARegs.CellTemperatureDiv     = (int16)(SysRegs.SysCellDivTemperatureF*10);
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x616,8,CANARegs.CellTemperaturelMAX,CANARegs.CellTemperaturelMIN,CANARegs.CellTemperatureAVG,CANARegs.CellTemperatureDiv);
               }
       break;
       case 26:
               CANARegs.CellVoltageMaxNum      = SysRegs.SysVoltageMaxNum;
               CANARegs.CellVoltageMinNum      = SysRegs.SysVoltageMinNum;
               CANARegs.CellTemperatureMaxNum  = SysRegs.SysTemperatureMaxNum;
               CANARegs.CellTemperatureMinNUM  = SysRegs.SysTemperatureMinNum;
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x617,8,CANARegs.CellVoltageMaxNum,CANARegs.CellVoltageMinNum,CANARegs.CellTemperatureMaxNum,CANARegs.CellTemperatureMinNUM);
               }
       break;
       case 30:
                CANARegs.SysPackIsoRegs   = 3000;
                CANARegs.SysCellInRegsMax = 100;
                CANARegs.SysSoHCapacity   = 3600;
                if(SysRegs.SysStateReg.bit.CANEnable==1)
                {
                    CANATX(0x618,8,CANARegs.SysPackAh,CANARegs.SysPackIsoRegs, CANARegs.SysCellInRegsMax, CANARegs.SysSoHCapacity);
                }
       break;
       case 35:
               CANARegs.MDNumber++;
               if(CANARegs.MDNumber>2)
               {
                   CANARegs.MDNumber=0;
               }
               CANARegs.MDVoltage[CANARegs.MDNumber]              = (Uint16)(SysRegs.MDVoltageF[CANARegs.MDNumber]*10);
               CANARegs.MDCellVoltAgv[CANARegs.MDNumber]          = (Uint16)(SysRegs.MDCellVoltAgvF[CANARegs.MDNumber]*1000);
               CANARegs.MDCellTempsAgv[CANARegs.MDNumber]         = (Uint16)(SysRegs.MDCellTempsAgvF[CANARegs.MDNumber]*10);
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x619,8,CANARegs.MDNumber,
                                  CANARegs.MDVoltage[CANARegs.MDNumber],CANARegs.MDCellVoltAgv[CANARegs.MDNumber],CANARegs.MDCellTempsAgv[CANARegs.MDNumber]);
               }
       break;
       case 40:

               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x61A,8,CANARegs.CellNumCount,
                                  CANARegs.SysCellVoltage[CANARegs.CellNumCount],CANARegs.SysCelltemperature[CANARegs.CellNumCount],CANARegs.SysCellInRegs[CANARegs.CellNumCount]=100);
               }
               CANARegs.CellNumCount++;
               if(CANARegs.CellNumCount>13)
               {
                   CANARegs.CellNumCount=0;
               }
       break;
       case 45:

       break;
       case 50:

            //   CANARegs.MDVoltage[4]              = (Uint16)(SysRegs.MDVoltageF[4]*10);
            //   CANARegs.MDCellVoltAgv[4]          = (Uint16)(SysRegs.MDCellVoltAgvF[4]*1000);
            //   CANARegs.MDCellTempsAgv[4]         = (Uint16)(SysRegs.MDCellTempsAgvF[4]*10);
            //   CANATX(0x61C,8,CANARegs.MDVoltage[4],CANARegs.MDCellVoltAgv[4],CANARegs.MDCellTempsAgv[4],0x0000);
       break;
       case 55:
               CANARegs.SlaveBMSNumCout++;

               CANARegs.SlaveBMSErrCout[0]=Slave1Regs.ErrorCount;
               CANARegs.SlaveBMSErrCout[1]=Slave2Regs.ErrorCount;

               if(CANARegs.SlaveBMSNumCout>1)
               {
                   CANARegs.SlaveBMSNumCout=0;
               }
               CANARegs.CANCom_0x61DDate0 = ComBine(CANARegs.SlaveBMSErrCout[CANARegs.SlaveBMSNumCout], CANARegs.SlaveBMSNumCout);
               CANARegs.CANCom_0x61DDate1 = ComBine(CANARegs.MailBox1RxCount, CANARegs.MailBox0RxCount);
               CANARegs.CANCom_0x61DDate2 = ComBine(CANARegs.MailBox3RxCount, CANARegs.MailBox2RxCount);
               CANARegs.CANCom_0x61DDate3 = 0x0000;
               CANATX(0x61B,8,CANARegs.CANCom_0x61DDate0,CANARegs.CANCom_0x61DDate1,CANARegs.CANCom_0x61DDate2,CANARegs.CANCom_0x61DDate3);

          // Uint16 CharCONSTVolt;
          // int16  CahrConstantCurrt;
               CANARegs.CharCONSTVolt=542;
               CANARegs.CahrConstantCurrt =300;
               CANARegs.SysPackPT  = (unsigned int)(SysRegs.SysPackParallelVoltageF*10);
               CANARegs.SysPackCT  = (unsigned int)(SysRegs.SysPackCurrentAsbF*10);
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x61E,8,CANARegs.CharCONSTVolt,CANARegs.CahrConstantCurrt,CANARegs.SysPackPT,CANARegs.SysPackCT);
               }
       break;
       case 60:
               if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
               {
                  /*
                   *    unsigned int     SysStatus              :3; // 0,1,2
                        unsigned int     SysRlyStatus           :3; // 3,4,5
                        unsigned int     SysProtectStatus       :2; // 6,7
                        unsigned int     SysSOCStatus           :2; // 8,9
                        unsigned int     SysDisCharMode         :1; // 10
                        unsigned int     HMICOMEnable           :1; // 11
                        unsigned int     HMIBalanceMode         :1; // 12
                        unsigned int     NRlyDOStatus           :1; // 13
                        unsigned int     PRlyDOStatus           :1; // 14
                        unsigned int     PreRlyDOStatus         :1; // 15
                        unsigned int     ISOSPICOMERR           :1; // 16
                        unsigned int     INCANCOMERR            :1; // 17
                        unsigned int     TCPIPTOMERR            :1; // 18
                        unsigned int     RS485COMERR            :1; // 19
                        unsigned int     ISORegERR              :1; // 20
                        unsigned int     MSDERR                 :1; // 21
                        unsigned int     RlyERR                 :1; // 22
                        unsigned int     INITOK                 :1; // 23
                        unsigned int     SysBalanceMode         :1; // 24
                        unsigned int     SysBalanceEn           :1; // 25
                        unsigned int     SysAalarm              :1; // 26
                        unsigned int     SysFault               :1; // 27
                        unsigned int     SysProtect             :1; // 28
                        unsigned int     CellVoltOk             :1; // 29
                        unsigned int     CellTempsOk            :1; // 30
                        unsigned int     SW31                   :1; // 31
                   */
                // CANATX(0x701,8,SysRegs.SysStateReg.Word.DataL,SysRegs.SysStateReg.Word.DataH,0X000,0x0000);
               }

               CANARegs.ChargerStateRegs.all=0;
               CANARegs.ChargerStateRegs.bit.BatNRly=1;
               CANARegs.ChargerStateRegs.bit.BatPRly=1;

               if(SysRegs.SysStateReg.bit.SysDisCharMode==1)
               {
                   CANARegs.ChargerStateRegs.bit.BSACHAEnable=1;
               }
               else
               {
                   CANARegs.ChargerStateRegs.bit.BSACHAEnable=0;
               }
               if(CANARegs.SysPackPT>=CANARegs.CharCONSTVolt)
               {
                   if(CANARegs.SysPackCT<=20)
                   {
                       CANARegs.ChargerStateRegs.bit.BSACHAEnable=0;
                   }
               }
               CANARegs.VcuCharRxCout++;
               if(CANARegs.CharRxFlg==1)
               {
                   SysRegs.SysStateReg.bit.SysDisCharMode=0;
               }
               else
               {
                   SysRegs.SysStateReg.bit.SysDisCharMode=1;
               }
               if(CANARegs.VcuCharRxCout>=20)
               {
                 //  CANARegs.VcuRxFlg=0;
                   CANARegs.VcuCharRxCout=0;
                   CANARegs.CharRxFlg=0;
               }
               if(SysRegs.SysStateReg.bit.CANEnable==1)
               {
                   CANATX(0x61f,8,CANARegs.ChargerStateRegs.all,0x0000,0x0000,0x0000);
               }
       break;
       case 65:
               if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_CellVoltReq==1))
               {
                 CANARegs.HMICellVoltCout++;
                 if(CANARegs.HMICellVoltCout>C_HmiCellVoltCount)
                 {
                     CANARegs.HMICellVoltCout=0;
                 }
           //      CANARegs.HMICellVoltNum=CANARegs.HMICellVoltCout*3;
           //      CANATX(0x702,8,CANARegs.HMICellVoltNum,
           //                     CANARegs.SysCellVoltage[CANARegs.HMICellVoltNum],
           //                     CANARegs.SysCellVoltage[CANARegs.HMICellVoltNum+1],
           //                     CANARegs.SysCellVoltage[CANARegs.HMICellVoltNum+2]);
               }
       break;
       case 70:
               if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_CellTempsReq==1))
               {
                   CANARegs.HMICellTempsCout++;
                   if(CANARegs.HMICellTempsCout >C_HmiCellTempCount)
                   {
                       CANARegs.HMICellTempsCout=0;
                   }
                   CANARegs.HMICellTempsNum=CANARegs.HMICellTempsCout*3;
                   //CANATX(0x703,8,CANARegs.HMICellTempsNum,
                   //               CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum],
                   //               CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum+1],
                   //               CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum+2]);
               }
       break;
       case 75:
               if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
               {
                  // CANATX(0x704,8,CANARegs.MailBox0RxCount,CANARegs.MailBox1RxCount,CANARegs.MailBox2RxCount,0x000);
               }
       break;
       case 80:
         /*    if((CANARegs.HMICMDRegs.bit.HMI_MODE==1)&&(CANARegs.HMICMDRegs.bit.HMI_CellTempsReq==1))
             {
               CANARegs.HMICellTempsCout++;
               if(CANARegs.HMICellVoltCout>C_HmiCellTempCount)
               {
                   CANARegs.HMICellVoltCout=0;
               }
               CANARegs.HMICellTempsNum=CANARegs.HMICellTempsCout*3;
               CANATX(0x704,8,CANARegs.HMICellVoltNum,
                              CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum],
                              CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum+1],
                              CANARegs.SysCelltemperature[CANARegs.HMICellTempsNum+3]);
             }
             */
       break;
       default:
       break;
   }

   switch(SysRegs.SysRegTimer300msecCount)
   {
       case 1:

       break;
       case 2:
               if(CANARegs.HMICMDRegs.bit.HMI_MODE==1)
               {
                   CANARegs.HMIISOSPIErrCount++;
                   if(CANARegs.HMIISOSPIErrCount>=C_HMIISOSPIErrCount)
                   {
                       CANARegs.HMIISOSPIErrCount=0;
                   }
                   CANARegs.HMIISOSPIErrNum=CANARegs.HMIISOSPIErrCount*3;
                   //CANATX(0x705,8,CANARegs.HMIISOSPIErrNum,
                   //               SysRegs.SlaveVoltErrCount[CANARegs.HMIISOSPIErrNum],
                   //               SysRegs.SlaveVoltErrCount[CANARegs.HMIISOSPIErrNum+1],
                   //               SysRegs.SlaveVoltErrCount[CANARegs.HMIISOSPIErrNum+2]);
               }

       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer1000msecCount)
   {
       case 1:
       break;
       default :
       break;
   }
   /*
    *
    */
   SysRegs.SysStateReg.bit.HMICOMEnable=CANARegs.HMICMDRegs.bit.HMI_MODE;
   SysRegs.SysStateReg.bit.HMIBalanceMode=CANARegs.HMICMDRegs.bit.HMI_CellBalaEn;
 //  SysRegs.SysStateReg.bit.NRlyDOStatus=SysRegs.SysDigitalInputReg.bit.NAUX;
 //  SysRegs.SysStateReg.bit.PRlyDOStatus=SysRegs.SysDigitalInputReg.bit.PAUX;
 //  SysRegs.SysStateReg.bit.PreRlyDOStatus=PrtectRelayRegs.State.bit.ProRlyDI;

   SysRegs.SysStateReg.bit.NRlyDOStatus=SysRegs.SysDigitalOutPutReg.bit.NRlyOUT;
   SysRegs.SysStateReg.bit.PRlyDOStatus=SysRegs.SysDigitalOutPutReg.bit.PRlyOUT;
   SysRegs.SysStateReg.bit.PreRlyDOStatus=PrtectRelayRegs.State.bit.ProRlyDI;


   SysRegs.SysStateReg.bit.SysRlyStatus = PrtectRelayRegs.StateMachine;
 //  SysRegs.SysStateReg.bit.SysSOCStatus = Frey60AhSocRegs.state;

   PrtectRelayRegs.State.bit.NRlyDI=SysRegs.SysDigitalOutPutReg.bit.NRlyOUT;
   PrtectRelayRegs.State.bit.PRlyDI=SysRegs.SysDigitalOutPutReg.bit.PRlyOUT;


   ProtectRlySateCheck(&PrtectRelayRegs);


   SysRegs.SysDigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRlyDO;
   SysRegs.SysDigitalOutPutReg.bit.PRlyOUT=PrtectRelayRegs.State.bit.PRlyDO;
   SysRegs.SysDigitalOutPutReg.bit.ProRlyOUT=PrtectRelayRegs.State.bit.PreRlyDO;
   SysRegs.SysProtectReg.bit.PackRly_Err=PrtectRelayRegs.State.bit.RlyFaulttSate;
   SysDigitalOutput(&SysRegs);
   InitECan();
   // Acknowledge this interrupt to receive more interrupts from group 1

//   LEDSysState_L;
   if(SysRegs.MainIsr1>3000) {SysRegs.MainIsr1=0;}
//


   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void ISR_CANRXINTA(void)
{
    struct ECAN_REGS ECanaShadow;
    if(ECanaRegs.CANGIF0.bit.GMIF0 == 1)
    {
        CANARegs.MailBoxRxCount++;
        if(CANARegs.MailBoxRxCount>250){CANARegs.MailBoxRxCount=0;LEDCANState_T;}
        if(ECanaRegs.CANRMP.bit.RMP0==1)
        {
            if(ECanaMboxes.MBOX0.MSGID.bit.STDMSGID==0x3C2)
            {
                SysRegs.CTCANErrCheck=0;
                CANARegs.MailBox0RxCount++;
                if(CANARegs.MailBox0RxCount>100){CANARegs.MailBox0RxCount=0;}
                SysRegs.SysCurrentData.byte.CurrentH   = (ECanaMboxes.MBOX0.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX0.MDL.byte.BYTE1);
                SysRegs.SysCurrentData.byte.CurrentL   = (ECanaMboxes.MBOX0.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX0.MDL.byte.BYTE3);
            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
           // ECanaShadow.CANRMP.all= 0;
            ECanaShadow.CANRMP.bit.RMP0 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP1==1)
        {

            if(ECanaMboxes.MBOX1.MSGID.bit.STDMSGID==0x600)
            {
               CANARegs.MailBox1RxCount++;
               if(CANARegs.MailBox1RxCount>250){CANARegs.MailBox1RxCount=0;}
               CANARegs.PMSCMDRegs.all      =  (ECanaMboxes.MBOX1.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX1.MDL.byte.BYTE0);
               SysRegs.VCUCANErrCheck=0;
            }
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            //ECanaShadow.CANRMP.all= 0;
            ECanaShadow.CANRMP.bit.RMP1 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;


        }
        if(ECanaRegs.CANRMP.bit.RMP2==1)
        {

            if(ECanaMboxes.MBOX2.MSGID.bit.STDMSGID==0x700)
            {
                SysRegs.HMICANErrCheck=0;
                CANARegs.MailBox2RxCount++;
                if(CANARegs.MailBox2RxCount>250){CANARegs.MailBox2RxCount=0;}

                //CANARegs.HMICMDRegs.all       =   (ECanaMboxes.MBOX2.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                //CANRXRegs.WORD700_1           =  (ECanaMboxes.MBOX2.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX2.MDL.byte.BYTE3);
                // CANARegs.HMICEllTempsAgv     =  (ECanaMboxes.MBOX2.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                // CANARegs.HMICEllVoltMin      =  (ECanaMboxes.MBOX2.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                //if(CANARegs.HMICMDRegs.bit.HMI_Reset==1)
                //{
                //    CANARegs.HMICMDRegs.bit.HMI_RlyEN=0;
                //}

                 ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
               //  ECanaShadow.CANRMP.all= 0;
                 ECanaShadow.CANRMP.bit.RMP2 = 1;
                 ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
            }

        }

         if(ECanaRegs.CANRMP.bit.RMP3==1)
        {
            if(ECanaMboxes.MBOX3.MSGID.bit.STDMSGID==0x702)
            {
                CANARegs.MailBox3RxCount++;
                if(CANARegs.MailBox3RxCount>250){CANARegs.MailBox3RxCount=0;}
                //CANRXRegs.VCUCMDCount=0;
                //CANRXRegs.PCCMDRegs.all      =  (ECanaMboxes.MBOX3.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                //CANRXRegs.WORD700_1          =  (ECanaMboxes.MBOX3.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX3.MDL.byte.BYTE3);
                //CANRXRegs.WORD700_2          =  (ECanaMboxes.MBOX3.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                //CANRXRegs.WORD700_3          =  (ECanaMboxes.MBOX3.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX3.MDH.byte.BYTE6);
            }
            CANARegs.VcuCharRxCout=0;
            CANARegs.CharRxFlg=1;
            ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
          //  ECanaShadow.CANRMP.all= 0;
            ECanaShadow.CANRMP.bit.RMP3 = 1;
            ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
        }

    }
   // ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
  //  ECanaShadow.CANRMP.all= 0;
  //  ECanaShadow.CANRMP.bit.RMP0 = 1;
  //  ECanaShadow.CANRMP.bit.RMP1 = 1;
  //  ECanaShadow.CANRMP.bit.RMP2 = 1;
  //  ECanaShadow.CANRMP.bit.RMP3 = 1;
  //  ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

    ECanaShadow.CANME.all=ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME0=1;    //0x5NA MCU Rx Enable
    ECanaShadow.CANME.bit.ME1=1;    //0x5NA MCU Rx Enable
    ECanaShadow.CANME.bit.ME2=1;    //0x5NB MCU Rx Enable
    ECanaShadow.CANME.bit.ME3=1;    //0x5NB MCU Rx Enable
    ECanaShadow.CANME.bit.ME31=1;   //CAN-A Tx Enable
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

   // IER |= 0x0100;                  // Enable INT9
   // EINT;

}//EOF
/*
interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
  // A_OVCHACurrent;
   EDIS;
}
*/
