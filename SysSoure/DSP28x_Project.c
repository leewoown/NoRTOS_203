
#include "parameter.h"
#include "SysVariable.h"
#include "DSP28x_Project.h"
#include "stdio.h"
#include "math.h"
#include <string.h>

extern void SysTimerINIT(SystemReg *s);
extern void CANRegVarINIT(CANAReg *P);
extern void SysVarINIT(SystemReg *s);
extern void SysDigitalInput(SystemReg *sys);
extern void SysDigitalOutput(SystemReg *sys);
extern void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);
extern void SysCalVoltageHandle(SystemReg *s);
extern void SysCalCurrentHandle(SystemReg *s);
extern void SysCalTemperatureHandle(SystemReg *s);
extern void MDCalVoltandTemsHandle(SystemReg *P);
extern void SysFaultCheck(SystemReg *s);
extern void SysAlarmtCheck(SystemReg *s);

extern int float32ToInt(float32 Vaule, Uint32 Num);
extern void TempTemps(SystemReg *s);
//extern SystemReg       SysRegs;

#define A 1664525
#define C 1013904223
#define M 4294967296 // 2^32


void TempTemps(SystemReg *s)
{

//    s->NumA=(float32)(s->MainIsr1/3000);
}

void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3)
{
    struct ECAN_REGS ECanaShadow;
   // unsigned int CANWatchDog;
    unsigned int Data0Low, Data0High, Data1Low, Data1High;
    unsigned int Data2Low, Data2High, Data3Low, Data3High;

 //   CANWatchDog=0;

    Data0Low  = 0x00ff&Data0;
    Data0High = 0x00ff&(Data0>>8);
    Data1Low  = 0x00ff&Data1;
    Data1High = 0x00ff&(Data1>>8);
    Data2Low  = 0x00ff&Data2;
    Data2High = 0x00ff&(Data2>>8);
    Data3Low  = 0x00ff&Data3;
    Data3High = 0x00ff&(Data3>>8);



    EALLOW;
    ECanaShadow.CANME.bit.ME31=0;
    ECanaRegs.CANME.bit.ME31= ECanaShadow.CANME.bit.ME31;

    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID=ID;

    ECanaShadow.CANME.bit.ME31=1;
    ECanaRegs.CANME.bit.ME31= ECanaShadow.CANME.bit.ME31;
    EDIS;

    ECanaMboxes.MBOX31.MSGCTRL.bit.DLC=Length;

    ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;
    ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;

    //CAN Tx Request
    ECanaShadow.CANTRS.all=0;
    ECanaShadow.CANTRS.bit.TRS31= 1;
    ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
    do
    {
        ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
        //  CANWatchDog++;
        //  if(CANWatchDog>20)
        //  {
        //      ECanaShadow.CANTA.bit.TA31=0;
        //  }
    }
    while(!ECanaShadow.CANTA.bit.TA31);

    //Tx Flag Clear
    //InitECan();
    ECanaShadow.CANTA.all = 0;
    ECanaShadow.CANTA.bit.TA31=1;                   // Clear TA5
    ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
   //
}
void SysTimerINIT(SystemReg *s)
{
    s->SysMachine=System_STATE_INIT;
    s->Maincount=0;
    s->MainIsr1=0;
    s->CANRXCOUNT=0;
    s->CANRXMailBox00Count=0;
    s->CANRXMailBox01Count=0;
    s->CANRXMailBox02Count=0;
    s->CANRXMailBox03Count=0;
    s->CANRXMailBox04Count=0;
    s->SysRegTimer5msecCount=0;
    s->SysRegTimer10msecCount=0;
    s->SysRegTimer50msecCount=0;
    s->SysRegTimer100msecCount=0;
    s->SysRegTimer300msecCount=0;
    s->SysRegTimer500msecCount=0;
    s->BalanceModeCount=0;
    s->BalanceTimeCount=0;

}
void SysVarINIT(SystemReg *s)
{
    s->CanComEable=0;
    s->PMSysCMDResg.all=0;
    s->SysStateReg.all=0;
    s->SysAlarmReg.all=0;
    s->SysFaultReg.all=0;
   // s->SysFaultBufReg.all=0;
    s->SysProtectReg.all=0;
    //s->SysProtectBufReg.all=0;
    s->SysDigitalInputReg.all=0;
    s->SysDigitalOutPutReg.all=0;
    s->SysCurrentData.all=0x80000000;
 //   s->PackCOMERR.all=0;
    s->SlaveISOSPIErrReg.all=0;
    s->IDSWReg.all=0;

//    s->PackCOMERR.all=0;

    s->Test=0;
    s->Maincount=0;
    s->MainIsr1=0;
    s->CANRXCOUNT=0;
    s->CANRXMailBox00Count=0;
    s->CANRXMailBox01Count=0;
    s->CANRXMailBox02Count=0;
    s->CANRXMailBox03Count=0;
    s->CANRXMailBox04Count=0;
    s->SysRegTimer5msecCount=0;
    s->SysRegTimer10msecCount=0;
    s->SysRegTimer50msecCount=0;
    s->SysRegTimer100msecCount=0;
    s->SysRegTimer300msecCount=0;
    s->SysRegTimer500msecCount=0;
    s->SysRegTimer1000msecCount=0;

    s->CellVoltsampling=0;
    s->CellTempssampling=0;
    s->VoltTempsReadCount=0;
//    s->SysCanRxCount=0;
    s->AlarmStatecount=0;
    s->ProtectStatecount=0;
    s->BalanceModeCount=0;
    s->BalanceTimeCount=0;

    s->RelayCheck=0;
    s->SysVoltageMaxNum=0;
    s->SysVoltageMinNum=0;
    s->SysTemperatureMaxNum=0;
    s->SysTemperatureMinNum=0;
    s->SysPackVoltageF=0;
    s->SysPackCurrentF=0;
    s->SysPackCurrentAsbF=0;
    s->SysCellMaxVoltageF=0;
    s->SysCellMinVoltageF=0;
    s->SysCellDivVoltageF=0;
    s->SysCellAgvVoltageF=0;
    s->BalanceRefVoltageF=0;
    s->SysPackParallelVoltageF=0;
    s->SysCellMaxTemperatureF=0;
    s->SysCellMaxTemperatureF=0;
    s->SysCellMinTemperatureF=0;
    s->SysCellDivTemperatureF=0;
    s->SysCellAgvTemperatureF=0;

    s->SysCHARGPWRContintyF=0;
    s->SysCHARGPWRContintyTGTF=0;
    s->SysCHARGPWRContintyDivF=0;

    s->SysDISCHAPWRContintyF=0;
    s->SysDISCHAPWRContintyTGTF=0;
    s->SysDISCHAPWRContintyDivF=0;

    s->SysCHARGPWRPeakF=0;
    s->SysDISCHAPWRPeakF=0;
    s->SysPackIsoRegsF=10.0;
    s->SysSOCF=0;
    s->SysSOHF=0;
    s->SysAhF=0;
    s->SysISOResisF=0;

   /*
    s->BAPackOCCount=0;
    s->BAPackOVCount=0;
    s->BAPackUVCount=0;
    s->BACellOVCount=0;
    s->BACellUVCount=0;
    s->BACellUBVCount=0;
    s->BACellUBTCount=0;*/

    s->VCUCANErrCheck=0;
    s->CTCANErrCheck=0;
    s->HMICANErrCheck=0;
    s->SlaveReadVoltEn.all =0;
    s->SlaveReadVoltEn.bit.SlaveBMS00=1;
    s->SlaveReadVoltEn.bit.SlaveBMS01=1;
    s->SlaveReadVoltEn.bit.SlaveBMS02=1;
    s->SlaveReadVoltEn.bit.SlaveBMS03=1;
    s->SlaveReadVoltEn.bit.SlaveBMS04=1;
    s->SlaveReadVoltEn.bit.SlaveBMS05=1;

    s->SlaveReadVoltEn.bit.SlaveBMS06=1;
    s->SlaveReadVoltEn.bit.SlaveBMS07=1;
    s->SlaveReadVoltEn.bit.SlaveBMS08=1;
    s->SlaveReadVoltEn.bit.SlaveBMS09=1;
    s->SlaveReadVoltEn.bit.SlaveBMS10=1;
    s->SlaveReadVoltEn.bit.SlaveBMS11=1;

    s->SlaveReadTempsEn.all = 0;
    s->SlaveReadTempsEn.bit.SlaveBMS00=1;
    s->SlaveReadTempsEn.bit.SlaveBMS01=1;
    s->SlaveReadTempsEn.bit.SlaveBMS02=1;
    s->SlaveReadTempsEn.bit.SlaveBMS03=1;
    s->SlaveReadTempsEn.bit.SlaveBMS04=1;
    s->SlaveReadTempsEn.bit.SlaveBMS05=1;

    s->SlaveReadTempsEn.bit.SlaveBMS06=1;
    s->SlaveReadTempsEn.bit.SlaveBMS07=1;
    s->SlaveReadTempsEn.bit.SlaveBMS08=1;
    s->SlaveReadTempsEn.bit.SlaveBMS09=1;
    s->SlaveReadTempsEn.bit.SlaveBMS10=1;
    s->SlaveReadTempsEn.bit.SlaveBMS11=1;


    memset(&s->SlaveVoltErrCount[0],0,sizeof(Uint16)*32);
    memset(&s->SlaveTempsErrCount[0],0,sizeof(Uint16)*32);
    memset(&s->SlaveBalanErrCount[0],0,sizeof(Uint16)*32);
    memset(&s->ProtectDelayCount[0],0,sizeof(Uint16)*32);
    memset(&s->SysCellVoltageF[0],0.0,sizeof(float32)*C_SysCellVoltEa);
    memset(&s->SysCelltemperatureF[0],0.0,sizeof(float32)*C_SysCellTempEa);
    memset(&s->MDVoltageF[0],0.0,sizeof(float32)*6);
    memset(&s->MDCellVoltAgvF[0],0.0,sizeof(float32)*6);
    memset(&s->MDCellTempsAgvF[0],0.0,sizeof(float32)*6);
}
void CANRegVarINIT(CANAReg *P)
{
/*    P->SWTypeVer=0;

    P->CellNumStart=0;;
    P-> NumberShift=0;;
    P->CellVotlageNumber=0;;
    P->CellVotlageMaxNumber=0;;
    P->CellVoltageNum=0;
    P->PMSCMDRegs.all=0;
    P->BAT80VDigitalOutPutReg.all=0;
    P->SwVerProducttype.all=0;
    P->BatConfParallelSerial.all=0;
    memset(&P->Salve1VoltageCell[0],0,12);
    P->CellNumTStart=0;
    P->NumberTShift=0;
    P->CellTemperatureNumber=0;
    P->CellTemperatureMaxNumber=0;
    P->CellTemperatureNum=0;
    memset(&P->Salve1temperatureCell[0],0,12);*/
    /*
     *
     */
    P->SysConFig=0;
    P->SysStatus.all=0;
    P->SysPackSOC=0;
    P->SysPackSOH=0;
    P->SysPackAh=0;
    P->SysPackAh=0;
    P->SysPackPT=0;
    P->SysPackVotageBuf=0;
    P->SysPackVotageBuf=0;
    P->SysCHARGPWRContinty=0;
    P->SysDISCHAPWRContinty=0;
    P->SysCHARGPWRPeak=0;
    P->SysDISCHAPWRPeak=0;
  //  P->SysCellVoltage[SysCellVoltCount];
  //  P->SysCelltemperature[SysCellVoltCount];
    P->CellVoltageMax=0;
    P->CellVoltageMin=0;
    P->CellVoltageAgv=0;
    P->CellVoltageDiv=0;
    P->CellVoltageMaxNum=0;
    P->CellVoltageMinNum=0;

    P->CellTemperaturelMAX=0;
    P->CellTemperaturelMIN=0;
    P->CellTemperatureAVG=0;
    P->CellTemperatureDiv=0;
    P->CellTemperatureMaxNum=0;
    P->CellTemperatureMinNUM=0;
    P->HMICMDRegs.all=0;
    P->HMICEllVoltMin=0;
    P->HMICEllTempsAgv=0;

    P->HMICellVoltCout=0;
    P->HMICellTempsCout=0;
    P->HMICellVoltNum=0;
    P->HMICellTempsNum=0;
    P->HMIISOSPIErrNum=0;
    P->MailBoxRxCount=0;
    P->MailBox0RxCount=0;
    P->MailBox1RxCount=0;
    P->MailBox2RxCount=0;
    P->MailBox3RxCount=0;
    memset(&P->MDVoltage[0],0.0,6);
    memset(&P->MDCellVoltAgv[0],0.0,6);
    memset(&P->MDCellTempsAgv[0],0.0,6);
    memset(&P->SysCellVoltage[0],0.0,C_SysCellTempEa);
    memset(&P->SysCelltemperature[0],0.0,C_SysCellTempEa);
    P->PMSCMDRegs.all =0;
    P->HMICMDRegs.all =0;
    P->ChargerStateRegs.all=0;
    P->SysStatus.all=0;
    P->DigitalOutPutReg.all=0;
    P->VcuRxFlg=0;
    P->CharRxFlg=0;
    P->VcuCharRxCout=0;
    P->CANCom_0x61DDate0=0;
    P->CANCom_0x61DDate1=0;
    P->CANCom_0x61DDate2=0;
    P->CANCom_0x61DDate3=0;
    P->SlaveBMSNumCout=0;
    P->CellNumCount=0;
}
void MDCalInit(SystemReg *P)
{
    memset(&P->MDVoltageF[0],0.0,sizeof(float32)*6);
    memset(&(P->MDCellVoltAgvF[0]),0.0,sizeof(float32)*6);
    memset(&P->MDCellTempsAgvF[0],0.0,sizeof(float32)*6);
    P->MDNumber=2;
}

void MDCalVoltandTemsHandle(SystemReg *P)
{

  int i=0;
  float32 ModVolt=0.0;
  float32 ModTemps=0.0;
  unsigned int MDCellPosStart=0;
  unsigned int MDCellPosCount=0;
  //unsigned int MDCellPosStop=0;
  if(P->MDNumber>C_SysModuleEa)
  {
      P->MDNumber=0;
  }
  MDCellPosStart = P->MDNumber  * C_ModuleMEAEa;
  MDCellPosCount = MDCellPosStart+C_ModuleMEAEa;
  switch(P->MDNumber)
  {
      case 0:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 1:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 2:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 3:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 4:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 5:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 6:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 7:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 8:
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModVolt=ModVolt+P->SysCellVoltageF[i];
              }
              for(i=MDCellPosStart; i<MDCellPosCount ;i++)
              {
                  ModTemps=ModTemps+P->SysCelltemperatureF[i];
              }
              P->MDVoltageF[P->MDNumber]=ModVolt;
              P->MDCellVoltAgvF[P->MDNumber]=P->MDVoltageF[P->MDNumber]/C_ModuleMEAEa;
              P->MDCellTempsAgvF[P->MDNumber]=ModTemps/C_ModuleMEAEa;
      break;
      case 9:

      break;
      default:
      break;
  }

}
void SysCalVoltageHandle(SystemReg *s)
{

    Uint16  CellCount=0;
    Uint16  CellSize=0;
    Uint16  ModuleCount=0;
    Uint16  ModuleSize=0;
    float32 SysModuleMaxVoltageF=0.0;
    float32 SysModuleMinVoltageF=0.0;
    float32 SysCellMaxVoltageF=0;
    float32 SysCellMinVoltageF=0;
    float32 SysVoltageBufF=0;
    SysCellMaxVoltageF =s->SysCellVoltageF[0];
    SysCellMinVoltageF =s->SysCellVoltageF[0];
    CellSize = C_SysCellVoltEa;//24
    ModuleSize=C_SysModuleEa;
    for(CellCount=0;CellCount<CellSize;CellCount++)
    {
         if (SysCellMaxVoltageF <= s->SysCellVoltageF[CellCount])
         {
             SysCellMaxVoltageF    =  s->SysCellVoltageF[CellCount];
             s->SysVoltageMaxNum=CellCount;
         }
         if (SysCellMinVoltageF >= s->SysCellVoltageF[CellCount])
         {
             SysCellMinVoltageF    =  s->SysCellVoltageF[CellCount];
             s->SysVoltageMinNum=CellCount;
         }
    }
    for(CellCount=0;CellCount<CellSize;CellCount++)
    s->SysCellMaxVoltageF    = SysCellMaxVoltageF;
    s->SysCellMinVoltageF    = SysCellMinVoltageF;
    s->SysCellDivVoltageF    = s->SysCellMaxVoltageF-s->SysCellMinVoltageF;
    CellSize = C_SysCellVoltEa;
    for(CellCount=0;CellCount<CellSize;CellCount++)
    {
        SysVoltageBufF = SysVoltageBufF+ s->SysCellVoltageF[CellCount];
    }
    s->SysPackVoltageF      = SysVoltageBufF;
    s->SysCellAgvVoltageF   =  (float32)s->SysPackVoltageF/CellSize;

    //
    SysModuleMinVoltageF =s->MDVoltageF[0];
    SysModuleMaxVoltageF =s->MDVoltageF[0];
    ModuleSize=2;
    for(ModuleCount=0;ModuleCount<ModuleSize;ModuleCount++)
    {
        if (SysModuleMaxVoltageF <= s->MDVoltageF[ModuleCount])
        {
            SysModuleMaxVoltageF =  s->MDVoltageF[ModuleCount];
        }
        if (SysModuleMinVoltageF >= s->MDVoltageF[ModuleCount])
        {
            SysModuleMinVoltageF    =  s->MDVoltageF[CellCount];
        }
    }
    s->SysPackParallelVoltageF= SysModuleMaxVoltageF;
}
void SysCalTemperatureHandle(SystemReg *s)
{

    Uint16  CellCount=0;
    Uint16  CellSize=0;
    float32 SysCellMaxTemperatureF=0;
    float32 SysCellMinTemperatureF=0;
    float32 SysTemperatureBufF=0;
    SysCellMaxTemperatureF =s->SysCelltemperatureF[0];
    SysCellMinTemperatureF =s->SysCelltemperatureF[0];
    CellSize = C_SysCellVoltEa;
    for(CellCount=0;CellCount<CellSize;CellCount++)
    {
         if (SysCellMaxTemperatureF <= s->SysCelltemperatureF[CellCount])
         {
             SysCellMaxTemperatureF    =  s->SysCelltemperatureF[CellCount];
             s->SysTemperatureMaxNum=CellCount;
         }
         if (SysCellMinTemperatureF >= s->SysCelltemperatureF[CellCount])
         {
             SysCellMinTemperatureF    =  s->SysCelltemperatureF[CellCount];
             s->SysTemperatureMinNum=CellCount;
         }
    }
    s->SysCellMaxTemperatureF    = SysCellMaxTemperatureF;
    s->SysCellMinTemperatureF    = SysCellMinTemperatureF;
    s->SysCellDivTemperatureF    = s->SysCellMaxTemperatureF-s->SysCellMinTemperatureF;
    CellSize = C_SysCellVoltEa;
    for(CellCount=0;CellCount<CellSize;CellCount++)
    {
        SysTemperatureBufF = SysTemperatureBufF+ s->SysCelltemperatureF[CellCount];
    }
    s->SysCellAgvTemperatureF   = (float32)SysTemperatureBufF/CellSize;
}

void SysCalCurrentHandle(SystemReg *s)
{
    long  CurrentCT  = 0;
    float32 Currentbuf = 0;
    CurrentCT  = s->SysCurrentData.all;
    CurrentCT  =  CurrentCT - 0x80000000;

    Currentbuf        =  ((float)CurrentCT)/1000;          // (mA to A) CAB500 resolution 1mA
    s->SysPackCurrentF  = -1.0 * Currentbuf;    // Decide Current sensor's direction

    if(s->SysPackCurrentF>=500.0)
    {
        s->SysPackCurrentF=500.0;
    }
    if(s->SysPackCurrentF<=-500.0)
    {
        s->SysPackCurrentF=-500.0;
    }
    if(s->SysPackCurrentF < 0)
    {
        s->SysPackCurrentAsbF =-1.0 * s->SysPackCurrentF;
    }
    else
    {
        s->SysPackCurrentAsbF =s->SysPackCurrentF;
    }

}
void SysAlarmtCheck(SystemReg *s)
{
    if(s->SysStateReg.bit.SysDisCharMode==1)
    {
        if(s->SysPackCurrentAsbF>=C_PackDISCH_OCFault)
        {
            s->SysAlarmReg.bit.CellTempsDisch_OT=1;
        }
    }
    if(s->SysStateReg.bit.SysDisCharMode==0)
    {
        if(s->SysPackCurrentAsbF>=C_PackCHARG_OCFault)
        {
            s->SysAlarmReg.bit.CellTempsCharg_OT=1;
        }
    }
    if(s->SysSOCF >= C_PackSOC_OVFault)
    {
        s->SysAlarmReg.bit.PackVSOC_OV=1;
    }
    if(s->SysSOCF <= C_PackSOC_UNFault)
    {
        s->SysAlarmReg.bit.PackVSOC_UN=1;
    }
    if(s->SysPackVoltageF>=C_PackVolt_OVFault)
    {
        s->SysAlarmReg.bit.PackVolt_OV=1;
    }
    if(s->SysPackVoltageF<= C_PackVolt_UNFault)
    {
        s->SysAlarmReg.bit.PackVolt_UN=1;
    }
    if(s->SysCellMaxVoltageF>= C_CellVolt_OVFault)
    {
        s->SysAlarmReg.bit.CellVolt_OV=1;
    }
    if(s->SysCellMinVoltageF<= C_CellVolt_UNFault)
    {
        s->SysAlarmReg.bit.CellVolt_UN=1;
    }
    if(s->SysCellDivVoltageF>=C_CellVolt_UBFault)
    {
        s->SysAlarmReg.bit.CellVolt_BL=1;
    }
    if(s->SysStateReg.bit.SysDisCharMode==1)
    {
        if(s->SysMaxTemperatureF>=C_CellTempsDISCH_OTFault)
        {
            s->SysAlarmReg.bit.CellTempsDisch_OT=1;
        }
        if(s->SysCellMinTemperatureF>=C_CellTempsDISCH_UTFault)
        {
            s->SysAlarmReg.bit.CellTempsDisCh_UT=1;
        }
    }
    if(s->SysStateReg.bit.SysDisCharMode==0)
    {
        if(s->SysMaxTemperatureF>=C_CellTempsCHARG_OTFault)
        {
            s->SysAlarmReg.bit.CellTempsCharg_OT=1;
        }
        if(s->SysCellMinTemperatureF>=C_CellTempsCHARG_UTFault)
        {
            s->SysAlarmReg.bit.CellTempsCharg_UT=1;
        }
    }
    if(s->SysCellDivTemperatureF>=C_CellTemps_UBFault)
    {
        s->SysAlarmReg.bit.CellTemp_BL=1;
    }
    if(s->SysDISCHAPWRContintyDivF>=C_PackPWRDISCH_UBPrtct)
    {
        s->SysAlarmReg.bit.PackDischarUnPWR_BL=1;
    }
    if(s->SysCHARGPWRContintyDivF>=C_PackPWRCHARG_UBPrtct)
    {
        s->SysAlarmReg.bit.PackCharUnPWR_BL=1;
    }
}
void SysFaultCheck(SystemReg *s)
{
    if(s->SysStateReg.bit.SysDisCharMode==1)
    {
        if(s->SysPackCurrentAsbF>=C_PackDISCH_OCFault)
        {
            s->SysFaultReg.bit.CellTempsDisch_OT=1;
        }
    }
    if(s->SysStateReg.bit.SysDisCharMode==0)
    {
        if(s->SysPackCurrentAsbF>=C_PackCHARG_OCFault)
        {
            s->SysFaultReg.bit.CellTempsCharg_OT=1;
        }
    }
    if(s->SysSOCF >=C_PackSOC_OVFault)
    {
        s->SysFaultReg.bit.PackVSOC_OV=1;
    }
    if(s->SysSOCF <= C_PackSOC_UNFault)
    {
        s->SysFaultReg.bit.PackVSOC_UN=1;
    }
    if(s->SysPackVoltageF>=C_PackVolt_OVFault)
    {
        s->SysFaultReg.bit.PackVolt_OV=1;
    }
    if(s->SysPackVoltageF<= C_PackVolt_UNFault)
    {
        s->SysFaultReg.bit.PackVolt_UN=1;
    }
    if(s->SysCellMaxVoltageF>= C_CellVolt_OVFault)
    {
        s->SysFaultReg.bit.CellVolt_OV=1;
    }
    if(s->SysCellMinVoltageF<= C_CellVolt_UNFault)
    {
        s->SysFaultReg.bit.CellVolt_UN=1;
    }
    if(s->SysCellDivVoltageF>=C_CellVolt_UBFault)
    {
        s->SysFaultReg.bit.CellVolt_BL=1;
    }
    if(s->SysStateReg.bit.SysDisCharMode==1)
    {
        if(s->SysMaxTemperatureF>=C_CellTempsDISCH_OTFault)
        {
            s->SysFaultReg.bit.CellTempsDisch_OT=1;
        }
        if(s->SysCellMinTemperatureF>=C_CellTempsDISCH_UTFault)
        {
            s->SysFaultReg.bit.CellTempsDisCh_UT=1;
        }
    }
    if(s->SysStateReg.bit.SysDisCharMode==0)
    {
        if(s->SysMaxTemperatureF>=C_CellTempsCHARG_OTFault)
        {
            s->SysFaultReg.bit.CellTempsCharg_OT=1;
        }
        if(s->SysCellMinTemperatureF>=C_CellTempsCHARG_UTFault)
        {
            s->SysFaultReg.bit.CellTempsCharg_UT=1;
        }
    }
    if(s->SysCellDivTemperatureF>=C_CellTemps_UBFault)
    {
        s->SysFaultReg.bit.CellTemp_BL=1;
    }

    if(s->SysDISCHAPWRContintyDivF >= C_PackPWRDISCH_UBPrtct)
    {
        s->SysFaultReg.bit.PackDischarUnPWR_BL=1;
    }
    if(s->SysCHARGPWRContintyDivF >= C_PackPWRCHARG_UBPrtct)
    {
        s->SysFaultReg.bit.PackCharUnPWR_BL=1;
    }
}
void SysProtectCheck(SystemReg *s)
{

    /*
     * 과전류
     */
    if(s->SysStateReg.bit.SysDisCharMode==1)
    {
        if(s->SysPackCurrentAsbF>=C_PackDISCH_OCPrtct)
        {
            s->ProtectDelayCount[0]++;
        }
        else
        {
            s->ProtectDelayCount[0]=0;

        }
        s->ProtectDelayCount[1]=0;
    }
    else
    {
        if(s->SysPackCurrentAsbF>=C_PackCHARG_OCPrtct)
        {
            s->ProtectDelayCount[1]++;
        }
        else
        {
            s->ProtectDelayCount[1]=0;
        }
        s->ProtectDelayCount[0]=0;
    }
    // SOC 과충전
    if(s->SysSOCF>=C_PackSOC_OVPrtct)
    {
        s->ProtectDelayCount[2]++;
    }
    else
    {
        s->ProtectDelayCount[2]=0;
    }
    // SOC 과방전
    if(s->SysSOCF<= C_PackSOC_UNPrtct)
    {
        s->ProtectDelayCount[3]++;
    }
    else
    {
        s->ProtectDelayCount[3]=0;
    }
    //PACK 과전압
    if(s->SysPackParallelVoltageF>=C_PackVolt_OVPrtct)
    {
        s->ProtectDelayCount[4]++;
    }
    else
    {
        s->ProtectDelayCount[4]=0;
    }
    //PACK 저전압
    if(s->SysPackParallelVoltageF<=C_PackVolt_UNPrtct)
    {
        s->ProtectDelayCount[5]++;
    }
    else
    {
        s->ProtectDelayCount[5]=0;
    }
    /*
    if(s->SysPackVoltageF>=C_PackVolt_OVPrtct)
    {
        s->ProtectDelayCount[4]++;
    }
    else
    {
        s->ProtectDelayCount[4]=0;
    }
    //PACK 저전압
    if(s->SysPackVoltageF<=C_PackVolt_UNPrtct)
    {
        s->ProtectDelayCount[5]++;
    }
    else
    {
        s->ProtectDelayCount[5]=0;
    }*/
    //셀 과전압
    if(s->SysCellMaxVoltageF>=C_CellVolt_OVPrtct)
    {
        s->ProtectDelayCount[6]++;
    }
    else
    {
        s->ProtectDelayCount[6]=0;
    }
    //셀 저전압
    if(s->SysCellMinVoltageF<=C_CellVolt_UNPrtct)
    {
        s->ProtectDelayCount[7]++;
    }
    else
    {
        s->ProtectDelayCount[7]=0;
    }
    //셀 전압 불균형
    if(s->SysCellDivVoltageF>=C_CellVolt_UBPrtct)
    {
        s->ProtectDelayCount[8]++;
    }
    else
    {
        s->ProtectDelayCount[8]=0;
    }
    // 방전 시에 셀 과온, 저온 보호
    if(s->SysStateReg.bit.SysDisCharMode==1)
    {
        if(s->SysMaxTemperatureF>=C_CellTempsDISCH_OTPrtct)
        {
            s->ProtectDelayCount[9]++;
        }
        else
        {
            s->ProtectDelayCount[9]=0;
        }
        if(s->SysCellMinTemperatureF<=C_CellTempsDISCH_UTPrtct)
        {
            s->ProtectDelayCount[10]++;
        }
        else
        {
            s->ProtectDelayCount[10]=0;
        }
    }
    if(s->SysStateReg.bit.SysDisCharMode==0)
    {
        if(s->SysCellMinTemperatureF>=C_CellTempsCHARG_OTPrtct)
        {
            s->ProtectDelayCount[11]++;
        }
        else
        {
            s->ProtectDelayCount[11]=0;
        }
        if(s->SysCellMinTemperatureF<=C_CellTempsCHARG_UTPrtct)
        {
            s->ProtectDelayCount[12]++;
        }
        else
        {
            s->ProtectDelayCount[12]=0;
        }
    }
    if(s->SysCellDivTemperatureF>=C_CellTemps_UBPrtct)
    {
        s->ProtectDelayCount[13]++;
    }
    else
    {
        s->ProtectDelayCount[13]=0;
    }
    if(s->SysDISCHAPWRContintyDivF>=C_PackPWRDISCH_UBPrtct)
    {
        s->ProtectDelayCount[15]++;
    }
    else
    {
        s->ProtectDelayCount[15]=0;
    }
    if(s->SysCHARGPWRContintyDivF>=C_PackPWRCHARG_UBPrtct)
    {
        s->ProtectDelayCount[14]++;
    }
    else
    {
        s->ProtectDelayCount[14]=0;
    }
    /*
     *
     */
    if(s->ProtectDelayCount[0] > C_PackDISCHOC_PrtectDelay)// C_PackDISCHOC
    {
        s->SysProtectReg.bit.PackVDisChar_OC=1;
    }
    if(s->ProtectDelayCount[1] > C_PackCHARGOC_PrtectDelay)//C_PackCHARGOC
    {
        s->SysProtectReg.bit.PackVChar_OC=1;
    }
    if(s->ProtectDelayCount[2] > C_PackSOCOV_PrtectDelay)
    {
        s->SysProtectReg.bit.PackVSOC_OV=1;
    }
    if(s->ProtectDelayCount[3] > C_PackSOCUN_PrtectDelay)
    {
        s->SysProtectReg.bit.PackVSOC_UN=1;
    }
    if(s->ProtectDelayCount[4] > C_PackVoltOV_PrtectDelay)
    {
        s->SysProtectReg.bit.PackVolt_OV=1;
    }
    if(s->ProtectDelayCount[5] > C_PackVoltUV_PrtectDelay)
    {
        s->SysProtectReg.bit.PackVolt_UN=1;
    }
    if(s->ProtectDelayCount[6] > C_CellVoltOV_PrtectDelay)
    {
        s->SysProtectReg.bit.CellVolt_OV=1;
    }
    if(s->ProtectDelayCount[7] > C_CellVoltUN_PrtectDelay)
    {
        s->SysProtectReg.bit.CellVolt_UN=1;
    }
    if(s->ProtectDelayCount[8] > C_CellVoltUB_PrtectDelay)
    {
        s->SysProtectReg.bit.CellVolt_BL=1;
    }
    if(s->SysStateReg.bit.SysDisCharMode==1)
    {
        if(s->ProtectDelayCount[9] > C_CellTempsDISCHOT_PrtectDelay)
        {
            s->SysProtectReg.bit.CellTempsDisch_OT=1;
        }
        if(s->ProtectDelayCount[10] > C_CellTempsCHARGOT_PrtectDelay)
        {
            s->SysProtectReg.bit.CellTempsDisCh_UT=1;
        }
        s->ProtectDelayCount[11]=0;
        s->ProtectDelayCount[12]=0;
    }
    if(s->SysStateReg.bit.SysDisCharMode==0)
    {
        if(s->ProtectDelayCount[11] > C_CellTempsCHARGOT_PrtectDelay)
        {
            s->SysProtectReg.bit.CellTempsCharg_OT=1;
        }
        if(s->ProtectDelayCount[12] > C_CellTempsCHARGUT_PrtectDelay)
        {
            s->SysProtectReg.bit.CellTempsCharg_UT=1;
        }
        s->ProtectDelayCount[9]=0;
        s->ProtectDelayCount[10]=0;
    }
    if(s->ProtectDelayCount[13] > C_CellTempsUB_PrtectDelay)
    {
        s->SysProtectReg.bit.CellTemp_BL=1;
    }
    if(s->ProtectDelayCount[14] > C_PackPWRCHARGUB_PrtectDelay)
    {
        s->SysProtectReg.bit.PackCharUnPWR_BL=1;
    }
    if(s->ProtectDelayCount[15] > C_PackPWRDISCHUB_PrtectDelay)
    {
        s->SysProtectReg.bit.PackDischarUnPWR_BL=1;
    }
    /*
     * Relay 에러 체크
     */
    if((s->SysMachine==System_STATE_INIT)||(s->SysMachine==System_STATE_STANDBY))
    {
        if((s->SysDigitalInputReg.bit.NAUX==1)||(s->SysDigitalInputReg.bit.PAUX==1))
        {
            s->SysProtectReg.bit.PackRly_Err=1;
        }
    }
    /*
     * ISOSPI 통신 상태 체크
     */
    if(s->SlaveISOSPIErrReg.Word.DataL>0)
    {
        s->SysProtectReg.bit.PackIOSPI_Err=1;
    }
    /*
     * CAN 통신 상태 체크
     */
    s->CTCANErrCheck++;
    if(s->CTCANErrCheck>=50)
    {
       s->SysProtectReg.bit.PackCT_Err=1;
    }
    s->VCUCANErrCheck++;
    if(s->VCUCANErrCheck>=500)
    {
       s->SysProtectReg.bit.PackCAN_Err=1;
    }
    /*
     *
     */
    if(s->SysPackIsoRegsF<C_PackISORegsister_URPrtct)
    {
        s->SysProtectReg.bit.PackISOReg_Err=1;
    }

}
int float32ToInt(float32 Vaule, Uint32 Num)
{
    Uint32 intVaule=0;
    intVaule = roundf(Vaule*10)/10;

    return (Uint32)intVaule;
}
void SysDigitalInput(SystemReg *sys)
{
    if(IDSW00==1)
    {
        sys->IDSWReg.bit.SW00=0;
    }
    else
    {
        sys->IDSWReg.bit.SW00=1;
    }
    if(IDSW01==1)
    {
        sys->IDSWReg.bit.SW01=0;
    }
    else
    {
        sys->IDSWReg.bit.SW01=1;
    }
/*
    if(IDSW02==1)
    {
        sys->IDSWReg.bit.SW02=0;
    }
    else
    {
        sys->IDSWReg.bit.SW02=1;
    }
    */
    if(IDSW03==1)
    {
        sys->IDSWReg.bit.SW03=0;
    }
    else
    {
        sys->IDSWReg.bit.SW03=1;
    }
    if(CANRX0INT==0)
    {
        sys->SysDigitalInputReg.bit.CANRX0=1;
    }
    else
    {
        sys->SysDigitalInputReg.bit.CANRX0=0;
    }
    if(CANRX1INT==0)
    {
        sys->SysDigitalInputReg.bit.CANRX1=1;
    }
    else
    {
        sys->SysDigitalInputReg.bit.CANRX1=0;
    }

    if(PRlyState==0)
    {
        sys->SysDigitalInputReg.bit.PAUX=1;
    }
    else
    {
        sys->SysDigitalInputReg.bit.PAUX=0;
    }
    if(NRlyState==0)
    {
        sys->SysDigitalInputReg.bit.NAUX=1;
    }
    else
    {
        sys->SysDigitalInputReg.bit.NAUX=0;
    }
/*    if(EMGSWDI==0)
    {
        sys->SysDigitalInputReg.bit.EMGSWStauts=1;
    }
    else
    {
        sys->SysDigitalInputReg.bit.EMGSWStauts=0;
    }
*/


}
void SysDigitalOutput(SystemReg *sys)
{
    if(sys->SysDigitalOutPutReg.bit.NRlyOUT==1)
    {
        NRlyOn;
    }
    else
    {
        NRlyOff;
    }
    if(sys->SysDigitalOutPutReg.bit.ProRlyOUT==1)
    {
        PRORlyOn;
    }
    else
    {
        PRORlyOff;
    }
    if(sys->SysDigitalOutPutReg.bit.PRlyOUT==1)
    {
        PRlyOn;
    }
    else
    {
        PRlyOff;
    }
/*
    if(sys->SysDigitalOutPutReg.bit.PWRLAMPOUT==1)
    {
        PWRLAMPOn;
    }
    else
    {
        sys->PWRLAMPCount++;
        if(sys->PWRLAMPCount>1200)
        {
            PWRLAMPTog;
            sys->PWRLAMPCount=0;
        }
    }
*/
    if(sys->SysDigitalOutPutReg.bit.LEDAlarmOUT==1)
    {
        sys->LEDFaultCount++;
        if(sys->LEDFaultCount>1200)
        {
            LEDFault_T;
            sys->LEDFaultCount=0;
        }
    }
    if(sys->SysDigitalOutPutReg.bit.LEDProtectOUT==1)
    {
        sys->LEDFaultCount++;
        if(sys->LEDFaultCount>200)
        {
            LEDFault_T;
            sys->LEDFaultCount=0;
        }
    }
    if((sys->SysDigitalOutPutReg.bit.LEDAlarmOUT==0)&&(sys->SysDigitalOutPutReg.bit.LEDProtectOUT==0))
    {
        LEDFault_H;
    }
    if(sys->SysDigitalOutPutReg.bit.PWRHOLD==1)
    {
        LatchResetRlyON;
        LatchSetRlyON;
    }
    else
    {
        LatchResetRlyOFF;
        LatchResetRlyOFF;
    }
}
void TimerinitHandle(TimerReg *timer)
{
  timer->state = TIMER_STATE_IDLE;
  timer->TimeCount = 0;
  timer->Start=0;
  timer->Stop=0;
  timer->OutState=0;
  timer->Reset=0;
  timer->TimerVaule=0;
}
void ProtectRelayTimerHandle(TimerReg *timer)
{
  switch (timer->state)
  {
    case TIMER_STATE_IDLE:
      // 타이머 시작
      if (timer->Start==1)
      {
        timer->state = TIMER_STATE_RUNNING;
      }
      if (timer->Reset==1)
      {
         timer->state = TIMER_STATE_CLEAR;
      }
      break;
    case TIMER_STATE_RUNNING:
      // 타이머 만료 확인
      if (timer->TimeCount >= timer->TimerVaule)
      {
        timer->state = TIMER_STATE_EXPIRED;
      }
      // 타이머 증가
      timer->TimeCount++;
      break;
    case TIMER_STATE_EXPIRED:
         // 타이머 만료 처리
         timer->OutState = 1;
         // 타이머 재시작
         if (timer->Reset==1)
         {
            timer->state = TIMER_STATE_CLEAR;
         }
    break;
    case TIMER_STATE_CLEAR:
         timer->state = TIMER_STATE_IDLE;
         timer->TimeCount = 0;
         timer->Start=0;
         timer->Stop=0;
         timer->OutState=0;
         timer->TimerVaule=0;
         timer->Reset=0;
    break;
  }
}
void PWRHoldHandle(SystemReg *P)
{
    if(P->SysStateReg.bit.INITOK==0)
    {
        P->SysDigitalOutPutReg.bit.PWRHOLD=0;
    }
    if(P->SysStateReg.bit.INITOK==1)
    {
        P->SysDigitalOutPutReg.bit.PWRHOLD=1;
    }
    if(P->SysCellDivVoltageF<= C_BalanceDivVoltage)
    {
        P->SysDigitalOutPutReg.bit.PWRHOLD=0;
    }
}




