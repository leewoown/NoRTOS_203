


#include "DSP28x_Project.h"
#include "parameter.h"
#include "ProtectRelay.h"
#include <stdio.h>
#include <math.h>
#include <string.h>


extern void ProtectRlySateCheck(PrtectRelayReg *P);
extern void ProtectRlyVarINIT(PrtectRelayReg *P);
extern void ProtectRlyOnInit(PrtectRelayReg *P);
extern void ProtectRlyOnHandle(PrtectRelayReg *P);
extern void ProtectRlyOffInit(PrtectRelayReg *P);
extern void ProtectRlyOffHandle(PrtectRelayReg *P);
extern void ProtectRlyEMSHandle(PrtectRelayReg *P);


void ProtectRlyVarINIT(PrtectRelayReg *P)
{
    P->State.all=0;
    P->WakeupOn_ProRlyOnCount=0;
    P->WakeupOn_ProRlyOffCount=0;
    P->WakeupOn_PRlyOnCount=0;
    P->WakeupOn_NRlyOnCount=0;
    P->WakeupOff_PRlyOffCount=0;
    P->WakeupOff_NRlyOffCount=0;
    P->Protect_ProRlyOnCount=0;
    P->Protect_ProRlyOffCount=0;
    P->Protect_PRlyOffCount=0;
    P->Protect_NRlyOffCount=0;
    P->WakeupOn_TimeCount=0;
    P->WakeupOff_TimeCount=0;
}

void ProtectRlySateCheck(PrtectRelayReg *P)
{
      switch(P->StateMachine)
      {
          case PrtctRly_INIT :
               ProtectRlyVarINIT(P);
               P->StateMachine=PrtctRly_STANDBY;
          break;
          case PrtctRly_STANDBY :
               if((P->State.bit.PRlyDI==1)||(P->State.bit.NRlyDI==1))
               {
                   P->State.bit.RlyFaulttSate=1;
                   P->StateMachine=PrtctRly_RLYProtect;
               }
               P->StateMachine=PrtctRly_Ready;
          break;
          case PrtctRly_Ready :
                ProtectRlyOnInit(P);
                ProtectRlyOffInit(P);
               if(P->State.bit.WakeUpEN==1)
               {
                  P->StateMachine =PrtctRly_RuningON;
               }
          break;
          case PrtctRly_RuningON :
              ProtectRlyOffInit(P);
              ProtectRlyOnHandle(P);
              P->WakeupOn_TimeCount++;
             if(P->WakeupOn_TimeCount>WakeUpONTimeOut)
              {
                  P->WakeupOn_TimeCount=P->WakeupOn_TimeCount+10;
                  P->State.bit.WakeupOnTiemrErr=1;
                  P->StateMachine=PrtctRly_RLYProtect;
              }
              if(P->State.bit.WakeUpEN==0)
              {
                  P->StateMachine =PrtctRly_RuningOFF;
              }
          break;
          case PrtctRly_RuningOFF :
               ProtectRlyOnInit(P);
               ProtectRlyOffHandle(P);
               P->WakeupOff_TimeCount++;
               if(P->WakeupOff_TimeCount>WakeUpOFFTimeOut)
               {
                   P->WakeupOff_TimeCount = WakeUpOFFTimeOut+10;
                   P->State.bit.WakeupOFFTiemrErr=1;
                   P->StateMachine=PrtctRly_RLYProtect;
               }
               if((P->State.bit.PRlyDI==0)&&(P->State.bit.NRlyDI==0))
               {
                   P->StateMachine =PrtctRly_Ready;
               }
          break;
          case PrtctRly_ProtectpOFF :
               ProtectRlyEMSHandle(P);
               P->StateMachine =PrtctRly_CLEAR;
          break;
          case PrtctRly_RLYProtect :
          //     P->State.bit.FaultSeqErr=1;
               //PRlyOff;
               //NRlyOff;
               //PRORlyOf
          break;
          case PrtctRly_CLEAR :
              ProtectRlyVarINIT(P);
              ProtectRlyOnInit(P);
              ProtectRlyOffInit(P);
              P->StateMachine =PrtctRly_INIT;
          break;
          default :
          break;

      }
}
void ProtectRlyOnInit(PrtectRelayReg *P)
{
    P->WakeupOn_ProRlyOnCount=0;
    P->WakeupOn_PRlyOnCount=0;
    P->WakeupOn_ProRlyOffCount=0;
    P->State.bit.WakeuPOffEND=0;
   // P->State.bit.WakeuPOnEND=0;
    P->WakeupOff_TimeCount=0;
}
void ProtectRlyOnHandle(PrtectRelayReg *P)
{
    P->State.bit.NRlyDO=1;
    if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==0))
    {
         P->WakeupOn_ProRlyOnCount++;
         if(P->WakeupOn_ProRlyOnCount>=WakeUpOnProRlyOnTime)
         {
             P->State.bit.PreRlyDO=1;
             P->State.bit.ProRlyDI=1;
             P->WakeupOn_ProRlyOnCount= WakeUpOnProRlyOnTime+10;
         }
    }
    if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==1)&&(P->State.bit.PRlyDI==0))
    {
        P->WakeupOn_PRlyOnCount++;
        if(P->WakeupOn_PRlyOnCount>=WakeUpOnPRlayOnTime)
        {
            P->State.bit.PRlyDO=1;
            P->WakeupOn_PRlyOnCount=WakeUpOnPRlayOnTime+10;
        }
    }
    if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==1)&&(P->State.bit.PRlyDI==1))
    {
        P->WakeupOn_ProRlyOffCount++;
        if(P->WakeupOn_ProRlyOffCount>=WakeUpOnProRlyOffTime)
        {
            P->State.bit.PreRlyDO=0;
            P->State.bit.ProRlyDI=0;
            P->WakeupOn_ProRlyOffCount=WakeUpOnProRlyOffTime+10;
        }
    }
    if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==1))
    {
        P->State.bit.NRlyDO=1;
        P->State.bit.PRlyDO=1;
        P->State.bit.WakeuPOnEND=1;
    }
}
void ProtectRlyOffInit(PrtectRelayReg *P)
{
    P->WakeupOff_NRlyOffCount=0;
    P->State.bit.WakeuPOnEND=0;
    P->WakeupOn_TimeCount=0;
    P->WakeupOff_TimeCount=0;
}
void ProtectRlyOffHandle(PrtectRelayReg *P)
{
    P->State.bit.PRlyDO=0;
    if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0))
    {
         P->WakeupOff_NRlyOffCount++;
         if(P->WakeupOff_NRlyOffCount>=WakeUpOFFTNRelayOFFTime)
         {
             P->State.bit.NRlyDO=0;
             P->State.bit.PreRlyDO=0;
             P->State.bit.ProRlyDI=0;
             P->WakeupOff_NRlyOffCount= WakeUpOFFTNRelayOFFTime+10;
             P->State.bit.WakeuPOffEND=1;
         }
    }
}
void ProtectRlyEMSHandle(PrtectRelayReg *P)
{
    P->State.bit.PRlyDO=0;
    P->State.bit.ProRlyDI=1;
    if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0))
    {
         P->Protect_NRlyOffCount++;
         if(P->Protect_NRlyOffCount>=ProtectOFFTNRelayOFFTime)
         {
             P->State.bit.NRlyDO=0;
             P->State.bit.PreRlyDO=0;
             P->State.bit.PRlyDO=0;
             P->WakeupOff_NRlyOffCount= WakeUpOFFTNRelayOFFTime+10;
         }
    }
}


