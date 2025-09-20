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

//#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
//#include "F2806x_Device.h"      // F2806x Headerfile Include File
//#include "F2806x_Examples.h"    // F2806x Examples Include File
//#include "DSP28x_Project.h"

#ifndef PROTECTRELAY_H


#define PROTECTRELAY_H

#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"

/*
 *
 */
#define WakeUpOnProRlyOnTime             500  // 50msec
#define WakeUpOnPRlayOnTime              200  // 50msec
#define WakeUpOnProRlyOffTime            500 //50msec, 초충 시간 관리
#define WakeUpONTimeOut                  5000

/*
 *
 */
#define WakeUpOFFTNRelayOFFTime           500  //
#define WakeUpOFFTimeOut                  1000
/*
 *
 */
#define ProtectOFFTNRelayOFFTime           50  //
#define ProtectWakeUpOFFTimeOut            1000

/*
#define ProRelayOnTime          50  // 50msec
#define PRelayOnTime            50  // 50msec
#define ProRelayOFFTime         250 // 250msec
#define WakeUpOnTimeOut         350*4 // 250msec
8
#define PRelayOFFTime           50  //
#define WakeUpOFFTimeOutCount   50*2
*/
typedef enum
{
    PrtctRly_INIT,
    PrtctRly_STANDBY,
    PrtctRly_Ready,
    PrtctRly_RuningON,
    PrtctRly_RuningOFF,
    PrtctRly_ProtectpOFF,
    PrtctRly_RLYProtect,
    PrtctRly_CLEAR
}ProtectRelayState;

struct ProtectRelayState_BIT
{       // bits   description
    unsigned int     WakeUpEN                   :1; // 0
    unsigned int     PRlyDI                     :1; // 1
    unsigned int     NRlyDI                     :1; // 2
    unsigned int     ProRlyDI                   :1; // 3
    unsigned int     PRlyDO                     :1; // 4
    unsigned int     NRlyDO                     :1; // 5
    unsigned int     PreRlyDO                   :1; // 6
    unsigned int     LatchRlyOn                 :1; // 7
    unsigned int     LatchRlyOFF                :1; // 8
    unsigned int     LatchRlyDI                 :1; // 9
    unsigned int     WakeuPOnEND                :1; // 10
    unsigned int     WakeuPOffEND               :1; // 11
    unsigned int     WakeupOnTiemrErr           :1; // 12
    unsigned int     WakeupOFFTiemrErr          :1; // 13
    unsigned int     FaultSeqErr                :1; // 14
    unsigned int     RlyFaulttSate              :1; // 15
};
union ProtectRelaySate_REG
{
   unsigned int     all;
   struct ProtectRelayState_BIT bit;
};

typedef struct PrtectRelay_Date
{
    /*
     *
     */
    ProtectRelayState StateMachine;
    Uint16 WakeupOn_ProRlyOnCount;
    Uint16 WakeupOn_ProRlyOffCount;
    Uint16 WakeupOn_PRlyOnCount;
    Uint16 WakeupOn_NRlyOnCount;
    Uint16 WakeupOn_TimeCount;

    Uint16 WakeupOff_PRlyOffCount;
    Uint16 WakeupOff_NRlyOffCount;
    Uint16 WakeupOff_TimeCount;

    Uint16 Protect_ProRlyOnCount;
    Uint16 Protect_ProRlyOffCount;
    Uint16 Protect_PRlyOffCount;
    Uint16 Protect_NRlyOffCount;
    union  ProtectRelaySate_REG    State;


}PrtectRelayReg;



extern void ProtectRelaySateCheck(PrtectRelayReg *P);
extern void ProtectRelayVarINIT(PrtectRelayReg *P);
extern void ProtectOffHandle(PrtectRelayReg *P);
extern void ProtectRelayWakeUpHandle(PrtectRelayReg *P);

#endif  // end of PARAMETER.H definition


//===========================================================================
// No more.
//===========================================================================
