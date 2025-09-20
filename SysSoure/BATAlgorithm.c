#include "DSP28x_Project.h"
#include "BATAlgorithm.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#if FarasisP52Ah
extern void CalFarasis52AhRegsInit(SocReg *P);
extern void CalFarasis52AhSocInit(SocReg *P);
extern void CalFarasis52AhSocHandle(SocReg *P);
#define C_Farasis52Ah_SOCX2     -138.71
#define C_Farasis52Ah_SOCX1     1195.70
#define C_Farasis52Ah_SOCX0    -2472.30
#define C_FarasisP52AhNorm     0.004//  0.0238//1/42Ah
#endif
#if Frey60Ah
extern void CalFrey60AhRegsInit(SocReg *P);
extern void CalFrey60AhSocInit(SocReg *P);
extern void CalFrey60AhSocHandle(SocReg *P);

#define C_Frey60Ah_SOCX0A    3352.4701
#define C_Frey60Ah_SOCX1A    -20858.4935
#define C_Frey60Ah_SOCX2A    32422.54


#define C_Frey60Ah_SOCX2B    21.8172
#define C_Frey60Ah_SOCX1B    1.8087
#define C_Frey60Ah_SOCX0B    -216.32

#define C_Frey60Ah_SOCX2C    1.4655
#define C_Frey60Ah_SOCX1C    4.8170
#define C_Frey60Ah_SOCX0C    15.83


#define C_Frey60Ah_SOCX2D    -3344.4816
#define C_Frey60Ah_SOCX1D    22607.0234
#define C_Frey60Ah_SOCX0D   - 38111.77

#define C_Frey60Ah_SOCX2E    0
#define C_Frey60Ah_SOCX1E    1499.9998
#define C_Frey60Ah_SOCX0E    -4912.50


//#define C_Frey60AhNorm       0.002631//1/380Ah;(0.0208//1/48Ah)
#define C_EVE380AhNorm       0.002631//1/380Ah;(0.0208//1/48Ah)

#endif

#if Kokam100Ah
void CalKokam100AhRegs(void)
{

}
void CalKokam100AhSocInit(void)
{

}

void Calkokam100AhSocHandle(void)
{

}
#endif

#if Kokam60Ah
void CalKokam60AhRegs(void)
{

}
void CalKokam60AhSocInit(void)
{

}

void Calkokam60AhSocHandle(void)
{

}

#endif



#if FarasisP52Ah
void CalFarasis52AhRegsInit(SocReg *P)
{
    P->SysSOCdtF=0.0;
    P->SysSoCCTF=0.0;
    P->SysPackAhNewF=0.0;
    P->SysPackAhOldF=0.0;
    P->SysPackAhF=0.0;
    P->SysPackSOCBufF1=0.0;
    P->SysPackSOCBufF2=0.0;
    P->SysPackSOCF=5.0;
    P->AVGXF=0.0;
    P->SOCX4InF=0.0;
    P->SOCX3InF=0.0;
    P->SOCX2InF=0.0;
    P->SOCX1InF=0.0;
    P->SOCX4OutF=0.0;
    P->SOCX3OutF=0.0;
    P->SOCX2OutF=0.0;
    P->SOCX1OutF=0.0;
    P->SOCbufF=0.0;
    P->SysSocInitF=0.0;
    P->CellAgvVoltageF=0.0;
    P->SoCStateRegs.all=0;
    P->CTCount=0;
    P->SysTime=0;
    P->SysSoCCTAbsF=0;
    P->state=SOC_STATE_IDLE;
}
void CalFarasis52AhSocInit(SocReg *P)
{

    // 60Ah
     P->AVGXF         =   P->CellAgvVoltageF;
     P->SOCX2InF      =   P->AVGXF  * P->AVGXF;
     P->SOCX1InF      =   P->AVGXF;
     P->SOCX2OutF     =   C_Farasis52Ah_SOCX2 * P->SOCX2InF;
     P->SOCX1OutF     =   C_Farasis52Ah_SOCX1 * P->SOCX1InF;
     P->SOCbufF       =   P->SOCX2OutF + P->SOCX1OutF + C_Farasis52Ah_SOCX0;
     /*
      *  보관법 계산식 필요함
      */
     if((P->SOCbufF >= 0.0)&&(P->SOCbufF < 20.0))
     {
         P->SOCbufF  =   P->SOCbufF-2;
     }
     if((P->SOCbufF >= 20)&&(P->SOCbufF < 40.0))
     {
         P->SOCbufF  =   P->SOCbufF+3;
     }
     if((P->SOCbufF >= 40)&&(P->SOCbufF < 80.0))
     {
         P->SOCbufF  =   P->SOCbufF+3.0;
     }
     if(P->SOCbufF >= 80.0)
     {
         P->SOCbufF  =   P->SOCbufF-3;
     }
     /*
      *
      */
     if(P->SOCbufF <=0.0)
     {
         P->SOCbufF = 0.0;
     }
     else if(P->SOCbufF > 98.0)
     {
         P->SOCbufF = 100.0;
     }
     P->SysSocInitF = P->SOCbufF;
}

//CalSocKokamInit(&KokamSocRegs);
//KokamSocRegs.state = SOC_STATE_RUNNING;
void CalFarasis52AhSocHandle(SocReg *P)
{
    P->SysTime++;
    if(P->SysTime>=C_SocSamPleCount)
    {
        if(P->SysSoCCTAbsF>=C_SocInitCTVaule)
        {
            P->SoCStateRegs.bit.CalMeth=1;
            P->CTCount=0;
        }
        else
        {
            P->CTCount++;
            if(P->CTCount>6000)
            {
                P->CTCount=6001;
                P->SoCStateRegs.bit.CalMeth=0;
            }
        }
        switch (P->state)
        {

            case SOC_STATE_RUNNING:
                 if(P->SoCStateRegs.bit.CalMeth==0)
                 {
                     /*
                      *
                      */
                     //52Ah

                     P->AVGXF         =   P->CellAgvVoltageF;
                     P->SOCX2InF      =   P->AVGXF  * P->AVGXF;
                     P->SOCX1InF      =   P->AVGXF;
                     P->SOCX2OutF     =   C_Farasis52Ah_SOCX2 * P->SOCX2InF;
                     P->SOCX1OutF     =   C_Farasis52Ah_SOCX1 * P->SOCX1InF;
                     P->SOCbufF       =   P->SOCX2OutF + P->SOCX1OutF + C_Farasis52Ah_SOCX0;
                     P->SOCbufF       =   P->SOCbufF+3.0;
                     /*
                      *  보관법 계산식 필요함
                      */
                     if((P->SOCbufF >= 0.0)&&(P->SOCbufF < 20.0))
                     {
                         P->SOCbufF  =   P->SOCbufF-2;
                     }
                     if((P->SOCbufF >= 20)&&(P->SOCbufF < 40.0))
                     {
                         P->SOCbufF  =   P->SOCbufF+3;
                     }
                     if((P->SOCbufF >= 40)&&(P->SOCbufF < 80.0))
                     {
                         P->SOCbufF  =   P->SOCbufF+3.0;
                     }
                     if(P->SOCbufF >= 80.0)
                     {
                         P->SOCbufF  =   P->SOCbufF-3;
                     }
                     /*
                      *
                      */
                     if(P->SOCbufF <=0.0)
                     {
                         P->SOCbufF = 0.0;
                     }
                     else if(P->SOCbufF > 98.0)
                     {
                         P->SOCbufF = 100.0;
                     }
                      if(P->SOCbufF <=0.0)
                      {
                          P->SOCbufF = 0.0;
                      }
                      else if(P->SOCbufF > 98.0)
                      {
                          P->SOCbufF = 100.0;
                      }
                      P->SysSocInitF = P->SOCbufF;
                      P->SysPackSOCF = P->SOCbufF;
                 }
                 if(P->SoCStateRegs.bit.CalMeth==1)
                 {
                     /*
                      *
                      */
                     P->SysSOCdtF = C_CTSampleTime*C_SocCumulativeTime; // CumulativeTime(1/3600) -> 누적시간
                     P->SysPackAhNewF = P->SysSoCCTF * P->SysSOCdtF;
                     P->SysPackAhF    = P->SysPackAhNewF + P->SysPackAhOldF;
                     P->SysPackAhOldF = P->SysPackAhF;
                     if(P->SysPackAhF <= -250.0)
                     {
                        P->SysPackAhF =-250.0;
                     }
                     if(P->SysPackAhF> 250.0)
                     {
                         P->SysPackAhF= 250.0;
                     }
                     /*
                     * SOC 변환
                     */
                     P->SysPackSOCBufF1 = P->SysPackAhF *C_FarasisP52AhNorm;//0.0125 ;// 1/80 --> 0.0125--> 일반화
                     P->SysPackSOCBufF2 = P->SysPackSOCBufF1*100.0; //--> 단위 변환 %
                     P->SysPackSOCF     = P->SysSocInitF+P->SysPackSOCBufF2;
                 }
                 P->state = SOC_STATE_Save;

            break;
            case SOC_STATE_Save:

                P->state = SOC_STATE_RUNNING;

            break;
            case SOC_STATE_CLEAR:

            break;
        }
        P->SysTime=0;
    }
}
#endif

#if Frey60Ah
void CalFrey60AhRegsInit(SocReg *P)
{
    P->SysSOCdtF=0.0;
    P->SysSoCCTF=0.0;
    P->SysPackAhNewF=0.0;
    P->SysPackAhOldF=0.0;
    P->SysPackAhF=0.0;
    P->SysPackSOCBufF1=0.0;
    P->SysPackSOCBufF2=0.0;
    P->SysPackSOCF=5.0;
    P->AVGXF=0.0;
    P->SOCX2InF=0.0;
    P->SOCX1InF=0.0;
    P->SOCX3InF=0.0;
    P->SOCX4InF=0.0;
    P->SOCX2OutF=0.0;
    P->SOCX1OutF=0.0;
    P->SOCX3OutF=0.0;
    P->SOCX4OutF=0.0;

    P->SOCX4InFAZore=0.0;
    P->SOCX3InFAZore=0.0;
    P->SOCX2InFAZore=0.0;
    P->SOCX1InFAZore=0.0;
    P->SOCX4OutFAZore=0.0;
    P->SOCX3OutFAZore=0.0;
    P->SOCX2OutFAZore=0.0;
    P->SOCX1OutFAZore=0.0;
    P->AZoreCalCout=0;



    P->SOCX4InFBZore=0.0;
    P->SOCX3InFBZore=0.0;
    P->SOCX2InFBZore=0.0;
    P->SOCX1InFBZore=0.0;
    P->SOCX4OutFBZore=0.0;
    P->SOCX3OutFBZore=0.0;
    P->SOCX2OutFBZore=0.0;
    P->SOCX1OutFBZore=0.0;
    P->BZoreCalCout=0.0;


    P->SOCX4InFCZore=0.0;
    P->SOCX3InFCZore=0.0;
    P->SOCX2InFCZore=0.0;
    P->SOCX1InFCZore=0.0;
    P->SOCX4OutFCZore=0.0;
    P->SOCX3OutFCZore=0.0;
    P->SOCX2OutFCZore=0.0;
    P->SOCX1OutFCZore=0.0;
    P->CZoreCalCout=0;

    P->SOCX4InFDZore=0.0;
    P->SOCX3InFDZore=0.0;
    P->SOCX2InFDZore=0.0;
    P->SOCX1InFDZore=0.0;
    P->SOCX4OutFDZore=0.0;
    P->SOCX3OutFDZore=0.0;
    P->SOCX2OutFDZore=0.0;
    P->SOCX1OutFDZore=0.0;
    P->DZoreCalCout=0;


    P->SOCX4InFEZore=0.0;
    P->SOCX3InFEZore=0.0;
    P->SOCX2InFEZore=0.0;
    P->SOCX1InFEZore=0.0;
    P->SOCX4OutFEZore=0.0;
    P->SOCX3OutFEZore=0.0;
    P->SOCX2OutFEZore=0.0;
    P->SOCX1OutFEZore=0.0;
    P->EZoreCalCout=0;




    P->SOCbufF=0.0;
    P->SysSocInitF=0.0;
    P->CellAgvVoltageF=0.0;
    P->SoCStateRegs.all=0;
    P->CTCount=0;
    P->SysTime=0;
    P->SysSoCCTAbsF=0;
    P->state=SOC_STATE_IDLE;
}
void CalFrey60AhSocInit(SocReg *P)
{

    /*
     * #define C_Frey60Ah_SOCX0A    3352.4701
       #define C_Frey60Ah_SOCX1A    -20858.4935
       #define C_Frey60Ah_SOCX2A    32422.54


       #define C_Frey60Ah_SOCX2B    12365.5914
       #define C_Frey60Ah_SOCX1B    -80393.0108
       #define C_Frey60Ah_SOCX0B    130685.77

       #define C_Frey60Ah_SOCX2C    1.4655
       #define C_Frey60Ah_SOCX1C    4.8170
       #define C_Frey60Ah_SOCX0C    15.83


       #define C_Frey60Ah_SOCX2D    -3344.4816
       #define C_Frey60Ah_SOCX1D    22607.0234
       #define C_Frey60Ah_SOCX0D   - 38111.77

       #define C_Frey60Ah_SOCX2E    0
       #define C_Frey60Ah_SOCX1E    1499.9998
       #define C_Frey60Ah_SOCX0E    -4912.50
     */


    // 60Ah
      P->AVGXF         =   P->CellAgvVoltageF;
         //IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_A_BOT, LFP_VOLT_A_TOP)  ((A) >  (MIN) && (A) <= (MAX))  // 초과 ~ 이하
      if(IS_ABOVE_AND_UNDER(P->AVGXF , LFP_VOLT_A_BOT, LFP_VOLT_A_TOP))
      {
          if(P->AVGXF<=3.03)
          {
              P->SOCbufF =0.0;
          }
          if(IS_ABOVE_AND_UNDER(P->AVGXF ,3.03, 3.2))
          {
              P->SOCbufF =5.0;
          }
          if(IS_ABOVE_AND_UNDER(P->AVGXF ,3.2, 3.21))
          {
              P->SOCbufF =10.0;
          }
          if(IS_ABOVE_AND_UNDER(P->AVGXF ,3.21, 3.215))
          {
              P->SOCbufF =15.0;
          }
          //#define LFP_VOLT_A_BOT   3.030
          //#define LFP_VOLT_A_TOP   3.215
          //#define C_Frey60Ah_SOCX0A    3352.4701
          //#define C_Frey60Ah_SOCX1A    -20858.4935
          //#define C_Frey60Ah_SOCX2A    32422.54
      /*    P->AZoreCalCout++;
          P->SOCX4InFAZore = 0;
          P->SOCX3InFAZore = 0;
          P->SOCX2InFAZore = P->AVGXF*P->AVGXF;
          P->SOCX1InFAZore = P->AVGXF;

          P->SOCX4OutFAZore = 0;
          P->SOCX3OutFAZore = 0;
          P->SOCX2OutFAZore = C_Frey60Ah_SOCX2A* P->SOCX2InFAZore;
          P->SOCX1OutFAZore = C_Frey60Ah_SOCX1A* P->SOCX1InFAZore;
          P->SOCbufF        = P->SOCX2OutFAZore+P->SOCX1OutFAZore+ C_Frey60Ah_SOCX0A;*/
          if(P->AZoreCalCout>3600)
          {
              P->AZoreCalCout=0;
          }
      }
     // IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_B_BOT, LFP_VOLT_B_TOP)  ((A) >  (MIN) && (A) <= (MAX))  // 초과 ~ 이하
      if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_B_BOT, LFP_VOLT_B_TOP))
      {
          //#define C_Frey60Ah_SOCX2B    2566.7
          //#define C_Frey60Ah_SOCX1B    -16384
          //#define C_Frey60Ah_SOCX0B     26156
          P->BZoreCalCout++;
          P->SOCX4InFBZore = 0;
          P->SOCX3InFBZore = 0;

          P->SOCX2InFBZore = P->AVGXF*P->AVGXF;
          P->SOCX1InFBZore = P->AVGXF;

          P->SOCX4OutFBZore = 0;
          P->SOCX3OutFBZore = 0;
          P->SOCX2OutFBZore = C_Frey60Ah_SOCX2B*P->SOCX2InFBZore;
          P->SOCX1OutFBZore = C_Frey60Ah_SOCX1B*P->SOCX1InFBZore;
          P->SOCbufF        = P->SOCX2OutFBZore + P->SOCX1OutFBZore+C_Frey60Ah_SOCX0B;
          if(P->BZoreCalCout>3600)
           {
               P->BZoreCalCout=0;
           }
      }

      if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_C_BOT, LFP_VOLT_C_TOP))
      {

          P->CZoreCalCout++;
          //#define C_Frey60Ah_SOCX1C    1313.1
          //#define C_Frey60Ah_SOCX0C   -4276.6
          P->SOCX4InFCZore = 0;
          P->SOCX3InFCZore = 0;
          P->SOCX2InFCZore = P->AVGXF*P->AVGXF;
          P->SOCX1InFCZore = P->AVGXF;

          P->SOCX4OutFCZore = 0;
          P->SOCX3OutFCZore = 0;
          P->SOCX2OutFCZore = C_Frey60Ah_SOCX2C*P->SOCX2InFCZore;
          P->SOCX1OutFCZore = C_Frey60Ah_SOCX1C*P->SOCX1InFCZore;
          P->SOCbufF        = P->SOCX2OutFCZore+P->SOCX1OutFCZore + C_Frey60Ah_SOCX0C;

          if(P->CZoreCalCout>3600)
           {
               P->CZoreCalCout=0;
           }
      }
      if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_D_BOT, LFP_VOLT_D_TOP))
      {
          P->DZoreCalCout++;
          P->SOCX4InFDZore = 0;
          P->SOCX3InFDZore = 0;
          P->SOCX2InFDZore = P->AVGXF*P->AVGXF;
          P->SOCX1InFDZore = P->AVGXF;

          P->SOCX4OutFDZore = 0;
          P->SOCX3OutFDZore = 0;
          P->SOCX2OutFDZore = C_Frey60Ah_SOCX2D*P->SOCX2InFDZore;
          P->SOCX1OutFDZore = C_Frey60Ah_SOCX1D*P->SOCX1InFDZore;
          P->SOCbufF        = P->SOCX2OutFDZore+P->SOCX1OutFDZore + C_Frey60Ah_SOCX0D;
          if(P->DZoreCalCout>3600)
           {
               P->DZoreCalCout=0;
           }
      }
      if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_E_BOT, LFP_VOLT_E_TOP))
      {
          P->EZoreCalCout++;
          P->SOCX4InFEZore = 0;
          P->SOCX3InFEZore = 0;
          P->SOCX2InFEZore = 0;
          P->SOCX1InFEZore = P->AVGXF;

          P->SOCX4OutFEZore = 0;
          P->SOCX3OutFEZore = 0;
          P->SOCX2OutFEZore = 0;
          P->SOCX1OutFEZore = C_Frey60Ah_SOCX1E*P->SOCX1InFEZore;
          P->SOCbufF        = P->SOCX1OutFEZore + C_Frey60Ah_SOCX0E;
          if(P->DZoreCalCout>3600)
           {
               P->DZoreCalCout=0;
           }
      }
     if(P->SOCbufF <=0.0)
     {
         P->SOCbufF = 0.0;
     }
     else if(P->SOCbufF > 90.0)
     {
         P->SOCbufF = 100.0;
     }
     P->SysSocInitF = P->SOCbufF;
}

void CalFrey60AhSocHandle(SocReg *P)
{
    P->SysTime++;
    P->AVGXF         =   P->CellAgvVoltageF;
    if(P->SysTime>=C_SocSamPleCount)
     {
         if(P->SysSoCCTAbsF>=C_SocInitCTVaule)
         {
             P->SoCStateRegs.bit.CalMeth=1;
             P->CTCount=0;
         }
         else
         {
             P->CTCount++;
             if(P->CTCount>6000)
             {
                 P->CTCount=6001;
                 P->SoCStateRegs.bit.CalMeth=0;
             }
         }
         switch (P->state)
         {

             case SOC_STATE_RUNNING:
                  if(P->SoCStateRegs.bit.CalMeth==0)
                  {
                      // 60Ah
                      P->AVGXF         =   P->CellAgvVoltageF;
                          //IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_A_BOT, LFP_VOLT_A_TOP)  ((A) >  (MIN) && (A) <= (MAX))  // 초과 ~ 이하
                       if(IS_ABOVE_AND_UNDER(P->AVGXF , LFP_VOLT_A_BOT, LFP_VOLT_A_TOP))
                       {
                           if(P->AVGXF<=3.03)
                           {
                               P->SOCbufF =0.0;
                           }
                           if(IS_ABOVE_AND_UNDER(P->AVGXF ,3.03, 3.2))
                           {
                               P->SOCbufF =5.0;
                           }
                           if(IS_ABOVE_AND_UNDER(P->AVGXF ,3.2, 3.21))
                           {
                               P->SOCbufF =10.0;
                           }
                           if(IS_ABOVE_AND_UNDER(P->AVGXF ,3.21, 3.215))
                           {
                               P->SOCbufF =15.0;
                           }
                           //#define LFP_VOLT_A_BOT   3.030
                           //#define LFP_VOLT_A_TOP   3.215
                           //#define C_Frey60Ah_SOCX0A    3352.4701
                           //#define C_Frey60Ah_SOCX1A    -20858.4935
                           //#define C_Frey60Ah_SOCX2A    32422.54
                       /*    P->AZoreCalCout++;
                           P->SOCX4InFAZore = 0;
                           P->SOCX3InFAZore = 0;
                           P->SOCX2InFAZore = P->AVGXF*P->AVGXF;
                           P->SOCX1InFAZore = P->AVGXF;

                           P->SOCX4OutFAZore = 0;
                           P->SOCX3OutFAZore = 0;
                           P->SOCX2OutFAZore = C_Frey60Ah_SOCX2A* P->SOCX2InFAZore;
                           P->SOCX1OutFAZore = C_Frey60Ah_SOCX1A* P->SOCX1InFAZore;
                           P->SOCbufF        = P->SOCX2OutFAZore+P->SOCX1OutFAZore+ C_Frey60Ah_SOCX0A;*/
                           if(P->AZoreCalCout>3600)
                           {
                               P->AZoreCalCout=0;
                           }
                       }
                      // IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_B_BOT, LFP_VOLT_B_TOP)  ((A) >  (MIN) && (A) <= (MAX))  // 초과 ~ 이하
                       if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_B_BOT, LFP_VOLT_B_TOP))
                       {
                           //#define C_Frey60Ah_SOCX2B    2566.7
                           //#define C_Frey60Ah_SOCX1B    -16384
                           //#define C_Frey60Ah_SOCX0B     26156
                           P->BZoreCalCout++;
                           P->SOCX4InFBZore = 0;
                           P->SOCX3InFBZore = 0;

                           P->SOCX2InFBZore = P->AVGXF*P->AVGXF;
                           P->SOCX1InFBZore = P->AVGXF;

                           P->SOCX4OutFBZore = 0;
                           P->SOCX3OutFBZore = 0;
                           P->SOCX2OutFBZore = C_Frey60Ah_SOCX2B*P->SOCX2InFBZore;
                           P->SOCX1OutFBZore = C_Frey60Ah_SOCX1B*P->SOCX1InFBZore;
                           P->SOCbufF        = P->SOCX2OutFBZore + P->SOCX1OutFBZore+C_Frey60Ah_SOCX0B;
                           if(P->BZoreCalCout>3600)
                            {
                                P->BZoreCalCout=0;
                            }
                       }

                       if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_C_BOT, LFP_VOLT_C_TOP))
                       {

                           P->CZoreCalCout++;
                           //#define C_Frey60Ah_SOCX1C    1313.1
                           //#define C_Frey60Ah_SOCX0C   -4276.6
                           P->SOCX4InFCZore = 0;
                           P->SOCX3InFCZore = 0;
                           P->SOCX2InFCZore = P->AVGXF*P->AVGXF;
                           P->SOCX1InFCZore = P->AVGXF;

                           P->SOCX4OutFCZore = 0;
                           P->SOCX3OutFCZore = 0;
                           P->SOCX2OutFCZore = C_Frey60Ah_SOCX2C*P->SOCX2InFCZore;
                           P->SOCX1OutFCZore = C_Frey60Ah_SOCX1C*P->SOCX1InFCZore;
                           P->SOCbufF        = P->SOCX2OutFCZore+P->SOCX1OutFCZore + C_Frey60Ah_SOCX0C;

                           if(P->CZoreCalCout>3600)
                            {
                                P->CZoreCalCout=0;
                            }
                       }
                       if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_D_BOT, LFP_VOLT_D_TOP))
                       {
                           P->DZoreCalCout++;
                           P->SOCX4InFDZore = 0;
                           P->SOCX3InFDZore = 0;
                           P->SOCX2InFDZore = P->AVGXF*P->AVGXF;
                           P->SOCX1InFDZore = P->AVGXF;

                           P->SOCX4OutFDZore = 0;
                           P->SOCX3OutFDZore = 0;
                           P->SOCX2OutFDZore = C_Frey60Ah_SOCX2D*P->SOCX2InFDZore;
                           P->SOCX1OutFDZore = C_Frey60Ah_SOCX1D*P->SOCX1InFDZore;
                           P->SOCbufF        = P->SOCX2OutFDZore+P->SOCX1OutFDZore + C_Frey60Ah_SOCX0D;
                           if(P->DZoreCalCout>3600)
                            {
                                P->DZoreCalCout=0;
                            }
                       }
                       if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_E_BOT, LFP_VOLT_E_TOP))
                       {
                           P->EZoreCalCout++;
                           P->SOCX4InFEZore = 0;
                           P->SOCX3InFEZore = 0;
                           P->SOCX2InFEZore = 0;
                           P->SOCX1InFEZore = P->AVGXF;

                           P->SOCX4OutFEZore = 0;
                           P->SOCX3OutFEZore = 0;
                           P->SOCX2OutFEZore = 0;
                           P->SOCX1OutFEZore = C_Frey60Ah_SOCX1E*P->SOCX1InFEZore;
                           P->SOCbufF        = P->SOCX1OutFEZore + C_Frey60Ah_SOCX0E;
                           if(P->DZoreCalCout>3600)
                            {
                                P->DZoreCalCout=0;
                            }
                       }
                       if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_F_BOT, LFP_VOLT_F_TOP))
                       {
                           P->FZoreCalCout++;
                           P->SOCbufF        = 92.0;
                           if(P->FZoreCalCout>3600)
                            {
                                P->FZoreCalCout=0;
                            }
                       }
                       if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_G_BOT, LFP_VOLT_G_TOP))
                       {
                           P->GZoreCalCout++;
                           P->SOCbufF        = 95.0;
                           if(P->GZoreCalCout>3600)
                            {
                                P->GZoreCalCout=0;
                            }
                       }
                       if(IS_ABOVE_AND_UNDER(P->AVGXF, LFP_VOLT_H_BOT, LFP_VOLT_H_TOP))
                       {
                           P->HZoreCalCout++;
                           P->SOCbufF        = 98.0;
                           if(P->HZoreCalCout>3600)
                            {
                                P->HZoreCalCout=0;
                            }
                       }
                       P->SysSocInitF = P->SOCbufF;
                      // P->SysPackSOCF = P->SOCbufF;
                  }
                  if(P->SoCStateRegs.bit.CalMeth==1)
                  {
                      /*
                       *
                       */
                      P->SysSOCdtF = C_CTSampleTime*C_SocCumulativeTime; // CumulativeTime(1/3600) -> 누적시간
                      P->SysPackAhNewF = P->SysSoCCTF * P->SysSOCdtF;
                      P->SysPackAhF    = P->SysPackAhNewF + P->SysPackAhOldF;
                      P->SysPackAhOldF = P->SysPackAhF;
                      if(P->SysPackAhF <= -380.0)
                      {
                         P->SysPackAhF =-380.0;
                      }
                      if(P->SysPackAhF> 380.0)
                      {
                          P->SysPackAhF= 380.0;
                      }
                      /*
                      * SOC 변환
                      */
                      P->SysPackSOCBufF1 = P->SysPackAhF *C_EVE380AhNorm;// 1/48(0.0208)
                      P->SysPackSOCBufF2 = P->SysPackSOCBufF1*100.0; //--> 단위 변환 %
                      P->SysPackSOCF     = P->SysSocInitF+P->SysPackSOCBufF2;
                  }
                  P->state = SOC_STATE_Save;

             break;
             case SOC_STATE_Save:

                 P->state = SOC_STATE_RUNNING;

             break;
             case SOC_STATE_CLEAR:

             break;
         }
         P->SysTime=0;
     }
}

#endif

#if EVE24060Ah
void CalEVE60AhRegsInit(SocReg *P)
{
    P->SysSOCdtF=0.0;
    P->SysSoCCTF=0.0;
    P->SysPackAhNewF=0.0;
    P->SysPackAhOldF=0.0;
    P->SysPackAhF=0.0;
    P->SysPackSOCBufF1=0.0;
    P->SysPackSOCBufF2=0.0;
    P->SysPackSOCF=5.0;
    P->AVGXF=0.0;

    P->SOCX2InF=0.0;
    P->SOCX1InF=0.0;
    P->SOCX3InF=0.0;
    P->SOCX4InF=0.0;

    P->SOCX2OutF=0.0;
    P->SOCX1OutF=0.0;
    P->SOCX3OutF=0.0;
    P->SOCX4OutF=0.0;

    P->SOCX4InFAZore=0.0;
    P->SOCX3InFAZore=0.0;
    P->SOCX2InFAZore=0.0;
    P->SOCX1InFAZore=0.0;
    P->SOCX4OutFAZore=0.0;
    P->SOCX3OutFAZore=0.0;
    P->SOCX2OutFAZore=0.0;
    P->SOCX1OutFAZore=0.0;
    P->AZoreCalCout=0;



    P->SOCX4InFBZore=0.0;
    P->SOCX3InFBZore=0.0;
    P->SOCX2InFBZore=0.0;
    P->SOCX1InFBZore=0.0;
    P->SOCX4OutFBZore=0.0;
    P->SOCX3OutFBZore=0.0;
    P->SOCX2OutFBZore=0.0;
    P->SOCX1OutFBZore=0.0;
    P->BZoreCalCout=0.0;


    P->SOCX4InFCZore=0.0;
    P->SOCX3InFCZore=0.0;
    P->SOCX2InFCZore=0.0;
    P->SOCX1InFCZore=0.0;
    P->SOCX4OutFCZore=0.0;
    P->SOCX3OutFCZore=0.0;
    P->SOCX2OutFCZore=0.0;
    P->SOCX1OutFCZore=0.0;
    P->CZoreCalCout=0;

    P->SOCX4InFDZore=0.0;
    P->SOCX3InFDZore=0.0;
    P->SOCX2InFDZore=0.0;
    P->SOCX1InFDZore=0.0;
    P->SOCX4OutFDZore=0.0;
    P->SOCX3OutFDZore=0.0;
    P->SOCX2OutFDZore=0.0;
    P->SOCX1OutFDZore=0.0;
    P->DZoreCalCout=0;


    P->SOCX4InFEZore=0.0;
    P->SOCX3InFEZore=0.0;
    P->SOCX2InFEZore=0.0;
    P->SOCX1InFEZore=0.0;
    P->SOCX4OutFEZore=0.0;
    P->SOCX3OutFEZore=0.0;
    P->SOCX2OutFEZore=0.0;
    P->SOCX1OutFEZore=0.0;
    P->EZoreCalCout=0;

    P->SOCbufF=0.0;
    P->SysSocInitF=0.0;
    P->CellAgvVoltageF=0.0;
    P->SoCStateRegs.all=0;
    P->CTCount=0;
    P->SysTime=0;
    P->SysSoCCTAbsF=0;
    P->state=SOC_STATE_IDLE;

}
void CalEVE60AhSocInit(SocReg *P)
{
    P->AVGXF= CLAMP(P->CellAgvVoltageF, V_MIN, V_MAX);

    if (P->AVGXF < V_Soc20)  // [2.628, 3.245)
    {
        P->SOCX1OutFAZore = A1 * P->AVGXF + B1;
        P->SysSocInitF= P->SOCX1OutFAZore;
    }
    else if (P->AVGXF < V_Soc40)   // [3.245, 3.296)
    {
        P->SOCX1OutFBZore = A2 * P->AVGXF + B2;
        P->SysSocInitF= P->SOCX1OutFBZore;
    }
    else if (P->AVGXF <= V_Soc60) // [3.296, 3.3059]  <-- Hermite 3차
    {
        hermite_soc_40_60(P->AVGXF);
        P->SysSocInitF= P->SOCX1OutFCZore;
    }
    else if (P->AVGXF < V_Soc80) // (3.3059, 3.332)
    {
        P->SOCX1OutFDZore= A4 * P->AVGXF + B4;
        P->SysSocInitF= P->SOCX1OutFDZore;
    }
    else // [3.332, 3.440]//
    {
        P->SOCX1OutFEZore= A5 * P->AVGXF + B5;
        P->SysSocInitF= P->SOCX1OutFEZore;
    }
}
void hermite_soc_40_60(SocReg *P)
{
    // Hermite basis with position & slope matching at both ends
    float32 x0 = H_V0;
    float32 x1 = H_V1;
    float32 y0 = H_S0;
    float32 y1 = H_S1;
    float32 m0 = H_M0;
    float32 m1 = H_M1;
    float32 dx   =0.0;
    float32 bufA = 0.0;
    float32 bufB = 0.0;
    float32 bufC = 0.0;
    float32 h00  = 0.0;
    float32 h10=0.0;
    float32 h01=0.0;
    float32 h11=0.0;

    // 전압을 구간으로 클램프
    P->AVGXF = CLAMP(P->CellAgvVoltageF , x0, x1);
    dx = x1 - x0;             // ~0.0099 V
    bufA  = (P->AVGXF  - x0)/ dx; // 0..1

    // basis
    bufB = bufA  * bufA ;
    bufC = bufB  * bufA ;
    h00 = (1.0f + 2.0f* bufA) * (1.0f -  bufA) * (1.0f - bufA); // 2t^3 - 3t^2 + 1
    h10 = bufA * (1.0f - bufA) * (1.0f - bufA);                 // t^3 - 2t^2 + t
    h01 = bufB * (3.0f - 2.0f*bufA);                            // -2t^3 + 3t^2
    h11 = bufB * (bufA - 1.0f);                                 // t^3 - t^2

    P->SOCX1OutFCZore = h00*y0 + h10*dx*m0 + h01*y1 + h11*dx*m1;
}
void CalEVE60ASocHandle(SocReg *P)
{

}

#endif
