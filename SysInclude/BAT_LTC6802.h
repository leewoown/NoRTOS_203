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

#ifndef BAT_LTC6802_H
#define BAT_LTC6802_H

#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"
#include "parameter.h"


#define MAX_LTC6804                             16
#define LTC6804_CMD_WRCFG                       (0x0001)
#define LTC6804_CMD_RDCFG                       (0x0002)
#define LTC6804_CMD_RDCVA                       (0x0004)
#define LTC6804_CMD_RDCVB                       (0x0006)
#define LTC6804_CMD_RDCVC                       (0x0008)
#define LTC6804_CMD_RDCVD                       (0x000a)
#define LTC6804_CMD_RDAUXA                      (0x000c)
#define LTC6804_CMD_RDAUXB                      (0x000e)
#define LTC6804_CMD_RDSTATA                     (0x0010)
#define LTC6804_CMD_RDSTATB                     (0x0012)
#define LTC6804_CMD_ADCV                        (0x0260)
#define LTC6804_CMD_ADOW                        (0x0228)
#define LTC6804_CMD_CVST                        (0x0207)
#define LTC6804_CMD_ADAX                        (0x0460)
#define LTC6804_CMD_AXST                        (0x0407)
#define LTC6804_CMD_ADSTAT                      (0x0468)
#define LTC6804_CMD_STATST                      (0x040f)
#define LTC6804_CMD_ADCVAX                      (0x043f)
#define LTC6804_CMD_CLRCELL                     (0x0711)
#define LTC6804_CMD_CLRAUX                      (0x0712)
#define LTC6804_CMD_CLRSTAT                     (0x0713)
#define LTC6804_CMD_PLADC                       (0x0714)
#define LTC6804_CMD_DIAGN                       (0x0715)
#define LTC6804_CMD_WRCOMM                      (0x0721)
#define LTC6804_CMD_RDCOMM                      (0x0722)
#define LTC6804_CMD_STCOMM                      (0x0723)

#define LTC6804_ICOM_WR_START                   (6)
#define LTC6804_ICOM_WR_STOP                    (1)
#define LTC6804_ICOM_WR_BLANK                   (0)
#define LTC6804_ICOM_WR_NO_TRANSMIT             (7)

#define LTC6804_ICOM_RD_START                   (6)
#define LTC6804_ICOM_RD_STOP                    (1)
#define LTC6804_ICOM_RD_SDA_LOW                 (0)
#define LTC6804_ICOM_RD_SDA_HIGH                (7)

#define LTC6804_FCOM_WR_MASTER_ACK              (0)
#define LTC6804_FCOM_WR_MASTER_NACK             (8)
#define LTC6804_FCOM_WR_MASTER_NACK_STOP        (9)

#define LTC6804_FCOM_RD_MASTER_ACK              (0)
#define LTC6804_FCOM_RD_SLAVE_ACK               (7)
#define LTC6804_FCOM_RD_SLAVE_NACK              (15)
#define LTC6804_FCOM_RD_SLAVE_ACK_MASTER_STOP   (1)
#define LTC6804_FCOM_RD_SLAVE_NACK_MASTER_STOP  (9)

#define     BMS_ID_0                        15
#define     BMS_ID_1                        14
#define     BMS_ID_2                        13
#define     BMS_ID_3                        12
#define     BMS_ID_4                        11
#define     BMS_ID_5                        10
#define     BMS_ID_6                        9
#define     BMS_ID_7                        8
#define     BMS_ID_8                        7
#define     BMS_ID_9                        6
#define     BMS_ID_10                       5
#define     BMS_ID_11                       4
#define     BMS_ID_12                       3
#define     BMS_ID_13                       2
#define     BMS_ID_14                       1
#define     BMS_ID_15                       0

#define     BMS_ID_B0                       0xF
#define     BMS_ID_B1                       0xE
#define     BMS_ID_B2                       0xD
#define     BMS_ID_B3                       0xC
#define     BMS_ID_B4                       0xB
#define     BMS_ID_B5                       0xA
#define     BMS_ID_B6                       0x9
#define     BMS_ID_B7                       0X8
#define     BMS_ID_B8                       0x7
#define     BMS_ID_B9                       0x6
#define     BMS_ID_B10                      0x5
#define     BMS_ID_B11                      0x4
#define     BMS_ID_B12                      0x3
#define     BMS_ID_B13                      0x2
#define     BMS_ID_B14                      0x1
#define     BMS_ID_B15                      0x0

#define     C_TempsAGain                    -9.535
#define     C_TempsBGain                    46.665
#define     C_TempsCGain                    -110.33
#define     C_TempsDGain                    117.68
#define     C_Slave_ACh                     0
#define     C_Slave_BCh                     1
struct LTC6802_WORD
{
    unsigned int DataL;
    unsigned int DataH;
};
typedef enum
{
    STATE_BATIDLE,
    STATE_BATSTANDBY,
    STATE_BATREAD,
    STATE_BATBALANCE,
    STATE_BATCLEAR
}BatLTC6802State;
typedef struct _LTC6804
{
    int count;
    char address[MAX_LTC6804];
} LTC6804_t;
struct BatteryBalance_BIT
{       // bits   description
   unsigned int     B_Cell00        :1; // 0
   unsigned int     B_Cell01        :1; // 1
   unsigned int     B_Cell02        :1; // 2
   unsigned int     B_Cell03        :1; // 3
   unsigned int     B_Cell04        :1; // 4
   unsigned int     B_Cell05        :1; // 5
   unsigned int     B_Cell06        :1; // 6
   unsigned int     B_Cell07        :1; // 7
   unsigned int     B_Cell08        :1; // 8
   unsigned int     B_Cell09        :1; // 9
   unsigned int     B_Cell10        :1; // 10
   unsigned int     B_Cell11        :1; // 11
   unsigned int     B_Cell12        :1; // 12
   unsigned int     B_Cell13        :1; // 13
   unsigned int     B_Cell14        :1; // 14
   unsigned int     B_Cell15        :1;  // 15
};
union BatteryBalance_REG
{
   unsigned int     all;
   struct BatteryBalance_BIT    bit;
};
/*
 *
 * uint8_t MD The adc conversion mode
 * uint8_t DCP Controls if Discharge is permitted during cell conversions
 * uint8_t CH  Determines which cells are measured during an ADC conversion command
 * uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command
 * int LTC6804_write_cmd(char address, short command, char data[], int len)
 *
 *|command    |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
 *|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
 *|ADCV:      |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
 *|ADAX:      |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 */
struct LTCADCVCommand_BIT
{       // bits   description
   unsigned int     B_Cell00        :1; // 0
   unsigned int     B_Cell01        :1; // 1
   unsigned int     B_Cell02        :1; // 2
   unsigned int     B_Cell03        :1; // 3
   unsigned int     B_Cell04        :1; // 4
   unsigned int     B_Cell05        :1; // 5
   unsigned int     B_Cell06        :1; // 6
   unsigned int     B_Cell07        :1; // 7
   unsigned int     B_Cell08        :1; // 8
   unsigned int     B_Cell09        :1; // 9
   unsigned int     B_Cell10        :1; // 10
   unsigned int     B_Cell11        :1; // 11
   unsigned int     B_Cell12        :1; // 12
   unsigned int     B_Cell13        :1; // 13
   unsigned int     B_Cell14        :1; // 14
   unsigned int     B_Cell15        :1;  // 15
};
union LTCADCVCommand_REG
{
   unsigned int                 all;
   struct LTCADCVCommand_BIT    bit;
};
struct LTCGPIOReg_BIT
{       // bits   description
   unsigned char     ADCCOPT         :1; // 0 ,DATASHEET, TABLE 46
   unsigned char     SWTRD           :1; // 1 ,SWTEN(Software Timer Enable Pin) at Logic 1  DATASHEET, TABLE 46
   unsigned char     REFON           :1; // 2
   unsigned char     GPIO1           :1; // 3
   unsigned char     GPIO2           :1; // 4
   unsigned char     GPIO3           :1; // 5
   unsigned char     GPIO4           :1; // 6
   unsigned char     GPIO5           :1; // 7
};
union LTCGPIOReg_REG
{
   char                     all;
   struct LTCGPIOReg_BIT    bit;
};
/*
struct BMSCOMERR_BIT
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

union BMSCOMERR_REG
{
    Uint32                    all;
    struct LTC6802_WORD       Word;
    struct BMSCOMERR_BIT      bit;
};*/
typedef struct Slave_Date
{
    /*
     *
     */

    short  Pec1;
    char   ID;
    char   IDTest;
    char   BalanceTable[6];

    char   ADCX[6];
    char   ADCV[6];

    unsigned int     len;
    unsigned int     ErrorCount;
    unsigned int     Error;

    unsigned int     CellVoltage[12];
    unsigned int     CellVoltageBuf[12];
    float32          CellVoltageF[12];
    float32          DivVoltageF[12];
    float32          SysCellMinVoltage;


    unsigned int     CellTemperatureADC[12];
    unsigned int     CellTemperatureNor[12];
    float32          CellTemperatureVF[12];
    float32          CellTemperatureF[12];
    float32          CellTemperatureFBuf[12];
    int              CellTemperatureBuf[12];
    int              CellTemperature[12];


    BatLTC6802State StateMachine;
    union BatteryBalance_REG    Balance;
    union LTCGPIOReg_REG        BATICDO;
//    union BMSCOMERR_REG         ISOSPIErr;

    unsigned int TempsChSeletCount;
    unsigned int TempsChSelet;
    unsigned int GPIO0ADC;
    unsigned int GPIORef;
//    unsigned int test0;
//    unsigned int test1;
    unsigned int SlaveCh;
}SlaveReg;

#endif  // end of PARAMETER.H definition


//===========================================================================
// No more.
//===========================================================================
