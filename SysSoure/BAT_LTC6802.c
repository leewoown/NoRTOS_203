#include "DSP28x_Project.h"
#include "parameter.h"
#include "ProtectRelay.h"
#include "BAT_LTC6802.h"
#include <stdio.h>
#include <math.h>
#include <string.h>



unsigned short pec15Table[256];
unsigned short CRC15POLY = 0x4599;
int ltc_state;
int ltc_error_count;
LTC6804_t LTC6804;
char LTC6804_init_table[6] =
{
    0xfc,       // GPIOx : pull down off, Ref ON, SWTEN, ADCOPT=0
    0x00,       // VUV[7:0]
    0x00,       // VOV[3:0]VUV[11:8]
    0x00,       // VOV[11:4]
    0x00,       // DCC8~DCC1
    0x00        // DCTO disable, DCC12~DCC9
};

//extern void BAT_InitSPI(void);
extern void SPI_BATWrite(unsigned int WRData);
extern void init_PEC15_Table(void);
extern unsigned short pec15(char *data, int len);
extern int LTC6804_Init();
//extern void LTC6804_WakeUp(int Ch);
extern int LTC6804_write(char address, short command, char data[], int len);
//extern int LTC6804_write_cmd(char address, short command, char data[], int len, int Ch);
extern int LTC6804_write_cmd(char address, short command, char data[], int len, int Ch);
//extern int LTC6804_read_cmd(char address, short command, char data[], int len, int Ch);
extern int LTC6804_read_cmd(char address, short command, char data[], int len, int Ch);
extern void LTC6804_WakeUp(int Ch);
extern void SPI_Write(unsigned int WRData);
extern unsigned int SPI_Read(void);
extern int SlaveBMSIint(SlaveReg *s);
extern void SlaveBMSDigiteldoutOHandler(SlaveReg *P);
extern void SalveTempsHandler(SlaveReg *s);
extern void SlaveVoltagHandler(SlaveReg *s);
/*
//void BAT_InitSPI(void)
{
    //  Uint16 i;

    SpiaRegs.SPICCR.bit.SPISWRESET      = 0;    // SPI SW Reset hold
    SpiaRegs.SPICCR.bit.CLKPOLARITY     = 1;    // Falling edge output
    SpiaRegs.SPICCR.bit.SPILBK          = 0;    // Disable Loopback
    SpiaRegs.SPICCR.bit.SPICHAR         = 0x7;  // 8bit character

    SpiaRegs.SPICTL.bit.SPIINTENA       = 0;    // Disable SPI int
    SpiaRegs.SPICTL.bit.TALK            = 1;    // Tx enable
    SpiaRegs.SPICTL.bit.MASTER_SLAVE    = 1;    // Master mode
    SpiaRegs.SPICTL.bit.CLK_PHASE       = 0;    // without delay mode
    SpiaRegs.SPICTL.bit.OVERRUNINTENA   = 0;    // Disable OverRun int

     //SpiaRegs.SPIBRR                   = 50;//150 / 5 - 1; // Baud rate = LSPCLK/(SPIBRR+1), ~500k (for 6804 testing)
     SpiaRegs.SPIBRR                     = 119;   //=170; // Baud rate = LSPCLK/(SPIBRR+1) 약 15/(3+1) = 3.75 MHz                                                        // when SPIBRR 3 to 127
     SpiaRegs.SPICCR.bit.SPISWRESET      = 1;    // SPI SW Reset release

}
*/
void SPI_BATWrite(unsigned int WRData)
{
    unsigned int Dummy;
    unsigned int Tmp;
    SpiaRegs.SPICCR.bit.SPICHAR = 0x07;
    Dummy = (WRData<<8)&0xFF00;
    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG);
    SpiaRegs.SPITXBUF = Dummy;              // Send
    while(SpiaRegs.SPISTS.bit.INT_FLAG!=1); // Wait for Tx   전송이 끝났거나 수신이 시작되면 1이됨.
    Tmp=SpiaRegs.SPIRXBUF;
    Tmp=Tmp;
}

unsigned int SPI_BATRead(void)
{
    Uint16 ReadData;
    Uint16 Dummy=0x0000;
    SpiaRegs.SPICCR.bit.SPICHAR = 0x07;
    Dummy = (Dummy<<8)&0xFF00;
    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG);
    SpiaRegs.SPITXBUF = Dummy;              // Send
    while(SpiaRegs.SPISTS.bit.INT_FLAG!=1); // Wait for Tx전송이 끝났거나 수신이 시작되면 1이됨.
    delay_us(30);
    ReadData = SpiaRegs.SPIRXBUF& 0xff;
    return (ReadData);
}
void BATSPIEnable_lowA(void)
{
//  delay_us(50);
   GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
 //  delay_us(50);
}

void BATSPIEnable_highA(void)
{
    delay_us(50);
    GpioDataRegs.GPASET.bit.GPIO10 = 1;

}
void BATSPIEnable_lowB(void)
{
    GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;
}

void BATSPIEnable_highB(void)
{
    delay_us(50);
    GpioDataRegs.GPBSET.bit.GPIO41 = 1;

}
void init_PEC15_Table(void)
{
    int i;
    int bit;

    for (i = 0; i < 256; i++)
    {
        unsigned short remainder = i << 7;
        for (bit = 8; bit > 0; --bit)
        {
            if (remainder & 0x4000)
            {
                remainder = (remainder << 1);
                remainder = (remainder ^ CRC15POLY);
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
        pec15Table[i] = remainder;
    }
}
unsigned short pec15(char *data, int len)
{
    unsigned short remainder,address;
    int i;
    remainder = 16;//PEC seed
    for (i = 0; i < len; i++)
    {
        address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
        remainder = (remainder << 8 ) ^ pec15Table[address];
    }
    return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}
int LTC6804_write(char address, short command, char data[], int len)
{
    char buffer[32];
    short pec;
    int i;

    if (len != 0 && len + 6 > 32)
    {
        return 0;
        //return 1;
    }
    // make command & it's PEC
    buffer[0] = 0x80|((address<<3)&0x78)|((command>>8) & 0x07);
    buffer[1] = command & 0xff;
    pec = pec15(buffer, 2);
    buffer[2] = (pec>>8);
    buffer[3] = pec;
    // make data & it's PEC
    if (len != 0)
    {
        for (i = 0; i < len; i++)
        {
            buffer[4 + i] = data[i];
        }
        pec = pec15(data, len);
        buffer[4 + i] = (pec >> 8);
        buffer[4 + i + 1] = pec;

        len += 2;       // + 2 bytes pec    중요!!! don't remove!!!
    }
    // send all
    for(i=0;i<len+4;i++)
    {
        SPI_BATWrite(buffer[i]);
    }
    return (len+4); // LEEWOOWON Changes on '19.04.13
    //return len;
}

int LTC6804_write_cmd(char address, short command, char data[], int len, int Ch)
{
    int ret;
    LTC6804_WakeUp(Ch);
    if(Ch==0){BATSPIEnable_lowA();} else{BATSPIEnable_lowB();}
    ret = LTC6804_write(address, command, data, len);
    if(Ch==0){ BATSPIEnable_highA();} else{BATSPIEnable_highB();}

    return ret;
}
int LTC6804_read_cmd(char address, short command, char data[], int len, int Ch)
{
    int i;
    int valid = 0;
    int ret;
    LTC6804_WakeUp(Ch);
    //BATSPIEnable_low();
    if(Ch==0){ BATSPIEnable_lowA();} else{BATSPIEnable_lowB();}
    ret = LTC6804_write(address, command, 0, 0);
    if (ret != 0 && len != 0)
    {
        unsigned short pecr;
        unsigned short pecg;
        for (i = 0; i < len; i++)
        {
            data[i] = SPI_Read();
        }
        pecr = (SPI_Read() << 8) & 0xff00;
        pecr |= (SPI_Read() & 0x00ff);
        pecg = pec15(data, len);
        if (pecr == pecg)
        {
            valid = 1;
            ltc_state = 1;
            ltc_error_count = 0;
        } else
        {
            ltc_state = 0;
            ltc_error_count += 1;
        }
    }
    //BATSPIEnable_high();
    if(Ch==0){BATSPIEnable_highA();} else{BATSPIEnable_highB();}
    return valid;
}
/*
int LTC6804_Init()
{
    int i;
    // write PEC table
    init_PEC15_Table();
    // init variables
    LTC6804.count = 2;          // 전체 팩 갯수
    LTC6804.address[0] = 0xf;   // 0번 팩 주소 설정
    LTC6804.address[1] = 0;     // 1번 팩 주소 설정
    ltc_state = 0;
    ltc_error_count = 0;
    // write initial values
    for (i = 0; i < LTC6804.count; i++)
    {
        LTC6804_write_cmd(LTC6804.address[i], LTC6804_CMD_WRCFG, (char *)LTC6804_init_table, sizeof(LTC6804_init_table));
    }
    return 0;
}
*/
void LTC6804_WakeUp(int Ch)
{
    // need to check
   // BATSPIEnable_low();
    if(Ch==0){BATSPIEnable_lowA();} else{BATSPIEnable_lowB();}
    SPI_BATWrite(0);
    SPI_BATWrite(0);
    SPI_BATWrite(0);
    SPI_BATWrite(0);
    SPI_BATWrite(0);
   // BATSPIEnable_high();
    if(Ch==0){BATSPIEnable_highA();} else{BATSPIEnable_highB();}
}

int LTC6804_DieTemperatureRead(int pack_id, float *temperature,int Ch)
{
    int ret;
    char addr = LTC6804.address[pack_id];
    char buf[6];
    unsigned short soc;
    unsigned short itmp;
    unsigned short va;
    ret=LTC6804_write_cmd(addr,LTC6804_CMD_ADSTAT | (2 << 7) | (0 << 0),0, 0,Ch);
    if(ret == 0)
    {
        return 0;
    }
    delay_us(1563);
    ret = LTC6804_read_cmd(addr, LTC6804_CMD_RDSTATA, buf, 6,Ch);
    if (ret == 0)
    {
        *temperature = 0;
        return ret;
    }
    soc   = ((buf[1] << 8) & 0xff00) | (buf[0] & 0x00ff);
    itmp  = ((buf[3] << 8) & 0xff00) | (buf[2] & 0x00ff);
    va    =  ((buf[5] << 8) & 0xff00) | (buf[4] & 0x00ff);
    soc   = soc;
    va    = va;
    *temperature = itmp;
    {
        // [die temp(C)] = (ITMP) * 100uV / (7.5mV) - 273
        //float tmp = (float)itmp * 100 / 7500 - 273.0;
        //*temperature = (float)itmp / 75.0 - 273.0;
    }

    return ret;
}
int LTC6804_SocItmpVaRead(int pack_id, unsigned short *osoc, float *oitmp, unsigned short *ova,int Ch)
{
    int ret;
    char addr = LTC6804.address[pack_id];
    char buf[6];
    unsigned short soc;
    unsigned short itmp;
    unsigned short va;

    ret = LTC6804_write_cmd(addr, LTC6804_CMD_ADSTAT | (2 << 7) | (0 << 0), 0, 0, Ch);
    if (ret == 0)
    {
        return 0;
    }

    delay_us(1563);

    ret = LTC6804_read_cmd(addr, LTC6804_CMD_RDSTATA, buf, 6,Ch);
    if (ret == 0)
    {
        *osoc = 0;
        *oitmp = 0.0;
        *ova = 0;
        return ret;
    }
    soc = ((buf[1] << 8) & 0xff00) | (buf[0] & 0x00ff);
    itmp = ((buf[3] << 8) & 0xff00) | (buf[2] & 0x00ff);
    va = ((buf[5] << 8) & 0xff00) | (buf[4] & 0x00ff);
    *osoc = soc;
    {
        // [die temp(C)] = (ITMP) * 100uV / (7.5mV) - 273
        //float tmp = (float)itmp * 100 / 7500 - 273.0;
        *oitmp = (float)itmp / 75.0 - 273.0;
    }
    *ova = va;

    return ret;
}
//int LTC6804_write_cmd(char address, short command, char data[], int len)
int SlaveBMSIint(SlaveReg *s)
{

    s->StateMachine = STATE_BATIDLE;
    if(s->StateMachine == STATE_BATIDLE)
    {

        s->TempsChSelet=0;
        s->TempsChSeletCount=0;
        s->GPIO0ADC=0;
        s->GPIORef=0;
        s->BATICDO.all=0;
        s->Balance.all=0;
        s->ErrorCount=0;
        memset(&s->CellVoltage[0],3200,12);
        memset(&s->CellVoltageBuf[0],3200,12);
        memset(&s->CellVoltageF[0],3.2,sizeof(float32)*12);
        memset(&s->DivVoltageF[0],0.0, sizeof(float32)*12);

        memset(&s->CellTemperatureADC[0],0,12);
        memset(&s->CellTemperatureNor[0],0,12);
        memset(&s->CellTemperatureVF[0],0.0,sizeof(float32)*12);
        memset(&s->CellTemperatureF[0], 20.5,sizeof(float32)*12);
        memset(&s->CellTemperatureFBuf[0], 20.5,sizeof(float32)*12);
        memset(&s->CellTemperatureBuf[0],250,12);
        memset(&s->CellTemperature[0],250,12);

        init_PEC15_Table();
        LTC6804_write_cmd(s->ID, LTC6804_CMD_WRCFG, (char *)LTC6804_init_table, sizeof(LTC6804_init_table),s->SlaveCh);
    }
    s->StateMachine = STATE_BATSTANDBY;
    return 0;
}
void SlaveBMSDigiteldoutOHandler(SlaveReg *P)
{
   // int ret;
    char CommandBuf[6];
    // 0xfc,       // GPIOx : pull down off, Ref ON, SWTEN, ADCOPT=0
    //int LTC6804_write_cmd(char address, short command, char data[], int len)
    P->BATICDO.bit.ADCCOPT=0;
    P->BATICDO.bit.SWTRD=0;
    P->BATICDO.bit.REFON =1;
    P->TempsChSeletCount++;

    if(P->TempsChSeletCount>12){P->TempsChSeletCount=0;}
    switch (P->TempsChSeletCount)
    {

        case 0 :
                  P->BATICDO.bit.GPIO1=1;

                  P->BATICDO.bit.GPIO2=0;
                  P->BATICDO.bit.GPIO3=0;
                  P->BATICDO.bit.GPIO4=0;
                  P->BATICDO.bit.GPIO5=0;
                  P->TempsChSelet =5;

         break;
         case 1 :
                 P->BATICDO.bit.GPIO1=1; //1

                 P->BATICDO.bit.GPIO2=1;
                 P->BATICDO.bit.GPIO3=0;
                 P->BATICDO.bit.GPIO4=0;
                 P->BATICDO.bit.GPIO5=0;
                  P->TempsChSelet =6;
         break;
         case 2 :

                 P->BATICDO.bit.GPIO1=1; //2

                 P->BATICDO.bit.GPIO2=0;
                 P->BATICDO.bit.GPIO3=1;
                 P->BATICDO.bit.GPIO4=0;
                 P->BATICDO.bit.GPIO5=0;
                 P->TempsChSelet =7;

         break;
         case 3 :

                 P->BATICDO.bit.GPIO1=1; //3

                 P->BATICDO.bit.GPIO2=1;
                 P->BATICDO.bit.GPIO3=1;
                 P->BATICDO.bit.GPIO4=0;
                 P->BATICDO.bit.GPIO5=0;
                 P->TempsChSelet =8;

         break;
         case 4 :

                 P->BATICDO.bit.GPIO1=1; //4

                 P->BATICDO.bit.GPIO2=0;
                 P->BATICDO.bit.GPIO3=0;
                 P->BATICDO.bit.GPIO4=1;
                 P->BATICDO.bit.GPIO5=0;
                 P->TempsChSelet =9;
         break;
         case 5 :

                 P->BATICDO.bit.GPIO1=1; //5

                 P->BATICDO.bit.GPIO2=1;
                 P->BATICDO.bit.GPIO3=0;
                 P->BATICDO.bit.GPIO4=1;
                 P->BATICDO.bit.GPIO5=0;
                 P->TempsChSelet =10;
         break;
         case 6 :

                 P->BATICDO.bit.GPIO1=1; //6

                 P->BATICDO.bit.GPIO2=0;
                 P->BATICDO.bit.GPIO3=1;
                 P->BATICDO.bit.GPIO4=1;
                 P->BATICDO.bit.GPIO5=0;
                 P->TempsChSelet =11;
         break;
         case 7 :

                 P->BATICDO.bit.GPIO1=1; //7

                 P->BATICDO.bit.GPIO2=1;
                 P->BATICDO.bit.GPIO3=1;
                 P->BATICDO.bit.GPIO4=1;
                 P->BATICDO.bit.GPIO5=0;
                 P->TempsChSelet =0;
         break;
         case 8 :

                 P->BATICDO.bit.GPIO1=1; //8

                 P->BATICDO.bit.GPIO2=0;
                 P->BATICDO.bit.GPIO3=0;
                 P->BATICDO.bit.GPIO4=0;
                 P->BATICDO.bit.GPIO5=1;
                 P->TempsChSelet =1;
         break;
         case 9 :

                 P->BATICDO.bit.GPIO1=1; //9

                 P->BATICDO.bit.GPIO2=1;
                 P->BATICDO.bit.GPIO3=0;
                 P->BATICDO.bit.GPIO4=0;
                 P->BATICDO.bit.GPIO5=1;
                 P->TempsChSelet =2;
         break;
         case 10 :

                 P->BATICDO.bit.GPIO1=1; //10

                 P->BATICDO.bit.GPIO2=0;
                 P->BATICDO.bit.GPIO3=1;
                 P->BATICDO.bit.GPIO4=0;
                 P->BATICDO.bit.GPIO5=1;
                 P->TempsChSelet =3;
         break;
         case 11 :

                 P->BATICDO.bit.GPIO1=1; //11

                 P->BATICDO.bit.GPIO2=1;
                 P->BATICDO.bit.GPIO3=1;
                 P->BATICDO.bit.GPIO4=0;
                 P->BATICDO.bit.GPIO5=1;
                 P->TempsChSelet =4;
         break;
         default :
         break;

    }

    CommandBuf[0] = (char) P->BATICDO.all;
    CommandBuf[1] = 0x00;
    CommandBuf[2] = 0x00;
    CommandBuf[3] = 0x00;
    CommandBuf[4] = 0x00;
    CommandBuf[5] = 0x00;

    P->Error=LTC6804_write_cmd(P->ID,LTC6804_CMD_WRCFG,CommandBuf,sizeof(CommandBuf),P->SlaveCh);
    if(P->Error==1)
    {
        P->ErrorCount=0;
    }
    else
    {
        P->ErrorCount++;
    }

}
void SalveTempsHandler(SlaveReg *s)
{
    float32 TempsAVaule=0;
    float32 TempsBVaule=0;
    float32 TempsCVaule=0;
    float32 TempsDVaule=0;
    float32 TempsAVaule3X=0;
    float32 TempsBVaule2X=0;
    float32 TempsCVaule1X=0;
    float32 TempsDVaule0x=0;
    float32 TempsVaule =0;

    s->Error  = LTC6804_write_cmd(s->ID,LTC6804_CMD_ADAX |(1 << 8)|(0 << 4)|(0 << 0),0,0,s->SlaveCh);
    if(s->Error==1)
    {
        s->ErrorCount=0;
    }
    else
    {
        s->ErrorCount++;
    }
    s->Error  = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDAUXA, s->ADCV,6,s->SlaveCh);
    if(s->Error==1)
    {
        s->ErrorCount=0;
        s->CellTemperatureADC[s->TempsChSelet] = ((s->ADCV[1] << 8) & 0xff00) | (s->ADCV[0]  & 0x00ff);//GPIO1
        s->CellTemperatureNor[s->TempsChSelet] = s->CellTemperatureADC[s->TempsChSelet]/10;
        s->CellTemperatureVF [s->TempsChSelet] = (float32)(s->CellTemperatureNor[s->TempsChSelet]*0.001);

       // s->CellTemperatureADC[1]             =((s->ADCV[3] << 8) & 0xff00) | (s->ADCV[2]  & 0x00ff);//GPIO2
       // s->CellTemperatureADC[2]             =((s->ADCV[5] << 8) & 0xff00) | (s->ADCV[4]  & 0x00ff);//GPIO3
    }
    s->Error  = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDAUXB, s->ADCV, 6,s->SlaveCh);
    if(s->Error==1)
    {
        s->ErrorCount=0;
      //  s->CellTemperatureADC[3]               =((s->ADCV[1] << 8) & 0xff00) | (s->ADCV[0]  & 0x00ff);//GPIO4
      //  s->CellTemperatureADC[s->TempsChSelet] =((s->ADCV[3] << 8) & 0xff00) | (s->ADCV[2]  & 0x00ff);//GPIO5
        s->GPIORef                               =((s->ADCV[5] << 8) & 0xff00) | (s->ADCV[4]  & 0x00ff);//Ref
    }
    else
    {
        s->ErrorCount++;
    }
    TempsAVaule3X = (s->CellTemperatureVF [s->TempsChSelet]*s->CellTemperatureVF [s->TempsChSelet])*s->CellTemperatureVF [s->TempsChSelet];
    TempsAVaule = C_TempsAGain*TempsAVaule3X;

    TempsBVaule2X = s->CellTemperatureVF [s->TempsChSelet]*s->CellTemperatureVF [s->TempsChSelet];
    TempsBVaule = C_TempsBGain*TempsBVaule2X;

    TempsCVaule1X = s->CellTemperatureVF [s->TempsChSelet];
    TempsCVaule = C_TempsCGain*TempsCVaule1X;

    TempsDVaule0x = C_TempsDGain;
    TempsDVaule = TempsDVaule0x;

    TempsVaule = TempsAVaule+TempsBVaule+TempsCVaule+TempsDVaule;
    s->CellTemperatureFBuf[s->TempsChSelet]= TempsVaule;
    s->CellTemperatureBuf[s->TempsChSelet] =(int)(TempsVaule*10);
    if(s->TempsChSelet==0)
    {
        memcpy(&s->CellTemperatureF[0], &s->CellTemperatureFBuf[0],sizeof(float32)*12);
        memcpy(&s->CellTemperature[0],  &s->CellTemperatureBuf[0], sizeof(int)*12);
    }

}
void SlaveVoltagHandler(SlaveReg *s)
{
    if(s->StateMachine==STATE_BATREAD)
    {

       s->Error = LTC6804_write_cmd(s->ID,LTC6804_CMD_ADCV | (1 << 8)|(0 << 4)|(0 << 0),0, 0,s->SlaveCh);
       if(s->Error==1)
       {
           s->ErrorCount=0;
       }
       else
       {
           s->ErrorCount++;
       }
       s->Error  = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVA, s->ADCX, 6,s->SlaveCh);
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[0]  = ((s->ADCX[1] << 8) & 0xff00) | (s->ADCX[0]  & 0x00ff);
           s->CellVoltageBuf[1]  = ((s->ADCX[3] << 8) & 0xff00) | (s->ADCX[2]  & 0x00ff);
           s->CellVoltageBuf[2]  = ((s->ADCX[5] << 8) & 0xff00) | (s->ADCX[4]  & 0x00ff);
           s->CellVoltage[0]     = s->CellVoltageBuf[0]/10;
           s->CellVoltage[1]     = s->CellVoltageBuf[1]/10;
           s->CellVoltage[2]     = s->CellVoltageBuf[2]/10;
       }
       else
       {
           s->ErrorCount++;
       }
       s->Error = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVB, s->ADCX, 6,s->SlaveCh);;
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[3] = ((s->ADCX[1] << 8 & 0xff00)  | (s->ADCX[0]) & 0x00ff);
           s->CellVoltageBuf[4] = ((s->ADCX[3] << 8 & 0xff00)  | (s->ADCX[2]) & 0x00ff);
           s->CellVoltageBuf[5] = ((s->ADCX[5] << 8 & 0xff00)  | (s->ADCX[4]) & 0x00ff);
           s->CellVoltage[3]    = s->CellVoltageBuf[3]/10;
           s->CellVoltage[4]    = s->CellVoltageBuf[4]/10;
           s->CellVoltage[5]    = s->CellVoltageBuf[5]/10;
       }
       else
       {
           s->ErrorCount++;
       }
       s->Error = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVC, s->ADCX, 6,s->SlaveCh);
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[6] = ((s->ADCX[1] << 8 & 0xff00)  | (s->ADCX[0]) & 0x00ff);
           s->CellVoltageBuf[7] = ((s->ADCX[3] << 8 & 0xff00)  | (s->ADCX[2]) & 0x00ff);
           s->CellVoltageBuf[8] = ((s->ADCX[5] << 8 & 0xff00)  | (s->ADCX[4]) & 0x00ff);
           // 소수정 3자리 변환
           s->CellVoltage[6] = s->CellVoltageBuf[6]/10;
           s->CellVoltage[7] = s->CellVoltageBuf[7]/10;
           s->CellVoltage[8] = s->CellVoltageBuf[8]/10;
       }
       else
       {
           s->ErrorCount++;
       }
       s->Error = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVD, s->ADCX, 6,s->SlaveCh);
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[9]  = ((s->ADCX[1] << 8 & 0xff00)  | (s->ADCX[0]) & 0x00ff);
           s->CellVoltageBuf[10] = ((s->ADCX[3] << 8 & 0xff00)  | (s->ADCX[2]) & 0x00ff);
           s->CellVoltageBuf[11] = ((s->ADCX[5] << 8 & 0xff00)  | (s->ADCX[4]) & 0x00ff);
           // 소수정 3자리 변환
           s->CellVoltage[9]    = s->CellVoltageBuf[9]/10;
           s->CellVoltage[10]   = s->CellVoltageBuf[10]/10;
           s->CellVoltage[11]   = s->CellVoltageBuf[11]/10;
       }
       else
       {
           s->ErrorCount++;
       }
       s->CellVoltageF[0]     = (float32)s->CellVoltage[0]*0.001;  //1
       s->CellVoltageF[1]     = (float32)s->CellVoltage[1]*0.001;  //2
       s->CellVoltageF[2]     = (float32)s->CellVoltage[2]*0.001;  //3
       s->CellVoltageF[3]     = (float32)s->CellVoltage[3]*0.001;  //4
       s->CellVoltageF[4]     = (float32)s->CellVoltage[4]*0.001;  //5
       s->CellVoltageF[5]     = (float32)s->CellVoltage[5]*0.001;  //6
       s->CellVoltageF[6]     = (float32)s->CellVoltage[6]*0.001;  //7
       s->CellVoltageF[7]     = (float32)s->CellVoltage[7]*0.001;  //8
       s->CellVoltageF[8]     = (float32)s->CellVoltage[8]*0.001;  //9
       s->CellVoltageF[9]     = (float32)s->CellVoltage[9]*0.001;  //10
       s->CellVoltageF[10]    = (float32)s->CellVoltage[10]*0.001; //11
       s->CellVoltageF[11]    = (float32)s->CellVoltage[11]*0.001; //11
    }


}
int SlaveBmsBalance(SlaveReg *s)
{
   //int ret;
    s->BATICDO.bit.ADCCOPT=0;
    s->BATICDO.bit.SWTRD=0;
    s->BATICDO.bit.REFON =1;
    s->BalanceTable[0]=(char)s->BATICDO.all;
    s->BalanceTable[1]=0x00;
    s->BalanceTable[2]=0x00;
    s->BalanceTable[3]=0x00;
    s->BalanceTable[4]= (s->Balance.all & 0xff);
    s->BalanceTable[5]= (s->BalanceTable[5] & 0xf0) | ((s->Balance.all >> 8) & 0x0f);
    s->Error = LTC6804_write_cmd(s->ID,LTC6804_CMD_WRCFG,s->BalanceTable,sizeof(s->BalanceTable),s->SlaveCh);
    if(s->Error==1)
    {
        s->ErrorCount=0;
    }
    else
    {
        s->ErrorCount++;
    }
    return s->Error;
}
void SlaveVoltagBalaHandler(SlaveReg *s)
{
    s->DivVoltageF[0]= s->CellVoltageF[0]-s->SysCellMinVoltage;
    if(s->DivVoltageF[0] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell00=1;
    }
    else
    {
        s->Balance.bit.B_Cell00=0;
    }

    s->DivVoltageF[1]=s->CellVoltageF[1]-s->SysCellMinVoltage;
    if(s->DivVoltageF[1] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell01=1;
    }
    else
    {
        s->Balance.bit.B_Cell01=0;
    }

    s->DivVoltageF[2]=s->CellVoltageF[2]-s->SysCellMinVoltage;
    if(s->DivVoltageF[2] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell02=1;
    }
    else
    {
        s->Balance.bit.B_Cell02=0;
    }
    s->DivVoltageF[3]=s->CellVoltageF[3]-s->SysCellMinVoltage;
    if(s->DivVoltageF[3] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell03=1;
    }
    else
    {
        s->Balance.bit.B_Cell03=0;
    }
    s->DivVoltageF[4]=s->CellVoltageF[4]-s->SysCellMinVoltage;
    if(s->DivVoltageF[4] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell04=1;
    }
    else
    {
        s->Balance.bit.B_Cell04=0;
    }
    s->DivVoltageF[5]=s->CellVoltageF[5]-s->SysCellMinVoltage;
    if(s->DivVoltageF[5] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell05=1;
    }
    else
    {
        s->Balance.bit.B_Cell05=0;
    }

    s->DivVoltageF[6]=s->CellVoltageF[6]-s->SysCellMinVoltage;
    if(s->DivVoltageF[6] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell06=1;
    }
    else
    {
        s->Balance.bit.B_Cell06=0;
    }

    s->DivVoltageF[7]=s->CellVoltageF[7]-s->SysCellMinVoltage;
    if(s->DivVoltageF[7] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell07=1;
    }
    else
    {
        s->Balance.bit.B_Cell07=0;
    }

    s->DivVoltageF[8]=s->CellVoltageF[8]-s->SysCellMinVoltage;
    if(s->DivVoltageF[8] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell08=1;
    }
    else
    {
        s->Balance.bit.B_Cell08=0;
    }
    s->DivVoltageF[9]=s->CellVoltageF[9]-s->SysCellMinVoltage;
    if(s->DivVoltageF[9] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell09=1;
    }
    else
    {
        s->Balance.bit.B_Cell09=0;
    }
    s->DivVoltageF[10]=s->CellVoltageF[10]-s->SysCellMinVoltage;
    if(s->DivVoltageF[10] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell10=1;
    }
    else
    {
        s->Balance.bit.B_Cell10=0;
    }
    s->DivVoltageF[11]=s->CellVoltageF[11]-s->SysCellMinVoltage;
    if(s->DivVoltageF[11] >= C_BalanceDivVoltage)
    {
        s->Balance.bit.B_Cell11=1;
    }
    else
    {
        s->Balance.bit.B_Cell11=0;
    }
}

