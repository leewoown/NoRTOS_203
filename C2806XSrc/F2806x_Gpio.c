//###########################################################################
//
// FILE:	F2806x_Gpio.c
//
// TITLE:	F2806x General Purpose I/O Initialization & Support Functions.
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

//
// Included Files
//
#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_Examples.h"   // F2806x Examples Include File

//
// InitGpio - This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example.
//
void InitGpio(void)
{
    EALLOW;
    //
    // Each GPIO pin can be:
    // a) a GPIO input/output
    // b) peripheral function 1
    // c) peripheral function 2
    // d) peripheral function 3
    // By default, all are GPIO Inputs
    //
    //GpioCtrlRegs.GPAMUX1.all = 0x0000;    // GPIO functionality GPIO0-GPIO15
    GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 0; // GPIO, DO, GATA_A, TOP_CAH
    GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 0; // GPIO, DO, GATA_B, TOP_DIS
    GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 0; // GPIO, DO, GATA_C, TOP_PRO
    GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 0; // GPIO, DO, GATA_D, BOT_CAH
    GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 0; // GPIO, DO, GATA_E, BOT_DIS
    GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 0; // GPIO, DO, GATA_F, STA_CAH
    GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0; // GIPO, DO, NRLY
    GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 0; // GIPO, DI, NRLY_AUX
    GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0; // GIPO, DI, RCTMF
    GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 0; // GIPO, DO, RCTEN
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; // GIPO, DO, BATEN
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; // GIPO, DO, NVRAMEN
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1; // TZ1,  DI, CANINT
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1; // TZ2,  DI, TCPINT
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1; // GPIO  DI, CANRX0INT
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0; // GPIO, DO, TCPRESET
    //GpioCtrlRegs.GPAMUX2.all = 0x0000;    // GPIO functionality GPIO16-GPIO31
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0; // GPIO, DI, DSW1
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0; // GPIO, DI, DSW2
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; // GPIO, DI, DSW3
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; // GPIO, DI, DSW4
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0; // GIPO, DO, 12VRLY_ON
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0; // GIPO, DO, 12VRLY_OFF
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0; // GIPO, DO, PRLAY
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0; // GIPO, DI, PRLY_AUX
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0; // GPIO, GATA_EN
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0; // NOT USED,
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0; // GIPO, IMDTOP_EN
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0; // GIPO, IMDBOT_EN
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1; // SCIRXA, DI, SCIRXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1; // SCITXA, DO, SCITXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1; // CANAL, DI, CANAL_IN
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1; // CANAH, DO, CANAH_IN
    //GpioCtrlRegs.GPBMUX1.all = 0x0000;    // GPIO functionality GPIO32-GPIO47
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0; // GIPO, DO, CAHRLY
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0; // GIPO, DI, CHA_AUX
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0; // NOT USED
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0; // TDI(사용 불가능)
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0; // TMS(사용 불가능)
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0; // TDO(사용 불가능)
    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0; // TCK(사용 불가능)
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0; // NOT CONNECTION(사용 불가능)
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0; // GPIO, GATA_H, STA_DIS
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // GPIO, SPIBATEN_1
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0; // GIPO, DO, TCPEN
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0; // GIPO, DO, SPICANEN
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0; // GPIO, DO, LED01(Fault),RS485 통신 필요 시 RS485A(RX)
    //GpioCtrlRegs.GPBMUX2.all = 0x0000  // GPIO functionality GPIO48-GPIO63
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0; // GPIO, DO, RE485EN
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0; // GPIO, DO, LED02(CANSTATE)
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0; // GPIO, DI, CANXIN1T
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0; // GPIO, NOT USED (회로 수정 필요함)
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1; //SPIMOSIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1; //SPIMISOA
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1; //SPICLK
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1; //NOT CONNECTION(사용 불가능,SPIEN)
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0; //GPIO, DO, LED00(STATE), RS485 통신 필요 시 RS485A(TX)


    //
    // Dig.IO funct. applies to AIO2,4,6,10,12,14
    //
//    GpioCtrlRegs.AIOMUX1.all = 0x0000;
    GpioCtrlRegs.AIOMUX1.bit.AIO2 = 2;    // Configure AIO2 for A2 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO4 = 2;    // Configure AIO4 for A4 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO6 = 2;    // Configure AIO6 for A6 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO10 = 2;   // Configure AIO10 for B2 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO12 = 2;   // Configure AIO12 for B4 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO14 = 2;   // Configure AIO14 for B6 (analog input) operation

//    GpioCtrlRegs.GPADIR.all = 0x0000;    // GPIO0-GPIO31 are GP inputs
    GpioCtrlRegs.GPADIR.bit.GPIO0  = 1; // GPIO, DO, GATA_A, TOP_CAH
    GpioCtrlRegs.GPADIR.bit.GPIO1  = 1; // GPIO, DO, GATA_B, TOP_DIS
    GpioCtrlRegs.GPADIR.bit.GPIO2  = 1; // GPIO, DO, GATA_C, TOP_PRO
    GpioCtrlRegs.GPADIR.bit.GPIO3  = 1; // GPIO, DO, GATA_D, BOT_CAH
    GpioCtrlRegs.GPADIR.bit.GPIO4  = 1; // GPIO, DO, GATA_E, BOT_DIS
    GpioCtrlRegs.GPADIR.bit.GPIO5  = 1; // GPIO, DO, GATA_F, STA_CAH
    GpioCtrlRegs.GPADIR.bit.GPIO6  = 1; // GIPO, DO, NRLY
    GpioCtrlRegs.GPADIR.bit.GPIO7  = 0; // GIPO, DI, NRLY_AUX
    GpioCtrlRegs.GPADIR.bit.GPIO8  = 0; // GIPO, DI, RCTMF
    GpioCtrlRegs.GPADIR.bit.GPIO9  = 1; // GIPO, DO, RCTEN
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; // GIPO, DO, BATEN_A
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1; // GIPO, DO, NVRAMEN
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 0; // TZ1,  DI, CANINT
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 0; // TZ2,  DI, TCPINT
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0; // GPIO  DI, CANRX0INT
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1; // GPIO, DO, TCPRESET
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 0; // GPIO, DI, DSW1
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0; // GPIO, DI, DSW2
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1; // GPIO, DI, DSW3
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0; // GPIO, DI, DSW4
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1; // GIPO, DO, 12VRLY_ON
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1; // GIPO, DO, 12VRLY_OFF
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1; // GIPO, DO, PRLAY
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0; // GIPO, DI, PRLY_AUX
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1; // GPIO, DO, GATA_EN
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0; // NOT USED,
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1; // GIPO, IMDTOP_EN
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1; // GIPO, IMDBOT_EN
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 0; // SCIRXA, DI, SCIRXA
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1; // SCITXA, DO, SCITXA
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0; // CANAL, DI, CANAL_IN
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1; // CANAH, DO, CANAH_IN
//    GpioCtrlRegs.GPBDIR.all = 0x0000;    // GPIO32-GPIO63 are inputs
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1; // GIPO, DO, CAHRLY
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0; // GIPO, DI, CHA_AUX
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0; // NOT USED
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 0; // TDI(사용 불가능)
    GpioCtrlRegs.GPBDIR.bit.GPIO36 = 0; // TMS(사용 불가능)
    GpioCtrlRegs.GPBDIR.bit.GPIO37 = 0; // TDO(사용 불가능)
    GpioCtrlRegs.GPBDIR.bit.GPIO38 = 0; // TCK(사용 불가능)
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0; // NOT CONNECTION(사용 불가능)
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0; // GPIO, DO, GATA_H, STA_DIS
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1; // GIPO, DO, BATEN_B
    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1; // GIPO, DO, TCPEN
    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1; // GIPO, DO, SPICANEN
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1; // GPIO, DO, LED01(Fault),RS485 통신 필요 시 RS485A(RX)
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1; // GPIO, DO, RE485EN
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1; // GPIO, DO, LED02(CANSTATE)
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0; // GPIO, DI, CANXIN1T
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0; // GPIO, NOT USED (회로 수정 필요함)
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1; //SPIMOSIA
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0; //SPIMISOA
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1; //SPICLK
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 0; //NOT CONNECTION(사용 불가능,SPIEN)
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1; //GPIO, DO, LED00(STATE) RS485 통신 필요 시 RS485A(TX)

    //
    // AIO2,4,6,10,12,14 are digital inputs
    GpioCtrlRegs.AIODIR.all = 0x0000;    

    //
    // Each input can have different qualification
    // a) input synchronized to SYSCLKOUT
    // b) input qualified by a sampling window
    // c) input sent asynchronously (valid for peripheral inputs only)
    //
    GpioCtrlRegs.GPAQSEL1.all = 0x0000;   // GPIO0-GPIO15 Synch to SYSCLKOUT
    GpioCtrlRegs.GPAQSEL2.all = 0x0000;   // GPIO16-GPIO31 Synch to SYSCLKOUT
    GpioCtrlRegs.GPBQSEL1.all = 0x0000;   // GPIO32-GPIO47 Synch to SYSCLKOUT
    GpioCtrlRegs.GPBQSEL2.all = 0x0000;	  // GPIO48-GPIO63 Synch to SYSCLKOUT

    //
    // Pull-ups can be enabled or disabled.
    //
    GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
    GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO44
    //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
    //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO44
    EDIS;
}

//
// End of file
//

