/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "DSP2833x_Mcbsp.h"
#include "INS_Init.h"
#include "McBSP_SPI.h"
//---------------------------------------------------------------------------
// MCBSP_INIT_DELAY determines the amount of CPU cycles in the 2 sample rate
// generator (SRG) cycles required for the Mcbsp initialization routine.
// MCBSP_CLKG_DELAY determines the amount of CPU cycles in the 2 clock
// generator (CLKG) cycles required for the Mcbsp initialization routine.
// For the functions defined in Mcbsp.c, MCBSP_INIT_DELAY and MCBSP_CLKG_DELAY
// are based off of either a 150 MHz SYSCLKOUT (default) or a 100 MHz SYSCLKOUT.
//
// CPU_FRQ_100MHZ and CPU_FRQ_150MHZ are defined in DSP2833x_Examples.h
//---------------------------------------------------------------------------

#pragma CODE_SECTION(init_mcbspa_spi, "xintfloadfuncs");
#pragma CODE_SECTION(init_mcbspb_spi, "xintfloadfuncs");
#pragma CODE_SECTION(mcbspa_spi_send8bit, "xintfloadfuncs");
#pragma CODE_SECTION(mcbspb_spi_send8bit, "xintfloadfuncs");
#pragma CODE_SECTION(delay_loop, "xintfloadfuncs");


#if CPU_FRQ_150MHZ                                          // For 150 MHz SYSCLKOUT(default)
  #define CPU_SPD              150E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif
#if CPU_FRQ_100MHZ                                          // For 100 MHz SYSCLKOUT
  #define CPU_SPD              100E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif

#define MCBSP_INIT_DELAY     2*(CPU_SPD/MCBSP_SRG_FREQ)                  // # of CPU cycles in 2 SRG cycles-init delay
#define CLKGDV_VAL           0
#define MCBSP_CLKG_DELAY     2*(CPU_SPD/(MCBSP_SRG_FREQ/(1+CLKGDV_VAL))) // # of CPU cycles in 2 CLKG cycles-init delay

void init_mcbspa_spi(void);
void init_mcbspb_spi(void);
Uint16 mcbspa_spi_send8bit(Uint16 data1);
Uint16 mcbspb_spi_send8bit(Uint16 data1);
void delay_loop(void);

void delay_loop(void)
{
    long      i;
    for (i = 0; i < MCBSP_INIT_DELAY; i++) {} //delay in McBsp init. must be at least 2 SRG cycles
}

void init_mcbspa_spi(void)
{
	// Set GPIOs for McBSP-A
	// MDXA   (MOSI) GPIO20 (O)
	// MDRA   (MISO) GPIO21 (I)
	// MCLKXA (SCLK) GPIO22 (O)
	// MFSXA  (CSO)  GPIO23 (O)
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;	   // Enable pull-up for GPIO20
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;	   // Enable pull-up for GPIO21
	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;	   // Enable pull-up for GPIO22
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;	   // Enable pull-up for GPIO23

	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;    // Out
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;    // In
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;    // Out
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;    // Out
	
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2;
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2;
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2;
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 2;
	EDIS;	
	
	// McBSP-A register settings
	McbspaRegs.SPCR2.all=0x0000;		 // Reset FS generator, sample rate generator & transmitter
	McbspaRegs.SPCR1.all=0x0000;		 // Reset Receiver, Right justify word, Digital loopback dis.
	McbspaRegs.PCR.all=0x0F08;			 // (CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
	McbspaRegs.SPCR1.bit.DLB = 1;		 // Enable loopback
	McbspaRegs.SPCR1.bit.CLKSTP = 2;	 // Together with CLKXP/CLKRP determines clocking scheme
	McbspaRegs.PCR.bit.CLKXP = 0;		 // CPOL = 0, CPHA = 0 rising edge no delay
	McbspaRegs.PCR.bit.CLKRP = 0;
	McbspaRegs.RCR2.bit.RDATDLY=01; 	 // FSX setup time 1 in master mode. 0 for slave mode (Receive)
	McbspaRegs.XCR2.bit.XDATDLY=01; 	 // FSX setup time 1 in master mode. 0 for slave mode (Transmit)
	
	//	McbspaRegs.RCR1.bit.RWDLEN1=5;	 // 32-bit word
	//	McbspaRegs.XCR1.bit.XWDLEN1=5;	 // 32-bit word
	
	McbspaRegs.SRGR2.all=0x2000;		 // CLKSM=1, FPER = 1 CLKG periods
	McbspaRegs.SRGR1.all= 0x000F;		 // Frame Width = 1 CLKG period, CLKGDV=16
	
	McbspaRegs.SPCR2.bit.GRST=1;		 // Enable the sample rate generator
	delay_loop();						 // Wait at least 2 SRG clock cycles
	McbspaRegs.SPCR2.bit.XRST=1;		 // Release TX from Reset
	McbspaRegs.SPCR1.bit.RRST=1;		 // Release RX from Reset
	McbspaRegs.SPCR2.bit.FRST=1;		 // Frame Sync Generator reset
}

void init_mcbspb_spi(void)
{
	// Set GPIOs for McBSP-B
	// MDXB   (MOSI) GPIO12 (O)
	// MDRB   (MISO) GPIO13 (I)
	// MCLKXB (SCLK) GPIO14 (O)
	// MFSXB  (CSO)  GPIO15 (O)
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;	   // Enable pull-up for GPIO12
	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;	   // Enable pull-up for GPIO13
	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;	   // Enable pull-up for GPIO14
	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;	   // Enable pull-up for GPIO15

	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;    // Out
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;    // In
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;    // Out
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;    // Out
	
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 3;
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 3;
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 3;
	EDIS;	
	
	// McBSP-B register settings
	McbspbRegs.SPCR2.all=0x0000;		 // Reset FS generator, sample rate generator & transmitter
	McbspbRegs.SPCR1.all=0x0000;		 // Reset Receiver, Right justify word, Digital loopback dis.
	McbspbRegs.PCR.all=0x0F08;			 // (CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
	McbspbRegs.SPCR1.bit.DLB = 1;		 // Enable loopback
	McbspbRegs.SPCR1.bit.CLKSTP = 2;	 // Together with CLKXP/CLKRP determines clocking scheme
	McbspbRegs.PCR.bit.CLKXP = 0;		 // CPOL = 0, CPHA = 0 rising edge no delay
	McbspbRegs.PCR.bit.CLKRP = 0;
	McbspbRegs.RCR2.bit.RDATDLY=01; 	 // FSX setup time 1 in master mode. 0 for slave mode (Receive)
	McbspbRegs.XCR2.bit.XDATDLY=01; 	 // FSX setup time 1 in master mode. 0 for slave mode (Transmit)
	
	//	McbspbRegs.RCR1.bit.RWDLEN1=5;	 // 32-bit word
	//	McbspbRegs.XCR1.bit.XWDLEN1=5;	 // 32-bit word
	
	McbspbRegs.SRGR2.all=0x2000;		 // CLKSM=1, FPER = 1 CLKG periods
	McbspbRegs.SRGR1.all= 0x000F;		 // Frame Width = 1 CLKG period, CLKGDV=16
	
	McbspbRegs.SPCR2.bit.GRST=1;		 // Enable the sample rate generator
	delay_loop();						 // Wait at least 2 SRG clock cycles
	McbspbRegs.SPCR2.bit.XRST=1;		 // Release TX from Reset
	McbspbRegs.SPCR1.bit.RRST=1;		 // Release RX from Reset
	McbspbRegs.SPCR2.bit.FRST=1;		 // Frame Sync Generator reset
}

Uint16 mcbspa_spi_send8bit(Uint16 data1)
{
	while(McbspaRegs.SPCR2.bit.XRDY == 0){} //waiting for DXR[2,1] to receive data
	//	  McbspaRegs.DXR2.all = 0x0000;
	McbspaRegs.DXR1.all = data1 & 0x00FF;

	// wait for receive data ready
	while( McbspaRegs.SPCR1.bit.RRDY == 0 ) {}
	return McbspaRegs.DRR1.all;
}

Uint16 mcbspb_spi_send8bit(Uint16 data1)
{
	while(McbspbRegs.SPCR2.bit.XRDY == 0){} //waiting for DXR[2,1] to receive data
	//	  McbspbRegs.DXR2.all = 0x0000;
	McbspbRegs.DXR1.all = data1 & 0x00FF;

	// wait for receive data ready
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}
	return McbspbRegs.DRR1.all;	
}

