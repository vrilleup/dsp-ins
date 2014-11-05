/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com)
*/

/* Initialize Functions for DSP INS */
/* 2008.08.07 by VrilleUp */

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "INS_Init.h"
#include "INS_Func.h"
#include "GPS.h"
#include "McBSP_SPI.h"
#include "SCI_A.h"

#include <stdio.h>
#include <stdarg.h>

/********************************************************/

#pragma CODE_SECTION(scic_init, "xintfloadfuncs");
#pragma CODE_SECTION(scic_start_tx, "xintfloadfuncs");
#pragma CODE_SECTION(scic_xmit, "xintfloadfuncs");
#pragma CODE_SECTION(scic_xmit_ex, "xintfloadfuncs");
#pragma CODE_SECTION(scic_fifo_init, "xintfloadfuncs");
#pragma CODE_SECTION(scib_init, "xintfloadfuncs");
#pragma CODE_SECTION(scib_start_tx, "xintfloadfuncs");
#pragma CODE_SECTION(scib_xmit, "xintfloadfuncs");
#pragma CODE_SECTION(scib_xmit_ex, "xintfloadfuncs");
#pragma CODE_SECTION(scib_fifo_init, "xintfloadfuncs");
#pragma CODE_SECTION(scia_init, "xintfloadfuncs");
#pragma CODE_SECTION(scia_start_tx, "xintfloadfuncs");
#pragma CODE_SECTION(scia_xmit, "xintfloadfuncs");
#pragma CODE_SECTION(scia_xmit_ex, "xintfloadfuncs");
#pragma CODE_SECTION(scia_fifo_init, "xintfloadfuncs");
#pragma CODE_SECTION(scia_print, "xintfloadfuncs");
#pragma CODE_SECTION(SPIA_init, "xintfloadfuncs");
#pragma CODE_SECTION(SPIA_Mode, "xintfloadfuncs");
#pragma CODE_SECTION(SPIA_xmit, "xintfloadfuncs");
#pragma CODE_SECTION(ADS8341_Read, "xintfloadfuncs");

// *********** THESE FUNCTIONS ARE IN FLASH	************
// init_INS
// InitXintf16Gpio
// init_zone7

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

extern Uint16 XintfLoadfuncsLoadStart;
extern Uint16 XintfLoadfuncsLoadEnd;
extern Uint16 XintfLoadfuncsRunStart;

extern void ADC_cal(void);

/********************************************************/

BYTE SCIA_TX_BUF[SCI_TX_BUF_MAX];
BYTE SCIA_RX_BUF[SCI_RX_BUF_MAX];
Uint16 SCIA_TX_COUNT_H = 0;
Uint16 SCIA_TX_COUNT_T = 0;
Uint16 SCIA_RX_COUNT = 0;
Uint16 SCIA_RX_NEW = FALSE;


BYTE SCIB_TX_BUF[SCI_TX_BUF_MAX];
//BYTE SCIB_RX_BUF[SCI_RX_BUF_MAX];
Uint16 SCIB_TX_COUNT_H = 0;
Uint16 SCIB_TX_COUNT_T = 0;
//Uint16 SCIB_RX_COUNT = 0;

BYTE SCIC_TX_BUF[SCI_TX_BUF_MAX];
BYTE SCIC_RX_BUF[SCI_RX_BUF_MAX];
Uint16 SCIC_TX_COUNT_H = 0;
Uint16 SCIC_TX_COUNT_T = 0;
Uint16 SCIC_RX_COUNT = 0;
Uint16 SCIC_RX_NEW = FALSE;

BYTE Period250ms = FALSE;
BYTE Period10ms  = FALSE;

BYTE IsSendOutData = FALSE;

static Uint16 timer0_count = 0;

BYTE   scia_print_buf[60];
Uint16 scia_print_count = 0;

/***************************************************/
void InitXintf16Gpio();
void init_zone7(void);

// functions of on-chip devices
void scia_init(Uint32 baudrate);
void scia_fifo_init(void);
void scia_xmit(unsigned char a);
void scia_xmit_ex(unsigned char a);
void scia_start_tx();
void scia_print(const char* str, int n, ...);

void scib_init(Uint32 baudrate);
void scib_fifo_init(void);
void scib_xmit(unsigned char a);
void scib_xmit_ex(unsigned char a);
void scib_start_tx();
//void scib_print(const char* str, int n, ...);

void scic_init(Uint32 baudrate);
void scic_fifo_init(void);
void scic_xmit(unsigned char a);
void scic_xmit_ex(unsigned char a);
void scic_start_tx();
//void scic_print(const char* str, int n, ...);

void SPIA_init(void);
void SPIA_Mode(Uint16 mode);
Uint16 SPIA_xmit(Uint16 data);

Uint32 ADS8341_Read(Uint16 ch);

void init_INS(void);

interrupt void cpu_timer0_isr(void);
interrupt void cpu_timer1_isr(void);
interrupt void cpu_timer2_isr(void);

interrupt void scia_rx_isr(void);
interrupt void scia_tx_isr(void);

interrupt void scib_rx_isr(void);
interrupt void scib_tx_isr(void);

interrupt void scic_rx_isr(void);
interrupt void scic_tx_isr(void);

//========================================================================

interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;

	timer0_count = (timer0_count + 1)%100;
	
	if (timer0_count == 0)
	{
		// every 1s
		GPIO53_TOGGLE();
	}
	
	if ((timer0_count%25) == 0)
	{
		// every 250ms
		Period250ms = TRUE;
	}

	// every 10ms
	Period10ms = TRUE;
	
	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void cpu_timer1_isr(void)
{
	EALLOW;
	CpuTimer1.InterruptCount++;
	//IsSenddata = 1;
	//The CPU acknowledges the interrupt.
	EDIS;
}

interrupt void cpu_timer2_isr(void)
{
	EALLOW;
	CpuTimer2.InterruptCount++;
	// The CPU acknowledges the interrupt.
	EDIS;
}

//=======================================================================

/*
	Init GPIOs for 16-bit width XINTF
*/
void InitXintf16Gpio()
{
     EALLOW;
     GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;  // XD15
     GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;  // XD14
     GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;  // XD13
     GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3;  // XD12
     GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3;  // XD11
     GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3;  // XD10
     GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3;  // XD19
     GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3;  // XD8
     GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3;  // XD7
     GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;  // XD6
     GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3;  // XD5
     GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3;  // XD4
     GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3;  // XD3
     GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3;  // XD2
     GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3;  // XD1
     GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 3;  // XD0

     GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 3;  // XA0/XWE1n
     GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 3;  // XA1
     GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;  // XA2
     GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;  // XA3
     GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 3;  // XA4
     GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 3;  // XA5
     GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 3;  // XA6
     GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 3;  // XA7

     GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 3;  // XA8
     GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 3;  // XA9
     GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 3;  // XA10
     GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 3;  // XA11
     GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 3;  // XA12
     GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 3;  // XA13
     GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 3;  // XA14
     GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 3;  // XA15
     GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 3;  // XA16
     GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;  // XA17
     GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;  // XA18
     GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // XA19

     GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 3;  // XREADY
	 GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 3;  // XRNW
     GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 3;  // XWE0

     GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 3;  // XZCS0
     GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;  // XZCS7
     GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // XZCS6
     EDIS;
}
// Configure the timing paramaters for Zone 7.
// Notes: 
//    This function should not be executed from XINTF
//    Adjust the timing based on the data manual and
//    external device requirements.     
void init_zone7(void)
{
	EALLOW;
    // Make sure the XINTF clock is enabled
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;
	EDIS; 
	
	// Configure the GPIO for XINTF with a 16-bit data bus
	// This function is in DSP2833x_Xintf.c
	InitXintf16Gpio();
	
    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = SYSCLKOUT 
	EALLOW;
    XintfRegs.XINTCNF2.bit.XTIMCLK = 0;
    // Buffer up to 3 writes
    XintfRegs.XINTCNF2.bit.WRBUFF = 3;
    // XCLKOUT is enabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;
    // XCLKOUT = XTIMCLK 
    XintfRegs.XINTCNF2.bit.CLKMODE = 0;   
	
    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING7.bit.XWRLEAD = 1;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 2;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 1;
    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 1;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 3;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 0;
    
    // don't double all Zone read/write lead/active/trail timing 
    XintfRegs.XTIMING7.bit.X2TIMING = 0;
	
    // Zone will not sample XREADY signal  
    XintfRegs.XTIMING7.bit.USEREADY = 0;
    XintfRegs.XTIMING7.bit.READYMODE = 0;
	
    // 1,1 = x16 data bus
    // 0,1 = x32 data bus
    // other values are reserved
    XintfRegs.XTIMING7.bit.XSIZE = 3;
    EDIS; 
	//Force a pipeline flush to ensure that the write to 
	//the last register configured occurs before returning.  
	asm(" RPT #7 || NOP"); 
}

//=======================================================================

// Test 1,SCIC  DLB, 8-bit word, baud rate 9.6K, default, 1 STOP bit, no parity
void scic_init(Uint32 baudrate)
{
    // Note: Clocks were turned on to the SCIC peripheral
    // in the InitSysCtrl() function

	// init GPIOs for SCI-C
	EALLOW;
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;	   // Enable pull-up for GPIO62 (SCIRXDC)
	   
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)
	   
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	EDIS;	  
	
    scic_fifo_init();

	// Buad Rate BBR = LSPCLK(37.5MHz)/(BuadRate*8) - 1

	baudrate = 37500000/(baudrate*8) - 1;

	// 1 stop bit,  No loopback
	// No parity,8 char bits,
	ScicRegs.SCICCR.all =0x0007;
	// async mode, idle-line protocol
	// enable TX, RX, internal SCICLK,
	ScicRegs.SCICTL1.all =0x0003;  
	
	// Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003;

    ScicRegs.SCILBAUD    = baudrate & 0xff;
    ScicRegs.SCIHBAUD    = (baudrate>>8) & 0xff;
	
	ScicRegs.SCICCR.bit.LOOPBKENA =0; // Disable loop back

	// enable receive interrupt
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;
	// enable transmit interrupt
	ScicRegs.SCICTL2.bit.TXINTENA = 1;
	// Relinquish SCI from Reset
	ScicRegs.SCICTL1.bit.SWRESET = 1;
	
}

void scic_start_tx()
{
	Uint16 tmp;

	if (ScicRegs.SCICTL2.bit.TXEMPTY)
	{
		if (SCIC_TX_COUNT_H != SCIC_TX_COUNT_T)
		{
			tmp = SCIC_TX_COUNT_T;
			SCIC_TX_COUNT_T = (SCIC_TX_COUNT_T+1)%SCI_TX_BUF_MAX;
			ScicRegs.SCITXBUF=SCIC_TX_BUF[tmp];
		}
	}
}

// Transmit a character from the SCI'
void scic_xmit(unsigned char a)
{
	CLOSE_SCIC_TX;

	if ((SCIC_TX_COUNT_H+1)%SCI_TX_BUF_MAX != SCIC_TX_COUNT_T)
	{
		// the tx of sciC buffer not full
		SCIC_TX_BUF[SCIC_TX_COUNT_H] = a;
		SCIC_TX_COUNT_H = (SCIC_TX_COUNT_H+1)%SCI_TX_BUF_MAX;
	}
	OPEN_SCIC_TX;
}

// Transmit a character from the SCI'
void scic_xmit_ex(unsigned char a)
{
	ScicRegs.SCITXBUF=a;
}

// Initalize the SCI FIFO
void scic_fifo_init()
{
	// reset FIFO and hold in reset
    ScicRegs.SCIFFTX.bit.TXFIFOXRESET=0;
    ScicRegs.SCIFFRX.bit.RXFIFORESET=0;

	// enable FIFO
	ScicRegs.SCIFFTX.bit.SCIFFENA = 0;
	// enable FIFO transmit interrupt
	ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
	// set FIFO tx_int level is empty FIFO
	ScicRegs.SCIFFTX.bit.TXFFIL = 0;

	// enable FIFO receive interrupt
	ScicRegs.SCIFFRX.bit.RXFFIENA = 0;
	// set FIFO rx_int level to 1 byte
	ScicRegs.SCIFFRX.bit.RXFFIL = 1;

	// set SCIFFCT register
    ScicRegs.SCIFFCT.all=0x0;

	// set FIFO to enable
    ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    ScicRegs.SCIFFRX.bit.RXFIFORESET=1;	

	ScicRegs.SCIFFTX.bit.SCIRST = 1;
	ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;

	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
}

/*
// Transmit a string from the SCI-C
void scic_print(const char* str, int n, ...)
{
    va_list args;
    char i=0;
	int32  r;
	char t[20];
	char tp = 0;
	
	while(str[i])
	{
		scic_xmit(str[i]);
		//while(ScicRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);		
		i++;
	}
	
    // retrieve the variable arguments
    va_start(args, n);
	
	for (i=0; i<n; i++)
	{
		r = va_arg(args, int32);
		
		scic_xmit(',');
		//while(ScicRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
		
		if (r<0)
		{
			scic_xmit('-');
			//while(ScicRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
			r = -r;
		}
		if (r==0)
		{
			scic_xmit('0');
			//while(ScicRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
		}
		
		tp = 0;
		while (r>0)
		{
			t[tp] = ((r%10) + '0');
			tp++;
			r = r/10;
		}
		while (tp>0)
		{
			scic_xmit(t[tp-1]);
			//while(ScicRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
			tp--;	
		}
	}
	
	va_end(args);
	scic_xmit('\n');
	//while(ScicRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
	
}
*/

//=======================================================================

// Test 1,SCIB  DLB, 8-bit word, baud rate 9.6K, default, 1 STOP bit, no parity
void scib_init(Uint32 baudrate)
{
    // Note: Clocks were turned on to the SCIB peripheral
    // in the InitSysCtrl() function

	// init GPIOs for SCI-B
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;	   // Enable pull-up for GPIO9 (SCITXDB)
	GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;	   // Enable pull-up for GPIO11 (SCIRXDB)
	   
	GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3;  // Asynch input GPIO11 (SCIRXDB)
	   
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;   // Configure GPIO9 for SCITXDB operation
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;   // Configure GPIO11 for SCIRXDB operation
	EDIS;	
	
    scib_fifo_init();

	// Buad Rate BBR = LSPCLK(37.5MHz)/(BuadRate*8) - 1
	baudrate = 37500000/(baudrate*8) - 1;
	
	// 1 stop bit,  No loopback
	// No parity,8 char bits,
	ScibRegs.SCICCR.all =0x0007;
	// async mode, idle-line protocol
	// enable TX, RX, internal SCICLK,
	ScibRegs.SCICTL1.all =0x0003;  
	
	// Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003;

	ScibRegs.SCILBAUD    = baudrate & 0xff;
	ScibRegs.SCIHBAUD    = (baudrate>>8) & 0xff;
	
	ScibRegs.SCICCR.bit.LOOPBKENA =0; // Disable loop back

	// enable receive interrupt
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;
	// enable transmit interrupt
	ScibRegs.SCICTL2.bit.TXINTENA = 1;
	// Relinquish SCI from Reset
	ScibRegs.SCICTL1.bit.SWRESET = 1;
}

void scib_start_tx()
{
	Uint16 tmp;

	if (ScibRegs.SCICTL2.bit.TXEMPTY)
	{
		if (SCIB_TX_COUNT_H != SCIB_TX_COUNT_T)
		{
			tmp = SCIB_TX_COUNT_T;
			SCIB_TX_COUNT_T = (SCIB_TX_COUNT_T+1)%SCI_TX_BUF_MAX;
			ScibRegs.SCITXBUF=SCIB_TX_BUF[tmp];
		}
	}	
}

// Transmit a character from the SCI'
void scib_xmit(unsigned char a)
{
	CLOSE_SCIB_TX;
	
	if ((SCIB_TX_COUNT_H+1)%SCI_TX_BUF_MAX != SCIB_TX_COUNT_T)
	{
		// the tx of sciB buffer not full
		SCIB_TX_BUF[SCIB_TX_COUNT_H] = a;
		SCIB_TX_COUNT_H = (SCIB_TX_COUNT_H+1)%SCI_TX_BUF_MAX;
	}
	
	OPEN_SCIB_TX;
}

// Transmit a character from the SCI'
void scib_xmit_ex(unsigned char a)
{
    ScibRegs.SCITXBUF=a;
}

// Initalize the SCI FIFO
void scib_fifo_init()
{
	// reset FIFO and hold in reset
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET=0;
    ScibRegs.SCIFFRX.bit.RXFIFORESET=0;

	// enable FIFO
	ScibRegs.SCIFFTX.bit.SCIFFENA = 0;
	// enable FIFO transmit interrupt
	ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
	// set FIFO tx_int level is empty FIFO
	ScibRegs.SCIFFTX.bit.TXFFIL = 0;

	// enable FIFO receive interrupt
	ScibRegs.SCIFFRX.bit.RXFFIENA = 0;
	// set FIFO rx_int level to 1 byte
	ScibRegs.SCIFFRX.bit.RXFFIL = 1;

	// set SCIFFCT register
    ScibRegs.SCIFFCT.all=0x0;

	// set FIFO to enable
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET=1;	

	ScibRegs.SCIFFTX.bit.SCIRST = 1;
	ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;

	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
}

/*
// Transmit a string from the SCI-B
void scib_print(const char* str, int n, ...)
{
    va_list args;
    char i=0;
	int32  r;
	char t[20];
	char tp = 0;
	
	while(str[i])
	{
		scib_xmit(str[i]);
		//while(ScibRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);		
		i++;
	}
	
    // retrieve the variable arguments
    va_start(args, n);
	
	for (i=0; i<n; i++)
	{
		r = va_arg(args, int32);
		
		scib_xmit(',');
		//while(ScibRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
		
		if (r<0)
		{
			scib_xmit('-');
			//while(ScibRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
			r = -r;
		}
		if (r==0)
		{
			scib_xmit('0');
			//while(ScibRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
		}
		
		tp = 0;
		while (r>0)
		{
			t[tp] = ((r%10) + '0');
			tp++;
			r = r/10;
		}
		while (tp>0)
		{
			scib_xmit(t[tp-1]);
			//while(ScibRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
			tp--;	
		}
	}
	
	va_end(args);
	scib_xmit('\n');
	//while(ScibRegs.SCIFFTX.bit.TXFFST > SCI_TX_FIFO_MAX);
	
}
*/

//===========================================================================
//===========================================================================

// Test 1,SCIA  DLB, 8-bit word, baud rate 9.6K, default, 1 STOP bit, no parity
void scia_init(Uint32 baudrate)
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    
	// config the SCIA GPIOs
	EALLOW;
	GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;	   // Enable pull-up for GPIO35 (SCITXDA)
	GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;	   // Enable pull-up for GPIO36 (SCIRXDA)
	   
	GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 3;  // Asynch input GPIO36 (SCIRXDA)
	   
	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 1;   // Configure GPIO35 for SCITXDA operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 1;   // Configure GPIO36 for SCIRXDA operation
	EDIS;	   

	scia_fifo_init();

	// Buad Rate BBR = LSPCLK(37.5MHz)/(BuadRate*8) - 1
	baudrate = 37500000/(baudrate*8) - 1;
	   
	// 1 stop bit,  No loopback
	// No parity,8 char bits,
	SciaRegs.SCICCR.all =0x0007;
	// async mode, idle-line protocol
	// enable TX, RX, internal SCICLK,
	SciaRegs.SCICTL1.all =0x0003;  
	
	// Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003;
	
	SciaRegs.SCILBAUD    = baudrate & 0xff;
	SciaRegs.SCIHBAUD    = (baudrate>>8) & 0xff;
	
	SciaRegs.SCICCR.bit.LOOPBKENA =0; // Disable loop back

	// enable receive interrupt
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
	// enable transmit interrupt
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	// Relinquish SCI from Reset
	SciaRegs.SCICTL1.bit.SWRESET = 1;
}

void scia_start_tx()
{
	Uint16 tmp;
	// wait for TXRDY
	if (SciaRegs.SCICTL2.bit.TXEMPTY)
	{
		if (SCIA_TX_COUNT_H != SCIA_TX_COUNT_T)
		{
			tmp = SCIA_TX_COUNT_T;
			SCIA_TX_COUNT_T = (SCIA_TX_COUNT_T+1)%SCI_TX_BUF_MAX;
			SciaRegs.SCITXBUF = SCIA_TX_BUF[tmp];
		}
	}
}

// Transmit a character from the SCI'
void scia_xmit(unsigned char a)
{
	CLOSE_SCIA_TX;
	
	if ((SCIA_TX_COUNT_H+1)%SCI_TX_BUF_MAX != SCIA_TX_COUNT_T)
	{
		// the tx of sciA buffer not full
		SCIA_TX_BUF[SCIA_TX_COUNT_H] = a;
		SCIA_TX_COUNT_H = (SCIA_TX_COUNT_H+1)%SCI_TX_BUF_MAX;
	}

    OPEN_SCIA_TX;
}

// Transmit a character from the SCI'
void scia_xmit_ex(unsigned char a)
{
    SciaRegs.SCITXBUF=a;
}

// Initalize the SCI FIFO
void scia_fifo_init()
{
	// reset FIFO and hold in reset
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=0;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=0;

	// enable FIFO
	SciaRegs.SCIFFTX.bit.SCIFFENA = 0;
	// enable FIFO transmit interrupt
	SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
	// set FIFO tx_int level is empty FIFO
	SciaRegs.SCIFFTX.bit.TXFFIL = 0;

	// enable FIFO receive interrupt
	SciaRegs.SCIFFRX.bit.RXFFIENA = 0;
	// set FIFO rx_int level to 1 byte
	SciaRegs.SCIFFRX.bit.RXFFIL = 1;

	// set SCIFFCT register
    SciaRegs.SCIFFCT.all=0x0;

	// set FIFO to enable
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;	

	SciaRegs.SCIFFTX.bit.SCIRST = 1;
	SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;

	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
}

// Transmit a string from the SCI-A
void scia_print(const char* str, int n, ...)
{
    va_list args;
    char i=0;
	int32  r;
	char t[20];
	char tp = 0;

	scia_print_count = 0;
	
	while(str[i])
	{
		scia_print_buf[scia_print_count++] = str[i];
		i++;
	}
	
    // retrieve the variable arguments
    va_start(args, n);
	
	for (i=0; i<n; i++)
	{
		r = va_arg(args, int32);

		scia_print_buf[scia_print_count++] = ',';
		
		if (r<0)
		{
			scia_print_buf[scia_print_count++] = '-';
			r = -r;
		}
		if (r==0)
		{
			scia_print_buf[scia_print_count++] = '0';
		}
		
		tp = 0;
		while (r>0)
		{
			t[tp] = ((r%10) + '0');
			tp++;
			r = r/10;
		}
		while (tp>0)
		{
			scia_print_buf[scia_print_count++] = t[tp-1];
			tp--;	
		}
	}
	
	va_end(args);

	scia_print_buf[scia_print_count++] = '\r';
	scia_print_buf[scia_print_count++] = '\n';
	scia_print_buf[scia_print_count++] = 0;

	for (i=0; i<scia_print_count; i=i+24)
	{
		SCIA_ClearSendFrame();
		m_pSendBuf[4] = 0xFE;
		memmove(&(m_pSendBuf[5]), scia_print_buf+i, 24);
		SCIA_SendFrame();
	}
	scia_start_tx();	
	
}

//===========================================================================
// No more.
//===========================================================================

void SPIA_init(void)
{

	// config GPIOs for SPI 
	EALLOW;
	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;	   // Enable pull-up for GPIO54 (SPIA)
	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;	   // Enable pull-up for GPIO55 (SPIA)
	GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;	   // Enable pull-up for GPIO56 (SPIA)
	
	GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 0;  // Asynch input GPIO55 (SPIA)
	
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;   // Configure GPIO54 for SPIA operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1;   // Configure GPIO55 for SPIA operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1;   // Configure GPIO56 for SPIA operation
	EDIS;

	// for ADS8341   : CLKPOLARITY = 0, CLK_PHASE = 1
	// for ADIS16355 : CLKPOLARITY = 1, CLK_PHASE = 0 

	//Bit 7, Reset = 0
	SpiaRegs.SPICCR.bit.SPISWRESET = 0;
	//Bit 6, polar of SCLK = 0
	SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
	//Bit 5, reserved
	//Bit 4, SPILBK = 0, SPI loopback Disabled
	SpiaRegs.SPICCR.bit.SPILBK = 0;
	//Bit 3~0, chars = 0111, 8bit word
	SpiaRegs.SPICCR.bit.SPICHAR = 0x07;
	
	//Bit 7~5, reserved
	//Bit 4, overrun INT Enable = 0
	SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0;
	//Bit 3, Clock-Phase = 1, half clock delay
	SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
	//Bit 2, Master/Slave = 1, Master = 1, Slave = 0
	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
	//Bit 1, Talk = 1, Output Enable
	SpiaRegs.SPICTL.bit.TALK = 1;
	//Bit 0, SPI INT Enable = 0
	SpiaRegs.SPICTL.bit.SPIINTENA = 0;
	
	//SPI BuadRate = LSPCLK/(SPIBBR+1)
	//             = 37.5MHz/(24+1)
	//             = 1.5MHz
	SpiaRegs.SPIBRR = 24;
	
	SpiaRegs.SPICCR.bit.SPISWRESET = 1;
}

void SPIA_Mode(Uint16 mode)
{
	// for ADS8341   : CLKPOLARITY = 0, CLK_PHASE = 1
	// for ADIS16355 : CLKPOLARITY = 1, CLK_PHASE = 0 

	if (mode == 0)
	{
		// for ADS8341 
		
		//Bit 7, Reset = 0
		SpiaRegs.SPICCR.bit.SPISWRESET = 0;
		//Bit 6, polar of SCLK = 0
		SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
		//Bit 3, Clock-Phase = 1, half clock delay
		SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
		//Bit 7, Reset = 1		
		SpiaRegs.SPICCR.bit.SPISWRESET = 1;
	}
	else if (mode == 1)
	{
		// for ADIS16355 
		
		//Bit 7, Reset = 0
		SpiaRegs.SPICCR.bit.SPISWRESET = 0;
		//Bit 6, polar of SCLK = 0
		SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;
		//Bit 3, Clock-Phase = 1, half clock delay
		SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
		//Bit 7, Reset = 1
		SpiaRegs.SPICCR.bit.SPISWRESET = 1;		
	}

}

Uint16 SPIA_xmit(Uint16 data)
{
	Uint32 result = 0;
	Uint32 i = 0;
	
	SpiaRegs.SPIDAT = data<<8;
	
	while (SpiaRegs.SPISTS.bit.INT_FLAG == 0 && i<1000)
	{
		i++;
	}
	
	result = SpiaRegs.SPIRXBUF;
	
	//if (i >= 100)
	//	result = result | 0xffff0000;
	
	return (result & 0xff);
}

Uint32 ADS8341_Read(Uint16 ch)
{
	Uint32 result = 0;
	Uint16 i = 0;
	Uint16 data = 0x87;

	SPIA_Mode(0);
	
	if (ch == 0)
	{
		data = data | (0x01<<4);
	}
	else if (ch == 1)
	{
		data = data | (0x05<<4);
	}
	else if (ch == 2)
	{
		data = data | (0x02<<4);
	}
	else if (ch == 3)
	{
		data = data | (0x06<<4);
	}
	
	CS_MAGNET();
	
	SPIA_xmit(data);
	
	for (i=0;i<5;i++)
		;
	
	result = SPIA_xmit(0x00);
	result = result << 8;
	result = result | SPIA_xmit(0x00);
	
	CS_NONE();
	
	result = (result << 1) & 0xffff;

	// reverse result for Magnet
	if (ch<3)
		result = 0xffff - result;
		
	return result;
}

void init_INS(void)
{
	
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the DSP2833x_SysCtrl.c file.
	InitSysCtrl();
	
	// Init for XINTF Zone7
	init_zone7();
	
	//  init GPIO53 for LED
	//  init GPIO52.51.50
	//           A2.A1.A0
	//  Y0 = IMU      GPIO50_CLEAR();GPIO51_CLEAR();GPIO52_CLEAR();
	//  Y1 = Magnet   GPIO50_SET();GPIO51_CLEAR();GPIO52_CLEAR();
	//  select none   GPIO50_SET();GPIO51_SET();GPIO52_SET();
	EALLOW;
	GPIO53_PUD = 1; //Disable Pullup
	GPIO53_DIR = 1; //output
	GPIO52_PUD = 1; //Disable Pullup
	GPIO52_DIR = 1; //output
	GPIO51_PUD = 1; //Disable Pullup
	GPIO51_DIR = 1; //output
	GPIO50_PUD = 1; //Disable Pullup
	GPIO50_DIR = 1; //output

	// init GPIO1 for SR of Magnet sensors
	GPIO1_PUD = 0; //Enable Pullup
	GPIO1_DIR = 1; //output
	EDIS;
	
	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;
	
	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP2833x_PieCtrl.c file.
	InitPieCtrl();
	
	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;
	
	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
	// This function is found in DSP2833x_PieVect.c.
	InitPieVectTable();
	
	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.TINT0  = &cpu_timer0_isr;
	PieVectTable.XINT13 = &cpu_timer1_isr;
	PieVectTable.TINT2  = &cpu_timer2_isr;

	PieVectTable.SCIRXINTA = &scia_rx_isr;
	PieVectTable.SCITXINTA = &scia_tx_isr;
	PieVectTable.SCIRXINTB = &scib_rx_isr;
	PieVectTable.SCITXINTB = &scib_tx_isr;
	PieVectTable.SCIRXINTC = &scic_rx_isr;
	PieVectTable.SCITXINTC = &scic_tx_isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers
	
	// Copy time critical code and Flash setup code to RAM
	// This includes the following ISR functions: epwm1_timer_isr(), epwm2_timer_isr()
	// epwm3_timer_isr and and InitFlash();
	// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the F28335.cmd file.
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//	MemCopy(&XintfLoadfuncsLoadStart, &XintfLoadfuncsLoadEnd, &XintfLoadfuncsRunStart);
	
	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM
//	InitFlash();
		
	// Step 4. Initialize the Device Peripheral. This function can be
	//         found in DSP2833x_CpuTimers.c
	InitCpuTimers();   // For this example, only initialize the Cpu Timers
	
	#if (CPU_FRQ_150MHZ)
		// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
		// 150MHz CPU Freq, 1 second Period (in uSeconds)
		
		ConfigCpuTimer(&CpuTimer0, 150, 10000);
		ConfigCpuTimer(&CpuTimer1, 150, 1000);
		ConfigCpuTimer(&CpuTimer2, 150, 1000000);
	#endif
	
	#if (CPU_FRQ_100MHZ)
		// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
		// 100MHz CPU Freq, 1 second Period (in uSeconds)
		
		ConfigCpuTimer(&CpuTimer0, 100, 10000);
		ConfigCpuTimer(&CpuTimer1, 100, 1000);
		ConfigCpuTimer(&CpuTimer2, 100, 1000000);
	#endif

	SPIA_init();		// Initalize SPI-A
	scia_init(115200);	// Initalize SCI-A for PC
	scib_init(115200);	// Initalize SCI-B for Embedded Device
	scic_init(9600);	// Initalize SCI-C for u-blox5 GPS
	init_mcbspa_spi();	// Initalize McBSP-A as SPI mode for Flash
	init_mcbspb_spi();	// Initalize McBSP-B as SPI mode for Barometer	
	
	// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
	// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
	// below settings must also be updated.
	
	//   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	//   CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	//   CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	
	// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
	// which is connected to CPU-Timer 1, and CPU int 14, which is connected
	// to CPU-Timer 2:
	IER |= M_INT1;
	IER |= M_INT13;
	IER |= M_INT14;

	// Enable CPU int8 and int9 which are connected to SCIs	
	IER |= M_INT8;
	IER |= M_INT9;

	// Enable TINT0 in the PIE: Group 1 interrupt 7
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	// Enable SCIRXINTA in the PIE: Group 9 interrupt 1
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
	// Enable SCITXINTA in the PIE: Group 9 interrupt 2
	PieCtrlRegs.PIEIER9.bit.INTx2 = 1;

	// Enable SCIRXINTB in the PIE: Group 9 interrupt 3
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
	// Enable SCITXINTB in the PIE: Group 9 interrupt 4
	PieCtrlRegs.PIEIER9.bit.INTx4 = 1;

	// Enable SCIRXINTC in the PIE: Group 8 interrupt 5
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;
	// Enable SCITXINTC in the PIE: Group 8 interrupt 6
	PieCtrlRegs.PIEIER8.bit.INTx6 = 1;
	
	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	PageModeSwitch();	// Switch Flash Page Mode
	CS_NONE();

	// Init for ADC
	//EALLOW;
	//SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
	//ADC_cal();
	//SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0;
	//EDIS;

	// begin timer0
	CpuTimer0Regs.TCR.bit.TSS = 0;
	// begin timer1
	CpuTimer1Regs.TCR.bit.TSS = 0;
	
}

/******************************************************/

interrupt void scia_tx_isr(void)
{
	//int i;
	// when the TX is ready
	if (SciaRegs.SCICTL2.bit.TXRDY
	&& SCIA_TX_COUNT_H != SCIA_TX_COUNT_T)
	{
		// the buffer is not empty
		// send out 1 byte
		SciaRegs.SCITXBUF = SCIA_TX_BUF[SCIA_TX_COUNT_T];
		SCIA_TX_COUNT_T = (SCIA_TX_COUNT_T+1)%SCI_TX_BUF_MAX;
	}
	// Acknowledge this interrupt to receive more interrupts from group 9
	// SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}

interrupt void scib_tx_isr(void)
{
	//int i;
	// when the TX is ready
	if ((SCIB_TX_COUNT_H != SCIB_TX_COUNT_T)
		&& ScibRegs.SCICTL2.bit.TXRDY)
	{
		// the buffer is not empty
		// send out no more than 16 bytes
		ScibRegs.SCITXBUF=SCIB_TX_BUF[SCIB_TX_COUNT_T];
		SCIB_TX_COUNT_T = (SCIB_TX_COUNT_T+1)%SCI_TX_BUF_MAX;
	}
	
	// Acknowledge this interrupt to receive more interrupts from group 9
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}

interrupt void scic_tx_isr(void)
{
	//int i;
	// when the TX is ready
	if ((SCIC_TX_COUNT_H != SCIC_TX_COUNT_T)
		&& ScicRegs.SCICTL2.bit.TXRDY)
	{
		// the buffer is not empty
		// send out no more than 16 bytes
		ScicRegs.SCITXBUF=SCIC_TX_BUF[SCIC_TX_COUNT_T];
		SCIC_TX_COUNT_T = (SCIC_TX_COUNT_T+1)%SCI_TX_BUF_MAX;
	}

	// Acknowledge this interrupt to receive more interrupts from group 8
	PieCtrlRegs.PIEACK.bit.ACK8 = 1;
}

/***************************************************/

interrupt void scia_rx_isr(void)
{
	// receive PC data in a frame format
	unsigned char t;

	if (SciaRegs.SCIRXST.bit.RXRDY)
	{
		t = SciaRegs.SCIRXBUF.bit.RXDT;
		if (IsDirectGPS)
		{
			scic_xmit_ex(t);
		}
		else
		{
			if (SCIA_RX_COUNT < SCI_RX_BUF_MAX)
			{
				SCIA_RX_BUF[SCIA_RX_COUNT++] = t;
				SCIA_RX_NEW = TRUE;
			}
		}
	}

	// Acknowledge this interrupt to receive more interrupts from group 9
    //SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    //SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}

interrupt void scib_rx_isr(void)
{
	// receive data from embedded devices 
	// discard all data
	Uint16 t;

	while (ScibRegs.SCIRXST.bit.RXRDY)
	{
		t = ScibRegs.SCIRXBUF.all;
	}

	// Acknowledge this interrupt to receive more interrupts from group 9
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;	
	
}

interrupt void scic_rx_isr(void)
{
	// receive data from u-blox5 GPS in UBX protocol
	unsigned char t;

	while (ScicRegs.SCIRXST.bit.RXRDY)
	{
		t = ScicRegs.SCIRXBUF.bit.RXDT;
		if (IsDirectGPS)
		{
			scia_xmit_ex(t);
		}	
		else
		{
			if (SCIC_RX_COUNT < SCI_RX_BUF_MAX)
			{
				SCIC_RX_BUF[SCIC_RX_COUNT++] = t;
				SCIC_RX_NEW = TRUE;
			}
		}
	}

	// Acknowledge this interrupt to receive more interrupts from group 8
	PieCtrlRegs.PIEACK.bit.ACK8 = 1;	
}


