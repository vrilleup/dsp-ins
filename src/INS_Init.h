/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com)
*/

//============================================
// Head file of INS Initialize Functions
// call init_INS() for all things
// 2008.08.07 by VrilleUp
//============================================

#ifndef INS_INIT_H
#define INS_INIT_H

#define SCI_TX_BUF_MAX  500
#define SCI_RX_BUF_MAX  500

//  Y0 = IMU      
#define CS_IMU()     GPIO50_CLEAR();GPIO51_CLEAR();GPIO52_CLEAR()
//  Y1 = Magnet   
#define CS_MAGNET()  GPIO50_SET();GPIO51_CLEAR();GPIO52_CLEAR()
//  select none   
#define CS_NONE()    GPIO50_SET();GPIO51_SET();GPIO52_SET()

#define OPEN_SCIA_RX 	SciaRegs.SCICTL2.bit.RXBKINTENA=1
#define OPEN_SCIB_RX 	ScibRegs.SCICTL2.bit.RXBKINTENA=1
#define OPEN_SCIC_RX 	ScicRegs.SCICTL2.bit.RXBKINTENA=1
#define CLOSE_SCIA_RX 	SciaRegs.SCICTL2.bit.RXBKINTENA=0
#define CLOSE_SCIB_RX 	ScibRegs.SCICTL2.bit.RXBKINTENA=0
#define CLOSE_SCIC_RX 	ScicRegs.SCICTL2.bit.RXBKINTENA=0

#define OPEN_SCIA_TX 	SciaRegs.SCICTL2.bit.TXINTENA=1
#define OPEN_SCIB_TX 	ScibRegs.SCICTL2.bit.TXINTENA=1
#define OPEN_SCIC_TX 	ScicRegs.SCICTL2.bit.TXINTENA=1
#define CLOSE_SCIA_TX 	SciaRegs.SCICTL2.bit.TXINTENA=0
#define CLOSE_SCIB_TX 	ScibRegs.SCICTL2.bit.TXINTENA=0
#define CLOSE_SCIC_TX 	ScicRegs.SCICTL2.bit.TXINTENA=0

#define RELOAD_TIMER0   CpuTimer0Regs.TCR.bit.TRB=1

extern void scia_init(Uint32 baudrate);
extern void scia_fifo_init(void);
extern void scia_xmit(unsigned char a);
extern void scia_xmit_ex(unsigned char a);
extern void scia_start_tx();
extern void scia_print(const char* str, int n, ...);

extern void scib_init(Uint32 baudrate);
extern void scib_fifo_init(void);
extern void scib_xmit(unsigned char a);
extern void scib_xmit_ex(unsigned char a);
extern void scib_start_tx();
//extern void scib_print(const char* str, int n, ...);

extern void scic_init(Uint32 baudrate);
extern void scic_fifo_init(void);
extern void scic_xmit(unsigned char a);
extern void scic_xmit_ex(unsigned char a);
extern void scic_start_tx();
//extern void scic_print(const char* str, int n, ...);

extern void SPIA_init(void);
extern void SPIA_Mode(Uint16 mode);
extern Uint16 SPIA_xmit(Uint16 data);

extern Uint32 ADS8341_Read(Uint16 ch);

extern void init_zone7(void);

extern void init_INS(void);


extern BYTE SCIA_TX_BUF[SCI_TX_BUF_MAX];
extern BYTE SCIA_RX_BUF[SCI_RX_BUF_MAX];
extern Uint16 SCIA_RX_COUNT;
extern Uint16 SCIA_RX_NEW;

extern BYTE SCIB_TX_BUF[SCI_TX_BUF_MAX];
//extern BYTE SCIB_RX_BUF[SCI_RX_BUF_MAX];

extern BYTE SCIC_TX_BUF[SCI_TX_BUF_MAX];
extern BYTE SCIC_RX_BUF[SCI_RX_BUF_MAX];
extern Uint16 SCIC_RX_COUNT;
extern Uint16 SCIC_RX_NEW;

extern BYTE Period250ms;
extern BYTE Period10ms;

extern BYTE IsSendOutData;

#endif
