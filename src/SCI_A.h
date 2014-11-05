/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#ifndef SCI_A_H
#define SCI_A_H


#define SCIA_FRAME_MAX 	50
#define m_nFrameSize 	30
#define m_S1 			0xA5
#define m_S2 			0x5A

extern BYTE m_pSendBuf[m_nFrameSize];

extern int  SCIA_AddBytes(unsigned char* p, Uint16 len);
extern void SCIA_ClearSendFrame();
extern void SCIA_SendFrame();
extern void SCIA_ProcessFrame();

extern BOOL IsResetEKF;

#endif
