/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "SCI_C.h"
#include "GPS.h"
#include "INS_Init.h"

#include <string.h>

#pragma CODE_SECTION(SCIC_SendAllData, "xintfloadfuncs");

SENDDATA_FRAME INS_SendData = {0xA5,0x5A,52,0xF1,0,
								0,0,0,
								0,0,0,
								0,0,0,
								0,0,0,
								0,0,
								0,0,0,
								0,0,0,
								0,0,0,
								0};

void SCIC_SendAllData()
{
	int i;
	BYTE* p;
	Uint16 sum = 0;

	p = (BYTE*)&INS_SendData;
	
	INS_SendData.S1 = 0xA5;
	INS_SendData.S2 = 0x5A;
	INS_SendData.Length = 52;
	INS_SendData.Command = 0xF1;
		
	for (i=0; i<(sizeof(SENDDATA_FRAME)-1); i++)
	{
		scib_xmit(p[i]);
		sum += p[i];
	}

	sum -= INS_SendData.S1;
	sum -= INS_SendData.S2;
	scib_xmit(sum&0xff);
	
	scib_start_tx();
}

