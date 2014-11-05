/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com)
*/

/*
	GPS data process
	For u-blox5 UBX protocol
	By VrilleUp
	2008.08.12 One World, One Dream!
*/
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#include <string.h>
#include "GPS.h"

#pragma CODE_SECTION(UBX_Addbytes, "xintfloadfuncs");
#pragma CODE_SECTION(FrameProcess, "xintfloadfuncs");

static unsigned char Buf[502];
static int ptrBuf = 0;
BOOL FrameProcess();

BOOL IsDirectGPS = FALSE;

//GPS_DATA UBXdata;
GPS_DATA UBXdata = {0,0, 8,8,8, 20,8,0, 0,0,0, 0.0,0.0,0.0,100.0,0.0, 0.0, 0, 0};

BOOL UBX_Addbytes(unsigned char* s, int len)
{
	BOOL result = FALSE;
	int i;

	for (i=0; i<len; i++)
	{
		if (ptrBuf == 0)
		{
			if (s[i] == 0xb5)
			{
				//start byte 1 of a frame 
				Buf[ptrBuf] = s[i];
				ptrBuf++;
			}
		}
		else if (ptrBuf == 1)
		{
			if (s[i] == 0x62)
			{
				//start byte 2 of a frame
				Buf[ptrBuf] = s[i];
				ptrBuf++;
			}
			else
			{
				ptrBuf = 0;
			}
		}
		else
		{
			//contents of the frame
			Buf[ptrBuf] = s[i];
			ptrBuf++;

			if (ptrBuf>6)
			{
				// there is a length information
				if ( ptrBuf>=(8+Buf[4]+((Buf[5])<<8)) )
				{
					// end of a frame
					//process the frame
					result = FrameProcess();
					//clean Buf
					ptrBuf = 0;
				}
			}
		}
		if (ptrBuf>=500)
			ptrBuf = 0;
	}

	return result;
}

BOOL FrameProcess()
{
	unsigned char CK_A;
	unsigned char CK_B;
	signed long tmpl;
	unsigned long tmpu;
	unsigned short tmpsu;
	BOOL result;	
	int i;

	CK_A = 0;
	CK_B = 0;
	tmpl = 0;
	tmpu = 0;
	tmpsu = 0;
	result = FALSE;
	
	if (!((Buf[2] == 0x01 && Buf[3] == 0x06) || 
			(Buf[2] == 0x01 && Buf[3] == 0x02) ||
			(Buf[2] == 0x01 && Buf[3] == 0x12) ||
			(Buf[2] == 0x01 && Buf[3] == 0x21)))
			return result;

	for (i=2; i<(ptrBuf-2); i++)
	{
		CK_A = (CK_A + Buf[i]) & 0xff;
		CK_B = (CK_B + CK_A) & 0xff;
	}

	if (CK_A != Buf[ptrBuf-2] || CK_B != Buf[ptrBuf-1])
	{
		// check sum error
		// UBXdata.fixMode = 0;
		// UBXdata.IsNew = FALSE;
		return FALSE;
	}

	// poll commands
	// B5 62 01 06 00 00 07 16 B5 62 01 02 00 00 03 0A B5 62 01 12 00 00 13 3A B5 62 01 21 00 00 22 67
	// B5 62 01 06 00 00 07 16 B5 62 01 02 00 00 03 0A B5 62 01 12 00 00 13 3A B5 62 01 21 00 00 22 67
	if (Buf[2] == 0x01 && Buf[3] == 0x06)
	{
		// NAV-SOL
		UBXdata.fixMode = Buf[16]; // 3D fix = 0x03
		UBXdata.snum = Buf[53];
		result = TRUE;
	}
	else if (Buf[2] == 0x01 && Buf[3] == 0x02)
	{
		// NAV-POSLLH
		tmpl = BYTE2int32(Buf[13],Buf[12],Buf[11],Buf[10]);
		UBXdata.lo = tmpl;
		tmpl = BYTE2int32(Buf[17],Buf[16],Buf[15],Buf[14]);
		UBXdata.la = tmpl;
		tmpl = BYTE2int32(Buf[25],Buf[24],Buf[23],Buf[22]);
		UBXdata.al = tmpl;
		tmpl = BYTE2int32(Buf[29],Buf[28],Buf[27],Buf[26]);
		UBXdata.scep = tmpu;
		result = TRUE;
	}
	else if (Buf[2] == 0x01 && Buf[3] == 0x12)
	{
		// NAV-VELNED
		tmpl = BYTE2int32(Buf[13],Buf[12],Buf[11],Buf[10]);
		UBXdata.speedN = (float32)tmpl/100.0;
		tmpl = BYTE2int32(Buf[17],Buf[16],Buf[15],Buf[14]);
		UBXdata.speedE = (float32)tmpl/100.0;
		tmpl = BYTE2int32(Buf[21],Buf[20],Buf[19],Buf[18]);
		UBXdata.speedD = (float32)tmpl/100.0;
		tmpl = BYTE2int32(Buf[33],Buf[32],Buf[31],Buf[30]);
		UBXdata.yaw = (float32)tmpl*3.14159/18000000.0;
		tmpl = BYTE2int32(Buf[37],Buf[36],Buf[35],Buf[34]);
		UBXdata.speedAcc = (float32)tmpu/100.0;
		UBXdata.IsNew = TRUE;
		result = TRUE;
	}
	else if (Buf[2] == 0x01 && Buf[3] == 0x21)
	{
		// NAV-TIMEUTC
		tmpsu = BYTE2Uint16(Buf[19],Buf[18]);
		UBXdata.yy = tmpsu;
		UBXdata.mo = Buf[20];
		UBXdata.dd = Buf[21];
		UBXdata.hh = Buf[22];
		UBXdata.mm = Buf[23];
		UBXdata.ss = Buf[24];
		result = TRUE;
	}

	return result;
}

