/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com)
*/

/*
	INS miscellaneous functions
	By VrilleUp
	2008.08.12 One World, One Dream!
*/
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "INS_Func.h"
#include "INS_Init.h"
#include "McBSP_SPI.h"
#include "GPS.h"

#include "AT45DB642D.h"
#include "matrix.h"		// matrix include file
#include "EKFilter.h"
#include "SCI_A.h"
#include "SCI_C.h"

#include <string.h>
#include <math.h>

#pragma CODE_SECTION(GetMagnet, "xintfloadfuncs");
#pragma CODE_SECTION(ReadFlashPara, "xintfloadfuncs");
#pragma CODE_SECTION(WriteFlashPara, "xintfloadfuncs");
#pragma CODE_SECTION(ReadStatus, "xintfloadfuncs");
#pragma CODE_SECTION(WaitReady, "xintfloadfuncs");
#pragma CODE_SECTION(BufferWrite, "xintfloadfuncs");
#pragma CODE_SECTION(BufferRead, "xintfloadfuncs");
#pragma CODE_SECTION(PageWrite, "xintfloadfuncs");
#pragma CODE_SECTION(PageRead, "xintfloadfuncs");
#pragma CODE_SECTION(PageModeSwitch, "xintfloadfuncs");
#pragma CODE_SECTION(INS_SendData2All, "xintfloadfuncs");


#define OVERTIME		50000 // in ms
#define SSPSendByte(x)  mcbspa_spi_send8bit(x)

INS_PARAMETER INS_Para;
Uint16 Magnet_AD[3];
BYTE   IsMagnetCali = FALSE;

void GetMagnet(float32* p, Uint16 * MAD);
void ReadFlashPara();
void WriteFlashPara();
BYTE ReadStatus();
BYTE WaitReady();
BYTE BufferWrite(Uint16 Offset,BYTE *Data, Uint16 Len);
BYTE BufferRead(Uint16 Offset,BYTE *Data, Uint16 Len);
BYTE PageWrite(Uint16 PageNo);
BYTE PageRead(Uint16 PageNo);
//BYTE BlockErase(Uint16 BlockNo);
//BYTE PageErase(Uint16 PageNo);
BYTE PageModeSwitch();
void INS_SendData2All(float32* GAB, VEC* states, Uint16 * MAGAD, BOOL IsFilterOn);

void GetMagnet(float32* p, Uint16 * MAD)
{
	float32 rX,rY,rZ;
	float32 norm;

	// get magnet readings
	MAD[0] = ADS8341_Read(0)>>1;
	MAD[1] = ADS8341_Read(1)>>1;
	MAD[2] = ADS8341_Read(2)>>1;
	
	rX = (MAD[0] - INS_Para.MX_Zero)*INS_Para.MX_Scale;
	rY = (MAD[1] - INS_Para.MY_Zero)*INS_Para.MY_Scale;
	rZ = (MAD[2] - INS_Para.MZ_Zero)*INS_Para.MZ_Scale;

	// calibrate hard and soft iron effect
	p[0] = INS_Para.MCali00*rX + INS_Para.MCali01*rY + INS_Para.MCali02*rZ;
	p[1] = INS_Para.MCali10*rX + INS_Para.MCali11*rY + INS_Para.MCali12*rZ;
	p[2] = INS_Para.MCali20*rX + INS_Para.MCali21*rY + INS_Para.MCali22*rZ;

	norm = sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
	p[0] = p[0]/norm;
	p[1] = p[1]/norm;
	p[2] = p[2]/norm;	
}

void ReadFlashPara()
{
	#ifdef READ_FLASH
		PageRead(1);
		BufferRead(20, (BYTE*)&INS_Para, sizeof(INS_PARAMETER));
		INS_Para.IsUseGPSVelo = TRUE;
		INS_Para.MX_Scale = 1.0;
		INS_Para.MY_Scale = 1.0;
		INS_Para.MZ_Scale = 1.0;
	#else
		INS_Para.MX_Zero = 21296;
		INS_Para.MY_Zero = 17692;
		INS_Para.MZ_Zero = 18233;
		INS_Para.MX_Scale = 1.0;
		INS_Para.MY_Scale = 1.0;
		INS_Para.MZ_Scale = 1.0;
		INS_Para.MCali00 = 2.0889910e-4;
		INS_Para.MCali01 = 0.0;
		INS_Para.MCali02 = 0.0;
		INS_Para.MCali10 = 0.0;
		INS_Para.MCali11 = 1.6015375e-4;
		INS_Para.MCali12 = 0.0;
		INS_Para.MCali20 = 0.0;
		INS_Para.MCali21 = 0.0;
		INS_Para.MCali22 = 1.6286645e-4;

		INS_Para.IsUseMagnet = TRUE;
		INS_Para.Is2DCali = TRUE;
		INS_Para.IsUseGPSVelo = TRUE;
		INS_Para.SendBackT = 10;
		INS_Para.IsNoGPS = FALSE;

		INS_Para.GPSAntDis0 = 0.0;
		INS_Para.GPSAntDis1 = 0.0;
		INS_Para.GPSAntDis2 = 0.0;
		
		INS_Para.R_Coeff_Velocity 	 = 0.0001;
		INS_Para.R_Coeff_Magnet 		 = 0.0001;
		INS_Para.R_Coeff_Acceleration = 0.01;
		
	#endif
}

void WriteFlashPara()
{
	BufferWrite(20, (BYTE*)&INS_Para, sizeof(INS_PARAMETER));
	PageWrite(1);
}

/**************************************************************/

// read Flash status
BYTE ReadStatus()
{
	BYTE t;
	
	t=SSPSendByte(0xD7);
	t=SSPSendByte(0x00);
	
	return t;
}

BYTE WaitReady()
{
	Uint32 i,j;
	BYTE t;

	t = ReadStatus();
	
	for(i = 0; (i<OVERTIME)&&((t & FLASH_STAT) != FLASH_STAT); i++)
	{
		for (j=0;j<100;j++)
			;
		t = ReadStatus();
	}
	
    if (i==OVERTIME)
    {
		return FALSE;
    }
		
	return TRUE;
}

/***************************************************************/

// write data into the buffer
BYTE BufferWrite(Uint16 Offset,BYTE *Data, Uint16 Len)
{
	Uint16 t;
	BYTE temp1,temp2;

	if(Offset + Len > PAGE_SIZE)
	{
		return FALSE;
	}

	if(WaitReady()==FALSE)
	{
		return FALSE;
	}
	
	temp1 = (BYTE)(Offset>>8);
	temp2 = (BYTE)(Offset);
	
	t = SSPSendByte(0x87);
	t = SSPSendByte(0x00);
	t = SSPSendByte(temp1);
	t = SSPSendByte(temp2);
	
	//t = SSPSendByte(0x00);
	
	for (t = 0; t < Len; t++)
	{
		SSPSendByte(Data[t]);	
	}
	
	return TRUE;
}

// read data from buffer
BYTE BufferRead(Uint16 Offset,BYTE *Data, Uint16 Len)
{
	Uint16 i;
	BYTE t;
	BYTE temp1,temp2;

	if(Offset + Len > PAGE_SIZE)
	{
		return FALSE;
	}
	
  	if(WaitReady()==FALSE)
	{
		return FALSE;
	}
	
	temp1 = (BYTE)(Offset>>8);
	temp2 = (BYTE)(Offset);
	
  	SSPSendByte(0xD4);
	SSPSendByte(0x00);
  	SSPSendByte(temp1);
  	SSPSendByte(temp2);
	SSPSendByte(0x00);
	for (i = 0; i < Len; i++)
  	{
		t = SSPSendByte(0x00);
		Data[i] = t;
  	}
	
  	return TRUE; 
}


// write data in the buffer into the flash
BYTE PageWrite(Uint16 PageNo)
{
	Uint16 t;
	BYTE temp1,temp2;
	
	if (PageNo >= FLASH_PAGES)
	{
		return FALSE; 
	}
	
	if(WaitReady()==FALSE)
	{
		return FALSE;
	}
	
	temp1 = (BYTE)(PageNo>>6);
	temp2 = (BYTE)(PageNo<<2);
	
	t = SSPSendByte(0x86);
	t = SSPSendByte(temp1);
	t = SSPSendByte(temp2);
	t = SSPSendByte(0x00);
	
	return TRUE;
}

// read data in the flash into the buffer
BYTE PageRead(Uint16 PageNo)
{
	BYTE temp1,temp2;
	//BYTE t;
	
  	if (PageNo >= FLASH_PAGES)
	{
		return FALSE; 
	}
	
	if(WaitReady()==FALSE)
	{
		return FALSE;
	}
	
	temp1 = (BYTE)(PageNo>>6);
	temp2 = (BYTE)(PageNo<<2);
	
  	SSPSendByte(0x53);
  	SSPSendByte(temp1);
  	SSPSendByte(temp2);
	SSPSendByte(0x00);
	
  	return TRUE; 
}

/*

// erase Block
BYTE BlockErase(Uint16 BlockNo)
{
	//Uint16 i;
	BYTE temp1,temp2;
	
	if (BlockNo>= FLASH_BLOCKS)
	{
		return FALSE;
	}
	
	if(WaitReady()==FALSE)
	{
		return FALSE;
	}
	
	temp1 = (BYTE)((BlockNo>>3)&0xff);
	temp2 = (BYTE)((BlockNo<<5)&0xff);
	
	SSPSendByte(0x50);
	SSPSendByte(temp1);
	SSPSendByte(temp2);
	SSPSendByte(0x00);	
	
	return TRUE;
}

// Erase Page
BYTE PageErase(Uint16 PageNo)
{
	//Uint16 i;
	BYTE temp1,temp2;

	if (PageNo >= FLASH_PAGES)
	{
		return FALSE;
	}
	
	if(WaitReady()==FALSE)
	{
		return FALSE;
	}
	
	temp1 = (BYTE)((PageNo>>6)&0xff);
	temp2 = (BYTE)((PageNo<<2)&0xff);
	
	SSPSendByte(0x81);
	SSPSendByte(temp1);
	SSPSendByte(temp2);
	SSPSendByte(0x00);
	
	return TRUE;
}

*/

// convert the PageSize of Flash to 1024 (only need to call once)
BYTE PageModeSwitch()
{
	BYTE t;
	
	SSPSendByte(0x3D);
	//SSPSendByte(0x00);
	SSPSendByte(0x2A);
	//SSPSendByte(0x00);
	SSPSendByte(0x80);
	//SSPSendByte(0x00);
	SSPSendByte(0xA6);
	//SSPSendByte(0x00);
	
	t = ReadStatus();
	//UARTSendChar(0,t);
	return(t);
}

/********************************************************/
// send data to PC
void INS_SendData2All(float32* GAB, VEC* states, Uint16 * MAGAD, BOOL IsFilterOn)
{
	BYTE tmp = 0;
	float32 p=0.0,r=0.0,y=0.0;
	int16 t1;
	Uint32 t2;
	int i;

	q2eul(states, &r, &p, &y);
	
	tmp = (UBXdata.fixMode==3)?1:0;
	INS_SendData.Status.bit.Attitude = IsFilterOn;
	INS_SendData.Status.bit.BodyAccl = 1;
	INS_SendData.Status.bit.BodyAngVelo = 1;
	INS_SendData.Status.bit.BodySpeed = (tmp&&IsFilterOn);
	INS_SendData.Status.bit.Height = tmp;
	INS_SendData.Status.bit.LaLo = tmp;
	INS_SendData.Status.bit.NEDSpeed = tmp;
	INS_SendData.Status.bit.UseGPS = (INS_Para.IsUseGPSVelo&&IsFilterOn);

	INS_SendData.Pitch = (int16)(p*572.9578);
	INS_SendData.Roll  = (int16)(r*572.9578);
	t1 = (int16)(y*572.9578);
	t1 = (t1 + 7200) % 3600;
	INS_SendData.Yaw   = t1;
	INS_SendData.SpeedN = (int16)(UBXdata.speedN*100.0);
	INS_SendData.SpeedE = (int16)(UBXdata.speedE*100.0);
	INS_SendData.SpeedD = (int16)(UBXdata.speedD*100.0);
	INS_SendData.SpeedBodyX = 0;
	INS_SendData.SpeedBodyY = 0;
	INS_SendData.SpeedBodyZ = 0;
	INS_SendData.Lo = UBXdata.lo;
	INS_SendData.La = UBXdata.la;
	INS_SendData.Height = UBXdata.al;
	INS_SendData.SatNumber = UBXdata.snum;
	INS_SendData.GPSPrecision = (int16)(UBXdata.scep*10);
	INS_SendData.GPSHour = UBXdata.hh;
	INS_SendData.GPSMinute = UBXdata.mm;
	INS_SendData.GPSSecond = UBXdata.ss;
	INS_SendData.AcclBodyX = (int16)(GAB[3]*1000.0);
	INS_SendData.AcclBodyY = (int16)(GAB[4]*1000.0);
	INS_SendData.AcclBodyZ = (int16)(GAB[5]*1000.0);
	INS_SendData.AngVeloBodyX = (int16)(GAB[0]*1000.0);
	INS_SendData.AngVeloBodyY = (int16)(GAB[1]*1000.0);
	INS_SendData.AngVeloBodyZ = (int16)(GAB[2]*1000.0);

	// send to embedded devices
	SCIC_SendAllData();

	// create all 1 frame to PC
	if (IsSendOutData)
	{
		/************* FRAME 1 *************/
		SCIA_ClearSendFrame();
		m_pSendBuf[4] = 0xF1;
		m_pSendBuf[5] = INS_SendData.Pitch & 0xff;
		m_pSendBuf[6] = (INS_SendData.Pitch>>8) & 0xff;
		m_pSendBuf[7] = INS_SendData.Roll & 0xff;
		m_pSendBuf[8] = (INS_SendData.Roll>>8) & 0xff;
		m_pSendBuf[9] = INS_SendData.Yaw & 0xff;
		m_pSendBuf[10] = (INS_SendData.Yaw>>8) & 0xff;

		t1 = (int16)(UBXdata.scep*10);
		m_pSendBuf[11] = t1 & 0xff;
		m_pSendBuf[12] = (t1>>8) & 0xff;
		m_pSendBuf[13] = UBXdata.snum;
		m_pSendBuf[14] = UBXdata.IsNew;
		m_pSendBuf[15] = UBXdata.fixMode;

		m_pSendBuf[16] = UBXdata.lo & 0xff;
		m_pSendBuf[17] = (UBXdata.lo>>8) & 0xff;
		m_pSendBuf[18] = (UBXdata.lo>>16) & 0xff;
		m_pSendBuf[19] = (UBXdata.lo>>24) & 0xff;

		m_pSendBuf[20] = UBXdata.la & 0xff;
		m_pSendBuf[21] = (UBXdata.la>>8) & 0xff;
		m_pSendBuf[22] = (UBXdata.la>>16) & 0xff;
		m_pSendBuf[23] = (UBXdata.la>>24) & 0xff;

		t1 = UBXdata.al/10;
		m_pSendBuf[24] = t1 & 0xff;
		m_pSendBuf[25] = (t1>>8) & 0xff;
		
		m_pSendBuf[26] = UBXdata.hh;
		m_pSendBuf[27] = UBXdata.mm;
		m_pSendBuf[28] = UBXdata.ss;
		SCIA_SendFrame();

		/************* FRAME 2 *************/
		SCIA_ClearSendFrame();
		m_pSendBuf[4] = 0xF2;
		t1 = (int16)(GAB[0]); // deg/s
		m_pSendBuf[5] = t1 & 0xff;
		m_pSendBuf[6] = (t1>>8) & 0xff;
		t1 = (int16)(GAB[1]); // deg/s
		m_pSendBuf[7] = t1 & 0xff;
		m_pSendBuf[8] = (t1>>8) & 0xff;
		t1 = (int16)(GAB[2]); // deg/s
		m_pSendBuf[9] = t1 & 0xff;
		m_pSendBuf[10] = (t1>>8) & 0xff;
		t1 = (int16)(GAB[3]*1000.0); 		// 0.001g
		m_pSendBuf[11] = t1 & 0xff;
		m_pSendBuf[12] = (t1>>8) & 0xff;
		t1 = (int16)(GAB[4]*1000.0); 		// 0.001g
		m_pSendBuf[13] = t1 & 0xff;
		m_pSendBuf[14] = (t1>>8) & 0xff;
		t1 = (int16)(GAB[5]*1000.0); 		// 0.001g
		m_pSendBuf[15] = t1 & 0xff;
		m_pSendBuf[16] = (t1>>8) & 0xff;

		if (IsMagnetCali)
		{
			t1 = (int16)(MAGAD[0]); 			// NO unit
			m_pSendBuf[17] = t1 & 0xff;
			m_pSendBuf[18] = (t1>>8) & 0xff;
			t1 = (int16)(MAGAD[1]); 			// NO unit
			m_pSendBuf[19] = t1 & 0xff;
			m_pSendBuf[20] = (t1>>8) & 0xff;
			t1 = (int16)(MAGAD[2]); 			// NO unit
			m_pSendBuf[21] = t1 & 0xff;
			m_pSendBuf[22] = (t1>>8) & 0xff;
		}
		else
		{
			t1 = (int16)(GAB[6]*1000.0); 		// NO unit
			m_pSendBuf[17] = t1 & 0xff;
			m_pSendBuf[18] = (t1>>8) & 0xff;
			t1 = (int16)(GAB[7]*1000.0); 		// NO unit
			m_pSendBuf[19] = t1 & 0xff;
			m_pSendBuf[20] = (t1>>8) & 0xff;
			t1 = (int16)(GAB[8]*1000.0); 		// NO unit
			m_pSendBuf[21] = t1 & 0xff;
			m_pSendBuf[22] = (t1>>8) & 0xff;
		}
		m_pSendBuf[23] = IsMagnetCali;
		SCIA_SendFrame();

		/************* FRAME 3 *************/
		SCIA_ClearSendFrame();
		m_pSendBuf[4] = 0xF3;

		for (i=0;i<5;i++)
		{
			memmove(&(t2),&(states->ve[i]),sizeof(float32));
			m_pSendBuf[i*4+5] = t2 & 0xff;
			m_pSendBuf[i*4+6] = (t2>>8)  & 0xff;
			m_pSendBuf[i*4+7] = (t2>>16) & 0xff;
			m_pSendBuf[i*4+8] = (t2>>24) & 0xff;
		}
		SCIA_SendFrame();

		/************* FRAME 4 *************/
		SCIA_ClearSendFrame();
		m_pSendBuf[4] = 0xF4;

		for (i=0;i<5;i++)
		{
			memmove(&(t2),&(states->ve[i+5]),sizeof(float32));
			m_pSendBuf[i*4+5] = t2 & 0xff;
			m_pSendBuf[i*4+6] = (t2>>8)  & 0xff;
			m_pSendBuf[i*4+7] = (t2>>16) & 0xff;
			m_pSendBuf[i*4+8] = (t2>>24) & 0xff;
		}
		SCIA_SendFrame();

		scia_start_tx();
	}
	
}


