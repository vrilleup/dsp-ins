/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "SCI_A.h"
#include "GPS.h"
#include "INS_Init.h"
#include "INS_Func.h"
#include "EKFilter.h"

#include <string.h>

#pragma CODE_SECTION(SCIA_AddOneByte, "xintfloadfuncs");
#pragma CODE_SECTION(SCIA_AddBytes, "xintfloadfuncs");
#pragma CODE_SECTION(SCIA_ClearSendFrame, "xintfloadfuncs");
#pragma CODE_SECTION(SCIA_SendFrame, "xintfloadfuncs");
#pragma CODE_SECTION(SCIA_ProcessFrame, "xintfloadfuncs");

BYTE m_pSendBuf[m_nFrameSize];

static BYTE m_pDataBuf[SCIA_FRAME_MAX];
static int  m_cDataBufCount = 0;

static BYTE m_pDataProcess[SCIA_FRAME_MAX];
BOOL IsResetEKF = FALSE;   // Reset EKF or not

int SCIA_AddOneByte(BYTE d)
{
	Uint16 sum;
	int k;
	
	if (m_cDataBufCount==0)
	{
		// first byte
		if (d == m_S1)
			m_pDataBuf[m_cDataBufCount++] = d;
	}
	else if (m_cDataBufCount==1)
	{
		if (d == m_S2)
			m_pDataBuf[m_cDataBufCount++] = d;
		else
			m_cDataBufCount = 0;
	}
	else if (m_cDataBufCount > 1)
	{
		m_pDataBuf[m_cDataBufCount++] = d;
		if (m_cDataBufCount == m_nFrameSize)
		{
			// get a whole frame, check sum
			m_cDataBufCount = 0;
			sum = 0;
			for (k = 0; k<(m_nFrameSize-1); k++)
				sum += m_pDataBuf[k];

			if ((sum & 0xff) == m_pDataBuf[m_nFrameSize-1])
			{
				// check sum OK
				// copy to another buffer for process
				for (k = 0; k<(m_nFrameSize-3); k++)
					m_pDataProcess[k] = m_pDataBuf[k+2];
				return TRUE;
			}
		}
	}

	return FALSE;
}


int SCIA_AddBytes(unsigned char* p, Uint16 len)
{
	int bResult = FALSE;
	int i;

	for (i=0; i<len; i++)
	{
		if (SCIA_AddOneByte(p[i]))
			bResult = TRUE;
	}

	return bResult;
}

void SCIA_ClearSendFrame()
{
	int i;
	for (i=0; i<m_nFrameSize; i++)
		m_pSendBuf[i] = 0;

	m_pSendBuf[0] = m_S1;
	m_pSendBuf[1] = m_S2;
	m_pSendBuf[2] = 0x98;
	m_pSendBuf[3] = 0x19;
	//m_pSendBuf[4] = 0xF1;
}

void SCIA_SendFrame()
{
	Uint32 sum = 0;
	int i;

	for (i=0; i<(m_nFrameSize-1); i++)
	{
		scia_xmit(m_pSendBuf[i]);
		sum += m_pSendBuf[i];
	}
	m_pSendBuf[m_nFrameSize-1] = (sum & 0xff);
	scia_xmit(m_pSendBuf[m_nFrameSize-1]);

	//scia_start_tx();
}

void SCIA_ProcessFrame()
{
	int32 t1,t2;
	switch (m_pDataProcess[2])
	{
	case 0xc1:
		// Setup a direct connection between GPS and PC
		CLOSE_SCIA_TX;
		CLOSE_SCIC_TX;
		IsDirectGPS = TRUE;
		break;
			
	case 0xd1:
		scia_print("Test OK!", 0);
		break;
	case 0xd2:
		IsSendOutData = TRUE;
		//scia_print("Start Send!",0);
		break;
	case 0xd3:
		IsSendOutData = FALSE;
		//scia_print("Stop Send!",0);
		break;
	case 0xd4:
		INS_Para.IsUseMagnet = TRUE;
		IsRefreshRH = TRUE;
		scia_print("Use Magnet",0);
		break;
	case 0xd5:
		INS_Para.IsUseMagnet = FALSE;
		IsRefreshRH = TRUE;
		scia_print("NOT Use Magnet",0);
		break;
	case 0xd6:
		WriteFlashPara();
		scia_print("FLASH Write",0);
		break;
	case 0xd7:
		ReadFlashPara();
		scia_print("FLASH Read",0);
		break;		
	case 0xd8:
		// magnet set and reset
		GPIO1_CLEAR();
		scia_print("Reset Magnet",0);
		break;
	case 0xd9:
		GPIO1_SET();		
		scia_print("Set Magnet",0);
		break;			
	case 0xda:
		IsMagnetCali = TRUE;
		scia_print("Magnet Cali ON",0);
		break;
	case 0xdb:
		IsMagnetCali = FALSE;
		scia_print("Magnet Cali OFF",0);
		break;			
	case 0xdc:
		INS_Para.SendBackT = BYTE2int16(m_pDataProcess[4], m_pDataProcess[3]);
		scia_print("Send Back Period(10ms)",1,INS_Para.SendBackT);
		break;			
	case 0xdd:
		IsResetEKF = TRUE;
		break;
		
	case 0xe1:
		INS_Para.IsNoGPS= TRUE;
		IsRefreshRH = TRUE;
		scia_print("NOT have GPS",0);
		break;
	case 0xe2:
		INS_Para.IsNoGPS= FALSE;
		IsRefreshRH = TRUE;
		scia_print("Have GPS",0);
		break;
		
	/*********************************************/
	case 0xa1:
		// Magnet Calibrate Result 00 01 02 10 and 2D or 3D and Zeros
		*((Uint32*)&(INS_Para.MCali00)) = BYTE2int32(	m_pDataProcess[6],
														m_pDataProcess[5],
														m_pDataProcess[4],
														m_pDataProcess[3]);
		*((Uint32*)&(INS_Para.MCali01)) = BYTE2int32(	m_pDataProcess[10],
														m_pDataProcess[9],
														m_pDataProcess[8],
														m_pDataProcess[7]);
		*((Uint32*)&(INS_Para.MCali02)) = BYTE2int32(	m_pDataProcess[14],
														m_pDataProcess[13],
														m_pDataProcess[12],
														m_pDataProcess[11]);
		*((Uint32*)&(INS_Para.MCali10)) = BYTE2int32(	m_pDataProcess[18],
														m_pDataProcess[17],
														m_pDataProcess[16],
														m_pDataProcess[15]);
		INS_Para.Is2DCali = m_pDataProcess[19];
		INS_Para.MX_Zero = BYTE2int16(m_pDataProcess[21], m_pDataProcess[20]);
		INS_Para.MY_Zero = BYTE2int16(m_pDataProcess[23], m_pDataProcess[22]);
		INS_Para.MZ_Zero = BYTE2int16(m_pDataProcess[25], m_pDataProcess[24]);

		IsRefreshRH = TRUE;
		scia_print("MagCali Result No.1",0);
		break;
	case 0xa2:
		// Magnet Calibrate Result 11 12 20 21 22
		*((Uint32*)&(INS_Para.MCali11)) = BYTE2int32(	m_pDataProcess[6],
														m_pDataProcess[5],
														m_pDataProcess[4],
														m_pDataProcess[3]);
		*((Uint32*)&(INS_Para.MCali12)) = BYTE2int32(	m_pDataProcess[10],
														m_pDataProcess[9],
														m_pDataProcess[8],
														m_pDataProcess[7]);
		*((Uint32*)&(INS_Para.MCali20)) = BYTE2int32(	m_pDataProcess[14],
														m_pDataProcess[13],
														m_pDataProcess[12],
														m_pDataProcess[11]);
		*((Uint32*)&(INS_Para.MCali21)) = BYTE2int32(	m_pDataProcess[18],
														m_pDataProcess[17],
														m_pDataProcess[16],
														m_pDataProcess[15]);
		*((Uint32*)&(INS_Para.MCali22)) = BYTE2int32(	m_pDataProcess[22],
														m_pDataProcess[21],
														m_pDataProcess[20],
														m_pDataProcess[19]);
		scia_print("MagCali Result No.2",0);		
		break;
	case 0xa3:
		// GPS antenna displacement
		*((Uint32*)&(INS_Para.GPSAntDis0)) = BYTE2int32(	m_pDataProcess[6],
														m_pDataProcess[5],
														m_pDataProcess[4],
														m_pDataProcess[3]);
		*((Uint32*)&(INS_Para.GPSAntDis1)) = BYTE2int32(	m_pDataProcess[10],
														m_pDataProcess[9],
														m_pDataProcess[8],
														m_pDataProcess[7]);
		*((Uint32*)&(INS_Para.GPSAntDis2)) = BYTE2int32(	m_pDataProcess[14],
														m_pDataProcess[13],
														m_pDataProcess[12],
														m_pDataProcess[11]);
		Setdis(INS_Para.GPSAntDis0, INS_Para.GPSAntDis1, INS_Para.GPSAntDis2);
		scia_print("GPSAnt Displacement",0);
		break;
	/*********************************************/
	case 0xa4:
		// EKF R matrix Coefficients
		*((Uint32*)&(INS_Para.R_Coeff_Velocity)) = 		BYTE2int32(	m_pDataProcess[6],
														m_pDataProcess[5],
														m_pDataProcess[4],
														m_pDataProcess[3]);
		*((Uint32*)&(INS_Para.R_Coeff_Magnet)) = 		BYTE2int32(	m_pDataProcess[10],
														m_pDataProcess[9],
														m_pDataProcess[8],
														m_pDataProcess[7]);
		*((Uint32*)&(INS_Para.R_Coeff_Acceleration)) = 	BYTE2int32(	m_pDataProcess[14],
														m_pDataProcess[13],
														m_pDataProcess[12],
														m_pDataProcess[11]);
		scia_print("R Coefficients",0);
		
		IsRefreshRH = TRUE;
		break;
	/*********************************************/
	default:
		break;
	}

}

