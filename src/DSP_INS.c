/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com)
*/

//###########################################################################
// DSP INS main file
// $Last modified date: 2008.08.07 by VrilleUp $
//###########################################################################


#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "ADIS16350.h"
#include "EKFilter.h"
#include "INS_Init.h"
#include "INS_Func.h"
#include "SCI_A.h"
#include "SCI_C.h"
#include "GPS.h"
#include "IGRFWMM.h"

#include <stdio.h>
#include <stdarg.h>

#include "matrix.h"		// matrix include file

// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped using
// the linker cmd file.

extern interrupt void cpu_timer0_isr(void);
extern interrupt void cpu_timer1_isr(void);
extern interrupt void cpu_timer2_isr(void);

extern interrupt void scia_rx_isr(void);
extern interrupt void scia_tx_isr(void);
extern interrupt void scib_rx_isr(void);
extern interrupt void scib_tx_isr(void);
extern interrupt void scic_rx_isr(void);
extern interrupt void scic_tx_isr(void);

#pragma CODE_SECTION(cpu_timer0_isr, "ramfuncs");
#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
#pragma CODE_SECTION(cpu_timer2_isr, "ramfuncs");

#pragma CODE_SECTION(scia_rx_isr, "ramfuncs");
#pragma CODE_SECTION(scia_tx_isr, "ramfuncs");
#pragma CODE_SECTION(scib_rx_isr, "ramfuncs");
#pragma CODE_SECTION(scib_tx_isr, "ramfuncs");
#pragma CODE_SECTION(scic_rx_isr, "ramfuncs");
#pragma CODE_SECTION(scic_tx_isr, "ramfuncs");

#pragma CODE_SECTION(main_func, "xintfloadfuncs");

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

extern Uint16 XintfLoadfuncsLoadStart;
extern Uint16 XintfLoadfuncsLoadEnd;
extern Uint16 XintfLoadfuncsRunStart;

void main_func(void);

void main(void)
{
	Uint16 r[8];

	// init all hardware
	init_INS();
	
	MemCopy((Uint16 *)0x380400, (Uint16 *)0x380407, r);

	// call main loop function
	main_func();
} // end of main

void main_func(void)
{
	float32 ap=0.0,ar=0.0,ay=0.0;
	float32 GAB[9]; // Gyro, Accl, Magnet
	Uint16 i;
	Uint16 n;
	Uint16 r;
	int32  api,ari,ayi;
	BYTE tmp,tmp1;
	Uint16 fix3D_level = 5;
	Uint16 fix3D_count = 5;
	BOOL   IsGPSVelocity = TRUE;
	BOOL   IsEKFInited = FALSE;

	VEC *ftmp,*btmp,*pos,*states;
	VEC *ut,*zt;

	ftmp = v_get(3);
	btmp = v_get(3);
	pos  = v_get(3);
	ut   = v_get(6);
	zt   = v_get(6);
	states = v_get(10);

	// init - read parameters from flash
	ReadFlashPara();
	IsResetEKF = FALSE;
	EKFReset:
	/////////////////////////////////////////////////////
	// init ADIS16350
	ADIS_Init(0);
	// init GPS - no init function
	
	scia_print("INS Start Now...", 0);

	/*****************************************************/

	for (i=0;i<10;i++)
		states->ve[i] = 0.0;
	states->ve[0] = 1.0;
	UBXdata.IsNew = FALSE;
	UBXdata.fixMode = 0;
	i = 0;
	ftmp->ve[0] = 0.0;
	ftmp->ve[1] = 0.0;
	ftmp->ve[2] = 0.0;
	btmp->ve[0] = 0.0;
	btmp->ve[1] = 0.0;
	btmp->ve[2] = 0.0;
	GAB[0] = 0.0;
	GAB[1] = 0.0;
	GAB[2] = 0.0;
	GAB[3] = 0.0;
	GAB[4] = 0.0;
	GAB[5] = 0.0;
	GAB[6] = 0.0;
	GAB[7] = 0.0;
	GAB[8] = 0.0;
	ap=0.0;
	ar=0.0;
	ay=0.0;
	IsGPSVelocity = TRUE;
	
	/*****************************************************/

	// wait for GPS 3D fix
	while(!(INS_Para.IsNoGPS))
	{
		if (SCIC_RX_NEW)
		{
			// if some new GPS data
			CLOSE_SCIC_RX;
			r = UBX_Addbytes(SCIC_RX_BUF, SCIC_RX_COUNT);
			SCIC_RX_COUNT = 0;
			SCIC_RX_NEW = FALSE;
			OPEN_SCIC_RX;

			if (r)
			{
				if (UBXdata.IsNew)
				{
					if (UBXdata.fixMode == 3)
					{
						i++;
						if (i >= 3)
						{
							scia_print("GPS 3D-fix...",0);
							break;
						}
					}
					UBXdata.IsNew = FALSE;
				}
			}
		}
		// send back data to PC every 250ms
		if (Period250ms)
		{
			// Get Accl and Magnet
			ADIS_ReadData(&(GAB[0]));
			GetMagnet(&(GAB[6]),Magnet_AD);		
			// send data to PC and embedded devices
			INS_SendData2All(&(GAB[0]), states, Magnet_AD, FALSE);
			Period250ms = FALSE;
		}
		// refresh frames from PC
		if (SCIA_RX_NEW)
		{
			CLOSE_SCIA_RX;
			r = SCIA_AddBytes(SCIA_RX_BUF, SCIA_RX_COUNT);
			SCIA_RX_COUNT = 0;
			SCIA_RX_NEW = FALSE;
			OPEN_SCIA_RX;
			if (r)
			{
				// received a new frame from PC
				// now data is in m_pDataProcess
				SCIA_ProcessFrame();
				// if reset EKF
				if (IsResetEKF)
				{
					IsResetEKF = FALSE;
					goto EKFReset;
				}
			}
		}
	}

	//*******************************************************
	
	n = 100;
	for (i=0;i<n;i++)
	{
		// Get Accl and Magnet
		ADIS_ReadData(&(GAB[0]));
		GetMagnet(&(GAB[6]),Magnet_AD);

		// calculate average
		ftmp->ve[0] = ftmp->ve[0]*i/(i+1) + GAB[3]/(i+1);
		ftmp->ve[1] = ftmp->ve[1]*i/(i+1) + GAB[4]/(i+1);
		ftmp->ve[2] = ftmp->ve[2]*i/(i+1) + GAB[5]/(i+1);
		btmp->ve[0] = btmp->ve[0]*i/(i+1) + GAB[6]/(i+1);
		btmp->ve[1] = btmp->ve[1]*i/(i+1) + GAB[7]/(i+1);
		btmp->ve[2] = btmp->ve[2]*i/(i+1) + GAB[8]/(i+1);
	}

	// init Attitude and EKF
	Init_Attitude(ftmp, btmp, IsEKFInited);
	IsEKFInited = TRUE;
	// and init the position and date
	pos->ve[0] = UBXdata.la*1.745329e-9;
	pos->ve[1] = UBXdata.lo*1.745329e-9;
	pos->ve[2] = UBXdata.al*0.001;
	//Update_Date(UBXdata.yy, UBXdata.mo, UBXdata.dd);
	//Update_b0(pos);

	// EKFilter is Starting
	getX(states);
	q2eul(states, &ar, &ap, &ay);
	api = ap*572.96;
	ari = ar*572.96;
	ayi = ay*572.96;
	scia_print("Init (PRY):", 3, api, ari, ayi);

	//*******************************************************

	// reset the timer0
	Period10ms = FALSE;
	Period250ms = FALSE;
	//RELOAD_TIMER0;
	tmp = 0;
	tmp1 = 0;

	for(;;)
	{
		// refresh GPS UBX frame
		if (SCIC_RX_NEW)
		{
			// if some new GPS data
			CLOSE_SCIC_RX;
			r = UBX_Addbytes(SCIC_RX_BUF, SCIC_RX_COUNT);
			SCIC_RX_COUNT = 0;
			SCIC_RX_NEW = FALSE;
			OPEN_SCIC_RX;
		}
		// do things every 10 ms
		if (Period10ms)
		{
			if (tmp > 1)
			{
				// not for the first time
				// Get Accl and Magnet
				ADIS_ReadData(&(GAB[0]));
				GetMagnet(&(GAB[6]),Magnet_AD);
				
		    // 1. perform prediction of EKF
				// u = Gx Gy Gz Ax Ay Az
				// set angular velocity to rad/s
				ut->ve[0] = GAB[0]*0.01745329;
				ut->ve[1] = GAB[1]*0.01745329;
				ut->ve[2] = GAB[2]*0.01745329;
				// set acceleration to m/s^2
				ut->ve[3] = GAB[3]*g;
				ut->ve[4] = GAB[4]*g;
				ut->ve[5] = GAB[5]*g;

				// reset after measurement flag
				//scia_print("P1",1,CpuTimer1.InterruptCount);
				timeUpdateStep(ut);
				//scia_print("P2",1,CpuTimer1.InterruptCount);

		    // 2. perform measurement of EKF
				if (UBXdata.IsNew)
				{
					// received a new GPS speed frame
					zt->ve[3] = GAB[6];
					zt->ve[4] = GAB[7];
					zt->ve[5] = GAB[8];
					// do measurement instead of predict of EKF
					if (UBXdata.fixMode == 3
					&& !INS_Para.IsNoGPS)
					{
						// z = Vx Vy Vz Mx My Mz
						zt->ve[0] = UBXdata.speedN;
						zt->ve[1] = UBXdata.speedE;
						zt->ve[2] = UBXdata.speedD;
						IsGPSVelocity = TRUE;
					}
					else
					{
						// z = Ax Ay Az Mx My Mz
						zt->ve[0] = GAB[3]*g;
						zt->ve[1] = GAB[4]*g;
						zt->ve[2] = GAB[5]*g;
						IsGPSVelocity = FALSE;
					}
					//scia_print("M1",1,CpuTimer1.InterruptCount);
					measureUpdateStep(zt, IsGPSVelocity);
					//measureUpdateStep(zt, TRUE);
					//scia_print("M2",1,CpuTimer1.InterruptCount);

					// reset new GPS flag
					UBXdata.IsNew = FALSE;
				}

				/******************************************/
				// send out data to PC and embedded device
				//scia_print("P3",1,CpuTimer1.InterruptCount);
				tmp1 = (tmp1+1)%(INS_Para.SendBackT);
				if (tmp1 == 0)
				{
					getX(states);
					//scia_print("P4",1,CpuTimer1.InterruptCount);
					INS_SendData2All(&(GAB[0]), states, Magnet_AD, TRUE);
					//scia_print("P5",1,CpuTimer1.InterruptCount);
				}
			}

			tmp++;
			if (tmp > 1)
				tmp = 2;
			Period10ms = FALSE;
		}

		// refresh frames from PC
		if (SCIA_RX_NEW)
		{
			CLOSE_SCIA_RX;
			r = SCIA_AddBytes(SCIA_RX_BUF, SCIA_RX_COUNT);
			SCIA_RX_COUNT = 0;
			SCIA_RX_NEW = FALSE;
			OPEN_SCIA_RX;
			if (r)
			{
				// received a new frame from PC
				// now data is in m_pDataProcess
				SCIA_ProcessFrame();
				if (IsResetEKF)
				{
					IsResetEKF = FALSE;
					goto EKFReset;
				}
			}
		}
	} // end of for(;;)
}

