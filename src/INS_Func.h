/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "matrix.h"		// matrix include file

#ifndef INS_FUNC_H
#define INS_FUNC_H

typedef struct _INS_PARAMETER_
{
	int32 MX_Zero;		// magnet sensor zero point
	int32 MY_Zero;
	int32 MZ_Zero;
	int32 IsUseMagnet;	// using magnet sensors or not
	int32 Is2DCali;		// using 2D magnet calibrate
	int32 IsUseGPSVelo;	// using GPS velocity or not
	int32 SendBackT;	// send data period (in 10ms)
	int32 IsNoGPS;		// using GPS or not

	float32 MCali00;	// magnet calibrate matrix
	float32 MCali01;
	float32 MCali02;
	float32 MCali10;
	float32 MCali11;
	float32 MCali12;
	float32 MCali20;
	float32 MCali21;
	float32 MCali22;

	float32 MX_Scale;
	float32 MY_Scale;
	float32 MZ_Scale;
	
	float32 GPSAntDis0;
	float32 GPSAntDis1;
	float32 GPSAntDis2;

	float32 R_Coeff_Velocity;
	float32 R_Coeff_Magnet;
	float32 R_Coeff_Acceleration;

	/*
	int32 Int32_Reserved11;
	int32 Int32_Reserved12;
	int32 Int32_Reserved13;
	int32 Int32_Reserved14;
	int32 Int32_Reserved15;
	int32 Int32_Reserved16;
	int32 Int32_Reserved17;
	int32 Int32_Reserved18;
	int32 Int32_Reserved19;
	int32 Int32_Reserved20;
	int32 Int32_Reserved21;
	int32 Int32_Reserved22;
	int32 Int32_Reserved23;
	int32 Int32_Reserved24;
	int32 Int32_Reserved25;
	*/

	/*
	float32 Float32_Reserved11;
	float32 Float32_Reserved12;
	float32 Float32_Reserved13;
	float32 Float32_Reserved14;
	float32 Float32_Reserved15;
	float32 Float32_Reserved16;
	float32 Float32_Reserved17;
	float32 Float32_Reserved18;
	float32 Float32_Reserved19;
	float32 Float32_Reserved20;
	float32 Float32_Reserved21;
	float32 Float32_Reserved22;
	float32 Float32_Reserved23;
	float32 Float32_Reserved24;
	float32 Float32_Reserved25;
*/
} INS_PARAMETER;

extern Uint16 Magnet_AD[3];
extern INS_PARAMETER INS_Para;

extern void GetMagnet(float32* p, Uint16 * MAD);
extern void ReadFlashPara();
extern void WriteFlashPara();
extern BYTE PageModeSwitch();

extern void ReadFlashPara();
extern void WriteFlashPara();
extern BYTE ReadStatus();
extern BYTE WaitReady();
extern BYTE BufferWrite(Uint16 Offset,BYTE *Data, Uint16 Len);
extern BYTE BufferRead(Uint16 Offset,BYTE *Data, Uint16 Len);
extern BYTE PageWrite(Uint16 PageNo);
extern BYTE PageRead(Uint16 PageNo);

extern BYTE   IsMagnetCali;

//extern BYTE BlockErase(Uint16 BlockNo);
//extern BYTE PageErase(Uint16 PageNo);

extern void INS_SendData2All(float32* GAB, VEC* states, Uint16 * MAGAD, BOOL IsFilterOn);
#endif


