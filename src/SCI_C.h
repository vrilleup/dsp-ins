/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#ifndef SCI_C_H
#define SCI_C_H

struct STATUS_BITS  {           // bits   description
   Uint16 Attitude:1;  
   Uint16 NEDSpeed:1;  
   Uint16 BodySpeed:1; 
   Uint16 LaLo:1;     
   Uint16 Height:1;    
   Uint16 BodyAccl:1; 
   Uint16 BodyAngVelo:1; 
   Uint16 UseGPS:1;
};

union SendData_Status{
   BYTE              all;
   struct STATUS_BITS    bit;
};

typedef struct _SendData_Frame_{
	BYTE S1;
	BYTE S2;
	BYTE Length;
	BYTE Command;
	union SendData_Status Status;
	int16 Pitch;		//0.1 degree
	int16 Roll;			//0.1 degree
	int16 Yaw;			//0.1 degree
	int16 SpeedN;		//0.01 m/s
	int16 SpeedE;		//0.01 m/s
	int16 SpeedD;		//0.01 m/s
	int16 SpeedBodyX;	//0.01 m/s
	int16 SpeedBodyY;	//0.01 m/s
	int16 SpeedBodyZ;	//0.01 m/s
	int32 Lo;			// 1e-7 degree
	int32 La;			// 1e-7 degree
	int32 Height;		// millmeter
	BYTE  SatNumber;
	int16 GPSPrecision;	//0.01m
	BYTE  GPSHour;
	BYTE  GPSMinute;
	BYTE  GPSSecond;
	int16 AcclBodyX;	//0.001 m/s^2
	int16 AcclBodyY;	//0.001 m/s^2
	int16 AcclBodyZ;	//0.001 m/s^2
	int16 AngVeloBodyX;	//0.01 degree/s
	int16 AngVeloBodyY;	//0.01 degree/s
	int16 AngVeloBodyZ;	//0.01 degree/s
	BYTE  CheckSum;
} SENDDATA_FRAME;

extern SENDDATA_FRAME INS_SendData;
extern void SCIC_SendAllData();

#endif

