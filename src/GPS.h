/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#ifndef GPS_H
#define GPS_H

typedef struct _GPS_DATA_
{
	Uint16  fixMode; 	// GPS fixing mode, 0=no fix, 2=2D fix, 3=3D fix
	BOOL  IsNew;
	Uint16  yy;			// UTC year
	unsigned char mo;			// UTC month
	unsigned char dd;			// UTC day
	unsigned char hh;			// UTC hour
	unsigned char mm;			// UTC minute
	unsigned char ss;			// UTC second
	int32 la;				// latitude in 1e-7 degree
	int32 lo;				// longitude in 1e-7 degree
	int32 al;				// altitude in millmeter
	float32 speedN;		// north speed in m/s
	float32 speedE;		// east speed in m/s
	float32 speedD;		// down speed in m/s
	float32 speedAcc;	// speed accuracy in m/s
	float32 spe;		// speed in m/s
	float32 yaw;		// course in rad
	unsigned char snum;			// num of satellites
	int32 scep;		// GPS CEP in millmeter
} GPS_DATA;

extern BOOL UBX_Addbytes(unsigned char* s, int len);
extern GPS_DATA UBXdata;
extern BOOL IsDirectGPS;

#endif




