/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#ifndef ADIS16350_H
#define ADIS16350_H

extern void 	ADIS_Init(Uint16 d);
extern Uint16 	ADIS_ReadParameter();
extern void 	ADIS_ReadData(float32* data);
extern int16 	ADIS_ReadTemperature(Uint16 axis);
extern Uint16 	ADIS_ReadAUX_ADC();

extern void 	ADIS_AutoNull();
extern void 	ADIS_PreciseAutoNull();

#endif

