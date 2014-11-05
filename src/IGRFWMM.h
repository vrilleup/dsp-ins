// IGRFWMM.h: interface for the CIGRFWMM class.
// http://williams.best.vwh.net/
//////////////////////////////////////////////////////////////////////

/* magfield.h -- compute local magnetic variation given position,
**               altitude, and date
**
** This is an implementation of the NIMA WMM 2000
**
**    http://www.nima.mil/GandG/ngdc-wmm2000.html
**    For WMM2000 coefficients:
**    ftp://ftp.ngdc.noaa.gov/Solid_Earth/Mainfld_Mag/DoD_Model/wmm.cof
**    For IGRF/DGRF coefficients:
**    http://swdcdb.kugi.kyoto-u.ac.jp/igrf/coef/igrfall.d
**
** Copyright (C) 2000  Edward A Williams <Ed_Williams@compuserve.com>
**
** Adapted from Excel 3.0 version 3/27/94 EAW
** Recoded in C++ by Starry Chan
** WMM95 added and rearranged in ANSI-C EAW 7/9/95
** Put shell around program and made Borland & GCC compatible EAW 11/22/95
** IGRF95 added 2/96 EAW
** WMM2000 IGR2000 added 2/00 EAW
** Released under GPL 3/26/00 EAW
** Adaptions and modifications for the SimGear project  3/27/2000 CLO
** WMM2005 IGRF2005 added 01/05 EAW
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License as
** published by the Free Software Foundation; either version 2 of the
** License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful, but
** WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
**
*/

/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com)
*/

// Modified by VrilleUp for TI DSP TMS320F28335
// Date: 2008.08.08
// One World, One Dream!

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#ifndef IGRFWMM_H
#define IGRFWMM_H

typedef float32 Mag_Real;
typedef Uint32 Mag_Uint32;

/* return variation (in radians) given geodetic latitude (radians), longitude
	(radians) ,height (m), (Julian) date and field model
	model=8: WMM2005, model=9: IGRF2005
	latitude N and longitude E are positive numbers
	
  Output field: Bx(N) By(E) Bz(Down) (in nTesla) dip (degrees down positive)
  variation (degrees E positive)
*/

extern void SGMagVar(Mag_Real lat, Mag_Real lon, Mag_Real h, 
			  							Mag_Uint32 dat, float32* field);
extern Mag_Real rad_to_deg(Mag_Real rad);
extern Mag_Real deg_to_rad(Mag_Real deg);

/* Convert date to Julian day    2000-2099 */
extern Mag_Uint32 yymmdd_to_julian_days(int yy,int mm,int dd);

#endif
