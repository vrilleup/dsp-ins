// IGRFWMM.cpp: implementation of the CIGRFWMM class.
// rewrite from the version of Edward A Williams
// <Ed_Williams@compuserve.com>
// http://williams.best.vwh.net/
//////////////////////////////////////////////////////////////////////

/*	module magfield.c */

/* Module to calculate magnetic variation and field given position,
**				 altitude, and date
** Implements the NIMA (formerly DMA) WMM and IGRF models
**
**	  http://www.nima.mil/GandG/ngdc-wmm2000.html
**	  For WMM2000 coefficients:
**	  ftp://ftp.ngdc.noaa.gov/Solid_Earth/Mainfld_Mag/DoD_Model/wmm.cof
**	  For IGRF/DGRF coefficients:
**	  http://swdcdb.kugi.kyoto-u.ac.jp/igrf/coef/igrfall.d
**
** Copyright (C) 2000  Edward A Williams <Ed_Williams@compuserve.com>
**
**	The routine uses a spherical harmonic expansion of the magnetic
** potential up to twelfth order, together with its time variation, as
** described in Chapter 4 of "Geomagnetism, Vol 1, Ed. J.A.Jacobs,
** Academic Press (London 1987)". The program first converts geodetic
** coordinates (lat/long on elliptic earth and altitude) to spherical
** geocentric (spherical lat/long and radius) coordinates. Using this,
** the spherical (B_r, B_theta, B_phi) magnetic field components are
** computed from the model. These are finally referred to surface (X, Y,
** Z) coordinates.
**
**	 Fields are accurate to better than 200nT, variation and dip to
** better than 0.5 degrees, with the exception of the declination near
** the magnetic poles (where it is ill-defined) where the error may reach
** 4 degrees or more.
**
**	 Variation is undefined at both the geographic and	
** magnetic poles, even though the field itself is well-behaved. To
** avoid the routine blowing up, latitude entries corresponding to
** the geographic poles are slightly offset. At the magnetic poles,
** the routine returns zero variation.
**
** HISTORY
** Adapted from EAW Excel 3.0 version 3/27/94 EAW
** Recoded in C++ by Starry Chan
** WMM95 added and rearranged in ANSI-C EAW 7/9/95
** Put shell around program and made Borland & GCC compatible EAW 11/22/95
** IGRF95 added 2/96 EAW
** WMM2000 IGR2000 added 2/00 EAW
** Released under GPL  3/26/00 EAW
** Adaptions and modifications for the SimGear project	3/27/2000 CLO
** Removed all pow() calls and made static roots[][] arrays to
** save many sqrt() calls on subsequent invocations
** 3/28/2000  Norman Vine -- nhv@yahoo.com
** Put in some bullet-proofing to handle magnetic and geographic poles.
** 3/28/2000 EAW
** Added missing comment close, the lack of which caused the altitude 
** correction to be omitted.
** 01/31/01 Jim Seymour (jseymour@LinxNet.com)
** 1/16/05 EAW	added wmm2005 igrf2005 models
** modified code for n=13 expansion for igrf2005
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

#include "IGRFWMM.h"
#include <math.h>


#pragma CODE_SECTION(yymmdd_to_julian_days, "xintfloadfuncs");
#pragma CODE_SECTION(deg_to_rad, "xintfloadfuncs");
#pragma CODE_SECTION(rad_to_deg, "xintfloadfuncs");
#pragma CODE_SECTION(SGMagVar, "xintfloadfuncs");


#define max_mag(a,b)	(((a) > (b)) ? (a) : (b))

//////////////////////////////////////////////////////////////////////
// Constants
static Mag_Real pi = 3.14159265358979;
/* major radius (km) IAU66 ellipsoid */
static Mag_Real a = 6378.16;
/* inverse flattening IAU66 ellipsoid = 1.0 / 298.25 */
// static Mag_Real f = 0.003352892;
/* minor radius b=a*(1-f) = 6378.16 * (1.0 -1.0 / 298.25 ) */
static Mag_Real b = 6356.775;
/* "mean radius" for spherical harmonic expansion */
static Mag_Real r_0 = 6371.2;

static Mag_Real P[14][14];
static Mag_Real DP[14][14];
static Mag_Real gnm[14][14];
static Mag_Real hnm[14][14];
static Mag_Real sm[14];
static Mag_Real cm[14];

static Mag_Real root[14];
static Mag_Real roots[14][14][2];

static int nmax = 13;

/* wmm2005 constants */
static Mag_Real gnm_wmm2005[13][13]=
{
	{	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{-29556.8,	-1671.7,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{ -2340.6,	 3046.9,   1657.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{  1335.4,	-2305.1,   1246.7,	  674.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{	919.8,	  798.1,	211.3,	 -379.4,	100.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{  -227.4,	  354.6,	208.7,	 -136.5,   -168.3,	  -14.1,	  0.0,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{	 73.2,	   69.7,	 76.7,	 -151.2,	-14.9,	   14.6,	-86.3,		0.0,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{	 80.1,	  -74.5,	 -1.4,	   38.5,	 12.4,		9.5,	  5.7,		1.8,	  0.0,		0.0,	  0.0,		0.0,	  0.0},
	{	 24.9,		7.7,	-11.6,	   -6.9,	-18.2,	   10.0,	  9.2,	  -11.6,	 -5.2,		0.0,	  0.0,		0.0,	  0.0},
	{	  5.6,		9.9,	  3.5,	   -7.0,	  5.1,	  -10.8,	 -1.3,		8.8,	 -6.7,	   -9.1,	  0.0,		0.0,	  0.0},
	{	 -2.3,	   -6.3,	  1.6,	   -2.6,	  0.0,		3.1,	  0.4,		2.1,	  3.9,	   -0.1,	 -2.3,		0.0,	  0.0},
	{	  2.8,	   -1.6,	 -1.7,		1.7,	 -0.1,		0.1,	 -0.7,		0.7,	  1.8,		0.0,	  1.1,		4.1,	  0.0},
	{	 -2.4,	   -0.4,	  0.2,		0.8,	 -0.3,		1.1,	 -0.5,		0.4,	 -0.3,	   -0.3,	 -0.1,	   -0.3,	 -0.1}
};

static Mag_Real hnm_wmm2005[13][13]=
{
	
	{	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0, 5079.8,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0,-2594.7, -516.7,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0, -199.9,  269.3, -524.2,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0,  281.5, -226.0,  145.8, -304.7,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0,   42.4,  179.8, -123.0,  -19.5,  103.6,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0,  -20.3,   54.7,   63.6,  -63.4,   -0.1,   50.4,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0,  -61.5,  -22.4,	7.2,   25.4,   11.0,  -26.4,   -5.1,	0.0,	0.0,	0.0,	0.0,	0.0},
	{	0.0,   11.2,  -21.0,	9.6,  -19.8,   16.1,	7.7,  -12.9,   -0.2,	0.0,	0.0,	0.0,	0.0},
	{	0.0,  -20.1,   12.9,   12.6,   -6.7,   -8.1,	8.0,	2.9,   -7.9,	6.0,	0.0,	0.0,	0.0},
	{	0.0,	2.4,	0.2,	4.4,	4.8,   -6.5,   -1.1,   -3.4,   -0.8,   -2.3,   -7.9,	0.0,	0.0},
	{	0.0,	0.3,	1.2,   -0.8,   -2.5,	0.9,   -0.6,   -2.7,   -0.9,   -1.3,   -2.0,   -1.2,	0.0},
	{	0.0,   -0.4,	0.3,	2.4,   -2.6,	0.6,	0.3,	0.0,	0.0,	0.3,   -0.9,   -0.4,	0.8}
};

static Mag_Real gtnm_wmm2005[13][13]=
{
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 8.0, 10.6,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{-15.1, -7.8, -0.8,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
	{ 0.4, -2.6, -1.2, -6.5,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{-2.5,	2.8, -7.0,	6.2, -3.8,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{-2.8,	0.7, -3.2, -1.1,  0.1, -0.8,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{-0.7,	0.4, -0.3,	2.3, -2.1, -0.6,  1.4,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.2, -0.1, -0.3,	1.1,  0.6,	0.5, -0.4,	0.6,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.1,	0.3, -0.4,	0.3, -0.3,	0.2,  0.4, -0.7,  0.4,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0}
};

static Mag_Real htnm_wmm2005[13][13]=
{
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0, -20.9,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
	{ 0.0, -23.2, -14.6,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	5.0, -7.0, -0.6,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	2.2,  1.6,	5.8,  0.1,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  1.7,	2.1,  4.8, -1.1,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0, -0.6, -1.9, -0.4, -0.5, -0.3,  0.7,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.6,  0.4,	0.2,  0.3, -0.8, -0.2,	0.1,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0, -0.2,  0.1,	0.3,  0.4,	0.1, -0.2,	0.4,  0.4,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0}
};

/* igrf2005 constants */
static Mag_Real gnm_igrf2005[14][14]=
{
	{	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{-29556.8,-1671.8,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{ -2340.5, 3047.0, 1656.9,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{  1335.7,-2305.3, 1246.8,	674.4,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{	919.8,	798.2,	211.5, -379.5,	100.2,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{  -227.6,	354.4,	208.8, -136.6, -168.3,	-14.1,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{	 72.9,	 69.6,	 76.6, -151.1,	-15.0,	 14.7,	-86.4,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{	 79.8,	-74.4,	 -1.4,	 38.6,	 12.3,	  9.4,	  5.5,	  2.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{	 24.8,	  7.7,	-11.4,	 -6.8,	-18.0,	 10.0,	  9.4,	-11.4,	 -5.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0},
	{	  5.6,	  9.8,	  3.6,	 -7.0,	  5.0,	-10.8,	 -1.3,	  8.7,	 -6.7,	 -9.2,	  0.0,	  0.0,	  0.0,	  0.0},
	{	 -2.2,	 -6.3,	  1.6,	 -2.5,	 -0.1,	  3.0,	  0.3,	  2.1,	  3.9,	 -0.1,	 -2.2,	  0.0,	  0.0,	  0.0},
	{	  2.9,	 -1.6,	 -1.7,	  1.5,	 -0.2,	  0.2,	 -0.7,	  0.5,	  1.8,	  0.1,	  1.0,	  4.1,	  0.0,	  0.0},
	{	 -2.2,	 -0.3,	  0.3,	  0.9,	 -0.4,	  1.0,	 -0.4,	  0.5,	 -0.3,	 -0.4,	  0.0,	 -0.4,	  0.0,	  0.0},
	{	 -0.2,	 -0.9,	  0.3,	  0.3,	 -0.4,	  1.2,	 -0.4,	  0.7,	 -0.3,	  0.4,	 -0.1,	  0.4,	 -0.1,	 -0.3}
};

static Mag_Real hnm_igrf2005[14][14]=
{
	{	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{	0.0, 5080.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{	0.0, -2594.9, -516.7,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
	{	0.0, -200.4,269.3, -524.5,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{	0.0,281.4, -225.8,145.7, -304.7,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{	0.0, 42.7,179.8, -123.0,-19.5,103.6,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{	0.0,-20.2, 54.7, 63.7,-63.4,  0.0, 50.3,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{	0.0,-61.4,-22.5,  6.9, 25.4, 10.9,-26.4, -4.8,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{	0.0, 11.2,-21.0,  9.7,-19.8, 16.1,	7.7,-12.8, -0.1,  0.0,	0.0,  0.0,	0.0,  0.0},
	{	0.0,-20.1, 12.9, 12.7, -6.7, -8.1,	8.1,  2.9, -7.9,  5.9,	0.0,  0.0,	0.0,  0.0},
	{	0.0,  2.4,	0.2,  4.4,	4.7, -6.5, -1.0, -3.4, -0.9, -2.3, -8.0,  0.0,	0.0,  0.0},
	{	0.0,  0.3,	1.4, -0.7, -2.4,  0.9, -0.6, -2.7, -1.0, -1.5, -2.0, -1.4,	0.0,  0.0},
	{	0.0, -0.5,	0.3,  2.3, -2.7,  0.6,	0.4,  0.0,	0.0,  0.3, -0.8, -0.4,	1.0,  0.0},
	{	0.0, -0.7,	0.3,  1.7, -0.5, -1.0,	0.0,  0.7,	0.2,  0.6,	0.4, -0.2, -0.5, -1.0}
};

static Mag_Real gtnm_igrf2005[14][14]=
{
	{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{ 8.8,	10.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{-15.0,  -6.9,	-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{-0.3,	-3.1,  -0.9,  -6.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{-2.5, 2.8,  -7.1, 5.9,  -3.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{-2.6, 0.4,  -3.0,	-1.2, 0.2,	-0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{-0.8, 0.2,  -0.2, 2.1,  -2.1,	-0.4, 1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{-0.4, 0.0,  -0.2, 1.1, 0.6, 0.4,  -0.5, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{-0.2, 0.2,  -0.2, 0.2,  -0.2, 0.2, 0.5,  -0.7, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
	{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
};

static Mag_Real htnm_igrf2005[14][14]=
{
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0, -21.3,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
	{ 0.0, -23.3, -14.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0},
	{ 0.0,	5.4, -6.5, -2.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	2.0,  1.8,	5.6,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	0.1,  1.8,	2.0,  4.5, -1.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0, -0.4, -1.9, -0.4, -0.4, -0.2,  0.9,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	0.8,  0.4,	0.1,  0.2, -0.9, -0.3,	0.3,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0, -0.2,  0.2,	0.2,  0.4,	0.2, -0.3,	0.5,  0.4,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0},
	{ 0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0,  0.0,	0.0}
};

/* Convert date to Julian day	 2000-2099 */
Mag_Uint32 yymmdd_to_julian_days(int yy,int mm,int dd)
{
	Mag_Uint32 jd;
	
	yy = 2000 + (yy%100);
	jd = dd - 32075L + 1461L * (yy + 4800L + (mm - 14) / 12 ) / 4;
	jd = jd + 367L * (mm - 2 - (mm - 14) / 12*12) / 12;
	jd = jd - 3 * ((yy + 4900L + (mm - 14) / 12) / 100) / 4;
	
	return(jd);
} 

/* Convert degrees to radians */
Mag_Real deg_to_rad(Mag_Real deg)
{
	return deg*pi/180.;
}

/* Convert radians to degrees */
Mag_Real rad_to_deg(Mag_Real rad)
{
	return rad*180./pi;
}

/* 
* return variation (in radians) given geodetic latitude (radians), longitude
* (radians) ,height (m) and (Julian) date
* model=8 is WMM2005, 9 is IGRF2005
* N and E lat and long are positive, S and W negative
*/

void SGMagVar(Mag_Real lat, Mag_Real lon, Mag_Real h, 
			  Mag_Uint32 dat, float32* field)
{
	int model = 9; // 9 is IGRF2005
		
	int n,m,nmaxl;
	Mag_Real yearfrac,sr,r,theta,c,s,psi,fn,fn_0,B_r,B_theta,B_phi,X,Y,Z;
	Mag_Real sinpsi, cospsi, inv_s;
	Mag_Real mm,c1_n,c2_n,c3_n,tmp;
	
	static int been_here = 0;
	
	Mag_Real sinlat;
	Mag_Real coslat;

	Mag_Real norm;
	
	/* output field B_r,B_th,B_phi,B_x,B_y,B_z */
	h = h/1000.0;

	sinlat = sin(lat);
	coslat = cos(lat);

	/* convert to geocentric */ 
	sr = sqrt(a*a*coslat*coslat + b*b*sinlat*sinlat);
	/* sr is effective radius */
	theta = atan2(coslat * (h*sr + a*a), sinlat * (h*sr + b*b));
	
	/* theta is geocentric co-latitude */
	
	r = h*h + 2.0*h * sr +
		(a*a*a*a - ( a*a*a*a - b*b*b*b ) * sinlat*sinlat ) / 
		(a*a - (a*a - b*b) * sinlat*sinlat );
	
	r = sqrt(r);
	
	/* r is geocentric radial distance */
	c = cos(theta);
	s = sin(theta);
	/* protect against zero divide at geographic poles */
	inv_s =  1.0 / (s + (s == 0.)*1.0e-8); 
	
	/*zero out arrays */
	for ( n = 0; n <= nmax; n++ ) {
		for ( m = 0; m <= n; m++ ) {
			P[n][m] = 0;
			DP[n][m] = 0;
		}
	}
	
	/* diagonal elements */
	P[0][0] = 1;
	P[1][1] = s;
	DP[0][0] = 0;
	DP[1][1] = c;
	P[1][0] = c ;
	DP[1][0] = -s;
	
	/* these values will not change for subsequent function calls */
	if( !been_here ) {
		for ( n = 2; n <= nmax; n++ ) {
			root[n] = sqrt((2.0*n-1) / (2.0*n));
		}
		
		for ( m = 0; m <= nmax; m++ ) {
			mm = m*m;
			for ( n = max_mag(m + 1, 2); n <= nmax; n++ ) {
				roots[m][n][0] = sqrt((n-1)*(n-1) - mm);
				roots[m][n][1] = 1.0 / sqrt( n*n - mm);
			}
		}
		been_here = 1;
	}
	
	for ( n=2; n <= nmax; n++ ) {
		/*	double root = sqrt((2.0*n-1) / (2.0*n)); */
		P[n][n] = P[n-1][n-1] * s * root[n];
		DP[n][n] = (DP[n-1][n-1] * s + P[n-1][n-1] * c) * root[n];
	}
	
	/* lower triangle */
	for ( m = 0; m <= nmax; m++ ) {
		/*	double mm = m*m;  */
		for ( n = max_mag(m + 1, 2); n <= nmax; n++ ) {
			/* double root1 = sqrt((n-1)*(n-1) - mm); */
			/* double root2 = 1.0 / sqrt( n*n - mm);  */
			P[n][m] = (P[n-1][m] * c * (2.0*n-1) -
				P[n-2][m] * roots[m][n][0]) * roots[m][n][1];
			DP[n][m] = ((DP[n-1][m] * c - P[n-1][m] * s) *
				(2.0*n-1) - DP[n-2][m] * roots[m][n][0]) * roots[m][n][1];
		}
	}
	
	/* compute gnm, hnm at dat */
	nmaxl = 12;  /* models except IGRF2005 */
	switch(model) {
	case 8: 	 /* WMM2005 */
		yearfrac = (dat - yymmdd_to_julian_days(5,1,1)) / 365.25;
		for (n=1;n<=nmaxl;n++)
			for (m = 0;m<=nmaxl;m++) {
				gnm[n][m] = gnm_wmm2005[n][m] + yearfrac * gtnm_wmm2005[n][m];
				hnm[n][m] = hnm_wmm2005[n][m] + yearfrac * htnm_wmm2005[n][m];
			}
			break;
	case 9: 	 /* IGRF2005 */
		yearfrac = (dat - yymmdd_to_julian_days(5,1,1)) / 365.25;
		nmaxl = 13;
		for (n=1;n<=nmaxl;n++)
			for (m = 0;m<=nmaxl;m++) {
				gnm[n][m] = gnm_igrf2005[n][m] + yearfrac * gtnm_igrf2005[n][m];
				hnm[n][m] = hnm_igrf2005[n][m] + yearfrac * htnm_igrf2005[n][m];
			}
			break;		
	}
	
	/* compute sm (sin(m lon) and cm (cos(m lon)) */
	for (m = 0;m<=nmaxl;m++) {
		sm[m] = sin(m * lon);
		cm[m] = cos(m * lon);
	}
	
	/* compute B fields */
	B_r = 0.0;
	B_theta = 0.0;
	B_phi = 0.0;
	fn_0 = r_0/r;
	fn = fn_0 * fn_0;
	
	for ( n = 1; n <= nmaxl; n++ ) {
		c1_n=0;
		c2_n=0;
		c3_n=0;
		for ( m = 0; m <= n; m++ ) {
			tmp = (gnm[n][m] * cm[m] + hnm[n][m] * sm[m]); 
			c1_n += tmp * P[n][m];
			c2_n += tmp * DP[n][m];
			c3_n +=  m * (gnm[n][m] * sm[m] - hnm[n][m] * cm[m]) * P[n][m];
		}
		/* fn=pow(r_0/r,n+2.0);   */
		fn *= fn_0;
		B_r += (n + 1) * c1_n * fn;
		B_theta -= c2_n * fn;
		B_phi += c3_n * fn * inv_s;
	}
	
	/* Find geodetic field components: */
	psi = theta - (pi / 2.0 - lat);
	sinpsi = sin(psi);
	cospsi = cos(psi);
	X = -B_theta * cospsi - B_r * sinpsi;
	Y = B_phi;
	Z = B_theta * sinpsi - B_r * cospsi;
	
	//field[0]=B_r;
	//field[1]=B_theta;
	//field[2]=B_phi;
	norm = sqrt(X*X + Y*Y + Z*Z);
	field[0]=X/norm;
	field[1]=Y/norm;
	field[2]=Z/norm;   /* output fields */
	/* find variation in radians */
	/* return zero variation at magnetic pole X=Y=0. */
	/* E is positive */
	return; 
}

