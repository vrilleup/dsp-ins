// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           École Polytechnique de Montréal
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


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

#include "DSP2833x_Device.h"		// DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"	// DSP2833x Examples Include File

#include "matrix.h"		// matrix include file

#ifndef EKFILTER_H
#define EKFILTER_H

	// call this [1]
	extern void Init_Attitude(VEC* _f, VEC* _b, BOOL IsInited);	
	// call this [2]
	extern void timeUpdateStep(VEC* u_);
	// call this [3]
	extern void measureUpdateStep(VEC* z_, BOOL _IsGPSVelocity);
	// call this [4]
	extern void getX(VEC* out);

	extern void q2eul(VEC* q, float32* roll, float32* pitch, float32* yaw);
	extern void eul2q(float32 roll, float32 pitch, float32 yaw, VEC* q);	

	extern void Setdis(float32 dis0, float32 dis1, float32 dis2);
	extern void Setb0(VEC* _b0);
	extern void Setg(float32 _g);
	extern void SetdT(float32 _dT);
	extern void Update_b0(VEC* pos);
	extern void Update_Date(int yy,int mm,int dd);
	extern void SetEKFPassword(Uint32 u1, Uint32 u2, Uint32 u3, Uint32 u4, Uint32 u5, Uint32 u6, Uint32 u7);

	extern float32 g;

#endif
