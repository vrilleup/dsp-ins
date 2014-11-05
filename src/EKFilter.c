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

#include <math.h>
#include "IGRFWMM.h"						// IGRFWMM include file
#include "EKFilter.h"						// EKFilter include file
#include "INS_Func.h"

#pragma CODE_SECTION(EKFilter_Init, "xintfloadfuncs");
#pragma CODE_SECTION(State_Init, "xintfloadfuncs");
#pragma CODE_SECTION(Filter_Init, "xintfloadfuncs");
#pragma CODE_SECTION(setDim, "xintfloadfuncs");
#pragma CODE_SECTION(setSizeX, "xintfloadfuncs");
#pragma CODE_SECTION(setSizeU, "xintfloadfuncs");
#pragma CODE_SECTION(setSizeW, "xintfloadfuncs");
#pragma CODE_SECTION(setSizeZ, "xintfloadfuncs");
#pragma CODE_SECTION(setSizeV, "xintfloadfuncs");
#pragma CODE_SECTION(NoModification, "xintfloadfuncs");
#pragma CODE_SECTION(makeQ, "xintfloadfuncs");
#pragma CODE_SECTION(makeV, "xintfloadfuncs");
#pragma CODE_SECTION(makeR, "xintfloadfuncs");
#pragma CODE_SECTION(makeDZ, "xintfloadfuncs");
#pragma CODE_SECTION(sizeUpdate, "xintfloadfuncs");
#pragma CODE_SECTION(factor, "xintfloadfuncs");
#pragma CODE_SECTION(timeUpdate, "xintfloadfuncs");
#pragma CODE_SECTION(measureUpdate, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseAImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseWImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseQImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseHImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseVImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseRImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeAImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeWImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeQImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeHImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeVImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeRImpl, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseA, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseW, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseQ, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseH, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseV, "xintfloadfuncs");
#pragma CODE_SECTION(makeBaseR, "xintfloadfuncs");
#pragma CODE_SECTION(makeProcess, "xintfloadfuncs");
#pragma CODE_SECTION(makeA, "xintfloadfuncs");
#pragma CODE_SECTION(makeW, "xintfloadfuncs");
#pragma CODE_SECTION(makeCommonProcess, "xintfloadfuncs");
#pragma CODE_SECTION(makeMeasure, "xintfloadfuncs");
#pragma CODE_SECTION(makeH, "xintfloadfuncs");
#pragma CODE_SECTION(makeCommonMeasure, "xintfloadfuncs");
#pragma CODE_SECTION(Init_Attitude, "xintfloadfuncs");
#pragma CODE_SECTION(timeUpdateStep, "xintfloadfuncs");
#pragma CODE_SECTION(measureUpdateStep, "xintfloadfuncs");
#pragma CODE_SECTION(getX, "xintfloadfuncs");
#pragma CODE_SECTION(q2eul, "xintfloadfuncs");
#pragma CODE_SECTION(eul2q, "xintfloadfuncs");
#pragma CODE_SECTION(Setdis, "xintfloadfuncs");
#pragma CODE_SECTION(Setb0, "xintfloadfuncs");
#pragma CODE_SECTION(Setg, "xintfloadfuncs");
#pragma CODE_SECTION(SetdT, "xintfloadfuncs");
#pragma CODE_SECTION(Update_b0, "xintfloadfuncs");
#pragma CODE_SECTION(Update_Date, "xintfloadfuncs");
#pragma CODE_SECTION(getSizeX, "xintfloadfuncs");
#pragma CODE_SECTION(getSizeU, "xintfloadfuncs");
#pragma CODE_SECTION(getSizeW, "xintfloadfuncs");
#pragma CODE_SECTION(getSizeZ, "xintfloadfuncs");
#pragma CODE_SECTION(getSizeV, "xintfloadfuncs");
#pragma CODE_SECTION(SetEKFPassword, "xintfloadfuncs");

//! \internal 
//! Flag : \a n has changed
#define KALMAN_N_MODIFIED    1

//! \internal
//! Flag : \a nu has changed
#define KALMAN_NU_MODIFIED  (1<<1)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NW_MODIFIED  (1<<2)

//! \internal
//! Flag : \a m has changed
#define KALMAN_M_MODIFIED   (1<<3)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NV_MODIFIED  (1<<4)

//! \internal
//! Flag : \a P has changed
#define KALMAN_P_MODIFIED   (1<<5)

//! \internal
//! Mask : used to reset dimension flags
#define KALMAN_LOWMASK      ((1<<8) - 1)

//! \internal
//! Flag : \a A has changed
#define KALMAN_A_MODIFIED   (1<<8)

//! \internal
//! Flag : \a W has changed
#define KALMAN_W_MODIFIED   (1<<9)

//! \internal
//! Flag : \a Q has changed
#define KALMAN_Q_MODIFIED   (1<<10)

//! \internal
//! Flag : \a U has changed
#define KALMAN_U_MODIFIED   (1<<11)

//! \internal
//! Mask : used to reset time update matrix flags
#define KALMAN_MIDMASK      ( ((1<<3) - 1) << 8 )

//! \internal
//! Flag : \a H has changed
#define KALMAN_H_MODIFIED   (1<<12)

//! \internal
//! Flag : \a V has changed
#define KALMAN_V_MODIFIED   (1<<13)

//! \internal
//! Flag : \a R has changed
#define KALMAN_R_MODIFIED   (1<<14)

//! \internal
//! Mask : used to reset measure update matrix flags
#define KALMAN_HIGHMASK     ( ((1<<4) - 1) << 12 )

// the begin index of matrix or vector
#define BEG 0
// the NULL pointer
#define EKF_NULL 0
// the epsion of EKF
#define EKF_SMALL 1.1e-4

#define ISNOGPS  	(INS_Para.IsNoGPS)		// Bollean flag for using GPS
#define ISGPSVELO  	(INS_Para.IsUseGPSVelo)	// Bollean flag for GPS signal
#define ISUSEMAGNET	(INS_Para.IsUseMagnet)	// Bollean flag for using magnet measurements
#define IS2DCALI	(INS_Para.Is2DCali)		// 2D or 3D calibrate

#define MAGNET_R 		(INS_Para.R_Coeff_Magnet) 
#define VELOCITY_R 		(INS_Para.R_Coeff_Velocity) 
#define ACCEL_R			(INS_Para.R_Coeff_Acceleration) 

#define GYRO_Q      0.000001
#define ACCEL_Q     0.002
#define MAGNET_RMAX 10000.0  // a big coefficient as if the measurement is totally unreliable

// define data types
typedef Uint16 K_UINT_16;
typedef Uint32 K_UINT_32;
typedef float32 T;
typedef MAT   Matrix;
typedef VEC   Vector;

//BOOL OQ = TRUE;
//BOOL OVR = TRUE;
//BOOL DBG = FALSE;
K_UINT_16 flags = 0;    //!< Bitfield keeping track of modified matrices.

Vector *f = EKF_NULL;
Vector *b = EKF_NULL;
Vector *x0 = EKF_NULL;
Matrix *P0 = EKF_NULL;

Vector* x = EKF_NULL;	//!< Corrected state vector.
Vector* u = EKF_NULL;	//!< Input vector.
Vector* z = EKF_NULL;	//!< Predicted measurement vector.
Vector* dz = EKF_NULL;	//!< Innovation vector.
Matrix* A = EKF_NULL;	//!< A jacobian matrix.
Matrix* W = EKF_NULL;	//!< A jacobian matrix.
Matrix* Q = EKF_NULL;	//!< Process noise covariance matrix.
Matrix* H = EKF_NULL;	//!< A jacobian matrix.
Matrix* V = EKF_NULL;	//!< A jacobian matrix.
Matrix* R = EKF_NULL;	//!< Measurement noise covariance matrix.

K_UINT_32 n =  0;        			//!< Size of the state vector.
K_UINT_32 nu = 0;       			//!< Size of the input vector.
K_UINT_32 nw = 0;       			//!< Size of the process noise vector.
K_UINT_32  m = 0;       			//!< Size of the measurement vector.
K_UINT_32 nv = 0;       			//!< Size of the measurement noise vector.

Matrix* U = EKF_NULL;		//!< Cholesky factorization of P.
//Matrix* W_ = EKF_NULL;		//!< Modified version of \a W to whiten process noise.
//Matrix* Q_ = EKF_NULL;		//!< Modified version of \a Q to whiten process noise.
//Matrix* H_ = EKF_NULL;		//!< Modified version of \a H to whiten measure noise.
Matrix* R_ = EKF_NULL;		//!< Modified version of \a R to whiten measure noise.

Vector* a = EKF_NULL;		//!< Temporary vector.
Vector* d = EKF_NULL;		//!< Temporary vector.
Vector* v = EKF_NULL;		//!< Temporary vector.
K_UINT_32 nn = 0;        	//!< Number of columns of \a U

Matrix* _tmpP = EKF_NULL;		//!< Temporary matrix.
Vector* _tmpx = EKF_NULL;		//!  Temporary vector.

BOOL modified_ = FALSE;		//!< Boolean flag used by \c NoModification().

Vector* dis = EKF_NULL;		// GPS ant displacement
Vector* b0 = EKF_NULL;		// Local magnet field
T g;							// Local gravity
T dT;						// Delta T between two samples
K_UINT_32 date; 			// Date of the day

// temporary variables
T f1,f2,f3,w1,w2,w3,wd1,wd2,wd3,q0,q1,q2,q3;
T wt1,wt2,wt3,qt1,qt2,qt3,qm01,qm02,qm03,qm12,qm13,qm23;
T q0f1,q1f1,q2f1,q3f1;
T q0f2,q1f2,q2f2,q3f2;
T q0f3,q1f3,q2f3,q3f3;
T q0half,q1half,q2half,q3half;
T wt1one,wt2one,wt3one;
T q0dou,q1dou,q2dou,q3dou;

BOOL   IsRefreshRH = FALSE; // indicating if R or H should be updated

void EKFilter_Init(K_UINT_32 n_, K_UINT_32 nu_, 
					   K_UINT_32 nw_, K_UINT_32 m_, 
					   K_UINT_32 nv_);
void State_Init(Vector* x_, Matrix* P_);
void Filter_Init(void);
void setDim(K_UINT_32 n_, K_UINT_32 nu_, 
				   K_UINT_32 nw_, K_UINT_32 m_, K_UINT_32 nv_);
void setSizeX(K_UINT_32 n_);
void setSizeU(K_UINT_32 nu_);
void setSizeW(K_UINT_32 nw_);
void setSizeZ(K_UINT_32 m_);
void setSizeV(K_UINT_32 nv_);
//static void step(Vector* u_, Vector* z_);
void NoModification();
void makeQ();
void makeV();
void makeR();
void makeDZ();
void sizeUpdate();
void factor(Matrix* P_);
//static void upperInvert(Matrix* P_);
void timeUpdate();
void measureUpdate(T dz, T r);
void makeBaseAImpl();
void makeBaseWImpl();
void makeBaseQImpl();
void makeBaseHImpl();
void makeBaseVImpl();
void makeBaseRImpl();
void makeAImpl();
void makeWImpl();
void makeQImpl();
void makeHImpl();
void makeVImpl();
void makeRImpl();
void makeBaseA();
void makeBaseW();
void makeBaseQ();
void makeBaseH();
void makeBaseV();
void makeBaseR();
void makeProcess();
void makeA();
void makeW();
void makeCommonProcess();
void makeMeasure();
void makeH();
void makeCommonMeasure();

void EKFilter_Init(K_UINT_32 n_, K_UINT_32 nu_, 
				   K_UINT_32 nw_, K_UINT_32 m_, 
				   K_UINT_32 nv_)
{
	Filter_Init();
	setDim(n_, nu_, nw_, m_, nv_);
	//sizeUpdate();
}

void setDim(K_UINT_32 n_, K_UINT_32 nu_, 
			K_UINT_32 nw_, K_UINT_32 m_, K_UINT_32 nv_) 
{
	setSizeX(n_);
	setSizeU(nu_);
	setSizeW(nw_);
	setSizeZ(m_);
	setSizeV(nv_);
}

K_UINT_32 getSizeX()
{
	return n;
}

K_UINT_32 getSizeU() 
{
	return nu;
}

K_UINT_32 getSizeW()
{
	return nw;
}

K_UINT_32 getSizeZ() 
{
	return m;
}

K_UINT_32 getSizeV()
{
	return nv;
}

void setSizeX(K_UINT_32 n_) 
{
	// verify : n_ > 0
	
	if (n_ != n) {
		flags |= KALMAN_N_MODIFIED;
		n = n_;
	}
}

void setSizeU(K_UINT_32 nu_) 
{
	if (nu_ != nu) {
		flags |= KALMAN_NU_MODIFIED;
		nu = nu_;
	}
}

void setSizeW(K_UINT_32 nw_) 
{
	if (nw_ != nw) {
		flags |= KALMAN_NW_MODIFIED;
		nw = nw_;
	}
}

void setSizeZ(K_UINT_32 m_) 
{
	if (m_ != m) {
		flags |= KALMAN_M_MODIFIED;
		m = m_;
	}
}

void setSizeV(K_UINT_32 nv_) 
{
	if (nv_ != nv) {
		flags |= KALMAN_NV_MODIFIED;
		nv = nv_;
	}
}

void Filter_Init(void)
{
	b0 = v_resize(b0,3);
	dis = v_resize(dis,3);
	
	Update_Date(8,8,8);	// set data, One World, One Dream!
	
	SGMagVar(0.6977448, 2.030389, 115.481,
	date, b0->ve);		// set local magnet field of Beijing

	g = 9.80665;		// nominal gravity 
	dT = 0.01;			// for 100Hz
	dis->ve[0] = INS_Para.GPSAntDis0;	// set GPS antenna displacement
	dis->ve[1] = INS_Para.GPSAntDis1;
	dis->ve[2] = INS_Para.GPSAntDis2;
}

void State_Init(Vector* x_, Matrix* P_) 
{	
	// verify : (x_.size() == n && P_.nrow() == n && P_.ncol() == n)
	x  = v_copy(x_, x);
	_tmpP = m_copy(P_,_tmpP);
	flags |= KALMAN_P_MODIFIED;
}

/*
void step(Vector* u_, Vector* z_) 
{
	timeUpdateStep(u_);
	measureUpdateStep(z_);
}
*/

void timeUpdateStep(Vector* u_) 
{
	// verif : u_.size() == nu
	//K_UINT_32 i;
	//K_UINT_32 j, k;

	sizeUpdate();
	if (!(flags & KALMAN_U_MODIFIED))
	{
		return;
	}
	
	u = v_swap(u_,u);

	makeCommonProcess();
	makeAImpl();
	makeWImpl();
	makeQImpl();
	makeProcess();

	timeUpdate();

	u = v_swap(u_,u);
	
	flags &= ~KALMAN_MIDMASK;
}

void measureUpdateStep(Vector* z_, BOOL _IsGPSVelocity) 
{
	// verif : z_.size() == m
	K_UINT_32 i, j;
	//K_UINT_32 k;

	float32 norm1,norm2;	

	if ((ISGPSVELO && !_IsGPSVelocity) || 
		((!(ISGPSVELO)) && _IsGPSVelocity))
	{
		ISGPSVELO = _IsGPSVelocity;
		IsRefreshRH = TRUE;
	}
	
	if (IsRefreshRH)
	{
		makeBaseH();
		makeBaseR();
		IsRefreshRH = FALSE;
	}
	
	sizeUpdate();
	if (!(flags & KALMAN_U_MODIFIED))
	{
		return;
	}
	
	if (m == 0) {
		return;
	}
	
	makeCommonMeasure();
	makeHImpl();
	makeVImpl();
	makeRImpl();    
	makeMeasure();	
	
	// verif : nv != 0

	// adjust z_ for 2D cali magnet sensors
	/*
	if (IS2DCALI)
	{
		norm1 = sqrt(z->ve[3]*z->ve[3] + z->ve[4]*z->ve[4]);
		norm2 = sqrt(z_->ve[3]*z_->ve[3] + z_->ve[4]*z_->ve[4]);
		z_->ve[3] = z_->ve[3]*norm1/norm2;
		z_->ve[4] = z_->ve[4]*norm1/norm2;
		z_->ve[5] = 0.0;
	}
	*/
	
	for (i = BEG; i < m + BEG; ++i)
		dz->ve[i] = z_->ve[i] - z->ve[i];
	
	makeDZ();

	//if (OVR) 
	//{
	// verif : m == nv
	if (flags & ( KALMAN_V_MODIFIED | KALMAN_R_MODIFIED ) ) 
	{
		for (i = BEG; i < m + BEG; ++i)
			R_->me[i][i] = V->me[i][i]*V->me[i][i]*R->me[i][i];
	}

	_tmpx = v_resize(_tmpx,n); // dx : innovation
	_tmpx = v_zero(_tmpx);	// resize _tmpx and set zero

	for (i = BEG; i < m + BEG; ++i) 
	{
		for (j = BEG; j < n + BEG; ++j)
			a->ve[j] = H->me[i][j];
			
		measureUpdate(dz->ve[i], R_->me[i][i]);
	}
		
	for (i = BEG; i < n + BEG; ++i)
		x->ve[i] += _tmpx->ve[i];

	/*
	if (!OVR) {
		H.swap(H_);
	}
	*/
	
	flags &= ~KALMAN_HIGHMASK;
}

void getX(Vector* out)
{
	out = v_copy(x, out);
}


/*
void calculateP(Matrix* out)
{
if (!(flags & KALMAN_P_MODIFIED)) {

  _tmpP.resize(n, n);         // keep this resize
  
	for (K_UINT_32 i = BEG; i < n + BEG; ++i) {
	
	  _tmpP(i,i) = U(i,i);
	  
		for (K_UINT_32 j = i + 1; j < n + BEG; ++j) {
		
		  _tmpP(i,j)  = U(i,j)*U(j,j);
		  _tmpP(i,i) += U(i,j)*_tmpP(i,j);
		  
			for (K_UINT_32 k = j + 1; k < n + BEG; ++k) {
			_tmpP(i,j) += U(i,k)*U(j,k)*U(k,k);
			}
			
			  _tmpP(j,i) = _tmpP(i,j);
			  
				}
				
				  }
				  
					}
					
					  return _tmpP;
					  }
*/

void NoModification() 
{
	modified_ = FALSE;
}

void makeQ() 
{
	NoModification();
}

void makeV() 
{
	NoModification();
}

void makeR() 
{
	NoModification();
}

void makeDZ() 
{
	
}

void sizeUpdate()
{
	K_UINT_32 i,j;
	Uint16 p[4];
	Uint32 tmp;

	if (flags & KALMAN_N_MODIFIED) {
		A = m_resize(A, n, n);
		makeBaseAImpl();
	}
	
	if (flags & (KALMAN_N_MODIFIED | KALMAN_NW_MODIFIED) ) {
		nn = n + nw;
		a = v_resize(a,nn);
		v = v_resize(v,nn);
		d = v_resize(d,nn);
		/*
		if (!OQ)
		W_.resize(n, nw);
		*/
		W = m_resize(W, n, nw);
		makeBaseWImpl();
	}

	flags |= KALMAN_U_MODIFIED;

	// KALMAN_N_MODIFIED imply KALMAN_P_MODIFIED
	// => KALMAN_N_MODIFIED must not be set OR KALMAN_P_MODIFIED must be set
	// => NOT  KALMAN_N_MODIFIED  OR  KALMAN_P_MODIFIED  must be set
	// verif : (flags ^ KALMAN_N_MODIFIED) & 
	//              (KALMAN_N_MODIFIED | KALMAN_P_MODIFIED)
	
	if (flags & KALMAN_P_MODIFIED) { 
		// this covers the case of KALMAN_N_MODIFIED = true also
		
		// We have a new matrix P : let's factorize it and store it in U
		// First, resize U and copy P in its left part
		U = m_resize(U, n, nn);

		for (i = BEG; i < n + BEG; ++i)
			for (j = BEG; j < n + BEG; ++j)
				U->me[i][j] = _tmpP->me[i][j];
			
			// Factorize
			factor(U);			
	} else if (flags & KALMAN_NW_MODIFIED) {
		// KALMAN_N_MODIFIED is necessarily false, else KALMAN_P_MODIFIED
		// would have been true
		
		// Let's just copy U in temporary matrix _tmpP of the right size,
		// then swap the matrices
		
		_tmpP = m_resize(_tmpP, n, nn);
		
		for (i = BEG; i < n + BEG; ++i)
			for (j = i; j < n + BEG; ++j)
				_tmpP->me[i][j] = U->me[i][j];
			_tmpP = m_swap(U, _tmpP);
	}
	
	if (flags & KALMAN_NW_MODIFIED) {
	/*
	if (!OQ)
    Q_.resize(nw, nw);
		*/
		Q = m_resize(Q, nw, nw);
		makeBaseQImpl();
	}
	
	if (m != 0) {
		if (flags & (KALMAN_N_MODIFIED | KALMAN_M_MODIFIED) ) {
		/*
		if (!OVR)
		H_.resize(m, n);
			*/
			H = m_resize(H, m, n);
			makeBaseHImpl();
		}
		
		if (flags & (KALMAN_M_MODIFIED | KALMAN_NV_MODIFIED) ) {
			V = m_resize(V, m, nv);
			makeBaseVImpl();
		}
		
		if (flags & KALMAN_NV_MODIFIED) {
			R = m_resize(R, nv, nv);
			makeBaseRImpl();
		}
		
		if (flags & KALMAN_M_MODIFIED) {
			R_ = m_resize(R_, m, m);
			z = v_resize(z, m);
			dz = v_resize(dz, m);
		}
		
	}

	flags &= ~KALMAN_LOWMASK;
}

void factor(Matrix* P_) 
{
	// ne pas vérifier que P_.ncol() == P_.nrow(), comme ça, même si
	// nrow() < ncol(), on peut factoriser la sous-matrice carrée de P
	// Utile pour factoriser U
	
	T alpha, beta;
	K_UINT_32 i, j, k, N;
	
	N = P_->m;
	for(j = N - 1 + BEG; j > BEG; --j) {
		alpha = ((T)1.0)/P_->me[j][j];
		for(k = BEG; k < j; ++k) {
			beta = P_->me[k][j];
			P_->me[k][j] = alpha*beta;
			for(i = BEG; i <= k; ++i)
				P_->me[i][k] -= beta*P_->me[i][j];
		}
	}
}

/*
void upperInvert(Matrix* P_) 
{
	T val;
	K_UINT_32 i, j, k, N = P_->m;
	for (i = N - 2 + BEG; i != (K_UINT_32)(BEG-1); --i) { // intended overflow if BEG==0
		for (k = i + 1; k < N + BEG; ++k) {
			val = P_->me[i][k];
			for (j = i + 1; j <= k - 1; ++j)
				val += P_->me[i][j]*P_->me[k][j];
			P_->me[k][i] = -val;
		}
	}
}
*/

// U    u     U-D covariance matrix (n,nn)
// A    phi   transition matrix (F) (n,n)
// W    g     process noise matrix (G) (n,nw)
// Q    q     process noise variance vector (nw) Q = diag(q)
// a, v, d temporary vectors
// U is updated

void timeUpdate() 
{
	K_UINT_32 i, j, k;
	T sigma, dinv;
	
	// U = phi * U
	// d = diag(U)
	// 
	// This algo could be faster
	// if phi is known to be diagonal
	// It could be almost zapped if phi=I
	for(j = n - 1 + BEG; j > BEG; --j) {
		for(i = BEG; i <= j; ++i)
			d->ve[i] = U->me[i][j];
		for(i = BEG; i < n + BEG; ++i) {
			U->me[i][j] = A->me[i][j];
			for(k = BEG; k < j; ++k)
				U->me[i][j] += A->me[i][k]*d->ve[k];
		}
	}
	
	d->ve[BEG] = U->me[BEG][BEG];
	for(j = BEG; j < n + BEG; ++j)
		U->me[j][BEG] = A->me[j][BEG];
	
	// d(n+1:nn) = q 
	// U(:,n+1:nn) = G 
	for(i = BEG; i < nw + BEG; ++i) {
		d->ve[i+n] = Q->me[i][i];
		for(j = BEG; j < n + BEG; ++j)
			U->me[j][i+n] = W->me[j][i];
	}
	
	// Gram-Schmidt
	// Too hard to simplify
	for(j = n - 1 + BEG; j != (K_UINT_32)(BEG-1); --j) { // intended overflow if BEG==0
		sigma = 0.0;
		for(k = BEG; k < nn + BEG; ++k) {
			v->ve[k] = U->me[j][k];
			a->ve[k] = d->ve[k]*v->ve[k];
			sigma += v->ve[k]*a->ve[k];
		}
		U->me[j][j] = sigma;
		if(j == BEG || sigma == 0.0) continue;
		dinv = (1.0)/sigma;
		for(k = BEG; k < j; ++k) {
			sigma = (T)(0.0);
			for(i = BEG; i < nn + BEG; ++i) 
				sigma += U->me[k][i]*a->ve[i];
			sigma *= dinv;
			for(i = BEG; i < nn + BEG; ++i) 
				U->me[k][i] -= sigma*v->ve[i];
			U->me[j][k] = sigma;
		}
	}
	
	// U = transpose(U)
	for(j = BEG + 1; j < n + BEG; ++j)
		for(i = BEG; i < j; ++i)
			U->me[i][j] = U->me[j][i];
}

// x     a priori estimate vector (n)
// U     a priori U-D covariance matrix (n,nn)
// dz    measurement diff (z - ax) (scalar)
// a     measurement coefficients vector (n) (a row of A, which is H)
//          a is destroyed
// r     measurement variance
// d is a temporary vector
// x and U are updated
// a is destroyed

void measureUpdate(T dz, T r) 
{
	K_UINT_32 i, j, k;
	T alpha, gamma, beta, lambda;
	
	// dz = dz - Hdx
	for (j = BEG; j < n + BEG; ++j)
		dz -= a->ve[j]*_tmpx->ve[j];
	
	// d = D * transpose(U) * a
	// a =     transpose(U) * a
	//
	// This algo could be faster
	// if A is known to be diagonal or I
	for(j = n - 1 + BEG; j > BEG; --j) 
	{
		for(k = BEG; k < j; ++k)
			a->ve[j] += U->me[k][j]*a->ve[k];
		d->ve[j] = U->me[j][j]*a->ve[j];
	}
	d->ve[BEG] = U->me[BEG][BEG]*a->ve[BEG];
	
	// UDU
	// Too hard to simplify
	alpha = r+d->ve[BEG]*a->ve[BEG];
	gamma = 1.0/alpha;
	U->me[BEG][BEG] = r*gamma*U->me[BEG][BEG];
	for(j = BEG + 1; j < n + BEG; ++j) 
	{
		beta = alpha;
		alpha += d->ve[j]*a->ve[j];
		lambda = -a->ve[j]*gamma;
		gamma = 1.0/alpha;
		U->me[j][j] *= beta*gamma;
		for(i = BEG; i < j; ++i) 
		{
			beta = U->me[i][j];
			U->me[i][j] = beta+d->ve[i]*lambda;
			d->ve[i] += d->ve[j]*beta;
		}
	}
	
	// dx = dx + K(dz - Hdx)
	dz *= gamma;
	for(j = BEG; j < n + BEG; ++j)
		_tmpx->ve[j] += d->ve[j]*dz;
}

void makeBaseAImpl() 
{
	modified_ = TRUE;
	makeBaseA();
	if (modified_)
		flags |= KALMAN_A_MODIFIED;
}

void makeBaseWImpl() 
{
	modified_ = TRUE;
	makeBaseW();
	if (modified_)
		flags |= KALMAN_W_MODIFIED;    
}

void makeBaseQImpl() 
{
	modified_ = TRUE;
	makeBaseQ();
	if (modified_)
		flags |= KALMAN_Q_MODIFIED;    
}

void makeBaseHImpl() 
{
	modified_ = TRUE;
	makeBaseH();
	if (modified_)
		flags |= KALMAN_H_MODIFIED;    
}

void makeBaseVImpl() 
{
	modified_ = TRUE;
	makeBaseV();
	if (modified_)
		flags |= KALMAN_V_MODIFIED;    
}

void makeBaseRImpl() 
{
	modified_ = TRUE;
	makeBaseR();
	if (modified_)
		flags |= KALMAN_R_MODIFIED;    
}

void makeAImpl() 
{
	modified_ = TRUE;
	makeA();
	if (modified_)
		flags |= KALMAN_A_MODIFIED;    
}

void makeWImpl() 
{
	modified_ = TRUE;
	makeW();
	if (modified_)
		flags |= KALMAN_W_MODIFIED;    
}

void makeQImpl() 
{
	modified_ = TRUE;
	makeQ();
	if (modified_)
		flags |= KALMAN_Q_MODIFIED;    
}

void makeHImpl() 
{
	modified_ = TRUE;
	makeH();
	if (modified_)
		flags |= KALMAN_H_MODIFIED;    
}

void makeVImpl() 
{
	modified_ = TRUE;
	makeV();
	if (modified_)
		flags |= KALMAN_V_MODIFIED;    
}

void makeRImpl() 
{
	modified_ = TRUE;
	makeR();
	if (modified_)
		flags |= KALMAN_R_MODIFIED;    
}

void Setdis(T dis0, T dis1, T dis2)
{
	if (dis->dim == 3)
	{
		dis->ve[0] = dis0;
		dis->ve[1] = dis1;
		dis->ve[2] = dis2;
	}
}

void Setb0(Vector* _b0)
{
	b0 = v_copy(_b0, b0);	
}

void Setg(T _g)
{
	g = _g;
}

void SetdT(T _dT)
{
	dT = _dT;
}

void Update_b0(Vector* pos)
{
	b0 = v_resize(b0,3);

	SGMagVar(pos->ve[0], pos->ve[1], pos->ve[2],
	date, b0->ve);
}

void Update_Date(int yy,int mm,int dd)
{
	date = yymmdd_to_julian_days(yy,mm,dd);
}

/*
	equations of EKF for INS/GSP (see the matlab .m file for details)
	
	x(k) = f[x(k-1),u(k-1)] + A*delta_x(k-1) + W*w(k-1)
	z(k) = h[x(k)] + H*delta_x(k) + V*v(k)

	Q = E[w(k)'w(k)]
	R = E[v(k)'v(k)]

*************************
*************************
f[x(k-1),u(k-1)] =
[   (-1/2*w1+1/2*wd1)*q1+(-1/2*w2+1/2*wd2)*q2+(-1/2*w3+1/2*wd3)*q3;
      (1/2*w1-1/2*wd1)*q0+(1/2*w3-1/2*wd3)*q2+(-1/2*w2+1/2*wd2)*q3;
      (1/2*w2-1/2*wd2)*q0+(-1/2*w3+1/2*wd3)*q1+(1/2*w1-1/2*wd1)*q3;
      (1/2*w3-1/2*wd3)*q0+(1/2*w2-1/2*wd2)*q1+(-1/2*w1+1/2*wd1)*q2;
   -(1-2*q2^2-2*q3^2)*f1-(2*q1*q2-2*q0*q3)*f2-(2*q1*q3+2*q0*q2)*f3;
   -(2*q1*q2+2*q0*q3)*f1-(1-2*q1^2-2*q3^2)*f2-(2*q2*q3-2*q0*q1)*f3;
 -(2*q1*q3-2*q0*q2)*f1-(2*q2*q3+2*q0*q1)*f2-(1-2*q1^2-2*q2^2)*f3+g;
                                                                 0;
                                                                 0;
                                                                 0];
*************************
*************************
A =                                                                 
eye(10) + 
dT*[                    0,          -1/2*w1+1/2*wd1,          -1/2*w2+1/2*wd2,          -1/2*w3+1/2*wd3,                        0,                        0,                        0,                   1/2*q1,                   1/2*q2,                   1/2*q3;
           1/2*w1-1/2*wd1,                        0,           1/2*w3-1/2*wd3,          -1/2*w2+1/2*wd2,                        0,                        0,                        0,                  -1/2*q0,                   1/2*q3,                  -1/2*q2;
           1/2*w2-1/2*wd2,          -1/2*w3+1/2*wd3,                        0,           1/2*w1-1/2*wd1,                        0,                        0,                        0,                  -1/2*q3,                  -1/2*q0,                   1/2*q1;
           1/2*w3-1/2*wd3,           1/2*w2-1/2*wd2,          -1/2*w1+1/2*wd1,                        0,                        0,                        0,                        0,                   1/2*q2,                  -1/2*q1,                  -1/2*q0;
          2*q3*f2-2*q2*f3,         -2*q2*f2-2*q3*f3,  4*q2*f1-2*q1*f2-2*q0*f3,  4*q3*f1+2*q0*f2-2*q1*f3,                        0,                        0,                        0,                        0,                        0,                        0;
         -2*q3*f1+2*q1*f3, -2*q2*f1+4*q1*f2+2*q0*f3,         -2*q1*f1-2*q3*f3, -2*q0*f1+4*q3*f2-2*q2*f3,                        0,                        0,                        0,                        0,                        0,                        0;
          2*q2*f1-2*q1*f2, -2*q3*f1-2*q0*f2+4*q1*f3,  2*q0*f1-2*q3*f2+4*q2*f3,         -2*q1*f1-2*q2*f2,                        0,                        0,                        0,                        0,                        0,                        0;
                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0;
                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0;
                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0,                        0];
*************************
*************************
W =
  dT*[    -1/2*q1,          -1/2*q2,          -1/2*q3,                0,                0,                0;
           1/2*q0,          -1/2*q3,           1/2*q2,                0,                0,                0;
           1/2*q3,           1/2*q0,          -1/2*q1,                0,                0,                0;
          -1/2*q2,           1/2*q1,           1/2*q0,                0,                0,                0;
                0,                0,                0, -1+2*q2^2+2*q3^2, -2*q1*q2+2*q0*q3, -2*q1*q3-2*q0*q2;
                0,                0,                0, -2*q1*q2-2*q0*q3, -1+2*q1^2+2*q3^2, -2*q2*q3+2*q0*q1;
                0,                0,                0, -2*q1*q3+2*q0*q2, -2*q2*q3-2*q0*q1, -1+2*q1^2+2*q2^2;
                0,                0,                0,                0,                0,                0;
                0,                0,                0,                0,                0,                0;
                0,                0,                0,                0,                0,                0];
*************************
*************************
h[x(k)] =
[v1+((2*q1*q2-2*q0*q3)*(w3-wd3)+(2*q1*q3+2*q0*q2)*(-w2+wd2))*d1+((1-2*q2^2-2*q3^2)*(-w3+wd3)+(2*q1*q3+2*q0*q2)*(w1-wd1))*d2+((1-2*q2^2-2*q3^2)*(w2-wd2)+(2*q1*q2-2*q0*q3)*(-w1+wd1))*d3;
 v2+((1-2*q1^2-2*q3^2)*(w3-wd3)+(2*q2*q3-2*q0*q1)*(-w2+wd2))*d1+((2*q1*q2+2*q0*q3)*(-w3+wd3)+(2*q2*q3-2*q0*q1)*(w1-wd1))*d2+((2*q1*q2+2*q0*q3)*(w2-wd2)+(1-2*q1^2-2*q3^2)*(-w1+wd1))*d3;
 v3+((2*q2*q3+2*q0*q1)*(w3-wd3)+(1-2*q1^2-2*q2^2)*(-w2+wd2))*d1+((2*q1*q3-2*q0*q2)*(-w3+wd3)+(1-2*q1^2-2*q2^2)*(w1-wd1))*d2+((2*q1*q3-2*q0*q2)*(w2-wd2)+(2*q2*q3+2*q0*q1)*(-w1+wd1))*d3;
 (1-2*q2^2-2*q3^2)*bm1+(2*q1*q2+2*q0*q3)*bm2+(2*q1*q3-2*q0*q2)*bm3;
 (2*q1*q2-2*q0*q3)*bm1+(1-2*q1^2-2*q3^2)*bm2+(2*q2*q3+2*q0*q1)*bm3;
 (2*q1*q3+2*q0*q2)*bm1+(2*q2*q3-2*q0*q1)*bm2+(1-2*q1^2-2*q2^2)*bm3];
*************************
*************************
H = 
[(-2*q3*(w3-wd3)+2*q2*(-w2+wd2))*d1+2*q2*(w1-wd1)*d2-2*q3*(-w1+wd1)*d3,                                     (2*q2*(w3-wd3)+2*q3*(-w2+wd2))*d1+2*q3*(w1-wd1)*d2+2*q2*(-w1+wd1)*d3,  (2*q1*(w3-wd3)+2*q0*(-w2+wd2))*d1+(-4*q2*(-w3+wd3)+2*q0*(w1-wd1))*d2+(-4*q2*(w2-wd2)+2*q1*(-w1+wd1))*d3, (-2*q0*(w3-wd3)+2*q1*(-w2+wd2))*d1+(-4*q3*(-w3+wd3)+2*q1*(w1-wd1))*d2+(-4*q3*(w2-wd2)-2*q0*(-w1+wd1))*d3,                                                                                                        1,                                                                                                        0,                                                                                                        0,                                                               (-2*q1*q3-2*q0*q2)*d2+(2*q1*q2-2*q0*q3)*d3,                                                               (2*q1*q3+2*q0*q2)*d1+(-1+2*q2^2+2*q3^2)*d3,                                                               (-2*q1*q2+2*q0*q3)*d1+(1-2*q2^2-2*q3^2)*d2;
 -2*q1*(-w2+wd2)*d1+(2*q3*(-w3+wd3)-2*q1*(w1-wd1))*d2+2*q3*(w2-wd2)*d3,   (-4*q1*(w3-wd3)-2*q0*(-w2+wd2))*d1+(2*q2*(-w3+wd3)-2*q0*(w1-wd1))*d2+(2*q2*(w2-wd2)-4*q1*(-w1+wd1))*d3,                                     2*q3*(-w2+wd2)*d1+(2*q1*(-w3+wd3)+2*q3*(w1-wd1))*d2+2*q1*(w2-wd2)*d3,   (-4*q3*(w3-wd3)+2*q2*(-w2+wd2))*d1+(2*q0*(-w3+wd3)+2*q2*(w1-wd1))*d2+(2*q0*(w2-wd2)-4*q3*(-w1+wd1))*d3,                                                                                                        0,                                                                                                        1,                                                                                                        0,                                                               (-2*q2*q3+2*q0*q1)*d2+(1-2*q1^2-2*q3^2)*d3,                                                               (2*q2*q3-2*q0*q1)*d1+(-2*q1*q2-2*q0*q3)*d3,                                                               (-1+2*q1^2+2*q3^2)*d1+(2*q1*q2+2*q0*q3)*d2;
 2*q1*(w3-wd3)*d1-2*q2*(-w3+wd3)*d2+(-2*q2*(w2-wd2)+2*q1*(-w1+wd1))*d3,    (2*q0*(w3-wd3)-4*q1*(-w2+wd2))*d1+(2*q3*(-w3+wd3)-4*q1*(w1-wd1))*d2+(2*q3*(w2-wd2)+2*q0*(-w1+wd1))*d3,  (2*q3*(w3-wd3)-4*q2*(-w2+wd2))*d1+(-2*q0*(-w3+wd3)-4*q2*(w1-wd1))*d2+(-2*q0*(w2-wd2)+2*q3*(-w1+wd1))*d3,                                     2*q2*(w3-wd3)*d1+2*q1*(-w3+wd3)*d2+(2*q1*(w2-wd2)+2*q2*(-w1+wd1))*d3,                                                                                                        0,                                                                                                        0,                                                                                                        1,                                                               (-1+2*q1^2+2*q2^2)*d2+(2*q2*q3+2*q0*q1)*d3,                                                               (1-2*q1^2-2*q2^2)*d1+(-2*q1*q3+2*q0*q2)*d3,                                                               (-2*q2*q3-2*q0*q1)*d1+(2*q1*q3-2*q0*q2)*d2;
            2*q3*bm2-2*q2*bm3,                                                                                        2*q2*bm2+2*q3*bm3,                                                                              -4*q2*bm1+2*q1*bm2-2*q0*bm3,                                                                              -4*q3*bm1+2*q0*bm2+2*q1*bm3,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0;
           -2*q3*bm1+2*q1*bm3,                                                                               2*q2*bm1-4*q1*bm2+2*q0*bm3,                                                                                        2*q1*bm1+2*q3*bm3,                                                                              -2*q0*bm1-4*q3*bm2+2*q2*bm3,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0;
            2*q2*bm1-2*q1*bm2,                                                                               2*q3*bm1-2*q0*bm2-4*q1*bm3,                                                                               2*q0*bm1+2*q3*bm2-4*q2*bm3,                                                                                        2*q1*bm1+2*q2*bm2,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0,                                                                                                        0];

*************************
*************************
V = eye(6);
*/

void makeBaseA()
{
	A->me[0][0] = 1.0;
	//A->me[0][1] = -wt1*dT;
	//A->me[0][2] = -wt2*dT;
	//A->me[0][3] = -wt3*dT;
	A->me[0][4] = 0.0;
	A->me[0][5] = 0.0;
	A->me[0][6] = 0.0;
	//A->me[0][7] = q1half*dT;
	//A->me[0][8] = q2half*dT;
	//A->me[0][9] = q3half*dT;

	//A->me[1][0] = wt1*dT;
	A->me[1][1] = 1.0;
	//A->me[1][2] = wt3*dT;
	//A->me[1][3] = -wt2*dT;
	A->me[1][4] = 0.0;
	A->me[1][5] = 0.0;
	A->me[1][6] = 0.0;
	//A->me[1][7] = -q0half*dT;
	//A->me[1][8] = q3half*dT;
	//A->me[1][9] = -q2half*dT;

	//A->me[2][0] = wt2*dT;
	//A->me[2][1] = -wt3*dT;
	A->me[2][2] = 1.0;
	//A->me[2][3] = wt1*dT;
	A->me[2][4] = 0.0;
	A->me[2][5] = 0.0;
	A->me[2][6] = 0.0;
	//A->me[2][7] = -q3half*dT;
	//A->me[2][8] = -q0half*dT;
	//A->me[2][9] = q1half*dT;
	
	//A->me[3][0] = wt3*dT;
	//A->me[3][1] = wt2*dT;
	//A->me[3][2] = -wt1*dT;
	A->me[3][3] = 1.0;
	A->me[3][4] = 0.0;
	A->me[3][5] = 0.0;
	A->me[3][6] = 0.0;
	//A->me[3][7] = q2half*dT;
	//A->me[3][8] = -q1half*dT;
	//A->me[3][9] = -q0half*dT;

	//A->me[4][0] = (q3f2-q2f3)*dT;
	//A->me[4][1] = (-q2f2-q3f3)*dT;
	//A->me[4][2] = (2.0*q2f1-q1f2-q0f3)*dT;
	//A->me[4][3] = (2.0*q3f1+q0f2-q1f3)*dT;
	A->me[4][4] = 1.0;
	A->me[4][5] = 0.0;
	A->me[4][6] = 0.0;
	A->me[4][7] = 0.0;
	A->me[4][8] = 0.0;
	A->me[4][9] = 0.0;

	//A->me[5][0] = (-q3f1+q1f3)*dT;
	//A->me[5][1] = (-q2f1+2.0*q1f2+q0f3)*dT;
	//A->me[5][2] = (-q1f1-q3f3)*dT;
	//A->me[5][3] = (-q0f1+2.0*q3f2-q2f3)*dT;
	A->me[5][4] = 0.0;
	A->me[5][5] = 1.0;
	A->me[5][6] = 0.0;
	A->me[5][7] = 0.0;
	A->me[5][8] = 0.0;
	A->me[5][9] = 0.0;
	
	//A->me[6][0] = (q2f1-q1f2)*dT;
	//A->me[6][1] = (-q3f1-q0f2+2.0*q1f3)*dT;
	//A->me[6][2] = (q0f1-q3f2+2.0*q2f3)*dT;
	//A->me[6][3] = (-q1f1-q2f2)*dT;
	A->me[6][4] = 0.0;
	A->me[6][5] = 0.0;
	A->me[6][6] = 1.0;
	A->me[6][7] = 0.0;
	A->me[6][8] = 0.0;
	A->me[6][9] = 0.0;	

	A->me[7][0] = 0.0;
	A->me[7][1] = 0.0;
	A->me[7][2] = 0.0;
	A->me[7][3] = 0.0;
	A->me[7][4] = 0.0;
	A->me[7][5] = 0.0;
	A->me[7][6] = 0.0;
	A->me[7][7] = 1.0;
	A->me[7][8] = 0.0;
	A->me[7][9] = 0.0;	

	A->me[8][0] = 0.0;
	A->me[8][1] = 0.0;
	A->me[8][2] = 0.0;
	A->me[8][3] = 0.0;
	A->me[8][4] = 0.0;
	A->me[8][5] = 0.0;
	A->me[8][6] = 0.0;
	A->me[8][7] = 0.0;
	A->me[8][8] = 1.0;
	A->me[8][9] = 0.0;

	A->me[9][0] = 0.0;
	A->me[9][1] = 0.0;
	A->me[9][2] = 0.0;
	A->me[9][3] = 0.0;
	A->me[9][4] = 0.0;
	A->me[9][5] = 0.0;
	A->me[9][6] = 0.0;
	A->me[9][7] = 0.0;
	A->me[9][8] = 0.0;
	A->me[9][9] = 1.0;	
}

void makeBaseW()
{
	int i,j;

	for (i=0;i<W->m;i++)
		for (j=0;j<W->n;j++)
			W->me[i][j] = 0.0;
}

void makeBaseQ()
{
	int i,j;
	for (i = 0; i<Q->m; i++)
		for (j=0; j<Q->n; j++)
			Q->me[i][j] = 0.0;

	Q->me[0][0] = GYRO_Q;	// gyro x
	Q->me[1][1] = GYRO_Q;	// gyro y
	Q->me[2][2] = GYRO_Q;	// gyro z
	Q->me[3][3] = ACCEL_Q;	// accelerometer x
	Q->me[4][4] = ACCEL_Q;	// accelerometer y
	Q->me[5][5] = ACCEL_Q;	// accelerometer z
}

void makeBaseH()
{
	int i,j;
	
	if (ISGPSVELO && (!(ISNOGPS)))
	{
		H->me[0][4] = 1.0;  
		H->me[0][5] = 0.0;  
		H->me[0][6] = 0.0;
		H->me[1][4] = 0.0;  
		H->me[1][5] = 1.0;  
		H->me[1][6] = 0.0;
		H->me[2][4] = 0.0;  
		H->me[2][5] = 0.0;  
		H->me[2][6] = 1.0;
		H->me[3][4] = 0.0;  
		H->me[3][5] = 0.0;  
		H->me[3][6] = 0.0;  
		H->me[3][7] = 0.0;  
		H->me[3][8] = 0.0;  
		H->me[3][9] = 0.0;
		H->me[4][4] = 0.0;  
		H->me[4][5] = 0.0;  
		H->me[4][6] = 0.0;  
		H->me[4][7] = 0.0;  
		H->me[4][8] = 0.0;  
		H->me[4][9] = 0.0;
		H->me[5][4] = 0.0;  
		H->me[5][5] = 0.0;  
		H->me[5][6] = 0.0;  
		H->me[5][7] = 0.0;  
		H->me[5][8] = 0.0;  
		H->me[5][9] = 0.0;
	}
	else
	{
		// observe acceleration instead of velocity
		for (i=0; i<6; i++)
			for (j=4; j<10; j++)
				H->me[i][j] = 0.0;

		H->me[2][0] = 0.0;
		H->me[2][3] = 0.0;
	}
	
}

void makeBaseV()
{
	int i;
	for (i = 0; i<V->m; i++)
		V->me[i][i] = 1.0;
}

void makeBaseR()
{
	int i,j;
	for (i = 0; i<R->m; i++)
		for (j=0; j<R->m; j++)
				R->me[i][j] = 0.0;

	if (ISGPSVELO && (!(ISNOGPS)))
	{
		R->me[0][0] = VELOCITY_R;	// velocity x
		R->me[1][1] = VELOCITY_R;	// velocity y
		R->me[2][2] = VELOCITY_R;	// velocity z
	}
	else
	{
		R->me[0][0] = ACCEL_R;		// acceleration x
		R->me[1][1] = ACCEL_R;		// acceleration y
		R->me[2][2] = ACCEL_R;		// acceleration z
	}
	
	if (ISUSEMAGNET)
	{
		R->me[3][3] = MAGNET_R;	// magnet x
		R->me[4][4] = MAGNET_R;	// magnet y
		if (IS2DCALI)
		{
			R->me[5][5] = MAGNET_RMAX;	// magnet z
		}
		else
		{
			R->me[5][5] = MAGNET_R;	// magnet z
		}
	}
	else
	{
		R->me[3][3] = MAGNET_RMAX;	// magnet x
		R->me[4][4] = MAGNET_RMAX;	// magnet y
		R->me[5][5] = MAGNET_RMAX;	// magnet z
	}
	
}

void makeProcess()
{
	T qsqrt;
	
	if ((!(ISGPSVELO)) || ISNOGPS)
	{
		// force velocity to zero when no GPS signal
		x->ve[4] = 0.0;
		x->ve[5] = 0.0;
		x->ve[6] = 0.0;
	}
	
	x->ve[0] += (-wt1*q1-wt2*q2-wt3*q3)*dT;
	x->ve[1] += ( wt1*q0+wt3*q2-wt2*q3)*dT;
	x->ve[2] += ( wt2*q0-wt3*q1+wt1*q3)*dT;
	x->ve[3] += ( wt3*q0+wt2*q1-wt1*q2)*dT;
	x->ve[4] += (-qt1*f1-(qm12-qm03)*f2-(qm13+qm02)*f3)*dT;
	x->ve[5] += (-(qm12+qm03)*f1-qt2*f2-(qm23-qm01)*f3)*dT;
	x->ve[6] += (-(qm13-qm02)*f1-(qm23+qm01)*f2-qt3*f3+g)*dT;
	//x->ve[7]; // unchanged
	//x->ve[8]; // unchanged
	//x->ve[9]; // unchanged

	qsqrt = sqrt(x->ve[0]*x->ve[0] + x->ve[1]*x->ve[1]
			+ x->ve[2]*x->ve[2] + x->ve[3]*x->ve[3]);

	x->ve[0] = x->ve[0]/qsqrt;
	x->ve[1] = x->ve[1]/qsqrt;
	x->ve[2] = x->ve[2]/qsqrt;
	x->ve[3] = x->ve[3]/qsqrt;

}

void makeA()
{
	A->me[0][1] = -wt1*dT;
	A->me[0][2] = -wt2*dT;
	A->me[0][3] = -wt3*dT;
	A->me[0][7] = q1half*dT;
	A->me[0][8] = q2half*dT;
	A->me[0][9] = q3half*dT;

	A->me[1][0] = wt1*dT;
	A->me[1][2] = wt3*dT;
	A->me[1][3] = -wt2*dT;
	A->me[1][7] = -q0half*dT;
	A->me[1][8] = q3half*dT;
	A->me[1][9] = -q2half*dT;

	A->me[2][0] = wt2*dT;
	A->me[2][1] = -wt3*dT;
	A->me[2][3] = wt1*dT;
	A->me[2][7] = -q3half*dT;
	A->me[2][8] = -q0half*dT;
	A->me[2][9] = q1half*dT;
	
	A->me[3][0] = wt3*dT;
	A->me[3][1] = wt2*dT;
	A->me[3][2] = -wt1*dT;
	A->me[3][7] = q2half*dT;
	A->me[3][8] = -q1half*dT;
	A->me[3][9] = -q0half*dT;

	A->me[4][0] = (q3f2-q2f3)*dT;
	A->me[4][1] = (-q2f2-q3f3)*dT;
	A->me[4][2] = (2.0*q2f1-q1f2-q0f3)*dT;
	A->me[4][3] = (2.0*q3f1+q0f2-q1f3)*dT;

	A->me[5][0] = (-q3f1+q1f3)*dT;
	A->me[5][1] = (-q2f1+2.0*q1f2+q0f3)*dT;
	A->me[5][2] = (-q1f1-q3f3)*dT;
	A->me[5][3] = (-q0f1+2.0*q3f2-q2f3)*dT;
	
	A->me[6][0] = (q2f1-q1f2)*dT;
	A->me[6][1] = (-q3f1-q0f2+2.0*q1f3)*dT;
	A->me[6][2] = (q0f1-q3f2+2.0*q2f3)*dT;
	A->me[6][3] = (-q1f1-q2f2)*dT;
	
}

void makeW()
{
	W->me[0][0] = -q1half*dT;
	W->me[0][1] = -q2half*dT;
	W->me[0][2] = -q3half*dT;

	W->me[1][0] = q0half*dT;
	W->me[1][1] = -q3half*dT;
	W->me[1][2] = q2half*dT;

	W->me[2][0] = q3half*dT;
	W->me[2][1] = q0half*dT;
	W->me[2][2] = -q1half*dT;

	W->me[3][0] = -q2half*dT;
	W->me[3][1] = q1half*dT;
	W->me[3][2] = q0half*dT;

	W->me[4][3] = (-qt1)*dT;
	W->me[4][4] = (-qm12+qm03)*dT;
	W->me[4][5] = (-qm13-qm02)*dT;	

	W->me[5][3] = (-qm12-qm03)*dT;
	W->me[5][4] = (-qt2)*dT;
	W->me[5][5] = (-qm23+qm01)*dT;	

	W->me[6][3] = (-qm13+qm02)*dT;
	W->me[6][4] = (-qm23-qm01)*dT;
	W->me[6][5] = (-qt3)*dT;
}

void makeCommonProcess()
{
	w1 = u->ve[0];
	w2 = u->ve[1];
	w3 = u->ve[2];
	f1 = u->ve[3];
	f2 = u->ve[4];
	f3 = u->ve[5];

	q0 = x->ve[0];
	q1 = x->ve[1];
	q2 = x->ve[2];
	q3 = x->ve[3];

	wd1 = x->ve[7];
	wd2 = x->ve[8];
	wd3 = x->ve[9];
	
	wt1 = 0.5*(w1-wd1);
	wt2 = 0.5*(w2-wd2);
	wt3 = 0.5*(w3-wd3);

	qt1 = 1.0-2.0*q2*q2-2.0*q3*q3;
	qt2 = 1.0-2.0*q1*q1-2.0*q3*q3;
	qt3 = 1.0-2.0*q1*q1-2.0*q2*q2;

	qm01 = 2.0*q0*q1;
	qm02 = 2.0*q0*q2;
	qm03 = 2.0*q0*q3;
	qm12 = 2.0*q1*q2;
	qm13 = 2.0*q1*q3;
	qm23 = 2.0*q2*q3;

	q0f1 = 2.0*q0*f1;
	q1f1 = 2.0*q1*f1;
	q2f1 = 2.0*q2*f1;
	q3f1 = 2.0*q3*f1;
	
	q0f2 = 2.0*q0*f2;
	q1f2 = 2.0*q1*f2;
	q2f2 = 2.0*q2*f2;
	q3f2 = 2.0*q3*f2;
	
	q0f3 = 2.0*q0*f3;
	q1f3 = 2.0*q1*f3;
	q2f3 = 2.0*q2*f3;
	q3f3 = 2.0*q3*f3;

	q0half = 0.5*q0;
	q1half = 0.5*q1;
	q2half = 0.5*q2;
	q3half = 0.5*q3;
	
}

void makeMeasure()
{
	if (ISGPSVELO && (!(ISNOGPS)))
	{
		z->ve[0] = x->ve[4]+
			((qm12-qm03)*(wt3one)+(qm13+qm02)*(-wt2one))*dis->ve[0]+((qt1)*(-wt3one)+(qm13+qm02)*(wt1one))*dis->ve[1]+((qt1)*(wt2one)+(qm12-qm03)*(-wt1one))*dis->ve[2];
		z->ve[1] = x->ve[5]+
			((qt2)*(wt3one)+(qm23-qm01)*(-wt2one))*dis->ve[0]+((qm12+qm03)*(-wt3one)+(qm23-qm01)*(wt1one))*dis->ve[1]+((qm12+qm03)*(wt2one)+(qt2)*(-wt1one))*dis->ve[2];
		z->ve[2] = x->ve[6]+
			((qm23+qm01)*(wt3one)+(qt3)*(-wt2one))*dis->ve[0]+((qm13-qm02)*(-wt3one)+(qt3)*(wt1one))*dis->ve[1]+((qm13-qm02)*(wt2one)+(qm23+qm01)*(-wt1one))*dis->ve[2];
	}
	else
	{
		z->ve[0] = (qm13-qm02)*g;
		z->ve[1] = (qm23+qm01)*g;
		z->ve[2] = (qt3)*g;
	}
	
	z->ve[3] = (qt1)*b0->ve[0]+(qm12+qm03)*b0->ve[1]+(qm13-qm02)*b0->ve[2];
	z->ve[4] = (qm12-qm03)*b0->ve[0]+(qt2)*b0->ve[1]+(qm23+qm01)*b0->ve[2];
	z->ve[5] = (qm13+qm02)*b0->ve[0]+(qm23-qm01)*b0->ve[1]+(qt3)*b0->ve[2];
}

void makeH()
{
	if (ISGPSVELO && (!(ISNOGPS)))
	{
		// observe velocity and magnet field	
		H->me[0][0] = (-q3dou*(wt3one)+q2dou*(-wt2one))*dis->ve[0]
						+q2dou*(wt1one)*dis->ve[1]
						-q3dou*(-wt1one)*dis->ve[2];
		H->me[0][1] = (q2dou*(wt3one)+q3dou*(-wt2one))*dis->ve[0]
						+q3dou*(wt1one)*dis->ve[1]
						+q2dou*(-wt1one)*dis->ve[2];
		H->me[0][2] = (q1dou*(wt3one)+q0dou*(-wt2one))*dis->ve[0]
						+(-4.0*q2*(-wt3one)+q0dou*(wt1one))*dis->ve[1]
						+(-4.0*q2*(wt2one)+q1dou*(-wt1one))*dis->ve[2]; 
		H->me[0][3] = (-q0dou*(wt3one)+q1dou*(-wt2one))*dis->ve[0]
						+(-4.0*q3*(-wt3one)+q1dou*(wt1one))*dis->ve[1]
						+(-4.0*q3*(wt2one)-q0dou*(-wt1one))*dis->ve[2];
		H->me[0][7] = (-qm13-qm02)*dis->ve[1]
						+(qm12-qm03)*dis->ve[2];
		H->me[0][8] = (qm13+qm02)*dis->ve[0]
						+(-qt1)*dis->ve[2];  
		H->me[0][9] = (-qm12+qm03)*dis->ve[0]
						+(qt1)*dis->ve[1];

		H->me[1][0] = -q1dou*(-wt2one)*dis->ve[0]
						+(q3dou*(-wt3one)-q1dou*(wt1one))*dis->ve[1]
						+q3dou*(wt2one)*dis->ve[2];
		H->me[1][1] = (-4.0*q1*(wt3one)-q0dou*(-wt2one))*dis->ve[0]
						+(q2dou*(-wt3one)-q0dou*(wt1one))*dis->ve[1]
						+(q2dou*(wt2one)-4.0*q1*(-wt1one))*dis->ve[2]; 
		H->me[1][2] = q3dou*(-wt2one)*dis->ve[0]
						+(q1dou*(-wt3one)+q3dou*(wt1one))*dis->ve[1]
						+q1dou*(wt2one)*dis->ve[2];
		H->me[1][3] = (-4.0*q3*(wt3one)+q2dou*(-wt2one))*dis->ve[0]
						+(q0dou*(-wt3one)+q2dou*(wt1one))*dis->ve[1]
						+(q0dou*(wt2one)-4.0*q3*(-wt1one))*dis->ve[2];  
		H->me[1][7] = (-qm23+qm01)*dis->ve[1]
						+(qt2)*dis->ve[2];
		H->me[1][8] = (qm23-qm01)*dis->ve[0]
						+(-qm12-qm03)*dis->ve[2];
		H->me[1][9] = (-qt2)*dis->ve[0]
						+(qm12+qm03)*dis->ve[1];

		H->me[2][0] = q1dou*(wt3one)*dis->ve[0]
						-q2dou*(-wt3one)*dis->ve[1]
						+(-q2dou*(wt2one)+q1dou*(-wt1one))*dis->ve[2]; 
		H->me[2][1] = (q0dou*(wt3one)-4.0*q1*(-wt2one))*dis->ve[0]
						+(q3dou*(-wt3one)-4.0*q1*(wt1one))*dis->ve[1]
						+(q3dou*(wt2one)+q0dou*(-wt1one))*dis->ve[2];  
		H->me[2][2] = (q3dou*(wt3one)-4.0*q2*(-wt2one))*dis->ve[0]
						+(-q0dou*(-wt3one)-4.0*q2*(wt1one))*dis->ve[1]
						+(-q0dou*(wt2one)+q3dou*(-wt1one))*dis->ve[2]; 
		H->me[2][3] = q2dou*(wt3one)*dis->ve[0]
						+q1dou*(-wt3one)*dis->ve[1]
						+(q1dou*(wt2one)+q2dou*(-wt1one))*dis->ve[2];  
		H->me[2][7] = (-qt3)*dis->ve[1]
						+(qm23+qm01)*dis->ve[2];
		H->me[2][8] = (qt3)*dis->ve[0]
						+(-qm13+qm02)*dis->ve[2];
		H->me[2][9] = (-qm23-qm01)*dis->ve[0]
						+(qm13-qm02)*dis->ve[1];

	}
	else
	{
		// observe acceleration and magnet field
		H->me[0][0] = -q2dou*g;
		H->me[0][1] = q3dou*g;
		H->me[0][2] = -q0dou*g;
		H->me[0][3] = q1dou*g;
		
		H->me[1][0] = q1dou*g;
		H->me[1][1] = q0dou*g;
		H->me[1][2] = q3dou*g;
		H->me[1][3] = q2dou*g;

		H->me[2][1] = -2.0*q1dou*g;
		H->me[2][2] = -2.0*q2dou*g;
	}
	
	H->me[3][0] = q3dou*b0->ve[1]-q2dou*b0->ve[2]; 
	H->me[3][1] = q2dou*b0->ve[1]+q3dou*b0->ve[2];
	H->me[3][2] = -4.0*q2*b0->ve[0]+q1dou*b0->ve[1]-q0dou*b0->ve[2];
	H->me[3][3] = -4.0*q3*b0->ve[0]+q0dou*b0->ve[1]+q1dou*b0->ve[2];  

	H->me[4][0] = -q3dou*b0->ve[0]+q1dou*b0->ve[2]; 
	H->me[4][1] = q2dou*b0->ve[0]-4.0*q1*b0->ve[1]+q0dou*b0->ve[2]; 
	H->me[4][2] = q1dou*b0->ve[0]+q3dou*b0->ve[2];
	H->me[4][3] = -q0dou*b0->ve[0]-4.0*q3*b0->ve[1]+q2dou*b0->ve[2];  

	H->me[5][0] = q2dou*b0->ve[0]-q1dou*b0->ve[1]; 
	H->me[5][1] = q3dou*b0->ve[0]-q0dou*b0->ve[1]-4.0*q1*b0->ve[2]; 
	H->me[5][2] = q0dou*b0->ve[0]+q3dou*b0->ve[1]-4.0*q2*b0->ve[2]; 
	H->me[5][3] = q1dou*b0->ve[0]+q2dou*b0->ve[1];  

}

void makeCommonMeasure()
{
	w1 = u->ve[0];
	w2 = u->ve[1];
	w3 = u->ve[2];

	q0 = x->ve[0];
	q1 = x->ve[1];
	q2 = x->ve[2];
	q3 = x->ve[3];

	wd1 = x->ve[7];
	wd2 = x->ve[8];
	wd3 = x->ve[9];
	
	wt1one = (w1-wd1);
	wt2one = (w2-wd2);
	wt3one = (w3-wd3);	
		
	qt1 = 1.0-2.0*q2*q2-2.0*q3*q3;
	qt2 = 1.0-2.0*q1*q1-2.0*q3*q3;
	qt3 = 1.0-2.0*q1*q1-2.0*q2*q2;

	qm01 = 2.0*q0*q1;
	qm02 = 2.0*q0*q2;
	qm03 = 2.0*q0*q3;
	qm12 = 2.0*q1*q2;
	qm13 = 2.0*q1*q3;
	qm23 = 2.0*q2*q3;

	q0dou = 2.0*q0;
	q1dou = 2.0*q1;
	q2dou = 2.0*q2;
	q3dou = 2.0*q3;
}

void q2eul(Vector* q, T* roll, T* pitch, T* yaw)
{
	*roll  = atan2(2.0*(q->ve[2]*q->ve[3]+q->ve[0]*q->ve[1]), (1.0-2.0*(q->ve[1]*q->ve[1]+q->ve[2]*q->ve[2])));
	*pitch = asin(-2.0*(q->ve[1]*q->ve[3]-q->ve[0]*q->ve[2]));    
	*yaw   = atan2(2.0*(q->ve[1]*q->ve[2]+q->ve[0]*q->ve[3]), (1.0-2.0*(q->ve[2]*q->ve[2]+q->ve[3]*q->ve[3])));
}

void eul2q(T roll, T pitch, T yaw, Vector* q)
{
	q->ve[0] = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) 
			+ sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
	q->ve[1] = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0) 
			- cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
	q->ve[2] = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0) 
			+ sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
	q->ve[3] = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0) 
			- sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);
}

void Init_Attitude(Vector* _f, Vector* _b, BOOL IsInited)
{
	T fnorm,bnorm;   // fnorm: gravity vector, bnorm: magnet field vector
	T roll,pitch,yaw;
	T Hy, Hx;
	int i,j;

	// Set dimensionalities for this EKF
	if (!IsInited)
	{
		EKFilter_Init(10, 6, 6, 6, 6);
		f = v_get(3);
		b = v_get(3);
		x0 = v_get(10);
		P0 = m_get(10,10);
	}

	// do initial alignment

	f = v_copy(_f,f);
	b = v_copy(_b,b);

	fnorm = sqrt(f->ve[0]*f->ve[0] + f->ve[1]*f->ve[1] 
		+ f->ve[2]*f->ve[2]);
	
	bnorm = sqrt(b->ve[0]*b->ve[0] + b->ve[1]*b->ve[1] 
		+ b->ve[2]*b->ve[2]);

	f->ve[0] = f->ve[0]/fnorm;
	f->ve[1] = f->ve[1]/fnorm;
	f->ve[2] = f->ve[2]/fnorm;
	
	b->ve[0] = b->ve[0]/bnorm;
	b->ve[1] = b->ve[1]/bnorm;
	b->ve[2] = b->ve[2]/bnorm;

	pitch = asin(-f->ve[0]);
	roll  = asin(f->ve[1]/cos(pitch));
	
	Hy    = -b->ve[1]*cos(roll) + b->ve[2]*sin(roll);
	Hx    = b->ve[0]*cos(pitch) 
		+ b->ve[1]*sin(pitch)*sin(roll) 
		+ b->ve[2]*sin(pitch)*cos(roll);
	yaw   = atan2(Hy, Hx) + atan2(b0->ve[1], b0->ve[0]);	

	eul2q(roll, pitch, yaw, x0);

	x0->ve[4] = 0.0;			// vx
	x0->ve[5] = 0.0;			// vy
	x0->ve[6] = 0.0;			// vz
	x0->ve[7] = 0.0;			// wdx
	x0->ve[8] = 0.0;			// wdy
	x0->ve[9] = 0.0;			// wdz

	for (i=0;i<P0->m;i++)
		for (j=0;j<P0->n;j++)
			P0->me[i][j] = 0.0;

	P0->me[0][0] = 0.000001;		// q0
	P0->me[1][1] = 0.000001;		// q1
	P0->me[2][2] = 0.000001;		// q2 
	P0->me[3][3] = 0.000001;		// q3
	P0->me[4][4] = 0.01;			// vx 
	P0->me[5][5] = 0.01;			// vy 
	P0->me[6][6] = 0.01;			// vz
	P0->me[7][7] = 0.0001;			// wdx
	P0->me[8][8] = 0.0001;			// wdy 
	P0->me[9][9] = 0.0001;			// wdz

	State_Init(x0, P0);
}



