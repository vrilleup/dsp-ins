
/**************************************************************************
**
** Copyright (C) 1993 David E. Steward & Zbigniew Leyk, all rights reserved.
**
**			     Meschach Library
** 
** This Meschach Library is provided "as is" without any express 
** or implied warranty of any kind with respect to this software. 
** In particular the authors shall not be liable for any direct, 
** indirect, special, incidental or consequential damages arising 
** in any way from use of the software.
** 
** Everyone is granted permission to copy, modify and redistribute this
** Meschach Library, provided:
**  1.  All copies contain this copyright notice.
**  2.  All modified copies shall carry a notice stating who
**      made the last modification and the date of such modification.
**  3.  No charge is made for this software or works derived from it.  
**      This clause shall not be construed as constraining other software
**      distributed on the same medium as this software, nor is a
**      distribution fee considered a charge.
**
***************************************************************************/

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

/*
		Type definitions for general purpose maths package
*/
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File


#include	<stdlib.h>
#include	<string.h>

#ifndef	MATRIXH

/* RCS id: $Id: matrix.h,v 1.18 1994/04/16 00:33:37 des Exp $ */

#define	MATRIXH	


#define  Real float32
/* standard copy & zero functions */
#define	MEM_COPY(from,to,size)	memmove((to),(from),(size))
#define	MEM_ZERO(where,size)	memset((where),0,(size))

/* unsigned integer type */
/************************************************************
#ifndef U_INT_DEF
typedef	unsigned int	u_int;
#define U_INT_DEF
#endif
************************************************************/

/* miscellaneous constants */
#define	VNULL	((VEC *)NULL)
#define	MNULL	((MAT *)NULL)
#define	PNULL	((PERM *)NULL)
#define	IVNULL	((IVEC *)NULL)
#define BDNULL  ((BAND *)NULL)

/* allocate one object of given type */
#define	NEW(type)	((type *)calloc((size_t)1,(size_t)sizeof(type)))

/* allocate num objects of given type */
#define	NEW_A(num,type)	((type *)calloc((size_t)(num),(size_t)sizeof(type)))

 /* re-allocate arry to have num objects of the given type */
#define	RENEW(var,num,type) \
    ((var)=(type *)((var) ? \
		    realloc((char *)(var),(size_t)((num)*sizeof(type))) : \
		    calloc((size_t)(num),(size_t)sizeof(type))))

#define	MEMCOPY(from,to,n_items,type) \
 MEM_COPY((char *)(from),(char *)(to),(unsigned)(n_items)*sizeof(type))

/* type independent min and max operations */
#ifndef max
#define	max(a,b)	((a) > (b) ? (a) : (b))
#endif /* max */
#ifndef min
#define	min(a,b)	((a) > (b) ? (b) : (a))
#endif /* min */

#ifndef TRUE
#define	TRUE	1
#endif
#ifndef FALSE
#define	FALSE	0
#endif

/* for input routines */
#define MAXLINE 81

/* macros that also check types and sets pointers to NULL */
#define	M_FREE(mat)	( m_free(mat),	(mat)=(MAT *)NULL )
#define V_FREE(vec)	( v_free(vec),	(vec)=(VEC *)NULL )

#define MAXDIM  	10000001

/* vector definition */
typedef	struct	{
		unsigned int	dim, max_dim;
		Real	*ve;
		} VEC;

/* matrix definition */
typedef	struct	{
		unsigned int	m, n;
		unsigned int	max_m, max_n, max_size;
		Real	**me,*base;	/* base is base of alloc'd mem */
		} MAT;

/* special purpose access routines */

/* Swaping routines, they are fast because 
   they only swap data pointers of Matrix of Vector */
extern	MAT	*m_swap(MAT *in,MAT *out);
extern	VEC	*v_swap(VEC *in,VEC *out);

/* Copying routines */
extern	MAT	*m_copy(MAT *in,MAT *out);
extern	VEC	*v_copy(VEC *in,VEC *out);

/* Initialisation routines -- to be zero, ones, random or identity */
extern	VEC *v_zero(VEC *);


/* Dynamic memory allocation */

/* Should use M_FREE/V_FREE/PX_FREE in programs instead of m/v/px_free()
   as this is considerably safer -- also provides a simple type check ! */
/* get/resize vector to given dimension */
extern	VEC *v_get(int);
extern	VEC *v_resize(VEC *,int);
/* get/resize matrix to be m x n */
extern	MAT *m_get(int,int); 
extern	MAT *m_resize(MAT *,int,int);

/* free (de-allocate) (band) matrices, vectors, permutations and 
   integer vectors */
//extern  int iv_free(IVEC *);
extern	int m_free(MAT *);
extern	int v_free(VEC *);

extern void	ezero(Real *dp, int len);

#endif 

