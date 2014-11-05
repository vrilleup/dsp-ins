
/**************************************************************************
**
** Copyright (C) 1993 David E. Steward & Zbigniew Leyk, all rights reserved.
**
**				 Meschach Library
** 
** This Meschach Library is provided "as is" without any express 
** or implied warranty of any kind with respect to this software. 
** In particular the authors shall not be liable for any direct, 
** indirect, special, incidental or consequential damages arising 
** in any way from use of the software.
** 
** Everyone is granted permission to copy, modify and redistribute this
** Meschach Library, provided:
**	1.	All copies contain this copyright notice.
**	2.	All modified copies shall carry a notice stating who
**		made the last modification and the date of such modification.
**	3.	No charge is made for this software or works derived from it.  
**		This clause shall not be construed as constraining other software
**		distributed on the same medium as this software, nor is a
**		distribution fee considered a charge.
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

/* memory.c 1.3 11/25/87 */

#include	"matrix.h"

// static	char	rcsid[] = "$Id: memory.c,v 1.13 1994/04/05 02:10:37 des Exp $";

#pragma CODE_SECTION(ezero, "xintfloadfuncs");
#pragma CODE_SECTION(m_swap, "xintfloadfuncs");
#pragma CODE_SECTION(v_swap, "xintfloadfuncs");
#pragma CODE_SECTION(m_copy, "xintfloadfuncs");
#pragma CODE_SECTION(v_copy, "xintfloadfuncs");
#pragma CODE_SECTION(v_zero, "xintfloadfuncs");
#pragma CODE_SECTION(m_get, "xintfloadfuncs");
#pragma CODE_SECTION(v_get, "xintfloadfuncs");
#pragma CODE_SECTION(m_free, "xintfloadfuncs");
#pragma CODE_SECTION(v_free, "xintfloadfuncs");
#pragma CODE_SECTION(m_resize, "xintfloadfuncs");
#pragma CODE_SECTION(v_resize, "xintfloadfuncs");


/* ezero -- zeros an array of floating point numbers */
void ezero(Real *dp, int len)
{
    /* else, need to zero the array entry by entry */
    int	i;
    for ( i = 0; i < len; i++ )
		dp[i] = 0.0;
}


/* Swap functions for Matrix and Vector*/

MAT *m_swap(MAT *in, MAT *out)
{
	Real *t;
	unsigned int i;
	
	if ( in==MNULL )
	{return NULL;}//error(E_NULL,"m_swap");
	if ( in==out )
		return (out);
	if ( out==MNULL || out->m < in->m || out->n < in->n )
		out = m_resize(out,in->m,in->n);
	
	/* swap pointer base */
	t = in->base;
	in->base = out->base;
	out->base = t;
	
	/* swap pointers me */
	for ( i=0; i<in->m; i++ )
	{
		t = in->me[i];
		in->me[i] = out->me[i];
		out->me[i] = t;
	}
	
	return (out);
}

VEC *v_swap(VEC *in, VEC *out)
{
	Real *t;
	
	if ( in==VNULL )
	{return NULL;}//error(E_NULL,"v_swap");
	if ( in==out )
		return (out);
	if ( out==VNULL || out->dim < in->dim )
		out = v_resize(out,in->dim);
	
	/* swap pointer base */
	t = in->ve;
	in->ve = out->ve;
	out->ve = t;
	
	return (out);
}

/* m_copy -- copies matrix into new area
-- out(i0:m,j0:n) <- in(i0:m,j0:n) */
MAT *m_copy(MAT *in, MAT *out)
{
	unsigned int	i, j;
	unsigned int i0;
	unsigned int j0;
	
	i0 = 0;
	j0 = 0;
	
	if ( in==MNULL )
	{return NULL;}//error(E_NULL,"m_copy");
	if ( in==out )
		return (out);
	if ( out==MNULL || out->m < in->m || out->n < in->n )
		out = m_resize(out,in->m,in->n);
	
	for ( i=i0; i < in->m; i++ )
//		MEM_COPY(&(in->me[i][j0]),&(out->me[i][j0]),
//		(in->n - j0)*sizeof(Real));
		for ( j=j0; j < in->n; j++ )
			out->me[i][j] = in->me[i][j];
	
	return (out);
}

/* v_copy -- copies vector into new area
-- out(i0:dim) <- in(i0:dim) */
VEC *v_copy(VEC *in, VEC *out)
{
	unsigned int i;
	unsigned int i0 = 0;
	
	if ( in==VNULL )
	{return NULL;}//error(E_NULL,"v_copy");
	if ( in==out )
		return (out);
	if ( out==VNULL || out->dim < in->dim )
		out = v_resize(out,in->dim);
	
	//MEM_COPY(&(in->ve[i0]),&(out->ve[i0]),(in->dim - i0)*sizeof(Real));
	for ( i=i0; i < in->dim; i++ )
		out->ve[i] = in->ve[i];
	
	return (out);
}

/* v_zero -- zero the vector x */
VEC *v_zero(VEC *x)
{
	if ( x == VNULL )
	{return NULL;}//error(E_NULL,"v_zero");
	
	ezero(x->ve,x->dim);
	/* for ( i = 0; i < x->dim; i++ )
	x->ve[i] = 0.0; */
	
	return x;
}

/* m_get -- gets an mxn matrix (in MAT form) by dynamic memory allocation
-- normally ALL matrices should be obtained this way
-- if either m or n is negative this will raise an error
-- note that 0 x n and m x 0 matrices can be created */
MAT *m_get(int m, int n)
{
	MAT	*matrix;
	int	i;
	
	if (m < 0 || n < 0) {return NULL;}
	//error(E_NEG,"m_get");
	
	if ((matrix=NEW(MAT)) == (MAT *)NULL ){return NULL;}
	//error(E_MEM,"m_get");
	
	matrix->m = m;		matrix->n = matrix->max_n = n;
	matrix->max_m = m;	matrix->max_size = m*n;
	matrix->base = (Real *)NULL;
	if ((matrix->me = (Real **)calloc(m,sizeof(Real *))) == 
		(Real **)NULL )
	{	free(matrix->base); free(matrix);
	   return NULL;
	   //error(E_MEM,"m_get");
	}
	
	for ( i = 0; i < m; i++ )
	{
		if ( (matrix->me[i]=NEW_A(n,Real)) == (Real *)NULL ){return NULL;}
		//error(E_MEM,"m_get");
	}
		
	return (matrix);
}


/* v_get -- gets a VEC of dimension 'size'
-- Note: initialized to zero */
VEC *v_get(int size)
{
	VEC	*vector;
	
	if (size < 0)
	{return NULL;}//error(E_NEG,"v_get");
	
	if ((vector=NEW(VEC)) == (VEC *)NULL )
	{return NULL;}//error(E_MEM,"v_get");
	
	vector->dim = vector->max_dim = size;
	if ((vector->ve=NEW_A(size,Real)) == (Real *)NULL )
	{
		free(vector);
		return NULL;
		//error(E_MEM,"v_get");
	}
	
	return (vector);
}

/* m_free -- returns MAT & asoociated memory back to memory heap */
int m_free(MAT *mat)
{
	int	i;
	
	if ( mat==(MAT *)NULL || (int)(mat->m) < 0 ||
		(int)(mat->n) < 0 )
		/* don't trust it */
		return (-1);
	
	for ( i = 0; i < mat->max_m; i++ )
	{
		if ( mat->me[i] != (Real *)NULL ) {
			free((char *)(mat->me[i]));
		}
	}
		
	if ( mat->me != (Real **)NULL ) {
		free((char *)(mat->me));
	}
		
	free((char *)mat);
		
	return (0);
}


/* v_free -- returns VEC & asoociated memory back to memory heap */
int v_free(VEC *vec)
{
	if ( vec==(VEC *)NULL || (int)(vec->dim) < 0 )
		/* don't trust it */
		return (-1);
	
	if ( vec->ve == (Real *)NULL ) {
		free((char *)vec);
	}
	else
	{
		free((char *)vec->ve);
		free((char *)vec);
	}
	
	return (0);
}



/* m_resize -- returns the matrix A of size new_m x new_n; A is zeroed
-- if A == NULL on entry then the effect is equivalent to m_get() */
MAT *m_resize(MAT *A,int new_m, int new_n)
{
	int	i;
	int	new_max_m, new_max_n, new_size, old_m, old_n;
	Real	*tmp;

	if (new_m < 0 || new_n < 0)
	{return NULL;}//error(E_NEG,"m_resize");
	
	if (A == NULL)
		return m_get(new_m,new_n);
	
	/* nothing was changed */
	if (new_m == A->m && new_n == A->n)
		return A;
	
	old_m = A->m;	old_n = A->n;
	if ( new_m > A->max_m )
	{	/* re-allocate A->me */
		
		A->me = RENEW(A->me,new_m,Real *);
		if ( ! A->me )
		{return NULL;}//error(E_MEM,"m_resize");
	}
	new_max_m = max(new_m,A->max_m);
	new_max_n = max(new_n,A->max_n);
	
	if ( A->max_n < new_n )
	{		
		for ( i = 0; i < A->max_m; i++ )
		{
			if ( (tmp = RENEW(A->me[i],new_max_n,Real)) == NULL )
			{return NULL;}//error(E_MEM,"m_resize");
			else { 
				A->me[i] = tmp;
			}
		}
		for ( i = A->max_m; i < new_max_m; i++ )
		{
			if ( (tmp = NEW_A(new_max_n,Real)) == NULL )
			{return NULL;}//error(E_MEM,"m_resize");
			else {
				A->me[i] = tmp;
			}
		}
	}
	else if ( A->max_m < new_m )
	{
		for ( i = A->max_m; i < new_m; i++ ) 
			if ( (A->me[i] = NEW_A(new_max_n,Real)) == NULL )
			{return NULL;}//error(E_MEM,"m_resize");
			
	}
	
	if ( old_n < new_n )
	{
		for ( i = 0; i < old_m; i++ )
			ezero(&(A->me[i][old_n]),new_n-old_n);
	}
	
	/* zero out the new rows.. */
	for ( i = old_m; i < new_m; i++ )
		ezero(A->me[i],new_n);
	
	A->max_m = new_max_m;
	A->max_n = new_max_n;
	A->max_size = A->max_m*A->max_n;
	A->m = new_m;	A->n = new_n;
	
	return A;
}

/* v_resize -- returns the vector x with dim new_dim
-- x is set to the zero vector */
VEC *v_resize(VEC *x, int new_dim)
{
	
	if (new_dim < 0)
	{return NULL;}//error(E_NEG,"v_resize");
	
	if (x == NULL)
		return v_get(new_dim);
	
	/* nothing is changed */
	if (new_dim == x->dim)
		return x;
	
	if ( x->max_dim == 0 )	/* assume that it's from sub_vec */
		return v_get(new_dim);
	
	if ( new_dim > x->max_dim )
	{
		
		x->ve = RENEW(x->ve,new_dim,Real);
		if ( ! x->ve )
		{return NULL;}//error(E_MEM,"v_resize");
		x->max_dim = new_dim;
	}
	
	if ( new_dim > x->dim )
		ezero(&(x->ve[x->dim]),new_dim - x->dim);
	x->dim = new_dim;
	
	return x;
}




