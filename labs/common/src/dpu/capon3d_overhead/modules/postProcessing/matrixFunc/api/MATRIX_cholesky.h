/**
 *   @file  MATRIX_cholesky.h
 *
 *   @brief
 *      Header file for Cholesky Decomposition and triangular matrix solver
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <people_counting/overhead_3d_people_counting/src/common/swpform.h>
#include "MATRIX_cholesky_dat.h"

#ifndef _TMS320C6600
#include <common/src/dpu/capon3d_overhead/modules/utilities/radar_c674x.h>
#endif

#ifndef __MATRIX_CHOLESKY_H
#define __MATRIX_CHOLESKY_H


void MATRIX_up_tri_solve(
	int32_t * L,  
	int32_t * Z,  
	int32_t * Y,  
    int32_t n
	);

void MATRIX_low_tri_solve(
	int32_t * L,  
	int32_t * Y,  
	int32_t * V,  
    int32_t n
	);


void MATRIX_cholesky(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t n
	);
  
void MATRIX_cholesky_2(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t	x);
  
void MATRIX_cholesky_4(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t	x);

void MATRIX_cholesky_flp(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t n
	);                                      

void MATRIX_cholesky_flp_full(
	cplxf_t * RESTRICT A, 
	cplxf_t * Ap,
	int32_t n
	); 

void MATRIX_cholesky_flp_inv(
	cplxf_t * RESTRICT A, 
	cplxf_t * Ap,
	int32_t n
	);
  
static __inline int32_t _smpy32_64 (int32_t a, int32_t b)
{
  //return( _mpyhir(a, b) + (_mpyluhs(a, b)>>15) ); -- c64 CPU
  return( (_hill((_mpy32ll(a, b)+0x40000000)<<1))  ); // -- c64x+ CPU
}

static __inline int32_t frac_dotp2(int32_t a, int32_t b, int32_t c, int32_t d)
{
  return(_mpyhir(a, b) +
         _mpyhir(c, d) +
         4*_dotprsu2(_packh2(b,d), _shru2(_pack2(a,c),1) )); 
}

static __inline int32_t frac_dotpn2(int32_t a, int32_t b, int32_t c, int32_t d)
{
  return(_mpyhir(a, b) -
         _mpyhir(c, d) +
         4*_dotpnrsu2(_packh2(b,d), _shru2(_pack2(a,c),1) )); 
}

static __inline  void recip_sqrt32(int32_t inputin, int32_t expin, int32_t *mantPtr, int32_t *expPtr)
{
	const int32_t TABLE_BIAS_FACTOR = 0x40000000; /* 0.5 */
	const int16_t NO_BITS_PER_WORD = 32;
	const int16_t LOG2_TABLESIZE = 8;
	const int32_t   SQRT_ONE_HALF = 0x5A82799A;  /* ~ sqrt(1/2) */
	int32_t bitcount;
	int32_t seed_x_of_n0, temp_oneandhalf;
	int32_t x, x2, x3, ax3, input;
	int32_t EvenOdd;
	int32_t offset;

	bitcount = _norm(inputin); 			/* Number of redundant sign bits */

	/* Normalize the recip_sqrt value "a" to the range 0.5-1.0 */
	input = inputin << bitcount;
	x = input - TABLE_BIAS_FACTOR;			/* Change Number from 0.5-1.0 to 0.0-0.5 */
							/* Retain Old Input value */
	offset = x >> (NO_BITS_PER_WORD - LOG2_TABLESIZE - 2); 
	/* Since 0 <= x < 0.5 we have (2 sign + 6 data bits) */
		
	seed_x_of_n0 = (int32_t)(recipSqtTable_hcn[offset]<<16);

	bitcount += (2 - expin); 			/* setup for exponent */
        /* increase bitcount by expin to account for exponent in  JH*/

	EvenOdd = bitcount & 1;				/* extract bit 1 for even/odd test of exponent */
	bitcount = bitcount >> 1; 			/* divide exponent by 2 */
		
	x2 = _smpyh(seed_x_of_n0, seed_x_of_n0); 	/* x[n] * x[n] */
	x3 = _smpy32_64(seed_x_of_n0, x2);		/* x[n]^3 */
	x3 = _mpyhir(seed_x_of_n0, x2);		/* x[n]^3 */
	ax3 = _smpy32_64(input, x3);			/* a*x[n]^3 */
	temp_oneandhalf = seed_x_of_n0 + (seed_x_of_n0>>1);	/* 1.5*x[n] */
	seed_x_of_n0 = temp_oneandhalf - 2*ax3;			/* x[n+1] = 1.5x[n] - 2a*x[n]^3 */

	x2 = _smpy32_64(seed_x_of_n0, seed_x_of_n0); 		/* x[n] * x[n] */
	x3 = _smpy32_64(seed_x_of_n0, x2);			/* x[n]^3 */
	ax3 = _smpy32_64(input, x3);				/* a*x[n]^3 */
	temp_oneandhalf = seed_x_of_n0 + (seed_x_of_n0>>1);	/* 1.5*x[n] */
	seed_x_of_n0 = temp_oneandhalf - 2*ax3;	
	
        /* if exponent is Odd multiply by sqrt(2) */
	if(EvenOdd) seed_x_of_n0 = 2*_smpy32_64(seed_x_of_n0, SQRT_ONE_HALF);
				
	seed_x_of_n0 = _abs(seed_x_of_n0);
    
	*expPtr = bitcount;
	*mantPtr = seed_x_of_n0; /* Return Mantisa/Exponent */
    return;
}

static __inline  void recip_32(int32_t input, int32_t exp, int32_t *mantPtr, int32_t *expPtr)
{
	int32_t TABLE_BIAS_FACTOR = 0x40000000; 		/* 0.5 */
	int16_t NO_BITS_PER_WORD = 32;
	int16_t LOG2_TABLESIZE = 6;
	int16_t bitcount;
	int32_t seed_x_of_n0;
	int32_t   x, x2, ax2;
	int32_t   offset;
		
	bitcount = _norm(input); 					/* Number of redundant sign bits */
	
	/* Normalize the recip_sqrt value "a" to the range 0.5.0 */
	input = input << bitcount;
	bitcount +=  (1-exp); 						/* setup for exponent + input exponent */
	x = input - TABLE_BIAS_FACTOR;				/* Change Number from 0.5.0 to 0.0-0.5 */
												/* Retain Old Input value */
	offset = x >> (NO_BITS_PER_WORD - LOG2_TABLESIZE - 2); 
	
	/* Since 0 <= x < 0.5 we have (2 sign + 6 data bits) */
	seed_x_of_n0 = (int32_t)(recipTable[offset]<<16);
		
	x2 = _smpyh(seed_x_of_n0, seed_x_of_n0);	/* x[n] * x[n] */
	ax2 =_smpy32_64(input, x2);					/* a*x[n]^2 */
	seed_x_of_n0 = 2*(seed_x_of_n0 - ax2);		/* x[n] = 2x[n] - 2a*x[n]^2 */
	x2 = _smpy32_64(seed_x_of_n0, seed_x_of_n0);	/* x[n] * x[n] */
	ax2 =_smpy32_64(input, x2);					/* a*x[n]^2 */
	seed_x_of_n0 = 2*(seed_x_of_n0 - ax2);		/* x[n] = 2x[n] - 2a*x[n]^2 */

    *mantPtr = seed_x_of_n0;			/* Return Mantisa/Exponent */

	*expPtr = bitcount;
    return;
}

#endif //__MATRIX_CHOLESKY_H

