/**
 *   @file  MATRIX_3x3MatInv.c
 *
 *   @brief
 *      3x3 matrix inversion functions
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

#include <modules/postProcessing/matrixFunc/api/MATRIX_3x3MatInv.h>


/** 
 *  \fn     void LTElib_single3x3MatInv(
 *                            IN      cplx32_t  * RESTRICT Input,
 *                            OUT     cplxf_t  * output);
 *
 *  \brief   Single 3x3 matrix (complex positive semi-definitive Hermitian) inversion using cofactor method.
 * 
 *  \param[in]    Input
 *              Input 3x3 matrix that needs to be inversed, stored sequentially 
 *              in format (0,0), (0,1), (0,2), (1,0), (1,1), (1,2),(2,0), (2,1)...(2,2).
 *
 *  \param[out]   output
 *              Output 3x3 matrix. Stored sequentially as the input.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */
void 		MATRIX_single3x3MatInv (  
								IN      cplx32_t * RESTRICT Input,
								OUT     cplxf_t * RESTRICT output)	
{
    float       oneOverdH;
    float 		det;
	__float2_t		B;
	float		A, E, K;
	__float2_t	x01, x02, x12;
	float		x00, x11, x22;
    __float2_t 		dtemp1, dtemp2;
    __x128_t	results;
	__float2_t	conjC;


	/****************************************************************
				[ x00   x01   x02 ]
		input = [ x01'  x11   x12 ]
				[ x02'  x12'  x22 ]
	    A = x11*x22 - |x12|^2
	    E = x00*x22 - |x02|^2
	    K = x00*x11 - |x01|^2
	    B = x02'*x12 - x01'*x22
	    C = x01'*x12' - x02'*x11
	    F = x02'*x01 - x00*x12'

		Z = x00*A + x01*B + x02*C

				 [ A   B'   C' ]
		output = [ B   E    F' ] * (1/Z)
				 [ C   F    K  ]
	 ****************************************************************/

		x01			=	_dintsp((int64_t)_amem8(&Input[1]));
		x02			=	_dintsp((int64_t)_amem8(&Input[2]));
		x12			=	_dintsp((int64_t)_amem8(&Input[5]));
		
		x00			=	(float) _hill(_amem8(&Input[0]));
		x11			=	(float) _hill(_amem8(&Input[4]));
		x22			=	(float) _hill(_amem8(&Input[8]));
		
		dtemp1		=	_dmpysp(x12, x12);
		A			=	x11 * x22 - _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x02, x02);
		E			=	x00 * x22 - _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x01, x01);
		K			=	x00 * x11 - _hif2(dtemp1) - _lof2(dtemp1);

		results		=	_cmpysp(x02, x12);
		dtemp1		=	_dsubsp(_hif2_128(results), _lof2_128(results));
		B			=	_dsubsp(dtemp1, _dmpysp(x01, _ftof2(x22, -x22)));
	
		/* note conj(C) is calculated here */
		results		=	_cmpysp(x01, x12);
		dtemp1		=	_daddsp(_hif2_128(results), _lof2_128(results));
		conjC		=	_dsubsp(dtemp1, _dmpysp(x02, _ftof2(x11, x11)));
	
		dtemp1		=	_dmpysp(x01, B);
		det			=	x00 * A + _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x02, conjC);
		det			=	det + _hif2(dtemp1) + _lof2(dtemp1);
		oneOverdH	=	_rcpsp(det);
		oneOverdH 	= 	oneOverdH * (2.f - det * oneOverdH);
		dtemp1		=	_fdmv_f2(oneOverdH, oneOverdH);

		results		=	_cmpysp(x02, x01);
		dtemp2		=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp2		=	_dsubsp(dtemp2, _dmpysp(x12, _ftof2(x00, -x00)));
		dtemp2		=	_dmpysp(dtemp2, dtemp1);
		_amem8_f2(&output[7])	=	dtemp2;
		_amem8_f2(&output[5])	=	_lltof2(_f2toll(dtemp2) ^ 0x0000000080000000);
	
		dtemp2		=	_dmpysp(B, dtemp1);
		_amem8_f2(&output[3])	=	dtemp2;
		_amem8_f2(&output[1])	=	_lltof2(_f2toll(dtemp2) ^ 0x0000000080000000);
		dtemp2		=	_dmpysp(conjC, dtemp1);
		_amem8_f2(&output[2])	=	dtemp2;
		_amem8_f2(&output[6])	=	_lltof2(_f2toll(dtemp2) ^ 0x0000000080000000);
		
		_amem8_f2(&output[0])	=	_ftof2(A * oneOverdH, 0.f);
		_amem8_f2(&output[4])	=	_ftof2(E * oneOverdH, 0.f);
		_amem8_f2(&output[8])	=	_ftof2(K * oneOverdH, 0.f);
}


/** 
 *  \fn     void MATRIX_multi3x3MatInvFltOut(
 *                            IN      cplx32_t  * RESTRICT in,
 *                            IN      int32_t  n,
 *                            OUT     cplxf_t  * out);
 *
 *  \brief   Multiple 3x3 matrices (complex positive semi-definitive Hermitian) inversion using cofactor method with floating-point output.
 * 
 *  \param[in]    Input
 *              Input 3x3 matrices that needs to be inverted, stored sequentially 
 *              in format (0,0), (0,1), (0,2), (1,0), (1,1), (1,2),(2,0), (2,1)...(2,2).
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    n
 *              Number of input matrices to be inverted.
 *
 *  \param[out]   out
 *              Output 3x3 matrix. Stored sequentially as the input. Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */
void 		MATRIX_multi3x3MatInvFltOut (  
								IN      cplx32_t * RESTRICT in,
								IN      int32_t 		n,
								OUT     cplxf_t * RESTRICT out)	
{
    float       oneOverdH;
    float 		det;
    int32_t		i;
    int64_t		* RESTRICT Input;
    __float2_t		* RESTRICT output;
    __float2_t 		dtemp1, dtemp2;
    __x128_t	results;
	__float2_t		B, conjC;
	float		A, E, K;
	__float2_t		x01, x02, x12;
	float		x00, x11, x22;


	/****************************************************************
				[ x00   x01   x02 ]
		input = [ x01'  x11   x12 ]
				[ x02'  x12'  x22 ]
	    A = x11*x22 - |x12|^2
	    E = x00*x22 - |x02|^2
	    K = x00*x11 - |x01|^2
	    B = x02'*x12 - x01'*x22
	    C = x01'*x12' - x02'*x11
	    F = x02'*x01 - x00*x12'

		Z = x00*A + x01*B + x02*C

				 [ A   B'   C' ]
		output = [ B   E    F' ] * (1/Z)
				 [ C   F    K  ]
	 ****************************************************************/

	Input		=	(int64_t *) in;
	output		=	(__float2_t *) out;
	#ifdef _TMS320C6X
    _nassert((int32_t) Input % 8 == 0);
	_nassert((int32_t) output % 8 == 0);
    #endif
	for ( i = 0; i < n; i++ )
	{
		x01			=	_dintsp((int64_t)Input[1]);
		x02			=	_dintsp((int64_t)Input[2]);
		x12			=	_dintsp((int64_t)Input[5]);
		
		x00			=	(float) _hill(Input[0]);
		x11			=	(float) _hill(Input[4]);
		x22			=	(float) _hill(Input[8]);
		
		Input		+=	9;

		dtemp1		=	_dmpysp(x12, x12);
		A			=	x11 * x22 - _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x02, x02);
		E			=	x00 * x22 - _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x01, x01);
		K			=	x00 * x11 - _hif2(dtemp1) - _lof2(dtemp1);

		results		=	_cmpysp(x02, x12);
		dtemp1		=	_dsubsp(_hif2_128(results), _lof2_128(results));
		B			=	_dsubsp(dtemp1, _dmpysp(x01, _ftof2(x22, -x22)));
	
		/* note conj(C) is calculated here */
		results		=	_cmpysp(x01, x12);
		dtemp1		=	_daddsp(_hif2_128(results), _lof2_128(results));
		conjC		=	_dsubsp(dtemp1, _dmpysp(x02, _ftof2(x11, x11)));
	
		dtemp1		=	_dmpysp(x01, B);
		det			=	x00 * A + _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x02, conjC);
		det			=	det + _hif2(dtemp1) + _lof2(dtemp1);
		oneOverdH	=	_rcpsp(det);
		oneOverdH 	= 	oneOverdH * (2.f - det * oneOverdH);
		dtemp1		=	_fdmv_f2(oneOverdH, oneOverdH);

		results		=	_cmpysp(x02, x01);
		dtemp2		=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp2		=	_dsubsp(dtemp2, _dmpysp(x12, _ftof2(x00, -x00)));
		dtemp2		=	_dmpysp(dtemp2, dtemp1);
		output[7]	=	dtemp2;
		output[5]	=	_lltof2(_f2toll(dtemp2) ^ 0x0000000080000000);
	
		dtemp2		=	_dmpysp(B, dtemp1);
		output[3]	=	dtemp2;
		output[1]	=	_lltof2(_f2toll(dtemp2) ^ 0x0000000080000000);
		dtemp2		=	_dmpysp(conjC, dtemp1);
		output[2]	=	dtemp2;
		output[6]	=	_lltof2(_f2toll(dtemp2) ^ 0x0000000080000000);
		
		output[0]	=	_ftof2(A * oneOverdH, 0.f);
		output[4]	=	_ftof2(E * oneOverdH, 0.f);
		output[8]	=	_ftof2(K * oneOverdH, 0.f);
		
		output		+=	9;
	}
		
}



/** 
 *  \fn     void MATRIX_multi3x3MatInvFxdOut(
 *                            IN      cplx32_t  * RESTRICT Input,
 *                            IN      int32_t  n,
 *                            IN      int32_t   range,
 *                            OUT     int32_t	* RESTRICT Qinv;
 *                            OUT     cplx16_t  * out);
 *
 *  \brief   Multiple 3x3 matrices (complex positive semi-definitive Hermitian) inversion using cofactor method
 *           with block fixed-point output.
 *
 *  \param[in]    Input
 *              Input 3x3 matrices that needs to be inverted, stored sequentially 
 *              in format (0,0), (0,1), (0,2), (1,0), (1,1), (1,2),(2,0), (2,1)...(2,2).
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    n
 *              Number of input matrices to be inverted.
 *
 *  \param[in]   range
 *              The full range of the output. For example, if we set it to 14, that means the real and imaginary
 *              part of the output elements will not exceed 14-bit with sign.
 *
 *  \param[out]   Qinv
 *              Output block Q value for the inverse matrices.
 *
 *  \param[out]   out
 *              Output 3x3 matrix. Stored sequentially as the input. Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */
void 		MATRIX_multi3x3MatInvFxdOut (  
								IN      cplx32_t * RESTRICT in,
								IN      int32_t 		n,
								IN      int32_t         range,
								OUT     int32_t	 * RESTRICT Qinv,
								OUT     cplx16_t * RESTRICT out)	
{
    float       oneOverdH;
    float 		det;
	float		max, magic;
    int32_t		i, itemp;
    int64_t		* RESTRICT Input;
    int32_t		* RESTRICT output;
    __float2_t 		dtemp1, dtemp2, dtemp3;
	int64_t		lltemp;
    __x128_t	results;
	__float2_t		B, conjC;
	float		A, E, K;
	__float2_t		x01, x02, x12;
	float		x00, x11, x22;
	int32_t     head = 16 - range;


	/****************************************************************
				[ x00   x01   x02 ]
		input = [ x01'  x11   x12 ]
				[ x02'  x12'  x22 ]
	    A = x11*x22 - |x12|^2
	    E = x00*x22 - |x02|^2
	    K = x00*x11 - |x01|^2
	    B = x02'*x12 - x01'*x22
	    C = x01'*x12' - x02'*x11
	    F = x02'*x01 - x00*x12'

		Z = x00*A + x01*B + x02*C

				 [ A   B'   C' ]
		output = [ B   E    F' ] * (1/Z)
				 [ C   F    K  ]
	 ****************************************************************/

	Input		=	(int64_t *) in;
	output		=	(int32_t *) out;
	#ifdef _TMS320C6X
    _nassert((int32_t) Input % 8 == 0);
	_nassert((int32_t) output % 8 == 0);
    #endif
	for ( i = 0; i < n; i++ )
	{
		x01			=	_dintsp((int64_t)Input[1]);
		x02			=	_dintsp((int64_t)Input[2]);
		x12			=	_dintsp((int64_t)Input[5]);
		
		x00			=	(float) _hill(Input[0]);
		x11			=	(float) _hill(Input[4]);
		x22			=	(float) _hill(Input[8]);
		
		Input		+=	9;

		dtemp1		=	_dmpysp(x12, x12);
		A			=	x11 * x22 - _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x02, x02);
		E			=	x00 * x22 - _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x01, x01);
		K			=	x00 * x11 - _hif2(dtemp1) - _lof2(dtemp1);

		max			=	A;
		if ( max < E )
			max		=	E;
		if ( max < K )
			max		=	K;

		results		=	_cmpysp(x02, x12);
		dtemp1		=	_dsubsp(_hif2_128(results), _lof2_128(results));
		B			=	_dsubsp(dtemp1, _dmpysp(x01, _ftof2(x22, -x22)));
	
		/* note conj(C) is calculated here */
		results		=	_cmpysp(x01, x12);
		dtemp1		=	_daddsp(_hif2_128(results), _lof2_128(results));
		conjC		=	_dsubsp(dtemp1, _dmpysp(x02, _ftof2(x11, x11)));
	
		dtemp1		=	_dmpysp(x01, B);
		det			=	x00 * A + _hif2(dtemp1) - _lof2(dtemp1);
		dtemp1		=	_dmpysp(x02, conjC);
		det			=	det + _hif2(dtemp1) + _lof2(dtemp1);
		oneOverdH	=	_rcpsp(det);
		oneOverdH 	= 	oneOverdH * (2.f - det * oneOverdH);
		dtemp1		=	_ftof2(oneOverdH, oneOverdH); 

		magic		=	_itof(0x04C00000 + (head<<23) + (_ftoi(max * oneOverdH) & 0xFF800000));
		dtemp3		=	_ftof2(magic, magic);
		results		=	_cmpysp(x02, x01);
		dtemp2		=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp2		=	_dsubsp(dtemp2, _dmpysp(x12, _ftof2(x00, -x00)));
		dtemp2		=	_dmpysp(dtemp2, dtemp1);
		lltemp		=	_f2toll(_daddsp(dtemp2, dtemp3));
		itemp		=	_pack2(_hill(lltemp), _loll(lltemp));
		output[7]	=	itemp;
		output[5]	=	_packhl2(itemp, _ssub2(0, itemp));
	
		dtemp2		=	_dmpysp(B, dtemp1);
		lltemp		=	_f2toll(_daddsp(dtemp2, dtemp3));
		itemp		=	_pack2(_hill(lltemp), _loll(lltemp));
		output[3]	=	itemp;
		output[1]	=	_packhl2(itemp, _ssub2(0, itemp));
		dtemp2		=	_dmpysp(conjC, dtemp1);
		lltemp		=	_f2toll(_daddsp(dtemp2, dtemp3));
		itemp		=	_pack2(_hill(lltemp), _loll(lltemp));
		output[2]	=	itemp;
		output[6]	=	_packhl2(itemp, _ssub2(0, itemp));
		
		output[0]	=	_pack2(_ftoi(A * oneOverdH + magic), 0);
		output[4]	=	_pack2(_ftoi(E * oneOverdH + magic), 0);
		output[8]	=	_pack2(_ftoi(K * oneOverdH + magic), 0);
		Qinv[i]		=	(127 + 23) - (_ftoi(magic) >> 23);
		
		output		+=	9;
	}
		
}

