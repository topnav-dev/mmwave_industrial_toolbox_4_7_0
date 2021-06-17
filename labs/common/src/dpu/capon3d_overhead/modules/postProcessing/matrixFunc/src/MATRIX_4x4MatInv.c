/**
 *   @file  MATRIX_4x4MatInv.c
 *
 *   @brief
 *      4x4 matrix inversion functions
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

#include <modules/postProcessing/matrixFunc/api/MATRIX_4x4MatInv.h>



/** 
 *  \fn     void MATRIX_single4x4MatInv(
 *                            IN      cplx32_t  * RESTRICT Input,
 *                            OUT     cplx16_t  * output);
 *
 *  \brief   Single 4x4 matrix (complex positive semi-definitive Hermitian) inversion using blockwise method.
 * 
 *  \param[in]    Input
 *              Input 4x4 matrix that needs to be inversed, stored sequentially 
 *              in format (0,0), (0,1), (0,2), (0, 3), (1,0), (1,1)...(3, 2), (3, 3).
 *
 *  \param[out]   output
 *              Output 4x4 inversion of the input. Stored sequentially as the input.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */

void 		MATRIX_single4x4MatInv (  
								cplx32_t * RESTRICT Input,
								cplxf_t * RESTRICT output)	
{
    float       oneOverdH;
    float       temp1, temp2, dH;

    float      A1I,A2I,A1Q,A2Q;
    float      A0I,A3I;
    float      B0I,B1I,B2I,B3I;
    float      B0Q,B1Q,B2Q,B3Q;
    float      C0I,C1I,C2I,C3I;
    float      C0Q,C1Q,C2Q,C3Q;
    float      CA0I,CA1I,CA2I,CA3I;
    float      CA0Q,CA1Q,CA2Q,CA3Q;
    float      BT0I,BT1I,BT2I,BT3I;
    float      BT0Q,BT1Q,BT2Q,BT3Q;
    float      AB0I,AB1I,AB2I,AB3I;
    float      AB0Q,AB1Q,AB2Q,AB3Q;
    float      D1I,D1Q;
    float      D0I,D3I;
    float      T1I,T1Q,T2I,T2Q;
    float      T0I,T3I;

    /* because there in a 32 bit input,
      scale down to avoid overflow on multiplication and to get the same output resolution */

	A0I = (float)Input[0].real;
	A3I = (float)Input[5].real;
	A1I = (float)Input[1].real; 
	A1Q = (float)Input[1].imag; 
    A2I = (float)A1I; 
	A2Q = (float) -A1Q;

	D0I = (float)Input[10].real;
	D3I = (float)Input[15].real;
	D1I = (float)Input[11].real;  
	D1Q = (float)Input[11].imag; 

	
	B0I = (float)Input[2].real; 
	B0Q = (float)Input[2].imag;
	B1I = (float)Input[3].real; 
	B1Q = (float)Input[3].imag;
	B2I = (float)Input[6].real; 
	B2Q = (float)Input[6].imag;
	B3I = (float)Input[7].real; 
	B3Q = (float)Input[7].imag;

    C0I = B0I; C0Q = -B0Q;
    C1I = B2I; C1Q = -B2Q;
    C2I = B1I; C2Q = -B1Q;
    C3I = B3I; C3Q = -B3Q;


	/* calculate inv(A)*/
	dH = A0I * A3I - A1I * A1I - A1Q * A1Q;

    oneOverdH = _rcpsp( dH );
	oneOverdH = oneOverdH * (2.f - dH * oneOverdH);
	oneOverdH = oneOverdH * (2.f - dH * oneOverdH);

    temp1 = A0I * oneOverdH;
	A0I = A3I*oneOverdH;
	A3I = temp1;

	A1I *= -oneOverdH;
	A1Q *= -oneOverdH;
	A2I =  A1I;
	A2Q = -A1Q;

	/* calculate C = C*inv(A) */
	/* C0 */
    CA0I = C0I * A0I + C1I * A2I - C1Q * A2Q;
    CA0Q = C0Q * A0I + C1I * A2Q + C1Q * A2I;

	/* C1 */
    CA1I = C0I * A1I + C1I * A3I - C0Q * A1Q;
    CA1Q = C0Q * A1I + C0I * A1Q + C1Q * A3I;

	/* C2 */
    CA2I = C2I * A0I + C3I * A2I - C3Q * A2Q;
    CA2Q = C2Q * A0I + C3I * A2Q + C3Q * A2I;

	/* C3 */
    CA3I = C2I * A1I + C3I * A3I - C2Q * A1Q;
    CA3Q = C2Q * A1I + C2I * A1Q + C3Q * A3I;

	/* calculate T = D - C*B */
	/* T should be Hermitian QT = HNorm*/
	/* CB[0] */
    T0I = D0I - ( CA0I * B0I + CA1I * B2I - CA0Q * B0Q - CA1Q * B2Q );
	
	/* CB[1] */
	/* CB[2] */
    T1I = D1I - ( CA0I * B1I + CA1I * B3I - CA0Q * B1Q - CA1Q * B3Q );
    T1Q = D1Q - ( CA0Q * B1I + CA1I * B3Q + CA0I * B1Q + CA1Q * B3I );

    //T2I =  T1I;
    //T2Q = -T1Q;

    T3I = D3I - ( CA2I * B1I + CA3I * B3I - CA2Q * B1Q - CA3Q * B3Q );

	/* Calculate T = inv(T) */
    dH = T0I * T3I - T1I * T1I - T1Q * T1Q;

    oneOverdH = _rcpsp( dH );
	oneOverdH = oneOverdH * (2.f - dH * oneOverdH);
//	oneOverdH = oneOverdH * (2.f - dH * oneOverdH);


    temp1 = T0I * oneOverdH;
	T0I = T3I * oneOverdH;
	T3I = temp1;

    T1I *= -oneOverdH;
    T1Q *= -oneOverdH;

    T2I =  T1I;
    T2Q = -T1Q;

	output[10].real		=	T0I;
	output[10].imag		=	0;
	output[15].real		=	T3I;
	output[15].imag		=	0;
	output[11].real		=	T1I;
	output[11].imag		=	T1Q;
	output[14].real		=	T2I;
	output[14].imag		=	T2Q;

	/* calculate B = inv(A)*B QB = QinvA + HNorm - 31 + exp */
	/* B0 */
	/* B2 */

    AB0I = B0I * A0I + A1I * B2I - A1Q * B2Q;
    AB0Q = B0Q * A0I + A1I * B2Q + A1Q * B2I;

    AB2I = B2I * A3I + A2I * B0I - A2Q * B0Q;
    AB2Q = B2Q * A3I + A2I * B0Q + A2Q * B0I;

	/* B1 */
    AB1I = B1I * A0I + A1I * B3I - A1Q * B3Q;
    AB1Q = B1Q * A0I + A1I * B3Q + A1Q * B3I;

	/* B3 */
    AB3I = B3I * A3I + A2I * B1I - A2Q * B1Q;
    AB3Q = B3Q * A3I + A2I * B1Q + A2Q * B1I;

	/* calculate B = B*T */
	/* B0 */
    BT0I = AB0I * T0I + AB1I * T2I - AB1Q * T2Q;
    BT0Q = AB0Q * T0I + AB1I * T2Q + AB1Q * T2I;

	/* B1 */
    BT1I = AB0I * T1I + AB1I * T3I - AB0Q * T1Q;
    BT1Q = AB0Q * T1I + AB0I * T1Q + AB1Q * T3I;

	/* B2 */
    BT2I = AB2I * T0I + AB3I * T2I - AB3Q * T2Q;
    BT2Q = AB2Q * T0I + AB3I * T2Q + AB3Q * T2I;

	/* B3 */
    BT3I = AB2I * T1I + AB3I * T3I - AB2Q * T1Q;
    BT3Q = AB2Q * T1I + AB2I * T1Q + AB3Q * T3I;

	output[2].real		=	-BT0I;
	output[2].imag		=	-BT0Q;
	output[3].real		=	-BT1I;
	output[3].imag		=	-BT1Q;
	output[6].real		=	-BT2I;
	output[6].imag		=	-BT2Q;
	output[7].real		=	-BT3I;
	output[7].imag		=	-BT3Q;


	output[8].real		=	-BT0I;
	output[8].imag		=	BT0Q;
	output[12].real		=	-BT1I;
	output[12].imag		=	BT1Q;
	output[9].real		=	-BT2I;
	output[9].imag		=	BT2Q;
	output[13].real		=	-BT3I;
	output[13].imag		=	BT3Q;

	/* calculate B*C */
	/* B*C should be Hermitian*/
	output[0].real		=	(A0I + BT0I * CA0I + BT1I * CA2I - BT0Q * CA0Q - BT1Q * CA2Q);
	output[0].imag		=	0;

    temp1 = A1I + BT0I * CA1I + BT1I * CA3I - BT0Q * CA1Q - BT1Q * CA3Q;
    temp2 = A1Q + BT0Q * CA1I + BT1I * CA3Q + BT0I * CA1Q + BT1Q * CA3I;
	output[1].real			=	temp1;
	output[1].imag			=	temp2;
	output[4].real			=	temp1;
	output[4].imag			=	-temp2;

	output[5].real		=	(A3I + BT2I * CA1I + BT3I * CA3I - BT2Q * CA1Q - BT3Q * CA3Q);
	output[5].imag		=	0;

}

#ifdef _TMS320C6600
/** 
 *  \fn     void MATRIX_multi4x4MatInvFltOut(
 *                            IN      cplx32_t  * RESTRICT Input,
 *                            IN      int32_t    * RESTRICT localScratch,
 *                            IN      int32_t  N,
 *                            OUT     cplxf_t  * RESTRICT invA);
 *
 *  \brief   Multiple 4x4 matrices (complex positive semi-definitive Hermitian) inversion using block-wisse method.
 * 
 *  \param[in]    Input
 *              Input 4x4 matrices that needs to be inverted, stored sequentially 
 *              in format (0,0), (0,1), (0,2), (0, 3), (1,0), (1,1)...(3, 2), (3, 3).
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    N
 *              Number of input matrices to be inverted. 
 *
 *  \param[in]    localScratch
 *              Pointer to the scratch pad needed for local intermediate buffers. Must be of size 32 * n 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]   invA
 *              Output 4x4 matrices. Stored sequentially as the input. Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */

void 		MATRIX_multi4x4MatInvFltOut (  
								IN  cplx32_t * RESTRICT Input,
								IN  int32_t		N,
								IN  int32_t    * RESTRICT localScratch,
								OUT cplxf_t * RESTRICT invA)	
{
	int32_t		ii;
	int64_t		* RESTRICT inPtr;
	__float2_t		* RESTRICT tempOutPtr;
	float  A0, A3, D0, D3, F0, F3;
	float  detA, oneOverdetA;
	__float2_t A1, D1, F1, C0, C1, C2, C3;
	__float2_t B0, B1, B2, B3;
	__float2_t dtemp, dtemp1, dtemp2, doneOverdetA;
	__x128_t results;
	__float2_t		* RESTRICT CC;
	__float2_t		* RESTRICT FF;
	__float2_t		* RESTRICT DD;
	__float2_t		* RESTRICT CPtr;
	__float2_t		* RESTRICT DPtr;
	__float2_t		* RESTRICT FPtr;


	CC		=	(__float2_t *) &localScratch[0];
	DD		=	(__float2_t *) &localScratch[8 * N];
	FF		=	(__float2_t *) &localScratch[12 * N];


	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(int64_t * ) Input;
	if ( N & 1 )
	{
		#ifdef _TMS320C6X
		#pragma UNROLL(2)
		#endif

		for ( ii = 0; ii < N - 1; ii++ )
		{
			/* Calculate d = inv(d) */
			dtemp			=   _dintsp(_amem8(&inPtr[10]));
			D0				=	_hif2(dtemp);
			dtemp			=	_dintsp(_amem8(&inPtr[15]));
			D3				=	_hif2(dtemp);
			D1				=	_dintsp(_amem8(&inPtr[11]));
			inPtr			+=	16;

			dtemp			=	_dmpysp(D1, D1);
			detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
			oneOverdetA		=	_rcpsp( detA );
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

			dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
			D0				=	_hif2(dtemp1);
			D3				=	_lof2(dtemp1);
			D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
			_amem8_f2(&DPtr[0])	=	dtemp1;
			_amem8_f2(&DPtr[1])	=	D1;
			DPtr			+=	2;
		}

		/* the last odd */
		/* Calculate d = inv(d) */
		dtemp			=   _dintsp(_amem8(&inPtr[10]));
		D0				=	_hif2(dtemp);
		dtemp			=	_dintsp(_amem8(&inPtr[15]));
		D3				=	_hif2(dtemp);
		D1				=	_dintsp(_amem8(&inPtr[11]));
		inPtr			+=	16;

		dtemp			=	_dmpysp(D1, D1);
		detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

		dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
		D0				=	_hif2(dtemp1);
		D3				=	_lof2(dtemp1);
		D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&DPtr[0])	=	dtemp1;
		_amem8_f2(&DPtr[1])	=	D1;
		DPtr			+=	2;

	}
	else
	{
		#ifdef _TMS320C6X
		#pragma UNROLL(2)
		#endif

		for ( ii = 0; ii < N; ii++ )
		{
			/* Calculate d = inv(d) */
			dtemp			=   _dintsp(_amem8(&inPtr[10]));
			D0				=	_hif2(dtemp);
			dtemp			=	_dintsp(_amem8(&inPtr[15]));
			D3				=	_hif2(dtemp);
			D1				=	_dintsp(_amem8(&inPtr[11]));
			inPtr			+=	16;

			dtemp			=	_dmpysp(D1, D1);
			detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
			oneOverdetA		=	_rcpsp( detA );
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

			dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
			D0				=	_hif2(dtemp1);
			D3				=	_lof2(dtemp1);
			D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
			_amem8_f2(&DPtr[0])	=	dtemp1;
			_amem8_f2(&DPtr[1])	=	D1;
			DPtr			+=	2;
		}
	}

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(int64_t * ) Input;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load b */
		B0				=	_dintsp(_amem8(&inPtr[2]));
		B1				=	_dintsp(_amem8(&inPtr[3]));
		B2				=	_dintsp(_amem8(&inPtr[6]));
		B3				=	_dintsp(_amem8(&inPtr[7]));
		inPtr			+=	16;

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);
		DPtr			+=	2;

		/* calculate c = b*inv(d) */
		results			=	_cmpysp(D1, B1);
		C0				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C0				=	_daddsp(C0, _dmpysp(B0, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B0);
		C1				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C1				=	_daddsp(C1, _dmpysp(B1, _ftof2(D3, D3)));
		results			=	_cmpysp(D1, B3);
		C2				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C2				=	_daddsp(C2, _dmpysp(B2, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B2);
		C3				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C3				=	_daddsp(C3, _dmpysp(B3, _ftof2(D3, D3)));
		_amem8_f2(&CPtr[0])	=	C0;
		_amem8_f2(&CPtr[1])	=	C1;
		_amem8_f2(&CPtr[2])	=	C2;
		_amem8_f2(&CPtr[3])	=	C3;
		CPtr			+=	4;
	}
	
	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	inPtr		=	(int64_t * ) Input;
	tempOutPtr	=	(__float2_t *) invA;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load a */
		dtemp			=	_dintsp(_amem8(&inPtr[0]));
		A0				=	_hif2(dtemp);
		dtemp			=	_dintsp(_amem8(&inPtr[5]));
		A3				=	_hif2(dtemp);
		A1				=	_dintsp(_amem8(&inPtr[1]));
		B0				=	_dintsp(_amem8(&inPtr[2]));
		B1				=	_dintsp(_amem8(&inPtr[3]));
		B2				=	_dintsp(_amem8(&inPtr[6]));
		B3				=	_dintsp(_amem8(&inPtr[7]));
		inPtr			+=	16;
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);
		CPtr			+=	4;

		/* calculate f = a - b *inv(d) * conj(b) -- Hermitian */
		results			=	_cmpysp(B0, C0);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B1, C1);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F0				=	A0 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C2);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B3, C3);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F3				=	A3 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C0);
		F1				=	_dsubsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(B3, C1);
		F1				=	_dsubsp(F1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		/* Calculate f = inv(f) */
		dtemp			=	_dmpysp(F1, F1);
		detA			=	F0 * F3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		//oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);
		
		dtemp1			=	_dmpysp(_ftof2(F3, F0), doneOverdetA);
		F1				=	_dmpysp(F1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&FPtr[0])	=	dtemp1;
		_amem8_f2(&FPtr[1])	=	F1;
		FPtr			+=	2;

		/* NW output */
		_amem8_f2(&tempOutPtr[0])	=	_lltof2(_f2toll(dtemp1) & 0xFFFFFFFF00000000); //_ftod(_hif(dtemp1), 0.f); //_lltod(_dtoll(dtemp1) & 0xFFFFFFFF00000000);
		_amem8_f2(&tempOutPtr[1])	=	F1;
		_amem8_f2(&tempOutPtr[5])	=	_ftof2(_lof2(dtemp1), 0.f);
		_amem8_f2(&tempOutPtr[4])	=	_lltof2(_f2toll(F1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	} 

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	tempOutPtr	=	(__float2_t *) invA;
	for ( ii = 0; ii < N; ii++ )
	{
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);

		dtemp			=	_amem8_f2(&FPtr[0]);
		F0				=	_hif2(dtemp);
		F3				=	_lof2(dtemp);
		F1				=	_amem8_f2(&FPtr[1]);
		CPtr			+=	4;
		DPtr			+=	2;
		FPtr			+=	2;

		/* NE output = - f * c, SW = conj(NW)*/
		results			=	_cmpysp(F1, C2);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp1			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C0, _ftof2(F0, F0)));
		_amem8_f2(&tempOutPtr[2])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[8])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C3);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp2			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C1, _ftof2(F0, F0)));
		B1				=	dtemp;
		_amem8_f2(&tempOutPtr[3])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[12])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C0);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C2, _ftof2(F3, F3)));
		_amem8_f2(&tempOutPtr[6])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[9])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C3, _ftof2(F3, F3)));
		B3				=	dtemp;
		_amem8_f2(&tempOutPtr[7])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[13])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
		

		
		/* SE output */
		/* inv(d) - conj(c) * inv(f) * c, whrer c = b * inv(d) */
		dtemp			=	_dmpysp(C0, C0);
		A0				=	D0 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C2, C2);
		A0				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C0, dtemp1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A0				+=	2.f * _hif2(dtemp);

		dtemp			=	_dmpysp(C1, C1);
		A3				=	D3 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C3, C3);
		A3				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C1, dtemp2);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A3				+=	2.f * _hif2(dtemp);
		_amem8_f2(&tempOutPtr[10])=	_ftof2(A0, 0.f);
		_amem8_f2(&tempOutPtr[15])=	_ftof2(A3, 0.f);

		results			=	_cmpysp(C0, B1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A1				=	_daddsp(D1, dtemp);
		results			=	_cmpysp(C2, B3);
		A1				=	_daddsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		_amem8_f2(&tempOutPtr[11])=	A1;
		_amem8_f2(&tempOutPtr[14])=	_lltof2(_f2toll(A1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	} 
}




/** 
 *  \fn     void MATRIX_multi4x4MatInvFxdOut(
 *                            IN      cplx32_t  * RESTRICT Input,
 *                            IN      int32_t    * RESTRICT localScratch,
 *                            IN      int32_t   range,
 *                            OUT     int32_t	* RESTRICT Qinv,
 *                            OUT     cplx16_t  * RESTRICT invA);
 *
 *  \brief   Multiple 4x4 matrices (complex positive semi-definitive Hermitian) inversion using block-wisse method.
 * 
 *  \param[in]    Input
 *              Input 4x4 matrices that needs to be inverted, stored sequentially 
 *              in format (0,0), (0,1), (0,2), (0, 3), (1,0), (1,1)...(3, 2), (3, 3).
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    N
 *              Number of input matrices to be inverted. N must be multiple of 4.
 *
 *  \param[in]    localScratch
 *              Pointer to the scratch pad needed for local intermediate buffers. Must be of size 32 * n 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]   range
 *              The full range of the output. For example, if we set it to 14, that means the real and imaginary
 *              part of the output elements will not exceed 14-bit with sign.
 *
 *  \param[out]   Qinv
 *              Output block Q value for the inverse matrices.
 *
 *  \param[out]   invA
 *              Output 4x4 matrices. Stored sequentially as the input. Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */

void 		MATRIX_multi4x4MatInvFxdOut (  
								IN  cplx32_t * RESTRICT Input,
								IN  int32_t		N,
								IN  int32_t    * RESTRICT localScratch,
								IN  int32_t   range,
								OUT int32_t	* RESTRICT Qinv,
								OUT cplx16_t * RESTRICT invA)	
{
	int32_t		ii;
	int64_t		* RESTRICT inPtr;
	float  A0, A3, D0, D3, F0, F3;
	float  detA, oneOverdetA;
	__float2_t A1, D1, F1, C0, C1, C2, C3;
	__float2_t B0, B1, B2, B3;
	__float2_t dtemp, dtemp1, dtemp2, doneOverdetA;
	__x128_t results;
	__float2_t		* RESTRICT CC;
	__float2_t		* RESTRICT FF;
	__float2_t		* RESTRICT DD;
	__float2_t		* RESTRICT CPtr;
	__float2_t		* RESTRICT DPtr;
	__float2_t		* RESTRICT FPtr;
	__float2_t		* RESTRICT tempOut;
	__float2_t		* RESTRICT tempOutPtr;
	float		magic, max;
	int32_t     head = 16 - range;
	int32_t		* RESTRICT OutPtr;
	int64_t		lltemp1, lltemp2, lltemp3, lltemp4;
	__float2_t	magicPair;


	CC		=	(__float2_t *) &localScratch[0];
	DD		=	(__float2_t *) &localScratch[8 * N];
	FF		=	(__float2_t *) &localScratch[12 * N];
	tempOut	=	(__float2_t *) &localScratch[16 * N];


	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(int64_t * ) Input;

	if ( N & 1 )
	{
		#ifdef _TMS320C6X
		#pragma UNROLL(2)
		#endif

		for ( ii = 0; ii < N - 1; ii++ )
		{
			/* Calculate d = inv(d) */
			dtemp			=   _dintsp(_amem8(&inPtr[10]));
			D0				=	_hif2(dtemp);
			dtemp			=	_dintsp(_amem8(&inPtr[15]));
			D3				=	_hif2(dtemp);
			D1				=	_dintsp(_amem8(&inPtr[11]));
			inPtr			+=	16;

			dtemp			=	_dmpysp(D1, D1);
			detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
			oneOverdetA		=	_rcpsp( detA );
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

			dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
			D0				=	_hif2(dtemp1);
			D3				=	_lof2(dtemp1);
			D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
			_amem8_f2(&DPtr[0])	=	dtemp1;
			_amem8_f2(&DPtr[1])	=	D1;
			DPtr			+=	2;
		}

		/* the last odd */
		/* Calculate d = inv(d) */
		dtemp			=   _dintsp(_amem8(&inPtr[10]));
		D0				=	_hif2(dtemp);
		dtemp			=	_dintsp(_amem8(&inPtr[15]));
		D3				=	_hif2(dtemp);
		D1				=	_dintsp(_amem8(&inPtr[11]));
		inPtr			+=	16;

		dtemp			=	_dmpysp(D1, D1);
		detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

		dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
		D0				=	_hif2(dtemp1);
		D3				=	_lof2(dtemp1);
		D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&DPtr[0])	=	dtemp1;
		_amem8_f2(&DPtr[1])	=	D1;
		DPtr			+=	2;

	}
	else
	{
		#ifdef _TMS320C6X
		#pragma UNROLL(2)
		#endif

		for ( ii = 0; ii < N; ii++ )
		{
			/* Calculate d = inv(d) */
			dtemp			=   _dintsp(_amem8(&inPtr[10]));
			D0				=	_hif2(dtemp);
			dtemp			=	_dintsp(_amem8(&inPtr[15]));
			D3				=	_hif2(dtemp);
			D1				=	_dintsp(_amem8(&inPtr[11]));
			inPtr			+=	16;

			dtemp			=	_dmpysp(D1, D1);
			detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
			oneOverdetA		=	_rcpsp( detA );
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
			doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

			dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
			D0				=	_hif2(dtemp1);
			D3				=	_lof2(dtemp1);
			D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
			_amem8_f2(&DPtr[0])	=	dtemp1;
			_amem8_f2(&DPtr[1])	=	D1;
			DPtr			+=	2;
		}
	}

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(int64_t * ) Input;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load b */
		B0				=	_dintsp(_amem8(&inPtr[2]));
		B1				=	_dintsp(_amem8(&inPtr[3]));
		B2				=	_dintsp(_amem8(&inPtr[6]));
		B3				=	_dintsp(_amem8(&inPtr[7]));
		inPtr			+=	16;

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);
		DPtr			+=	2;

		/* calculate c = b*inv(d) */
		results			=	_cmpysp(D1, B1);
		C0				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C0				=	_daddsp(C0, _dmpysp(B0, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B0);
		C1				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C1				=	_daddsp(C1, _dmpysp(B1, _ftof2(D3, D3)));
		results			=	_cmpysp(D1, B3);
		C2				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C2				=	_daddsp(C2, _dmpysp(B2, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B2);
		C3				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C3				=	_daddsp(C3, _dmpysp(B3, _ftof2(D3, D3)));
		_amem8_f2(&CPtr[0])	=	C0;
		_amem8_f2(&CPtr[1])	=	C1;
		_amem8_f2(&CPtr[2])	=	C2;
		_amem8_f2(&CPtr[3])	=	C3;
		CPtr			+=	4;
	}
	
	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	inPtr		=	(int64_t * ) Input;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load a */
		dtemp			=	_dintsp(_amem8(&inPtr[0]));
		A0				=	_hif2(dtemp);
		dtemp			=	_dintsp(_amem8(&inPtr[5]));
		A3				=	_hif2(dtemp);
		A1				=	_dintsp(_amem8(&inPtr[1]));
		B0				=	_dintsp(_amem8(&inPtr[2]));
		B1				=	_dintsp(_amem8(&inPtr[3]));
		B2				=	_dintsp(_amem8(&inPtr[6]));
		B3				=	_dintsp(_amem8(&inPtr[7]));
		inPtr			+=	16;
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);
		CPtr			+=	4;

		/* calculate f = a - b *inv(d) * conj(b) -- Hermitian */
		results			=	_cmpysp(B0, C0);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B1, C1);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F0				=	A0 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C2);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B3, C3);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F3				=	A3 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C0);
		F1				=	_dsubsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(B3, C1);
		F1				=	_dsubsp(F1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		/* Calculate f = inv(f) */
		dtemp			=	_dmpysp(F1, F1);
		detA			=	F0 * F3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		//oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);
		
		dtemp1			=	_dmpysp(_ftof2(F3, F0), doneOverdetA);
		F1				=	_dmpysp(F1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&FPtr[0])	=	dtemp1;
		_amem8_f2(&FPtr[1])	=	F1;
		FPtr			+=	2;
	} 

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	tempOutPtr	=	tempOut;
	for ( ii = 0; ii < N; ii++ )
	{
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);

		dtemp			=	_amem8_f2(&FPtr[0]);
		F0				=	_hif2(dtemp);
		F3				=	_lof2(dtemp);
		F1				=	_amem8_f2(&FPtr[1]);
		CPtr			+=	4;
		DPtr			+=	2;
		FPtr			+=	2;

		/* NE output = - f * c, SW = conj(NW)*/
		results			=	_cmpysp(F1, C2);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp1			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C0, _ftof2(F0, F0)));
		_amem8_f2(&tempOutPtr[0])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);

		results			=	_cmpysp(F1, C3);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp2			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C1, _ftof2(F0, F0)));
		B1				=	dtemp;
		_amem8_f2(&tempOutPtr[1])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);

		results			=	_cmpysp(F1, C0);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C2, _ftof2(F3, F3)));
		_amem8_f2(&tempOutPtr[2])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);

		results			=	_cmpysp(F1, C1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C3, _ftof2(F3, F3)));
		B3				=	dtemp;
		_amem8_f2(&tempOutPtr[3])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		

		
		/* SE output */
		/* inv(d) - conj(c) * inv(f) * c, whrer c = b * inv(d) */
		dtemp			=	_dmpysp(C0, C0);
		A0				=	D0 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C2, C2);
		A0				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C0, dtemp1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A0				+=	2.f * _hif2(dtemp);

		dtemp			=	_dmpysp(C1, C1);
		A3				=	D3 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C3, C3);
		A3				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C1, dtemp2);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A3				+=	2.f * _hif2(dtemp);
		_amem8_f2(&tempOutPtr[4])=	_ftof2(A0, A3);

		results			=	_cmpysp(C0, B1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A1				=	_daddsp(D1, dtemp);
		results			=	_cmpysp(C2, B3);
		A1				=	_daddsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		_amem8_f2(&tempOutPtr[5])=	A1;

		tempOutPtr		+=	6;
	} 

	
	FPtr		=	(__float2_t *) FF;
	tempOutPtr	=	tempOut;
	OutPtr		=	(int32_t *) invA;
	for ( ii = 0; ii < N; ii++ )
	{
		dtemp1			=	_amem8_f2(&FPtr[0]);
		F1				=	_amem8_f2(&FPtr[1]);
		FPtr			+=	2;

		max				=	_hif2(dtemp1);
		if ( _lof2(dtemp1) > max )
			max			=	_lof2(dtemp1);

		dtemp2			=	_amem8_f2(&tempOutPtr[4]);
		A1				=	_amem8_f2(&tempOutPtr[5]);
		if (_hif2(dtemp2) > max )
			max			=	_hif2(dtemp2);
		if (_lof2(dtemp2) > max )
			max			=	_lof2(dtemp2);
		magic			=	_itof(0x04C00000 + (head<<23) + (_ftoi(max) & 0xFF800000));
		Qinv[ii]			=	(127 + 23) - (_ftoi(magic) >> 23);
		magicPair		=	_fdmv_f2(magic, magic);

		/* NW output */
		lltemp1			=	_f2toll(_daddsp(dtemp1, magicPair));
		lltemp2			=	_f2toll(_daddsp(F1, magicPair));
		#ifdef _BIG_ENDIAN
		_amem8(&OutPtr[0])	=	_itoll(_pack2(_hill(lltemp1), 0), _pack2(_hill(lltemp2), _loll(lltemp2)));
		lltemp2			=	_dapys2(0x1000100010008000, lltemp2);
		_amem8(&OutPtr[4])	=	_itoll(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_loll(lltemp1), 0));
		#else			
		_amem8(&OutPtr[0])	=	_itoll(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_hill(lltemp1), 0));
		lltemp2			=	_dapys2(0x1000100010008000, lltemp2);
		_amem8(&OutPtr[4])	=	_itoll(_pack2(_loll(lltemp1), 0), _pack2(_hill(lltemp2), _loll(lltemp2)));
		#endif

		/* NE and SW output */
		C0				=	_amem8_f2(&tempOutPtr[0]);
		C1				=	_amem8_f2(&tempOutPtr[1]);
		C2				=	_amem8_f2(&tempOutPtr[2]);
		C3				=	_amem8_f2(&tempOutPtr[3]);
		tempOutPtr		+=	6;
		#ifdef _BIG_ENDIAN
		lltemp1			=	_f2toll(_daddsp(C0, magicPair));
		lltemp2			=	_f2toll(_daddsp(C1, magicPair));
		lltemp3			=	_dmv(_pack2(_hill(lltemp1), _loll(lltemp1)), _pack2(_hill(lltemp2), _loll(lltemp2)));
		lltemp1			=	_f2toll(_daddsp(C2, magicPair));
		lltemp2			=	_f2toll(_daddsp(C3, magicPair));
		lltemp4			=	_dmv(_pack2(_hill(lltemp1), _loll(lltemp1)), _pack2(_hill(lltemp2), _loll(lltemp2)));
		_amem8(&OutPtr[2])	=	lltemp3;
		_amem8(&OutPtr[6])	=	lltemp4;
		lltemp3			=	_dapys2(0x1000800010008000, lltemp3);
		lltemp4			=	_dapys2(0x1000800010008000, lltemp4);
		_amem8(&OutPtr[8])	=	_itoll(_hill(lltemp3), _hill(lltemp4));
		_amem8(&OutPtr[12])	=	_itoll(_loll(lltemp3), _loll(lltemp4));
		#else			
		lltemp1			=	_f2toll(_daddsp(C0, magicPair));
		lltemp2			=	_f2toll(_daddsp(C1, magicPair));
		lltemp3			=	_dmv(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_hill(lltemp1), _loll(lltemp1)));
		lltemp1			=	_f2toll(_daddsp(C2, magicPair));
		lltemp2			=	_f2toll(_daddsp(C3, magicPair));
		lltemp4			=	_dmv(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_hill(lltemp1), _loll(lltemp1)));
		_amem8(&OutPtr[2])	=	lltemp3;
		_amem8(&OutPtr[6])	=	lltemp4;
		lltemp3			=	_dapys2(0x1000800010008000, lltemp3);
		lltemp4			=	_dapys2(0x1000800010008000, lltemp4);
		_amem8(&OutPtr[8])	=	_itoll(_loll(lltemp4), _loll(lltemp3));
		_amem8(&OutPtr[12])	=	_itoll(_hill(lltemp4), _hill(lltemp3));
		#endif


		/* SE output */
		lltemp1			=	_f2toll(_daddsp(dtemp2, magicPair));
		lltemp2			=	_f2toll(_daddsp(A1, magicPair));
		#ifdef _BIG_ENDIAN
		_amem8(&OutPtr[10])	=	_itoll(_pack2(_hill(lltemp1), 0), _pack2(_hill(lltemp2), _loll(lltemp2)));
		lltemp2			=	_dapys2(0x1000100010008000, lltemp2);
		_amem8(&OutPtr[14])	=	_itoll(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_loll(lltemp1), 0));
		#else			
		_amem8(&OutPtr[10])	=	_itoll(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_hill(lltemp1), 0));
		lltemp2			=	_dapys2(0x1000100010008000, lltemp2);
		_amem8(&OutPtr[14])	=	_itoll(_pack2(_loll(lltemp1), 0), _pack2(_hill(lltemp2), _loll(lltemp2)));
		#endif

		OutPtr			+=	16;

	} 
}
#endif
