/**
 *   @file  MATRIX_cholesky.c
 *
 *   @brief
 *      Cholesky Decomposition and triangular matrix solver
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

#ifdef _TMS320C6X
#include <c6x.h>
#endif

#include <common/src/dpu/capon3d/modules/postProcessing/matrixFunc/api/MATRIX_cholesky.h>

#define DEBUG_ON (1)



void 
MATRIX_low_tri_solve(
	int32_t * L,  
	int32_t * Y,  
	int32_t * V,  
    int32_t n
	)
{
int32_t i,j;
int32_t Vr, Vi;
int32_t LYr, LYi;
int32_t frac,rexp;

	for ( j = 0; j < n; j++ ) 
	{
		int64_t Vi_Vr, Li_Lr, Yi_Yr, Di_Dr;
    
    	Vi_Vr	= _amem8(&V[2*j]);
		#ifdef _LITTLE_ENDIAN
		Vr		=	_loll(Vi_Vr); 
		Vi		=	_hill(Vi_Vr);
		#else
		Vr		=	_hill(Vi_Vr); 
		Vi		=	_loll(Vi_Vr);
		#endif

    	Vr		>>=	3;
		Vi		>>=	3;
    	
    	for ( i = 0; i < j; i++ ) 
		{
    		Li_Lr     = _amem8(&L[2*(n*j+i)]);
    		Yi_Yr     = _amem8(&Y[2*i]);
			#ifdef _LITTLE_ENDIAN
	        LYr = frac_dotpn2(_loll(Li_Lr),_loll(Yi_Yr), _hill(Li_Lr),_hill(Yi_Yr));   
            LYi = frac_dotp2 (_loll(Li_Lr),_hill(Yi_Yr), _hill(Li_Lr),_loll(Yi_Yr));
			#else
	        LYr = frac_dotpn2(_hill(Li_Lr),_hill(Yi_Yr), _loll(Li_Lr),_loll(Yi_Yr));   
            LYi = frac_dotp2 (_hill(Li_Lr),_loll(Yi_Yr), _loll(Li_Lr),_hill(Yi_Yr));
			#endif
            Vr = _ssub(Vr, LYr);                                  
            Vi = _ssub(Vi, LYi);                                           
    	}
    	Di_Dr     = _amem8(&L[2*(n*j+j)]);
			#ifdef _LITTLE_ENDIAN
        recip_32(_loll(Di_Dr), 0, &frac,&rexp);
    	_amem8(&Y[2*j]) = _itoll( _sshl(_smpy32_64(Vi,frac),rexp), _sshl(_smpy32_64(Vr,frac),rexp) );
			#else
        recip_32(_hill(Di_Dr), 0, &frac,&rexp);
    	_amem8(&Y[2*j]) = _itoll(  _sshl(_smpy32_64(Vr,frac),rexp),_sshl( _smpy32_64(Vi,frac),rexp) );
			#endif
	}		
}

void 
MATRIX_up_tri_solve(
	int32_t * L,  
	int32_t * Z,  
	int32_t * Y,  
    int32_t n
	)
{
int32_t i,j;
int32_t Yr, Yi;
int32_t LZr, LZi;
int32_t frac,rexp;


	for(j = n - 1; j >= 0;j-- ) 
	{
		int64_t Zi_Zr, Li_Lr, Yi_Yr, Di_Dr;
    
    	Yi_Yr	=	_amem8(&Y[2*j]);
		#ifdef _LITTLE_ENDIAN
		Yr		=	_loll(Yi_Yr); 
		Yi		=	_hill(Yi_Yr);
		#else
		Yr		=	_hill(Yi_Yr); 
		Yi		=	_loll(Yi_Yr);
		#endif

    	for( i = n - 1; i > j; i-- ) 
		{
    		Li_Lr     = _amem8(&L[2*(n*i+j)]);
    		Zi_Zr     = _amem8(&Z[2*i]);
			#ifdef _LITTLE_ENDIAN
            LZr = frac_dotp2  (_loll(Li_Lr),_loll(Zi_Zr), _hill(Li_Lr),_hill(Zi_Zr));
	        LZi = frac_dotpn2 (_loll(Li_Lr),_hill(Zi_Zr), _hill(Li_Lr),_loll(Zi_Zr));   
			#else
            LZr = frac_dotp2  (_hill(Li_Lr),_hill(Zi_Zr), _loll(Li_Lr),_loll(Zi_Zr));
	        LZi = frac_dotpn2 (_hill(Li_Lr),_loll(Zi_Zr), _loll(Li_Lr),_hill(Zi_Zr));   
			#endif
            Yr = _ssub(Yr, LZr);                                  
            Yi = _ssub(Yi, LZi);                                           
    	}
    	Di_Dr     = _amem8(&L[2*(n*j+j)]);
			#ifdef _LITTLE_ENDIAN
        recip_32(_loll(Di_Dr), 0, &frac,&rexp);
    	_amem8(&Z[2*j]) = _itoll( _sshl(_smpy32_64(Yi,frac),rexp), _sshl(_smpy32_64(Yr,frac),rexp) );
			#else
        recip_32(_hill(Di_Dr), 0, &frac,&rexp);
    	_amem8(&Z[2*j]) = _itoll( _sshl(_smpy32_64(Yr,frac),rexp), _sshl(_smpy32_64(Yi,frac),rexp)  );
			#endif
	}		
}



void 
MATRIX_cholesky(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t n
	)                                      
{                                                                     
	int32_t i, j, k;                                                        
	int32_t sq, exp;                                                        
	int32_t sum0, sum1;                                     
	int32_t b0, b1;                                                         
	int32_t jqg;  
                                                                    
	for (j = 0; j < n-1; j++)    	
	{                                                                   
	    recip_sqrt32(A[2 * n * j + 2 * j], 0, &sq, &exp);   
	    	    
	    for (k = j; k < n; k++ )  
		{
			int64_t qg_re_im;  int32_t jq;

            jqg			=	( n * k + j ) << 1;  
			jq			=	_mvd(jqg);  /* copy data */
            
			qg_re_im	=	_amem8(&A[jqg]);

			#ifdef _LITTLE_ENDIAN
	        sum0		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
	        sum1		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ;            
            _amem8(&Ap[jq])		=	_itoll(sum1,sum0);
			#else
	        sum0		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ;            
	        sum1		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
            _amem8(&Ap[jq])		=	_itoll(sum0,sum1);
			#endif
	    }                                                               

	    for( k = j + 1; k < n; k++ )   
		{                                                               
			int64_t a2_a0;               
	    
	        a2_a0		=	_amem8(&A[2 * (n * k + j)]);    
	        for( i = j + 1; i < k + 1; i++ )     
			{                                                           
				int64_t sum1_sum0, a3_a1;
		       
				sum1_sum0	=	_amem8(&A[2 * ( n * k + i )]);
				a3_a1		=	_amem8(&A[2 * ( n * i + j )]);  
				#ifdef _LITTLE_ENDIAN
				b0			=	frac_dotp2 (_loll(a2_a0),_loll(a3_a1), _hill(a2_a0),_hill(a3_a1));   
		        b1			=	frac_dotpn2(_hill(a2_a0),_loll(a3_a1), _loll(a2_a0),_hill(a3_a1));
	            sum0		=	_ssub(_loll(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_hill(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)])	=	_itoll(sum1,sum0);
				#else
		        b0			=	frac_dotp2 (_hill(a2_a0),_hill(a3_a1), _loll(a2_a0),_loll(a3_a1));   
		        b1			=	(frac_dotpn2(_loll(a2_a0),_hill(a3_a1), _hill(a2_a0),_loll(a3_a1)));
	            sum0		=	_ssub(_hill(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_loll(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)]) =	_itoll(sum0,sum1);
				#endif
	        }                                                           
	    }                                                               
	}  
	
	recip_sqrt32(A[2*n*n-2], 0, &sq, &exp);                          
	sum0		=	_sshl(_smpy32_64(A[2*n*n-2], sq), exp);                         
	#ifdef _LITTLE_ENDIAN
    _amem8(&Ap[2*n*n-2]) = _itoll(0,sum0);
	#else
    _amem8(&Ap[2*n*n-2]) = _itoll(sum0,0);
	#endif
}           




void 
MATRIX_cholesky_flp(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t n
	)                                      
{                                                                     
	int32_t i, j, k;                                                        
	//int32_t sq, exp;                                                        
	int32_t sum0, sum1;                                     
	int32_t b0, b1;                                                         
	int32_t jqg;  
	float invsqrt, ftemp;
                                                                    
	for (j = 0; j < n-1; j++)    	
	{                                                                   
	    //recip_sqrt32(A[2 * n * j + 2 * j], 0, &sq, &exp);
		ftemp		=	(float) (A[2 * n * j + 2 * j]);
		invsqrt		=	_rsqrsp(ftemp * 2.f);
		invsqrt		=	invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
		invsqrt		=	65536.f * invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	    	    
	    for (k = j; k < n; k++ )  
		{
			int64_t qg_re_im;  int32_t jq;

            jqg			=	( n * k + j ) << 1;  
			jq			=	_mvd(jqg);  /* copy data */
            
			qg_re_im	=	_amem8(&A[jqg]);

			#ifdef _LITTLE_ENDIAN
	        //sum0		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
	        //sum1		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ; 
			sum0		=	_loll(qg_re_im);
	        sum0		=	(int32_t) (((float) sum0) *  invsqrt + 0.5f);
			sum1		=	_hill(qg_re_im);
	        sum1		=	(int32_t) (((float) sum1) *  invsqrt + 0.5f);            
            _amem8(&Ap[jq])		=	_itoll(sum1, sum0);
			#else
	        //sum0		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ;            
	        //sum1		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
			sum0		=	_hill(qg_re_im);
	        sum0		=	(int32_t) (((float) sum0) *  invsqrt + 0.5f);
			sum1		=	_loll(qg_re_im);
	        sum1		=	(int32_t) (((float) sum1) *  invsqrt + 0.5f);            
            _amem8(&Ap[jq])		=	_itoll(sum0, sum1);
			#endif
	    }                                                               

	    for( k = j + 1; k < n; k++ )   
		{                                                               
			int64_t a2_a0;               
	    
	        a2_a0		=	_amem8(&A[2 * (n * k + j)]);    
	        for( i = j + 1; i < k + 1; i++ )     
			{                                                           
				int64_t sum1_sum0, a3_a1;
		       
				sum1_sum0	=	_amem8(&A[2 * ( n * k + i )]);
				a3_a1		=	_amem8(&A[2 * ( n * i + j )]);  
				#ifdef _LITTLE_ENDIAN
				b0			=	frac_dotp2 (_loll(a2_a0),_loll(a3_a1), _hill(a2_a0),_hill(a3_a1));   
		        b1			=	frac_dotpn2(_hill(a2_a0),_loll(a3_a1), _loll(a2_a0),_hill(a3_a1));
	            sum0		=	_ssub(_loll(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_hill(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)])	=	_itoll(sum1,sum0);
				#else
		        b0			=	frac_dotp2 (_hill(a2_a0),_hill(a3_a1), _loll(a2_a0),_loll(a3_a1));   
		        b1			=	(frac_dotpn2(_loll(a2_a0),_hill(a3_a1), _hill(a2_a0),_loll(a3_a1)));
	            sum0		=	_ssub(_hill(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_loll(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)]) =	_itoll(sum0,sum1);
				#endif
	        }                                                           
	    }                                                               
	}  
	
	//recip_sqrt32(A[2*n*n-2], 0, &sq, &exp);                          
	//sum0		=	_sshl(_smpy32_64(A[2*n*n-2], sq), exp);                         
	ftemp		=	(float) (A[2*n*n-2]);
	invsqrt		=	_rsqrsp(ftemp * 2.f);
	//invsqrt		=	_rsqrsp(ftemp);
	invsqrt		=	invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	invsqrt		=	32768.f * invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	sum0		=	(int32_t) (ftemp * invsqrt *  2.f + 0.5f);
	#ifdef _LITTLE_ENDIAN
    _amem8(&Ap[2*n*n-2]) = _itoll(0,sum0);
	#else
    _amem8(&Ap[2*n*n-2]) = _itoll(sum0,0);
	#endif
}           


void 
MATRIX_cholesky_flp_full(
	cplxf_t * RESTRICT A, 
	cplxf_t * Ap,
	int32_t n
	)                                      
{                                                                     
	int32_t i, j, k;                                                        
	float invsqrt, ftemp;
	__float2_t invsqrt2;
	__float2_t * RESTRICT ptrA;
	__float2_t * RESTRICT ptrAp;
	__float2_t qg_re_im;  
     

	ptrA	=	(__float2_t *)A;
	ptrAp	=	(__float2_t *)Ap;

	for (j = 0; j < n-1; j++)    	
	{                                                                   
		ftemp		=	 A[n * j + j].real;
		invsqrt		=	_rsqrsp(ftemp);
		invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
		invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
		invsqrt2	=	_ftof2(invsqrt, invsqrt);
	    	    
	
        for (k = j; k < n; k++ )
        {

            qg_re_im    =   _amem8_f2(&ptrA[n * k + j]);
            _amem8_f2(&ptrAp[n * k + j])        =   _dmpysp(qg_re_im, invsqrt2);
        }

        for( k = j + 1; k < n; k++ )
        {
            __float2_t a2_a0;

            a2_a0       =   _amem8_f2(&ptrA[n * k + j]);
            for( i = j + 1; i < k + 1; i++ )
            {
                __float2_t sum1_sum0, a3_a1;

                sum1_sum0   =   _amem8_f2(&ptrA[ n * k + i]);
                a3_a1       =   _amem8_f2(&ptrA[ n * i + j]);
                _amem8_f2(&ptrAp[n*k+i])    =   _dsubsp(sum1_sum0, _complex_conjugate_mpysp(a3_a1, a2_a0));
            }
        }
	}

	ftemp		=	A[n*n-1].real;
	invsqrt		=	_rsqrsp(ftemp);
	invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
	invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
	ftemp		=	ftemp * invsqrt;
    _amem8_f2(&Ap[n*n-1]) = _ftof2(ftemp, 0.f);
}           


void 
MATRIX_cholesky_flp_inv(
	cplxf_t * RESTRICT A, 
	cplxf_t * Ap,
	int32_t n
	)                                      
{                                                                     
	int32_t i, j, k;                                                        
	float invsqrt, ftemp, si;
	__float2_t invsqrt2, f2temp, inv2;
	__float2_t * RESTRICT ptrA;
	__float2_t * RESTRICT ptrA1;
	__float2_t qg_re_im;  
     
	/*find conjugate of upper Cholesky solution of  A*/
	ptrA	=	(__float2_t *)A;
	ptrA1	=	(__float2_t *)A;

	for (j = 0; j < n-1; j++)    	
	{                                                                   
		ftemp		=	 _hif2(_amem8_f2(&ptrA[n * j + j]));
		invsqrt		=	_rsqrsp(ftemp);
		invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
		invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
		invsqrt2	=	_ftof2(invsqrt, invsqrt);
	    	    
        for (k = j; k < n; k++ )
        {
            qg_re_im    =   _amem8_f2(&ptrA[n * j + k]);
            _amem8_f2(&ptrA1[n * j + k])        =   _dmpysp(qg_re_im, invsqrt2);
        }

        for( k = j + 1; k < n; k++ )
        {
            __float2_t a2_a0;

            a2_a0       =   _amem8_f2(&ptrA[n * j + k]);
            for( i = j + 1; i < k + 1; i++ )
            {
                __float2_t sum1_sum0, a3_a1;

                sum1_sum0   =   _amem8_f2(&ptrA[ n * i + k]);
                a3_a1       =   _amem8_f2(&ptrA[ n * j + i]);
                _amem8_f2(&ptrA1[n*i+k])    =   _dsubsp(sum1_sum0, _complex_conjugate_mpysp(a3_a1, a2_a0));
            }
        }
	}  
	
	ftemp		=	_hif2(_amem8_f2(&ptrA[n * n - 1]));
	invsqrt		=	_rsqrsp(ftemp);
	invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
	invsqrt		=	invsqrt * (1.5f - 0.5f * ftemp * invsqrt * invsqrt);
	ftemp		=	ftemp * invsqrt;
    _amem8_f2(&ptrA[n*n-1]) = _ftof2(ftemp, 0.f);

	/*find inverse of A*/
	//initialize output buffer
	/*m			=	(n * n) >> 1;	
	ptrA1	=	(__float2_t *)Ap;
	for (i = 0; i < m; i++)
	{
		_amem8_f2(ptrA1++)	=	_ftof2(0.f, 0.f);
		_amem8_f2(ptrA1++)	=	_ftof2(0.f, 0.f);
	}*/
	ptrA1	=	(__float2_t *)Ap;
	ptrA	=	(__float2_t *)A;
	for (i = 0; i < n; i++)
	{
		ftemp		=	_hif2(_amem8_f2(&ptrA[n * i + i]));
		si			=	_rcpsp(ftemp);
		si			=	si * (2.f - ftemp * si);
		si			=	si * (2.f - ftemp * si);
		inv2		=	_ftof2(si, si);
        _amem8_f2(&ptrA[n*i + i]) = _ftof2(si, 0.f);
		for (j = 0; j < n; j++)
		{
			_amem8_f2(&ptrA[i * n + j])		=	_dmpysp(inv2, _amem8_f2(&ptrA[i * n + j]));
		}
	}

	for (j = n-1; j >= 0; j--)
	{
		f2temp	=	_amem8_f2(&ptrA[n * j + j]);
		for (i = j + 1; i < n; i++ )
		{
			f2temp	=	_dsubsp(f2temp, _complex_conjugate_mpysp(_amem8_f2(&ptrA[n * j + i]), _amem8_f2(&ptrA1[n * i + j])));
		}
		//_amem8_f2(&ptrA1[n * j + j])	=	_complex_mpysp(f2temp, _amem8_f2(&ptrA[j * n + j]));
        _amem8_f2(&ptrA1[n * j + j])    =   f2temp;

		for (i = j - 1; i >= 0; i-- )
		{
			f2temp	=	_ftof2(0.f, 0.f);
			for (k = i + 1; k < n; k++ )
			{
				f2temp	=	_dsubsp(f2temp, _complex_conjugate_mpysp(_amem8_f2(&ptrA[n * i + k]), _amem8_f2(&ptrA1[n * k + j])));
			}
			//f2temp                          =   _complex_mpysp(f2temp, _amem8_f2(&ptrA[i * n + i]));
			_amem8_f2(&ptrA1[n * i + j])	=	f2temp;
			_amem8_f2(&ptrA1[n * j + i])	=	_ftof2(_hif2(f2temp), -_lof2(f2temp));
			//_amem8_f2(&ptrA1[n * j + i])	=	f2temp;
		}

	}

}           



#define N2 (2)

void 
MATRIX_cholesky_2(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t	x
	)                                      
{                                                                     
	int32_t i, j, k;                                                        
	//int32_t sq, exp;                                                        
	int32_t sum0, sum1;                                     
	int32_t b0, b1, n=2;                                                         
	int32_t jqg;  
	float invsqrt, ftemp;
                     
#ifdef _TMS320C6X
	_nassert(n==2);                                                                    
#endif	
	for (j = 0; j < n-1; j++)    	
	{                                                                   
	    //recip_sqrt32(A[2 * n * j + 2 * j], 0, &sq, &exp);
		ftemp		=	(float) (A[2 * n * j + 2 * j]);
		invsqrt		=	_rsqrsp(ftemp * 2.f);
		invsqrt		=	invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
		invsqrt		=	65536.f * invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	    	    
	    for (k = j; k < n; k++ )  
		{
			int64_t qg_re_im;  int32_t jq;

            jqg			=	( n * k + j ) << 1;  
			jq			=	_mvd(jqg);  /* copy data */
            
			qg_re_im	=	_amem8(&A[jqg]);

			#ifdef _LITTLE_ENDIAN
	        //sum0		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
	        //sum1		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ; 
			sum0		=	_loll(qg_re_im);
	        sum0		=	(int32_t) (((float) sum0) *  invsqrt + 0.5f);
			sum1		=	_hill(qg_re_im);
	        sum1		=	(int32_t) (((float) sum1) *  invsqrt + 0.5f);            
            _amem8(&Ap[jq])		=	_itoll(sum1, sum0);
			#else
	        //sum0		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ;            
	        //sum1		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
			sum0		=	_hill(qg_re_im);
	        sum0		=	(int32_t) (((float) sum0) *  invsqrt + 0.5f);
			sum1		=	_loll(qg_re_im);
	        sum1		=	(int32_t) (((float) sum1) *  invsqrt + 0.5f);            
            _amem8(&Ap[jq])		=	_itoll(sum0, sum1);
			#endif
	    }                                                               

	    for( k = j + 1; k < n; k++ )   
		{                                                               
			int64_t a2_a0;               
	    
	        a2_a0		=	_amem8(&A[2 * (n * k + j)]);    
	        for( i = j + 1; i < k + 1; i++ )     
			{                                                           
				int64_t sum1_sum0, a3_a1;
		       
				sum1_sum0	=	_amem8(&A[2 * ( n * k + i )]);
				a3_a1		=	_amem8(&A[2 * ( n * i + j )]);  
				#ifdef _LITTLE_ENDIAN
				b0			=	frac_dotp2 (_loll(a2_a0),_loll(a3_a1), _hill(a2_a0),_hill(a3_a1));   
		        b1			=	frac_dotpn2(_hill(a2_a0),_loll(a3_a1), _loll(a2_a0),_hill(a3_a1));
	            sum0		=	_ssub(_loll(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_hill(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)])	=	_itoll(sum1,sum0);
				#else
		        b0			=	frac_dotp2 (_hill(a2_a0),_hill(a3_a1), _loll(a2_a0),_loll(a3_a1));   
		        b1			=	(frac_dotpn2(_loll(a2_a0),_hill(a3_a1), _hill(a2_a0),_loll(a3_a1)));
	            sum0		=	_ssub(_hill(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_loll(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)]) =	_itoll(sum0,sum1);
				#endif
	        }                                                           
	    }                                                               
	}  
	
	//recip_sqrt32(A[2*n*n-2], 0, &sq, &exp);                          
	//sum0		=	_sshl(_smpy32_64(A[2*n*n-2], sq), exp);                         
	ftemp		=	(float) (A[2*n*n-2]);
	invsqrt		=	_rsqrsp(ftemp * 2.f);
	invsqrt		=	invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	invsqrt		=	32768.f * invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	sum0		=	(int32_t) (ftemp * invsqrt + 0.5f);                         
	#ifdef _LITTLE_ENDIAN
    _amem8(&Ap[2*n*n-2]) = _itoll(0,sum0);
	#else
    _amem8(&Ap[2*n*n-2]) = _itoll(sum0,0);
	#endif
}           

#define N4 (4)

void 
MATRIX_cholesky_4(
	int32_t * RESTRICT A, 
	int32_t * Ap,
	int32_t	x
	)                                      
{                                                                     
	int32_t i, j, k;                                                        
	//int32_t sq, exp;                                                        
	int32_t sum0, sum1;                                     
	int32_t b0, b1, n=4;                                                         
	int32_t jqg;  
	float invsqrt, ftemp;
                     
#ifdef _TMS320C6X
	_nassert(n==4);  
#endif	
	for (j = 0; j < n-1; j++)    	
	{                                                                   
	    //recip_sqrt32(A[2 * n * j + 2 * j], 0, &sq, &exp);
		ftemp		=	(float) (A[2 * n * j + 2 * j]);
		invsqrt		=	_rsqrsp(ftemp * 2.f);
		invsqrt		=	invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
		invsqrt		=	65536.f * invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	    	    
	    for (k = j; k < n; k++ )  
		{
			int64_t qg_re_im;  int32_t jq;

            jqg			=	( n * k + j ) << 1;  
			jq			=	_mvd(jqg);  /* copy data */
            
			qg_re_im	=	_amem8(&A[jqg]);

			#ifdef _LITTLE_ENDIAN
	        //sum0		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
	        //sum1		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ; 
			sum0		=	_loll(qg_re_im);
	        sum0		=	(int32_t) (((float) sum0) *  invsqrt + 0.5f);
			sum1		=	_hill(qg_re_im);
	        sum1		=	(int32_t) (((float) sum1) *  invsqrt + 0.5f);            
            _amem8(&Ap[jq])		=	_itoll(sum1, sum0);
			#else
	        //sum0		=	_sshl(_smpy32_64(_hill(qg_re_im), sq), exp) ;            
	        //sum1		=	_sshl(_smpy32_64(_loll(qg_re_im), sq), exp) ;            
			sum0		=	_hill(qg_re_im);
	        sum0		=	(int32_t) (((float) sum0) *  invsqrt + 0.5f);
			sum1		=	_loll(qg_re_im);
	        sum1		=	(int32_t) (((float) sum1) *  invsqrt + 0.5f);            
            _amem8(&Ap[jq])		=	_itoll(sum0, sum1);
			#endif
	    }                                                               

	    for( k = j + 1; k < n; k++ )   
		{                                                               
			int64_t a2_a0;               
	    
	        a2_a0		=	_amem8(&A[2 * (n * k + j)]);    
	        for( i = j + 1; i < k + 1; i++ )     
			{                                                           
				int64_t sum1_sum0, a3_a1;
		       
				sum1_sum0	=	_amem8(&A[2 * ( n * k + i )]);
				a3_a1		=	_amem8(&A[2 * ( n * i + j )]);  
				#ifdef _LITTLE_ENDIAN
				b0			=	frac_dotp2 (_loll(a2_a0),_loll(a3_a1), _hill(a2_a0),_hill(a3_a1));   
		        b1			=	frac_dotpn2(_hill(a2_a0),_loll(a3_a1), _loll(a2_a0),_hill(a3_a1));
	            sum0		=	_ssub(_loll(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_hill(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)])	=	_itoll(sum1,sum0);
				#else
		        b0			=	frac_dotp2 (_hill(a2_a0),_hill(a3_a1), _loll(a2_a0),_loll(a3_a1));   
		        b1			=	(frac_dotpn2(_loll(a2_a0),_hill(a3_a1), _hill(a2_a0),_loll(a3_a1)));
	            sum0		=	_ssub(_hill(sum1_sum0), b0);                                  
	            sum1		=	_ssub(_loll(sum1_sum0), b1);                                           
                _amem8(&Ap[2*(n*k+i)]) =	_itoll(sum0,sum1);
				#endif
	        }                                                           
	    }                                                               
	}  
	
	//recip_sqrt32(A[2*n*n-2], 0, &sq, &exp);                          
	//sum0		=	_sshl(_smpy32_64(A[2*n*n-2], sq), exp);                         
	ftemp		=	(float) (A[2*n*n-2]);
	invsqrt		=	_rsqrsp(ftemp * 2.f);
	invsqrt		=	invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	invsqrt		=	32768.f * invsqrt * (1.5f - ftemp * invsqrt * invsqrt);
	sum0		=	(int32_t) (ftemp * invsqrt + 0.5f);                         
	#ifdef _LITTLE_ENDIAN
    _amem8(&Ap[2*n*n-2]) = _itoll(0,sum0);
	#else
    _amem8(&Ap[2*n*n-2]) = _itoll(sum0,0);
	#endif
}           


void MATRIX_low_tri_solve_4x4(
	int32_t * L,  
	int32_t * Y,  
	int32_t * V,  
    int32_t n
	)
{
	int32_t i,j;
	int32_t Vr, Vi;
	int32_t LYr, LYi;
	int32_t frac,rexp;

	int64_t Vi_Vr, Li_Lr, Yi_Yr, Di_Dr;

    // j = 0
    Vi_Vr	= _amem8(&V[0]);
	#ifdef _LITTLE_ENDIAN
	Vr		=	_loll(Vi_Vr); 
	Vi		=	_hill(Vi_Vr);
	#else
	Vr		=	_hill(Vi_Vr); 
	Vi		=	_loll(Vi_Vr);
	#endif

    Vr		>>=	3;
	Vi		>>=	3;
    
    Di_Dr		=	_amem8(&L[0]);
	#ifdef _LITTLE_ENDIAN
    recip_32(_loll(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[0])		=	_itoll( _sshl(_smpy32_64(Vi,frac),rexp), _sshl(_smpy32_64(Vr,frac),rexp) );
	#else
    recip_32(_hill(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[0])		=	_itoll(  _sshl(_smpy32_64(Vr,frac),rexp),_sshl( _smpy32_64(Vi,frac),rexp) );
	#endif

    // j = 1
    Vi_Vr	= _amem8(&V[2]);
	#ifdef _LITTLE_ENDIAN
	Vr		=	_loll(Vi_Vr); 
	Vi		=	_hill(Vi_Vr);
	#else
	Vr		=	_hill(Vi_Vr); 
	Vi		=	_loll(Vi_Vr);
	#endif

    Vr		>>=	3;
	Vi		>>=	3;
    
    Li_Lr	=	_amem8(&L[8]);
    Yi_Yr   =	_amem8(&Y[0]);
	#ifdef _LITTLE_ENDIAN
	LYr		=	frac_dotpn2(_loll(Li_Lr),_loll(Yi_Yr), _hill(Li_Lr),_hill(Yi_Yr));   
    LYi		=	frac_dotp2 (_loll(Li_Lr),_hill(Yi_Yr), _hill(Li_Lr),_loll(Yi_Yr));
	#else
	LYr		=	frac_dotpn2(_hill(Li_Lr),_hill(Yi_Yr), _loll(Li_Lr),_loll(Yi_Yr));   
    LYi		=	frac_dotp2 (_hill(Li_Lr),_loll(Yi_Yr), _loll(Li_Lr),_hill(Yi_Yr));
	#endif
    Vr		=	_ssub(Vr, LYr);                                  
    Vi		=	_ssub(Vi, LYi);                                           

    Di_Dr		=	_amem8(&L[10]);
	#ifdef _LITTLE_ENDIAN
    recip_32(_loll(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[2])		=	_itoll( _sshl(_smpy32_64(Vi,frac),rexp), _sshl(_smpy32_64(Vr,frac),rexp) );
	#else
    recip_32(_hill(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[2])		=	_itoll(  _sshl(_smpy32_64(Vr,frac),rexp),_sshl( _smpy32_64(Vi,frac),rexp) );
	#endif

    // j = 2
	j		=	2;
    Vi_Vr	=	_amem8(&V[4]);
	#ifdef _LITTLE_ENDIAN
	Vr		=	_loll(Vi_Vr); 
	Vi		=	_hill(Vi_Vr);
	#else
	Vr		=	_hill(Vi_Vr); 
	Vi		=	_loll(Vi_Vr);
	#endif

    Vr		>>=	3;
	Vi		>>=	3;
    
	for ( i = 0; i < j; i++ ) 
	{
    	Li_Lr	=	_amem8(&L[2 * (n * j + i)]);
    	Yi_Yr   =	_amem8(&Y[2 * i]);
		#ifdef _LITTLE_ENDIAN
	    LYr		=	frac_dotpn2(_loll(Li_Lr),_loll(Yi_Yr), _hill(Li_Lr),_hill(Yi_Yr));   
        LYi		=	frac_dotp2 (_loll(Li_Lr),_hill(Yi_Yr), _hill(Li_Lr),_loll(Yi_Yr));
		#else
	    LYr		=	frac_dotpn2(_hill(Li_Lr),_hill(Yi_Yr), _loll(Li_Lr),_loll(Yi_Yr));   
        LYi		=	frac_dotp2 (_hill(Li_Lr),_loll(Yi_Yr), _loll(Li_Lr),_hill(Yi_Yr));
		#endif
        Vr		=	_ssub(Vr, LYr);                                  
        Vi		=	_ssub(Vi, LYi);                                           
    }

    Di_Dr		=	_amem8(&L[20]);
	#ifdef _LITTLE_ENDIAN
    recip_32(_loll(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[4])		=	_itoll( _sshl(_smpy32_64(Vi,frac),rexp), _sshl(_smpy32_64(Vr,frac),rexp) );
	#else
    recip_32(_hill(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[4])		=	_itoll(  _sshl(_smpy32_64(Vr,frac),rexp),_sshl( _smpy32_64(Vi,frac),rexp) );
	#endif

    // j = 3
	j		=	3;
    Vi_Vr	=	_amem8(&V[6]);
	#ifdef _LITTLE_ENDIAN
	Vr		=	_loll(Vi_Vr); 
	Vi		=	_hill(Vi_Vr);
	#else
	Vr		=	_hill(Vi_Vr); 
	Vi		=	_loll(Vi_Vr);
	#endif

    Vr		>>=	3;
	Vi		>>=	3;
    
	for ( i = 0; i < j; i++ ) 
	{
    	Li_Lr	=	_amem8(&L[2 * (n * j + i)]);
    	Yi_Yr   =	_amem8(&Y[2 * i]);
		#ifdef _LITTLE_ENDIAN
	    LYr		=	frac_dotpn2(_loll(Li_Lr),_loll(Yi_Yr), _hill(Li_Lr),_hill(Yi_Yr));   
        LYi		=	frac_dotp2 (_loll(Li_Lr),_hill(Yi_Yr), _hill(Li_Lr),_loll(Yi_Yr));
		#else
	    LYr		=	frac_dotpn2(_hill(Li_Lr),_hill(Yi_Yr), _loll(Li_Lr),_loll(Yi_Yr));   
        LYi		=	frac_dotp2 (_hill(Li_Lr),_loll(Yi_Yr), _loll(Li_Lr),_hill(Yi_Yr));
		#endif
        Vr		=	_ssub(Vr, LYr);                                  
        Vi		=	_ssub(Vi, LYi);                                           
    }

    Di_Dr		=	_amem8(&L[2*(n*j+j)]);
	#ifdef _LITTLE_ENDIAN
    recip_32(_loll(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[6])		=	_itoll( _sshl(_smpy32_64(Vi,frac),rexp), _sshl(_smpy32_64(Vr,frac),rexp) );
	#else
    recip_32(_hill(Di_Dr), 0, &frac,&rexp);
    _amem8(&Y[6])		=	_itoll(  _sshl(_smpy32_64(Vr,frac),rexp),_sshl( _smpy32_64(Vi,frac),rexp) );
	#endif

}

void  MATRIX_up_tri_solve_4x4(
	int32_t * L,  
	int32_t * Z,  
	int32_t * Y,  
    int32_t n
	)
{
	int32_t i,j;
	int32_t Yr, Yi;
	int32_t LZr, LZi;
	int32_t frac,rexp;


	for(j = n - 1; j >= 0;j-- ) 
	{
		int64_t Zi_Zr, Li_Lr, Yi_Yr, Di_Dr;
    
    	Yi_Yr	=	_amem8(&Y[2*j]);
		#ifdef _LITTLE_ENDIAN
		Yr		=	_loll(Yi_Yr); 
		Yi		=	_hill(Yi_Yr);
		#else
		Yr		=	_hill(Yi_Yr); 
		Yi		=	_loll(Yi_Yr);
		#endif

    	for( i = n - 1; i > j; i-- ) 
		{
    		Li_Lr   =	_amem8(&L[2 * (n * i + j)]);
    		Zi_Zr	=	_amem8(&Z[2 * i]);
			#ifdef _LITTLE_ENDIAN
            LZr		=	frac_dotp2  (_loll(Li_Lr),_loll(Zi_Zr), _hill(Li_Lr),_hill(Zi_Zr));
	        LZi		=	frac_dotpn2 (_loll(Li_Lr),_hill(Zi_Zr), _hill(Li_Lr),_loll(Zi_Zr));   
			#else
            LZr		=	frac_dotp2  (_hill(Li_Lr),_hill(Zi_Zr), _loll(Li_Lr),_loll(Zi_Zr));
	        LZi		=	frac_dotpn2 (_hill(Li_Lr),_loll(Zi_Zr), _loll(Li_Lr),_hill(Zi_Zr));   
			#endif
            Yr		=	_ssub(Yr, LZr);                                  
            Yi		=	_ssub(Yi, LZi);                                           
    	}
    	Di_Dr		=	_amem8(&L[2*(n*j+j)]);
		#ifdef _LITTLE_ENDIAN
        recip_32(_loll(Di_Dr), 0, &frac,&rexp);
    	_amem8(&Z[2*j])		=	_itoll( _sshl(_smpy32_64(Yi,frac),rexp), _sshl(_smpy32_64(Yr,frac),rexp) );
		#else
        recip_32(_hill(Di_Dr), 0, &frac,&rexp);
    	_amem8(&Z[2*j])		=	_itoll( _sshl(_smpy32_64(Yr,frac),rexp), _sshl(_smpy32_64(Yi,frac),rexp)  );
		#endif
	}		
}
