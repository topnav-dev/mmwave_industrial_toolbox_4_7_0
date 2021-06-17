/**
 *   @file  MATRIX_EVDJacobian.c
 *
 *   @brief
 *      EVD using Jacobi method
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

#include <modules/postProcessing/matrixFunc/api/MATRIX_EVDJacobian.h>

#ifndef _TMS320C6600
#include "radar_c674x.h"
#endif
#define first_NR_iteration 1
#define scd_NR_iteration 0 
//#define Re_arranged_Equation

/**
 *  \fn     void MATRIX_EVDFloat(
 *			IN       int32_t matrixSize,
 * 			IN       float tolerance,
 * 			INOUT    cplxf_t* RESTRICT eigenValPtr,
 * 			INOUT    cplxf_t* RESTRICT eigenVectMatPtr,
 * 			INOUT    int32_t * iteration)  
 * 
 *  \brief   This is inplace implementation of Modified Jacobian Method for Eigen Value Decomposition of Hermitian symmetric and positive definite matrix in floating-point. 
 * 			 It also works for non-positive definite as long as it is Hermitian symmetric. For Hermitian symmetric matrix A, this function decomposes A into V*D*V' by doing 
 * 			 sequence of Givens rotations to minimize off-diagonal norm of the matrix:  where V is the eigenvector matrix and D is the diagonal matrix of Eigen values. Input 
 * 			 parameter "Tolerance" is used as the limit for the off-diagonal norm of the matrix calculated after every Givens rotations. Choosing the Tolerance decides the 
 * 			 precision of the Eigen values and therefore the performance of the loop (i.e. number of iterations). User has to do performance analysis with different tolerance
 * 			 values to get the required precision using the matlab project at "../unittest/matlab".
 *
 *  \param[in]    matrixSize
 * 				  Matrix size, for 8x8 Matrix, matrixSize variable equals to 8. This implementation is optimized for upto 8x8 matrix.
 
 *  \param[in]    Tolerance
 * 				  Input parameter "Tolerance" is used as the stopping criteria for the sequence of Givens rotations in the Jacobian methods as it is used as limit for the 
 * 				  off-diagonal norm calculated after every rotation. Tolerance value decides the precision needed for the generated Eigen values and therefore the performance 
 * 				  of the loop (i.e. number of Givens rotations). 
 
 *  \param[in,out] eigenValPtr
 * 				   This is input as well as output parameter. Input is the Complex Hermitian matrix of dimension size X size. Output is all diagonal matrix of eignevalues. 
 * 				   All elements of the Matrix A_Mat are stored in 1-d array row-wise. Storage requirement for this array is = 2*4*size*size bytes. 
 *                 The 1-d arrays must be aligned on the 8 byte boundary using #pragma DATA_ALIGN. 
 *                 Below formula will help visualize how A_Mat(row, column) complex Hermitian single precision floating-point matrix of size N is stored in memory.
 *                 For example i represents row and j represents column and the 1-d array float variable is A_mat[2*N*N]. So row 1 of A_Mat is stored is A_mat[0 : 2*N-1],
 *                 2nd row of A_Mat is stored in A_mat[2*N:4*N-1] and Mth row of A_Mat is stored in A_mat[2*N*M : 2*N*M + 2*N - 1].
 * 
 * 
 *  \param[in,out] eigenVectPtr
 * 				   The pointer to Eigen vector matrix stored as 1-D array row-wise. This array pointer needs to be aligned on the 8 byte boundary. It is recommended to 
 *                 warm-up the cache with these values to avoid any cache penalties. Note that 1-d array pointed by eigenVectPtr needs to be pre-initialized with identity
 *                 matrix (I) of same size. This can be done by calling library function MATRIX_EVDFloat_init() function.
 *  
 *  \param[in,out] iteration
 * 				    Input: Maximum iteration allowed for a given performance and precision.
 * 				    Output: Total number of iterations executed by the Jacobian method to get the desired precision for the off-diagonal norm specified by Tolerance.
 *
 *  \pre      eigenVectMatPtr array matrixr needs to be pre-initialized with identity matrix of size equal to matrixSize. This can be done by calling MATRIX_EVD_init() function.
 *            eigValPtr and eigenVectMatPtr array pointers must be aligned on a double word (8-byte) boundary using #pragma DATA_ALIGN. 
 *
 *  \post
 *
 *  \sa
 *
 */
 
 
						 
void MATRIX_EVDFloat(		
		int32_t matrixSize, 
		float tolerance,
		cplxf_t* RESTRICT eigenValPtr,		
		cplxf_t* RESTRICT eigenVectMatPtr,
		int32_t * iteration) 
   {
		int32_t nc, nr, idx, nr_, nc_;
		
		float offDiagNormA, offDiagNormAsqr, oneOverSqrtOffDiagNormA, frobNormA, frobNormAsqr, oneOverSqrtfrobNormA, eps, zeroThrsld;
		float zeroValues = 0, numElems; //Indicate number of zeros inside the matrix to avoid rotating values close to zeros.
		__float2_t frobNormAsqr_0;		
		float * RESTRICT eigValPtr;
		float * RESTRICT eigVectMatPtr_in, * RESTRICT eigVectMatPtr_out, * RESTRICT eigValPtr_in,  * RESTRICT eigValPtr_out;
		
		#ifndef Re_arranged_Equation
		float mu1, mu2;
		#endif
		
		/*Local variables to while loop defined globally for verification*/
		float delta, delta_temp, deltaSquare, mu1_2, oneOverSqrtAbsVal, absValSqr;
		float mu1_2_temp, mu2_2, mu2_2_temp, oneOver_mu1_2_temp, oneOver_mu2_2_temp, oneOverSqrtDeltaSquare, sqrtDeltaSquare;
		__float2_t x1, x2, y1, y2;
		float x1f, x2f, y1f, y2f;
		__float2_t tempSqr;
		__float2_t  offDiagNormAsqr_0;
		
		
		#ifdef Re_arranged_Equation
		float oneOver_mu1_2_1, oneOver_mu2_2_1;
		#endif		
		float mu1_2_1, mu2_2_1;
		__float2_t absValSqr_temp; //for verification only
		__float2_t ejtheata, minus_ejtheata;
		__float2_t value_nr__nr, value_nr__nc, y1_value_nr__nr, y2_value_nr__nr, value_nc_nc_;		
		__float2_t vector_nr__nr, vector_nr__nc, y1_vector_nr__nr, y2_vector_nr__nr, temp___float2_t;
		__x128_t y1_vector_nr__nr_128, y2_vector_nr__nr_128, y1_value_nr__nr_128, y2_value_nr__nr_128;		
		
		__float2_t value_nr_nc_, y1_value_nr_nc_, y2_value_nr_nc_;
		__x128_t y1_value_nr_nc__128, y2_value_nr_nc__128;
		int maxIteration = 1, jacobianIteration = 0;
		if ((iteration[0] >1) && (iteration[0] <100))
		maxIteration = iteration[0];
		
		frobNormAsqr_0 = _ftof2(0,0);
        tempSqr = _ftof2(0,0);
        offDiagNormAsqr_0 = _ftof2(0,0);
		
		eigValPtr = (float * ) eigenValPtr;
		
		eigVectMatPtr_in = (float * ) eigenVectMatPtr;
		eigVectMatPtr_out = (float * ) eigenVectMatPtr;
		
		eigValPtr_in = (float * ) eigenValPtr;
		eigValPtr_out = (float * ) eigenValPtr;
		
		offDiagNormAsqr = 0;		
		
		/* Calculate the off-diagonal Norm as well as Frobenius Norm used for stopping criteria */
		/*Outer loop can be unrolled by 2. But it seems it will not give improvment sine the matrix size is small */
		for(nr = 0; nr < matrixSize; nr ++) {						 
			for(nc = nr + 1; nc < matrixSize; nc++) { /* Matrix is Hermitian symmetric */
				temp___float2_t = _amem8_f2(&eigValPtr[2*(nr*matrixSize + nc)]); /* Data is stored row-wise in the memory */
				tempSqr = _dmpysp(temp___float2_t, temp___float2_t); 
				offDiagNormAsqr_0 = _daddsp(tempSqr, offDiagNormAsqr_0);		
			}
		}
		offDiagNormAsqr += _hif2(offDiagNormAsqr_0) + _lof2(offDiagNormAsqr_0);
	    
		offDiagNormAsqr = 2.0 * offDiagNormAsqr; /*Off diagonal Norm */
		frobNormAsqr = offDiagNormAsqr;

		
		oneOverSqrtOffDiagNormA = _rsqrsp(offDiagNormAsqr); /* 1/sqrt() instruction 8-bit mantissa precision*/
		/* One NR interpolation for 16-bit precision*/
	    oneOverSqrtOffDiagNormA = oneOverSqrtOffDiagNormA * (1.5f - (offDiagNormAsqr*0.5f)
		                          * oneOverSqrtOffDiagNormA *oneOverSqrtOffDiagNormA); 
		#if scd_NR_iteration
		/* 2nd NR interpolation for 24-bit precision*/
		oneOverSqrtOffDiagNormA = oneOverSqrtOffDiagNormA * (1.5f - (offDiagNormAsqr*0.5f)
		                          * oneOverSqrtOffDiagNormA *oneOverSqrtOffDiagNormA); 
		#endif
								  
		offDiagNormA = offDiagNormAsqr * oneOverSqrtOffDiagNormA; /*sqrt() approximation */
	    
	    
	    
		for(nr = 0; nr < matrixSize; nr++) {
			tempSqr = _dmpysp(_amem8_f2(&eigValPtr[2*(nr*matrixSize + nr)]), _amem8_f2(&eigValPtr[2*(nr*matrixSize + nr)]));
			frobNormAsqr_0 = _daddsp(frobNormAsqr_0, tempSqr);
		}		
		frobNormAsqr += _hif2(frobNormAsqr_0) + _lof2(frobNormAsqr_0);
		
		oneOverSqrtfrobNormA = _rsqrsp(frobNormAsqr); /* 1/sqrt() instruction 8-bit mantissa precision*/
		/* One NR interpolation for 16-bit precision*/
	    oneOverSqrtfrobNormA = oneOverSqrtfrobNormA * (1.5f - (frobNormAsqr*0.5f)
		                          * oneOverSqrtfrobNormA *oneOverSqrtfrobNormA); 
		#if scd_NR_iteration
		/* 2nd NR interpolation for 24-bit precision*/
		oneOverSqrtfrobNormA = oneOverSqrtfrobNormA * (1.5f - (frobNormAsqr*0.5f)
		                          * oneOverSqrtfrobNormA *oneOverSqrtfrobNormA); 
		#endif						  
		frobNormA = frobNormAsqr * oneOverSqrtfrobNormA; /*sqrt() approximation */
		
		zeroThrsld = frobNormA * 1e-3;
		
		eps = frobNormA * tolerance;
		numElems = (matrixSize*matrixSize>>1)-matrixSize;
		while((offDiagNormA > eps) && (zeroValues <= numElems)&& (jacobianIteration <= maxIteration)) 
		{
			jacobianIteration++;
			zeroValues = 0;
			for(nr = 0; nr < matrixSize; nr++) {
				for(nc = nr+1; nc < matrixSize; nc++){
					/* Do closed form 2x2 EVD and calculate eignevalues and
					* eigenvectors that diagonalize a 2x2 matrix and 
					* generate Jacobian rotations matrix i.e.
					* J(row,col,theta) of same size as original matrix
					* For example
					*             a b
					* A[2x2] =    c d      */
				
					idx = 2*(nc*matrixSize + nr);
					absValSqr_temp = _dmpysp(_amem8_f2(&eigValPtr[idx]), _amem8_f2(&eigValPtr[idx]));
				    
				    absValSqr = _hif2(absValSqr_temp) + _lof2(absValSqr_temp);
				    if(absValSqr > zeroThrsld)
						{
						oneOverSqrtAbsVal = _rsqrsp(absValSqr);
						
						/* One NR interpolation for 16-bit precision*/
					    #if first_NR_iteration
						oneOverSqrtAbsVal = oneOverSqrtAbsVal * (1.5f - (absValSqr*0.5f)
			                          * oneOverSqrtAbsVal * oneOverSqrtAbsVal);
                        /* * 2nd NR interpolation for 24-bit precision*/
						#if scd_NR_iteration
						oneOverSqrtAbsVal = oneOverSqrtAbsVal * (1.5f - (absValSqr*0.5f)
			                          * oneOverSqrtAbsVal * oneOverSqrtAbsVal); 
					    #endif
					    #endif
                        ejtheata = _amem8_f2(&eigValPtr[2*(nr*matrixSize + nc)]);
                                                
						ejtheata = _dmpysp(ejtheata, _ftof2(oneOverSqrtAbsVal,oneOverSqrtAbsVal) );
						minus_ejtheata = _lltof2(_itoll(0x80000000,0x80000000) ^ _f2toll(ejtheata));
					
					    #ifndef _LITTLE_ENDIAN
						delta_temp = eigValPtr[2*(nr*matrixSize + nr)] - eigValPtr[2*(nc*matrixSize + nc)];//----------
						#else
						delta_temp = eigValPtr[2*(nr*matrixSize + nr)+1] - eigValPtr[2*(nc*matrixSize + nc)+1];//----------
						#endif
						delta = delta_temp * oneOverSqrtAbsVal;
						deltaSquare = delta*delta + 4; 
						oneOverSqrtDeltaSquare = _rsqrsp(deltaSquare);
						#if first_NR_iteration
						/* One NR interpolation for 16-bit precision*/
						oneOverSqrtDeltaSquare = oneOverSqrtDeltaSquare * (1.5f - (deltaSquare*0.5f)
			                          * oneOverSqrtDeltaSquare * oneOverSqrtDeltaSquare);
                        #if scd_NR_iteration
						/* 2nd NR interpolation for 24-bit precision*/
						oneOverSqrtDeltaSquare = oneOverSqrtDeltaSquare * (1.5f - (deltaSquare*0.5f)
			                          * oneOverSqrtDeltaSquare * oneOverSqrtDeltaSquare); 
						#endif
						#endif
									  
						sqrtDeltaSquare = oneOverSqrtDeltaSquare * deltaSquare;
					
						mu1_2_temp = sqrtDeltaSquare - delta;	
						mu2_2_temp = sqrtDeltaSquare + delta;
						
						oneOver_mu1_2_temp = _rcpsp(mu1_2_temp);
						#if first_NR_iteration
						oneOver_mu1_2_temp = oneOver_mu1_2_temp * (2.f - mu1_2_temp * oneOver_mu1_2_temp);
						#if scd_NR_iteration
						oneOver_mu1_2_temp = oneOver_mu1_2_temp * (2.f - mu1_2_temp * oneOver_mu1_2_temp);
						#endif
						#endif
						
						#ifndef Re_arranged_Equation
						mu1 = 2*oneOver_mu1_2_temp;
						#endif
						
						mu1_2 = 4*oneOver_mu1_2_temp*oneOver_mu1_2_temp;
						mu1_2_1 = mu1_2 + 1;
					
						oneOver_mu2_2_temp = _rcpsp(mu2_2_temp);
						#if first_NR_iteration
						oneOver_mu2_2_temp = oneOver_mu2_2_temp * (2.f - mu2_2_temp * oneOver_mu2_2_temp);
						#if scd_NR_iteration
						oneOver_mu2_2_temp = oneOver_mu2_2_temp * (2.f - mu2_2_temp * oneOver_mu2_2_temp);
						#endif
						#endif
						
						#ifndef Re_arranged_Equation
						mu2 = 2*oneOver_mu2_2_temp;
						#endif
						
						mu2_2 = 4*oneOver_mu2_2_temp*oneOver_mu2_2_temp;
						mu2_2_1 = mu2_2 + 1;
						
						#ifdef Re_arranged_Equation //***********************************
						oneOver_mu1_2_temp = _rcpsp(mu1_2);
						#if first_NR_iteration
						oneOver_mu1_2_temp = oneOver_mu1_2_temp * (2.f - mu1_2 * oneOver_mu1_2_temp);
						#if scd_NR_iteration
						oneOver_mu1_2_temp = oneOver_mu1_2_temp * (2.f - mu1_2 * oneOver_mu1_2_temp);
						#endif
						#endif
						
						oneOver_mu1_2_1 = oneOver_mu1_2_temp + 1;
						
						oneOver_mu2_2_temp = _rcpsp(mu2_2);
						#if first_NR_iteration
						oneOver_mu2_2_temp = oneOver_mu2_2_temp * (2.f - mu2_2 * oneOver_mu2_2_temp);
						#if scd_NR_iteration
						oneOver_mu2_2_temp = oneOver_mu2_2_temp * (2.f - mu2_2 * oneOver_mu2_2_temp);
						#endif
						#endif
						oneOver_mu2_2_1 = oneOver_mu2_2_temp + 1;
						
						#endif //************************
						
						x1f = _rsqrsp(mu1_2_1);
						#if first_NR_iteration
						/* One NR interpolation for 16-bit precision*/
						x1f = x1f * (1.5f - (mu1_2_1*0.5f)* x1f *x1f);
						#if scd_NR_iteration 
						/* 2nd NR interpolation for 24-bit precision*/ 
						x1f = x1f * (1.5f - (mu1_2_1*0.5f)* x1f *x1f);
						#endif
						#endif
						 
						x1 = _ftof2(x1f,x1f);
						
						x2f = _rsqrsp(mu2_2_1);
						#if first_NR_iteration
						/* One NR interpolation for 16-bit precision*/
						x2f = x2f * (1.5f - (mu2_2_1*0.5f)* x2f *x2f);
						#if scd_NR_iteration  
						/* 2nd NR interpolation for 24-bit precision*/
						x2f = x2f * (1.5f - (mu2_2_1*0.5f)* x2f *x2f);
						#endif 
						#endif
						x2 = _ftof2(x2f,x2f);
						
						#ifdef Re_arranged_Equation
						y1f = _rsqrsp(oneOver_mu1_2_1);
						#if first_NR_iteration
						/* One NR interpolation for 16-bit precision */
						y1f = y1f * (1.5f - (oneOver_mu1_2_1*0.5f)* y1f * y1f);
						#if scd_NR_iteration 
						/* 2nd NR interpolation for 24-bit precision */ 
						y1f = y1f * (1.5f - (oneOver_mu1_2_1*0.5f)* y1f * y1f);
						#endif
						#endif
						
						y1 = _dmpysp(ejtheata, _ftof2(y1f,y1f));						
						y2f = _rsqrsp(oneOver_mu2_2_1);
						#if first_NR_iteration
						/*One NR interpolation for 16-bit precision */
						y2f = y2f * (1.5f - (oneOver_mu2_2_1*0.5f)* y2f * y2f);
						#if scd_NR_iteration 
						/* 2nd NR interpolation for 24-bit precision */ 
						y2f = y2f * (1.5f - (oneOver_mu2_2_1*0.5f)* y2f * y2f);
						#endif
						#endif
						
						y2 = _dmpysp(minus_ejtheata, _ftof2(y2f,y2f));
						#else //If not defined Re_arranged_Equation
						y1f = mu1*x1f;
						y1 = _dmpysp(ejtheata, _ftof2(y1f,y1f));
						
						y2f = mu2*x2f;
						y2 = _dmpysp(minus_ejtheata, _ftof2(y2f,y2f));
						
						#endif 
							
						for(nr_ = 0; nr_ < matrixSize; nr_++)
						{
							/*Update Eigen Vector V: V = V * J_pqt */
							vector_nr__nr = 	_amem8_f2(&eigVectMatPtr_in[2*(nr_*matrixSize + nr)]);
							vector_nr__nc = 	_amem8_f2(&eigVectMatPtr_in[2*(nr_*matrixSize + nc)]);
							
							y1_vector_nr__nr_128 = _cmpysp(y1, vector_nr__nr);
							//y1_vector_nr__nr = _daddsp(_hif2_128(y1_vector_nr__nr_128) , _lof2_128(y1_vector_nr__nr_128));
							y1_vector_nr__nr = _dsubsp(_hif2_128(y1_vector_nr__nr_128) , _lof2_128(y1_vector_nr__nr_128));
							
							y2_vector_nr__nr_128 = _cmpysp(y2, vector_nr__nr);
							//y2_vector_nr__nr = _daddsp(_hif2_128(y2_vector_nr__nr_128) , _lof2_128(y2_vector_nr__nr_128));
							y2_vector_nr__nr = _dsubsp(_hif2_128(y2_vector_nr__nr_128) , _lof2_128(y2_vector_nr__nr_128));
							
							_amem8_f2(&eigVectMatPtr_out[2*(nr_*matrixSize + nr)]) = _daddsp(y1_vector_nr__nr, _dmpysp(x1, vector_nr__nc)); 
							_amem8_f2(&eigVectMatPtr_out[2*(nr_*matrixSize + nc)]) = _daddsp(y2_vector_nr__nr, _dmpysp(x2, vector_nr__nc));
							
						}
						
						for(nr_ = 0; nr_ < matrixSize; nr_++)
						{
							/*Update eigValPtr (i.e. Input Matrix): A_Mat = J_pqt'*A_Mat*J_pqt
							 * Step 1 B_Mat = A_Mat*J_pqt            */
							value_nr__nr = 	_amem8_f2(&eigValPtr_in[2*(nr_*matrixSize + nr)]);
							value_nr__nc = 	_amem8_f2(&eigValPtr_in[2*(nr_*matrixSize + nc)]);
							
							y1_value_nr__nr_128 = _cmpysp(y1, value_nr__nr);
							y1_value_nr__nr = _daddsp(_hif2_128(y1_value_nr__nr_128) , _lof2_128(y1_value_nr__nr_128));
							
							y2_value_nr__nr_128 = _cmpysp(y2, value_nr__nr);
							y2_value_nr__nr = _daddsp(_hif2_128(y2_value_nr__nr_128) , _lof2_128(y2_value_nr__nr_128));
							
							_amem8_f2(&eigValPtr_out[2*(nr_*matrixSize + nr)]) = _daddsp(y1_value_nr__nr, _dmpysp(x1, value_nr__nc)); 
							_amem8_f2(&eigValPtr_out[2*(nr_*matrixSize + nc)]) = _daddsp(y2_value_nr__nr, _dmpysp(x2, value_nr__nc));
						}
						
						/* Step 2 A_Mat = J_pqt'*B_Mat*/ 				 			
						
						for(nc_ = 0; nc_ < matrixSize; nc_++)
						{
							value_nr_nc_ = 	_amem8_f2(&eigValPtr_in[2*(nr*matrixSize + nc_)]);
							value_nc_nc_ = 	_amem8_f2(&eigValPtr_in[2*(nc*matrixSize + nc_)]);
							
							y1_value_nr_nc__128 = _cmpysp(y1, value_nr_nc_);
							y1_value_nr_nc_ = _dsubsp(_hif2_128(y1_value_nr_nc__128) , _lof2_128(y1_value_nr_nc__128));
							
							y2_value_nr_nc__128 = _cmpysp(y2, value_nr_nc_);
							y2_value_nr_nc_ = _dsubsp(_hif2_128(y2_value_nr_nc__128) , _lof2_128(y2_value_nr_nc__128));
							
							_amem8_f2(&eigValPtr_out[2*(nr*matrixSize + nc_)]) = _daddsp(y1_value_nr_nc_, _dmpysp(x1, value_nc_nc_)); 
							_amem8_f2(&eigValPtr_out[2*(nc*matrixSize + nc_)]) = _daddsp(y2_value_nr_nc_, _dmpysp(x2, value_nc_nc_));
							
						}
					}//End of if(absValSqr > zeroThrsld)
					else
					zeroValues++;						
							
				}/* End of nc = 0:matrixSize-1 loop */
			
			} /* End of nr = 0:matrixSize-1 loop */
			
			/* Re-calculate off diagonal Norm for stopping criteria */
			offDiagNormAsqr = 0;
			/* Calculate the off-diagonal Norm as well as Frobenius Norm used for stopping criteria */
			for(nr = 0; nr < matrixSize; nr++) {				 
				for(nc = nr + 1; nc < matrixSize; nc++) { /* Matrix is Hermitian symmetric */				
					tempSqr = _dmpysp(_amem8_f2(&eigValPtr[2*(nr*matrixSize + nc)]), _amem8_f2(&eigValPtr[2*(nr*matrixSize + nc)])); 
					offDiagNormAsqr += _hif2(tempSqr) + _lof2(tempSqr);					
				}			
			}
			
			offDiagNormAsqr = 2.0 * offDiagNormAsqr; /*Off diagonal Norm */
			oneOverSqrtOffDiagNormA = _rsqrsp(offDiagNormAsqr); /* 1/sqrt() instruction 8-bit mantissa precision*/
			/* One NR interpolation for 16-bit precision*/
			oneOverSqrtOffDiagNormA = oneOverSqrtOffDiagNormA * (1.5f - (offDiagNormAsqr*0.5f)
									  * oneOverSqrtOffDiagNormA *oneOverSqrtOffDiagNormA); 
			#if scd_NR_iteration
			/* 2nd NR interpolation for 24-bit precision*/
			oneOverSqrtOffDiagNormA = oneOverSqrtOffDiagNormA * (1.5f - (offDiagNormAsqr*0.5f)
									  * oneOverSqrtOffDiagNormA *oneOverSqrtOffDiagNormA); 
			#endif						  
			offDiagNormA = offDiagNormAsqr * oneOverSqrtOffDiagNormA; /*sqrt() approximation */
			
			
		}  /* End of While loop */
		iteration[0] = jacobianIteration;
		
   }
  
   
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 

				  
				  

