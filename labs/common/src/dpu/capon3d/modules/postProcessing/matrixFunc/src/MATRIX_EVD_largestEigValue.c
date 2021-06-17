/**
 *   @file  MATRIX_EVD_largestEigValue.c
 *
 *   @brief
 *      Manton's method to find largest eigenvalue and its correspongding eigen vector
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
#include <modules/postProcessing/matrixFunc/api/MATRIX_EVD_largestEigValue.h>

#define first_NR_iteration 1
#define scd_NR_iteration 0
//#define matrixSize_8 1

/**
 *  \fn     void MATRIX_EVD_largestEigValueFloat(
 * 			 IN     cplxf_t * RESTRICT A_Mat,
 * 			 IN		float * RESTRICT sctrachBufPtr,
 * 			 IN     int32_t   size,
 * 			 IN		float Tolerance, 
 * 			 OUT	float *   eigenValPtr,
 * 			 INOUT	cplxf_t*  RESTRICT eigenVectPtr,
 * 			 INOUT	int16_t * iteration)		 
 * 
 *  \brief   Reduce complexity Eigen Value Decomposition, to get extreme eigen value and corresponding eigen vector, based on Jonathon Manton's iterative 
 *           method based on as explained in http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=00955263. Input parameter "Tolerance" is used as the stoping 
 *           criteria of this iterative method. Choosing the Tolrence decides the precision of the eigen values and thereofore the performance of the loop
 *           (i.e. number of Manton's iteration). User has to do performance analysis with different tolerance values to get the required precision.
 *           Input, scratch buffer and output arrays must be aligned on a double word (8-byte) boundary using #pragma DATA_ALIGN. 
 *
 *  \param[in]    A_Mat
 *              Input Complex Hermitian Matrix of dimension matSize X matSize. All elements of the Matrix A_Mat are stored in 1-d array row-wise format. 
 *  			Storage requirement for this array is =  2*4*matSize*matSize bytes. To avoid cache penalties it is recommended to warm-up L1D cache with 
 * 				A_Mat using touch() loop. A_Mat must be aligned on the 8 byte boundary using #pragma DATA_ALIGN.
 *  			Below formula will help visualize how A_Mat(row, column) complex Hermitian single precision floating-point matrix of size N is stored in memory.
 * 				For example i represents row and j represents column and the 1-d array float variable is A_mat[2*N*N]. So row 1 of A_Mat is stored is A_mat[0 : 2*N-1],
 * 				2nd row of A_Mat is stored in A_mat[2*N:4*N-1] and Mth row of A_Mat is stored in A_mat[2*N*M : 2*N*M + 2*N - 1].
 *
 *  \param[in]    sctrachBufPtr
 *              The scratchpad memory needed for intermediate data storage. This array must be aligned on the 8-byte boundary using #pragma DATA_ALIGN. 
 * 				Storage requiement for this array is = (4*matSize*matSize + 6*matSize)*4 bytes, where matSize is the size of the matrix.
 *
 *  \param[in]    size
 *              It contains Matrix size. For 8x8 Matrix, size variable equals to 8.
 *
 *  \param[in]    Tolerance
 *              Tolerance specify the minimum threshold needed for the residual error calculated in the algorithm and used as stopping criteria for the Manton's iterations
 * 				to approximate extreme Eigen vector. Based on the performance analysis in MATLAB to get precision accurate to at least 2 decimal digit tolerance needs to be 
 *  		    0.01. MATLAB code is also checked-in to the library for the user to play with the desired precision and decide tolerance as well as maximum number of iterations.
 *
 *  \param[out]    eigenValPtr
 *              Pointer to the extreme eigen value variable.
 *
 *  \param[in,out] eigenVectPtr
 *              The pointer to the array where Eigen vector, of size equals to  1 X matSize, for the extreme Eigen value is stored. eigenVectorPtr complex array needs
 * 				to be pre-initialized with all zeros except the real part of the first element. Real part of the first element needs to be initialized with 1.0. 
 * 				This is the initial estimate of the eigenvector of the corresponding extreme Eigen value of the matrix and Manton's iteration method iterated to converge
 * 				to the actual complex eigenvectors. It is recommended to warm-up the cache to take advantage of the writeback cache. This array needs to be aligned on the
 * 				8 byte boundary.
 *   
 *  \param[in,out] iteration
 *              Input: Maximum iteration allowed for a given performance and precision.
 * 				Output: Total number of iterations executed by the Manton's algorithm for a given precision (based on tolerance parameter).
 *
 *  \pre      eigenVectorPtr complex array needs to be pre-initialized with all zeros except the real part of the first element. Real part of the first element needs 
 *            to be initialized with 1.0. This is the initial estimate of the eignevector of the corresponding extreme eigenvalue of the matrix and Manton's iteration method  
 *            iterated to converge to the actual complex eigenvectors. 
 *
 *  \post
 *
 *  \sa
 *
 */
						 
void MATRIX_EVD_largestEigValueFloat(
        cplxf_t * RESTRICT A_Mat,
		float * RESTRICT sctrachBufPtr, 
		int32_t   size,
		float Tolerance,		
		float *   eigenValPtr,
		cplxf_t*  RESTRICT eigenVectPtr,
		int16_t * iteration)		 
     {
		int32_t i, j;
		
		float * RESTRICT zVecPtr;
		float * RESTRICT eVecPtr;
		float * RESTRICT A_Mat_Orig_Ptr;
  		float * RESTRICT A_Mat_Ptr, * RESTRICT A_Mat_xPtr;		 		
  		float * RESTRICT A_bar_MatPtr, * RESTRICT A_bar_Mat_zPtr;  		
		__float2_t tempValue0, tempValue1;
		__x128_t temp128_03;
	    
	    __float2_t tempValue0_1, tempValue1_1, tempValue0_2, tempValue1_2, tempValue0_3, tempValue1_3;
	     __x128_t temp128_03_1, temp128_03_2, temp128_03_3;
	     __x128_t temp128_03_4, temp128_03_5, temp128_03_6, temp128_03_7; 
		 __float2_t tempSum_0, tempSum_1, tempSum_2, tempSum_3;
		 __float2_t tempSum_03_0, temp_03_1, tempSum_03_1, temp_03_2, tempSum_03_2, temp_03_3, tempSum_03_3;
		 __float2_t temp_03_4, tempSum_03_4, temp_03_5, tempSum_03_5, temp_03_6, tempSum_03_6, temp_03_7, tempSum_03_7; 		 
	    
		
		__float2_t tempSum;
		float sum = 0;
		__float2_t temp_03, tempSum_03;
	    __float2_t gamma_double;		
				
		float lambda = 0;		
		float alpha, alphaSq2, alphaCu4;
		float beta, betaSq;
		float gamma;
		float betaSqAlphaCu4,rSqrtBetaSqAlphaCu4, SqrtBetaSqAlphaCu4;
		float numerator, denominator; 
		float rSqrttempSum;
		__float2_t rSqrttempSum_double;
		int16_t  count = 0;
		float maxIteration = 1; //Default is 2 iteration
		if(iteration[0] >1 && iteration[0] < 10)
		   maxIteration = iteration[0];
		A_Mat_Orig_Ptr =   (float *) A_Mat;
		A_Mat_Ptr      = ( sctrachBufPtr);  
		A_bar_Mat_zPtr = ( sctrachBufPtr) + 2*size*size;
		A_bar_MatPtr   = ( sctrachBufPtr) + 2*size*size + 2*size; //(float *) A_bar_Mat;
		A_Mat_xPtr     = ( sctrachBufPtr) + 4*size*size + 2*size; //(float *) A_Mat_x;
	    zVecPtr        = ( sctrachBufPtr) + 4*size*size + 4*size; //(float *) zVec;	    
	    eVecPtr        =  (float *) eigenVectPtr;
	    
	    
		/**********************************************/
		/* Calculate -A                               */
	    /**********************************************/	    	
	    for(i = 0; i < size; i++){
	    	for(j = 0; j< size; j++){
				tempValue1 = _amem8_f2(&A_Mat_Orig_Ptr[2*i * size + 2*j]);
				_amem8_f2(&A_Mat_Ptr[2*i * size + 2*j]) = _lltof2((_f2toll(tempValue1) ^ _itoll(0x80000000,0x80000000)));				 
			}
		}
		
        do {
        	count++;
			/**********************************************/					
			/*Calculate A_Mat_x i.e. partial lambda */
			/**********************************************/
			//Outer loop manually unroll by 8
		    for(i = 0; i < size/8; i++) {
				tempSum_03   = _ftof2(0,0);
				tempSum_03_1 =  _ftof2(0,0);
				tempSum_03_2 =  _ftof2(0,0);
				tempSum_03_3 =  _ftof2(0,0);												
				tempSum_03_4 =  _ftof2(0,0);
				tempSum_03_5 =  _ftof2(0,0);
				tempSum_03_6 =  _ftof2(0,0);
				tempSum_03_7 =  _ftof2(0,0);
				for(j = 0; j < size; j++){					
					temp128_03 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03    = _daddsp(_hif2_128(temp128_03), _lof2_128(temp128_03));
					tempSum_03 = _daddsp(tempSum_03, temp_03);
					
					temp128_03_1 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i+1) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_1    = _daddsp(_hif2_128(temp128_03_1), _lof2_128(temp128_03_1));
					tempSum_03_1 = _daddsp(tempSum_03_1, temp_03_1);
					
					temp128_03_2 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i+2) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_2    = _daddsp(_hif2_128(temp128_03_2), _lof2_128(temp128_03_2));
					tempSum_03_2 = _daddsp(tempSum_03_2, temp_03_2);
					
					temp128_03_3 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i+3) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_3    = _daddsp(_hif2_128(temp128_03_3), _lof2_128(temp128_03_3));
					tempSum_03_3 = _daddsp(tempSum_03_3, temp_03_3);
					
					temp128_03_4 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i+4) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_4    = _daddsp(_hif2_128(temp128_03_4), _lof2_128(temp128_03_4));
					tempSum_03_4 = _daddsp(tempSum_03_4, temp_03_4);
					
					temp128_03_5 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i+5) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_5    = _daddsp(_hif2_128(temp128_03_5), _lof2_128(temp128_03_5));
					tempSum_03_5 = _daddsp(tempSum_03_5, temp_03_5);
					
					temp128_03_6 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i+6) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_6    = _daddsp(_hif2_128(temp128_03_6), _lof2_128(temp128_03_6));
					tempSum_03_6 = _daddsp(tempSum_03_6, temp_03_6);
					
					temp128_03_7 = _cmpysp(_amem8_f2(&A_Mat_Ptr[2*(8*i+7) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_7    = _daddsp(_hif2_128(temp128_03_7), _lof2_128(temp128_03_7));
					tempSum_03_7 = _daddsp(tempSum_03_7, temp_03_7);
				   }
				_amem8_f2(&A_Mat_xPtr[2*(8*i)])      = tempSum_03;			
				_amem8_f2(&A_Mat_xPtr[2*(8*i+1)])    = tempSum_03_1;
				_amem8_f2(&A_Mat_xPtr[2*(8*i+2)])    = tempSum_03_2;			
				_amem8_f2(&A_Mat_xPtr[2*(8*i+3)])    = tempSum_03_3;
				_amem8_f2(&A_Mat_xPtr[2*(8*i+4)])    = tempSum_03_4;			
				_amem8_f2(&A_Mat_xPtr[2*(8*i+5)])    = tempSum_03_5;
				_amem8_f2(&A_Mat_xPtr[2*(8*i+6)])    = tempSum_03_6;			
				_amem8_f2(&A_Mat_xPtr[2*(8*i+7)])    = tempSum_03_7;
				
			}						
			/*************************************************/
			/*Calculate lambda = x' * A_Mat * x                              */
			/*************************************************/
			tempSum_03 =  _ftof2(0,0);
			tempSum_03_1 =  _ftof2(0,0);
			tempSum_03_2 =  _ftof2(0,0);
			tempSum_03_3 =  _ftof2(0,0);
			#if(matrixSize_8)
			#pragma MUST_ITERATE(2,2,)
			#else
			#pragma MUST_ITERATE(2,,)
			#endif
			//Loop manually unroll by 4
			for(i = 0; i < size/4; i++){
				temp128_03 = _cmpysp(_amem8_f2(&eVecPtr[2*(4*i)]), _amem8_f2(&A_Mat_xPtr[2*(4*i)]));
				temp_03    = _dsubsp(_hif2_128(temp128_03), _lof2_128(temp128_03));
				tempSum_03 = _daddsp(tempSum_03, temp_03);
				
				temp128_03_1 = _cmpysp(_amem8_f2(&eVecPtr[2*(4*i+1)]), _amem8_f2(&A_Mat_xPtr[2*(4*i+1)]));
				temp_03_1    = _dsubsp(_hif2_128(temp128_03_1), _lof2_128(temp128_03_1));
				tempSum_03_1 = _daddsp(tempSum_03_1, temp_03_1);
				
				temp128_03_2 = _cmpysp(_amem8_f2(&eVecPtr[2*(4*i+2)]), _amem8_f2(&A_Mat_xPtr[2*(4*i+2)]));
				temp_03_2    = _dsubsp(_hif2_128(temp128_03_2), _lof2_128(temp128_03_2));
				tempSum_03_2 = _daddsp(tempSum_03_2, temp_03_2);
				
				temp128_03_3 = _cmpysp(_amem8_f2(&eVecPtr[2*(4*i+3)]), _amem8_f2(&A_Mat_xPtr[2*(4*i+3)]));
				temp_03_3    = _dsubsp(_hif2_128(temp128_03_3), _lof2_128(temp128_03_3));
				tempSum_03_3 = _daddsp(tempSum_03_3, temp_03_3);				
				
			}
			lambda = _hif2(tempSum_03) + _hif2(tempSum_03_1) + _hif2(tempSum_03_2) + _hif2(tempSum_03_3);
			
			/*************************************************/
			/*Calculate A_bar = A -lambda*I                  */
			/* Only diagonal element gets updated            */
			/*************************************************/
			/* Copy loop */
			for(i = 0; i < size; i++) {
				for(j = 0; j < size; j++){
				   _amem8_f2(&A_bar_MatPtr[2*i * size + 2*j]) = _amem8_f2(&A_Mat_Ptr[2*i * size + 2*j]);
				}
			}
			/* Update diagonal element */
		
		    #if(matrixSize_8)
		    #pragma MUST_ITERATE(8,8, )
		    #pragma UNROLL(8)
		    #else
		    #pragma MUST_ITERATE(8,, )
		    #endif
			for(i = 0; i < size; i++) {
				#ifdef _LITTLE_ENDIAN
				A_bar_MatPtr[2*i*(size + 1)+1] = A_Mat_Ptr[2*i*(size + 1)+1] - lambda;
				#else
				A_bar_MatPtr[2*i*(size + 1)] = A_Mat_Ptr[2*i*(size + 1)] - lambda;
				#endif
			}			
			/*********************************************************/
			/* Calculate Z vector vector 						     */
			/*   z = A_Mat_bar * x                                  */  
			/*********************************************************/
		  #if(matrixSize_8)
		   #pragma MUST_ITERATE(1, 1,)
		  #else
		   #pragma MUST_ITERATE(1,,)
		  #endif
		  //Outer loop manually unroll by 8
		    for(i = 0; i < size/8; i++) {
		    	tempSum_03 =  _ftof2(0,0);
				tempSum_03_1 =  _ftof2(0,0);
				tempSum_03_2 =  _ftof2(0,0);
				tempSum_03_3 =  _ftof2(0,0);
				tempSum_03_4 =  _ftof2(0,0);
				tempSum_03_5 =  _ftof2(0,0);
				tempSum_03_6 =  _ftof2(0,0);
				tempSum_03_7 =  _ftof2(0,0);
				for(j = 0; j < size; j++){					
					temp128_03 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03    = _daddsp(_hif2_128(temp128_03), _lof2_128(temp128_03));
					tempSum_03 = _daddsp(tempSum_03, temp_03);
					
					temp128_03_1 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+1) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_1    = _daddsp(_hif2_128(temp128_03_1), _lof2_128(temp128_03_1));
					tempSum_03_1 = _daddsp(tempSum_03_1, temp_03_1);
					
					temp128_03_2 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+2) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_2    = _daddsp(_hif2_128(temp128_03_2), _lof2_128(temp128_03_2));
					tempSum_03_2 = _daddsp(tempSum_03_2, temp_03_2);
					
					temp128_03_3 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+3) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_3    = _daddsp(_hif2_128(temp128_03_3), _lof2_128(temp128_03_3));
					tempSum_03_3 = _daddsp(tempSum_03_3, temp_03_3);
					
					temp128_03_4 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+4) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_4    = _daddsp(_hif2_128(temp128_03_4), _lof2_128(temp128_03_4));
					tempSum_03_4 = _daddsp(tempSum_03_4, temp_03_4);
					
					temp128_03_5 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+5) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_5    = _daddsp(_hif2_128(temp128_03_5), _lof2_128(temp128_03_5));
					tempSum_03_5 = _daddsp(tempSum_03_5, temp_03_5);
					
					temp128_03_6 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+6) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_6    = _daddsp(_hif2_128(temp128_03_6), _lof2_128(temp128_03_6));
					tempSum_03_6 = _daddsp(tempSum_03_6, temp_03_6);
					
					temp128_03_7 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+7) * size + 2*j]), _amem8_f2(&eVecPtr[2*j]));
					temp_03_7    = _daddsp(_hif2_128(temp128_03_7), _lof2_128(temp128_03_7));
					tempSum_03_7 = _daddsp(tempSum_03_7, temp_03_7);
					}
				_amem8_f2(&zVecPtr[2*(8*i)])      = tempSum_03;			
				_amem8_f2(&zVecPtr[2*(8*i+1)])    = tempSum_03_1;
				_amem8_f2(&zVecPtr[2*(8*i+2)])    = tempSum_03_2;
				_amem8_f2(&zVecPtr[2*(8*i+3)])    = tempSum_03_3;
				_amem8_f2(&zVecPtr[2*(8*i+4)])    = tempSum_03_4;			
				_amem8_f2(&zVecPtr[2*(8*i+5)])    = tempSum_03_5;
				_amem8_f2(&zVecPtr[2*(8*i+6)])    = tempSum_03_6;			
				_amem8_f2(&zVecPtr[2*(8*i+7)])    = tempSum_03_7;
			 }
			 
		    /*********************************************************/
			/* Calculate alpha = z'*z                                */
			/*********************************************************/
			
			tempSum_0 =  _ftof2(0,0);
			tempSum_1 =  _ftof2(0,0);
			tempSum_2 =  _ftof2(0,0);
			tempSum_3 =  _ftof2(0,0);
			#if(matrixSize_8)
			#pragma MUST_ITERATE(2,2,)
			#else
			#pragma MUST_ITERATE(2,,)
			#endif
			//Loop manually unroll by 4
			for(i = 0; i< size/4; i++){
				tempValue0 = _amem8_f2(&zVecPtr[2*(4*i)]);
				tempValue1 = _dmpysp(tempValue0, tempValue0);
				tempSum_0  = _daddsp(tempSum_0, tempValue1);
								
				tempValue0_1 = _amem8_f2(&zVecPtr[2*(4*i+1)]);
				tempValue1_1 = _dmpysp(tempValue0_1, tempValue0_1);
				tempSum_1 = _daddsp(tempSum_1, tempValue1_1);
				
				tempValue0_2 = _amem8_f2(&zVecPtr[2*(4*i+2)]);
				tempValue1_2 = _dmpysp(tempValue0_2, tempValue0_2);
				tempSum_2  = _daddsp(tempSum_2, tempValue1_2);
								
				tempValue0_3 = _amem8_f2(&zVecPtr[2*(4*i+3)]);
				tempValue1_3 = _dmpysp(tempValue0_3, tempValue0_3);
				tempSum_3 = _daddsp(tempSum_3, tempValue1_3);
			}
			tempSum_0 = _daddsp(tempSum_0, tempSum_2);
			tempSum_1 = _daddsp(tempSum_1, tempSum_3);
			tempSum = _daddsp(tempSum_0, tempSum_1);
			alpha = _hif2(tempSum) + _lof2(tempSum); /*Real value */
			
			/**********************************************/					
			/*Calculate A_Mat_bar * z i.e. partial beta */
			/**********************************************/			
			#if(matrixSize_8)
			#pragma MUST_ITERATE(1, 1,)
		    #else
		    #pragma MUST_ITERATE(1,,)
		    #endif
			for(i = 0; i < size/8; i++) {
				tempSum_03 =  _ftof2(0,0);
				tempSum_03_1 =  _ftof2(0,0);
				tempSum_03_2 =  _ftof2(0,0);
				tempSum_03_3 =  _ftof2(0,0);
			    tempSum_03_4 =  _ftof2(0,0);
				tempSum_03_5 =  _ftof2(0,0);
				tempSum_03_6 =  _ftof2(0,0);
				tempSum_03_7 =  _ftof2(0,0);
			    	
				for(j = 0; j < size; j++){					
					
					temp128_03 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03    = _daddsp(_hif2_128(temp128_03), _lof2_128(temp128_03));
					tempSum_03 = _daddsp(tempSum_03, temp_03);
					
					temp128_03_1 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+1) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03_1    = _daddsp(_hif2_128(temp128_03_1), _lof2_128(temp128_03_1));
					tempSum_03_1 = _daddsp(tempSum_03_1, temp_03_1);
					
					temp128_03_2 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+2) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03_2    = _daddsp(_hif2_128(temp128_03_2), _lof2_128(temp128_03_2));
					tempSum_03_2 = _daddsp(tempSum_03_2, temp_03_2);
					
					temp128_03_3 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+3) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03_3    = _daddsp(_hif2_128(temp128_03_3), _lof2_128(temp128_03_3));
					tempSum_03_3 = _daddsp(tempSum_03_3, temp_03_3);
					
					temp128_03_4 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+4) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03_4    = _daddsp(_hif2_128(temp128_03_4), _lof2_128(temp128_03_4));
					tempSum_03_4 = _daddsp(tempSum_03_4, temp_03_4);
					
					temp128_03_5 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+5) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03_5    = _daddsp(_hif2_128(temp128_03_5), _lof2_128(temp128_03_5));
					tempSum_03_5 = _daddsp(tempSum_03_5, temp_03_5);
					
					temp128_03_6 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+6) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03_6    = _daddsp(_hif2_128(temp128_03_6), _lof2_128(temp128_03_6));
					tempSum_03_6 = _daddsp(tempSum_03_6, temp_03_6);
					
					temp128_03_7 = _cmpysp(_amem8_f2(&A_bar_MatPtr[2*(8*i+7) * size + 2*j]), _amem8_f2(&zVecPtr[2*j]));
					temp_03_7    = _daddsp(_hif2_128(temp128_03_7), _lof2_128(temp128_03_7));
					tempSum_03_7 = _daddsp(tempSum_03_7, temp_03_7);
				   }
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i)])      = tempSum_03;			
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i+1)])    = tempSum_03_1;
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i+2)])    = tempSum_03_2;
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i+3)])    = tempSum_03_3;
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i+4)])    = tempSum_03_4;			
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i+5)])    = tempSum_03_5;
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i+6)])    = tempSum_03_6;			
				_amem8_f2(&A_bar_Mat_zPtr[2*(8*i+7)])    = tempSum_03_7;
							
			}
						
			/*************************************************/
			/*Calculate beta = z* A_Mat_bar *z'              */
			/*************************************************/
			tempSum_03_0 =  _ftof2(0,0);
			tempSum_03_1 =  _ftof2(0,0);
			tempSum_03_2 =  _ftof2(0,0);
			tempSum_03_3 =  _ftof2(0,0);
						
			#if(matrixSize_8)
			#pragma MUST_ITERATE(2,2,)
			#else
			#pragma MUST_ITERATE(2,,)
			#endif
						for(i = 0; i < size/4; i++){
				temp_03 = _dmpysp(_amem8_f2(&zVecPtr[2*(4*i)]), _amem8_f2(&A_bar_Mat_zPtr[2*(4*i)]));
				tempSum_03_0 = _daddsp(tempSum_03_0, temp_03); 
				
				temp_03_1 = _dmpysp(_amem8_f2(&zVecPtr[2*(4*i+1)]), _amem8_f2(&A_bar_Mat_zPtr[2*(4*i+1)]));
				tempSum_03_1 = _daddsp(tempSum_03_1, temp_03_1); 
				
				temp_03_2 = _dmpysp(_amem8_f2(&zVecPtr[2*(4*i+2)]), _amem8_f2(&A_bar_Mat_zPtr[2*(4*i+2)]));
				tempSum_03_2 = _daddsp(tempSum_03_2, temp_03_2); 
				
				temp_03_3 = _dmpysp(_amem8_f2(&zVecPtr[2*(4*i+3)]), _amem8_f2(&A_bar_Mat_zPtr[2*(4*i+3)]));
				tempSum_03_3 = _daddsp(tempSum_03_3, temp_03_3);
			}			
			tempSum_03_0 = _daddsp(tempSum_03_0, tempSum_03_2);
			tempSum_03_1 = _daddsp(tempSum_03_1, tempSum_03_3);
			tempSum_03 = _daddsp(tempSum_03_0, tempSum_03_1);
			beta = _hif2(tempSum_03) + _lof2(tempSum_03);
			
			/**************************************************/
			
			alphaSq2 = (alpha * alpha)*2;
			betaSq = beta * beta;
			alphaCu4 = (alphaSq2 * alpha)*2;
			betaSqAlphaCu4 = betaSq + alphaCu4;
							
			rSqrtBetaSqAlphaCu4  = _rsqrsp(betaSqAlphaCu4); /* 1/sqrt() instruction 8-bit mantissa precision*/
			#if first_NR_iteration
			/* First NR interpolation for 16-bit precision*/
			rSqrtBetaSqAlphaCu4 = rSqrtBetaSqAlphaCu4 * (1.5f - (betaSqAlphaCu4*0.5f)
									  * rSqrtBetaSqAlphaCu4 * rSqrtBetaSqAlphaCu4); 
			#if scd_NR_iteration
			/* 2nd NR interpolation for 24-bit precision*/
			rSqrtBetaSqAlphaCu4 = rSqrtBetaSqAlphaCu4 * (1.5f - (betaSqAlphaCu4*0.5f)
									  * rSqrtBetaSqAlphaCu4 * rSqrtBetaSqAlphaCu4);
			#endif						   
			#endif
			
			SqrtBetaSqAlphaCu4 = rSqrtBetaSqAlphaCu4 * betaSqAlphaCu4;
			numerator = SqrtBetaSqAlphaCu4 - beta;
			
			denominator = _rcpsp(alphaSq2);
			#if first_NR_iteration
			/* First NR interpolation for 16-bit precision*/
			denominator = denominator * (2.f - alphaSq2 * denominator);
			#if scd_NR_iteration
			/* 2nd NR interpolation for 24-bit precision*/
			denominator = denominator * (2.f - alphaSq2 * denominator);
			#endif
			#endif
			
			gamma = numerator*denominator;
			
			gamma_double = _ftof2(gamma, gamma);		
						
			/*********************************************************/
			/* Update eigen vector 	x = x - gamma*z								 */
			/*********************************************************/
			tempSum_0 =  _ftof2(0,0);
			tempSum_1 =  _ftof2(0,0);
			for(i = 0; i< size/2; i++){				
				tempValue1 = _dsubsp(_amem8_f2(&eVecPtr[2*2*i]), _dmpysp(gamma_double, _amem8_f2(&zVecPtr[2*2*i])));
				_amem8_f2(&eVecPtr[2*2*i]) = tempValue1;
				tempSum_0 = _daddsp(tempSum_0, _dmpysp(tempValue1, tempValue1));				
				
				tempValue1_1 = _dsubsp(_amem8_f2(&eVecPtr[2*(2*i+1)]), _dmpysp(gamma_double, _amem8_f2(&zVecPtr[2*(2*i+1)])));
				_amem8_f2(&eVecPtr[2*(2*i+1)]) = tempValue1_1;
				tempSum_1 = _daddsp(tempSum_1, _dmpysp(tempValue1_1, tempValue1_1));
			}
			tempSum = _daddsp(tempSum_0, tempSum_1);
			sum = _hif2(tempSum) + _lof2(tempSum);
			
			rSqrttempSum  = _rsqrsp(sum); /* 1/sqrt() instruction 8-bit mantissa precision*/
			#if first_NR_iteration
			/* First NR interpolation for 16-bit precision*/
			rSqrttempSum = rSqrttempSum * (1.5f - (sum*0.5f)
									  * rSqrttempSum * rSqrttempSum); 
			#if scd_NR_iteration
			/* 2nd NR interpolation for 24-bit precision*/
			rSqrttempSum = rSqrttempSum * (1.5f - (sum*0.5f)
									  * rSqrttempSum * rSqrttempSum); 
			#endif
			#endif
			/***********************************************************/
			/* Normalize updated eigen vector                          */
			/***********************************************************/
			
			rSqrttempSum_double = _ftof2(rSqrttempSum, rSqrttempSum);
			#if(matrixSize_8)
			#pragma MUST_ITERATE(4,4,)
			#pragma UNROLL(2)  /* Note size has to less than or equal to 8*/
		    #else
		    #pragma MUST_ITERATE(4,,)
			#pragma UNROLL(2)  /* Note size has to less than or equal to 8*/
		    #endif		    
			for(i = 0; i< size; i++)
				_amem8_f2(&eVecPtr[2*i]) = _dmpysp(rSqrttempSum_double, _amem8_f2(&eVecPtr[2*i]));
			
		}/* End of do */
		while ((alpha > Tolerance) && (count < maxIteration) );
		
		eigenValPtr[0] = -lambda;
		iteration[0] = count;
		
		
		
   }
   
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 
						 

				  
				  

