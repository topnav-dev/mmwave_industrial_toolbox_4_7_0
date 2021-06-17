/**
 *   @file  MATRIX_EVD_largestEigValue.h
 *
 *   @brief
 *      Header file for Manton's method to find largest eigenvalue and its correspongding eigen vector
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
#ifndef MATRIX_EVD_LARGESTEIGVALUEFLOAT_H
#define MATRIX_EVD_LARGESTEIGVALUEFLOAT_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif
 
#ifdef _TMS320C6X
#include "c6x.h"
#endif

 
#include "swpform.h"

/**
 *  \fn     void MATRIX_EVD_largestEigValueFloat(   
 *                  IN    cplxf_t * RESTRICT A_Mat, 
 *                  IN    float * RESTRICT sctrachBufPtr, 
 *                  IN    int32_t   size,                 
 *                  IN    float Tolerance,                    
 *                  OUT   float *   eigenValPtr,          
 *                  INOUT cplxf_t*  RESTRICT eigenVectPtr,
 *                  INOUT int16_t * iteration);           
 * 
 *  \brief   Reduce complexity Eigen Value Decomposition, to get extreme eigen value and corresponding eigen vector, based on Jonathon Manton's iterative 
 *           method based on as explained in http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=00955263. Input parameter "Tolerance" is used as the stoping 
 *           criteria of this iterative method. Choosing the Tolrence decides the precision of the eigen values and thereofore the performance of the loop
 *           (i.e. number of Manton's iteration). User has to do performance analysis with different tolerance values to get the required precision.
 *           Input, scratch buffer and output arrays must be aligned on a double word (8-byte) boundary using DATA_ALIGN pragma. 
 *
 *  \param[in]    A_Mat
 *              Input Complex Hermitian Matrix of dimension matSize X matSize. All elements of the Matrix A_Mat are stored in 1-d array row-wise format. 
 *              Storage requirement for this array is =  2*4*matSize*matSize bytes. To avoid cache penalties it is recommended to warm-up L1D cache with 
 *              A_Mat using touch() loop. A_Mat must be aligned on the 8 byte boundary using DATA_ALIGN pragma.
 *              Below formula will help visualize how A_Mat(row, column) complex Hermitian single precision floating-point matrix of size N is stored in memory.
 *              For example i represents row and j represents column and the 1-d array float variable is A_mat[2*N*N]. So row 1 of A_Mat is stored is A_mat[0 : 2*N-1],
 *              2nd row of A_Mat is stored in A_mat[2*N:4*N-1] and Mth row of A_Mat is stored in A_mat[2*N*M : 2*N*M + 2*N - 1].
 *
 *  \param[in]    sctrachBufPtr
 *              The scratchpad memory needed for intermediate data storage. This array must be aligned on the 8-byte boundary using DATA_ALIGN pragma. 
 *              Storage requiement for this array is = (4*matSize*matSize + 6*matSize)*4 bytes, where matSize is the size of the matrix.
 *
 *  \param[in]    size
 *              It contains Matrix size. For 8x8 Matrix, size variable equals to 8.
 *
 *  \param[in]    Tolerance
 *              Tolerance specify the minimum threshold needed for the residual error calculated in the algorithm and used as stopping criteria for the Manton's iterations
 *              to approximate extreme Eigen vector. Based on the performance analysis in MATLAB to get precision accurate to at least 2 decimal digit tolerance needs to be 
 *              0.01. MATLAB code is also checked-in to the library for the user to play with the desired precision and decide tolerance as well as maximum number of iterations.
 *
 *  \param[out]    eigenValPtr
 *              Pointer to the extreme eigen value variable.
 *
 *  \param[in,out] eigenVectPtr
 *              The pointer to the array where Eigen vector, of size equals to  1 X matSize, for the extreme Eigen value is stored. eigenVectorPtr complex array needs
 *              to be pre-initialized with all zeros except the real part of the first element. Real part of the first element needs to be initialized with 1.0. 
 *              This is the initial estimate of the eigenvector of the corresponding extreme Eigen value of the matrix and Manton's iteration method iterated to converge
 *              to the actual complex eigenvectors. It is recommended to warm-up the cache to take advantage of the writeback cache. This array needs to be aligned on the
 *              8 byte boundary.
 *   
 *  \param[in,out] iteration
 *              Input: Maximum iteration allowed for a given performance and precision.
 *              Output: Total number of iterations executed by the Manton's algorithm for a given precision (based on tolerance parameter).
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
        IN    cplxf_t * RESTRICT A_Mat,
        IN    float * RESTRICT sctrachBufPtr, 
        IN    int32_t   size,
        IN    float Tolerance,      
        OUT   float *   eigenValPtr,
        INOUT cplxf_t*  RESTRICT eigenVectPtr,
        INOUT int16_t * iteration);
                        
#ifdef __cplusplus
} 
#endif

#endif
