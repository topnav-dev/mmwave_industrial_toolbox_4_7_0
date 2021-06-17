/**
 *   @file  MATRIX_EVDJacobian.h
 *
 *   @brief
 *      Header file for EVD using Jacobi method
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
#ifndef MATRIX_EVDFLOAT_H
#define MATRIX_EVDFLOAT_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif
 
#ifdef _TMS320C6X
#include "c6x.h"
#endif

 
#include "swpform.h"

/**
 *  \fn    void MATRIX_EVDFloat(                            
 *              IN     int32_t matrixSize,               
 *              IN     float tolerance,                  
 *              INOUT  cplxf_t* RESTRICT eigenValPtr,         
 *              INOUT  cplxf_t* RESTRICT eigenVectMatPtr,
 *              INOUT  int32_t * iteration);             
 * 
 *  \brief   This is inplace implementation of Modified Jacobian Method for Eigen Value Decomposition of Hermitian symmetric and positive definite matrix in floating-point. 
 *           It also works for non-positive definite as long as it is Hermitian symmetric. For Hermitian symmetric matrix A, this function decomposes A into V*D*V' by doing 
 *           sequence of Givens rotations to minimize off-diagonal norm of the matrix:  where V is the eigenvector matrix and D is the diagonal matrix of Eigen values. Input 
 *           parameter "Tolerance" is used as the limit for the off-diagonal norm of the matrix calculated after every Givens rotations. Choosing the Tolerance decides the 
 *           precision of the Eigen values and therefore the performance of the loop (i.e. number of iterations). User has to do performance analysis with different tolerance
 *           values to get the required precision using the matlab project at "../unittest/matlab".
 *
 *  \param[in]    matrixSize
 *                Matrix size, for 8x8 Matrix, matrixSize variable equals to 8. This implementation is optimized for upto 8x8 matrix.
 
 *  \param[in]    tolerance
 *                Input parameter "Tolerance" is used as the stopping criteria for the sequence of Givens rotations in the Jacobian methods as it is used as limit for the 
 *                off-diagonal norm calculated after every rotation. Tolerance value decides the precision needed for the generated Eigen values and therefore the performance 
 *                of the loop (i.e. number of Givens rotations). 
 
 *  \param[in,out] eigenValPtr
 *                 This is input as well as output parameter. Input is the Complex Hermitian matrix of dimension size X size. Output is all diagonal matrix of eignevalues. 
 *                 All elements of the Matrix A_Mat are stored in 1-d array row-wise. Storage requirement for this array is = 2*4*size*size bytes. 
 *                 The 1-d arrays must be aligned on the 8 byte boundary using DATA_ALIGN pragma. 
 *                 Below formula will help visualize how A_Mat(row, column) complex Hermitian single precision floating-point matrix of size N is stored in memory.
 *                 For example i represents row and j represents column and the 1-d array float variable is A_mat[2*N*N]. So row 1 of A_Mat is stored is A_mat[0 : 2*N-1],
 *                 2nd row of A_Mat is stored in A_mat[2*N:4*N-1] and Mth row of A_Mat is stored in A_mat[2*N*M : 2*N*M + 2*N - 1].
 * 
 * 
 *  \param[in,out] eigenVectMatPtr
 *                 The pointer to Eigen vector matrix stored as 1-D array row-wise. This array pointer needs to be aligned on the 8 byte boundary. It is recommended to 
 *                 warm-up the cache with these values to avoid any cache penalties. Note that 1-d array pointed by eigenVectPtr needs to be pre-initialized with identity
 *                 matrix (I) of same size. This can be done by calling library function MATRIX_EVDFloat_init() function.
 *  
 *  \param[in,out] iteration
 *                  Input: Maximum iteration allowed for a given performance and precision.
 *                  Output: Total number of iterations executed by the Jacobian method to get the desired precision for the off-diagonal norm specified by Tolerance.
 *
 *  \pre      eigenVectMatPtr array matrixr needs to be pre-initialized with identity matrix of size equal to matrixSize. This can be done by calling MATRIX_EVD_init() function.
 *            eigValPtr and eigenVectMatPtr array pointers must be aligned on a double word (8-byte) boundary using DATA_ALIGN pragma. 
 *
 *  \post
 *
 *  \sa
 *
 */
 
 
                         
void MATRIX_EVDFloat(       
        IN     int32_t matrixSize, 
        IN     float tolerance,
        INOUT  cplxf_t* RESTRICT eigenValPtr,       
        INOUT  cplxf_t* RESTRICT eigenVectMatPtr,
        INOUT  int32_t * iteration); 

#ifdef __cplusplus
} 
#endif

#endif
