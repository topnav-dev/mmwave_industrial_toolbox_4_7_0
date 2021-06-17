/**
 *   @file  MATRIX_8x8MatInv.h
 *
 *   @brief
 *      Header file for 8x8 matrix inversion functions
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

#ifndef MATRIX_8X8MATINVER
#define MATRIX_8X8MATINVER

#include "swpform.h"
#ifdef _TMS320C6X
#include "c6x.h"
#endif

#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif


/** 
 *  \fn     void MATRIX_single8x8MatInv (                            
 *                                IN      cplx32_t * RESTRICT Input, 
 *                                IN      int32_t * RESTRICT scratch,
 *                                OUT     cplxf_t  * output);        
 *
 *  \brief   Single 8x8 matrix (complex positive semi-definitive Hermitian) inversion using block wise method.
 * 
 *  \param[in]    Input
 *              Input 8x8 matrix that needs to be inversed, stored sequentially 
 *              in format (1,1), (1,2).... (1,8), (2,1), (2,2)...(8,8). 
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]    scratch
 *              Input pointer to the scratch memory. Must be of size 7 * 2 * 16 = 288 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]   output
 *              Output 8x8 matrix. Stored sequentially as the input.
 *              Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   MATRIX_4x4_BWInversionfp
 * 
 */

extern void MATRIX_single8x8MatInv (  
                             IN      cplx32_t * RESTRICT Input,
                             IN      int32_t * RESTRICT scratch,
                             OUT     cplxf_t  * output);


/** 
 *  \fn     void MATRIX_multi8x8MatInvFltOut (                       
 *                                IN      cplx32_t * RESTRICT Input, 
 *                                IN      int32_t	  N,               
 *                                IN      int32_t * RESTRICT scratch,
 *                                OUT     cplxf_t  * output);        
 *
 *  \brief   Multiple 8x8 matrices (complex positive semi-definitive Hermitian) inversion using block wise method with floating-point output.
 * 
 *  \param[in]    Input
 *              Input pointer to N of 8x8 matrices that needs to be inversed, stored sequentially 
 *              in format (0,0), (0,1).... (0,7), (1,0), (1,1)...(7,7). 
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    N
 *              Number of 8x8 matrix to be inverted. .
 *
 *  \param[out]    scratch
 *              Input pointer to the scratch memory. Must be of size 6 * N * 2 * 16 = 192 * N 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]   output
 *              Output 8x8 matrices. Stored sequentially as the input.
 *              Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */


extern void MATRIX_multi8x8MatInvFltOut (  
                             IN      cplx32_t * RESTRICT Input,
                             IN      int32_t	  N,
                             IN      int32_t * RESTRICT scratch,
                             OUT     cplxf_t  * output);

/** 
 *  \fn     void MATRIX_multi8x8MatInvFxdOut (                       
 *                                IN      cplx32_t * RESTRICT Input, 
 *                                IN      int32_t	  N,               
 *                                IN      int32_t * RESTRICT scratch,
 *								  IN      int32_t         range,                    
 *								  OUT     int32_t	 * RESTRICT Qinv,                 
 *                                OUT     cplx16_t  * output);       
 *
 *  \brief   Multiple 8x8 matrices (complex positive semi-definitive Hermitian) inversion using block wise method with fixed-point output.
 * 
 *  \param[in]    Input
 *              Input pointer to N of 8x8 matrices that needs to be inversed, stored sequentially 
 *              in format (0,0), (0,1).... (0,7), (1,0), (1,1)...(7,7). 
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    N
 *              Number of 8x8 matrix to be inverted. .
 *
 *  \param[out]    scratch
 *              Input pointer to the scratch memory. Must be of size 6 * N * 2 * 16  + 64 * 2 * N = 320 * N 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]   range
 *              The full range of the output. For example, if we set it to 14, that means the real and imaginary
 *              part of the output elements will not exceed 14-bit with sign.
 *
 *  \param[out]   Qinv
 *              Output block Q value for the inverse matrices.
 *
 *  \param[out]   output
 *              Output 8x8 matrices. Stored sequentially as the input.
 *              Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */


extern void MATRIX_multi8x8MatInvFxdOut (  
                             IN      cplx32_t * RESTRICT Input,
                             IN      int32_t	  N,
                             IN      int32_t * RESTRICT scratch,
							 IN      int32_t         range,
							 OUT     int32_t	 * RESTRICT Qinv,
                             OUT     cplx16_t  * output);

#endif //MATRIX_8X8MATINVER
