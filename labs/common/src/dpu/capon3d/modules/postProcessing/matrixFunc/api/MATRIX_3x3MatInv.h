/**
 *   @file  MATRIX_3x3MatInv.h
 *
 *   @brief
 *      Header file for 3x3 matrix inversion functions
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

#include "swpform.h"
#ifdef _TMS320C6X
#include "c6x.h"
#endif


#ifndef MATRIX_3x3MATINVER
#define MATRIX_3x3MATINVER


/** 
 *  \fn     void 		MATRIX_single3x3MatInv (             
 *          							IN      cplx32_t * RESTRICT Input,   
 *          							OUT     cplxf_t * RESTRICT output);		
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
extern void 		MATRIX_single3x3MatInv (  
								IN      cplx32_t * RESTRICT Input,
								OUT     cplxf_t * RESTRICT output);		
								
/** 
 *  \fn     void 		MATRIX_multi3x3MatInvFltOut (   
 *          							IN      cplx32_t * RESTRICT Input, 
 *          							IN      int32_t 		n,            
 *          							OUT     cplxf_t * RESTRICT out);
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
extern void 		MATRIX_multi3x3MatInvFltOut (  
								IN      cplx32_t * RESTRICT Input,
								IN      int32_t 		n,
								OUT     cplxf_t * RESTRICT out);


								
/** 
 *  \fn     void 		MATRIX_multi3x3MatInvFxdOut (    
 *          							IN      cplx32_t * RESTRICT Input,  
 *          							IN      int32_t 		n,             
 *          							IN      int32_t         range,   
 *          							OUT     int32_t	 * RESTRICT Qinv,
 *          							OUT     cplx16_t * RESTRICT out);
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
extern void 		MATRIX_multi3x3MatInvFxdOut (  
								IN      cplx32_t * RESTRICT Input,
								IN      int32_t 		n,
								IN      int32_t         range,
								OUT     int32_t	 * RESTRICT Qinv,
								OUT     cplx16_t * RESTRICT out);
#endif //MATRIX_3x3MATINVER
