/**
 *   @file  MATRIX_4x4MatInv.h
 *
 *   @brief
 *      Header file for 4x4 matrix inversion functions
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


#ifndef MATRIX_4x4MATINVER
#define MATRIX_4x4MATINVER


/** 
 *  \fn     void 		MATRIX_single4x4MatInv (             
 *          							IN      cplx32_t * RESTRICT Input,   
 *          							OUT     cplxf_t * RESTRICT output);		
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

extern void 		MATRIX_single4x4MatInv (  
								IN      cplx32_t * RESTRICT Input,
								OUT     cplxf_t * RESTRICT output);								


/** 
 *  \fn     void 		MATRIX_multi4x4MatInvFltOut (              
 *          							IN      cplx32_t * RESTRICT Input,         
 *          							IN      int32_t		N,                        
 *          							IN      int32_t    * RESTRICT localScratch,
 *          							OUT     cplxf_t * RESTRICT invA);          
 *
 *  \brief   Multiple 4x4 matrices inversion (complex positive semi-definitive Hermitian) using block-wiss method.
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

extern void 		MATRIX_multi4x4MatInvFltOut (  
								IN      cplx32_t * RESTRICT Input,
								IN      int32_t		N,
								IN      int32_t    * RESTRICT localScratch,
								OUT     cplxf_t * RESTRICT invA);


/** 
 *  \fn     void 		MATRIX_multi4x4MatInvFxdOut (          
 *          							IN  cplx32_t * RESTRICT Input,         
 *          							IN  int32_t		N,                        
 *          							IN  int32_t    * RESTRICT localScratch,
 *          							IN  int32_t   range,                   
 *          							OUT int32_t	* RESTRICT Qinv,           
 *          							OUT cplx16_t * RESTRICT invA);         
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

extern void 		MATRIX_multi4x4MatInvFxdOut (  
								IN  cplx32_t * RESTRICT Input,
								IN  int32_t		N,
								IN  int32_t    * RESTRICT localScratch,
								IN  int32_t   range,
								OUT int32_t	* RESTRICT Qinv,
								OUT cplx16_t * RESTRICT invA);

#endif //MATRIX_4x4MATINVER
