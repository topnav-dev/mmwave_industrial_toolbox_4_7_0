/*!
 *  \file   RADARDEMO_distanceUtil_priv.h
 *
 *  \brief   Header file for RADARDEMO_distanceUtil_priv.c
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/
#ifndef RADARDEMO_DISTANCEUTIL_PRIV_H
#define RADARDEMO_DISTANCEUTIL_PRIV_H

//! \brief include file <swpform.h>
//!
#include <swpform.h>

#ifdef _TMS320C6X
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#endif


/*!
 *   \fn     RADARDEMO_distanceUtilSqrEuc
 *
 *   \brief   Calculate the square Euclidean distance between input and codebook. Also output the minimum distance and its index.
 *
 *   \param[in]    numCodewords
 *               number of codewords in the codebook.
 *
 *   \param[in]    codewordDim
 *               Dimension of the codeword.
 *
 *   \param[in]    sigIn
 *               Pointer to the input signal, a vector of size codewordDim. 
 *
 *   \param[in]    codebook
 *               Pointer to the codebook, in format of (c[0][0], c[0][1], ...c[0][codewordDim-1], c[1][0], c[1][1], ...c[0][codewordDim-1], ...c[numCodewords-1][codewordDim-1].
 *
 *   \param[out]    minDistCWIndex
 *               Output index of the minimum distance.
 *
 *   \param[out]    sqrEucDist
 *               Output minimum distance, length of numCodewords.
 *
 *   \ret        Minimum distance.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
float RADARDEMO_distanceUtilSqrEuc(
                            IN  int16_t numCodewords,
							IN  int8_t  codewordDim,
                            IN  float *  RESTRICT sigIn,
                            IN  float *  RESTRICT codebook,
							OUT uint16_t * RESTRICT minDistCWIndex, 
							OUT float * RESTRICT sqrEucDist);

/*!
 *   \fn     RADARDEMO_distanceUtilMahalanobis
 *
 *   \brief   Calculate the Mahalanobis distance between input and codebook. Also output the minimum distance and its index.
 *
 *   \param[in]    numCodewords
 *               number of codewords in the codebook.
 *
 *   \param[in]    codewordDim
 *               Dimension of the codeword.
 *
 *   \param[in]    sigIn
 *               Pointer to the input signal, a vector of size codewordDim. 
 *
 *   \param[in]    codebook
 *               Pointer to the coebook, in format of (c[0][0], c[0][1], ...c[0][codewordDim-1], c[1][0], c[1][1], ...c[0][codewordDim-1], ...c[numCodewords-1][codewordDim-1].
 *
 *   \param[in]    codebookRCholinv
 *               Pointer to the upper inverse of the cholesky covariance matrix of the codebook per codeword, in format of (r[0][0], r[0][1], ...c[0][codewordDim-1], r[1][1], ...c[0][codewordDim-1], ...r[codewordDim-1][codewordDim-1]. 
 *               This buffer only store the upper matrix element by element.
 *
 *   \param[out]    minDistCWIndex
 *               Output index of the minimum distance.
 *
 *   \param[out]    sqrEucDist
 *               Output minimum distance, length of numCodewords.
 *
 *   \ret        Minimum distance.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
float RADARDEMO_distanceUtilMahalanobis(
                            IN  int16_t numCodewords,
							IN  int8_t  codewordDim,
                            IN  float *  RESTRICT sigIn,
                            IN  float *  RESTRICT codebook,
                            IN  float *  RESTRICT codebookRCholinv,
							OUT uint16_t * RESTRICT minDistCWIndex, 
							OUT float * RESTRICT mahalanobisDist);

/*!
 *   \fn     RADARDEMO_distanceFindMin
 *
 *   \brief   Find the minimum distance and its index.
 *
 *   \param[in]    numInputs
 *               number of input samples.
 *
 *   \param[in]    distIn
 *               Pointer to the input distance samples. 
 *
 *   \param[out]    minDistIndex
 *               Output index of the minimum distance.
 *
 *   \ret        Minimum distance.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern float RADARDEMO_distanceFindMin(
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT distIn,
							OUT uint16_t * RESTRICT minDistIndex);

#endif //RADARDEMO_DISTANCEUTIL_PRIV_H

