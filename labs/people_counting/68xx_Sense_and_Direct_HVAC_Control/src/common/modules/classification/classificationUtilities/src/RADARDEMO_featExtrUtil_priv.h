/*!
 *  \file   RADARDEMO_featExtrUtil_priv.h
 *
 *  \brief   Header file for RADARDEMO_featExtrUtil_priv.c
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
#ifndef RADARDEMO_FEATEXTRUTIL_PRIV_H
#define RADARDEMO_FEATEXTRUTIL_PRIV_H

#include <swpform.h>

#ifdef _TMS320C6X
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#endif


/*!
 *   \fn     RADARDEMO_featExtrUtilCoMEst
 *
 *   \brief   Estimate center of mass of the input signal with weighting factor. 
 *
 *   \param[in]    numInputs
 *               number of samples in the input vector
 *
 *   \param[in]    sigIn
 *               Pointer to the input signal. 
 *
 *   \param[in]    weight
 *               Pointer to the weighting factor.
 *
 *   \ret        Center of mass
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
float RADARDEMO_featExtrUtilCoMEst(
                            IN  int16_t numInputs,
                            IN  float * RESTRICT sigIn,
                            IN  float * RESTRICT  weight);

/*!
 *   \fn     RADARDEMO_featExtrUtilStatisticsEst
 *
 *   \brief   Calculate the statistics of the input signal, min, max, mean, std, and optional histogram.
 *
 *   \param[in]    numInputs
 *               number of samples in the input vector
 *
 *   \param[in]    sigIn
 *               Pointer to the input signal. 
 *
 *   \param[in]    histoBinSize
 *               Bin size of the histogram
 *
 *   \param[in]    numHistoBins
 *               Number of bins in the histogram.
 *
 *   \param[in]    sigCoM
 *               Center of mass for the input signal, used for histogram
 *
 *   \param[out]    min
 *               The minimum value of the signal.
 *
 *   \param[out]    max
 *               The maximum value of the signal.
 *
 *   \param[out]    mean
 *               The average of the signal.
 *
 *   \param[out]    std
 *               The standard deviation of the signal.
 *
 *   \param[out]    histPtr
 *               Pointer to the output histogram of the input signal, with signal center of mass being the middle point of the bins 
 *               (e.g., removed sigCoM from the signal), and bin width defined in histoBinSize. If set to NULL, then no calculation.
 *
 *   \pre       User needs to guarentee max(abs(sigIn - sigCoM))/histoBinSize < length(histogram)/2, to avoid memory access violation.
 *              
 *
 *   \post      none
 *
 *
 */
void  RADARDEMO_featExtrUtilStatisticsEst(
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT sigIn,
                            IN  float histoBinSize,
                            IN  int16_t numHistoBins,
                            IN  float sigCoM,
                            OUT float   *  RESTRICT min,
                            OUT float   *  RESTRICT max,
                            OUT float   *  RESTRICT mean,
                            OUT float *  RESTRICT std,
                            OUT uint32_t *  RESTRICT histPtr
							);


/*!
 *   \fn     RADARDEMO_featExtrUtilStatisticsEst
 *
 *   \brief   Calculate the statistics of the input signal, min, max, mean, std, and optional histogram.
 *
 *   \param[in]    rmCoMinStatsFlag
 *               Flag, if set to 1, mandates removing center of mass from output statistics of max and min. 
 *               Otherwise set to 0. 
 *
 *   \param[in]    numInputs
 *               number of samples in the input vector
 *
 *   \param[in]    sigInX
 *               Pointer to the input signal, X axis. 
 *
 *   \param[in]    sigInY
 *               Pointer to the input signal, Y axis. 
 *
 *   \param[in]    histoBinSize
 *               Bin size of the histogram
 *
 *   \param[in]    numHistoBins
 *               Number of bins in the histogram.
 *
 *   \param[in]    sigCoMX
 *               Center of mass for the input signal in X axis, used for histogram
 *
 *   \param[in]    sigCoMY
 *               Center of mass for the input signal in Y axis, used for histogram
 *
 *   \param[out]    min
 *               The minimum value of the signal.
 *
 *   \param[out]    max
 *               The maximum value of the signal.
 *
 *   \param[out]    mean
 *               The average of the signal.
 *
 *   \param[out]    std
 *               The standard deviation of the signal.
 *
 *   \param[out]    histPtr
 *               Pointer to the output histogram of the input signal, with signal center of mass being the middle point of the bins 
 *               (e.g., removed sigCoM from the signal), and bin width defined in histoBinSize. If set to NULL, then no calculation.
 *
 *   \pre       User needs to guarentee max(abs(sigIn - sigCoM))/histoBinSize < length(histogram)/2, to avoid memory access violation.
 *              
 *
 *   \post      none
 *
 *
 */
void  RADARDEMO_featExtrUtil2DStatisticsEst(
                            IN  int8_t  rmCoMinStatsFlag,
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT sigInX,
                            IN  float *  RESTRICT sigInY,
                            IN  float histoBinSize,
                            IN  int16_t numHistoBins,
                            IN  float sigCoMX,
                            IN  float sigCoMY,
                            OUT float   *  RESTRICT min,
                            OUT float   *  RESTRICT max,
                            OUT float   *  RESTRICT mean,
                            OUT float *  RESTRICT std,
                            OUT uint32_t *  RESTRICT histPtr
							);
#endif //RADARDEMO_FEATEXTRUTIL_PRIV_H

