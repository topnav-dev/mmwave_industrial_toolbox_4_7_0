
/**
 *  \file   RADARDEMO_aoaEst2DCaponBF_angleEst.c
 *
 *   \brief   Estimate the azimuth and elevation angle of arrival using 2D BF.
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "RADARDEMO_aoaEst2DCaponBF_priv.h"

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif



/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_aeEstElevOnly
 *
 *   \brief   Use Capon beamforming to estimation elevation angle in 2D domain.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    azimuthIdx
 *               Input azimuth index for a detected point. 
 *
 *   \param[in]    capon_handle
 *               Input capon beamforming handle. 
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    elevEst
 *               Output elevation estimations, length of numAngleOut are valid.
 *
 *   \param[out]    peakPow
 *               Output peak power estimations, length of numAngleOut are valid.
 *
 *   \param[out]    beamFilter
 *               Output beamFilter estimations, in complex float, length of numAngleOut * aeEstimation_handle->nRxAnt are valid.
 *
 *   \param[out]    azimElevHeatMap
 *               Output azim-elev heatMap estimations.
 *
 *   \ret       numAngleOut: Number of estimated angle pairs.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
int32_t RADARDEMO_aoaEst2DCaponBF_aeEstElevOnly(
				IN uint8_t bfFlag,
				IN uint16_t azimuthIdx,
				IN RADARDEMO_aoaEst2DCaponBF_handle   * capon_handle,
				IN cplxf_t * RESTRICT invRnMatrices,
				OUT float  * RESTRICT azimEst, 
				OUT float  * RESTRICT elevEst, 
				OUT float  * RESTRICT peakPow, 
				OUT cplxf_t  * RESTRICT beamFilter, 
				OUT float    * RESTRICT azimElevHeatMap
			)
{
	int32_t		i, j, k, rnIdx, elevIdx, rnIdx1, scratchOffset, nRxAnt, numElevationBins, maxElevInd, numAngleOut, elevIdxLeft, elevIdxRight;
	__float2_t	* RESTRICT steeringVecAzimInit;
	__float2_t	* RESTRICT steeringVecElevInit;
	__float2_t	* RESTRICT steeringVecInit;
	__float2_t	* RESTRICT beamFilterPtr;
	float		* RESTRICT heatMapPtr;
	__float2_t  f2temp, acc0f2;
	float		output, result, maxVal, tempAzim;
	float       muIdxInterp;

	nRxAnt				=	capon_handle->nRxAnt;
	numElevationBins	=	capon_handle->raHeatMap_handle->elevSearchLen;

#ifdef _TMS320C6X
	_nassert(nRxAnt %4	==	0);
    _nassert(numElevationBins >= 32);
#endif


	if  (nRxAnt == 12)
	{
	    int32_t bankIdx, nBanks, bankSize, tempSteerVecLen, steerVecSizeInBytes, rnIdxRec, procSize;
	    float   diagSum, output1;
	    __float2_t  * RESTRICT steeringVecPtr;

        j               =   0;
        //diagSum           =   _hif2(_amem8_f2(&invRnMatrices[0]));
        diagSum         =   0;
        for (i = 0; i < nRxAnt; i++)
        {
            diagSum     +=  _hif2(_amem8_f2(&invRnMatrices[i * nRxAnt + i - j]));
            j           +=  i + 1;
        }
        diagSum         =   diagSum * 0.5f;

	    steerVecSizeInBytes =   (int32_t) (numElevationBins * nRxAnt * 8);

	    tempSteerVecLen     =   steerVecSizeInBytes;
	    nBanks              =   0;
        for (; tempSteerVecLen > 0; tempSteerVecLen -= (int32_t)capon_handle->aeEstimation_handle->scratchPadSize)
        {
            nBanks++;
        }

	    bankSize            =   (int32_t)((float)numElevationBins * _rcpsp((float)nBanks) + 1.f);
	    tempSteerVecLen     =   numElevationBins;
        steeringVecAzimInit =   (__float2_t *) &capon_handle->raHeatMap_handle->steeringVecAzim[azimuthIdx * nRxAnt];

        bankIdx             =   0;
        maxElevInd          =   0;
        maxVal              =   0.f;
	    for (; tempSteerVecLen > 0; tempSteerVecLen -= bankSize)
	    {
            steeringVecInit =   (__float2_t *) &capon_handle->aeEstimation_handle->scratchPad[0];
            steeringVecElevInit                         =   (__float2_t *) capon_handle->raHeatMap_handle->steeringVecElev;
            procSize        =   bankSize;
            if (tempSteerVecLen < procSize)
                procSize    =   tempSteerVecLen;
#ifdef _TMS320C6X
            _nassert(procSize >= 20);
#endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {

                #ifdef _TMS320C6X
                #pragma UNROLL(12);
                #endif
                for (i = 0; i < nRxAnt; i++ )
                {
                    _amem8_f2(&steeringVecInit[elevIdx * nRxAnt + i])      =   _complex_mpysp(_amem8_f2(&steeringVecElevInit[(bankIdx * bankSize + elevIdx)* nRxAnt + i]), _amem8_f2(&steeringVecAzimInit[i]));
                }
            }

            heatMapPtr                                  =   azimElevHeatMap;

            // complete manual unroll for 12 antennas
            // 01 to 06
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   0;

                // skip 00
                rnIdx++;

                // 01 and 10
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[1]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 =   (_hif2(acc0f2) - _lof2(acc0f2));

                // 02 and 20
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[2]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 03 and 30
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[3]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 04 and 40
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[4]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 05 and 50
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[5]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 06 and 60
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[6]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1 + diagSum;
            }
            rnIdxRec                    =   rnIdx;

            // for 07 to 12
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 07 and 70
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[7]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 08 and 80
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 09 and 90
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 0a and a0
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 0b and b0
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[0]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 11
                rnIdx++;

                // 12 and 21
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[2]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }

            rnIdxRec                    =   rnIdx;

            // for 13 to 18
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 13 and 31
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[3]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 14 and 41
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[4]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 15 and 51
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[5]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 16 and 61
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[6]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 17 and 71
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[7]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 18 and 81
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }
            rnIdxRec                    =   rnIdx;

            // for 19 to 25
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 19 and 91
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                 // 1a and a1
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 1b and b1
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[1]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 22
                rnIdx++;

                // 23 and 32
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[3]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 24 and 42
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[4]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 25 and 52
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[5]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }
            rnIdxRec                    =   rnIdx;

            // for 26 to 2b
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 26 and 62
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[6]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

               // 27 and 72
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[7]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

               // 28 and 82
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 29 and 92
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 2a and a2
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 2b and b2
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[2]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 33
                rnIdx++;

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
             }
             rnIdxRec                    =   rnIdx;

             // for 34 to 39
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
             for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
             {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 34 and 43
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[4]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=  (_hif2(acc0f2) - _lof2(acc0f2));

                // 35 and 53
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[5]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 36 and 63
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[6]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 37 and 73
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[7]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 38 and 83
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 39 and 93
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }

            rnIdxRec                    =   rnIdx;

            // for 3a to 48
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 3a and a3
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 3b and b3
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[3]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 44
                 rnIdx++;

                 // 45 and 54
                 f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                 acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[4]), f2temp);
                 f2temp                  =   _amem8_f2(&steeringVecPtr[5]);
                 acc0f2                  =   _dmpysp(f2temp, acc0f2);
                 output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                 // 46 and 64
                 f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                 acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[4]), f2temp);
                 f2temp                  =   _amem8_f2(&steeringVecPtr[6]);
                 acc0f2                  =   _dmpysp(f2temp, acc0f2);
                 output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                 // 47 and 74
                 f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                 acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[4]), f2temp);
                 f2temp                  =   _amem8_f2(&steeringVecPtr[7]);
                 acc0f2                  =   _dmpysp(f2temp, acc0f2);
                 output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                 // 48 and 84
                 f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                 acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[4]), f2temp);
                 f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                 acc0f2                  =   _dmpysp(f2temp, acc0f2);
                 output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }

            rnIdxRec                    =   rnIdx;

            // for 49 to 58
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 49 and 94
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[4]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 4a and a4
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[4]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 4b and b4
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[4]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 55
                 rnIdx++;

                 // 56 and 65
                 f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                 acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[5]), f2temp);
                 f2temp                  =   _amem8_f2(&steeringVecPtr[6]);
                 acc0f2                  =   _dmpysp(f2temp, acc0f2);
                 output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                 // 57 and 75
                 f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                 acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[5]), f2temp);
                 f2temp                  =   _amem8_f2(&steeringVecPtr[7]);
                 acc0f2                  =   _dmpysp(f2temp, acc0f2);
                 output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                 // 58 and 85
                 f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                 acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[5]), f2temp);
                 f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                 acc0f2                  =   _dmpysp(f2temp, acc0f2);
                 output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }
            rnIdxRec                    =   rnIdx;

            // for 59 to 69
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 59 and 95
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[5]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 5a and a5
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[5]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 5b and b5
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[5]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 66
                rnIdx++;

                 // 67 and 76
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[6]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[7]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 68 and 86
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[6]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 69 and 96
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[6]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }
            rnIdxRec                    =   rnIdx;

            // for 6a to 8a
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 6a and a6
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[6]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 6b and b6
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[6]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 77
                rnIdx++;

                // 78 and 87
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[7]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[8]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 79 and 97
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[7]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 7a and a7
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[7]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 7b and b7
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[7]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 88
                rnIdx++;

                heatMapPtr[bankIdx * bankSize + elevIdx]   =   output1;
            }
            rnIdxRec                    =   rnIdx;

            // for 89 to end
            #ifdef _TMS320C6X
            #pragma MUST_ITERATE(20,,);
            #endif
            for (elevIdx = 0; elevIdx < procSize; elevIdx++ )
            {
                steeringVecPtr          =   (__float2_t *) &steeringVecInit[elevIdx * nRxAnt];
                rnIdx                   =   rnIdxRec;
                output1                 =   heatMapPtr[bankIdx * bankSize + elevIdx];

                // 89 and 98
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[8]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[9]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 8a and a8
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[8]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 8b and b8
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[8]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // skip 99
                rnIdx++;

                // 9a and a9
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[9]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[10]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // 9b and b9
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[9]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));

                // skip aa
                rnIdx++;

                // ab and ba
                f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecPtr[10]), f2temp);
                f2temp                  =   _amem8_f2(&steeringVecPtr[11]);
                acc0f2                  =   _dmpysp(f2temp, acc0f2);
                output1                 +=   (_hif2(acc0f2) - _lof2(acc0f2));


                output                  =   2.f * output1;

                result                  =   _rcpsp(output);
                result                  =   result * (2.f - output * result);
                result                  =   result * (2.f - output * result);

                if (!bfFlag)
                  result              =   output;
                heatMapPtr[bankIdx * bankSize + elevIdx]   =   result;
                if (maxVal < result)
                {
                    maxVal              =   result;
                    maxElevInd          =   bankIdx * bankSize + elevIdx;
                }
            }
            bankIdx++;
        }
	}
	else
	{
	    scratchOffset   =   0;
	    steeringVecInit =   (__float2_t *) &capon_handle->aeEstimation_handle->scratchPad[scratchOffset];
	    scratchOffset   =   scratchOffset + 2 * capon_handle->aeEstimation_handle->nRxAnt;

	    maxVal                                      =   0.f;
	    maxElevInd                                  =   0;
	    heatMapPtr                                  =   azimElevHeatMap;
	    steeringVecAzimInit                         =   (__float2_t *) &capon_handle->raHeatMap_handle->steeringVecAzim[azimuthIdx * nRxAnt];
	    steeringVecElevInit                         =   (__float2_t *) capon_handle->raHeatMap_handle->steeringVecElev;
		for (elevIdx = 0; elevIdx < numElevationBins; elevIdx++ )
		{
			for (i = 0; i < nRxAnt; i++ )
			{
				_amem8_f2(&steeringVecInit[i])		=	_complex_mpysp(_amem8_f2(steeringVecElevInit++), _amem8_f2(&steeringVecAzimInit[i]));	
			}
            rnIdx                           =   0;
            output                          =   0.f;
            for (i = 0; i < nRxAnt; i++)
            {
                output                      +=  _hif2(_amem8_f2(&invRnMatrices[rnIdx++]));
                for (j = i + 1; j < nRxAnt; j++)
                {
                    f2temp                  =   _amem8_f2(&invRnMatrices[rnIdx++]);
                    acc0f2                  =   _complex_conjugate_mpysp(_amem8_f2(&steeringVecInit[i]), f2temp);
                    acc0f2                  =   _complex_mpysp(_amem8_f2(&steeringVecInit[j]), acc0f2);
                    output                  +=  2.f * _hif2(acc0f2);
                }
            }
            result                  =   _rcpsp(output);
            result                  =   result * (2.f - output * result);
            result                  =   result * (2.f - output * result);

            if (!bfFlag)
                result              =   output;
			heatMapPtr[elevIdx]		=	result;
			if (maxVal < result)
			{
				maxVal				=	result;
				maxElevInd			=	elevIdx;
			}
		}
	}
	//TODO later: add multipeak search, as well as peak expansion
	//

	//reconstruct beam filter for detected (azimuthIdx, maxInd) pair, current single peak and no peak expansion
	for (i = 0; i < nRxAnt; i++ )
	{
		_amem8_f2(&steeringVecInit[i])		=	_complex_mpysp(_amem8_f2(&capon_handle->raHeatMap_handle->steeringVecElev[maxElevInd * nRxAnt + i]), _amem8_f2(&steeringVecAzimInit[i]));	
	}
	beamFilterPtr							=	(__float2_t *) beamFilter;
    rnIdx								=   0;
    for (i = 0; i < nRxAnt; i++)
    {
        rnIdx1							=	i;
        acc0f2							=	_complex_mpysp(_amem8_f2(&steeringVecInit[i]), _amem8_f2(&invRnMatrices[rnIdx++]));
        k								=	nRxAnt - 1;
        for (j = 0; j < i; j++)
        {
            acc0f2						=	_daddsp(acc0f2, _complex_conjugate_mpysp(_amem8_f2(&invRnMatrices[rnIdx1]), _amem8_f2(&steeringVecInit[j])));
            rnIdx1						=	rnIdx1 + k;
            k--;
        }

        for (j = i + 1; j < nRxAnt; j++)
        {
            acc0f2						=	_daddsp(acc0f2, _complex_mpysp(_amem8_f2(&steeringVecInit[j]), _amem8_f2(&invRnMatrices[rnIdx++])));
        }

        _amem8_f2(beamFilterPtr++)		=	acc0f2;
    }
	numAngleOut							=	1;

	elevIdxLeft							=	maxElevInd - 1;
	if (elevIdxLeft < 0)	
		elevIdxLeft						=	0;
	elevIdxRight						=	maxElevInd + 1;
	if (elevIdxRight >= numElevationBins)
		elevIdxRight					=	numElevationBins - 1;
	muIdxInterp							=	((float)elevIdxLeft * heatMapPtr[elevIdxLeft] + (float)maxElevInd * heatMapPtr[maxElevInd] + (float)elevIdxRight * heatMapPtr[elevIdxRight])
											/(heatMapPtr[elevIdxLeft] + heatMapPtr[maxElevInd] + heatMapPtr[elevIdxRight]);

	elevEst[0]							=	(float)asinsp_i(capon_handle->raHeatMap_handle->muInit + capon_handle->raHeatMap_handle->muStep * muIdxInterp);
	tempAzim							=	divsp_i((capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * azimuthIdx), (float)cossp_i(elevEst[0]));
	if (_fabs(tempAzim) < 1.f)
	{
		azimEst[0]						=	(float)asinsp_i(tempAzim);
		peakPow[0]						=	maxVal;
		numAngleOut						=	1;
	}
	else
		numAngleOut						=	0;

	return (numAngleOut);
}

/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_aeEstElevAzim
 *
 *   \brief   Use Capon beamforming to estimation azimuth and elevation angle in 2D domain.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    azimuthIdx
 *               Input azimuth index for a detected point. 
 *
 *   \param[in]    capon_handle
 *               Input capon beamforming handle. 
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    elevEst
 *               Output elevation estimations, length of numAngleOut are valid.
 *
 *   \param[out]    peakPow
 *               Output peak power estimations, length of numAngleOut are valid.
 *
 *   \param[out]    beamFilter
 *               Output beamFilter estimations, in complex float, length of numAngleOut * aeEstimation_handle->nRxAnt are valid.
 *
 *   \param[out]    azimElevHeatMap
 *               Output azim-elev heatMap estimations.
 *
 *   \ret       numAngleOut: Number of estimated angle pairs.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
int32_t RADARDEMO_aoaEst2DCaponBF_aeEstElevAzim(
				IN uint8_t bfFlag,
				IN uint16_t azimuthIdx,
				IN RADARDEMO_aoaEst2DCaponBF_handle   * capon_handle,
				IN cplxf_t * RESTRICT invRnMatrices,
				OUT float  * RESTRICT azimEst, 
				OUT float  * RESTRICT elevEst, 
				OUT float  * RESTRICT peakPow, 
				OUT cplxf_t  * RESTRICT beamFilter, 
				OUT float    * RESTRICT azimElevHeatMap
			)
{
	int32_t		i, j, elevIdx, azimIdx, rnIdx, scratchOffset, nRxAnt, numAzimuthBins, numElevationBins, maxElevInd, maxAzimInd, numAngleOut;
	__float2_t	* RESTRICT steeringVecAzimInit;
	__float2_t	* RESTRICT steeringVecElevInit;
	__float2_t	* RESTRICT steeringVec;
	__float2_t	* RESTRICT beamFilterPtr;
	//__float2_t	* RESTRICT invRnMatPtr;
	__float2_t	* RESTRICT RnInv;
	float		* RESTRICT heatMapPtr;
	__float2_t  f2temp, acc0f2, acc1f2;
	float		output, result, maxVal, tempAzim;

	scratchOffset	=	0;
	RnInv			=	(__float2_t *) &capon_handle->aeEstimation_handle->scratchPad[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * capon_handle->aeEstimation_handle->nRxAnt * capon_handle->aeEstimation_handle->nRxAnt;  /*Store the full RnInv matrix*/
	steeringVec		=	(__float2_t *) &capon_handle->aeEstimation_handle->scratchPad[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * capon_handle->aeEstimation_handle->nRxAnt;

	nRxAnt				=	capon_handle->nRxAnt;
	numAzimuthBins		=	capon_handle->raHeatMap_handle->azimSearchLen;
	numElevationBins	=	capon_handle->raHeatMap_handle->elevSearchLen;

	rnIdx		=	0;
	for (i = 0; i < nRxAnt; i++)
	{
		_amem8_f2(&RnInv[i * nRxAnt + i])		=	_amem8_f2(&invRnMatrices[rnIdx++]);
		for (j = 0; j < i; j++)
		{
			f2temp								=	_amem8_f2(&invRnMatrices[rnIdx++]);
			_amem8_f2(&RnInv[i * nRxAnt + j])	=	f2temp;
			_amem8_f2(&RnInv[j * nRxAnt + i])	=	_ftof2(_hif2(f2temp), -_lof2(f2temp));
		}
	}

	maxVal										=	0.f;
	maxElevInd									=	0;
	maxAzimInd									=	0;
	heatMapPtr									=	azimElevHeatMap;


	for (elevIdx = 0; elevIdx < numElevationBins; elevIdx++ )
	{
		steeringVecAzimInit							=	(__float2_t *) capon_handle->raHeatMap_handle->steeringVecAzim;
		steeringVecElevInit							=	(__float2_t *) &capon_handle->raHeatMap_handle->steeringVecElev[numElevationBins * nRxAnt];

		for (azimIdx = 0; azimIdx < numAzimuthBins; azimIdx++ )
		{
			for (i = 0; i < nRxAnt; i++ )
			{
				_amem8_f2(&steeringVec[i])	=	_complex_mpysp(_amem8_f2(&steeringVecElevInit[i]), _amem8_f2(steeringVecAzimInit++));	
			}
			
			acc1f2								=	_ftof2(0.f, 0.f);
			for (i = 0; i < nRxAnt; i++)
			{
				acc0f2								=	_ftof2(0.f, 0.f);

				for (j = 0; j < nRxAnt; j++)
				{
					acc0f2							=	_daddsp(acc0f2, _complex_conjugate_mpysp(_amem8_f2(&steeringVec[j]), _amem8_f2(&invRnMatrices[i * nRxAnt + j])));
				}
				acc1f2								=	_daddsp(acc1f2, _complex_conjugate_mpysp(acc0f2, _amem8_f2(&steeringVec[j])));
			}
			output					=	_hif2(acc1f2);
			result					=	_rcpsp(output);
			result					=	result * (2.f - output * result);
			result					=	result * (2.f - output * result);

			if (!bfFlag)	
				result				=	_hif2(acc1f2);
			*heatMapPtr++			=	result;
			if (maxVal < result)
			{
				maxVal				=	result;
				maxElevInd			=	elevIdx;
				maxAzimInd			=	azimIdx;
			}
		}
	}
	//TODO later: add multipeak search, as well as peak expansion
	//

	//reconstruct beam filter for detected (azimuthIdx, maxInd) pair, current single peak and no peak expansion
	for (i = 0; i < nRxAnt; i++ )
	{
		_amem8_f2(&steeringVec[i])	=	_complex_mpysp(_amem8_f2(&capon_handle->raHeatMap_handle->steeringVecAzim[maxAzimInd * nRxAnt + i]), _amem8_f2(&capon_handle->raHeatMap_handle->steeringVecElev[maxElevInd * nRxAnt + i]));	
	}
	beamFilterPtr							=	(__float2_t *) beamFilter;
	for (i = 0; i < nRxAnt; i++)
	{
		acc0f2								=	_ftof2(0.f, 0.f);

		for (j = 0; j < nRxAnt; j++)
		{
			acc0f2							=	_daddsp(acc0f2, _complex_conjugate_mpysp(_amem8_f2(&steeringVec[j]), _amem8_f2(&invRnMatrices[i * nRxAnt + j])));
		}
		_amem8_f2(beamFilterPtr++)			=	acc0f2;
	}
	elevEst[0]								=	(float)asinsp_i(capon_handle->raHeatMap_handle->muInit + capon_handle->raHeatMap_handle->muStep * maxElevInd);
	tempAzim								=	divsp_i((capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * maxAzimInd), (float)cossp_i(elevEst[0]));
	if (_fabs(tempAzim) < 1.f)
	{
		azimEst[0]								=	(float)asinsp_i(tempAzim);
		peakPow[0]								=	maxVal;
		numAngleOut								=	1;
	}
	else
		numAngleOut								=	0;

	return (numAngleOut);
}


/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_aeEstZoomin
 *
 *   \brief   Use Capon beamforming to estimation azimuth and elevation angle in 2D domain.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    azimuthIdx
 *               Input coarse estimation of azimuth index for a detected point. 
 *
 *   \param[in]    elevationIdx
 *               Input coarse estimation of elevation index for a detected point. 
 *
 *   \param[in]    noise
 *               Input noise estimation for a detected point.  
 *
 *   \param[in]    aeEstimation_handle
 *               Input azimuth elevation estimation handle. 
 *
 *   \param[in]    raHeatMap_handle
 *               Input range angle heatmap estimation handle. 
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    peakPow
 *               Output peak power estimations, length of numAngleOut are valid.
 *
 *   \param[out]    beamFilter
 *               Output beamFilter estimations, in complex float, length of numAngleOut * aeEstimation_handle->nRxAnt are valid.
 *
 *   \ret       numAngleOut: Number of estimated angle pairs.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
int32_t RADARDEMO_aoaEst2DCaponBF_aeEstZoomin(
				IN uint8_t bfFlag,
				IN uint16_t azimuthIdx,
				IN uint16_t elevationIdx,
				IN float noise,
				IN RADARDEMO_aoaEst2D_aeEst_handle   * aeEstimation_handle,
				IN RADARDEMO_aoaEst2DCaponBF_handle   * capon_handle,
				IN cplxf_t * RESTRICT invRnMatrices,
				OUT float  * RESTRICT azimEst, 
				OUT float  * RESTRICT elevEst, 
				OUT float  * RESTRICT peakPow, 
				OUT cplxf_t  * RESTRICT beamFilter
			)
{
	int32_t		i, j, k, elevIdx, azimIdx, rnIdx, rnIdx1, scratchOffset, nRxAnt, numAzimuthBins, numElevationBins, maxElevInd, maxAzimInd, numAngleOut;
	__float2_t	* RESTRICT steeringVecAzimInit;
	__float2_t	* RESTRICT steeringVecAzimStep;
	__float2_t	* RESTRICT steeringVecElevInit;
	__float2_t	* RESTRICT steeringVecElevStep;
	__float2_t	* RESTRICT steeringVecInit;
	__float2_t	* RESTRICT steeringVecCopy;
	__float2_t	* RESTRICT steeringVec;
	__float2_t	* RESTRICT beamFilterPtr;
	float		* RESTRICT azimElevHeatMap;
	//__float2_t	* RESTRICT RnInv;
	float		* RESTRICT heatMapPtr;
	__float2_t  f2temp, acc0f2;
	float		output, result, maxVal, minVal, tempAzim, tempElev, sharpness;

	numAngleOut		=	0;

	scratchOffset	=	0;
	steeringVecInit =	(__float2_t *) &aeEstimation_handle->scratchPad[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * aeEstimation_handle->nRxAnt;
	steeringVec		=	(__float2_t *) &aeEstimation_handle->scratchPad[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * aeEstimation_handle->nRxAnt;
	steeringVecCopy =	(__float2_t *) &aeEstimation_handle->scratchPad[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * aeEstimation_handle->nRxAnt;
	azimElevHeatMap	=	(float *)&aeEstimation_handle->scratchPad[scratchOffset];
	scratchOffset	=	scratchOffset + aeEstimation_handle->azimSearchLen * aeEstimation_handle->elevSearchLen;

	nRxAnt				=	aeEstimation_handle->nRxAnt;
	numAzimuthBins		=	aeEstimation_handle->azimSearchLen;
	numElevationBins	=	aeEstimation_handle->elevSearchLen;

	minVal										=	1.0e25;
	maxVal										=	0.f;
	maxElevInd									=	0;
	maxAzimInd									=	0;
	heatMapPtr									=	azimElevHeatMap;
	steeringVecAzimInit							=	(__float2_t *) aeEstimation_handle->steeringVecAzimInit;
	steeringVecAzimStep							=	(__float2_t *) aeEstimation_handle->steeringVecAzimStep;
	steeringVecElevInit							=	(__float2_t *) aeEstimation_handle->steeringVecElevInit;
	steeringVecElevStep							=	(__float2_t *) aeEstimation_handle->steeringVecElevStep;

	// prepare steering vector starting values for the azimuthIdx(th) azimuth bin
	for (i = 0; i < nRxAnt; i++ )
	{
		_amem8_f2(&steeringVecInit[i])		    =	_complex_mpysp(_amem8_f2(&steeringVecAzimInit[i]), _amem8_f2(&capon_handle->raHeatMap_handle->steeringVec[(elevationIdx * capon_handle->raHeatMap_handle->azimSearchLen + azimuthIdx) * nRxAnt + i]));
	}

	for (azimIdx = 0; azimIdx < numAzimuthBins; azimIdx++ )
	{
		for (i = 0; i < nRxAnt; i++ )
		{
			_amem8_f2(&steeringVec[i])	=	_complex_mpysp(_amem8_f2(&steeringVecElevInit[i]), _amem8_f2(&steeringVecInit[i]));	
		}
		for (elevIdx = 0; elevIdx < numElevationBins; elevIdx++ )
		{
			rnIdx				=	0;
			output				=	0.f;
			for (i = 0; i < nRxAnt; i++)
			{
				output			+=	_hif2(_amem8_f2(&invRnMatrices[rnIdx++]));
				for (j = i + 1; j < nRxAnt; j++)
				{
					f2temp		=	_amem8_f2(&invRnMatrices[rnIdx++]);
					acc0f2		=	_complex_conjugate_mpysp(_amem8_f2(&steeringVec[i]), f2temp);
					acc0f2		=	_complex_mpysp(_amem8_f2(&steeringVec[j]), acc0f2);
					output		+=	2.f * _hif2(acc0f2);
				}
			}
			result				=	_rcpsp(output);
			result				=	result * (2.f - output * result);
			result				=	result * (2.f - output * result);

			if (!bfFlag)	
				result			=	output;
			*heatMapPtr++		=	result;
			if (minVal > result)
				minVal			=	result;	
			if (maxVal < result)
			{
				maxVal			=	result;
				maxElevInd		=	elevIdx;
				maxAzimInd		=	azimIdx;
				for (i = 0; i < nRxAnt; i++ )
				{
					_amem8_f2(&steeringVecCopy[i])	=	_amem8_f2(&steeringVec[i]);	
				}
			}
			for (i = 0; i < nRxAnt; i++ )
			{
				_amem8_f2(&steeringVec[i])	=	_complex_mpysp(_amem8_f2(&steeringVecElevStep[i]), _amem8_f2(&steeringVec[i]));	
			}
		}
		for (i = 0; i < nRxAnt; i++ )
		{
			_amem8_f2(&steeringVecInit[i])	=	_complex_mpysp(_amem8_f2(&steeringVecAzimStep[i]), _amem8_f2(&steeringVecInit[i]));	
		}
	}


	//peak expansion, only when the peak SNR exceeding threshold 
	if ( ( aeEstimation_handle->peakExpSamples > 0 ) && ( maxVal * _rcpsp(noise) > aeEstimation_handle->peakExpSNRThr) )
	{
		float		pExpThr;
		int32_t		startElev, endElev, startAzim, endAzim, antIdx;
		
		beamFilterPtr						=	(__float2_t *) beamFilter;

		startAzim							=	maxAzimInd - aeEstimation_handle->peakExpSamples;
		if ( startAzim < 0 )
			startAzim						=	0;
		endAzim								=	maxAzimInd + aeEstimation_handle->peakExpSamples;
		if ( endAzim >= (int32_t) aeEstimation_handle->azimSearchLen )
			endAzim							=	aeEstimation_handle->azimSearchLen - 1;
		startElev							=	maxElevInd - aeEstimation_handle->peakExpSamples;
		if ( startElev < 0 )
			startElev						=	0;
		endElev								=	maxElevInd + aeEstimation_handle->peakExpSamples;
		if ( endElev >= (int32_t) aeEstimation_handle->elevSearchLen )
			endElev							=	aeEstimation_handle->elevSearchLen - 1;

		sharpness							=	(maxVal - minVal) * _rcpsp(maxVal + minVal);
		pExpThr								=	maxVal * (aeEstimation_handle->peakExpRelThr - sharpness);
		for (azimIdx = 0; azimIdx < (maxAzimInd - startAzim); azimIdx++ )
		{
			for (antIdx = 0; antIdx < nRxAnt; antIdx++ )
			{
				_amem8_f2(&steeringVecInit[antIdx])	=	_complex_conjugate_mpysp(_amem8_f2(&steeringVecAzimStep[antIdx]), _amem8_f2(&steeringVecCopy[antIdx]));	
			}
		}


		for (azimIdx = startAzim; azimIdx <= endAzim; azimIdx++)
		{
			for (elevIdx = 0; elevIdx < (maxElevInd - startElev); elevIdx++ )
			{
				for (antIdx = 0; antIdx < nRxAnt; antIdx++ )
				{
					_amem8_f2(&steeringVec[antIdx])	=	_complex_conjugate_mpysp(_amem8_f2(&steeringVecElevStep[antIdx]), _amem8_f2(&steeringVecInit[antIdx]));	
				}
			}
			for (elevIdx = startElev; elevIdx <= endElev; elevIdx++)
			{
				result									=	azimElevHeatMap[azimIdx * aeEstimation_handle->elevSearchLen + elevIdx];
				if (result > pExpThr)
				{

					tempElev							=	(float)asinsp_i(aeEstimation_handle->muInit + aeEstimation_handle->muStep * elevIdx + capon_handle->raHeatMap_handle->muInit + capon_handle->raHeatMap_handle->muStep * elevationIdx);
					tempAzim							=	divsp_i((aeEstimation_handle->nuInit + aeEstimation_handle->nuStep * azimIdx + capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * azimuthIdx), (float)cossp_i(tempElev));
					//if (_fabs(tempAzim) < 1.0f)
					if (_fabs(tempAzim) < _fabs(capon_handle->raHeatMap_handle->nuInit -  capon_handle->raHeatMap_handle->nuStep))
					{
						//limit the azim estimation to the FoV
						//if ( tempAzim < capon_handle->raHeatMap_handle->nuInit -  capon_handle->raHeatMap_handle->nuStep )
						//	tempAzim					=	capon_handle->raHeatMap_handle->nuInit -  capon_handle->raHeatMap_handle->nuStep;
						//if ( tempAzim > capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * capon_handle->raHeatMap_handle->azimSearchLen)
						//	tempAzim					=	capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * capon_handle->raHeatMap_handle->azimSearchLen;

						elevEst[numAngleOut]			=	tempElev; 
						azimEst[numAngleOut]			=	(float)asinsp_i(tempAzim);
						peakPow[numAngleOut]			=	result;

						rnIdx								=   0;
						for (i = 0; i < nRxAnt; i++)
						{
							rnIdx1						=	i;
							acc0f2						=	_complex_mpysp(_amem8_f2(&steeringVec[i]), _amem8_f2(&invRnMatrices[rnIdx++]));
							k							=	nRxAnt - 1;
							for (j = 0; j < i; j++)
							{
								acc0f2					=	_daddsp(acc0f2, _complex_conjugate_mpysp(_amem8_f2(&invRnMatrices[rnIdx1]), _amem8_f2(&steeringVec[j])));
								rnIdx1					=	rnIdx1 + k;
								k--;
							}

							for (j = i + 1; j < nRxAnt; j++)
							{
								acc0f2					=	_daddsp(acc0f2, _complex_mpysp(_amem8_f2(&steeringVec[j]), _amem8_f2(&invRnMatrices[rnIdx++])));
							}

							_amem8_f2(beamFilterPtr++)	=	acc0f2;
						}
						numAngleOut						+=	1;
					}
				}
				for (antIdx = 0; antIdx < nRxAnt; antIdx++ )
				{
					_amem8_f2(&steeringVec[antIdx])	=	_complex_mpysp(_amem8_f2(&steeringVecElevStep[antIdx]), _amem8_f2(&steeringVec[antIdx]));	
				}
			}
			for (antIdx = 0; antIdx < nRxAnt; antIdx++ )
			{
				_amem8_f2(&steeringVecInit[antIdx])	=	_complex_mpysp(_amem8_f2(&steeringVecAzimStep[antIdx]), _amem8_f2(&steeringVecInit[antIdx]));	
			}
		}
	}
	else
	{
		beamFilterPtr						=	(__float2_t *) beamFilter;
		tempElev							=	(float)asinsp_i(aeEstimation_handle->muInit + aeEstimation_handle->muStep * maxElevInd + capon_handle->raHeatMap_handle->muInit + capon_handle->raHeatMap_handle->muStep * elevationIdx);
		tempAzim							=	divsp_i((aeEstimation_handle->nuInit + aeEstimation_handle->nuStep * maxAzimInd + capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * azimuthIdx), (float)cossp_i(tempElev));
		//if (_fabs(tempAzim) < 1.0f)
		if (_fabs(tempAzim) < _fabs(capon_handle->raHeatMap_handle->nuInit -  capon_handle->raHeatMap_handle->nuStep))
		{
			//limit the azim estimation to the FoV
			//if ( tempAzim < capon_handle->raHeatMap_handle->nuInit -  capon_handle->raHeatMap_handle->nuStep )
			//	tempAzim					=	capon_handle->raHeatMap_handle->nuInit -  capon_handle->raHeatMap_handle->nuStep;
			//if ( tempAzim > capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * capon_handle->raHeatMap_handle->azimSearchLen)
			//	tempAzim					=	capon_handle->raHeatMap_handle->nuInit + capon_handle->raHeatMap_handle->nuStep * capon_handle->raHeatMap_handle->azimSearchLen;

			elevEst[numAngleOut]			=	tempElev; 
			azimEst[numAngleOut]			=	(float)asinsp_i(tempAzim);
			peakPow[numAngleOut]			=	maxVal;

			rnIdx								=   0;
			for (i = 0; i < nRxAnt; i++)
			{
				rnIdx1							=	i;
				acc0f2							=	_complex_mpysp(_amem8_f2(&steeringVecCopy[i]), _amem8_f2(&invRnMatrices[rnIdx++]));
				k								=	nRxAnt - 1;
				for (j = 0; j < i; j++)
				{
					acc0f2						=	_daddsp(acc0f2, _complex_conjugate_mpysp(_amem8_f2(&invRnMatrices[rnIdx1]), _amem8_f2(&steeringVecCopy[j])));
					rnIdx1						=	rnIdx1 + k;
					k--;
				}

				for (j = i + 1; j < nRxAnt; j++)
				{
					acc0f2						=	_daddsp(acc0f2, _complex_mpysp(_amem8_f2(&steeringVecCopy[j]), _amem8_f2(&invRnMatrices[rnIdx++])));
				}

				_amem8_f2(beamFilterPtr++)		=	acc0f2;
			}
			numAngleOut						+=	1;
		}
	}

	return (numAngleOut);
}

