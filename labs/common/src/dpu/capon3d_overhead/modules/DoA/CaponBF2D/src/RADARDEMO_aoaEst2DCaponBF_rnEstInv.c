
/**
 *  \file   RADARDEMO_aoaEst2DCaponBF_rnEstInv.c
 *
 *   \brief   Estimate the Rn matrix and find inversion for 2D BF.
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
 *   \fn     RADARDEMO_aoaEst2DCaponBF_covInv
 *
 *   \brief   Per range bin, estimate the covariance matrices from input 1D FFT results, and calculate the inverse of these matrices.
 *
 *   \param[in]    invFlag
 *               Flag to indicate matrix inversion will be performed. 
 *               If set to 1, output invRnMatrices will contain inversion of covariance matrices.
 *               If set to 0, output invRnMatrices will contain covariance matrices without inversion.
 *
 *   \param[in]    gamma
 *               Scaling factor for diagnal loading.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size of nRxAnt * nRxAnt * 2 * 4 bytes.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    virtAntInd2Proc
 *               Input array that defines the antennas need to be processed, contains indices out of the full virtual antenna array indices.
 *
 *   \param[in]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    invRnMatrices
 *               Output inverse of covariance matrices for the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
void		RADARDEMO_aoaEst2DCaponBF_covInv(
				IN uint8_t invFlag,
				IN float gamma,
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN int32_t * scratch,
				IN uint8_t * virtAntInd2Proc,
				IN cplx16_t * inputAntSamples,
				OUT cplxf_t  * invRnMatrices)
{
	int32_t		antIdx, chirpIdx, i, j, rnIdx, scratchOffset;
	cplx16_t	* RESTRICT input1;
	cplx16_t	* RESTRICT input2;
	__float2_t     * RESTRICT Rn;
	__float2_t     * RESTRICT RnInv;
	int64_t		lltemp, llinput1, llinput2;
	__float2_t     acc, acc1, acc2, acc3, scale2;
	int32_t       itemp1;
	//cplxf_t		* RESTRICT invRn;
	float       ftemp, diagSum;

#ifdef _TMS320C6X
	_nassert(nChirps % 8	== 0);
	_nassert(nRxAnt  % 4	== 0);
#endif

	scratchOffset	=	0;
	Rn			=	(__float2_t *) &scratch[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * nRxAnt * nRxAnt;  /*Store the full Rn matrix*/
	RnInv		=	(__float2_t *) &scratch[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * nRxAnt * nRxAnt;  /*Store the full RnInv matrix*/

	ftemp			=	_rcpsp((float)nChirps);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	
	scale2			=	_ftof2(ftemp, ftemp);

	/*Rn estimation */
	diagSum         =   0.f;
	for (antIdx = 0; antIdx < nRxAnt; antIdx++)
	{
		input1		=	(cplx16_t *) &inputAntSamples[virtAntInd2Proc[antIdx] * nChirps];

		//i = antIdx case -- diagonal elements
		acc			=	_ftof2(0.f, 0.f);
		acc1		=	_ftof2(0.f, 0.f);
		acc2		=	_ftof2(0.f, 0.f);
		acc3		=	_ftof2(0.f, 0.f);
		for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 8)
		{
			llinput1	=	_amem8(&input1[chirpIdx]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc			=	_daddsp(acc, _dintsp(lltemp));
			llinput1	=	_amem8(&input1[chirpIdx + 2]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc2		=	_daddsp(acc2, _dintsp(lltemp));
			
			llinput1	=	_amem8(&input1[chirpIdx + 4]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc1		=	_daddsp(acc1, _dintsp(lltemp));
			llinput1	=	_amem8(&input1[chirpIdx + 6]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc3		=	_daddsp(acc3, _dintsp(lltemp));
			
		}
		acc							=	_daddsp(acc, acc1);
		acc							=	_daddsp(acc, acc2);
		acc							=	_daddsp(acc, acc3);
		acc							=	_dmpysp(acc, scale2);
		_amem8_f2(&Rn[antIdx * nRxAnt + antIdx])	=	_ftof2(_hif2(acc), 0.f);
		diagSum                     +=   _hif2(acc);

		for (i = antIdx + 1; i < nRxAnt; i++)
		{
			input2		=	(cplx16_t *) &inputAntSamples[virtAntInd2Proc[i] * nChirps];

			acc			=	_ftof2(0.f, 0.f);
			acc1		=	_ftof2(0.f, 0.f);

            #ifdef _TMS320C6X
            #pragma UNROLL (2);
            #endif
			for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 4)
			{
				llinput1	=	_amem8(&input1[chirpIdx]);
				llinput2	=	_amem8(&input2[chirpIdx]);
				itemp1		=	_hill(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_cmpy(_hill(llinput1), itemp1);
				itemp1		=	_loll(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
				acc			=	_daddsp(acc, _dintsp(lltemp));
				llinput1	=	_amem8(&input1[chirpIdx + 2]);
				llinput2	=	_amem8(&input2[chirpIdx + 2]);
				itemp1		=	_hill(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_cmpy(_hill(llinput1), itemp1);
				itemp1		=	_loll(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
				acc1		=	_daddsp(acc1, _dintsp(lltemp));
			}
			acc							=	_daddsp(acc, acc1);
			acc							=	_dmpysp(acc, scale2);
			_amem8_f2(&Rn[i * nRxAnt + antIdx])		=	acc;
			_amem8_f2(&Rn[antIdx * nRxAnt + i])		=	_ftof2(_hif2(acc), -_lof2(acc));
		}
	}

	if (invFlag)
	{
	    cplxf_t     * RESTRICT tempPtr;

		if (nRxAnt == 8)
		    diagSum	*=	0.125f;
		else if (nRxAnt == 4)
		    diagSum	*=	0.25f;
		else if (nRxAnt == 12)
		    diagSum	*=	(1.f/12.f);
		else
		    diagSum	*=	_rcpsp((float)nRxAnt);

		diagSum		*=	gamma;
		acc			=	_ftof2(diagSum, 0.f);
		tempPtr     =   (cplxf_t     *) Rn;
		for (i = 0; i < nRxAnt; i++)
		{
			//_amem8_f2(&Rn[i * nRxAnt + i])	=	_daddsp(_amem8_f2(&Rn[i * nRxAnt + i]), acc);
		    tempPtr[i * nRxAnt + i].real        =   tempPtr[i * nRxAnt + i].real + diagSum;
		}


		/* matrix inversion */
		MATRIX_cholesky_flp_inv(
			(cplxf_t *) Rn, 
			(cplxf_t *) RnInv,
			nRxAnt
		);   

		// only output the upper triangle for memory savings
		rnIdx		=	0;
		for (i = 0; i < nRxAnt; i++)
		{
			_amem8_f2(&invRnMatrices[rnIdx++]) = _amem8_f2(&RnInv[i * nRxAnt + i]);
			for (j = i + 1; j < nRxAnt; j++)
			{
				_amem8_f2(&invRnMatrices[rnIdx++]) = _amem8_f2(&RnInv[i * nRxAnt + j]);
			}
		}
	}
	else
	{
		// only output the upper triangle for memory savings
		rnIdx		=	0;
		rnIdx		=	0;
		for (i = 0; i < nRxAnt; i++)
		{
			_amem8_f2(&invRnMatrices[rnIdx++]) = _amem8_f2(&RnInv[i * nRxAnt + i]);
			for (j = i + 1; j < nRxAnt; j++)
			{
				_amem8_f2(&invRnMatrices[rnIdx++]) = _amem8_f2(&RnInv[i * nRxAnt + j]);
			}
		}
	}
}

