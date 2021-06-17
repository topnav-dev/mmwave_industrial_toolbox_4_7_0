
/**
 *  \file   RADARDEMO_aoaEst2DCaponBF_staticRemoval.c
 *
 *   \brief   Estimate the angle of arrival using 2D BF.
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
 *   \fn     RADARDEMO_aoaEst2DCaponBF_clutterRemoval
 *
 *   \brief   Per range bin, removal static clutter from the input signal.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed. 
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    outputAntSamples
 *               output samples from after clutter removal for the current (one) range bin.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    static_information
 *               Zero Doppler antenna samples for the range bin.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
void		RADARDEMO_aoaEst2DCaponBF_clutterRemoval(
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN cplx16_t * inputAntSamples,
				OUT cplx16_t * outputAntSamples,
				OUT cplxf_t  * static_information)
{
#ifdef RADARDEMO_AOARADARCUDE_RNGCHIRPANT
	int32_t		antIdx, chirpIdx;
	cplx16_t	* RESTRICT input1;
	cplx16_t	* RESTRICT input2;
	cplx16_t	* RESTRICT input3;
	cplx16_t	* RESTRICT input4;
	int64_t		* RESTRICT output1;
	int64_t		* RESTRICT output2;
	int64_t		* RESTRICT output3;
	int64_t		* RESTRICT output4;
	int64_t		llinput1, llinput2;
	__float2_t     acc1, acc2, acc3, acc4, scale2;
	int32_t       itemp1,itemp2;
	float       ftemp;
	int64_t		mean12, mean34;
	int64_t		intAcc;

#ifdef _TMS320C6X
	_nassert(nRxAnt %4	==	0);
	_nassert(nChirps %8	==	0);
#endif

	ftemp			=	_rcpsp((float)nChirps);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	
	scale2			=	_ftof2(ftemp, ftemp);

	for (antIdx = 0; antIdx < nRxAnt; antIdx += 4)
	{
		input1		=	(cplx16_t *) &inputAntSamples[antIdx];
		input2		=	(cplx16_t *) &inputAntSamples[antIdx + nRxAnt * (nChirps >> 1)];
		input3		=	(cplx16_t *) &inputAntSamples[antIdx + 2];
		input4		=	(cplx16_t *) &inputAntSamples[(antIdx + 2)  + nRxAnt * (nChirps >> 1)];
		acc1		=	_ftof2(0.f, 0.f);
		acc2		=	_ftof2(0.f, 0.f);
		acc3		=	_ftof2(0.f, 0.f);
		acc4		=	_ftof2(0.f, 0.f);
		for (chirpIdx = 0; chirpIdx < nRxAnt * (nChirps >> 1); chirpIdx += nRxAnt)
		{
			intAcc	=	_dsadd2(_amem8(&input1[chirpIdx]), _amem8(&input2[chirpIdx]));
			acc1	=	_daddsp(acc1, _dinthsp(_loll(intAcc)));
			acc2	=	_daddsp(acc2, _dinthsp(_hill(intAcc)));
			intAcc	=	_dsadd2(_amem8(&input3[chirpIdx]), _amem8(&input4[chirpIdx]));
			acc3	=	_daddsp(acc3, _dinthsp(_loll(intAcc)));
			acc4	=	_daddsp(acc4, _dinthsp(_hill(intAcc)));
		}

		acc1		=	_dmpysp(acc1, scale2);
		_amem8_f2(static_information++)	=	acc1;
		acc2		=	_dmpysp(acc2, scale2);
		_amem8_f2(static_information++)	=	acc2;
		acc3		=	_dmpysp(acc3, scale2);
		_amem8_f2(static_information++)	=	acc3;
		acc4		=	_dmpysp(acc4, scale2);
		_amem8_f2(static_information++)	=	acc4;

		itemp1		=	_dspinth(acc1);
		itemp2		=	_dspinth(acc2);
		mean12		=	_itoll(itemp2,itemp1);
		itemp1		=	_dspinth(acc3);
		itemp2		=	_dspinth(acc4);
		mean34		=	_itoll(itemp2,itemp1);

		input1		=	(cplx16_t *) &inputAntSamples[antIdx];
		input2		=	(cplx16_t *) &inputAntSamples[antIdx + nRxAnt];
		output1		=	(int64_t *)&outputAntSamples[antIdx * nChirps];
		output2		=	(int64_t *)&outputAntSamples[(antIdx + 1) * nChirps];
		output3		=	(int64_t *)&outputAntSamples[(antIdx + 2) * nChirps];
		output4		=	(int64_t *)&outputAntSamples[(antIdx + 3) * nChirps];
		for (chirpIdx = 0; chirpIdx < nRxAnt * nChirps ; chirpIdx += 2 * nRxAnt)
		{
			llinput1	=	_dssub2(_amem8(&input1[chirpIdx]), mean12);
			llinput2	=	_dssub2(_amem8(&input2[chirpIdx]), mean12);
			_amem8(output1++)	=	_itoll(_loll(llinput2), _loll(llinput1));
			_amem8(output2++)	=	_itoll(_hill(llinput2), _hill(llinput1));

			llinput1	=	_dssub2(_amem8(&input1[chirpIdx+2]), mean34);
			llinput2	=	_dssub2(_amem8(&input2[chirpIdx+2]), mean34);
			_amem8(output3++)	=	_itoll(_loll(llinput2), _loll(llinput1));
			_amem8(output4++)	=	_itoll(_hill(llinput2), _hill(llinput1));
		}
	}
#endif


#ifdef RADARDEMO_AOARADARCUDE_RNGANTCHIRP
	int32_t		antIdx, chirpIdx;
	cplx16_t	* RESTRICT input1;
	cplx16_t	* RESTRICT output;
	int64_t		llinput1;
	__float2_t     acc, acc1, scale2;
	int32_t       itemp1;
	float       ftemp;
	int64_t		mean2;
	int64_t		intAcc;

#ifdef _TMS320C6X
	_nassert(nRxAnt %4	==	0);
	_nassert(nChirps %8	==	0);
#endif
	ftemp			=	_rcpsp((float)nChirps);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	
	scale2			=	_ftof2(ftemp, ftemp);

	for (antIdx = 0; antIdx < nRxAnt; antIdx++)
	{
		input1		=	(cplx16_t *) &inputAntSamples[antIdx * nChirps];
		acc			=	_ftof2(0.f, 0.f);
		acc1		=	_ftof2(0.f, 0.f);
		for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 8)
		{
			intAcc	=	_dsadd2(_amem8(&input1[chirpIdx]), _amem8(&input1[chirpIdx+2]));
			itemp1	=	_sadd2(_hill(intAcc), _loll(intAcc));
			acc		=	_daddsp(acc, _dinthsp(itemp1));
			intAcc	=	_dsadd2(_amem8(&input1[chirpIdx+4]), _amem8(&input1[chirpIdx+6]));
			itemp1	=	_sadd2(_hill(intAcc), _loll(intAcc));
			acc1	=	_daddsp(acc1, _dinthsp(itemp1));
		}
		acc			=	_daddsp(acc, acc1);
			
		acc			=	_dmpysp(acc, scale2);
		_amem8_f2(static_information)	=	acc;
		static_information++;
		itemp1		=	_dspinth(acc);
		mean2		=	_itoll(itemp1,itemp1);
		output		=	(cplx16_t *)outputAntSamples;
		for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 4)
		{
			llinput1	=	_amem8(&input1[chirpIdx]);
			_amem8(&output[chirpIdx])	=	_dssub2(llinput1, mean2);
			llinput1	=	_amem8(&input1[chirpIdx + 2]);
			_amem8(&output[chirpIdx + 2])	=	_dssub2(llinput1, mean2);
		}
	}
#endif


}
