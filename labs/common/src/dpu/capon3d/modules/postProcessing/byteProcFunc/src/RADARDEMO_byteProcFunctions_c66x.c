/*! 
 *  \file   RADARDEMO_byteProcFunctions_c66x.c
 *
 *  \brief   C66x optimized code for byte processing. 
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

#include <modules/postProcessing/byteProcFunc/api/RADARDEMO_byteProcFunctions.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x


/*!
   \fn     RADARDEMO_convKernel_c66x

   \brief   2D convolution kernel optimized for C66x. 
   
   \param[in]    sizeX
               X dimension.

   \param[in]    sizeY
               Y dimension.

   \param[in]   inputPtr
               Input 2D grid. Must be aligned to 8-byte boundary.

   \param[in]   inputFilt
               Input 2D fitler, assuming always 5x5 size (hardcoded) and assuming symmetry.

   \param[out]    outputPtr
               Output 2D grid. Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_convKernel_c66x(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							IN  uint8_t * inputFilt,
							OUT uint8_t * outputPtr)
{
	int32_t i, j, ii, jj, iistart, iiend, jjstart, jjend;
	int32_t sum, temp1, temp2, temp3, temp4, temp5, iSample;
	int32_t guard = 2; /* hard coded of (5 >> 2)*/
	int32_t * RESTRICT input1;
	int32_t * RESTRICT input2;
	int32_t * RESTRICT input3;
	int32_t * RESTRICT input4;
	int32_t * RESTRICT input5;
	int8_t * RESTRICT output;
	int64_t llrightoneout, llsample1, llsample2, llsample3, llsample4, llsample5;
	int32_t sum4, sum3, sum2, sum1; 
	uint64_t llfilt1, llfilt2, llfilt3, llfilt4, llfilt5;

	/*prepare filter coef in order of 4 4 4 4 3 2 1 0 in uint64_t register */
	llfilt1		=	_mem8(&inputFilt[0]);
	llfilt2		=	_mem8(&inputFilt[5]);
	llfilt3		=	_mem8(&inputFilt[10]);
	llfilt4		=	_mem8(&inputFilt[15]);
	llfilt5		=	_mem8(&inputFilt[20]);

	temp1		=	_extu(_hill(llfilt1), 24, 24);
	temp1		=	(temp1 << 8) | temp1;
	temp1		=	_pack2(temp1, temp1);
	llfilt1		=	_itoll(temp1, _loll(llfilt1));

	temp1		=	_extu(_hill(llfilt2), 24, 24);
	temp1		=	(temp1 << 8) | temp1;
	temp1		=	_pack2(temp1, temp1);
	llfilt2		=	_itoll(temp1, _loll(llfilt2));

	temp1		=	_extu(_hill(llfilt3), 24, 24);
	temp1		=	(temp1 << 8) | temp1;
	temp1		=	_pack2(temp1, temp1);
	llfilt3		=	_itoll(temp1, _loll(llfilt3));

	temp1		=	_extu(_hill(llfilt4), 24, 24);
	temp1		=	(temp1 << 8) | temp1;
	temp1		=	_pack2(temp1, temp1);
	llfilt4		=	_itoll(temp1, _loll(llfilt4));

	temp1		=	_extu(_hill(llfilt5), 24, 24);
	temp1		=	(temp1 << 8) | temp1;
	temp1		=	_pack2(temp1, temp1);
	llfilt5		=	_itoll(temp1, _loll(llfilt5));

	/*edge processing */
	/*row 0 middle */
	input3 = (int32_t *) &inputPtr[0*sizeY+0];
	input4 = (int32_t *) &inputPtr[1*sizeY+0];
	input5 = (int32_t *) &inputPtr[2*sizeY+0];
	output = (int8_t *) &outputPtr[0*sizeY+2];
	for (j = 2; j < (int32_t)sizeY - 2; j+=4)
	{
		llsample3		=	_mem8(input3++);
		llsample4		=	_mem8(input4++);
		llsample5		=	_mem8(input5++);

		llrightoneout	=	_mpyu4ll(_hill(llsample3), _hill(llfilt3));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample4), _hill(llfilt4)));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample5), _hill(llfilt5)));

		sum1			=	_extu(_loll(llrightoneout), 16, 16);
		sum1			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum1			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum1			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		*output++		=	sum1 >> 8;

		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		llsample5		=	llsample5 >> 8;
		sum2			=	_extu(_loll(llrightoneout), 0, 16);
		sum2			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum2			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum2			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		*output++		=	sum2 >> 8;

		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		llsample5		=	llsample5 >> 8;
		sum3			=	_extu(_hill(llrightoneout), 16, 16);
		sum3			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum3			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum3			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		*output++		=	sum3 >> 8;

		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		llsample5		=	llsample5 >> 8;
		sum4			=	_extu(_hill(llrightoneout), 0, 16);
		sum4			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum4			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum4			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		*output++		=	sum4 >> 8;
	}
	
	/*row 1 middle */
	input2 = (int32_t *) &inputPtr[0*sizeY+0];
	input3 = (int32_t *) &inputPtr[1*sizeY+0];
	input4 = (int32_t *) &inputPtr[2*sizeY+0];
	input5 = (int32_t *) &inputPtr[3*sizeY+0];
	output = (int8_t *) &outputPtr[1*sizeY+2];
	for (j = 2; j < (int32_t)sizeY - 2; j+=4)
	{
		llsample2		=	_mem8(input2++);
		llsample3		=	_mem8(input3++);
		llsample4		=	_mem8(input4++);
		llsample5		=	_mem8(input5++);

		llrightoneout	=	_mpyu4ll(_hill(llsample2), _hill(llfilt2));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample3), _hill(llfilt3)));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample4), _hill(llfilt4)));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample5), _hill(llfilt5)));

		sum1			=	_extu(_loll(llrightoneout), 16, 16);
		sum1			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum1			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum1			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum1			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		*output++		=	sum1 >> 8;

		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		llsample5		=	llsample5 >> 8;
		sum2			=	_extu(_loll(llrightoneout), 0, 16);
		sum2			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum2			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum2			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum2			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		*output++		=	sum2 >> 8;

		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		llsample5		=	llsample5 >> 8;
		sum3			=	_extu(_hill(llrightoneout), 16, 16);
		sum3			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum3			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum3			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum3			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		*output++		=	sum3 >> 8;

		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		llsample5		=	llsample5 >> 8;
		sum4			=	_extu(_hill(llrightoneout), 0, 16);
		sum4			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
		sum4			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum4			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum4			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		*output++		=	sum4 >> 8;
	}
	/*row N-2 middle */	
	input1 = (int32_t *) &inputPtr[(sizeX - 4)*sizeY+0];
	input2 = (int32_t *) &inputPtr[(sizeX - 3)*sizeY+0];
	input3 = (int32_t *) &inputPtr[(sizeX - 2)*sizeY+0];
	input4 = (int32_t *) &inputPtr[(sizeX - 1)*sizeY+0];
	output = (int8_t *) &outputPtr[(sizeX-2)*sizeY+2];
	for (j = 2; j < (int32_t)sizeY - 2; j+=4)
	{
		llsample1		=	_mem8(input1++);
		llsample2		=	_mem8(input2++);
		llsample3		=	_mem8(input3++);
		llsample4		=	_mem8(input4++);

		llrightoneout	=	_mpyu4ll(_hill(llsample1), _hill(llfilt1));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample2), _hill(llfilt2)));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample3), _hill(llfilt3)));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample4), _hill(llfilt4)));

		sum1			=	_extu(_loll(llrightoneout), 16, 16);
		sum1			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum1			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum1			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum1			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum1 >> 8;

		llsample1		=	llsample1 >> 8;
		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		sum2			=	_extu(_loll(llrightoneout), 0, 16);
		sum2			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum2			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum2			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum2			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum2 >> 8;

		llsample1		=	llsample1 >> 8;
		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		sum3			=	_extu(_hill(llrightoneout), 16, 16);
		sum3			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum3			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum3			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum3			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum3 >> 8;

		llsample1		=	llsample1 >> 8;
		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		llsample4		=	llsample4 >> 8;
		sum4			=	_extu(_hill(llrightoneout), 0, 16);
		sum4			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
		sum4			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum4			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum4			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum4 >> 8;
	}

	/*row N-1 middle */	
	input1 = (int32_t *) &inputPtr[(sizeX - 3)*sizeY+0];
	input2 = (int32_t *) &inputPtr[(sizeX - 2)*sizeY+0];
	input3 = (int32_t *) &inputPtr[(sizeX - 1)*sizeY+0];
	output = (int8_t *) &outputPtr[(sizeX-1)*sizeY+2];
	for (j = 2; j < (int32_t)sizeY - 2; j+=4)
	{
		llsample1		=	_mem8(input1++);
		llsample2		=	_mem8(input2++);
		llsample3		=	_mem8(input3++);

		llrightoneout	=	_mpyu4ll(_hill(llsample1), _hill(llfilt1));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample2), _hill(llfilt2)));
		llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample3), _hill(llfilt3)));

		sum1			=	_extu(_loll(llrightoneout), 16, 16);
		sum1			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum1			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum1			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum1 >> 8;

		llsample1		=	llsample1 >> 8;
		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		sum2			=	_extu(_loll(llrightoneout), 0, 16);
		sum2			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum2			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum2			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum2 >> 8;

		llsample1		=	llsample1 >> 8;
		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		sum3			=	_extu(_hill(llrightoneout), 16, 16);
		sum3			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum3			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum3			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum3 >> 8;

		llsample1		=	llsample1 >> 8;
		llsample2		=	llsample2 >> 8;
		llsample3		=	llsample3 >> 8;
		sum4			=	_extu(_hill(llrightoneout), 0, 16);
		sum4			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
		sum4			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
		sum4			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));
		*output++		=	sum4 >> 8;
	}
	/*left TOP edge */
	for (i = 0; i < 2; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = 0; j < 2; j++)
		{
			sum = 0; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < guard) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					sum += inputPtr[(i - guard + ii) * sizeY + j - guard + jj] * inputFilt[ii * 5 + jj];
				}
			}
			outputPtr[i * sizeY + j] = sum >> 8; 
		}
	}
	/*left bottom edge */
	for (i = (int32_t) sizeX - 2; i < (int32_t) sizeX; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = 0; j < 2; j++)
		{
			sum = 0; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < guard) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					sum += inputPtr[(i - guard + ii) * sizeY + j - guard + jj] * inputFilt[ii * 5 + jj];
				}
			}
			outputPtr[i * sizeY + j] = sum >> 8; 
		}
	}
	//right top edge
	for (i = 0; i < 2; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = (int32_t)sizeY-2; j < (int32_t)sizeY; j++)
		{
			sum = 0; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < guard) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					sum += inputPtr[(i - guard + ii) * sizeY + j - guard + jj] * inputFilt[ii * 5 + jj];
				}
			}
			outputPtr[i * sizeY + j] = sum >> 8; 
		}
	}
	//right bottome edge
	for (i = (int32_t)sizeX-2; i < (int32_t)sizeX; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = (int32_t)sizeY-2; j < (int32_t)sizeY; j++)
		{
			sum = 0; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < guard) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					sum += inputPtr[(i - guard + ii) * sizeY + j - guard + jj] * inputFilt[ii * 5 + jj];
				}
			}
			outputPtr[i * sizeY + j] = sum >> 8; 
		}
	}

	/*left edge middle portion*/
	#pragma UNROLL(2)
	for (i = 2; i < (int32_t)sizeX - 2; i++) 
	{
		input1	=	(int32_t *) &inputPtr[(i-2)*sizeY+0];
		input2	=	(int32_t *) &inputPtr[(i-1)*sizeY+0];
		input3	=	(int32_t *) &inputPtr[(i-0)*sizeY+0];
		input4	=	(int32_t *) &inputPtr[(i+1)*sizeY+0];
		input5	=	(int32_t *) &inputPtr[(i+2)*sizeY+0];
		output	=	(int8_t *) &outputPtr[i * sizeY];
		
		temp1	=	_amem4(input1);
		temp2	=	_amem4(input2);
		temp3	=	_amem4(input3);
		temp4	=	_amem4(input4);
		temp5	=	_amem4(input5);
		sum1	=	_dotpu4(temp1, _shrmb(_hill(llfilt1), _loll(llfilt1)));
		sum1	+=	_dotpu4(temp2, _shrmb(_hill(llfilt2), _loll(llfilt2)));
		sum1	+=	_dotpu4(temp3, _shrmb(_hill(llfilt3), _loll(llfilt3)));
		sum1	+=	_dotpu4(temp4, _shrmb(_hill(llfilt4), _loll(llfilt4)));
		sum1	+=	_dotpu4(temp5, _shrmb(_hill(llfilt5), _loll(llfilt5)));
		output[1] = sum1 >> 8;

		temp1	=	_shlmb(0, temp1);
		temp2	=	_shlmb(0, temp2);
		temp3	=	_shlmb(0, temp3);
		temp4	=	_shlmb(0, temp4);
		temp5	=	_shlmb(0, temp5);
		sum1	=	_dotpu4(temp1, _shrmb(_hill(llfilt1), _loll(llfilt1)));
		sum1	+=	_dotpu4(temp2, _shrmb(_hill(llfilt2), _loll(llfilt2)));
		sum1	+=	_dotpu4(temp3, _shrmb(_hill(llfilt3), _loll(llfilt3)));
		sum1	+=	_dotpu4(temp4, _shrmb(_hill(llfilt4), _loll(llfilt4)));
		sum1	+=	_dotpu4(temp5, _shrmb(_hill(llfilt5), _loll(llfilt5)));
		output[0] = sum1 >> 8;
	}

	/*right edge middle portion*/
	#pragma UNROLL(2)
	for (i = 2; i < (int32_t)sizeX - 2; i++) 
	{
		input1	=	(int32_t *) &inputPtr[(i-2)*sizeY+sizeY-4];
		input2	=	(int32_t *) &inputPtr[(i-1)*sizeY+sizeY-4];
		input3	=	(int32_t *) &inputPtr[(i-0)*sizeY+sizeY-4];
		input4	=	(int32_t *) &inputPtr[(i+1)*sizeY+sizeY-4];
		input5	=	(int32_t *) &inputPtr[(i+2)*sizeY+sizeY-4];
		output	=	(int8_t *) &outputPtr[i * sizeY+sizeY-2];
		
		temp1	=	_amem4(input1);
		temp2	=	_amem4(input2);
		temp3	=	_amem4(input3);
		temp4	=	_amem4(input4);
		temp5	=	_amem4(input5);
		sum1	=	_dotpu4(temp1, _loll(llfilt1));
		sum1	+=	_dotpu4(temp2, _loll(llfilt2));
		sum1	+=	_dotpu4(temp3, _loll(llfilt3));
		sum1	+=	_dotpu4(temp4, _loll(llfilt4));
		sum1	+=	_dotpu4(temp5, _loll(llfilt5));
		output[0] = sum1 >> 8;

		temp1	=	_shrmb(0, temp1);
		temp2	=	_shrmb(0, temp2);
		temp3	=	_shrmb(0, temp3);
		temp4	=	_shrmb(0, temp4);
		temp5	=	_shrmb(0, temp5);
		sum1	=	_dotpu4(temp1, _loll(llfilt1));
		sum1	+=	_dotpu4(temp2, _loll(llfilt2));
		sum1	+=	_dotpu4(temp3, _loll(llfilt3));
		sum1	+=	_dotpu4(temp4, _loll(llfilt4));
		sum1	+=	_dotpu4(temp5, _loll(llfilt5));
		output[1] = sum1 >> 8;
	}

	/*middle portion*/
	for (i = 2; i < (int32_t)sizeX - 2; i++) 
	{
		input1 = (int32_t *) &inputPtr[(i-2)*sizeY+0];
		input2 = (int32_t *) &inputPtr[(i-1)*sizeY+0];
		input3 = (int32_t *) &inputPtr[(i-0)*sizeY+0];
		input4 = (int32_t *) &inputPtr[(i+1)*sizeY+0];
		input5 = (int32_t *) &inputPtr[(i+2)*sizeY+0];
		output = (int8_t *) &outputPtr[i * sizeY+2];
		for (j = 2; j < (int32_t)sizeY - 2; j+=4)
		{
			llsample1		=	_mem8(input1++);
			llsample2		=	_mem8(input2++);
			llsample3		=	_mem8(input3++);
			llsample4		=	_mem8(input4++);
			llsample5		=	_mem8(input5++);

			llrightoneout	=	_mpyu4ll(_hill(llsample1), _hill(llfilt1));
			llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample2), _hill(llfilt2)));
			llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample3), _hill(llfilt3)));
			llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample4), _hill(llfilt4)));
			llrightoneout	=	_dsadd2(llrightoneout, _mpyu4ll(_hill(llsample5), _hill(llfilt5)));

			sum1			=	_extu(_loll(llrightoneout), 16, 16);
			sum1			+=	_dotpu4(_loll(llsample5), _loll(llfilt5));
			sum1			+=	_dotpu4(_loll(llsample4), _loll(llfilt4));
			sum1			+=	_dotpu4(_loll(llsample3), _loll(llfilt3));
			sum1			+=	_dotpu4(_loll(llsample2), _loll(llfilt2));
			sum1			+=	_dotpu4(_loll(llsample1), _loll(llfilt1));

			sum2			=	_extu(_loll(llrightoneout), 0, 16);
			iSample			=	_shrmb(_hill(llsample5), _loll(llsample5));
			sum2			+=	_dotpu4(iSample, _loll(llfilt5));
			iSample			=	_shrmb(_hill(llsample4), _loll(llsample4));
			sum2			+=	_dotpu4(iSample, _loll(llfilt4));
			iSample			=	_shrmb(_hill(llsample3), _loll(llsample3));
			sum2			+=	_dotpu4(iSample, _loll(llfilt3));
			iSample			=	_shrmb(_hill(llsample2), _loll(llsample2));
			sum2			+=	_dotpu4(iSample, _loll(llfilt2));
			iSample			=	_shrmb(_hill(llsample1), _loll(llsample1));
			sum2			+=	_dotpu4(iSample, _loll(llfilt1));
			temp1			=	_pack2(sum2, sum1);

			sum3			=	_extu(_hill(llrightoneout), 16, 16);
			iSample			=	_packlh2(_hill(llsample5), _loll(llsample5));
			sum3			+=	_dotpu4(iSample, _loll(llfilt5));
			iSample			=	_packlh2(_hill(llsample4), _loll(llsample4));
			sum3			+=	_dotpu4(iSample, _loll(llfilt4));
			iSample			=	_packlh2(_hill(llsample3), _loll(llsample3));
			sum3			+=	_dotpu4(iSample, _loll(llfilt3));
			iSample			=	_packlh2(_hill(llsample2), _loll(llsample2));
			sum3			+=	_dotpu4(iSample, _loll(llfilt2));
			iSample			=	_packlh2(_hill(llsample1), _loll(llsample1));
			sum3			+=	_dotpu4(iSample, _loll(llfilt1));

			sum4			=	_extu(_hill(llrightoneout), 0, 16);
			iSample			=	_shlmb(_loll(llsample5), _hill(llsample5));
			sum4			+=	_dotpu4(iSample, _loll(llfilt5));
			iSample			=	_shlmb(_loll(llsample4), _hill(llsample4));
			sum4			+=	_dotpu4(iSample, _loll(llfilt4));
			iSample			=	_shlmb(_loll(llsample3), _hill(llsample3));
			sum4			+=	_dotpu4(iSample, _loll(llfilt3));
			iSample			=	_shlmb(_loll(llsample2), _hill(llsample2));
			sum4			+=	_dotpu4(iSample, _loll(llfilt2));
			iSample			=	_shlmb(_loll(llsample1), _hill(llsample1));
			sum4			+=	_dotpu4(iSample, _loll(llfilt1));
			temp2			=	_pack2(sum4, sum3);

			_mem4(output)	=	_packh4(temp2, temp1);
			output			+=	4;
		}
	}
}

/*!
   \fn     RADARDEMO_threshold_c66x

   \brief   2D threshold optimized for C66x. 
   
   \param[in]    sizeX
               X dimension, always assuming power of 2 and greater than 64.

   \param[in]    sizeY
               Y dimension, always assuming power of 2 and greater than 64.

   \param[in]   inputPtr
               Input 2D grid. Must be aligned to 8-byte boundary.

   \param[in]   inputThreshold
               Input threshold.

   \param[out]    outputPtr
               Output 2D grid 1-bit per decision, in packed 8-bit format. Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_threshold_c66x(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							IN  uint8_t inputThreshold,
							OUT uint8_t * outputPtr)
{
	int32_t i;
	uint32_t result1, result2, temp;
	uint64_t  lltemp1, llthreshold;
	uint64_t * RESTRICT input1;
	uint64_t * RESTRICT output;
	
	
	temp		=	(inputThreshold << 8) | inputThreshold;
	temp		=	_pack2(temp, temp);
	llthreshold	=	_itoll(temp, temp);
	output	=	(uint64_t *)&outputPtr[0];
	input1	=	(uint64_t *)&inputPtr[0];
	for (i = 0; i < (int32_t)sizeX * sizeY >> 6; i++)
	{
		lltemp1		=	_amem8(input1++);
		result1		=	_dcmpgtu4(lltemp1, llthreshold);
		lltemp1		=	_amem8(input1++);
		result1		=	_dcmpgtu4(lltemp1, llthreshold) | _rotl(result1, 8);
		lltemp1		=	_amem8(input1++);
		result1		=	_dcmpgtu4(lltemp1, llthreshold) | _rotl(result1, 8);
		lltemp1		=	_amem8(input1++);
		result1		=	_dcmpgtu4(lltemp1, llthreshold) | _rotl(result1, 8);

		lltemp1		=	_amem8(input1++);
		result2		=	_dcmpgtu4(lltemp1, llthreshold);
		lltemp1		=	_amem8(input1++);
		result2		=	_dcmpgtu4(lltemp1, llthreshold) | _rotl(result2, 8);
		lltemp1		=	_amem8(input1++);
		result2		=	_dcmpgtu4(lltemp1, llthreshold) | _rotl(result2, 8);
		lltemp1		=	_amem8(input1++);
		result2		=	_dcmpgtu4(lltemp1, llthreshold) | _rotl(result2, 8);
		_amem8(output++)	=	_itoll(_bitr(result2), _bitr(result1));
	}
}


/*!
   \fn     RADARDEMO_morphErosion_c66x

   \brief   Morphological erosion operation optimized for C66x.  
   
   \param[in]    sizeX
               X dimension.

   \param[in]    sizeY
               Y dimension.

   \param[in]   inputPtr
               Input 2D grid. Must be aligned to 8-byte boundary.

   \param[in]   neighborhood
               Input 2D neighborhood, assuming always 5x5 size (hardcoded) 1 bit indicator per byte.

   \param[out]    outputPtr
               Output 2D grid. Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_morphErosion_c66x(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							IN  uint8_t * neighborhood,
							OUT uint8_t * outputPtr)
{
	int32_t i, j, ii, jj, iistart, iiend, jjstart, jjend, temp1, temp2;
	uint8_t min, temp;
	int32_t guard = 2; /* hard coded of (5 >> 2)*/
	
	int32_t * RESTRICT input1;
	int32_t * RESTRICT input2;
	int32_t * RESTRICT input3;
	int32_t * RESTRICT input4;
	int32_t * RESTRICT input5;
	int8_t * RESTRICT output;
	int64_t llrightoneout, llsample1, llsample2, llsample3, llsample4, llsample5, lltemp1;
	uint64_t llfilt1, llfilt2, llfilt3, llfilt4, llfilt5;

	// 4 edges, will try to optimize later
	for (i = 0; i < 2; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = 0; j < (int32_t)sizeY; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj] *  neighborhood[ii * 5 + jj] ;
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}

	for (i = (int32_t)sizeX - 2; i < (int32_t)sizeX; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = 0; j < (int32_t)sizeY; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj] *  neighborhood[ii * 5 + jj] ;
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}

	for (i = 0; i < (int32_t)sizeX; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = 0; j < 2; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj] *  neighborhood[ii * 5 + jj] ;
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}
	for (i = 0; i < (int32_t)sizeX; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = (int32_t)sizeY - 2; j < (int32_t)sizeY; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj] *  neighborhood[ii * 5 + jj] ;
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}


	//middle portion
	/*prepare neighborhood definition in order of 4 4 4 4 3 2 1 0 in uint64_t register, each bit is expanded to the whole byte */

	lltemp1		=	_mem8(&neighborhood[0]);
	temp1		=	_cmpgtu4(_loll(lltemp1), 0);
	temp1		=	_xpnd4(temp1);
	temp2		=	_xpnd4(_xpnd4(_hill(lltemp1)));
	llfilt1		=	_itoll(temp2, temp1);

	lltemp1		=	_mem8(&neighborhood[5]);
	temp1		=	_cmpgtu4(_loll(lltemp1), 0);
	temp1		=	_xpnd4(temp1);
	temp2		=	_xpnd4(_xpnd4(_hill(lltemp1)));
	llfilt2		=	_itoll(temp2, temp1);

	lltemp1		=	_mem8(&neighborhood[10]);
	temp1		=	_cmpgtu4(_loll(lltemp1), 0);
	temp1		=	_xpnd4(temp1);
	temp2		=	_xpnd4(_xpnd4(_hill(lltemp1)));
	llfilt3		=	_itoll(temp2, temp1);

	lltemp1		=	_mem8(&neighborhood[15]);
	temp1		=	_cmpgtu4(_loll(lltemp1), 0);
	temp1		=	_xpnd4(temp1);
	temp2		=	_xpnd4(_xpnd4(_hill(lltemp1)));
	llfilt4		=	_itoll(temp2, temp1);

	lltemp1		=	_mem8(&neighborhood[20]);
	temp1		=	_cmpgtu4(_loll(lltemp1), 0);
	temp1		=	_xpnd4(temp1);
	temp2		=	_xpnd4(_xpnd4(_hill(lltemp1)));
	llfilt5		=	_itoll(temp2, temp1);

	for (i = 2; i < (int32_t)sizeX - 2; i++) 
	{
		input1 = (int32_t *) &inputPtr[(i-2)*sizeY+0];
		input2 = (int32_t *) &inputPtr[(i-1)*sizeY+0];
		input3 = (int32_t *) &inputPtr[(i-0)*sizeY+0];
		input4 = (int32_t *) &inputPtr[(i+1)*sizeY+0];
		input5 = (int32_t *) &inputPtr[(i+2)*sizeY+0];
		output = (int8_t *) &outputPtr[i * sizeY+2];
		for (j = 2; j < (int32_t)sizeY - 2; j+=4)
		{
			llsample1		=	_mem8(input1++);
			llsample2		=	_mem8(input2++);
			llsample3		=	_mem8(input3++);
			llsample4		=	_mem8(input4++);
			llsample5		=	_mem8(input5++);

			llrightoneout	=	llsample1 & llfilt1;
			llrightoneout   =	_dminu4(llrightoneout, (llsample2 & llfilt2));
			llrightoneout   =	_dminu4(llrightoneout, (llsample3 & llfilt3));
			llrightoneout   =	_dminu4(llrightoneout, (llsample4 & llfilt4));
			llrightoneout   =	_dminu4(llrightoneout, (llsample5 & llfilt5));
			temp1			=	_minu4(_loll(llrightoneout), _rotl(_loll(llrightoneout), 16));
			temp2			=	_extu(_hill(llrightoneout), 24, 24);
			temp2			=	_pack2(temp2, temp2);
			temp1           =	_unpklu4(temp1);
			temp1			=	_minu4(temp1, temp2);
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			*output++		=	temp1 & 0xFF;

			llsample1		=	llsample1 >> 8;
			llsample2		=	llsample2 >> 8;
			llsample3		=	llsample3 >> 8;
			llsample4		=	llsample4 >> 8;
			llsample5		=	llsample5 >> 8;
			temp1			=	_loll(llsample1) & _loll(llfilt1);
			temp1			=	_minu4(temp1, _loll(llsample2) & _loll(llfilt2));
			temp1			=	_minu4(temp1, _loll(llsample3) & _loll(llfilt3));
			temp1			=	_minu4(temp1, _loll(llsample4) & _loll(llfilt4));
			temp1			=	_minu4(temp1, _loll(llsample5) & _loll(llfilt5));
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			min				=	_extu(_hill(llrightoneout), 16, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;

			llsample1		=	llsample1 >> 8;
			llsample2		=	llsample2 >> 8;
			llsample3		=	llsample3 >> 8;
			llsample4		=	llsample4 >> 8;
			llsample5		=	llsample5 >> 8;
			temp1			=	_loll(llsample1) & _loll(llfilt1);
			temp1			=	_minu4(temp1, _loll(llsample2) & _loll(llfilt2));
			temp1			=	_minu4(temp1, _loll(llsample3) & _loll(llfilt3));
			temp1			=	_minu4(temp1, _loll(llsample4) & _loll(llfilt4));
			temp1			=	_minu4(temp1, _loll(llsample5) & _loll(llfilt5));
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			min				=	_extu(_hill(llrightoneout), 8, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;

			llsample1		=	llsample1 >> 8;
			llsample2		=	llsample2 >> 8;
			llsample3		=	llsample3 >> 8;
			llsample4		=	llsample4 >> 8;
			llsample5		=	llsample5 >> 8;
			temp1			=	_loll(llsample1) & _loll(llfilt1);
			temp1			=	_minu4(temp1, _loll(llsample2) & _loll(llfilt2));
			temp1			=	_minu4(temp1, _loll(llsample3) & _loll(llfilt3));
			temp1			=	_minu4(temp1, _loll(llsample4) & _loll(llfilt4));
			temp1			=	_minu4(temp1, _loll(llsample5) & _loll(llfilt5));
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			min				=	_extu(_hill(llrightoneout), 0, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;
		}
	}
}	



/*!
   \fn     RADARDEMO_morphErosion_c66x2

   \brief   Morphological erosion operation optimized for C66x, simple version, assuming neighborhood is all 1s 5x5 square.  
   
   \param[in]    sizeX
               X dimension.

   \param[in]    sizeY
               Y dimension.

   \param[in]   inputPtr
               Input 2D grid. Must be aligned to 8-byte boundary.

   \param[in]   neighborhood
               Input 2D neighborhood, assuming always 5x5 size (hardcoded) 1 bit indicator per byte.

   \param[out]    outputPtr
               Output 2D grid. Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_morphErosion_c66x2(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							OUT uint8_t * outputPtr)
{
	int32_t i, j, ii, jj, iistart, iiend, jjstart, jjend, temp1, temp2;
	uint8_t min, temp;
	int32_t guard = 2; /* hard coded of (5 >> 2)*/
	
	int32_t * RESTRICT input1;
	int32_t * RESTRICT input2;
	int32_t * RESTRICT input3;
	int32_t * RESTRICT input4;
	int32_t * RESTRICT input5;
	int8_t * RESTRICT output;
	int64_t llrightoneout, llsample1, llsample2, llsample3, llsample4, llsample5, lltemp1;

	// 4 edges, will try to optimize later
	//upper 2 rows
	for (i = 0; i < 2; i++) 
	{
		input2 = (int32_t *) &inputPtr[(i-1)*sizeY+0];
		input3 = (int32_t *) &inputPtr[(i-0)*sizeY+0];
		input4 = (int32_t *) &inputPtr[(i+1)*sizeY+0];
		input5 = (int32_t *) &inputPtr[(i+2)*sizeY+0];
		output = (int8_t *) &outputPtr[i * sizeY+2];
		//#pragma UNROLL(2)
		for (j = 2; j < (int32_t)sizeY - 2; j+=4)
		{
			llrightoneout	=	_mem8(input2++);
			if (i == 0)		
				llrightoneout	=	0xFFFFFFFFFFFFFFFF;
			llsample3		=	_mem8(input3++);
			llsample4		=	_mem8(input4++);
			llsample5		=	_mem8(input5++);

			llrightoneout   =	_dminu4(llrightoneout, llsample3);
			llrightoneout   =	_dminu4(llrightoneout, llsample4);
			llrightoneout   =	_dminu4(llrightoneout, llsample5);
			temp1			=	_minu4(_loll(llrightoneout), _rotl(_loll(llrightoneout), 16));
			temp2			=	_extu(_hill(llrightoneout), 24, 24);
			temp2			=	_pack2(temp2, temp2);
			temp1           =	_unpklu4(temp1);
			temp1			=	_minu4(temp1, temp2);
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			*output++		=	temp1 & 0xFF;
			/*
			min				=	_extu(_hill(llrightoneout), 24, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;*/

			temp2			=	_shrmb(_hill(llrightoneout), _loll(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			temp2			=	_extu(_hill(llrightoneout), 16, 24);
			temp2			=	_pack2(temp2, temp2);
			temp1           =	_unpklu4(temp1);
			temp1			=	_minu4(temp1, temp2);
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			*output++		=	temp1 & 0xFF;
			/*
			min				=	_extu(_hill(llrightoneout), 16, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;
			*/
			temp2			=	_packlh2(_hill(llrightoneout), _loll(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			min				=	_extu(_hill(llrightoneout), 8, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;

			temp2			=	_shlmb(_loll(llrightoneout),_hill(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			min				=	_extu(_hill(llrightoneout), 0, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;
		}
	}
	//lower 2 rows
	for (i = (int32_t)sizeX - 2; i < (int32_t)sizeX; i++) 
	{
		input1 = (int32_t *) &inputPtr[(i-2)*sizeY+0];
		input2 = (int32_t *) &inputPtr[(i-1)*sizeY+0];
		input3 = (int32_t *) &inputPtr[(i-0)*sizeY+0];
		input4 = (int32_t *) &inputPtr[(i+1)*sizeY+0];
		output = (int8_t *) &outputPtr[i * sizeY+2];
		//#pragma UNROLL(2)
		for (j = 2; j < (int32_t)sizeY - 2; j+=4)
		{
			llsample1	=	_mem8(input1++);
			llsample2	=	_mem8(input2++);
			llsample3		=	_mem8(input3++);
			llsample4		=	_mem8(input4++);
			if (i == (int32_t)sizeX - 1)		
				llsample4	=	0xFFFFFFFFFFFFFFFF;

			llrightoneout	=	llsample1;
			llrightoneout   =	_dminu4(llrightoneout, llsample2);
			llrightoneout   =	_dminu4(llrightoneout, llsample3);
			llrightoneout   =	_dminu4(llrightoneout, llsample4);
			temp1			=	_minu4(_loll(llrightoneout), _rotl(_loll(llrightoneout), 16));
			temp2			=	_extu(_hill(llrightoneout), 24, 24);
			temp2			=	_pack2(temp2, temp2);
			temp1           =	_unpklu4(temp1);
			temp1			=	_minu4(temp1, temp2);
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			*output++		=	temp1 & 0xFF;
			/*
			min				=	_extu(_hill(llrightoneout), 24, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;*/

			temp2			=	_shrmb(_hill(llrightoneout), _loll(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			temp2			=	_extu(_hill(llrightoneout), 16, 24);
			temp2			=	_pack2(temp2, temp2);
			temp1           =	_unpklu4(temp1);
			temp1			=	_minu4(temp1, temp2);
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			*output++		=	temp1 & 0xFF;
			/*
			min				=	_extu(_hill(llrightoneout), 16, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;
			*/
			temp2			=	_packlh2(_hill(llrightoneout), _loll(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			min				=	_extu(_hill(llrightoneout), 8, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;

			temp2			=	_shlmb(_loll(llrightoneout),_hill(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			min				=	_extu(_hill(llrightoneout), 0, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;
		}
	}

	//left 2 columns
	for (i = 2; i < (int32_t)sizeX - 2; i++) 
	{
		input1 = (int32_t *) &inputPtr[(i-2)*sizeY+0];
		input2 = (int32_t *) &inputPtr[(i-1)*sizeY+0];
		input3 = (int32_t *) &inputPtr[(i-0)*sizeY+0];
		input4 = (int32_t *) &inputPtr[(i+1)*sizeY+0];
		input5 = (int32_t *) &inputPtr[(i+2)*sizeY+0];
		output = (int8_t *) &outputPtr[i * sizeY];

		temp2	=	_minu4(_amem4(input1++), _amem4(input2++));
		temp2	=	_minu4(temp2, _amem4(input3++));
		temp2	=	_minu4(temp2, _amem4(input4++));
		temp2	=	_minu4(temp2, _amem4(input5++));
		temp1	=	_minu4(temp2, _rotl(temp2, 16));
		min		=	_extu(temp1, 16, 24);
		if (min > _extu(temp1, 24, 24))
			min =	_extu(temp1, 24, 24);
		output[1] = min;

		temp2	|=	0xFF000000;
		temp1	=	_minu4(temp2, _rotl(temp2, 16));
		min		=	_extu(temp1, 16, 24);
		if (min > _extu(temp1, 24, 24))
			min =	_extu(temp1, 24, 24);
		output[0] = min;
	}
	//right 2 columns
	for (i = 2; i < (int32_t)sizeX-2; i++) 
	{
		input1 = (int32_t *) &inputPtr[(i-2)*sizeY+sizeY-4];
		input2 = (int32_t *) &inputPtr[(i-1)*sizeY+sizeY-4];
		input3 = (int32_t *) &inputPtr[(i-0)*sizeY+sizeY-4];
		input4 = (int32_t *) &inputPtr[(i+1)*sizeY+sizeY-4];
		input5 = (int32_t *) &inputPtr[(i+2)*sizeY+sizeY-4];
		output = (int8_t *) &outputPtr[i * sizeY + sizeY-2];

		temp2	=	_minu4(_amem4(input1++), _amem4(input2++));
		temp2	=	_minu4(temp2, _amem4(input3++));
		temp2	=	_minu4(temp2, _amem4(input4++));
		temp2	=	_minu4(temp2, _amem4(input5++));
		temp1	=	_minu4(temp2, _rotl(temp2, 16));
		min		=	_extu(temp1, 16, 24);
		if (min > _extu(temp1, 24, 24))
			min =	_extu(temp1, 24, 24);
		output[0] = min;

		temp2	|=	0x000000FF;
		temp1	=	_minu4(temp2, _rotl(temp2, 16));
		min		=	_extu(temp1, 16, 24);
		if (min > _extu(temp1, 24, 24))
			min =	_extu(temp1, 24, 24);
		output[1] = min;
	}

	//upper left edge
	for (i = 0; i < 2; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = 0; j < 2; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj];
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}
	//upper right edge
	for (i = 0; i < 2; i++)
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = (int32_t)sizeY-2; j < (int32_t)sizeY; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj];
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}
	//lower left edge
	for (i = (int32_t)sizeX - 2; i < (int32_t)sizeX; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = 0; j < 2; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj];
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}

	for (i = (int32_t)sizeX - 2; i < (int32_t)sizeX; i++) 
	{
		iistart 	= 	0;
		iiend 		= 	5;
		if (i < guard) 
		{
			iistart 	= 	guard - i;
		}
		if (i > (int32_t)sizeX - 1 - guard)
		{
			iiend 		=	sizeX - i + guard;
		}
		for (j = (int32_t)sizeY - 2; j < (int32_t)sizeY; j++)
		{
			min = 255; 
			jjstart 	= 	0;
			jjend 		= 	5;
			if (j < 2) 
			{
				jjstart 	= 	guard - j;
			}
			if (j > (int32_t)sizeY - 1 - guard)
			{
				jjend 		=	sizeY - j + guard;
			}

			for (ii = iistart; ii < iiend; ii++)
			{
				for (jj = jjstart; jj < jjend; jj++)
				{
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj];
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}

	//middle portion
	for (i = 2; i < (int32_t)sizeX - 2; i++) 
	{
		input1 = (int32_t *) &inputPtr[(i-2)*sizeY+0];
		input2 = (int32_t *) &inputPtr[(i-1)*sizeY+0];
		input3 = (int32_t *) &inputPtr[(i-0)*sizeY+0];
		input4 = (int32_t *) &inputPtr[(i+1)*sizeY+0];
		input5 = (int32_t *) &inputPtr[(i+2)*sizeY+0];
		output = (int8_t *) &outputPtr[i * sizeY+2];
		//#pragma UNROLL(2)
		for (j = 2; j < (int32_t)sizeY - 2; j+=4)
		{
			llsample1		=	_mem8(input1++);
			llsample2		=	_mem8(input2++);
			llsample3		=	_mem8(input3++);
			llsample4		=	_mem8(input4++);
			llsample5		=	_mem8(input5++);

			llrightoneout	=	llsample1;
			llrightoneout   =	_dminu4(llrightoneout, llsample2);
			llrightoneout   =	_dminu4(llrightoneout, llsample3);
			llrightoneout   =	_dminu4(llrightoneout, llsample4);
			llrightoneout   =	_dminu4(llrightoneout, llsample5);
			temp1			=	_minu4(_loll(llrightoneout), _rotl(_loll(llrightoneout), 16));
			temp2			=	_extu(_hill(llrightoneout), 24, 24);
			temp2			=	_pack2(temp2, temp2);
			temp1           =	_unpklu4(temp1);
			temp1			=	_minu4(temp1, temp2);
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			*output++		=	temp1 & 0xFF;
			/*
			min				=	_extu(_hill(llrightoneout), 24, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;*/

			temp2			=	_shrmb(_hill(llrightoneout), _loll(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			temp2			=	_extu(_hill(llrightoneout), 16, 24);
			temp2			=	_pack2(temp2, temp2);
			temp1           =	_unpklu4(temp1);
			temp1			=	_minu4(temp1, temp2);
			temp1			=	_minu4(temp1, _rotl(temp1, 16));
			*output++		=	temp1 & 0xFF;
			/*
			min				=	_extu(_hill(llrightoneout), 16, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;
			*/
			temp2			=	_packlh2(_hill(llrightoneout), _loll(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			min				=	_extu(_hill(llrightoneout), 8, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;

			temp2			=	_shlmb(_loll(llrightoneout),_hill(llrightoneout));
			temp1			=	_minu4(temp2, _rotl(temp2, 16));
			min				=	_extu(_hill(llrightoneout), 0, 24);
			if (min > _extu(temp1, 24, 24))
				min = _extu(temp1, 24, 24);
			if (min > _extu(temp1, 16, 24))
				min = _extu(temp1, 16, 24);
			*output++		=	min;
		}
	}
}	



