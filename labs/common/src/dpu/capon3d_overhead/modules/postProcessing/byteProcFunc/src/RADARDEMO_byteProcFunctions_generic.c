/*! 
 *  \file   RADARDEMO_byteProcFunctions_generic.c
 *
 *  \brief   Generic C code for byte processing. 
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
   \fn     RADARDEMO_convKernel_generic

   \brief   2D convolution kernel in generic C. 
   
   \param[in]    sizeX
               X dimension.

   \param[in]    sizeY
               Y dimension.

   \param[in]   inputPtr
               Input 2D grid, stored row by row. Must be aligned to 8-byte boundary.

   \param[in]   inputFilt
               Input 2D fitler, assuming always 5x5 size (hardcoded) and assuming symmetry, stored row by row.

   \param[out]    outputPtr
               Output 2D grid, stored row by row. Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_convKernel_generic(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							IN  uint8_t * inputFilt,
							OUT uint8_t * outputPtr)
{
	int32_t i, j, ii, jj, iistart, iiend, jjstart, jjend;
	int32_t sum;
	int32_t guard = 2; /* hard coded of (5 >> 2)*/
	
	for (i = 0; i < (int32_t) sizeX; i++) 
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
	
}

/*!
   \fn     RADARDEMO_threshold_generic

   \brief   2D threshold in generic C. 
   
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

void	RADARDEMO_threshold_generic(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							IN  uint8_t inputThreshold,
							OUT uint8_t * outputPtr)
{
	int32_t i, j;
	uint8_t results, *input;
	
	input	=	inputPtr;
	for (i = 0; i < (int32_t)sizeX; i++) 
	{
		input	=	&inputPtr[i * sizeY];
		for (j = 0; j < (int32_t)sizeY >> 3; j++)
		{
			results	=	0;
			if (input[8 * j + 0] > inputThreshold)
				results |= 0x80;
			if (input[8 * j + 1] > inputThreshold)
				results |= 0x40;
			if (input[8 * j + 2] > inputThreshold)
				results |= 0x20;
			if (input[8 * j + 3] > inputThreshold)
				results |= 0x10;
			if (input[8 * j + 4] > inputThreshold)
				results |= 0x8;
			if (input[8 * j + 5] > inputThreshold)
				results |= 0x4;
			if (input[8 * j + 6] > inputThreshold)
				results |= 0x2;
			if (input[8 * j + 7] > inputThreshold)
				results |= 0x1;
		
			*outputPtr++ = results;
		}
	}
}

/*!
   \fn     RADARDEMO_morphErosion_generic

   \brief   Morphological erosion operation in generic C. 
   
   \param[in]    sizeX
               X dimension.

   \param[in]    sizeY
               Y dimension.

   \param[in]   inputPtr
               Input 2D grid. Must be aligned to 8-byte boundary.

   \param[in]   neighborhood
               Input 2D neighborhood, assuming always 5x5 size (hardcoded) bit indicator packed in lower 5-bit of a byte.

   \param[out]    outputPtr
               Output 2D grid. Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_morphErosion_generic(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							IN  uint8_t * neighborhood,
							OUT uint8_t * outputPtr)
{
	int32_t i, j, ii, jj, iistart, iiend, jjstart, jjend;
	uint8_t min, temp;
	int32_t guard = 2; /* hard coded of (5 >> 2)*/
	
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
	
}


/*!
   \fn     RADARDEMO_morphErosion_generic2

   \brief   Morphological erosion operation in generic C, simple version, assuming neighborhood is all 1s 5x5 square.  
   
   \param[in]    sizeX
               X dimension.

   \param[in]    sizeY
               Y dimension.

   \param[in]   inputPtr
               Input 2D grid. Must be aligned to 8-byte boundary.

   \param[in]   neighborhood
               Input 2D neighborhood, assuming always 5x5 size (hardcoded) bit indicator packed in lower 5-bit of a byte.

   \param[out]    outputPtr
               Output 2D grid. Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_morphErosion_generic2(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							OUT uint8_t * outputPtr)
{
	int32_t i, j, ii, jj, iistart, iiend, jjstart, jjend;
	uint8_t min, temp;
	int32_t guard = 2; /* hard coded of (5 >> 2)*/
	
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
					temp = inputPtr[(i - guard + ii) * sizeY +j - guard + jj];
					if (temp < min)
						min = temp;
				}
			}
			outputPtr[i* sizeY +j] = min; 
		}
	}
	
}

