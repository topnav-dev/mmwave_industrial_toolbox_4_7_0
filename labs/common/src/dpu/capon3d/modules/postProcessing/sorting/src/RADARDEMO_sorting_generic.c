/*! 
 *  \file   RADARDEMO_sorting_generic.c
 *
 *  \brief   Generic C code for sorting function. 
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

#include <modules/postProcessing/sorting/api/RADARDEMO_sorting.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x

DEBUG(FILE * testout;)

static void TopDownMerge(
		float *  A,
		int32_t iBegin,
		int32_t iMiddle,
		int32_t iEnd,
		float *  B,
		int16_t *  indxA,
		int16_t *  indxB)
{
	int32_t i, j, k, flag;

    i 	=	iBegin;
    j 	= 	iMiddle;
 
	DEBUG(fprintf(testout, "B = 0x%x\n", (uint32_t)B));
	DEBUG(fprintf(testout, "A = 0x%x\n", (uint32_t)A));
	//DEBUG(fprintf(testout, "indxB = 0x%x\n", (uint32_t)indxB));
	//DEBUG(fprintf(testout, "indxA = 0x%x\n", (uint32_t)indxA));
	DEBUG(fprintf(testout, "iBegin = %d\n", (uint32_t)iBegin));
	DEBUG(fprintf(testout, "iEnd = %d\n", (uint32_t)iEnd));
	//DEBUG(fprintf(testout, "iMiddle = %d\n", (uint32_t)iMiddle));

    // While there are elements in the left or right runs...
    for (k = iBegin; k < iEnd; k++)
    {
#if 1
    	flag 				=	(A[i] <= A[j]);
    	flag 				|=	(j >= iEnd);
    	flag 				&=	(i < iMiddle);

		//DEBUG(fprintf(testout, "flag = %d\n", flag));
    	if (flag)
    		B[k] 			= 	A[i];
    	else
    		B[k] 			= 	A[j];

    	if (flag)
    		indxB[k]		= 	indxA[i];
    	else
    		indxB[k]		= 	indxA[j];

    	if(flag)
    		i++;
    	else
    		j++;
		DEBUG(fprintf(testout, "B[%d] = %f\n", k, B[k]));
#else
        // If left run head exists and is <= existing right run head.
        if (i < iMiddle && (j >= iEnd || A[i] <= A[j]))
        {
            B[k] 			= 	A[i];
            indxB[k] 		=	indxA[i];
            i 				= 	i + 1;
			DEBUG(fprintf(testout, "flag = %d\n", 1));
        }
        else
        {
            B[k] 			= 	A[j];
            indxB [k] 		=	indxA[j];
            j 				= 	j + 1;
			DEBUG(fprintf(testout, "flag = %d\n", 0));
        }
#endif
    }
}

static void TopDownSplitMerge(
		float * B,
		int32_t iBegin,
		int32_t iEnd,
		float * A,
		int16_t * indxB,
		int16_t * indxA)
{
	int32_t iMiddle;

    if( iEnd - iBegin < 2 )                       // if run size == 1
        return;                                 //   consider it sorted

    // split the run longer than 1 item into halves
    iMiddle = (iEnd + iBegin) / 2;              // iMiddle = mid point
    // recursively sort both runs from array A[] into B[]
    TopDownSplitMerge(A, iBegin,  iMiddle, B, indxA, indxB);  // sort the left  run
    TopDownSplitMerge(A, iMiddle,    iEnd, B, indxA, indxB);  // sort the right run
    // merge the resulting runs from array B[] into A[]
    TopDownMerge(B, iBegin, iMiddle, iEnd, A, indxB, indxA);
}


/*!
   \fn     RADARDEMO_sortingMerge_generic

   \brief   Merge sort function in generic C, in ascend order.
   
   \param[in]    len
               Length of the input vector.

   \param[in, out]   inputPtr
               Input vector. 

   \param[in, out]   indxBuf
               index buffer, length of 2 * len int16_t type. Second half is used as scratch buffer.

   \param[out]    outputPtr
               Output sorted vector

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_sortingMerge_generic(
                            IN  uint32_t len,
							INOUT  float * inputPtr,
							INOUT int16_t * indxBuf,
							OUT float * outputPtr)
{
	int32_t i;
	int16_t *indxBuffIn, *indxBuffOut;

	DEBUG(testout = fopen("test.dat", "a"));
	
	for( i = 0; i < (int32_t) len; i++ )
	{
        outputPtr[i] 	= 	inputPtr[i];
	}

	indxBuffIn 	=	(int16_t *) &indxBuf[0];
	indxBuffOut	=	(int16_t *) &indxBuf[len];
	for( i = 0; i < (int32_t) len; i++ )
	{
		indxBuffIn[i] 	= 	i;
		indxBuffOut[i] 	= 	i;
	}

	TopDownSplitMerge(outputPtr, 0, len, inputPtr, indxBuffOut, indxBuffIn);

	DEBUG(fclose(testout));

}


/*!
   \fn     RADARDEMO_sortingSimple_generic

   \brief   Simple sort function in generic C, in ascend order.

   \param[in]    len
               Length of the input vector.

   \param[in,out]   inputPtr
               Input vector, output will be stored in place.

   \param[out]    indxArray
               Output index array for the sorted vector

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_sortingSimple_generic(
                            IN  uint32_t len,
							INOUT  float * inputPtr,
							OUT int16_t * indxArray)
{
	int32_t i, j, indx, itemp;
	float min, ftemp;

	for( i = 0; i < (int32_t) len; i++ )
	{
		indxArray[i] = i;
	}
	for( i = 0; i < (int32_t) len; i++ )
	{
		min 		=	1.0e38;
		for (j = i; j < (int32_t)len; j++)
		{
			if(inputPtr[j] < min)
			{
				min		=	inputPtr[j];
				indx 	=	j;
			}
		}
		ftemp 			=	inputPtr[i];
		inputPtr[i]		=	inputPtr[indx];
		inputPtr[indx]	=	ftemp;
		itemp			=	indxArray[i];
		indxArray[i]	=	indxArray[indx];
		indxArray[indx]	=	itemp;
	}
}


/*!
   \fn     RADARDEMO_sortingBubble_generic

   \brief   Bubble sort function in generic C, in ascend order.

   \param[in]    len
               Length of the input vector.

   \param[in,out]   inputPtr
               Input vector, output will be stored in place.

   \param[out]    indxArray
               Output index array for the sorted vector

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_sortingBubble_generic(
                            IN  uint32_t len,
							INOUT  float * inputPtr,
							OUT int16_t * indxArray)
{

	int32_t i, j, indx;
	float ftemp;

	for( i = 0; i < (int32_t) len; i++ )
	{
		indxArray[i] = i;
	}
	for( i = 0; i < (int32_t)len - 1; i++ )
	{
		for (j = 0; j < (int32_t)len - i - 1; j++)
		{
			if(inputPtr[j] > inputPtr[j + 1])
			{
				ftemp			=	inputPtr[j];
				inputPtr[j]	    =	inputPtr[j + 1];
				inputPtr[j + 1] =	ftemp;
				indx 			=	indxArray[j];
				indxArray[j]	=	indxArray[j + 1];
				indxArray[j + 1]	=	indx;
			}
		}
	}
}


/*!
   \fn     RADARDEMO_osSimple_generic

   \brief   Simple ordered statistics function in generic C, in ascend order.

   \param[in]    len
               Length of the input vector.

   \param[in]    k
               Rank out the output ordered statistics.

   \param[in,out]   inputPtr
               Input vector, output will be stored in place.

   \param[out]    indxArray
               Output index array for the sorted vector

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_osSimple_generic(
                            IN  uint32_t len,
							IN  uint32_t k,
							INOUT  float * inputPtr,
							OUT int16_t * indxArray)
{
	int32_t i, j, indx, itemp;
	float min, ftemp;

	for( i = 0; i < (int32_t) len; i++ )
	{
		indxArray[i] = i;
	}
	for( i = 0; i < (int32_t) k; i++ )
	{
		min 		=	1.0e38;
		for (j = i; j < (int32_t)len; j++)
		{
			if(inputPtr[j] < min)
			{
				min		=	inputPtr[j];
				indx 	=	j;
			}
		}
		ftemp 			=	inputPtr[i];
		inputPtr[i]		=	inputPtr[indx];
		inputPtr[indx]	=	ftemp;
		itemp			=	indxArray[i];
		indxArray[i]	=	indxArray[indx];
		indxArray[indx]	=	itemp;
	}
}


/*!
   \fn     RADARDEMO_sortingSimple16_generic

   \brief   Simple sort function in generic C for 16-bit fixed point input, in ascend order.

   \param[in]    len
               Length of the input vector.

   \param[in,out]   inputPtr
               Input vector, output will be stored in place.

   \param[out]    indxArray
               Output index array for the sorted vector

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_sortingSimple16_generic(
                            IN  uint32_t len,
							INOUT  int16_t * inputPtr,
							OUT int16_t * indxArray)
{
	int32_t i, j, indx, itemp;
	int16_t min;

	for( i = 0; i < (int32_t) len; i++ )
	{
		indxArray[i] = i;
	}
	for( i = 0; i < (int32_t) len; i++ )
	{
		min 		=	32767;
		for (j = i; j < (int32_t)len; j++)
		{
			if(inputPtr[j] < min)
			{
				min		=	inputPtr[j];
				indx 	=	j;
			}
		}
		itemp 			=	inputPtr[i];
		inputPtr[i]		=	inputPtr[indx];
		inputPtr[indx]	=	itemp;
		itemp			=	indxArray[i];
		indxArray[i]	=	indxArray[indx];
		indxArray[indx]	=	itemp;
	}
}
