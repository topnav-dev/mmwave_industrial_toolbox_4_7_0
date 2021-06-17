/*! 
 *  \file   RADARDEMO_sorting_C66x.c
 *
 *  \brief   C66x optimized code for sorting function. 
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
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x
DEBUG(FILE * testout1;)

static void topDownMergeSortPair(
		float * RESTRICT A,
		int32_t len,
		float * RESTRICT B,
		int16_t * RESTRICT indxA,
		int16_t * RESTRICT indxB)
{
	int32_t		i, half;
	int32_t		index1, index2, index3, index4, itemp, flag1, flag2, flag3, flag4;
	__float2_t	input1, input2, input3, input4, f2temp1, ftemp2;
	int16_t * RESTRICT indxA1, * RESTRICT indxB1;
	float * RESTRICT A1, * RESTRICT B1;

	half			=	len >> 1;
	indxA1			=	(int16_t *)&indxA[half];
	indxB1			=	(int16_t *)&indxB[half];
	A1				=	(float *) &A[half];
	B1				=	(float *) &B[half];

	for (i = 0; i < half; i += 4)
	{
		index1		=	_amem4(&indxA[i]);
		index2		=	_amem4(&indxA1[i]);

		input1		=	_amem8_f2(&A[i]);
		input2		=	_amem8_f2(&A1[i]);

		f2temp1		=	_dsubsp(input1, input2);
		flag1		=	(_hif2(f2temp1) < 0);
		flag2		=	(_lof2(f2temp1) < 0);
		//flag1		=	_ext(_ftoi(_hif2(f2temp1)), 0, 31);
		//flag2		=	_ext(_ftoi(_lof2(f2temp1)), 0, 31);

		if(!flag1)
			ftemp2	=	_ftof2(_hif2(input1), _hif2(input2));
		else
			ftemp2	=	_ftof2(_hif2(input2), _hif2(input1));
		_amem8_f2(&B[i])	=	ftemp2;

		if(!flag1)
			itemp	=	_packh2(index1, index2);
		else
			itemp	=	_packh2(index2, index1);
		_amem4(&indxB[i])	=	itemp;

		if(!flag2)
			ftemp2	=	_ftof2(_lof2(input1), _lof2(input2));
		else
			ftemp2	=	_ftof2(_lof2(input2), _lof2(input1));
		_amem8_f2(&B1[i])	=	ftemp2;

		if(!flag2)
			itemp	=	_pack2(index1, index2);
		else
			itemp	=	_pack2(index2, index1);
		_amem4(&indxB1[i])	=	itemp;


		index3		=	_amem4(&indxA[i + 2]);
		index4		=	_amem4(&indxA1[i + 2]);

		input3		=	_amem8_f2(&A[i + 2]);
		input4		=	_amem8_f2(&A1[i + 2]);

		f2temp1		=	_dsubsp(input3, input4);
		flag3		=	(_hif2(f2temp1) < 0);
		flag4		=	(_lof2(f2temp1) < 0);
		//flag3		=	_ext(_ftoi(_hif2(f2temp1)), 0, 31);
		//flag4		=	_ext(_ftoi(_lof2(f2temp1)), 0, 31);

		if(!flag3)
			ftemp2	=	_ftof2(_hif2(input3), _hif2(input4));
		else
			ftemp2	=	_ftof2(_hif2(input4), _hif2(input3));
		_amem8_f2(&B[i + 2])	=	ftemp2;

		if(!flag3)
			itemp	=	_packh2(index3, index4);
		else
			itemp	=	_packh2(index4, index3);
		_amem4(&indxB[i + 2])	=	itemp;

		if(!flag4)
			ftemp2	=	_ftof2(_lof2(input3), _lof2(input4));
		else
			ftemp2	=	_ftof2(_lof2(input4), _lof2(input3));
		_amem8_f2(&B1[i + 2])	=	ftemp2;

		if(!flag4)
			itemp	=	_pack2(index3, index4);
		else
			itemp	=	_pack2(index4, index3);
		_amem4(&indxB1[i + 2])	=	itemp;
	}
}


static void topDownMergeFinalStage(
		float * RESTRICT A,
		int32_t iBegin,
		int32_t iMiddle,
		int32_t iEnd,
		float * RESTRICT B,
		int16_t * RESTRICT indxA,
		int16_t * RESTRICT indxB)
{
	int32_t i, j, k, flag;

    i 	=	iBegin;
    j 	= 	iMiddle;


    // While there are elements in the left or right runs...
    for (k = iBegin; k < iEnd; k++)
    {
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
    }
}

static void topDownMerge(
		float * RESTRICT A,
		int32_t iBegin,
		int32_t iMiddle,
		int32_t iEnd,
		float * RESTRICT B,
		int16_t * RESTRICT indxA,
		int16_t * RESTRICT indxB)
{
	int32_t i1, j1, i2, j2, k, flag, iMiddleNew, iMiddle1, iMiddle2, iEnd1, iEnd2, delta;

	DEBUG(fprintf(testout1, "B = 0x%x\n", (uint32_t)B));
	DEBUG(fprintf(testout1, "A = 0x%x\n", (uint32_t)A));
	//DEBUG(fprintf(testout1, "indxB = 0x%x\n", (uint32_t)indxB));
	//DEBUG(fprintf(testout1, "indxA = 0x%x\n", (uint32_t)indxA));
	DEBUG(fprintf(testout1, "iBegin = %d\n", (uint32_t)iBegin));
	DEBUG(fprintf(testout1, "iEnd = %d\n", (uint32_t)iEnd));
	//DEBUG(fprintf(testout1, "iMiddle = %d\n", (uint32_t)iMiddle));

	iMiddleNew	=	(iEnd + iBegin) / 2;
	iEnd1		=	iMiddleNew;
	iEnd2		=	iEnd;
	iMiddle1	=	(iBegin + iEnd1) / 2;
	iMiddle2	=	(iEnd2 + iMiddleNew) / 2;

    i1 		=	iBegin;
    j1 		= 	iMiddle1;

    i2 	=	iMiddleNew;
    j2 	= 	iMiddle2;

	delta	=	iMiddleNew - iBegin;

	{
		// While there are elements in the left or right runs...
		for (k = iBegin; k < iMiddleNew; k++)
		{
    		flag 				=	(A[i1] <= A[j1]);
    		flag 				|=	(j1 >= iEnd1);
    		flag 				&=	(i1 < iMiddle1);
			//DEBUG(fprintf(testout1, "flag1 = %d\n", flag));
    		if (flag)
    			B[k] 			= 	A[i1];
    		else
    			B[k] 			= 	A[j1];

    		if (flag)
    			indxB[k]		= 	indxA[i1];
    		else
    			indxB[k]		= 	indxA[j1];

    		if(flag)
    			i1++;
    		else
    			j1++;

    		flag 				=	(A[i2] <= A[j2]);
    		flag 				|=	(j2 >= iEnd2);
    		flag 				&=	(i2 < iMiddle2);
			//DEBUG(fprintf(testout1, "flag2 = %d\n", flag));
    		if (flag)
    			B[k + delta] 		= 	A[i2];
    		else
    			B[k + delta] 		= 	A[j2];

    		if (flag)
    			indxB[k + delta]	= 	indxA[i2];
    		else
    			indxB[k + delta]	= 	indxA[j2];

    		if(flag)
    			i2++;
    		else
    			j2++;

		}
	}
	for (k = iBegin; k < iEnd; k++)
	{
		DEBUG(fprintf(testout1, "B[%d] = %f\n", k, B[k]));
	}
}

static void topDownSplitMerge(
		float * B,
		int32_t iBegin,
		int32_t iEnd,
		float * A,
		int16_t * indxB,
		int16_t * indxA)
{
	int32_t iMiddle;

    if( iEnd - iBegin < 8 )                       // if run size == 1
        return;                                 //   consider it sorted

    // split the run longer than 1 item into halves
    iMiddle = (iEnd + iBegin) / 2;              // iMiddle = mid point
    // recursively sort both runs from array A[] into B[]
    topDownSplitMerge(A, iBegin,  iMiddle, B, indxA, indxB);  // sort the left  run
    topDownSplitMerge(A, iMiddle, iEnd, B, indxA, indxB);  // sort the right run
    // merge the resulting runs from array B[] into A[]
    topDownMerge(B, iBegin, iMiddle, iEnd, A, indxB, indxA);
}


/*!
   \fn     RADARDEMO_sortingMerge_C66x

   \brief   Merge sort function in optimized C66x code, in ascend order.
   
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

void	RADARDEMO_sortingMerge_C66x(
                            IN  uint32_t len,
							INOUT  float * outputPtr,
							INOUT int16_t * indxBuf,
							OUT float * inputPtr)
{
	int32_t i, flip;
	int16_t *indxBuffIn, *indxBuffOut;

	DEBUG(testout1 = fopen("test1.dat", "a"));
	for( i = 0; i < (int32_t) len; i++ )
	{
        inputPtr[i] 	= 	outputPtr[i];
	}

	indxBuffIn 	=	(int16_t *) &indxBuf[len];
	indxBuffOut	=	(int16_t *) &indxBuf[0];
	for( i = 0; i < (int32_t) len; i++ )
	{
		indxBuffIn[i] 	= 	i;
		indxBuffOut[i] 	= 	i;
	}

	flip	=	_norm(len);
	if (flip & 1)
	{
		topDownMergeSortPair(
			inputPtr,
			len,
			outputPtr,
			indxBuffIn,
			indxBuffOut);
	}
	else
	{
		topDownMergeSortPair(
			outputPtr,
			len,
			inputPtr,
			indxBuffOut,
			indxBuffIn);
	}
	//topDownSplitMerge(inputPtr, 0, len, outputPtr, indxBuffIn, indxBuffOut);
	//topDownMergeFinalStage(outputPtr, 0, (len>>1), len, inputPtr, indxBuffOut, indxBuffIn);
	topDownSplitMerge(outputPtr, 0, len, inputPtr, indxBuffOut, indxBuffIn);
	topDownMergeFinalStage(inputPtr, 0, (len>>1), len, outputPtr, indxBuffIn, indxBuffOut);
	DEBUG(fclose(testout1));
}

/*!
   \fn     RADARDEMO_sortingSimple_C66x

   \brief   Simple sort function in optimized C66x code, in ascend order.

   \param[in]    len
               Length of the input vector.

   \param[in,out]   inputPtr
               Input vector, output will be stored in place, must be aligned to 8-byte bounday.

   \param[out]    indxArray
               Output index array for the sorted vector, must be aligned to 8-byte bounday.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_sortingSimple_C66x(
                            IN  uint32_t len,
							INOUT  float * inputPtr,
							OUT int16_t * indxArray)
{
	int32_t i, j, k, indx, itemp, indx1, indx2, indx3, tempLen;
	float min, ftemp, min1, min2, min3;
	__float2_t f2input;

	for( i = 0; i < (int32_t) len; i+=2 )
	{
		_amem4(&indxArray[i]) = _pack2(i+1, i);
	}
	for( i = 0; i < (int32_t) len; i++ )
	{
		if (i&1)
		{
			min		=	inputPtr[i];
			indx 	=	i;
			k		=	1;
		}
		else
		{
			min 		=	1.0e38;
			indx		=	0;
			k			=	0;
		}
		min1	=	min;
		indx1	=	indx;
		min2	=	min;
		indx2	=	indx;
		min3	=	min;
		indx3	=	indx;
		tempLen =	len - ((i+k) & 3);
		for (j = i + k; j < tempLen; j+=4)
		{
			f2input		=	_amem8_f2(&inputPtr[j]);
			if(_hif2(f2input) < min)
			{
				min		=	_hif2(f2input);
				indx 	=	j + 1;
			}

			if(_lof2(f2input) < min1)
			{
				min1	=	_lof2(f2input);
				indx1 	=	j;
			}
			f2input		=	_amem8_f2(&inputPtr[j + 2]);
			if(_hif2(f2input) < min2)
			{
				min2	=	_hif2(f2input);
				indx2 	=	j + 3;
			}

			if(_lof2(f2input) < min3)
			{
				min3	=	_lof2(f2input);
				indx3 	=	j + 2;
			}
		}
		if (tempLen != len)
		{
			f2input		=	_amem8_f2(&inputPtr[j]);
			if(_hif2(f2input) < min)
			{
				min		=	_hif2(f2input);
				indx 	=	j+1;
			}

			if(_lof2(f2input) < min1)
			{
				min1	=	_lof2(f2input);
				indx1 	=	j;
			}
		}
		if(min1 < min)
		{
			min 		=	min1;
			indx		=	indx1;
		}
		if(min2 < min)
		{
			min 		=	min2;
			indx		=	indx2;
		}
		if(min3 < min)
		{
			min 		=	min3;
			indx		=	indx3;
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
   \fn     RADARDEMO_sortingBubble_C66x

   \brief   Bubble sort function in optimized C66x code, in ascend order.

   \param[in]    len
               Length of the input vector.

   \param[in,out]   inputPtr
               Input vector, output will be stored in place.

   \param[out]    indxArray
               Output index array for the sorted vector, must be aligned to 8-byte bounday.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_sortingBubble_C66x(
                            IN  uint32_t len,
							INOUT  float * inputPtr,
							OUT int16_t * indxArray)
{

	int32_t i, j, indx, flag;
	float ftemp;

	for( i = 0; i < (int32_t) len; i++ )
	{
		indxArray[i] = i;
	}
	for( i = 0; i < (int32_t)len - 1; i++ )
	{
		flag  	=	0;
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
				flag  			=  	1;
			}
		}
		if (flag == 0)
			break;
	}
}

/*!
   \fn     RADARDEMO_osSimple_C66x

   \brief   Simple ordered statistics function in optimized C66x, in ascend order.

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

void	RADARDEMO_osSimple_C66x(
                            IN  uint32_t len,
							IN  uint32_t rank,
							INOUT  float * inputPtr,
							OUT int16_t * indxArray)
{
	int32_t i, j, k, indx, itemp, indx1, indx2, indx3, tempLen;
	float min, ftemp, min1, min2, min3;
	__float2_t f2input;

	for( i = 0; i < (int32_t) len; i+=2 )
	{
		_amem4(&indxArray[i]) = _pack2(i+1, i);
	}
	for( i = 0; i < (int32_t) rank; i++ )
	{
		if (i&1)
		{
			min		=	inputPtr[i];
			indx 	=	i;
			k		=	1;
		}
		else
		{
			min 		=	1.0e38;
			indx		=	0;
			k			=	0;
		}
		min1	=	min;
		indx1	=	indx;
		min2	=	min;
		indx2	=	indx;
		min3	=	min;
		indx3	=	indx;
		tempLen =	len - ((i+k) & 3);
		for (j = i + k; j < tempLen; j+=4)
		{
			f2input		=	_amem8_f2(&inputPtr[j]);
			if(_hif2(f2input) < min)
			{
				min		=	_hif2(f2input);
				indx 	=	j + 1;
			}

			if(_lof2(f2input) < min1)
			{
				min1	=	_lof2(f2input);
				indx1 	=	j;
			}
			f2input		=	_amem8_f2(&inputPtr[j + 2]);
			if(_hif2(f2input) < min2)
			{
				min2	=	_hif2(f2input);
				indx2 	=	j + 3;
			}

			if(_lof2(f2input) < min3)
			{
				min3	=	_lof2(f2input);
				indx3 	=	j + 2;
			}
		}
		if (tempLen != len)
		{
			f2input		=	_amem8_f2(&inputPtr[j]);
			if(_hif2(f2input) < min)
			{
				min		=	_hif2(f2input);
				indx 	=	j+1;
			}

			if(_lof2(f2input) < min1)
			{
				min1	=	_lof2(f2input);
				indx1 	=	j;
			}
		}
		if(min1 < min)
		{
			min 		=	min1;
			indx		=	indx1;
		}
		if(min2 < min)
		{
			min 		=	min2;
			indx		=	indx2;
		}
		if(min3 < min)
		{
			min 		=	min3;
			indx		=	indx3;
		}
		ftemp 			=	inputPtr[i];
		inputPtr[i]		=	inputPtr[indx];
		inputPtr[indx]	=	ftemp;
		itemp			=	indxArray[i];
		indxArray[i]	=	indxArray[indx];
		indxArray[indx]	=	itemp;
	}
}


static void topDownMerge16FinalStage(
		int16_t * RESTRICT A,
		int32_t iBegin,
		int32_t iMiddle,
		int32_t iEnd,
		int16_t * RESTRICT B,
		int16_t * RESTRICT indxA,
		int16_t * RESTRICT indxB)
{
	int32_t i, j, k, flag;

    i 	=	iBegin;
    j 	= 	iMiddle;


    // While there are elements in the left or right runs...
    for (k = iBegin; k < iEnd; k++)
    {
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
    }
}

static void topDownMerge16FinalStageV2(
		int16_t * RESTRICT A,
		int32_t iBegin,
		int32_t iMiddle,
		int32_t iEnd,
		int16_t * RESTRICT B,
		int16_t * RESTRICT indxA,
		int16_t * RESTRICT indxB)
{
	int32_t i, j, k, flag;
	int32_t in_i, in_j, index_i, index_j, min, min_index, itemp;

    i 	=	iBegin;
    j 	= 	iMiddle;


    // While there are elements in the left or right runs...
    for (k = iBegin; k < iEnd; k+=2)
    {
		in_i				=	_mem4(&A[i]);
		in_j				=	_mem4(&A[j]);
		index_i				=	_mem4(&indxA[i]);
		index_j				=	_mem4(&indxA[j]);

		itemp				=	_cmplt2(in_i, in_j);

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
    }
}

static void topDownMerge16(
		int16_t * RESTRICT A,
		int32_t iBegin,
		int32_t iMiddle,
		int32_t iEnd,
		int16_t * RESTRICT B,
		int16_t * RESTRICT indxA,
		int16_t * RESTRICT indxB)
{
	int32_t i1, j1, i2, j2, k, flag, iMiddleNew, iMiddle1, iMiddle2, iEnd1, iEnd2, delta;

	DEBUG(fprintf(testout1, "B = 0x%x\n", (uint32_t)B));
	DEBUG(fprintf(testout1, "A = 0x%x\n", (uint32_t)A));
	//DEBUG(fprintf(testout1, "indxB = 0x%x\n", (uint32_t)indxB));
	//DEBUG(fprintf(testout1, "indxA = 0x%x\n", (uint32_t)indxA));
	DEBUG(fprintf(testout1, "iBegin = %d\n", (uint32_t)iBegin));
	DEBUG(fprintf(testout1, "iEnd = %d\n", (uint32_t)iEnd));
	//DEBUG(fprintf(testout1, "iMiddle = %d\n", (uint32_t)iMiddle));

	iMiddleNew	=	(iEnd + iBegin) / 2;
	iEnd1		=	iMiddleNew;
	iEnd2		=	iEnd;
	iMiddle1	=	(iBegin + iEnd1) / 2;
	iMiddle2	=	(iEnd2 + iMiddleNew) / 2;

    i1 		=	iBegin;
    j1 		= 	iMiddle1;

    i2 	=	iMiddleNew;
    j2 	= 	iMiddle2;

	delta	=	iMiddleNew - iBegin;

	{
		// While there are elements in the left or right runs...
		for (k = iBegin; k < iMiddleNew; k++)
		{
    		flag 				=	(A[i1] <= A[j1]);
    		flag 				|=	(j1 >= iEnd1);
    		flag 				&=	(i1 < iMiddle1);
			//DEBUG(fprintf(testout1, "flag1 = %d\n", flag));
    		if (flag)
    			B[k] 			= 	A[i1];
    		else
    			B[k] 			= 	A[j1];

    		if (flag)
    			indxB[k]		= 	indxA[i1];
    		else
    			indxB[k]		= 	indxA[j1];

    		if(flag)
    			i1++;
    		else
    			j1++;

    		flag 				=	(A[i2] <= A[j2]);
    		flag 				|=	(j2 >= iEnd2);
    		flag 				&=	(i2 < iMiddle2);
			//DEBUG(fprintf(testout1, "flag2 = %d\n", flag));
    		if (flag)
    			B[k + delta] 		= 	A[i2];
    		else
    			B[k + delta] 		= 	A[j2];

    		if (flag)
    			indxB[k + delta]	= 	indxA[i2];
    		else
    			indxB[k + delta]	= 	indxA[j2];

    		if(flag)
    			i2++;
    		else
    			j2++;

		}
	}
	for (k = iBegin; k < iEnd; k++)
	{
		DEBUG(fprintf(testout1, "B[%d] = %f\n", k, B[k]));
	}
}

static void topDownMerge16V2(
		int16_t * RESTRICT A,
		int32_t iBegin,
		int32_t iMiddle,
		int32_t iEnd,
		int16_t * RESTRICT B,
		int16_t * RESTRICT indxA,
		int16_t * RESTRICT indxB)
{
	int32_t i1, j1, i2, j2, k, flag, iMiddleNew, i1Bound, i2Bound, j1Bound, j2Bound, delta;

	DEBUG(fprintf(testout1, "B = 0x%x\n", (uint32_t)B));
	DEBUG(fprintf(testout1, "A = 0x%x\n", (uint32_t)A));
	//DEBUG(fprintf(testout1, "indxB = 0x%x\n", (uint32_t)indxB));
	//DEBUG(fprintf(testout1, "indxA = 0x%x\n", (uint32_t)indxA));
	DEBUG(fprintf(testout1, "iBegin = %d\n", (uint32_t)iBegin));
	DEBUG(fprintf(testout1, "iEnd = %d\n", (uint32_t)iEnd));
	//DEBUG(fprintf(testout1, "iMiddle = %d\n", (uint32_t)iMiddle));

	iMiddleNew	=	(iEnd + iBegin) / 2;
	i1Bound		=	2 * iMiddleNew - 4;
	j1Bound		=	2 * iMiddleNew - 3;
	i2Bound		=	2 * iMiddleNew - 2;
	j2Bound		=	2 * iMiddleNew - 1;

    i1 		=	0;
    j1 		= 	1;

    i2 		=	2;
    j2 		= 	3;

	delta	=	(iEnd + iBegin) / 2;

	{
		// While there are elements in the left or right runs...
		for (k = iBegin; k < iMiddleNew; k++)
		{
    		flag 				=	(A[i1] <= A[j1]);
    		flag 				|=	(j1 > j1Bound);
    		flag 				&=	(i1 <= i1Bound);
			//DEBUG(fprintf(testout1, "flag1 = %d\n", flag));
    		if (flag)
    			B[k] 			= 	A[i1];
    		else
    			B[k] 			= 	A[j1];

    		if (flag)
    			indxB[k]		= 	4 * indxA[i1] + 0;
    		else
    			indxB[k]		= 	4 * indxA[j1] + 1;

    		if(flag)
    			i1 += 4;
    		else
    			j1 += 4;

    		flag 				=	(A[i2] <= A[j2]);
    		flag 				|=	(j2 > j2Bound);
    		flag 				&=	(i2 <= i2Bound);
			//DEBUG(fprintf(testout1, "flag2 = %d\n", flag));
    		if (flag)
    			B[k + delta] 		= 	A[i2];
    		else
    			B[k + delta] 		= 	A[j2];

    		if (flag)
    			indxB[k + delta]	= 	4 * indxA[i2] + 2;
    		else
    			indxB[k + delta]	= 	4 * indxA[j2] + 3;

    		if(flag)
    			i2 += 4;
    		else
    			j2 += 4;

		}
	}
	for (k = iBegin; k < iEnd; k++)
	{
		DEBUG(fprintf(testout1, "B[%d] = %f\n", k, B[k]));
	}
}


/*!
   \fn     RADARDEMO_sorting16_C66x

   \brief   Sort function in optimized C66x for 16-bit fixed point input, in ascend order.

   \param[in]    len
               Length of the input vector, must be multiple of 8.

   \param[in,out]   inputPtr
               Input vector, contents will be corrupted. Must be aligned to 8-byte boundary.

   \param[out]   outputPtr
               Output vector.

   \param[out]    indxArray
               Output index array for the sorted vector, of length 2*len, with second half used for temperary storage.
			   Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY, length must be multiple of 8.

   \post      none


 */

void	RADARDEMO_sorting16_C66x(
                            IN  uint32_t len,
							IN  int16_t * RESTRICT inputPtr,
							OUT  int16_t * RESTRICT outputPtr,
							OUT int16_t * RESTRICT indxArray)
{
	int32_t i, j, itemp, itemp1, lenOver4, lenOver8_;
	int16_t * RESTRICT tempIndBuf, * RESTRICT indxArray1;
	int64_t llMax, llMaxIdx, llinput, llmask;
	int64_t llMax1, llMaxIdx1, llinput1, llmask1;

	
	//step 1: simple sort equally divided 4 sections
	tempIndBuf		=	(int16_t *)&indxArray[0];
	indxArray1		=	(int16_t *)&indxArray[len];
	for( i = 0; i < (int32_t) (len>>2); i++ )
	{
		indxArray1[4 * i]		=	i;
		indxArray1[4 * i + 1]	=	i;
		indxArray1[4 * i + 2]	=	i;
		indxArray1[4 * i + 3]	=	i;
	}
	lenOver4			=	 (int32_t) (len>>2);
	for( i = 0; i < lenOver4; i++ )
	{
		llMax			=	_amem8(&inputPtr[0]);
		llMaxIdx		=	0;
		llMax1			=	_amem8(&inputPtr[4]);
		llMaxIdx1		=	0x0001000100010001;
		lenOver8_		=	lenOver4 >> 1;
		for (j = 1; j < lenOver8_; j++ )
		{
			llinput		=	_amem8(&inputPtr[8 * j]);
			llinput1	=	_amem8(&inputPtr[8 * j + 4]);
			itemp		=	_dcmpgt2(llinput, llMax);
			itemp1		=	_dcmpgt2(llinput1, llMax1);
			llmask		=	_dxpnd2(itemp);
			llmask1		=	_dxpnd2(itemp1);
			llMax		=	_dmax2(llinput, llMax);
			llMax1		=	_dmax2(llinput1, llMax1);
			llMaxIdx	=	(_amem8(&indxArray1[8 * j]) & llmask) | (llMaxIdx & (0xFFFFFFFFFFFFFFFF ^ llmask));
			llMaxIdx1	=	(_amem8(&indxArray1[8 * j + 4]) & llmask1) | (llMaxIdx1 & (0xFFFFFFFFFFFFFFFF ^ llmask1));
		}
		if(0)
		//if ((lenOver4 - i ) & 1)
		{
			llinput		=	_amem8(&inputPtr[8 * j]);
			itemp		=	_dcmpgt2(llinput, llMax);
			llmask		=	_dxpnd2(itemp);
			llMax		=	_dmax2(llinput, llMax);
			llMaxIdx	=	(_amem8(&indxArray1[8 * j]) & llmask) | (llMaxIdx & (0xFFFFFFFFFFFFFFFF ^ llmask));
		}
		itemp			=	_dcmpgt2(llMax, llMax1);
		llmask			=	_dxpnd2(itemp);
		llMax			=	_dmax2(llMax, llMax1);
		llMaxIdx		=	(llMaxIdx & llmask) | (llMaxIdx1 & (0xFFFFFFFFFFFFFFFF ^ llmask));

		llMaxIdx		=	(llMaxIdx << 2) + 0x0003000200010000;

		tempIndBuf[lenOver4 - i - 1]		=	_ext(_loll(llMaxIdx), 16, 16);
		tempIndBuf[2 * lenOver4 - i - 1]	=	_ext(_loll(llMaxIdx), 0, 16);
		tempIndBuf[3 * lenOver4 - i - 1]	=	_ext(_hill(llMaxIdx), 16, 16);
		tempIndBuf[4 * lenOver4 - i - 1]	=	_ext(_hill(llMaxIdx), 0, 16);

		outputPtr[lenOver4 - i - 1]			=	_ext(_loll(llMax), 16, 16);
		outputPtr[2 * lenOver4 - i - 1]		=	_ext(_loll(llMax), 0, 16);
		outputPtr[3 * lenOver4 - i - 1]		=	_ext(_hill(llMax), 16, 16);
		outputPtr[4 * lenOver4 - i - 1]		=	_ext(_hill(llMax), 0, 16);


		inputPtr[_ext(_loll(llMaxIdx), 16, 16)]	=	-32768;
		inputPtr[_ext(_loll(llMaxIdx), 0, 16)]	=	-32768;
		inputPtr[_ext(_hill(llMaxIdx), 16, 16)]	=	-32768;
		inputPtr[_ext(_hill(llMaxIdx), 0, 16)]	=	-32768;
	}

	//merge 4 sorted subgroups
	topDownMerge16(outputPtr, 0, (len>>1), len, inputPtr, tempIndBuf, indxArray1);
	topDownMerge16FinalStage(inputPtr, 0, (len>>1), len, outputPtr, indxArray1, tempIndBuf);
}


/*!
   \fn     RADARDEMO_sorting16_C66xV2

   \brief   Sort function in optimized C66x for 16-bit fixed point input, in ascend order.

   \param[in]    len
               Length of the input vector, must be multiple of 8.

   \param[in,out]   inputPtr
               Input vector, contents will be corrupted. Must be aligned to 8-byte boundary.

   \param[out]   outputPtr
               Output vector.

   \param[out]    indxArray
               Output index array for the sorted vector, of length 2*len, with second half used for temperary storage.
			   Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY, length must be multiple of 8.

   \post      none


 */

void	RADARDEMO_sorting16_C66xV2(
                            IN  uint32_t len,
							IN  int16_t * RESTRICT inputPtr,
							OUT  int16_t * RESTRICT outputPtr,
							OUT int16_t * RESTRICT indxArray)
{
	int32_t i, j, itemp, itemp1, lenOver4, lenOver8_;
	int16_t * RESTRICT tempIndBuf, * RESTRICT indxArray1;
	int64_t llMax, llMaxIdx, llinput, llmask;
	int64_t llMax1, llMaxIdx1, llinput1, llmask1, llindex, llindex1;

	
	//step 1: simple sort equally divided 4 sections
	tempIndBuf		=	(int16_t *)&indxArray[len];
	indxArray1		=	(int16_t *)&indxArray[0];
	for( i = 0; i < (int32_t) (len>>2); i++ )
	{
		indxArray1[4 * i]		=	i;
		indxArray1[4 * i + 1]	=	i;
		indxArray1[4 * i + 2]	=	i;
		indxArray1[4 * i + 3]	=	i;
	}
	lenOver4			=	 (int32_t) (len>>2);
	for( i = 0; i < lenOver4 - 1; i++ )
	{
		llMax			=	_amem8(&inputPtr[0]);
		llMaxIdx		=	0;
		llMax1			=	_amem8(&inputPtr[4]);
		llMaxIdx1		=	0x0001000100010001;

		llindex         =	0x0002000200020002;
		llindex1        =	0x0003000300030003;
		lenOver8_		=	(lenOver4 - i) >> 1;

		for (j = 1; j < lenOver8_; j++ )
		{
			llinput		=	_amem8(&inputPtr[8 * j]);
			llinput1	=	_amem8(&inputPtr[8 * j + 4]);
			itemp		=	_dcmpgt2(llinput, llMax);
			itemp1		=	_dcmpgt2(llinput1, llMax1);
			llmask		=	_dxpnd2(itemp);
			llmask1		=	_dxpnd2(itemp1);
			llMax		=	_dmax2(llinput, llMax);
			llMax1		=	_dmax2(llinput1, llMax1);
			llMaxIdx	=	(llindex & llmask) | (llMaxIdx & (0xFFFFFFFFFFFFFFFF ^ llmask));
			llMaxIdx1	=	(llindex1 & llmask1) | (llMaxIdx1 & (0xFFFFFFFFFFFFFFFF ^ llmask1));
			llindex		=	llindex + 0x0002000200020002;
			llindex1	=	llindex1 + 0x0002000200020002;
		}
		//if(0)
		if ((lenOver4 - i ) & 1)
		{
			llinput		=	_amem8(&inputPtr[8 * j]);
			itemp		=	_dcmpgt2(llinput, llMax);
			llmask		=	_dxpnd2(itemp);
			llMax		=	_dmax2(llinput, llMax);
			llMaxIdx	=	(llindex & llmask) | (llMaxIdx & (0xFFFFFFFFFFFFFFFF ^ llmask));
			llindex		=	llindex + 0x0002000200020002;
		}
		itemp			=	_dcmpgt2(llMax, llMax1);
		llmask			=	_dxpnd2(itemp);
		llMax			=	_dmax2(llMax, llMax1);
		llMaxIdx		=	(llMaxIdx & llmask) | (llMaxIdx1 & (0xFFFFFFFFFFFFFFFF ^ llmask));

		llMaxIdx1		=	(llMaxIdx << 2) + 0x0003000200010000;

		//swap data and indices
		itemp			=	indxArray1[_ext(_loll(llMaxIdx1), 16, 16)];
		indxArray1[_ext(_loll(llMaxIdx1), 16, 16)]	=	indxArray1[4 * (lenOver4 - i) - 4];
		indxArray1[4 * (lenOver4 - i) - 4]			=	itemp;

		itemp			=	indxArray1[_ext(_loll(llMaxIdx1), 0, 16)];
		indxArray1[_ext(_loll(llMaxIdx1), 0, 16)]	=	indxArray1[4 * (lenOver4 - i) - 3];
		indxArray1[4 * (lenOver4 - i) - 3]			=	itemp;

		itemp			=	indxArray1[_ext(_hill(llMaxIdx1), 16, 16)];
		indxArray1[_ext(_hill(llMaxIdx1), 16, 16)]	=	indxArray1[4 * (lenOver4 - i) - 2];
		indxArray1[4 * (lenOver4 - i) - 2]			=	itemp;

		itemp			=	indxArray1[_ext(_hill(llMaxIdx1), 0, 16)];
		indxArray1[_ext(_hill(llMaxIdx1), 0, 16)]	=	indxArray1[4 * (lenOver4 - i) - 1];
		indxArray1[4 * (lenOver4 - i) - 1]			=	itemp;

		inputPtr[_ext(_loll(llMaxIdx1), 16, 16)]	=	inputPtr[4 * (lenOver4 - i) - 4];
		inputPtr[_ext(_loll(llMaxIdx1), 0, 16)]	=	inputPtr[4 * (lenOver4 - i) - 3];
		inputPtr[_ext(_hill(llMaxIdx1), 16, 16)]	=	inputPtr[4 * (lenOver4 - i) - 2];
		inputPtr[_ext(_hill(llMaxIdx1), 0, 16)]	=	inputPtr[4 * (lenOver4 - i) - 1];

		inputPtr[4 * (lenOver4 - i) - 4]			=	_ext(_loll(llMax), 16, 16);
		inputPtr[4 * (lenOver4 - i) - 3]		=	_ext(_loll(llMax), 0, 16);
		inputPtr[4 * (lenOver4 - i) - 2]		=	_ext(_hill(llMax), 16, 16);
		inputPtr[4 * (lenOver4 - i) - 1]		=	_ext(_hill(llMax), 0, 16);

	}

	//merge 4 sorted subgroups
	topDownMerge16V2(inputPtr, 0, (len>>1), len, outputPtr, indxArray1, tempIndBuf);
	topDownMerge16FinalStage(outputPtr, 0, (len>>1), len, inputPtr, tempIndBuf, indxArray1);
}
