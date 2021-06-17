/*! 
 *  \file   RADARDEMO_sorting.h
 *
 *  \brief   Header file for sorting functions
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

#ifndef RADARDEMO_SORTING_H
#define RADARDEMO_SORTING_H

#include <swpform.h>

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
							IN  float * inputPtr,
							INOUT int16_t * indxBuf,
							OUT float * outputPtr);


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
							OUT float * inputPtr);

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
							OUT int16_t * indxArray);

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
							OUT int16_t * indxArray);


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
							OUT int16_t * indxArray);

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
							OUT int16_t * indxArray);


/*!
   \fn     RADARDEMO_sortingSimple_generic

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
							OUT int16_t * indxArray);

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
							IN  uint32_t k,
							INOUT  float * inputPtr,
							OUT int16_t * indxArray);


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
							OUT int16_t * indxArray);


/*!
   \fn     RADARDEMO_sorting16_C66x

   \brief   Sort function in optimized C66x for 16-bit fixed point input, in ascend order.

   \param[in]    len
               Length of the input vector.

   \param[in,out]   inputPtr
               Input vector, contents will be corrupted.

   \param[out]   outputPtr
               Output vector.

   \param[out]    indxArray
               Output index array for the sorted vector, of length 2*len, with second half used for temperary storage.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_sorting16_C66x(
                            IN  uint32_t len,
							IN  int16_t * inputPtr,
							OUT  int16_t * outputPtr,
							OUT int16_t * indxArray);


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
							OUT int16_t * RESTRICT indxArray);
#endif //RADARDEMO_BYTEPROC_H



