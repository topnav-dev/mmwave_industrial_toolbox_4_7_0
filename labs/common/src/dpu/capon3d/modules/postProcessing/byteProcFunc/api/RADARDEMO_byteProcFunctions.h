/*! 
 *  \file   RADARDEMO_byteProcFunctions.h
 *
 *  \brief   Header file for byte processing functions
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

#ifndef RADARDEMO_BYTEPROC_H
#define RADARDEMO_BYTEPROC_H

#include <swpform.h>

/*!
   \fn     RADARDEMO_convKernel_generic

   \brief   2D convolution kernel in generic C. 
   
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

void	RADARDEMO_convKernel_generic(
                            IN  uint32_t sizeX,
                            IN  uint32_t sizeY,
							IN  uint8_t * inputPtr,
							IN  uint8_t * inputFilt,
							OUT uint8_t * outputPtr);


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
							OUT uint8_t * outputPtr);


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
							OUT uint8_t * outputPtr);


/*!
   \fn     RADARDEMO_morphErosion_generic2

   \brief   Morphological erosion operation in generic C, simple version. 
   
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
							OUT uint8_t * outputPtr);

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
							OUT uint8_t * outputPtr);

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
							OUT uint8_t * outputPtr);


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
               Input 2D neighborhood, assuming always 5x5 size (hardcoded) bit indicator packed in lower 5-bit of a byte.

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
							OUT uint8_t * outputPtr);


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
							OUT uint8_t * outputPtr);
#endif //RADARDEMO_BYTEPROC_H

