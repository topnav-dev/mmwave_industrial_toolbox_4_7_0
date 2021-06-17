/**
 *   @file  Util_ClassifierkNN_inputParser.c
 *
 *   @brief
 *      Utility function for parsing input data for classifier-kNN test.
 *      The format has to be the same as the format of
 *      People counting/Traffic monitoring output.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
 */
#ifndef UTIL_CLASSIFIERKNN_INPUTPARSER_H
#define UTIL_CLASSIFIERKNN_INPUTPARSER_H
#include "swpform.h"
#include <math.h>
 
#include <modules/classification/featureExtraction/api/RADARDEMO_featExtraction.h>
#include <modules/classification/classifier_kNN/utils/Util_input_message.h>

#define MAX_POWER (100000.f)
#define MAX_AZIMUTH (90.f)

/**
 * \enum Util_ClassifierkNN_inputParser_ErrorCode
 *  \brief   enum for knn input parser error code.
 */

typedef enum
{
	UTIL_CLASSIFIERKNN_INPUTPARSER_NOERROR = 0,				/**< No error */
	UTIL_CLASSIFIERKNN_INPUTPARSER_WRONG_TYPES,				/**< Invalid input types*/
	UTIL_CLASSIFIERKNN_INPUTPARSER_WRONG_PC_LEN,			/**< Invalid point cloud length. */ 
	UTIL_CLASSIFIERKNN_INPUTPARSER_WRONG_TL_LEN				/**< Invalid target list length. */ 
} Util_ClassifierkNN_inputParser_ErrorCode;


/*!
 *   \fn     Util_ClassifierkNN_inputParser
 *
 *   \brief   Utility function to parse input from the TLV to the format for feature extraction for kNN classifier.
 *
 *   \param[in]    pointcloudTLV
 *               Point to the input point cloud TLV for the current frame.
 *
 *   \param[in]    targetListTLV
 *               Pointer to the input target list TLV for the current frame.
 *
 *   \param[in]    targetIndexTLV
 *               Pointer to the input target index TLV for the current frame.
 *
 *   \param[in]    maxNumDetPnts
 *               Input maximum number of detected points per frame. 
 *
 *   \param[in]    maxNumTrks
 *               Input maximum number of tracks supported in the system. 
 *
 *   \param[in]    maxDoppler
 *               Input maximum doppler from the low level processing chain, for input validation purposes. 
 *
 *   \param[in]    maxDoppler
 *               Input maximum range from the low level processing chain, for input validation purposes. 
 *
 *   \param[in]    xBuff
 *               Input buffer hold x pos info for all the points from point cloud, stored in sequence of active tracks. 
 *
 *   \param[in]    yBuff
 *               Input buffer hold y pos info for all the points from point cloud, stored in sequence of active tracks. 
 *
 *   \param[in]    dopplerBuff
 *               Input buffer hold doppler info for all the points from point cloud, stored in sequence of active tracks. 
 *
 *   \param[in]    powerBuff
 *               Input buffer hold power info for all the points from point cloud, stored in sequence of active tracks. 
 *
 *   \param[out]    parsedData
 *               Pointer to the parsed output data. It's a pointer the the array 
 *               of RADARDEMO_featExtract_input strauctures with size of maximum number of tracks 
 *               supported by the system. See \ref RADARDEMO_featExtract_input for details. 
 *
 *   \ret        Error code, see \ref Util_ClassifierkNN_inputParser_ErrorCode for details.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */

extern Util_ClassifierkNN_inputParser_ErrorCode Util_ClassifierkNN_inputParser(
			IN void * pointcloudTLV,
			IN void * targetListTLV,
			IN void * targetIndexTLV,
			IN uint32_t maxNumDetPnts,
			IN uint32_t maxNumTrks,
			IN float maxDoppler,
			IN float maxrange,
			IN float *xBuff,
			IN float *yBuff,
			IN float *dopplerBuff,
			IN float *powerBuff,
			OUT RADARDEMO_featExtract_input * parsedData
		);

#ifdef OPTIMIZED4DSP
#ifndef SINSP_I_
#define SINSP_I_ 1

static inline float sinsp_i (float a)
{
  float InvPI =  0.318309886183791;
  float One   =  1.0;
  float MAX   =  1048576.0;
  float Zero  =  0.0;
  float s1    = -1.666665668e-1;
  float s2    =  8.333025139e-3;
  float s3    = -1.980741872e-4;
  float s4    =  2.601903036e-6;
  float C1    =  3.140625;
  float C2    =  9.67653589793e-4;
  float Sign, X, Y, Z, F, G, R;
  int   N;

  Sign = One;
  Y    = a;

  if (_fabsf(Y) > MAX) {
    Y = Zero;
  }

  X = Y * InvPI;            /* X = Y * (1/PI)  */
  N = _spint(X);            /* N = integer part of X  */
  Z = (float) N;

  if ((N & 1) != 0.0f) {
    Sign = -Sign;           /* Quadrant 3 or 4 */
  }

  F = (Y - Z*C1) - Z*C2;
  G = F * F;
  R = (((s4*G + s3)*G + s2)*G + s1)*G;

  return ((F + F*R) * Sign);
}

#endif

#ifndef COSSP_I_
#define COSSP_I_ 1
static inline float cossp_i (float a)
{
  float   Zero   =  0.0;
  float   MAX    =  1048576.0;
  float   MIN    =  2.4414062e-4;
  float   Sign   =  1;
  float   InvPI  =  0.318309886183791;
  float	  HalfPI =  1.5707963268;
  float   s4     =  2.601903036e-6;
  float   s3     = -1.980741872e-4;
  float   s2     =  8.333025139e-3;
  float   s1     = -1.666665668e-1;
  float   C1     =  3.140625;
  float   C2     =  9.67653589793e-4;
  float   X, Y, Z, F, G, R;
  int     N;

  Y = _fabsf(a) + HalfPI;

  if (Y > MAX) {
    Y = HalfPI;
  }

  X = Y * InvPI;            /* X = Y * (1/PI)         */
  N = _spint(X);            /* N = integer part of X  */
  Z = N;                    /* Z = float (N)          */

  if ((N&1) != 0) {
    Sign = -Sign;           /* quad. 3 or 4   */
  }

  F = (Y - (Z*C1)) - (Z*C2);
  R = F;

  if (F < Zero) {
    R = -R;
  }

  if (R < MIN) {
    return (R*Sign);
  }

  G = F*F;
  R = (((s4*G+s3)*G+s2)*G+s1)*G;

  return ((F + F*R)*Sign);
}

#endif /* COSSP_I_ */
#endif //OPTIMIZED4DSP

#endif //UTIL_CLASSIFIERKNN_INPUTPARSER_H
