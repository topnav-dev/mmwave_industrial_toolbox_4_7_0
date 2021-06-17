/*!
 *  \file   RADARDEMO_svmKernelUtil_priv.h
 *
 *  \brief   Header file for RADARDEMO_svmKernelUtil_priv.c
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
#ifndef RADARDEMO_SVMKERNELUTIL_PRIV_H
#define RADARDEMO_SVMKERNELUTIL_PRIV_H

//! \brief include file <swpform.h>
//!
#include <swpform.h>

#ifdef _TMS320C6X
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#endif


/*!
 *   \fn     RADARDEMO_svmPredictLinearKernal
 *
 *   \brief   SVM prediction utility for linear kernal. 
 *
 *   \param[in]    numFeatPerInput
 *               number of features per input.
 *
 *   \param[in]    inputVector
 *               Pointer to the input vector, size of numFeatPerInput.
 *
 *   \param[in]    beta
 *               Pointer to the linear kernal vector, size of numFeatPerInput.
 *
 *   \param[in]    scale
 *               Input scale.
 *
 *   \param[in]    bias
 *               Input bias.
 *
 *   \param[out]    score
 *               Pointer to the output scores, if >= 0 decision = 1; otherwise decision = -1.
 *
 *  \ret		decision: output tags, 1 or -1.
 *
 *   \pre       1. Only support 2 class.
 *              2. Only support tags of 1 and -1. 
 *
 *   \post      none
 *
 *
 */
extern int8_t RADARDEMO_svmPredictLinearKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
                            IN  float *  RESTRICT beta,
							IN  float    scale,
							IN  float    bias,
							OUT float * RESTRICT score);

/*!
 *   \fn     RADARDEMO_svmPredictGaussianKernal
 *
 *
 *   \brief   SVM prediction utility for Gaussian kernal. 
 *
 *   \param[in]    numFeatPerInput
 *               number of features per input.
 *
 *   \param[in]    inputVector
 *               Pointer to the input vector, size of numFeatPerInput.
 *
 *   \param[in]    scale
 *               Input scale.
 *
 *   \param[in]    svLen
 *               Length of the support vector.
 *
 *   \param[in]    sv
 *               Pointer to the input support vector, in format of sv0feat0, sv0feat1, sv0feat2,...,sv1feat0,
 *               sv1feat1, sv1feat2, ..., svNfeatM...., size of svLen*numFeatPerInput.
 *
 *   \param[in]    svTag
 *               Pointer to the tags for the input support vector, size of svLen.
 *
 *   \param[in]    alpha
 *               Pointer to the weights for the input support vector, size of svLen.
 *
 *   \param[in]    bias
 *               Input bias.
 *
 *   \param[out]    score
 *               Pointer to the output scores, if >= 0 decision = 1; otherwise decision = -1.
 *
 *  \ret		decision: output tags, 1 or -1.
 *
 *   \pre       1. Only support 2 class.
 *              2. Only support tags of 1 and -1. 
 *
 *   \post      none
 *
 *
 */
extern int8_t RADARDEMO_svmPredictGaussianKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
							IN  float    scale,
                            IN  int16_t  svLen,
                            IN  float *  RESTRICT sv,
                            IN  int8_t *  RESTRICT svTag,
                            IN  float *  RESTRICT alpha,
							IN  float    bias,
							OUT float * RESTRICT score);
/*!
 *   \fn     RADARDEMO_svmPredictPolyKernal
 *
 *
 *   \brief   SVM prediction utility for polynomial kernal. 
 *
 *   \param[in]    numFeatPerInput
 *               number of features per input.
 *
 *   \param[in]    inputVector
 *               Pointer to the input vector, size of numFeatPerInput.
 *
 *   \param[in]    scale
 *               Input scale.
 *
 *   \param[in]    order
 *               Input polynomial order.
 *
 *   \param[in]    svLen
 *               Length of the support vector.
 *
 *   \param[in]    sv
 *               Pointer to the input support vector, in format of sv0feat0, sv0feat1, sv0feat2,...,sv1feat0,
 *               sv1feat1, sv1feat2, ..., svNfeatM...., size of svLen*numFeatPerInput.
 *
 *   \param[in]    svTag
 *               Pointer to the tags for the input support vector, size of svLen.
 *
 *   \param[in]    alpha
 *               Pointer to the weights for the input support vector, size of svLen.
 *
 *   \param[in]    bias
 *               Input bias.
 *
 *   \param[out]    score
 *               Pointer to the output scores, if >= 0 decision = 1; otherwise decision = -1.
 *
 *  \ret		decision: output tags, 1 or -1.
 *
 *   \pre       1. Only support 2 class.
 *              2. Only support tags of 1 and -1. 
 *
 *   \post      none
 *
 *
 */
extern int8_t RADARDEMO_svmPredictPolyKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
							IN  float    scale,
							IN  int8_t  order,
                            IN  int16_t  svLen,
                            IN  float *  RESTRICT sv,
                            IN  int8_t *  RESTRICT svTag,
                            IN  float *  RESTRICT alpha,
							IN  float    bias,
							OUT float * RESTRICT score);


#ifdef OPTIMIZED4DSP
#ifndef EXPSP_I_H_
#define EXPSP_I_H_ 1

extern double ti_math_kTable[4];
extern double ti_math_jTable[4];

inline float expsp_i (float a)
{
  float log2_base_x16 =   1.442695041 * 16.0;
  float Halfe         =   0.5f;
  float LnMine        = -87.33654475;
  float LnMaxe        =  88.72283905;
  float Maxe          =   3.402823466E+38;
  float c0            =   0.1667361910f;
  float c1            =   0.4999999651f;
  float c2            =   0.9999998881f;
  float P1            =   0.04331970214844;
  float P2            =   1.99663646e-6;
  float pol, r, r2, r3, res;
  unsigned int Ttemp;
  int          J, K, N;
  double       dT;

  /* Get N such that |N - x*16/ln(2)| is minimized */
  N = (int) (a * log2_base_x16 + Halfe);
  if ((a * log2_base_x16) < -Halfe) {
    N--;
  }

  /* Argument reduction, r, and polynomial approximation pol(r) */
  r  = (a - P1 * (float) N) - P2 * (float) N;
  r2 = r * r;
  r3 = r * r2;

  pol = (r * c2) + ((r3 * c0) + (r2 * c1));

  /* Get index for ktable and jtable */
  K  = _extu (N, 28, 30);
  J  = N & 0x3;
  dT = ti_math_kTable[K] * ti_math_jTable[J];

  /* Scale exponent to adjust for 2^M */
  Ttemp = _hi(dT) + (unsigned int) ((N >> 4) << 20);
  dT    = _itod(Ttemp, _lo(dT));

  res = (float) (dT * (1.0f + pol));

  /* Early exit for small a */
  if (_extu(_ftoi(a), 1, 24) < 114) {
    res = (1.0f + a);
  }

  /* < LnMin returns 0 */
  if (a < LnMine) {
    res = 0.0;
  }

  /* > LnMax returns MAX */
  if (a > LnMaxe) {
    res = Maxe;
  }

  return(res);
}

#endif

#if 0
//#ifndef POWSP_I_
//#define POWSP_I_ 1


static inline float logspMod_powspi (float a)
{
  double  ln2  =  0.693147180559945;
  float   MAXe =  3.402823466E+38;
  float   c1   = -0.2302894f;
  float   c2   =  0.1908169f;
  float   c3   = -0.2505905f;
  float   c4   =  0.3333164f;
  float   c5   = -0.5000002f;
  float   pol, r1, r2, r3, r4, res;
  double  dr, frcpax, rcp, T;
  int     N, T_index;

  /* r = x * frcpa(x) -1 */
  rcp = _rcpdp((double) a);
  frcpax = _itod(_clr(_hi(rcp),0,16), 0);
  dr = frcpax * (double) a  - 1.0;

  /* Polynomial p(r) that approximates ln(1+r) - r */
  r1 = (float) dr;
  r2 = r1*r1;
  r3 = r1*r2;
  r4 = r2*r2;

  pol  = c5*r2 + ((c4*r3) + ((c2*r1 + c3) + c1*r2)*r4);

  /* Reconstruction: result = T + r + p(r) */
  N       = _extu(_hi(frcpax),  1, 21) - 1023;
  T_index = _extu(_hi(frcpax), 12, 29);
  T       = ti_math_logtable[T_index] - ln2 * (double) N;
  res     = (dr + T) + (double) pol;

  if (a > MAXe) {
    res = 88.72283905313;
  }

  return (res);
}

static inline float expspMod_powspi (float a)
{
  float log2_base_x16 =   1.442695041 * 16.0;
  float Halfe         =   0.5f;
  float LnMine        = -87.33654475;
  float LnMaxe        =  88.72283905;
  float Maxe          =   3.402823466E+38;
  float c0            =   0.1667361910f;
  float c1            =   0.4999999651f;
  float c2            =   0.9999998881f;
  float P1            =   0.04331970214844;
  float P2            =   1.99663646e-6;
  float pol, r, r2, r3, res;
  unsigned int Ttemp;
  int          J, K, N;
  double       dT;

  /* Get N such that |N - x*16/ln(2)| is minimized */
  N = (int) (a * log2_base_x16 + Halfe);
  if ((a * log2_base_x16) < -Halfe) {
    N--;
  }

  /* Argument reduction, r, and polynomial approximation pol(r) */
  r  = (a - P1 * (float) N) - P2 * (float) N;
  r2 = r * r;
  r3 = r * r2;

  pol = (r * c2) + ((r3 * c0) + (r2 * c1));

  /* Get index for ktable and jtable */
  K  = _extu (N, 28, 30);
  J  = N & 0x3;
  dT = ti_math_kTable[K] * ti_math_jTable[J];

  /* Scale exponent to adjust for 2^M */
  Ttemp = _hi(dT) + (unsigned int) ((N >> 4) << 20);
  dT    = _itod(Ttemp, _lo(dT));

  res = (float) (dT * (1.0f + pol));

  /* Early exit for small a */
  if (_extu(_ftoi(a), 1, 24) < 114) {
    res = (1.0f + a);
  }

  /* < LnMin returns 0 */
  if (a < LnMine) {
    res = 0.0;
  }

  /* > LnMax returns MAX */
  if (a > LnMaxe) {
    res = Maxe;
  }

  return(res);
}

inline float powsp_i (float a, float b)
{
  float arg, W, X2;
  int   y, Sign = 1;

  y = _spint(b);

  if ((a < 0.0) & (b == (float)y)) {
    if (y & 1) {
      Sign = -1;
    }
  }

  arg = _fabsf(a);
  W   = b    * logspMod_powspi (arg);
  if (arg == 1.0) {
    W = 0.0;
  }
  X2  = Sign * expspMod_powspi (W);

  if ((a < 0.0) & (b != (float)y)) {
    X2 = _itof(0x7fffffff);
  }

  if (a == 0.0) {
    X2 = (b >= 0.0) ? 0.0 : _itof(0x7F800000);
  }

  if (b == 0.0) {
    X2 = 1.0;
  }

  return (X2);
}

#endif


#endif



#endif //RADARDEMO_SVMKERNELUTIL_PRIV_H

