/*! 
 *  \file   radar_commonMath.h
 *
 *  \brief   Inline functions for math utilities.
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

#ifndef _RADARDEMO_COMMONMATH_H
#define _RADARDEMO_COMMONMATH_H

#include <people_counting/68xx_3D_people_counting/src/common/swpform.h>

INLINE double divdp_i (double a, double b)
{
  double  X;
  double  TWO   =  2.0;

  X = _rcpdp(b);
  X = X * (TWO - b*X);
  X = X * (TWO - b*X);
  X = X * (TWO - b*X);
  X = X * a;

  if (a == 0.0) {
    X = 0.0;
  }
  return (X);
}

INLINE double sindp_i (double a)
{
  double  InvPI =  0.31830988618379067154;
  double  C1    =  3.1416015625;
  double  C2    = -8.908910206761537356617e-6;
  double  r8    =  2.7204790957888846175e-15;
  double  r7    = -7.6429178068910467734e-13;
  double  r6    =  1.6058936490371589114e-10;
  double  r5    = -2.5052106798274584544e-8;
  double  r4    =  2.7557319210152756119e-6;
  double  r3    = -1.9841269841201840457e-4;
  double  r2    =  8.3333333333331650314e-3;
  double  r1    = -1.6666666666666665052e-1;
  double  MAX   =  1.073741824e+09;
  double  Zero  =  0.0;
  double  Sign  =  1.0;
  double  X, Z, F1, F2, G, R;
  int     N;

  F1 = a;

  if (_fabs(F1) > MAX) {
    F1 = Zero;
  }
      
  X = F1 * InvPI;           /* X = Y/PI  */
  N = _dpint(X);
  Z = (double) N;                         
  
  if ((N & 1) != 0) {
    Sign = -Sign;           /* neg. quadrants  */
  }

  F1 = (F1 - Z*C1) - Z*C2;
  F2 = F1*F1;
  G  = F2*F2;
        
  R = (((G*r8 + r6)*G + r4)*G + r2)*G;
  X = (((G*r7 + r5)*G + r3)*G + r1)*F2;
  R =  R + X;
  G = (F1 + F1*R) * Sign;  

  return (G);           
}

INLINE double cosdp_i (double a)
{
  double  C1     =  3.1416015625;
  double  C2     = -8.908910206761537356617e-6;
  double  r8     =  2.7204790957888846175e-15;
  double  r7     = -7.6429178068910467734e-13;
  double  r6     =  1.6058936490371589114e-10;
  double  r5     = -2.5052106798274584544e-8;
  double  r4     =  2.7557319210152756119e-6;
  double  r3     = -1.9841269841201840457e-4;
  double  r2     =  8.3333333333331650314e-3;
  double  r1     = -1.6666666666666665052e-1;
  double  MAX    =  1.073741824e+09;
  double  HalfPI =  1.57079632679489661923;
  double  InvPI  =  0.31830988618379067154;
  double  Sign   =  1.0;
  double  X, Z, F, F2, G, R;
  int     N;

  F = _fabs(a) + HalfPI;
  
  if (F > MAX) {
    F = HalfPI;
  }

  X = F * InvPI;            /* X = Y * (1/PI)  */
  N = _dpint(X);
  Z = (N);                  /* Z = double (N)  */
  
  if ((N&1) != 0) {
    Sign = -Sign; 	        /* neg. quadrants  */
  }
  
  F  = (F - (Z*C1)) - (Z*C2);
  R  = _fabs(F);
  F2 = F*F;
  G  = F2*F2;
  R = (((G*r8 + r6)*G + r4)*G + r2)*G;
	X = (((G*r7 + r5)*G + r3)*G + r1)*F2;
	R = R + X;
  G = (F + F*R)*Sign;  

  return (G);           
}

INLINE float cossp_i (float a)
{
  float   Zero   =  0.0; 
  float   MAX    =  1048576.0;
  float   MIN    =  (float)2.4414062e-4;
  float   Sign   =  1;
  float   InvPI  =  (float)0.318309886183791;
  float	  HalfPI =  (float)1.5707963268;
  float   s4     =  (float)2.601903036e-6;
  float   s3     = (float)-1.980741872e-4;
  float   s2     =  (float)8.333025139e-3;
  float   s1     = (float)-1.666665668e-1;
  float   C1     =  3.140625f;
  float   C2     =  (float)9.67653589793e-4;
  float   X, Y, Z, F, G, R;
  int     N;

  Y = _fabsf(a) + HalfPI;

  if (Y > MAX) {
    Y = HalfPI;
  }

  X = Y * InvPI;            /* X = Y * (1/PI)         */
  N = _spint(X);            /* N = integer part of X  */
  Z = (float)N;                    /* Z = float (N)          */
  
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


INLINE float sinsp_i (float a)
{
  float InvPI =  0.318309886183791f;
  float One   =  1.0f;
  float MAX   =  1048576.0f;
  float Zero  =  0.0f;
  float s1    = (float)-1.666665668e-1;
  float s2    =  (float)8.333025139e-3; 
  float s3    = (float)-1.980741872e-4;
  float s4    =  (float)2.601903036e-6; 
  float C1    =  (float)3.140625f;
  float C2    =  (float)9.67653589793e-4;
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


/* =========================================================================== */
/* The sqrtsp function returns the square root of a real floating-point value. */
/* =========================================================================== */

INLINE float sqrtsp_asinsp_i (float x)
{
  const float  half  = 0.5f;
  const float  OneP5 = 1.5f;
  float  y, y0, y1, y2, x_half;

  x_half =  x * half;
  y0 = _rsqrsp(x);                           /* y0 = 1/ sqrt(x) */

  y1 = OneP5 - (y0 * y0 * x_half);
  y1 = y0 * y1;
  y2 = y1*(OneP5 - (y1 * y1 * x_half));
  y = x * y2;

  if (x <= 0.0f) {
    y = 0.0f;
  }

  return y;
} /* sqrtsp_asinsp_i */


/* ======================================================================== */
/* Polynomial calculation to estimate the arc_sine funtion.                 */
/* The polynomial used is as follows:                                       */
/*   pol = x + c2 x^3 + c4 x^5 + c6 x^7 + c8 x^9 + c10 x^11 + c12 x^13 +    */
/*          c14 x^15 + c16 x^17,                                            */
/* where x is the input, c2 through c16 are the corresponding coefficients  */
/* to the polynomial, and pol is the result of the polynomial. This         */
/* polynomial only covers inputs in the range [0, 1/sqrt(2)].               */
/* ======================================================================== */

INLINE float pol_est_asinsp_i (float x)
{
  /* coefficients for the polynomial for asin(x) */
  const float c16 =  0.053002771381990f;
  const float c14 = -0.010980624698693f;
  const float c12 =  0.020659425186833f;
  const float c10 =  0.022862784546374f;
  const float c8 =   0.030636056280974f;
  const float c6 =   0.044450959710588f;
  const float c4 =   0.075034659380970f;
  const float c2 =   0.166664771293503f;

  float x2, x4, x6, x8, x10, x12;
  float pol ,tmp1, tmp2;

  /* calculate the powers of x */
  x2  = x * x;
  x4  = x2 * x2;
  x6  = x2 * x4;
  x8  = x4 * x4;
  x10 = x6 * x4;
  x12 = x8 * x4;

  /* ====================================================================== */
  /* The polynomial calculation is done in two seperate parts.              */
  /*   tmp1 =  c2 x^2 + c4 x^4 + c6 x^6 + c8 x^8                            */
  /*   tmp2 =  c10 x^10 + c12 x^12 + c14 x^14 + c16 x^16                    */
  /* In order to reduce the number of multiplications x is factored out of  */
  /* the polynomial and multiplied by later.                                */
  /* ====================================================================== */

  tmp1 = ((c8 * x8) + (c6 * x6)) + ((c4 * x4) + (c2 * x2));
  tmp2 = (((c16 * x4 + c14 * x2) + c12) * x12) + (c10 * x10);

  pol = tmp1  + tmp2;
  pol = pol * x + x;

  return pol;
} /* pol_est_asinsp_i */



/* ====================================================================== */
/* The type of calculation for asin(x) depends on the value of x:         */
/*                                                                        */
/* for x_abs <= 1/sqrt(2), res = pol_est_asinsp_i (input x)               */
/* for x_abs > 1/sqrt(2),  res = pi/2 - pol_est_asinsp_i (input a)        */
/*                         a = sqrt(1 - x^2)                              */
/* where x_abs is the absolute value of the input, a is calculated as     */
/* shown above and it's used as an input for the polynomial, and res is   */
/* the value for asin(x).                                                 */
/* ====================================================================== */

INLINE float asinsp_i (float x)
{
  const float pi2   = 1.570796327f;           /* pi/2 */
  const float rsqr2 = 0.7071067811f;
  float       s     = 1.0f;
  float res, x_abs, a, temp;

  x_abs = _fabsf(x);


  if(x_abs > rsqr2){                          /* |x| > 1/sqrt(2) */
    temp = 1.0f - (x_abs * x_abs);
    a = sqrtsp_asinsp_i(temp);                /* a= sqrt(1 - x^2) */
    temp = pol_est_asinsp_i(a);
    res = pi2 - temp;
  }
  else{                                       /* |x| <= 1/sqrt(2) */
    res = pol_est_asinsp_i(x_abs);
  }

  if(x < 0.0f){
    s = -s;                                   /* sign var */
  }

  if(x_abs > 1.0f){
    res = _itof(0x7FFFFFFF);                  /* NaN */
  }

  return (res*s);                              /* restore sign for quadrant 3 & 4*/
} /* asinsp_i */

INLINE float divsp_i (float a, float b)
{
  float TWO  = 2.0;
  float Maxe = 3.402823466E+38;
  float X;

  X = _rcpsp(b);
  X = X*(TWO - b*X);
  X = X*(TWO - b*X);
  X = a*X;

  if (a == 0.0f) {     
    X = 0.0f;
  }
  
  if (_fabsf(b) > Maxe && _fabs(a) <= Maxe) {     
    X = 0.0f;
  }
  
  return (X);
}

#endif //_RADARDEMO_COMMONMATH_H
