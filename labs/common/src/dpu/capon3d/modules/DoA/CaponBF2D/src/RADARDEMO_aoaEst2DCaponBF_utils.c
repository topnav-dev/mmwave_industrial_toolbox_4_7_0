/*! 
 *  \file   RADARDEMO_aoaEst2DCaponBF_utils.c
 *
 *  \brief   Utility functions for FFT function. 
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
#ifdef _TMS320C6X
#include "c6x.h"
#endif
#include <common/src/dpu/capon3d/modules/utilities/radar_commonMath.h>

#ifndef _GENTWIDDLE_FFTFLOAT
#define _GENTWIDDLE_FFTFLOAT

#ifdef _TMS320C6600 //C66x

void tw_gen_float(float *w, int n)
{
    int i, j, k;
    const double PI = 3.141592654;

    for (j = 1, k = 0; j <= n >> 2; j = j << 2)
    {
        for (i = 0; i < n >> 2; i += j)
        {
#ifdef _LITTLE_ENDIAN
            w[k]     = (float) sindp_i (2 * PI * divdp_i((double)i,  (double)n));
            w[k + 1] = (float) cosdp_i (2 * PI * divdp_i((double)i,  (double)n));
            w[k + 2] = (float) sindp_i (4 * PI * divdp_i((double)i,  (double)n));
            w[k + 3] = (float) cosdp_i (4 * PI * divdp_i((double)i,  (double)n));
            w[k + 4] = (float) sindp_i (6 * PI * divdp_i((double)i,  (double)n));
            w[k + 5] = (float) cosdp_i (6 * PI * divdp_i((double)i,  (double)n));
#else
            w[k]     = (float)  cosdp_i (2 * PI * divdp_i((double)i,  (double)n));
            w[k + 1] = (float) -sindp_i (2 * PI * divdp_i((double)i,  (double)n));
            w[k + 2] = (float)  cosdp_i (4 * PI * divdp_i((double)i,  (double)n));
            w[k + 3] = (float) -sindp_i (4 * PI * divdp_i((double)i,  (double)n));
            w[k + 4] = (float)  cosdp_i (6 * PI * divdp_i((double)i,  (double)n));
            w[k + 5] = (float) -sindp_i (6 * PI * divdp_i((double)i,  (double)n));
#endif
            k += 6;
        }
    }
}
#else  //C674x

void tw_gen_float (float *w, int n)
{
    int i, j, k;
    double x_t, y_t, theta1, theta2, theta3;
    const double PI = 3.141592654;
	//double invn;
	
    for (j = 1, k = 0; j <= n >> 2; j = j << 2)
    {
        for (i = 0; i < n >> 2; i += j)
        {
            theta1 = 2 * PI * divdp_i((double)i,  (double)n);
            x_t = cosdp_i (theta1);
            y_t = sindp_i (theta1);
            w[k] = (float) x_t;
            w[k + 1] = (float) y_t;

            theta2 = 4 * PI * divdp_i((double)i,  (double)n);
            x_t = cosdp_i (theta2);
            y_t = sindp_i (theta2);
            w[k + 2] = (float) x_t;
            w[k + 3] = (float) y_t;

            theta3 = 6 * PI * divdp_i((double)i,  (double)n);
            x_t = cosdp_i (theta3);
            y_t = sindp_i (theta3);
            w[k + 4] = (float) x_t;
            w[k + 5] = (float) y_t;
            k += 6;
        }
    }
}
#endif

#endif //_GENTWIDDLE_FFTFLOAT
