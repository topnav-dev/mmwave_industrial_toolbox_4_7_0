/*!
 *   \file   RADARDEMO_svmKernelUtil_priv.c
 *
 *   \brief   Utility functions for SVM kernal function.
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

#include "RADARDEMO_svmKernelUtil_priv.h"
#include <math.h>

#ifdef _TMS320C6X
#include "c6x.h"
#endif

#ifdef OPTIMIZED4DSP

/* ti_math_kTable */
double ti_math_kTable[4] = {
  1.000000000,              /* 2^(0/4) */
  1.189207115,              /* 2^(1/4) */
  1.414213562,              /* 2^(2/4) */
  1.681792831               /* 2^(3/4) */
};

/* ti_math_jTable */
double ti_math_jTable[4] = {
  1.000000000,              /* 2^(0/16) */
  1.044273782,              /* 2^(1/16) */
  1.090507733,              /* 2^(2/16) */
  1.138788635               /* 2^(3/16) */
};



//! \copydoc RADARDEMO_svmPredictLinearKernal
int8_t RADARDEMO_svmPredictLinearKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
                            IN  float *  RESTRICT beta,
							IN  float    scale,
							IN  float    bias,
							OUT float * RESTRICT score)
 {
	int32_t j;
	float sum, oneOverScale;
	int8_t itemp;
	
	oneOverScale	=	1.f/scale;
	sum			=	0.f;
	for ( j = 0; j < numFeatPerInput; j++ )
	{
		sum 		+=	(inputVector[j] * beta[j]);
	}
	sum			*=	oneOverScale;
	sum			+=	bias;
	*score	=	sum;
	itemp       =   1;
	if ( sum < 0 )
	{
		itemp	=	-1;
	}
	return(itemp);
}

//! \copydoc RADARDEMO_svmPredictGaussianKernal
int8_t RADARDEMO_svmPredictGaussianKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
							IN  float    scale,
                            IN  int16_t  svLen,
                            IN  float *  RESTRICT sv,
                            IN  int8_t *  RESTRICT svTag,
                            IN  float *  RESTRICT alpha,
							IN  float    bias,
							OUT float * RESTRICT score)
 {
	int32_t i, j;
	float oneOverScale2, sum, fProd, facc1, ftemp;
	int8_t itemp;
	
	sum = 0.f;
	oneOverScale2	=	1.f/(scale*scale);
	if (numFeatPerInput == 2)
	{
		for ( i = 0; i < svLen; i++ )
		{
			fProd			=	alpha[i] * (float) svTag[i];
			facc1			=	0.f;
			ftemp			=	inputVector[0] - sv[i * numFeatPerInput + 0];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[1] - sv[i * numFeatPerInput + 1];
			facc1			+=	ftemp * ftemp;
			facc1		*=	oneOverScale2;
			facc1		=	expsp_i(-facc1);
			fProd		*=  facc1;
			sum			+=	fProd;
		}
	}
	else if (numFeatPerInput == 3)
	{
		for ( i = 0; i < svLen; i++ )
		{
			fProd			=	alpha[i] * (float) svTag[i];
			facc1			=	0.f;
			ftemp			=	inputVector[0] - sv[i * numFeatPerInput + 0];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[1] - sv[i * numFeatPerInput + 1];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[2] - sv[i * numFeatPerInput + 2];
			facc1			+=	ftemp * ftemp;
			facc1		*=	oneOverScale2;
			facc1		=	expsp_i(-facc1);
			fProd		*=  facc1;
			sum			+=	fProd;
		}
	}
	else if (numFeatPerInput == 4)
	{
		for ( i = 0; i < svLen; i++ )
		{
			fProd			=	alpha[i] * (float) svTag[i];
			facc1			=	0.f;
			ftemp			=	inputVector[0] - sv[i * numFeatPerInput + 0];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[1] - sv[i * numFeatPerInput + 1];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[2] - sv[i * numFeatPerInput + 2];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[3] - sv[i * numFeatPerInput + 3];
			facc1			+=	ftemp * ftemp;
			facc1		*=	oneOverScale2;
			facc1		=	expsp_i(-facc1);
			fProd		*=  facc1;
			sum			+=	fProd;
		}
	}
	else if (numFeatPerInput == 5)
	{
		for ( i = 0; i < svLen; i++ )
		{
			fProd			=	alpha[i] * (float) svTag[i];
			facc1			=	0.f;
			ftemp			=	inputVector[0] - sv[i * numFeatPerInput + 0];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[1] - sv[i * numFeatPerInput + 1];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[2] - sv[i * numFeatPerInput + 2];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[3] - sv[i * numFeatPerInput + 3];
			facc1			+=	ftemp * ftemp;
			ftemp			=	inputVector[4] - sv[i * numFeatPerInput + 4];
			facc1			+=	ftemp * ftemp;
			facc1		*=	oneOverScale2;
			facc1		=	expsp_i(-facc1);
			fProd		*=  facc1;
			sum			+=	fProd;
		}
	}
	else
	{
		for ( i = 0; i < svLen; i++ )
		{
			fProd			=	alpha[i] * (float) svTag[i];
			facc1			=	0.f;
			for ( j = 0; j < numFeatPerInput; j++ )
			{
				ftemp	=	inputVector[j] - sv[i * numFeatPerInput + j];
				facc1	+=	ftemp * ftemp;
			}
			facc1		*=	oneOverScale2;
			facc1		=	expsp_i(-facc1);
			fProd		*=  facc1;
			sum			+=	fProd;
		}

	}
	sum			+=	bias;
	*score	=	sum;
	itemp       =   1;
	if ( sum < 0 )
	{
		itemp	=	-1;
	}
	return(itemp);
}

//! \copydoc RADARDEMO_svmPredictPolyKernal
int8_t RADARDEMO_svmPredictPolyKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
							IN  float    scale,
							IN  int8_t  order,
                            IN  int16_t  svLen,
                            IN  float *  RESTRICT sv,
                            IN  int8_t *  RESTRICT svTag,
                            IN  float *  RESTRICT alpha,
							IN  float    bias,
							OUT float * RESTRICT score)
 {
	int32_t i, j;
	float oneOverScale2, sum, fProd, facc1, ftemp;
	int8_t itemp;
	
	sum = 0.f;
	oneOverScale2	=	1.f/(scale*scale);
	if(order == 3)
	{
		if(numFeatPerInput == 2)
		{
			for ( i = 0; i < svLen; i++ )
			{
				fProd		=	alpha[i] * (float) svTag[i];
				facc1		=	0.f;
				ftemp		=	inputVector[0] * sv[i * numFeatPerInput + 0];
				facc1		+=	ftemp;
				ftemp		=	inputVector[1] * sv[i * numFeatPerInput + 1];
				facc1		+=	ftemp;
				facc1		*=	oneOverScale2;
				facc1		+=	1.f;
				facc1		=	facc1 * facc1 * facc1;
				fProd		*=  facc1;
				sum			+=	fProd;
			}
		}
		else if(numFeatPerInput == 3)
		{
			for ( i = 0; i < svLen; i++ )
			{
				fProd		=	alpha[i] * (float) svTag[i];
				facc1		=	0.f;
				ftemp		=	inputVector[0] * sv[i * numFeatPerInput + 0];
				facc1		+=	ftemp;
				ftemp		=	inputVector[1] * sv[i * numFeatPerInput + 1];
				facc1		+=	ftemp;
				ftemp		=	inputVector[2] * sv[i * numFeatPerInput + 2];
				facc1		+=	ftemp;
				facc1		*=	oneOverScale2;
				facc1		+=	1.f;
				facc1		=	facc1 * facc1 * facc1;
				fProd		*=  facc1;
				sum			+=	fProd;
			}
		}
		else if(numFeatPerInput == 4)
		{
			for ( i = 0; i < svLen; i++ )
			{
				fProd		=	alpha[i] * (float) svTag[i];
				facc1		=	0.f;
				ftemp		=	inputVector[0] * sv[i * numFeatPerInput + 0];
				facc1		+=	ftemp;
				ftemp		=	inputVector[1] * sv[i * numFeatPerInput + 1];
				facc1		+=	ftemp;
				ftemp		=	inputVector[2] * sv[i * numFeatPerInput + 2];
				facc1		+=	ftemp;
				ftemp		=	inputVector[3] * sv[i * numFeatPerInput + 3];
				facc1		+=	ftemp;
				facc1		*=	oneOverScale2;
				facc1		+=	1.f;
				facc1		=	facc1 * facc1 * facc1;
				fProd		*=  facc1;
				sum			+=	fProd;
			}
		}
		else if(numFeatPerInput == 5)
		{
			for ( i = 0; i < svLen; i++ )
			{
				fProd		=	alpha[i] * (float) svTag[i];
				facc1		=	0.f;
				ftemp		=	inputVector[0] * sv[i * numFeatPerInput + 0];
				facc1		+=	ftemp;
				ftemp		=	inputVector[1] * sv[i * numFeatPerInput + 1];
				facc1		+=	ftemp;
				ftemp		=	inputVector[2] * sv[i * numFeatPerInput + 2];
				facc1		+=	ftemp;
				ftemp		=	inputVector[3] * sv[i * numFeatPerInput + 3];
				facc1		+=	ftemp;
				ftemp		=	inputVector[4] * sv[i * numFeatPerInput + 4];
				facc1		+=	ftemp;
				facc1		*=	oneOverScale2;
				facc1		+=	1.f;
				facc1		=	facc1 * facc1 * facc1;
				fProd		*=  facc1;
				sum			+=	fProd;
			}
		}
		else
		{
			for ( i = 0; i < svLen; i++ )
			{
				fProd			=	alpha[i] * (float) svTag[i];
				facc1			=	0.f;
				for ( j = 0; j < numFeatPerInput; j++ )
				{
					ftemp	=	inputVector[j] * sv[i * numFeatPerInput + j];
					facc1	+=	ftemp;
				}
				facc1		*=	oneOverScale2;
				facc1		+=	1.f;
				facc1		=	facc1 * facc1 * facc1;
				fProd		*=  facc1;
				sum			+=	fProd;
			}
		}
	}
	else
	{
		for ( i = 0; i < svLen; i++ )
		{
			fProd			=	alpha[i] * (float) svTag[i];
			facc1			=	0.f;
			for ( j = 0; j < numFeatPerInput; j++ )
			{
				ftemp	=	inputVector[j] * sv[i * numFeatPerInput + j];
				facc1	+=	ftemp;
			}
			facc1		*=	oneOverScale2;
			facc1		+=	1.f;
			facc1		=	pow(facc1, order);
			fProd		*=  facc1;
			sum			+=	fProd;
		}
	}
	sum			+=	bias;
	*score	=	sum;
	itemp       =   1;
	if ( sum < 0 )
	{
		itemp	=	-1;
	}
	return(itemp);
}

#else
//! \copydoc RADARDEMO_svmPredictLinearKernal
int8_t RADARDEMO_svmPredictLinearKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
                            IN  float *  RESTRICT beta,
							IN  float    scale,
							IN  float    bias,
							OUT float * RESTRICT score)
 {
	int32_t j;
	float sum, oneOverScale;
	int8_t itemp;
	
	oneOverScale	=	1.f/scale;
	sum			=	0.f;
	for ( j = 0; j < numFeatPerInput; j++ )
	{
		sum 		+=	(inputVector[j] * beta[j]);
	}
	sum			*=	oneOverScale;
	sum			+=	bias;
	*score	=	sum;
	itemp       =   1;
	if ( sum < 0 )
	{
		itemp	=	-1;
	}
	return(itemp);
}

//! \copydoc RADARDEMO_svmPredictGaussianKernal
int8_t RADARDEMO_svmPredictGaussianKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
							IN  float    scale,
                            IN  int16_t  svLen,
                            IN  float *  RESTRICT sv,
                            IN  int8_t *  RESTRICT svTag,
                            IN  float *  RESTRICT alpha,
							IN  float    bias,
							OUT float * RESTRICT score)
 {
	int32_t i, j;
	float oneOverScale2, sum, fProd, facc1, ftemp;
	int8_t itemp;
	
	sum = 0.f;
	oneOverScale2	=	1.f/(scale*scale);
	for ( i = 0; i < svLen; i++ )
	{
		fProd			=	alpha[i] * (float) svTag[i];
		facc1			=	0.f;
		for ( j = 0; j < numFeatPerInput; j++ )
		{
			ftemp	=	inputVector[j] - sv[i * numFeatPerInput + j];
			facc1	+=	ftemp * ftemp;
		}
		facc1		*=	oneOverScale2;
		facc1		=	exp(-facc1);
		fProd		*=  facc1;
		sum			+=	fProd;
	}
	
	sum			+=	bias;
	*score	=	sum;
	itemp       =   1;
	if ( sum < 0 )
	{
		itemp	=	-1;
	}
	return(itemp);
}

//! \copydoc RADARDEMO_svmPredictPolyKernal
int8_t RADARDEMO_svmPredictPolyKernal(
							IN  int8_t  numFeatPerInput,
                            IN  float *  RESTRICT inputVector,
							IN  float    scale,
							IN  int8_t  order,
                            IN  int16_t  svLen,
                            IN  float *  RESTRICT sv,
                            IN  int8_t *  RESTRICT svTag,
                            IN  float *  RESTRICT alpha,
							IN  float    bias,
							OUT float * RESTRICT score)
 {
	int32_t i, j;
	float oneOverScale2, sum, fProd, facc1, ftemp;
	int8_t itemp;
	
	sum = 0.f;
	oneOverScale2	=	1.f/(scale*scale);
	for ( i = 0; i < svLen; i++ )
	{
		fProd			=	alpha[i] * (float) svTag[i];
		facc1			=	0.f;
		for ( j = 0; j < numFeatPerInput; j++ )
		{
			ftemp	=	inputVector[j] * sv[i * numFeatPerInput + j];
			facc1	+=	ftemp;
		}
		facc1		*=	oneOverScale2;
		facc1		+=	1.f;
		facc1		=	pow(facc1, order);
		fProd		*=  facc1;
		sum			+=	fProd;
	}
	
	sum			+=	bias;
	*score	=	sum;
	itemp       =   1;
	if ( sum < 0 )
	{
		itemp	=	-1;
	}
	return(itemp);
}
#endif
