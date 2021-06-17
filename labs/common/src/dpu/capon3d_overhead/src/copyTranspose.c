/*!
 *  \file   copyTranspose.c
 *
 *  \brief   Copy with transpose function.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include <common/src/dpu/capon3d_overhead/include/copyTranspose.h>

#if 0
uint32_t copyTranspose(uint32_t * src, uint32_t * dest, uint32_t size, int32_t offset, uint32_t stride, uint32_t pairs)
{
	int32_t i, j, k;
	j = 0;
	for(i = 0; i < (int32_t)size; i++)
	{
		for (k = 0; k < (int32_t)pairs; k++)
		{
			dest[j+k+i*offset] = src[pairs * i + k];
		}
		j += (int32_t)stride;
	}
	return(1);
}
#else

// for optimization purposes specific for 3D capon people counting, will ignore offset and pair parameter.
uint32_t copyTranspose(uint32_t * RESTRICT src, uint32_t * RESTRICT dest, uint32_t size, int32_t offset, uint32_t stride, uint32_t pairs)
{
    int32_t     i;
    int32_t     sizeOver4;
    uint64_t    * RESTRICT input, lltemp1;
    uint32_t    * RESTRICT output;
    uint32_t    * RESTRICT input1;

    sizeOver4       =   (int32_t) (size >> 2);
    input           =   (uint64_t *) src;
    output          =   dest;

    for(i = 0; i < sizeOver4; i++)
    {
        lltemp1     =   _amem8(input++);
        * output    =   _loll(lltemp1);
        output      +=  stride;
        * output    =   _hill(lltemp1);
        output      +=  stride;
        lltemp1     =   _amem8(input++);
        * output    =   _loll(lltemp1);
        output      +=  stride;
        * output    =   _hill(lltemp1);
        output      +=  stride;
    }

    input1          =   (uint32_t *) src;
    i               =   i * 4;
    for(; i < (int32_t)size; i++)
    {
        * output    =   input1[i];
        output      +=  stride;
    }

    return(1);
}

#endif

