/*!
 *   \file   RADARDEMO_aoaEstDML.c
 *
 *  \brief   Estimate the angle of arrival using DML.
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

#include <modules/DoA/DML/api/RADARDEMO_aoaEstDML.h>
#include "RADARDEMO_aoaEstDM_priv.h"
#include <math.h>
#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif
#define PIOVER180 (3.141592653589793/180.0)
#define ONEEIGHTYOVERPI (180.0/3.141592653589793)
#define PISQROVER180 (3.141592653589793 * 3.141592653589793f / 180.0)
#define PI        (3.141592653589793f)

/*
extern int32_t  RADARDEMO_aoaEstimationDML_generic(
                            IN  cplxf_t * Rn,
                            IN  cplxf_t * steeringVec,
                            IN  int32_t steeringVecSize,
                            IN  float   * scratchPad,
                            IN  int32_t nRxAnt,
                            OUT int32_t   * angleEst);
*/

/*!
   \fn     RADARDEMO_aoaEstimationDML_create

   \brief   Create and initialize RADARDEMO_aoaEstimationDML module.

   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_aoaEstimationDML module.

   \param[out]    errorCode
               Output error code.

   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.

   \pre       none

   \post      none


 */

void    * RADARDEMO_aoaEstimationDML_create(
                            IN  RADARDEMO_aoAEstDML_config * moduleConfig,
                            OUT RADARDEMO_aoAEstDML_errorCode * errorCode)

{
    uint32_t    i, j;
    double       ftemp1, freal1, fimag1, frealJ, fimagJ;
    RADARDEMO_aoAEstDML_handle * handle;
    //FILE *testSteerVec = fopen("steeringVec.bin","wb");

    *errorCode  =   RADARDEMO_AOADML_NO_ERROR;

    /* Check antenna spacing, if it's not a uniform linear array, return with NULL */
    if (moduleConfig->antSpacing[0] != 0)
        *errorCode = RADARDEMO_AOADML_ANTSPACE_NOTSUPPORTED;
    for (i = 1; i < moduleConfig->nRxAnt; i++ )
    {
        if (moduleConfig->antSpacing[i] - moduleConfig->antSpacing[i-1] != 1)
            *errorCode = RADARDEMO_AOADML_ANTSPACE_NOTSUPPORTED;
    }
    /* Check number of antenna , only supporting 4 and 8 at this point */
    if ( (moduleConfig->nRxAnt != 4) && ( moduleConfig->nRxAnt != 8) )
        *errorCode = RADARDEMO_AOADML_NUMANT_NOTSUPPORTED;


    if (*errorCode > RADARDEMO_AOADML_NO_ERROR)
        return (NULL);

    handle              =   (RADARDEMO_aoAEstDML_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_aoAEstDML_handle), 1);
    if (handle == NULL)
    {
        *errorCode = RADARDEMO_AOADML_FAIL_ALLOCATE_HANDLE;
        return (handle);
    }

    handle->nRxAnt      =   moduleConfig->nRxAnt;
    handle->estRange    =   moduleConfig->estRange;
    handle->estResolution   =   moduleConfig->estResolution;
    handle->minimumOutputConf   =   moduleConfig->minimumOutputConf;
    handle->angleEstBound   =   moduleConfig->angleEstBound;

    handle->steeringVecSize =   (uint32_t) ((2.f * handle->estRange) / (handle->estResolution)) + 1;
    if (moduleConfig->enableMultiStageSearch == 0)
    {
        handle->firstStageSearchStep = 1;
    }
    else
    {
        if (handle->steeringVecSize < 50)
            handle->firstStageSearchStep = 1;
        else if (handle->steeringVecSize < 90)
            handle->firstStageSearchStep = 2;
        else if (handle->steeringVecSize < 170)
            handle->firstStageSearchStep = 4;
        else if (handle->steeringVecSize < 250)
            handle->firstStageSearchStep = 6;
        else
            handle->firstStageSearchStep = 8;
    }

    handle->steeringVec =   (cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->nRxAnt * handle->steeringVecSize * sizeof(cplxf_t), 8);
    if (handle->steeringVec == NULL)
    {
        *errorCode  =   RADARDEMO_AOADML_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }
    handle->scratchPad  =   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, (5 * handle->steeringVecSize + 4 + 2 * handle->nRxAnt * handle->nRxAnt) * sizeof(uint32_t), 8);
    if (handle->scratchPad == NULL)
    {
        *errorCode  =   RADARDEMO_AOADML_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }

    /* Ant0's steeringVec is 1 for all angle possiblities, so we don't save them*/
    for (i = 0; i < handle->steeringVecSize; i++)
    {
        ftemp1          =   (double) sin((-handle->estRange + (double) i * handle->estResolution) * (double)PIOVER180);
        freal1          =   (double) cos(-PI*ftemp1);
        fimag1          =   (double) sin(-PI*ftemp1);
        frealJ          =   freal1;
        fimagJ          =   fimag1;
        handle->steeringVec[(handle->nRxAnt - 1) * i + 0].real = (float)frealJ;
        handle->steeringVec[(handle->nRxAnt - 1) * i + 0].imag = (float)fimagJ;
        for (j = 2; j < handle->nRxAnt; j++)
        {
            ftemp1      =   frealJ;
            frealJ      =   frealJ * freal1 - fimagJ * fimag1;
            fimagJ      =   ftemp1 * fimag1 + fimagJ * freal1;
            handle->steeringVec[(handle->nRxAnt - 1) * i + j - 1].real = (float)frealJ;
            handle->steeringVec[(handle->nRxAnt - 1) * i + j - 1].imag = (float)fimagJ;
        }
    }

    //printf("handle->steeringVecSize = %d\n", handle->steeringVecSize);
    //fwrite(handle->steeringVec, 4, handle->steeringVecSize * 6, testSteerVec);
    //fclose(testSteerVec);

    return((void *)handle);

}

/*!
   \fn     RADARDEMO_aoaEstimationDML_delete

   \brief   Delete RADARDEMO_aoaEstimationDML module.

   \param[in]    handle
               Module handle.

   \pre       none

   \post      none


 */

void    RADARDEMO_aoaEstimationDML_delete(
                            IN  void * handle)
{
    RADARDEMO_aoAEstDML_handle *aoaEstDMLInst;

    aoaEstDMLInst   =   (RADARDEMO_aoAEstDML_handle *) handle;

    radarOsal_memFree(aoaEstDMLInst->steeringVec, aoaEstDMLInst->steeringVecSize * aoaEstDMLInst->nRxAnt * sizeof(cplxf_t));
    radarOsal_memFree(aoaEstDMLInst->scratchPad, aoaEstDMLInst->steeringVecSize * sizeof(uint32_t));

    radarOsal_memFree(handle, sizeof(RADARDEMO_aoAEstDML_handle));
}



/*!
   \fn     RADARDEMO_aoaEstimationDML_run

   \brief   Estimate the angle of arrival of each detected object using DML.

   \param[in]    handle
               Module handle.

   \param[in]    inputSignal
               Input antenna signal for detected object.

   \param[out]    estOutput
               Pointer to the estimation output.
   \ret  error code
   \pre       none

   \post      none


 */

RADARDEMO_aoAEstDML_errorCode   RADARDEMO_aoaEstimationDML_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoAEst_input * input,
                            OUT RADARDEMO_aoAEst_output   * estOutput)

{
    int32_t     i, j, k;
    RADARDEMO_aoAEstDML_handle *aoaEstDMLInst;
    cplxf_t     * Rn, * inputSignal;
    float       * localScratch;
    int32_t       angleEst[2];
    float         normVar[2];
    RADARDEMO_aoAEstDML_errorCode errorCode = RADARDEMO_AOADML_NO_ERROR;
    float       noiseP, dwx[2], dwt, ftemp1, ftemp2, tempAngle[2], conf[2], temprcp, signalP;
    double      dtemp1;

    aoaEstDMLInst   =   (RADARDEMO_aoAEstDML_handle *) handle;

    inputSignal     =   input->inputAntSamples;
    noiseP          =   input->inputNoisePow/4.f;
    if ( inputSignal == NULL)
        errorCode   =   RADARDEMO_AOADML_INOUTPTR_NOTCORRECT;
    if (  estOutput == NULL)
        errorCode   =   RADARDEMO_AOADML_INOUTPTR_NOTCORRECT;
    if (  aoaEstDMLInst->scratchPad == NULL)
        errorCode   =   RADARDEMO_AOADML_INOUTPTR_NOTCORRECT;
    if ( aoaEstDMLInst->steeringVec == NULL)
        errorCode   =   RADARDEMO_AOADML_INOUTPTR_NOTCORRECT;
    if (errorCode > RADARDEMO_AOADML_NO_ERROR)
        return (errorCode);

    Rn              =   (cplxf_t *) &aoaEstDMLInst->scratchPad[0];
    localScratch    =   (float *) &aoaEstDMLInst->scratchPad[aoaEstDMLInst->nRxAnt * aoaEstDMLInst->nRxAnt * 2];

    k               =   0;
    for (i = 0; i < (int32_t) aoaEstDMLInst->nRxAnt; i++ )
    {
        for (j = i; j < (int32_t) aoaEstDMLInst->nRxAnt; j++ )
        {
            _amem8_f2(&Rn[k]) = _complex_conjugate_mpysp(_amem8_f2(&inputSignal[i]), _amem8_f2(&inputSignal[j]));
            k++;
        }
    }
    signalP     =   Rn[0].real + Rn[4].real + Rn[7].real + Rn[9].real;

/*
    estOutput->numOutput = RADARDEMO_aoaEstimationDML_generic(
                            Rn,
                            aoaEstDMLInst->steeringVec,
                            aoaEstDMLInst->steeringVecSize,
                            localScratch,
                            aoaEstDMLInst->nRxAnt,
                            angleEst);

        estOutput->outputAngles[0] = -(aoaEstDMLInst->estRange) + (float) angleEst[0] * aoaEstDMLInst->estResolution;
        if (estOutput->numOutput > 1)
            estOutput->outputAngles[1] = -(aoaEstDMLInst->estRange) + (float) angleEst[1] * aoaEstDMLInst->estResolution;

*/

    if (aoaEstDMLInst->nRxAnt == 4)
    {
        estOutput->numOutput = RADARDEMO_aoaEstimationDML_4ant(
                            Rn,
                            aoaEstDMLInst->steeringVec,
                            aoaEstDMLInst->steeringVecSize,
                            localScratch,
                            aoaEstDMLInst->firstStageSearchStep,
                            normVar,
                            angleEst);
    }
    else if (aoaEstDMLInst->nRxAnt == 8)
    {
        estOutput->numOutput = RADARDEMO_aoaEstimationDML_8ant(
                            Rn,
                            aoaEstDMLInst->steeringVec,
                            aoaEstDMLInst->steeringVecSize,
                            localScratch,
                            aoaEstDMLInst->firstStageSearchStep,
                            normVar,
                            angleEst);
    }
    j = 0;
    tempAngle[j]        =   -(aoaEstDMLInst->estRange) + (float) angleEst[0] * aoaEstDMLInst->estResolution;
    if((tempAngle[j] <= aoaEstDMLInst->angleEstBound) && (tempAngle[j] >= -aoaEstDMLInst->angleEstBound))
    {
		k				=	0;
        dtemp1          =   cos(tempAngle[j] * PIOVER180);
        dwx[j]           =   (float)dtemp1 * 2.236068f * (float)PISQROVER180;  /* 2.236068 = sqrt(5)*/
        dwx[j]           =   dwx[j] * dwx[j];
        dwt             =   dwx[j] * signalP;
        ftemp1          =   normVar[k];
        temprcp         =   _rsqrsp(ftemp1);
        temprcp         =   temprcp * (1.5f - (0.5f * ftemp1) * temprcp * temprcp);
        temprcp         =   temprcp * (1.5f - (0.5f * ftemp1) * temprcp * temprcp);
        ftemp1          =   normVar[k] * temprcp;               /*ftemp1 = sqrt(normVar[0])*/
        ftemp2          =   ftemp1 * noiseP;
        temprcp         =   _rcpsp(ftemp2);
        temprcp         =   temprcp * (2.f - ftemp2 * temprcp);
        temprcp         =   temprcp * (2.f - ftemp2 * temprcp);
        ftemp1          =   dwt * temprcp;
        conf[j]         =   _rsqrsp(ftemp1) * (float)ONEEIGHTYOVERPI * 0.5f;
        j++;
    }

    tempAngle[j]        =   -(aoaEstDMLInst->estRange) + (float) angleEst[1] * aoaEstDMLInst->estResolution;
    if((tempAngle[j] <= aoaEstDMLInst->angleEstBound) && (tempAngle[j] >= -aoaEstDMLInst->angleEstBound))
    {
		k				=	1;
        dtemp1          =   cos(tempAngle[j] * PIOVER180);
        dwx[j]          =   (float)dtemp1 * 2.236068f * (float)PISQROVER180;  /* 2.236068 = sqrt(5)*/
        dwx[j]          =   dwx[j] * dwx[j];
        dwt             =   dwx[j] * signalP;
        ftemp1          =   normVar[k];
        temprcp         =   _rsqrsp(ftemp1);
        temprcp         =   temprcp * (1.5f - (0.5f * ftemp1) * temprcp * temprcp);
        temprcp         =   temprcp * (1.5f - (0.5f * ftemp1) * temprcp * temprcp);
        ftemp1          =   normVar[k] * temprcp;               /*ftemp1 = sqrt(normVar[0])*/
        ftemp2          =   ftemp1 * noiseP;
        temprcp         =   _rcpsp(ftemp2);
        temprcp         =   temprcp * (2.f - ftemp2 * temprcp);
        temprcp         =   temprcp * (2.f - ftemp2 * temprcp);
        ftemp1          =   dwt * temprcp;
        conf[j]         =   _rsqrsp(ftemp1) * (float)ONEEIGHTYOVERPI * 0.5f;
        j++;
    }

    i               =   0;
    if (j == 2)
    {
        ftemp1          =   normVar[0] * _rcpsp(dwx[0]);
        normVar[0]      =   ftemp1;
        ftemp2          =   normVar[1] * _rcpsp(dwx[1]);
        normVar[1]      =   ftemp2;

        if (ftemp1 < RADARDEMO_AOAESTDML_NORMVAETHR)
        {
            if (ftemp2 < RADARDEMO_AOAESTDML_NORMVAETHR)
            {
                if( conf[0] < aoaEstDMLInst->minimumOutputConf )
                {
                    estOutput->outputAngles[i]  =   tempAngle[0];
                    estOutput->outputVar[i] =   conf[0];
                    i++;
                }

                if( conf[1] < aoaEstDMLInst->minimumOutputConf )
                {
                    estOutput->outputAngles[i]  =   tempAngle[1];
                    estOutput->outputVar[i] =   conf[1];
                    i++;
                }
            }
            else
            {
                if( conf[0] < aoaEstDMLInst->minimumOutputConf )
                {
                    estOutput->outputAngles[i]  =   tempAngle[0];
                    estOutput->outputVar[i] =   conf[0];
                    i++;
                }
            }
        }
        else if (ftemp2 < RADARDEMO_AOAESTDML_NORMVAETHR)
        {
            if( conf[1] < aoaEstDMLInst->minimumOutputConf )
            {
                estOutput->outputAngles[i]  =   tempAngle[1];
                estOutput->outputVar[i] =   conf[1];
                i++;
            }
        }
        else
        {
            if( conf[0] < aoaEstDMLInst->minimumOutputConf )
            {
                estOutput->outputAngles[i]  =   tempAngle[0];
                estOutput->outputVar[i] =   conf[0];
                i++;
            }


            if( conf[1] < aoaEstDMLInst->minimumOutputConf )
            {
                estOutput->outputAngles[i]  =   tempAngle[1];
                estOutput->outputVar[i] =   conf[1];
                i++;
            }
        }
    }
    else if (j == 1)
    {
        ftemp1          =   normVar[k] * _rcpsp(dwx[0]);
        normVar[0]      =   ftemp1;
        if( conf[0] < aoaEstDMLInst->minimumOutputConf )
        {
            estOutput->outputAngles[i]  =   tempAngle[0];
            estOutput->outputVar[i] =   conf[0] ;
            i++;
        }

    }
    estOutput->numOutput    =   i;

    return (errorCode);
}

