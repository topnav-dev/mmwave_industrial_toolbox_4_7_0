/**
 *   @file  antenna_geometry.c
 *
 *   @brief
 *
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>

/* mmWave SDK drivers/common Include Files */
#include <ti/board/antenna_geometry.h>

#if defined (SOC_XWR16XX)
/**
 * @brief Antenna geometry for XWR1642
 *
 */
ANTDEF_AntGeometry gAntDef_default = {
    .txAnt = {
             {0, 0},
             {4, 0}
     },
    .rxAnt = {
             {0, 0},
             {1, 0},
             {2, 0},
             {3, 0}
      }
};
#else
/**
 * @brief Antenna geometry for IWR6843 AOP
 *
 */
ANTDEF_AntGeometry gAntDef_IWR6843AOP = {
     .txAnt = {
                {0, 0},
                {2, 2},
                {0, 2}
       },
     .rxAnt = {
                {1, 1},
                {1, 0},
                {0, 1},
                {0, 0}
              }
};
/**
 * @brief Antenna geometry for IWR6843ISK-ODS
 *
 */
ANTDEF_AntGeometry gAntDef_IWR6843ODS = {
     .txAnt = {
                {0, 0},
                {2, 0},
                {2, 2}
       },
     .rxAnt = {
                {0, 0},
                {0, 1},
                {1, 1},
                {1, 0}
              }
};
/**
 * @brief Antenna geometry for standard EVM boards: XWR1843, IWR6843
 *
 */
ANTDEF_AntGeometry gAntDef_default = {
     .txAnt = {
                {0, 1},
                {2, 0},
                {4, 1}
       },
     .rxAnt = {
                {0, 0},
                {1, 0},
                {2, 0},
                {3, 0}
              }
};
#endif


