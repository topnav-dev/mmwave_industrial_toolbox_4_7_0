/**
 *   @file  MATRIX_cholesky_dat.h
 *
 *   @brief
 *      Header file for tables used for Cholesky Decomposition
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

#include <people_counting/68xx_3D_people_counting/src/common/swpform.h>



#ifndef __MATRIX_CHOLESKY_DAT_H
#define __MATRIX_CHOLESKY_DAT_H


#ifdef __cplusplus
extern "C" {
#endif

/* Marker for auto-documentation - do not modify the following line */
/* BEGIN: FUNC                                                      */

#ifndef __DEM_CHOLESKY_DAT_C
extern
#endif
const uint16_t recipSqtTable_hcn[256];

#ifndef __DEM_CHOLESKY_DAT_C
extern
#endif
const int16_t recipTable[64];

#ifndef __DEM_CHOLESKY_DAT_C
extern
#endif
const uint8_t recipSqtTable_asm[256];


/* Marker for auto-documentation - do not modify the following line */
/* END: FUNC                                                        */

#ifdef __cplusplus
}
#endif

#endif  //__MATRIX_CHOLESKY_DAT_H


