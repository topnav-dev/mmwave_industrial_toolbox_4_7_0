/*
 *  @file transport.h
 *  @brief This file contains routines that are used in the transport layer of SBL.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <ti/drivers/qspiflash/qspiflash.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup SBL_INTERNAL_DATA_STRUCTURE
 @{ */

/**
 * @brief   UART baud rate at which the application metaimage file will be transferred.
 */
#define SBL_UART_BAUDRATE		            115200U

/**
 * @brief   This is the maximum number of retransmissions the xmodem will attempt before canceling the download.
 */
#define SBL_XMODEM_MAX_RETRANSMISSIONS      10U

/**
 * @brief   This is the maximum number of times the xmodem will attempt to establish a transfer.
 */
#define SBL_XMODEM_MAX_WAIT                 20U

/**
@}
*/

/* Global data buffers used for SPI test */
#define SPI_DATA_BLOCK_SIZE     128
volatile uint8_t    txBuf[SPI_DATA_BLOCK_SIZE];
volatile uint8_t    rxBuf[SPI_DATA_BLOCK_SIZE];
/* Test message definitio for SPI test as a slave with external devices */
#define MAGIC_NUMBER            0x3456
#define SPI_TEST_MSGLEN         128
#define SPI_TEST_SYNC_MSGLEN    0
#define SWAP_BYTES(x)           (( (x&0xff) << 8) | ((x >>8) & 0xff))



/* SBL Commands */
#define  SBL_COMMAND_SYNC_MSG                   0xAB01
#define  SBL_COMMAND_SYNC_MSG_WAIT              0xAB02
#define  SBL_COMMAND_SYNC_MSG_RCVD              0xAB03
#define  SBL_COMMAND_SM_MSG1                    0xAB04
#define  SBL_COMMAND_SM_MSG1_RESP               0xAB05
#define  SBL_COMMAND_UPDATE_FLASH               0xAB06
#define  SBL_COMMAND_AUTO_BOOT                  0xAB07 
#define  SBL_COMMAND_UPDATING_FLASH             0xAB08
#define  SBL_COMMAND_DATA                       0xAB0A
#define  SBL_COMMAND_ACK                        0xAB0B
#define  SBL_COMMAND_CHECKSUM_ERR               0xAB0C
#define  SBL_COMMAND_PRINTF                     0xAB0D
#define  SBL_COMMAND_TERMINATE                  0xAB10  





/* SPI Test message definition */
typedef struct spiTestMsg
{
    /* Test message MAGIC number */
    uint16_t msgId;

    /* Test message sequence number */
    uint16_t seqNo;

    /* Test message valid data length */
    uint16_t dataLen;

    /* Test message checksum */
    uint16_t checksum;

    /* Test message data payload */
    uint8_t data[SPI_TEST_MSGLEN - 8];
}spiTestMsg;

extern void SBL_transportInit(void);
extern void SBL_transportDeinit(void);
extern int32_t SBL_transportConfig();
extern int32_t SBL_transportRead(uint8_t* buffer, uint32_t size);
extern int32_t SBL_transportWrite(uint8_t* buffer, uint32_t size);
extern void SBL_printf(const uint8_t *pcFormat, ...);
extern int32_t SBL_transportDownloadFile(QSPIFlash_Handle QSPIFlashHandle, uint32_t flashAddr, uint32_t maxSize);
#ifdef __cplusplus
}
#endif
#endif /* TRANSPORT_H */
