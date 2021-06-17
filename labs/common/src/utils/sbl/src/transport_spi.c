/**
 *   @file  transport.c
 *
 *   @brief
 *      This file contain the SPI, XMODEM transport specific funtions.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdio.h>
#include <string.h>

/* MMWSDK include file. */
#include <ti/drivers/qspiflash/qspiflash.h>
#include <ti/common/sys_common.h>

/* SPI Header file */
#include <ti/drivers/spi/SPI.h>

/* SBL internal include file. */
#include <common/src/utils/sbl/include/sbl_internal.h>

/* CRC16 include file. */
#include <common/src/utils/sbl/include/crc16.h>

/**************************************************************************
 ************************** Local Definitions *****************************
 **************************************************************************/
#define SBL_SPI_LOG_BUFF_SIZE           120U
#define SBL_XMODEM_HEADER_SIZE          3U
#define SBL_XMODEM_BUFFER_SIZE          1024U
#define SBL_XMODEM_CRC_SIZE             2U
#define SBL_XMODEM_DATABUFFER_SIZE      (SBL_XMODEM_BUFFER_SIZE + SBL_XMODEM_HEADER_SIZE + SBL_XMODEM_CRC_SIZE + 1U)
#define SBL_SOH                         0x01U
#define SBL_STX                         0x02U
#define SBL_EOT                         0x04U
#define SBL_ACK                         0x06U
#define SBL_NAK                         0x15U
#define SBL_CAN                         0x18U

/**************************************************************************
 *************************** Function Definitions *************************
 **************************************************************************/
//static int32_t SBL_verifyCRC(uint8_t* dataBuffer, uint32_t dataLength, uint32_t verifyCRC);
//static void SBL_discardInput(void);

/**
 *  @b Description
 *  @n
 *      SPI read only function.
 *
 *   @param[in] handle            SPI driver handle
 *   @param[in] dataLen           Number of bytes need to be transferred
 *   @param[in] buffer            Pointer of the buffer
 *   @param[in] slaveIndex        Slave index for the SPI transfer
 *
 *  @retval    Successful         =0
 *                 Failed         <0
 */
static int32_t Spoof_spiRead(const SPI_Handle handle, void* buffer, uint32_t dataLen)
{
    SPI_Transaction transaction;

    /* Configure Data Transfer */
    transaction.count = dataLen;
    transaction.txBuf = NULL;
    transaction.rxBuf = buffer;
    transaction.slaveIndex = 0;

    /* Start Data Transfer */
    if (SPI_transfer(handle, &transaction) != true)
    {
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      SPI write only function.
 *
 *   @param[in] handle            SPI driver handle
 *   @param[in] dataLen           Number of bytes need to be transferred
 *   @param[in] buffer            Pointer of the buffer
 *   @param[in] slaveIndex        Slave index for the SPI transfer
 *
 *  @retval    Successful         =0
 *                 Failed         <0
 */
static int32_t Spoof_spiWrite(const SPI_Handle handle, void* buffer, uint32_t dataLen)
{
    SPI_Transaction transaction;

    /* Configure Data Transfer */
    transaction.count = dataLen;
    transaction.txBuf = buffer;
    transaction.rxBuf = NULL;
    transaction.slaveIndex = 0;

    /* Start Data Transfer */
    if (SPI_transfer(handle, &transaction) != true)
    {
        return -1;
    }
    return 0;
}

#if 0

/*!
 *  @b Description
 *  @n
 *      This function is used to verify the CRC or checksum of a block of data.
 *
 *  @param[in]  dataBuffer
 *      Pointer to the data buffer.
 *  @param[in]  dataLength
 *      Size of the data buffer.
 *  @param[in]  verifyCRC
 *      Flag indicating whether CRC or checksum has to be verified.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   SBL Error code
 */
static int32_t SBL_verifyCRC(uint8_t* dataBuffer, uint32_t dataLength, uint32_t verifyCRC)
{
    int32_t     retVal = MINUS_ONE;

    if (dataLength != 0)
    {
    	if (verifyCRC)
        {
    		uint16_t        CRC16Bit;
            uint16_t        rxCRC;

            /* Compute the 16 bit CRC. */
            CRC16Bit = crc16_ccitt(dataBuffer, dataLength);

            /* Read the CRC from the packet. */
            rxCRC = (dataBuffer[dataLength] << 8U) + dataBuffer[dataLength + 1U];

            if (CRC16Bit == rxCRC)
            {
                retVal = 0;
            }
	    }
    	else
        {
		    uint8_t     checksum = 0;
    		uint32_t    index;

            /* Add all the bytes while dropping any carry overs to compute checksum. */
		    for (index = 0; index < dataLength; index++)
            {
	    		checksum += dataBuffer[index];
		    }

            /* Compare to the 8 bit checksum received in the packet */
	    	if (checksum == dataBuffer[dataLength])
            {
                retVal = 0;
            }
        }
    }
	return retVal;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to discard the incoming SPI stream incase an error is detected.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void SBL_discardInput(void)
{
	uint8_t     rxData;

    Spoof_spiRead(gSblMCB.spiHandle, (uint8_t*)&rxData, 1U);
    return;
}

#endif

/*!
 *  @b Description
 *  @n
 *      This function initializes the SPI interface that is
 *      used as transport to download the file.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void SBL_transportInit(void)
{
	/* Initialize the SPI */
    SPI_init();
}

/*!
 *  @b Description
 *  @n
 *      This function de-initializes the SPI interface that is
 *      used as transport to download the file.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void SBL_transportDeinit(void)
{
	/* Close the SPI peripheral */
    SPI_close(gSblMCB.spiHandle);
}

/*!
 *  @b Description
 *  @n
 *      Opens the SPI instance in the specified mode.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   SBL Error code
 */

int32_t SBL_transportConfig(void)
{

    SPI_Params     params;
    int32_t         retVal;

   /* Setup the default SPI Parameters */
    SPI_Params_init(&params);

    params.mode = SPI_SLAVE;
    params.frameFormat = SPI_POL0_PHA1;
    params.pinMode = SPI_PINMODE_4PIN_CS;
    params.shiftFormat = SPI_MSB_FIRST;

    /* When communicating with PC through FTDI, it is difficult to toggle CS for every two bytes, hence set csHold to 1.
       In this mode, the highest working SPI clock is 2MHz */
    params.csHold = 1;

    /* Enable DMA and set DMA channels */
    params.dmaEnable = 1;
    params.dmaHandle = gSblMCB.dmaHandle;
    params.u.slaveParams.dmaCfg.txDmaChanNum =1U;
    params.u.slaveParams.dmaCfg.rxDmaChanNum =0U;



    /* Open the SPI Instance */
    gSblMCB.spiHandle = SPI_open(0, &params);


    if (gSblMCB.spiHandle == NULL)
    {
	retVal = SBL_EINVAL;
    }
    else
    {
        retVal = SBL_EOK;
    }
	return retVal;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to read a input character from the SPI peripheral.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
int32_t SBL_transportRead(uint8_t* buffer, uint32_t size)
{
    int32_t     retVal;

	/* Read using the SPI peripheral */
    retVal = Spoof_spiRead(gSblMCB.spiHandle, (uint8_t*)buffer, size);

    return retVal;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to read a input character from the SPI peripheral.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
int32_t SBL_transportWrite(uint8_t* buffer, uint32_t size)
{
    int32_t     retVal;

	/* Read using the SPI peripheral */
    retVal = Spoof_spiWrite(gSblMCB.spiHandle, (uint8_t*)buffer, size);

    return retVal;
}

/*!
 *  @b Description
 *  @n
 *      Takes the Formatted input string and dumps it on the SPI console
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval Not Applicable
 */

void SBL_printf(const uint8_t *pcFormat, ...)
{
    uint8_t*        pcTemp = NULL;
    uint8_t         logspiBuff[120];
    va_list         list;
    int32_t         i,iRet = 0;
    spiTestMsg      *pTestMsgOut;

    pcTemp = &logspiBuff[0U];
    pTestMsgOut = (spiTestMsg*)&txBuf[0]; 

	va_start(list, pcFormat);
	iRet = vsnprintf((char *)pcTemp, SBL_SPI_LOG_BUFF_SIZE, (const char *)pcFormat, list);
	va_end(list);
	if((iRet > MINUS_ONE) && (iRet < SBL_SPI_LOG_BUFF_SIZE))
	{
                  pTestMsgOut->msgId = SBL_COMMAND_PRINTF;
                  pTestMsgOut->dataLen = iRet;
                  pTestMsgOut->seqNo = 0;
                  /* Swap bytes */
                  for (i=0;i<iRet/2;i++)
                  {
                     pTestMsgOut->data[2*i+1] = *pcTemp++;
                     pTestMsgOut->data[2*i]   = *pcTemp++;

                  }    
                  if(iRet%2)
                  {
                     pTestMsgOut->data[2*i+1] = *pcTemp++;
                  }  
                       
                  SBL_transportWrite((void *)pTestMsgOut, SPI_TEST_MSGLEN);

	}
    return;
}

/*!
 *  @b Description
 *  @n
 *      This function downloads the application meta image file over SPI using XMODEM.
 *      The downloaded file is written to SFLASH pointed to by the flash address.
 *      XMODEM receive routine is implemented based on the following sender<->receiver handshake.
 *      NAK and CAN handshake is not shown below.
 *
 *      RECEIVER ("s -k foo.bar")       SENDER ("foo.bar open x.x minutes")
 *
 *          C
 *                                          STX 01 FE Data[1024] CRC CRC
 *          ACK
 *                                          STX 02 FD Data[1024] CRC CRC
 *          ACK
 *                                          SOH 03 FC Data[128] CRC CRC
 *          ACK
 *                                          SOH 04 FB Data[100] CPMEOF[28] CRC CRC
 *          ACK
 *                                          EOT
 *          ACK
 *
 *  @param[in]  qspiFlashHandle
 *      Handle of QSPI Flash module.
 *  @param[in]  flashAddr
 *      Address of SFLASH location where the application meta image is written to.
 *  @param[in]  maxSize
 *      Maximum size of meta image.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Number of bytes read.
 *  @retval
 *      Error   -   SBL Error code.
 */

int32_t SBL_transportDownloadFile(QSPIFlash_Handle qspiFlashHandle, uint32_t flashAddr, uint32_t maxSize)
{
      uint8_t     dataBuffer[128];
      uint16_t    dataBufferSize = 0;
      int32_t     dataLength = 0;
      uint8_t*    ptrData;
      spiTestMsg     *pTestMsgIn,*pTestMsgOut;  
      int i;

      SBL_printf("Debug: Start the image download using SPI\r\n");

      while(1)
      { 
           uint16_t checksum=0;
           SBL_transportRead((void *)rxBuf, SPI_TEST_MSGLEN);
           pTestMsgIn = (spiTestMsg*)&rxBuf[0];
           pTestMsgOut = (spiTestMsg*)&txBuf[0];

	   /* Skip if it is not IMAGE DATA
              Exception: Terminate command.    
           */ 
           if(pTestMsgIn->msgId != SWAP_BYTES(SBL_COMMAND_DATA))
           {

              if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_TERMINATE))
              {
                  pTestMsgOut->msgId = SBL_COMMAND_TERMINATE;
                  pTestMsgOut->dataLen = 0;
                  pTestMsgOut->seqNo = 0;
                  SBL_transportWrite((void *)pTestMsgOut, SPI_TEST_MSGLEN);
                  break;
               }

            continue;   
          }    
          ptrData = dataBuffer;
          dataBufferSize=SWAP_BYTES(pTestMsgIn->dataLen);
          for(i=0;i<dataBufferSize;i++)
          {
             checksum += pTestMsgIn->data[i];
             *ptrData++ = pTestMsgIn->data[i];
          } 
        
         /* Validate the packet using checksum */
         if(checksum !=  SWAP_BYTES(pTestMsgIn->checksum))
          { 
              /* Request for retransmission of packet */
              gSblMCB.trans.retransErrors++;
              pTestMsgOut->msgId = SBL_COMMAND_CHECKSUM_ERR;
              pTestMsgOut->dataLen = 0;
              pTestMsgOut->seqNo = gSblMCB.trans.numRxPackets;
              SBL_transportWrite((void *)pTestMsgOut, SPI_TEST_MSGLEN);
              continue;
          } 

            /* Ack the packet */ 
            pTestMsgOut->msgId = SBL_COMMAND_ACK;
            pTestMsgOut->dataLen = 0;
            pTestMsgOut->seqNo = SWAP_BYTES(pTestMsgIn->seqNo);
            SBL_transportWrite((void *)pTestMsgOut, SPI_TEST_MSGLEN);

          /* Check for duplicate packets */
          if(gSblMCB.trans.numRxPackets >  SWAP_BYTES(pTestMsgIn->seqNo))
          {
             continue;
          }
   
          QSPIFlash_singleWrite(qspiFlashHandle, flashAddr, dataBufferSize, (uint8_t *)&dataBuffer[0]);

          flashAddr += dataBufferSize;
		    		
          dataLength += dataBufferSize;
          if(dataLength > maxSize)
          {

            SBL_printf("Size of flash image exceeds max size");
  
          }  
          /* Increment number of packets received  */ 
          gSblMCB.trans.numRxPackets++;
      }   
	return dataLength;
}


