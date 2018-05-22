/******************************************************************************

 @file oad_client.c

 @brief OAD Client

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc13x0_sdk_2_10_00_
 Release Date: 2018-04-09 00:04:23
 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <ti/sysbios/knl/Clock.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/flash.h)

#include "oad/native_oad/oad_client.h"
#include "oad/native_oad/oad_protocol.h"
#include "oad/native_oad/oad_storage.h"

#include "RadioProtocol.h"
#include "NodeTask.h"
#include "NodeRadioTask.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

#define FW_VERSION "v1.0"

/*!
 OAD block variables.
 */
static uint16_t oadBNumBlocks = 0;
static uint16_t oadBlock = 0;
static uint8_t oadServerAddr = {0};
static bool oadInProgress = false;

static uint32_t oadRetries = 0;
static uint32_t oadRetriesTotal = 0;

OADClient_Params_t oadClientParams;

static Clock_Struct oadBlockReqClkStruct;
static Clock_Handle oadBlockReqClkHandle;

static Clock_Struct oadRspPollClkStruct;
static Clock_Handle oadRspPollClkHandle;

/******************************************************************************
 Local function prototypes
 *****************************************************************************/

static void oadClockInitialize(void);
static void oadBlockReqClkSet(uint32_t oadTimeout);
static void oadRsPollClkSet(uint32_t timeout);
static void oadBlockReqClkCallback(UArg a0);
static void oadRspPollClkCallback(UArg a0);
static void oadFwVersionReqCb(void* pSrcAddr);
static void oadImgIdentifyReqCb(void* pSrcAddr, uint8_t imgId, uint8_t *imgMetaData);
static void oadBlockRspCb(void* pSrcAddr, uint8_t imgId, uint16_t blockNum, uint8_t *blkData);
void* oadRadioAccessAllocMsg(uint32_t msgLen);
static OADProtocol_Status_t oadRadioAccessPacketSend(void* pDstAddr, uint8_t *pMsg, uint32_t msgLen);

/******************************************************************************
 Callback tables
 *****************************************************************************/

static OADProtocol_RadioAccessFxns_t  oadRadioAccessFxns =
    {
      oadRadioAccessAllocMsg,
      oadRadioAccessPacketSend
    };

static OADProtocol_MsgCBs_t oadMsgCallbacks =
    {
      /*! Incoming FW Req */
      oadFwVersionReqCb,
      /*! Incoming FW Version Rsp */
      NULL,
      /*! Incoming Image Identify Req */
      oadImgIdentifyReqCb,
      /*! Incoming Image Identify Rsp */
      NULL,
      /*! Incoming OAD Block Req */
      NULL,
      /*! Incoming OAD Block Rsp */
      oadBlockRspCb
    };

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize this application.

 Public function defined in sensor.h
 */
void OADClient_open(OADClient_Params_t *params)
{
    OADProtocol_Params_t OADProtocol_params;

    memcpy(&oadClientParams, params, sizeof(OADClient_Params_t));

    OADProtocol_Params_init(&OADProtocol_params);
    OADProtocol_params.pRadioAccessFxns = &oadRadioAccessFxns;
    OADProtocol_params.pProtocolMsgCallbacks = &oadMsgCallbacks;

    OADProtocol_open(&OADProtocol_params);

    oadClockInitialize();
}

/*!
 Application task processing.

 Public function defined in sensor.h
 */
void OADClient_processEvent(uint32_t *pEvent)
{
    /* Is it time to send the next sensor data message? */
    if(*pEvent & oadClientParams.oadReqEventBit)
    {
        static int32_t lastBlock = -1;
        static bool oadComplete = false;

        if(oadComplete)
        {
            /* reset to BIM */
            SysCtrlSystemReset();

            /* Clear complete flag in case reset did not work */
            oadComplete = false;

        }

        if(oadInProgress)
        {
            if(oadBlock == lastBlock)
            {
                /*
                 * allow 3 retries before aborting
                 */
                if(oadRetries++ > OADProtocol_MAX_RETRIES)
                {
                    /*abort OAD */
                    oadInProgress = false;

                    /* Stop OAD timer */
                    oadBlockReqClkSet(0);

                    /* stop rsp poll timer */
                    oadRsPollClkSet(0);

                    OADStorage_close();

                    NodeTask_displayOadStatusUpdate(OADStorage_Failed);

                    lastBlock = -1;
                }
                else
                {
                    /* reduce the lastBlock by 1 to force a re-request */
                    lastBlock -= 1;
                }

                oadRetriesTotal++;
            }
        }

        if(oadInProgress)
        {
            if(oadBlock < oadBNumBlocks)
            {
                lastBlock = oadBlock;

                /* Send Block request specifying number of blocks per multi
                 * blocks. The OAD_MULTI_BLOCK_SIZE hard coded here, but it is
                 * intended that an advanced user would use information from
                 * oadRetries and Sensor_msgStats to implement a sliding window
                 * algorithm
                 */
                OADProtocol_sendOadImgBlockReq(&oadServerAddr, 0, oadBlock, 1);

                /* update display */
                NodeTask_displayOadBlockUpdate(oadBlock, oadBNumBlocks, oadRetriesTotal);

                /*
                 * Send poll request to get rsp in OADProtocol_BLOCK_REQ_POLL_DELAY ms
                 * to flush the block response from OAD servers MAC Queue
                 */
                oadRsPollClkSet(OADProtocol_BLOCK_REQ_POLL_DELAY);

                /*
                 * set timer to time out in OADProtocol_BLOCK_REQ_RATE ms to request
                 * or re-request block
                 * */
                oadBlockReqClkSet(OADProtocol_BLOCK_REQ_RATE);
            }
            else
            {
                OADStorage_Status_t status;

                /*
                 * Check that CRC is correct and mark the image as new
                 * image to be booted in to by BIM on next reset
                 */
                status = OADStorage_imgFinalise();

                /* Display result */
                NodeTask_displayOadStatusUpdate(status);

                /* Stop any further OAD message processing */
                oadInProgress = false;

                /* If OAD was successful reset the device */
                if(status == OADStorage_Status_Success)
                {
                    oadComplete = true;

                    /*
                     * Set time 1 more time to reset the device
                     * after the display has been written to
                     */
                    oadBlockReqClkSet(OADProtocol_BLOCK_REQ_RATE);
                }
                else
                {
                    //something went wrong abort
                    oadBlockReqClkSet(0);
                }
            }
        }
    }
}

/******************************************************************************
 Local Functions
 *****************************************************************************/

/*!
 * @brief   OAD block req timeout handler function.
 *
 * @param   a0 - ignored
 */
static void oadBlockReqClkCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    Event_post(oadClientParams.eventHandle, oadClientParams.oadReqEventBit);
}


/*!
 * @brief   OAD prsp poll timeout handler function.
 *
 * @param   a0 - ignored
 */
static void oadRspPollClkCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    /* send ADC reading to poll for rsp */
    Event_post(oadClientParams.eventHandle, oadClientParams.oadRspPollEventBit);
}

static void oadClockInitialize(void)
{
    /* Create clock object which is used for oad block requests */
    Clock_Params clkParams;
    Clock_Params_init(&clkParams);

    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&oadBlockReqClkStruct, oadBlockReqClkCallback, 1, &clkParams);
    oadBlockReqClkHandle = Clock_handle(&oadBlockReqClkStruct);

    Clock_construct(&oadRspPollClkStruct, oadRspPollClkCallback, 1, &clkParams);
    oadRspPollClkHandle = Clock_handle(&oadRspPollClkStruct);

}

/*!
 Set the oad block req clock.

 Public function defined in ssf.h
 */
static void oadBlockReqClkSet(uint32_t timeout)
{
    /* setup timeout for fast report timeout */
    Clock_setTimeout(oadBlockReqClkHandle,
                     timeout * 1000 / Clock_tickPeriod);

    /* start fast report and timeout */
    Clock_start(oadBlockReqClkHandle);
}

/*!
 Set the oad rsp poll clock.

 Public function defined in ssf.h
 */
static void oadRsPollClkSet(uint32_t timeout)
{
    /* setup timeout for fast report timeout */
    Clock_setTimeout(oadRspPollClkHandle,
                     timeout * 1000 / Clock_tickPeriod);

    /* start fast report and timeout */
    Clock_start(oadRspPollClkHandle);
}

/*!
 * @brief      FW Version request callback
 */
static void oadFwVersionReqCb(void* pSrcAddr)
{
    char fwVersionStr[OADProtocol_FW_VERSION_STR_LEN] = FW_VERSION;

    //TODO: Get version from OAD header

    //Send response back
    OADProtocol_sendFwVersionRsp(pSrcAddr, fwVersionStr);
}

/*!
 * @brief      OAD image identify request callback
 */
static void oadImgIdentifyReqCb(void* pSrcAddr, uint8_t imgId, uint8_t *imgMetaData)
{
    /*
     * Ignore imgId - its not used in this
     * implementation as there is only 1 image available
     */
    (void) imgId;

    oadServerAddr = (uint8_t) *((uint8_t*)pSrcAddr);

    oadBNumBlocks = OADStorage_imgIdentifyWrite(imgMetaData);
    oadBlock = 0;

    if(oadBNumBlocks)
    {
        oadInProgress = true;
        oadRetries = 0;
        oadRetriesTotal = 0;

        //Send success response back
        OADProtocol_sendOadIdentifyImgRsp(pSrcAddr, 1);

        //Set OAD time out to OADProtocol_BLOCK_REQ_RATE ms
        oadBlockReqClkSet(OADProtocol_BLOCK_REQ_RATE);
    }
    else
    {
        //Send fail response back
        OADProtocol_sendOadIdentifyImgRsp(pSrcAddr, 0);
    }
}

/*!
 * @brief      OAD image block response callback
 */
static void oadBlockRspCb(void* pSrcAddr, uint8_t imgId, uint16_t blockNum, uint8_t *blkData)
{
    /*
     * Ignore imgId - its not used in this
     * implementation as there is only 1 image available
     */
    (void) imgId;

    if( (blockNum == oadBlock) && (oadInProgress))
    {
        oadBlock++;
        oadRetries = 0;

        OADStorage_imgBlockWrite(blockNum, blkData);
    }
}

/*!
 * @brief      Radio access function for OAD module to send messages
 */
void* oadRadioAccessAllocMsg(uint32_t msgLen)
{
    uint8_t *msgBuffer;

    /*
     * Allocate with 2 byte before the oad msg buffer for
     * addr and packet ID.
     */
    msgBuffer = (uint8_t*) malloc(msgLen + 2);
	if(msgBuffer == NULL)
    {
        return NULL;
    }
    memset(msgBuffer, 0, msgLen + 2);

    return msgBuffer + 2;
}

/*!
 * @brief      Radio access function for OAD module to send messages
 */
static OADProtocol_Status_t oadRadioAccessPacketSend(void* pDstAddr, uint8_t *pMsgPayload, uint32_t msgLen)
{
    OADProtocol_Status_t status = OADProtocol_Failed;
    uint8_t* pMsg;
    uint8_t dstAddr = (uint8_t) *((uint8_t*)pDstAddr);
    /*
     * buffer should have been allocated with oadRadioAccessAllocMsg,
     * so 2 byte before the oad msg buffer was allocated for the source
     * addr and Packet ID. Source addr will be filled in by
     * NodeRadioTask_sendOadMsg
     */
    pMsg = pMsgPayload - 2;
    pMsg[RADIO_PACKET_PKTTYPE_OFFSET] = RADIO_PACKET_TYPE_OAD_PACKET;

    NodeRadioTask_sendOadMsg( dstAddr, pMsg, msgLen + 2);

    //free the memory allocated in oadRadioAccessAllocMsg
    free(pMsg);

    return status;
}

