/******************************************************************************

 @file fh_pib.c

 @brief TIMAC 2.0 FH PIB API

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016-2018, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: simplelink_cc13x0_sdk_2_10_00_
 Release Date: 2018-04-09 00:04:22
 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/
#include <stddef.h>
#include "fh_api.h"
#include "fh_pib.h"
#include "fh_util.h"
#include "mac_mgmt.h"

#ifdef FEATURE_FREQ_HOP_MODE

/******************************************************************************
  ROM jump table will support
  1. TIMAC ROM image build
  2. TIMAC stack build without ROM image
  3. TIMAC stack with calling ROM image
 *****************************************************************************/
#include "rom_jt.h"

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/******************************************************************************
 Local variables
 *****************************************************************************/

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/

uint16_t FHPIB_getIndex(uint16_t fhPibId)
{
    if((fhPibId >= FH_PIB_ID_START) && (fhPibId <= FH_PIB_ID_END))
    {
        return(fhPibId - FH_PIB_ID_START + FH_PIB_ID_OFFSET);
    }
    else
    {
        return(FH_PIB_ID_INVALID);
    }
}

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 FHPIB_reset

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API void FHPIB_reset(void)
{
    /* copy FH-related PIB defaults */
    //FHPIB_db = FHPIB_defaults;
    MAP_osal_memcpy((void *)&FHPIB_db, (const void *)&FHPIB_defaults, sizeof(FHPIB_db));
}

/*!
 FHPIB_getLen

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API uint8_t FHPIB_getLen(uint16_t fhPibId)
{
    uint16_t i;

    if((i = MAP_FHPIB_getIndex(fhPibId)) == FH_PIB_ID_INVALID)
    {
        return(0);
    }

    return (FH_PibTbl[i].len);
}

/*!
 FHPIB_set

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API FHAPI_status FHPIB_set(uint16_t fhPibId, void *pData)
{
    uint16_t i;
    uint8_t pibLen;
    uint8_t numChannels, maxChannels, excludedChannels;
    halIntState_t intState;

    if((i = MAP_FHPIB_getIndex(fhPibId)) == FH_PIB_ID_INVALID)
    {
        return(FHAPI_STATUS_ERR_NOT_SUPPORTED_PIB);
    }

    /* do range check; no range check if min and max are zero */
    if ((FH_PibTbl[i].min != 0) || (FH_PibTbl[i].max != 0))
    {
        /* if min == max, this is a read-only attribute */
        if (FH_PibTbl[i].min == FH_PibTbl[i].max)
        {
            return(FHAPI_STATUS_ERR_READ_ONLY_PIB);
        }

        /* range check for general case */
        pibLen = FH_PibTbl[i].len;
        switch(pibLen)
        {
            case FH_UINT8_SIZE:
                if ((*((uint8_t *) pData) < FH_PibTbl[i].min)
                 || (*((uint8_t *) pData) > FH_PibTbl[i].max))
                {
                    return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
                }
                break;
            case FH_UINT16_SIZE:
                if ((*((uint16_t *) pData) < FH_PibTbl[i].min)
                 || (*((uint16_t *) pData) > FH_PibTbl[i].max))
                {
                    return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
                }
                break;
            case FH_UINT32_SIZE:
                if ((*((uint32_t *) pData) < FH_PibTbl[i].min)
                 || (*((uint32_t *) pData) > FH_PibTbl[i].max))
                {
                    return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
                }
                break;
            default:
                break;
        }
    }

    HAL_ENTER_CRITICAL_SECTION(intState);
    switch(fhPibId)
    {
        case FHPIB_NET_NAME:
            MAP_osal_memset((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, 0,
                    FH_PibTbl[i].len);
            MAP_osal_memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                    MAP_osal_strlen((char *)pData) < FH_PibTbl[i].len ?
                    MAP_osal_strlen((char *)pData) : FH_PibTbl[i].len);
            break;
        case FHPIB_BC_FIXED_CHANNEL:
        case FHPIB_UC_FIXED_CHANNEL:
            maxChannels = MAP_FHUTIL_getMaxChannels();
            if (*((uint16_t *) pData) < maxChannels)
            {
                MAP_osal_memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                        FH_PibTbl[i].len);
            }
            else
            {
                HAL_EXIT_CRITICAL_SECTION(intState);
                return FHAPI_STATUS_ERR_INVALID_PARAM_PIB;
            }
            break;
        case FHPIB_BC_EXCLUDED_CHANNELS:
        case FHPIB_UC_EXCLUDED_CHANNELS:
            maxChannels = MAP_FHUTIL_getMaxChannels();
            excludedChannels = MAP_FHUTIL_getBitCounts((uint8_t *)pData, maxChannels);
            numChannels = maxChannels - excludedChannels;
            if(numChannels > 0)
            {
                MAP_osal_memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                        FH_PibTbl[i].len);
            }
            else
            {
                HAL_EXIT_CRITICAL_SECTION(intState);
                return FHAPI_STATUS_ERR_INVALID_PARAM_PIB;
            }
            if(fhPibId == FHPIB_UC_EXCLUDED_CHANNELS)
            {
                FH_hnd.ucNumChannels = numChannels;
            }
            else if(fhPibId == FHPIB_BC_EXCLUDED_CHANNELS)
            {
                FH_hnd.bcNumChannels = numChannels;
            }
            break;
        default:
            MAP_osal_memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                    FH_PibTbl[i].len);
            break;
    }
    /* sanity checking for number of non-sllep and leep node */
    if (fhPibId == FHPIB_NUM_MAX_NON_SLEEP_NODES)
    {
        /* reset the sleep node to zero */
        FHPIB_db.macMaxSleepNodes = 0;
    }

    if (fhPibId == FHPIB_NUM_MAX_SLEEP_NODES)
    {
        /* sanity checking to make sure total number is not greater than MAX device
         * table size
         */
        if (  ((FHPIB_db.macMaxNonSleepNodes + FHPIB_db.macMaxSleepNodes) > FHPIB_MAX_NUM_DEVICE ) ||
              ((FHPIB_db.macMaxNonSleepNodes == 0 ) && (FHPIB_db.macMaxSleepNodes == 0) ) )
        {
            /* back to default */
            FHPIB_db.macMaxNonSleepNodes = FHPIB_defaults.macMaxNonSleepNodes;
            FHPIB_db.macMaxSleepNodes = FHPIB_defaults.macMaxSleepNodes;
            HAL_EXIT_CRITICAL_SECTION(intState);
            return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
        }
    }
    HAL_EXIT_CRITICAL_SECTION(intState);
    return(FHAPI_STATUS_SUCCESS);
}

/*!
 FHPIB_get

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API FHAPI_status FHPIB_get(uint16_t fhPibId, void *pData)
{
    uint16_t i;
    halIntState_t intState;

    if((i = MAP_FHPIB_getIndex(fhPibId)) == FH_PIB_ID_INVALID)
    {
        return(FHAPI_STATUS_ERR_NOT_SUPPORTED_PIB);
    }

    HAL_ENTER_CRITICAL_SECTION(intState);
    MAP_osal_memcpy(pData, (uint8_t *)&FHPIB_db + FH_PibTbl[i].offset,
            FH_PibTbl[i].len);
    HAL_EXIT_CRITICAL_SECTION(intState);
    return(FHAPI_STATUS_SUCCESS);
}
#else
/*!
 FHPIB_getLen

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API uint8_t FHPIB_getLen(uint16_t fhPibId)
{
    (void)fhPibId;
    return(0);
}

/*!
 FHPIB_get

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API FHAPI_status FHPIB_get(uint16_t fhPibId, void *pData)
{
    (void)fhPibId;
    (void)pData;
    return(FHAPI_STATUS_ERR);
}
#endif /* FEATURE_FREQ_HOP_MODE */
