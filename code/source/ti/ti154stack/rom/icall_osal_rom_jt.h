/******************************************************************************

 @file icall_osal_rom_jt.h

 @brief Icall and Osal API directly map the function to function jump table

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

#ifndef ICALL_OSAL_ROM_JT_H
#define ICALL_OSAL_ROM_JT_H

#ifdef USE_ICALL
#include "icall.h"
#endif

#include "osal.h"
#include "saddr.h"

#include "rom_jt_def.h"

/*
** ICALL OSAL API Proxy
** ROM-to ROM or ROM-to-Flash function
** if there is any patch function, replace the corresponding entries
*/

#define ICALL_OSAL_API_BASE_INDEX                   (0)

//ICall_CSState ICall_enterCSImpl(void)
#define MAP_ICall_enterCriticalSection                  ((ICall_CSState     (*)(void ))             ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+0))
//void ICall_leaveCSImpl(ICall_CSState key)
#define MAP_ICall_leaveCriticalSection                  ((void     (*)(ICall_CSState ))             ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+1))


#define MAP_osal_mem_alloc                              ((void *    (*)(uint16_t ))                         ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+2))
#define MAP_osal_mem_free                               ((void     (*)(void * ))                            ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+3))
#define MAP_osal_memcmp                                 ((uint8     (*)(const void *,const void *,unsigned int ))       ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+4))
#define MAP_osal_memcpy                                 ((void *    (*)(void *, const void *, unsigned int ))           ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+5))
#define MAP_osal_memset                                 ((void *    (*)(void *, uint8, int ))                           ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+6))

#define MAP_osal_msg_allocate                           ((uint8 *    (*)(uint16 ))                          ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+7))
#define MAP_osal_msg_deallocate                         ((uint8      (*)(uint8 * ))                         ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+8))
#define MAP_osal_msg_dequeue                            ((void *    (*)(osal_msg_q_t * ))                   ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+9))
#define MAP_osal_msg_enqueue                            ((void *    (*)(osal_msg_q_t *, void *  ))          ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+10))
#define MAP_osal_msg_enqueue_max                        ((uint8    (*)(osal_msg_q_t *, void *, uint8  ))    ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+11))

#define MAP_osal_msg_extract                            ((void    (*)(osal_msg_q_t *, void *,void * ))      ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+12))
#define MAP_osal_msg_push                               ((void    (*)(osal_msg_q_t *, void * ))             ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+13))
#define MAP_osal_msg_receive                            ((uint8 *  (*)(uint8 ))                             ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+14))
#define MAP_osal_msg_send                               ((uint8    (*)(uint8, uint8 * ))                    ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+15))
#define MAP_osal_pwrmgr_task_state                      ((uint8    (*)(uint8, uint8  ))                     ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+16))
#define MAP_osal_set_event                              ((uint8    (*)(uint8, uint16  ))                    ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+17))
#define MAP_osal_strlen                                 ((int      (*)(char *  ))                           ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+18))


#define MAP_memcmp                                      ((int (*)(const void *, const void *, unsigned int ))  \
                                                                                                            ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+19))


#define MAP_ICall_getTicks                              ((uint_fast32_t    (*)(void ))                      ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+20))
#define MAP_ICall_setTimer                              ((ICall_Errno    (*)(uint_fast32_t,ICall_TimerCback,void *,ICall_TimerID * ))     \
                                                                                                            ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+21))
#define MAP_ICall_stopTimer                             ((void  (*)(ICall_TimerID))                         ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+22))

#define MAP_sAddrCmp                                    ((bool  (*)(const sAddr_t *,const sAddr_t *))       ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+23))
#define MAP_sAddrCpy                                    ((void  (*)(sAddr_t *,const sAddr_t *))             ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+24))
#define MAP_sAddrExtCpy                                 ((void *  (*)(uint8 * ,const uint8 *))              ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+25))
#define MAP_sAddrExtCmp                                 ((bool  (*)(uint8 * ,const uint8 *))                ROM_ICALL_OSAL_JT_OFFSET(ICALL_OSAL_API_BASE_INDEX+26))


/*
 * Handle all macros
 * 1. undefine the macro
 * 2. redefine macro
 * 3. any function call will use function pointer
 *
 */
#undef HAL_ENTER_CRITICAL_SECTION
#define HAL_ENTER_CRITICAL_SECTION(x)           st(x = MAP_ICall_enterCriticalSection();)
#undef HAL_EXIT_CRITICAL_SECTION
#define HAL_EXIT_CRITICAL_SECTION(x)            MAP_ICall_leaveCriticalSection(x)


#endif
