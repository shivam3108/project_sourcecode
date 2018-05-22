/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/

/* Application Header files */
#include "StateMachine.h" 
#include "rfSynchronizedPacket.h"
#include "smartrf_settings/smartrf_settings.h"

/* XDCtools Header files */ 
#include <xdc/std.h>

/*Board Header Files*/
 #include "Board.h"

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Assert.h>

/* TI-RTOS Header files */ 
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/rf/RF.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)


/* Define events that can be posted to the application state machine */
typedef enum {
    Event_TxModeButtonPushed = StateMachine_Event00,
    Event_LedButtonPushed = StateMachine_Event01,
} Event;

/* Declare state handler functions for the application. */
StateMachine_DECLARE_STATE(SetupState);
StateMachine_DECLARE_STATE(PeriodicBeaconState);
StateMachine_DECLARE_STATE(SpontaneousBeaconState);


/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 *   - Button pins are high by default and pulled to low when pressed.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_PIN_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
#if defined __CC1352R1_LAUNCHXL_BOARD_H__
    Board_DIO30_RFSW | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif    
	PIN_TERMINATE
};


/***** Defines *****/
#define BEACON_INTERVAL_MS  500
#define MAIN_TASK_STACK_SIZE 1024
#define MAIN_TASK_PRIORITY   1        /* Lowest priority by default */

/***** Prototypes *****/
void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId);
void PeriodicBeaconState_txCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
Task_Struct mainTask;    /* not static so you can see in ROV */
static uint8_t mainTaskStack[MAIN_TASK_STACK_SIZE];

static PIN_Handle pinHandle;
static PIN_State pinState;

static RF_Object rfObject;
static RF_Handle rfHandle;

static BeaconPacket message;
static StateMachine_Struct stateMachine;

/***** Function definitions *****/

int main(void)
{
    /* Call driver init functions. */
    Board_initGeneral();

    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);
    Assert_isTrue(pinHandle != NULL, NULL);

    /* Setup callback for button pins */
    PIN_Status status = PIN_registerIntCb(pinHandle, &buttonCallbackFunction);
    Assert_isTrue(status == PIN_SUCCESS, NULL);

    // Initialise the application state machine.
    StateMachine_construct(&stateMachine);

    /* Initialise the main task. It will execute
     * the state machine StateMachine_exec() function.
     */
    Task_Params params;
    Task_Params_init(&params);
    params.stackSize = MAIN_TASK_STACK_SIZE;
    params.priority = MAIN_TASK_PRIORITY;
    params.stack = &mainTaskStack;
    params.arg0 = (UArg)&stateMachine;
    params.arg1 = (UArg)SetupState;
    Task_construct(&mainTask, (Task_FuncPtr)&StateMachine_exec, &params, NULL);

    /* Start BIOS */
    BIOS_start();

    return (0);
}



/* Pin interrupt Callback function board buttons configured in the pinTable. */
void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId) {

    /* Debounce the button with a short delay */
    CPUdelay(CPU_convertMsToDelayCycles(5));
    if (PIN_getInputValue(pinId) == 1)
    {
        return;
    }

    switch (pinId)
    {
    case Board_PIN_BUTTON0:
        StateMachine_postEvents(&stateMachine, Event_LedButtonPushed);
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, !PIN_getInputValue(Board_PIN_LED1));
        break;
    case Board_PIN_BUTTON1:
        StateMachine_postEvents(&stateMachine, Event_TxModeButtonPushed);
        break;
    }
}


void SetupState_function()
{
    /* Prepare the packet */
    RF_cmdPropTx.pktLen = sizeof(message);
    RF_cmdPropTx.pPkt = (uint8_t*)&message;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTime = 0;

    message.beaconInterval = RF_convertMsToRatTicks(BEACON_INTERVAL_MS);

    /* Request access to the radio. This does not power-up the RF core, but only initialise
     * the driver and cache the setup command. */
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    Assert_isTrue(rfHandle != NULL, NULL);

    /* Increase the SWI priority of the internal state machine in the RF driver. The default
     * value is 0. This is necessary because the button callback blocks for a long time and
     * might disturb the RF driver state machine.
     */
    uint32_t swiPriority = 1;
    RF_control(rfHandle, RF_CTRL_SET_SWI_PRIORITY, &swiPriority);

    /* Set the frequency. Now the RF driver powers the RF core up and runs the setup command from above.
     * The FS command is executed and also cached for later use when the RF driver does an automatic
     * power up. */
    RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    Assert_isTrue(((result == RF_EventLastCmdDone) && ((volatile RF_Op*)&RF_cmdFs)->status == DONE_OK), NULL);

    /* Use the current time as an anchor point for future time stamps.
     * The Nth transmission in the future will be exactly N * 500ms after
     * this time stamp.  */
    RF_cmdPropTx.startTime = RF_getCurrentTime();

    /* A trigger in the past is triggered as soon as possible.
     * No error is given.
     * This avoids assertion when button debouncing causes delay in TX trigger.  */
    RF_cmdPropTx.startTrigger.pastTrig = 1;

    /* Route the PA signal to an LED to indicate ongoing transmissions.
     * Available signals are listed in the proprietary RF user's guide.
     */
    PINCC26XX_setMux(pinHandle, Board_PIN_LED2, PINCC26XX_MUX_RFC_GPO1);

    StateMachine_setNextState(&stateMachine, PeriodicBeaconState);
}

void PeriodicBeaconState_function()
{
    /* Set absolute TX time in the future to utilise "deferred dispatching of commands with absolute timing".
     * This is explained in the proprietary RF user's guide. */
    RF_cmdPropTx.startTime += RF_convertMsToRatTicks(BEACON_INTERVAL_MS);

    message.txTime = RF_cmdPropTx.startTime;
    message.ledState = PIN_getInputValue(Board_PIN_LED1);

    /* Because the TX command is due in 500ms and we use TRIG_ABSTIME as start trigger type,
     * the RF driver will now power down the RF core and and wait until ~1.5ms before RF_cmdPropTx.startTime.
     * Then the RF driver will power-up the RF core, re-synchronise the RAT and re-run the setup procedure.
     * The setup procedure includes RF_cmdPropRadioDivSetup and RF_cmdFs from above.
     * This will guarantee that RF_cmdPropTx is delivered to the RF core right before it has
     * to start. This is fully transparent to the application. It appears as the RF core was
     * never powered down.
     * This concept is explained in the proprietary RF user's guide. */
    RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
    Assert_isTrue(((result == RF_EventLastCmdDone) && ((volatile RF_Op*)&RF_cmdPropTx)->status == PROP_DONE_OK), NULL);

    if (StateMachine_pendEvents(&stateMachine, Event_TxModeButtonPushed, BIOS_NO_WAIT) & Event_TxModeButtonPushed)
    {
        StateMachine_setNextState(&stateMachine, SpontaneousBeaconState);
    }
    else
    {
        // PeriodicBeaconState_function() will be entered again.
        StateMachine_setNextState(&stateMachine, PeriodicBeaconState);
    }
}


void SpontaneousBeaconState_function()
{
    StateMachine_EventMask events = StateMachine_pendEvents(&stateMachine, Event_TxModeButtonPushed | Event_LedButtonPushed, BIOS_WAIT_FOREVER);

    /* We need to find the next synchronized time slot that is far enough
     * in the future to allow the RF driver to power up the RF core.
     * We use 2 ms as safety margin. */
    uint32_t currentTime = RF_getCurrentTime() + RF_convertMsToRatTicks(2);
    uint32_t intervalsSinceLastPacket = DIV_INT_ROUND_UP(currentTime - RF_cmdPropTx.startTime, RF_convertMsToRatTicks(BEACON_INTERVAL_MS));
    RF_cmdPropTx.startTime += intervalsSinceLastPacket * RF_convertMsToRatTicks(BEACON_INTERVAL_MS);

    if (events & Event_TxModeButtonPushed)
    {
        StateMachine_setNextState(&stateMachine, PeriodicBeaconState);
    }

    if (events & Event_LedButtonPushed)
    {
        message.txTime = RF_cmdPropTx.startTime;
        message.ledState = PIN_getInputValue(Board_PIN_LED1);

        /* Because the TX command is due in 500ms and we use TRIG_ABSTIME as start trigger type,
         * the RF driver will now power down the RF core and and wait until ~1.5ms before RF_cmdPropTx.startTime.
         * Then the RF driver will power-up the RF core, re-synchronise the RAT and re-run the setup procedure.
         * The setup procedure includes RF_cmdPropRadioDivSetup and RF_cmdFs from above.
         * This will guarantee that RF_cmdPropTx is delivered to the RF core right before it has
         * to start. This is fully transparent to the application. It appears as the RF core was
         * never powered down.
         * This concept is explained in the proprietary RF user's guide. */
        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
        Assert_isTrue(((result == RF_EventLastCmdDone) && ((volatile RF_Op*)&RF_cmdPropTx)->status == PROP_DONE_OK), NULL);
    }
}
